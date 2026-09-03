#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/estimator/internal/imu_interval.hpp"

namespace phad::estimator::internal
{

  struct InitialMovingPose
  {
    std::uint64_t     m_frame_index = 0;
    common::Timestamp m_timestamp{ 0 };
    Eigen::Isometry3d m_T_W_B = Eigen::Isometry3d::Identity();
  };

  struct InitialMovingPimConfig
  {
    Eigen::Matrix3d m_acc_cov         = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d m_gyr_cov         = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d m_integration_cov = Eigen::Matrix3d::Zero();
  };

  struct InitialMovingMathInput
  {
    std::vector<InitialMovingPose>     m_poses;
    std::vector<NormalizedImuInterval> m_intervals;
    InitialMovingPimConfig             m_pim;
    double                             m_gravity_mps2 = 9.81;
  };

  struct InitialMovingCandidate
  {
    Eigen::Vector3d              m_bg_radps = Eigen::Vector3d::Zero();
    Eigen::Vector3d              m_g_W      = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> m_v_W;
  };

  struct InitialMovingCanonicalSystem
  {
    Eigen::VectorXd             m_residual;
    Eigen::MatrixXd             m_candidate_jacobian;
    Eigen::MatrixXd             m_visual_pose_jacobian;
    Eigen::MatrixXd             m_pim_covariance;
    Eigen::Matrix<double, 3, 2> m_gravity_tangent_basis =
        Eigen::Matrix<double, 3, 2>::Zero();
  };

  enum class InitialMovingMathErrorCode : std::uint8_t
  {
    kDimension = 0,
    kContract  = 1,
    kNumerical = 2
  };

  struct InitialMovingMathError
  {
    InitialMovingMathErrorCode m_code = InitialMovingMathErrorCode::kContract;
    std::string                m_detail;
  };

  using InitialMovingCanonicalResult =
      std::variant<InitialMovingCanonicalSystem, InitialMovingMathError>;
  using InitialMovingMatrixResult =
      std::variant<Eigen::MatrixXd, InitialMovingMathError>;

  struct InitialMovingJointFit
  {
    double       m_statistic     = 0.0;
    std::int64_t m_residual_rank = 0;
    std::int64_t m_fitted_rank   = 0;
    std::int64_t m_effective_dof = 0;
  };

  using InitialMovingJointResult =
      std::variant<InitialMovingJointFit, InitialMovingMathError>;

  inline constexpr double kInitialMovingNormalizedConditionLimit =
      67'108'864.0;

  struct InitialMovingLinearSolution
  {
    Eigen::VectorXd       m_delta;
    std::uint32_t         m_rank          = 0;
    std::uint32_t         m_required_rank = 0;
    std::optional<double> m_normalized_condition;
    double                m_normalized_condition_limit =
        kInitialMovingNormalizedConditionLimit;
  };

  enum class InitialMovingLinearFailureCode : std::uint8_t
  {
    kRankDeficient  = 0,
    kIllConditioned = 1
  };

  struct InitialMovingLinearFailure
  {
    InitialMovingLinearFailureCode m_code =
        InitialMovingLinearFailureCode::kRankDeficient;
    std::uint32_t         m_rank          = 0;
    std::uint32_t         m_required_rank = 0;
    std::optional<double> m_normalized_condition;
  };

  using InitialMovingLinearResult =
      std::variant<InitialMovingLinearSolution,
                   InitialMovingLinearFailure,
                   InitialMovingMathError>;

  struct InitialMovingGyroSolution
  {
    Eigen::Vector3d       m_bg_radps = Eigen::Vector3d::Zero();
    std::uint32_t         m_rank     = 0;
    std::optional<double> m_normalized_condition;
  };

  using InitialMovingGyroResult =
      std::variant<InitialMovingGyroSolution,
                   InitialMovingLinearFailure,
                   InitialMovingMathError>;

  struct InitialMovingGravityVelocitySolution
  {
    Eigen::Vector3d              m_g_W = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> m_v_W;
    std::uint32_t                m_rank = 0;
    std::optional<double>        m_normalized_condition;
  };

  using InitialMovingGravityVelocityResult =
      std::variant<InitialMovingGravityVelocitySolution,
                   InitialMovingLinearFailure,
                   InitialMovingMathError>;

  enum class InitialMovingCandidateFailureCode : std::uint8_t
  {
    kRankDeficient        = 0,
    kIllConditioned       = 1,
    kNoConvergence        = 2,
    kJointDofInsufficient = 3,
    kJointInconsistent    = 4
  };

  struct InitialMovingCandidateFailure
  {
    InitialMovingCandidateFailureCode m_code =
        InitialMovingCandidateFailureCode::kRankDeficient;
    std::optional<InitialMovingLinearFailure> m_linear_failure;
    std::optional<InitialMovingJointFit>      m_joint_fit;
    std::optional<double>                     m_joint_limit;
    std::optional<InitialMovingGyroSolution>  m_gyro;
    std::optional<InitialMovingGravityVelocitySolution>
        m_gravity_velocity;
  };

  struct InitialMovingCandidateSolution
  {
    InitialMovingCandidate               m_candidate;
    InitialMovingJointFit                m_joint_fit;
    double                               m_joint_limit       = 0.0;
    std::uint32_t                        m_refinement_rounds = 0;
    InitialMovingGyroSolution            m_gyro;
    InitialMovingGravityVelocitySolution m_gravity_velocity;
  };

  using InitialMovingCandidateResult =
      std::variant<InitialMovingCandidateSolution,
                   InitialMovingCandidateFailure,
                   InitialMovingMathError>;

  struct InitialMovingSolverLimits
  {
    std::uint32_t m_max_refinement_corrections = 8U;
  };

  [[nodiscard]] InitialMovingCanonicalResult
  buildInitialMovingCanonicalSystem(
      const InitialMovingMathInput& input,
      const InitialMovingCandidate& candidate );

  [[nodiscard]] InitialMovingMatrixResult
  buildInitialMovingInnovationCovariance(
      const InitialMovingCanonicalSystem& system,
      const Eigen::MatrixXd&              visual_pose_covariance );

  [[nodiscard]] InitialMovingJointResult
  evaluateInitialMovingJointConsistency(
      const Eigen::VectorXd& residual,
      const Eigen::MatrixXd& candidate_jacobian,
      const Eigen::MatrixXd& innovation_covariance );

  [[nodiscard]] std::int64_t initialMovingStrictNumericalRank(
      const Eigen::VectorXd& singular_values ) noexcept;

  [[nodiscard]] InitialMovingLinearResult solveInitialMovingLinearSystem(
      const Eigen::VectorXd& residual,
      const Eigen::MatrixXd& jacobian,
      const Eigen::MatrixXd& covariance,
      std::uint32_t          required_rank );

  [[nodiscard]] bool initialMovingConditionAccepted(
      double condition ) noexcept;

  [[nodiscard]] InitialMovingGyroResult
  solveInitialMovingGyroBiasFromZero(
      const InitialMovingMathInput& input,
      const Eigen::MatrixXd&        visual_pose_covariance );

  [[nodiscard]] InitialMovingGravityVelocityResult
  solveInitialMovingGravityVelocity(
      const InitialMovingMathInput& input,
      const Eigen::MatrixXd&        visual_pose_covariance,
      const Eigen::Vector3d&        gyro_bias_radps );

  [[nodiscard]] InitialMovingCandidateResult solveInitialMovingCandidate(
      const InitialMovingMathInput& input,
      const Eigen::MatrixXd&        visual_pose_covariance,
      InitialMovingSolverLimits     limits = {} );

  [[nodiscard]] bool initialMovingJointFitAccepted(
      double statistic, double limit ) noexcept;

}  // namespace phad::estimator::internal

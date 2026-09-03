#include "phad/estimator/internal/initial_moving_root_math.hpp"

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

namespace phad::estimator::internal
{
  namespace
  {

    [[nodiscard]] InitialMovingMathError error(
        const InitialMovingMathErrorCode code, std::string detail )
    {
      return InitialMovingMathError{ code, std::move( detail ) };
    }

    [[nodiscard]] Eigen::Matrix<double, 3, 2> gravityTangentBasis(
        const Eigen::Vector3d& gravity )
    {
      const Eigen::Vector3d direction = gravity.normalized();
      Eigen::Vector3d       reference = Eigen::Vector3d::UnitX();
      if ( std::abs( direction.y() ) < std::abs( direction.x() ) &&
           std::abs( direction.y() ) <= std::abs( direction.z() ) )
      {
        reference = Eigen::Vector3d::UnitY();
      }
      else if ( std::abs( direction.z() ) < std::abs( direction.x() ) &&
                std::abs( direction.z() ) < std::abs( direction.y() ) )
      {
        reference = Eigen::Vector3d::UnitZ();
      }
      Eigen::Matrix<double, 3, 2> basis;
      basis.col( 0 ) =
          ( reference - direction * direction.dot( reference ) ).normalized();
      basis.col( 1 ) = direction.cross( basis.col( 0 ) ).normalized();
      return basis;
    }

    [[nodiscard]] std::shared_ptr<gtsam::PreintegrationParams> makeParams(
        const InitialMovingMathInput& input,
        const Eigen::Vector3d&        gravity )
    {
      auto params = std::make_shared<gtsam::PreintegrationParams>( gravity );
      params->setAccelerometerCovariance( input.m_pim.m_acc_cov );
      params->setGyroscopeCovariance( input.m_pim.m_gyr_cov );
      params->setIntegrationCovariance( input.m_pim.m_integration_cov );
      return params;
    }

    [[nodiscard]] gtsam::PreintegratedImuMeasurements preintegrate(
        const InitialMovingMathInput& input,
        const InitialMovingCandidate& candidate,
        const NormalizedImuInterval&  interval )
    {
      const gtsam::imuBias::ConstantBias bias(
          Eigen::Vector3d::Zero(), candidate.m_bg_radps );
      gtsam::PreintegratedImuMeasurements pim(
          makeParams( input, candidate.m_g_W ), bias );
      for ( const ImuIntervalStep& step : interval.m_steps )
      {
        pim.integrateMeasurement( step.m_acc_mean_mps2,
                                  step.m_gyr_mean_radps, step.m_dt_s );
      }
      return pim;
    }

    [[nodiscard]] Eigen::Matrix<double, 9, 3> gravityJacobian(
        const gtsam::PreintegratedImuMeasurements& pim,
        const gtsam::Pose3&                        pose_i,
        const Eigen::Vector3d&                     velocity_i,
        const gtsam::Pose3&                        pose_j,
        const Eigen::Vector3d&                     velocity_j,
        const gtsam::imuBias::ConstantBias&        bias,
        const Eigen::Vector3d&                     gravity,
        gtsam::Vector9&                            residual )
    {
      const gtsam::NavState state_i( pose_i, velocity_i );
      const gtsam::NavState state_j( pose_j, velocity_j );
      const gtsam::Vector9  bias_corrected = pim.biasCorrectedDelta( bias );
      const gtsam::Vector9  xi             = state_i.correctPIM(
          bias_corrected, pim.deltaTij(), gravity, std::nullopt, false );
      gtsam::Matrix9        D_predict_delta;
      const gtsam::NavState predicted =
          state_i.retract( xi, {}, D_predict_delta );
      gtsam::Matrix9 D_error_predicted;
      residual =
          state_j.localCoordinates( predicted, {}, D_error_predicted );

      Eigen::Matrix<double, 9, 3> D_xi_gravity =
          Eigen::Matrix<double, 9, 3>::Zero();
      const Eigen::Matrix3d R_B_W =
          pose_i.rotation().matrix().transpose();
      D_xi_gravity.block<3, 3>( 3, 0 ) =
          0.5 * pim.deltaTij() * pim.deltaTij() * R_B_W;
      D_xi_gravity.block<3, 3>( 6, 0 ) =
          pim.deltaTij() * R_B_W;
      return D_error_predicted * D_predict_delta * D_xi_gravity;
    }

    [[nodiscard]] bool validInput( const InitialMovingMathInput& input,
                                   const InitialMovingCandidate& candidate )
    {
      if ( input.m_poses.size() < 2U ||
           input.m_intervals.size() + 1U != input.m_poses.size() ||
           candidate.m_v_W.size() != input.m_poses.size() ||
           !candidate.m_bg_radps.allFinite() ||
           !candidate.m_g_W.allFinite() ||
           !std::isfinite( input.m_gravity_mps2 ) ||
           !( input.m_gravity_mps2 > 0.0 ) ||
           std::abs( candidate.m_g_W.norm() - input.m_gravity_mps2 ) >
               1e-9 * std::max( 1.0, input.m_gravity_mps2 ) ||
           !input.m_pim.m_acc_cov.allFinite() ||
           !input.m_pim.m_gyr_cov.allFinite() ||
           !input.m_pim.m_integration_cov.allFinite() )
      {
        return false;
      }
      for ( std::size_t index = 0U; index < input.m_poses.size(); ++index )
      {
        if ( !input.m_poses[ index ].m_T_W_B.matrix().allFinite() ||
             !candidate.m_v_W[ index ].allFinite() ||
             ( index > 0U &&
               input.m_poses[ index ].m_timestamp <=
                   input.m_poses[ index - 1U ].m_timestamp ) )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] bool validCovariance(
        const Eigen::MatrixXd& covariance,
        const Eigen::Index     expected_dimension )
    {
      if ( covariance.rows() != expected_dimension ||
           covariance.cols() != expected_dimension ||
           !covariance.allFinite() )
      {
        return false;
      }
      constexpr double kTolerance = 1e-12;
      const double     scale =
          std::max( 1.0, covariance.cwiseAbs().maxCoeff() );
      if ( ( covariance - covariance.transpose() ).cwiseAbs().maxCoeff() >
           kTolerance * scale )
      {
        return false;
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
          0.5 * ( covariance + covariance.transpose() ) );
      return eigen.info() == Eigen::Success &&
             eigen.eigenvalues().minCoeff() >= -kTolerance * scale;
    }

  }  // namespace

  InitialMovingCanonicalResult buildInitialMovingCanonicalSystem(
      const InitialMovingMathInput& input,
      const InitialMovingCandidate& candidate )
  {
    if ( !validInput( input, candidate ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving math input is invalid" );
    }

    const std::size_t  frame_count    = input.m_poses.size();
    const std::size_t  interval_count = input.m_intervals.size();
    const Eigen::Index residual_count =
        static_cast<Eigen::Index>( 9U * interval_count );
    const Eigen::Index candidate_count =
        static_cast<Eigen::Index>( 5U + 3U * frame_count );
    const Eigen::Index visual_count =
        static_cast<Eigen::Index>( 6U * ( frame_count - 1U ) );

    InitialMovingCanonicalSystem system;
    system.m_residual = Eigen::VectorXd::Zero( residual_count );
    system.m_candidate_jacobian =
        Eigen::MatrixXd::Zero( residual_count, candidate_count );
    system.m_visual_pose_jacobian =
        Eigen::MatrixXd::Zero( residual_count, visual_count );
    system.m_pim_covariance =
        Eigen::MatrixXd::Zero( residual_count, residual_count );
    system.m_gravity_tangent_basis =
        gravityTangentBasis( candidate.m_g_W );

    try
    {
      const gtsam::imuBias::ConstantBias bias(
          Eigen::Vector3d::Zero(), candidate.m_bg_radps );
      for ( std::size_t index = 0U; index < interval_count; ++index )
      {
        const InitialMovingPose&     pose_i   = input.m_poses[ index ];
        const InitialMovingPose&     pose_j   = input.m_poses[ index + 1U ];
        const NormalizedImuInterval& interval = input.m_intervals[ index ];
        if ( interval.m_steps.empty() || interval.m_nodes.size() < 2U ||
             interval.m_nodes.size() - 1U != interval.m_steps.size() )
        {
          return error( InitialMovingMathErrorCode::kContract,
                        "initial moving normalized nodes and steps mismatch" );
        }
        if ( interval.m_nodes.front().timestamp !=
                 interval.m_raw.m_t_begin ||
             interval.m_nodes.back().timestamp != interval.m_raw.m_t_end )
        {
          return error(
              InitialMovingMathErrorCode::kContract,
              "initial moving normalized node endpoints do not close" );
        }
        if ( interval.m_raw.m_t_begin != pose_i.m_timestamp ||
             interval.m_raw.m_t_end != pose_j.m_timestamp )
        {
          return error( InitialMovingMathErrorCode::kContract,
                        "initial moving interval endpoints do not match poses" );
        }
        for ( const sensor::ImuMeasurement& node : interval.m_nodes )
        {
          if ( !std::all_of( node.accel_mps2.begin(),
                             node.accel_mps2.end(),
                             []( const double value ) {
                               return std::isfinite( value );
                             } ) ||
               !std::all_of( node.gyro_radps.begin(),
                             node.gyro_radps.end(),
                             []( const double value ) {
                               return std::isfinite( value );
                             } ) )
          {
            return error( InitialMovingMathErrorCode::kContract,
                          "initial moving normalized node is non-finite" );
          }
        }
        std::int64_t summed_duration_ns = 0;
        for ( std::size_t step_index = 0U;
              step_index < interval.m_steps.size(); ++step_index )
        {
          const ImuIntervalStep& step = interval.m_steps[ step_index ];
          const std::int64_t     current_node_ns =
              interval.m_nodes[ step_index ].timestamp.nanoseconds();
          const std::int64_t next_node_ns =
              interval.m_nodes[ step_index + 1U ].timestamp.nanoseconds();
          bool means_match = true;
          for ( std::size_t axis = 0U; axis < 3U; ++axis )
          {
            const Eigen::Index eigen_axis =
                static_cast<Eigen::Index>( axis );
            const double expected_acc =
                0.5 * ( interval.m_nodes[ step_index ].accel_mps2[ axis ] +
                        interval.m_nodes[ step_index + 1U ]
                            .accel_mps2[ axis ] );
            const double expected_gyr =
                0.5 * ( interval.m_nodes[ step_index ].gyro_radps[ axis ] +
                        interval.m_nodes[ step_index + 1U ]
                            .gyro_radps[ axis ] );
            means_match =
                means_match &&
                step.m_acc_mean_mps2( eigen_axis ) == expected_acc &&
                step.m_gyr_mean_radps( eigen_axis ) == expected_gyr;
          }
          const double expected_dt_s =
              static_cast<double>( step.m_dt_ns ) * 1e-9;
          if ( !step.m_acc_mean_mps2.allFinite() ||
               !step.m_gyr_mean_radps.allFinite() || step.m_dt_ns <= 0 ||
               !std::isfinite( step.m_dt_s ) ||
               !( step.m_dt_s > 0.0 ) || step.m_dt_s != expected_dt_s ||
               !means_match ||
               next_node_ns <= current_node_ns ||
               current_node_ns >
                   std::numeric_limits<std::int64_t>::max() - step.m_dt_ns ||
               current_node_ns + step.m_dt_ns != next_node_ns )
          {
            return error(
                InitialMovingMathErrorCode::kContract,
                "initial moving normalized step is invalid" );
          }
          if ( summed_duration_ns >
               std::numeric_limits<std::int64_t>::max() - step.m_dt_ns )
          {
            return error(
                InitialMovingMathErrorCode::kContract,
                "initial moving normalized duration overflows int64" );
          }
          summed_duration_ns += step.m_dt_ns;
        }
        const std::int64_t raw_begin_ns =
            interval.m_raw.m_t_begin.nanoseconds();
        if ( interval.m_duration_ns <= 0 ||
             summed_duration_ns != interval.m_duration_ns ||
             raw_begin_ns >
                 std::numeric_limits<std::int64_t>::max() -
                     interval.m_duration_ns ||
             raw_begin_ns + interval.m_duration_ns !=
                 interval.m_raw.m_t_end.nanoseconds() )
        {
          return error( InitialMovingMathErrorCode::kContract,
                        "initial moving normalized interval does not close" );
        }

        const gtsam::PreintegratedImuMeasurements pim =
            preintegrate( input, candidate, interval );
        const gtsam::Pose3   T_W_B_i( pose_i.m_T_W_B.matrix() );
        const gtsam::Pose3   T_W_B_j( pose_j.m_T_W_B.matrix() );
        gtsam::Matrix96      H_pose_i;
        gtsam::Matrix93      H_velocity_i;
        gtsam::Matrix96      H_pose_j;
        gtsam::Matrix93      H_velocity_j;
        gtsam::Matrix96      H_bias;
        const gtsam::Vector9 residual = pim.computeErrorAndJacobians(
            T_W_B_i, candidate.m_v_W[ index ], T_W_B_j,
            candidate.m_v_W[ index + 1U ], bias, H_pose_i, H_velocity_i,
            H_pose_j, H_velocity_j, H_bias );
        gtsam::Vector9                    gravity_residual;
        const Eigen::Matrix<double, 9, 3> H_gravity = gravityJacobian(
            pim, T_W_B_i, candidate.m_v_W[ index ], T_W_B_j,
            candidate.m_v_W[ index + 1U ], bias, candidate.m_g_W,
            gravity_residual );
        if ( !residual.isApprox( gravity_residual, 1e-11 ) )
        {
          return error( InitialMovingMathErrorCode::kNumerical,
                        "canonical residual paths disagree" );
        }

        const Eigen::Index row =
            static_cast<Eigen::Index>( 9U * index );
        system.m_residual.segment<9>( row ) = residual;
        system.m_candidate_jacobian.block<9, 3>( row, 0 ) =
            H_bias.block<9, 3>( 0, 3 );
        system.m_candidate_jacobian.block<9, 2>( row, 3 ) =
            H_gravity *
            ( input.m_gravity_mps2 *
              system.m_gravity_tangent_basis );
        const Eigen::Index velocity_i =
            static_cast<Eigen::Index>( 5U + 3U * index );
        system.m_candidate_jacobian.block<9, 3>( row, velocity_i ) =
            H_velocity_i;
        system.m_candidate_jacobian.block<9, 3>( row, velocity_i + 3 ) =
            H_velocity_j;
        if ( index > 0U )
        {
          const Eigen::Index pose_i_column =
              static_cast<Eigen::Index>( 6U * ( index - 1U ) );
          system.m_visual_pose_jacobian.block<9, 6>(
              row, pose_i_column ) = H_pose_i;
        }
        const Eigen::Index pose_j_column =
            static_cast<Eigen::Index>( 6U * index );
        system.m_visual_pose_jacobian.block<9, 6>(
            row, pose_j_column ) = H_pose_j;
        system.m_pim_covariance.block<9, 9>( row, row ) =
            pim.preintMeasCov();
      }
    }
    catch ( const std::exception& exception )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    std::string( "initial moving PIM evaluation failed: " ) +
                        exception.what() );
    }
    if ( !system.m_residual.allFinite() ||
         !system.m_candidate_jacobian.allFinite() ||
         !system.m_visual_pose_jacobian.allFinite() ||
         !system.m_pim_covariance.allFinite() ||
         !system.m_gravity_tangent_basis.allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving canonical system is non-finite" );
    }
    return system;
  }

  InitialMovingMatrixResult buildInitialMovingInnovationCovariance(
      const InitialMovingCanonicalSystem& system,
      const Eigen::MatrixXd&              visual_pose_covariance )
  {
    const Eigen::Index residual_count = system.m_residual.rows();
    if ( residual_count <= 0 ||
         system.m_visual_pose_jacobian.rows() != residual_count ||
         system.m_visual_pose_jacobian.cols() !=
             visual_pose_covariance.rows() ||
         !validCovariance( visual_pose_covariance,
                           visual_pose_covariance.rows() ) ||
         !validCovariance( system.m_pim_covariance, residual_count ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving covariance operands are invalid" );
    }
    Eigen::MatrixXd innovation =
        system.m_visual_pose_jacobian * visual_pose_covariance *
            system.m_visual_pose_jacobian.transpose() +
        system.m_pim_covariance;
    if ( !validCovariance( innovation, residual_count ) )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving innovation covariance is invalid" );
    }
    return 0.5 * ( innovation + innovation.transpose() );
  }

  InitialMovingJointResult evaluateInitialMovingJointConsistency(
      const Eigen::VectorXd& residual,
      const Eigen::MatrixXd& candidate_jacobian,
      const Eigen::MatrixXd& innovation_covariance )
  {
    if ( residual.rows() <= 0 ||
         candidate_jacobian.rows() != residual.rows() ||
         candidate_jacobian.cols() <= 0 || !residual.allFinite() ||
         !candidate_jacobian.allFinite() ||
         !validCovariance( innovation_covariance, residual.rows() ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving joint operands are invalid" );
    }

    const Eigen::MatrixXd symmetric =
        0.5 * ( innovation_covariance +
                innovation_covariance.transpose() );
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen( symmetric );
    if ( eigen.info() != Eigen::Success ||
         !eigen.eigenvalues().allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving supported factorization failed" );
    }
    const double lambda_max = eigen.eigenvalues().maxCoeff();
    if ( !std::isfinite( lambda_max ) || !( lambda_max > 0.0 ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving covariance has empty support" );
    }
    constexpr double          kRankTolerance = 1e-10;
    std::vector<Eigen::Index> supported;
    for ( Eigen::Index index = 0; index < eigen.eigenvalues().size();
          ++index )
    {
      if ( eigen.eigenvalues()( index ) >
           kRankTolerance * lambda_max )
      {
        supported.push_back( index );
      }
    }
    if ( supported.empty() )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving covariance has no supported eigenvalue" );
    }

    Eigen::MatrixXd whitening(
        static_cast<Eigen::Index>( supported.size() ), residual.rows() );
    for ( std::size_t row = 0U; row < supported.size(); ++row )
    {
      const Eigen::Index eigen_index = supported[ row ];
      whitening.row( static_cast<Eigen::Index>( row ) ) =
          eigen.eigenvectors().col( eigen_index ).transpose() /
          std::sqrt( eigen.eigenvalues()( eigen_index ) );
    }
    const Eigen::VectorXd whitened_residual = whitening * residual;
    const Eigen::MatrixXd whitened_jacobian =
        whitening * candidate_jacobian;
    if ( !whitened_residual.allFinite() ||
         !whitened_jacobian.allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving whitening is non-finite" );
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        whitened_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if ( !svd.singularValues().allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving supported Jacobian SVD failed" );
    }
    const std::int64_t fitted_rank =
        initialMovingStrictNumericalRank( svd.singularValues() );
    if ( fitted_rank < 0 )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving supported Jacobian rank is invalid" );
    }
    const std::int64_t residual_rank =
        static_cast<std::int64_t>( supported.size() );
    const double statistic = whitened_residual.squaredNorm();
    if ( !std::isfinite( statistic ) )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving joint statistic is non-finite" );
    }
    return InitialMovingJointFit{
        .m_statistic     = statistic,
        .m_residual_rank = residual_rank,
        .m_fitted_rank   = fitted_rank,
        .m_effective_dof = residual_rank - fitted_rank };
  }

  std::int64_t initialMovingStrictNumericalRank(
      const Eigen::VectorXd& singular_values ) noexcept
  {
    if ( singular_values.size() <= 0 || !singular_values.allFinite() ||
         !( singular_values( 0 ) > 0.0 ) )
    {
      return -1;
    }
    constexpr double kRankTolerance = 1e-10;
    const double     threshold      = kRankTolerance * singular_values( 0 );
    std::int64_t     rank           = 0;
    for ( const double sigma : singular_values )
    {
      if ( sigma > threshold )
      {
        ++rank;
      }
    }
    return rank;
  }

  InitialMovingLinearResult solveInitialMovingLinearSystem(
      const Eigen::VectorXd& residual,
      const Eigen::MatrixXd& jacobian,
      const Eigen::MatrixXd& covariance,
      const std::uint32_t    required_rank )
  {
    if ( residual.rows() <= 0 || jacobian.rows() != residual.rows() ||
         jacobian.cols() <= 0 ||
         static_cast<Eigen::Index>( required_rank ) != jacobian.cols() ||
         !residual.allFinite() || !jacobian.allFinite() ||
         !validCovariance( covariance, residual.rows() ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving linear operands are invalid" );
    }
    const Eigen::MatrixXd symmetric =
        0.5 * ( covariance + covariance.transpose() );
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen( symmetric );
    if ( eigen.info() != Eigen::Success ||
         !eigen.eigenvalues().allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving linear covariance factorization failed" );
    }
    const double lambda_max = eigen.eigenvalues().maxCoeff();
    if ( !std::isfinite( lambda_max ) || !( lambda_max > 0.0 ) )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving linear covariance has empty support" );
    }
    constexpr double          kRankTolerance = 1e-10;
    std::vector<Eigen::Index> supported;
    for ( Eigen::Index index = 0; index < eigen.eigenvalues().size();
          ++index )
    {
      if ( eigen.eigenvalues()( index ) >
           kRankTolerance * lambda_max )
      {
        supported.push_back( index );
      }
    }
    Eigen::MatrixXd whitening(
        static_cast<Eigen::Index>( supported.size() ), residual.rows() );
    for ( std::size_t row = 0U; row < supported.size(); ++row )
    {
      const Eigen::Index eigen_index = supported[ row ];
      whitening.row( static_cast<Eigen::Index>( row ) ) =
          eigen.eigenvectors().col( eigen_index ).transpose() /
          std::sqrt( eigen.eigenvalues()( eigen_index ) );
    }
    const Eigen::VectorXd whitened_residual = whitening * residual;
    const Eigen::MatrixXd whitened_jacobian = whitening * jacobian;
    if ( !whitened_residual.allFinite() ||
         !whitened_jacobian.allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving linear whitening is non-finite" );
    }

    Eigen::MatrixXd normalized = whitened_jacobian;
    for ( Eigen::Index column = 0; column < normalized.cols(); ++column )
    {
      const double norm = normalized.col( column ).norm();
      if ( !std::isfinite( norm ) )
      {
        return error( InitialMovingMathErrorCode::kNumerical,
                      "initial moving linear column norm is non-finite" );
      }
      if ( !( norm > 0.0 ) )
      {
        return InitialMovingLinearFailure{
            .m_code                 = InitialMovingLinearFailureCode::kRankDeficient,
            .m_rank                 = 0U,
            .m_required_rank        = required_rank,
            .m_normalized_condition = std::nullopt };
      }
      normalized.col( column ) /= norm;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> normalized_svd(
        normalized, Eigen::ComputeThinU | Eigen::ComputeThinV );
    const std::int64_t rank_signed =
        initialMovingStrictNumericalRank(
            normalized_svd.singularValues() );
    if ( rank_signed < 0 )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving normalized rank is invalid" );
    }
    const std::uint32_t   rank = static_cast<std::uint32_t>( rank_signed );
    std::optional<double> condition;
    if ( rank >= required_rank )
    {
      condition = normalized_svd.singularValues()( 0 ) /
                  normalized_svd.singularValues()(
                      static_cast<Eigen::Index>( required_rank - 1U ) );
      if ( !std::isfinite( *condition ) )
      {
        return error( InitialMovingMathErrorCode::kNumerical,
                      "initial moving normalized condition is non-finite" );
      }
    }
    if ( rank < required_rank )
    {
      return InitialMovingLinearFailure{
          .m_code                 = InitialMovingLinearFailureCode::kRankDeficient,
          .m_rank                 = rank,
          .m_required_rank        = required_rank,
          .m_normalized_condition = condition };
    }
    if ( !initialMovingConditionAccepted( *condition ) )
    {
      return InitialMovingLinearFailure{
          .m_code                 = InitialMovingLinearFailureCode::kIllConditioned,
          .m_rank                 = rank,
          .m_required_rank        = required_rank,
          .m_normalized_condition = condition };
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> physical_svd(
        whitened_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV );
    const Eigen::VectorXd delta = physical_svd.solve( -whitened_residual );
    if ( !delta.allFinite() )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving physical delta is non-finite" );
    }
    return InitialMovingLinearSolution{
        .m_delta                = delta,
        .m_rank                 = rank,
        .m_required_rank        = required_rank,
        .m_normalized_condition = condition,
        .m_normalized_condition_limit =
            kInitialMovingNormalizedConditionLimit };
  }

  bool initialMovingConditionAccepted( const double condition ) noexcept
  {
    return std::isfinite( condition ) &&
           condition <= kInitialMovingNormalizedConditionLimit;
  }

  InitialMovingGyroResult solveInitialMovingGyroBiasFromZero(
      const InitialMovingMathInput& input,
      const Eigen::MatrixXd&        visual_pose_covariance )
  {
    InitialMovingCandidate zero_candidate;
    zero_candidate.m_g_W =
        Eigen::Vector3d( 0.0, 0.0, -input.m_gravity_mps2 );
    zero_candidate.m_v_W.assign( input.m_poses.size(),
                                 Eigen::Vector3d::Zero() );
    InitialMovingCanonicalResult canonical_result =
        buildInitialMovingCanonicalSystem( input, zero_candidate );
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &canonical_result ) )
    {
      return *error_value;
    }
    const InitialMovingCanonicalSystem& system =
        std::get<InitialMovingCanonicalSystem>( canonical_result );
    InitialMovingMatrixResult innovation_result =
        buildInitialMovingInnovationCovariance(
            system, visual_pose_covariance );
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &innovation_result ) )
    {
      return *error_value;
    }
    const Eigen::MatrixXd& innovation =
        std::get<Eigen::MatrixXd>( innovation_result );

    const std::size_t  interval_count = input.m_intervals.size();
    const Eigen::Index gyro_rows =
        static_cast<Eigen::Index>( 3U * interval_count );
    Eigen::VectorXd gyro_residual = Eigen::VectorXd::Zero( gyro_rows );
    Eigen::MatrixXd gyro_jacobian =
        Eigen::MatrixXd::Zero( gyro_rows, 3 );
    Eigen::MatrixXd gyro_covariance =
        Eigen::MatrixXd::Zero( gyro_rows, gyro_rows );
    for ( std::size_t row_interval = 0U;
          row_interval < interval_count; ++row_interval )
    {
      const Eigen::Index source_row =
          static_cast<Eigen::Index>( 9U * row_interval );
      const Eigen::Index target_row =
          static_cast<Eigen::Index>( 3U * row_interval );
      gyro_residual.segment<3>( target_row ) =
          system.m_residual.segment<3>( source_row );
      gyro_jacobian.block<3, 3>( target_row, 0 ) =
          system.m_candidate_jacobian.block<3, 3>( source_row, 0 );
      for ( std::size_t column_interval = 0U;
            column_interval < interval_count; ++column_interval )
      {
        const Eigen::Index source_column =
            static_cast<Eigen::Index>( 9U * column_interval );
        const Eigen::Index target_column =
            static_cast<Eigen::Index>( 3U * column_interval );
        gyro_covariance.block<3, 3>( target_row, target_column ) =
            innovation.block<3, 3>( source_row, source_column );
      }
    }

    InitialMovingLinearResult solve_result =
        solveInitialMovingLinearSystem(
            gyro_residual, gyro_jacobian, gyro_covariance, 3U );
    if ( const auto* failure =
             std::get_if<InitialMovingLinearFailure>( &solve_result ) )
    {
      return *failure;
    }
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &solve_result ) )
    {
      return *error_value;
    }
    const InitialMovingLinearSolution& solved =
        std::get<InitialMovingLinearSolution>( solve_result );
    return InitialMovingGyroSolution{
        .m_bg_radps             = solved.m_delta,
        .m_rank                 = solved.m_rank,
        .m_normalized_condition = solved.m_normalized_condition };
  }

  InitialMovingGravityVelocityResult solveInitialMovingGravityVelocity(
      const InitialMovingMathInput& input,
      const Eigen::MatrixXd&        visual_pose_covariance,
      const Eigen::Vector3d&        gyro_bias_radps )
  {
    InitialMovingCandidate candidate;
    candidate.m_bg_radps = gyro_bias_radps;
    candidate.m_g_W =
        Eigen::Vector3d( 0.0, 0.0, -input.m_gravity_mps2 );
    candidate.m_v_W.assign( input.m_poses.size(),
                            Eigen::Vector3d::Zero() );
    InitialMovingCanonicalResult canonical_result =
        buildInitialMovingCanonicalSystem( input, candidate );
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &canonical_result ) )
    {
      return *error_value;
    }
    const InitialMovingCanonicalSystem& system =
        std::get<InitialMovingCanonicalSystem>( canonical_result );
    InitialMovingMatrixResult innovation_result =
        buildInitialMovingInnovationCovariance(
            system, visual_pose_covariance );
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &innovation_result ) )
    {
      return *error_value;
    }
    const Eigen::MatrixXd& innovation =
        std::get<Eigen::MatrixXd>( innovation_result );

    const std::size_t  interval_count = input.m_intervals.size();
    const std::size_t  frame_count    = input.m_poses.size();
    const Eigen::Index pv_rows =
        static_cast<Eigen::Index>( 6U * interval_count );
    const Eigen::Index pv_columns =
        static_cast<Eigen::Index>( 2U + 3U * frame_count );
    Eigen::VectorXd pv_residual = Eigen::VectorXd::Zero( pv_rows );
    Eigen::MatrixXd pv_jacobian =
        Eigen::MatrixXd::Zero( pv_rows, pv_columns );
    Eigen::MatrixXd pv_covariance =
        Eigen::MatrixXd::Zero( pv_rows, pv_rows );
    for ( std::size_t row_interval = 0U;
          row_interval < interval_count; ++row_interval )
    {
      const Eigen::Index source_row =
          static_cast<Eigen::Index>( 9U * row_interval + 3U );
      const Eigen::Index target_row =
          static_cast<Eigen::Index>( 6U * row_interval );
      pv_residual.segment<6>( target_row ) =
          system.m_residual.segment<6>( source_row );
      pv_jacobian.block( target_row, 0, 6, pv_columns ) =
          system.m_candidate_jacobian.block(
              source_row, 3, 6, pv_columns );
      for ( std::size_t column_interval = 0U;
            column_interval < interval_count; ++column_interval )
      {
        const Eigen::Index source_column =
            static_cast<Eigen::Index>( 9U * column_interval + 3U );
        const Eigen::Index target_column =
            static_cast<Eigen::Index>( 6U * column_interval );
        pv_covariance.block<6, 6>( target_row, target_column ) =
            innovation.block<6, 6>( source_row, source_column );
      }
    }
    const std::uint32_t required_rank =
        static_cast<std::uint32_t>( 3U * frame_count + 2U );
    InitialMovingLinearResult solve_result =
        solveInitialMovingLinearSystem(
            pv_residual, pv_jacobian, pv_covariance, required_rank );
    if ( const auto* failure =
             std::get_if<InitialMovingLinearFailure>( &solve_result ) )
    {
      return *failure;
    }
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &solve_result ) )
    {
      return *error_value;
    }
    const InitialMovingLinearSolution& solved =
        std::get<InitialMovingLinearSolution>( solve_result );
    const Eigen::Vector3d refined_gravity =
        candidate.m_g_W +
        input.m_gravity_mps2 * system.m_gravity_tangent_basis *
            solved.m_delta.segment<2>( 0 );
    if ( !refined_gravity.allFinite() ||
         !( refined_gravity.norm() > 0.0 ) )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving gravity update is invalid" );
    }
    InitialMovingGravityVelocitySolution result;
    result.m_g_W =
        input.m_gravity_mps2 * refined_gravity.normalized();
    result.m_v_W.resize( frame_count );
    for ( std::size_t index = 0U; index < frame_count; ++index )
    {
      result.m_v_W[ index ] = solved.m_delta.segment<3>(
          static_cast<Eigen::Index>( 2U + 3U * index ) );
    }
    result.m_rank                 = solved.m_rank;
    result.m_normalized_condition = solved.m_normalized_condition;
    return result;
  }

  InitialMovingCandidateResult solveInitialMovingCandidate(
      const InitialMovingMathInput&   input,
      const Eigen::MatrixXd&          visual_pose_covariance,
      const InitialMovingSolverLimits limits )
  {
    if ( limits.m_max_refinement_corrections > 8U )
    {
      return error( InitialMovingMathErrorCode::kContract,
                    "initial moving refinement budget exceeds eight" );
    }
    const auto candidateFailure = [](
                                      const InitialMovingLinearFailure&                   failure,
                                      std::optional<InitialMovingGyroSolution>            gyro             = std::nullopt,
                                      std::optional<InitialMovingGravityVelocitySolution> gravity_velocity = std::nullopt ) {
      return InitialMovingCandidateFailure{
          .m_code =
              failure.m_code ==
                      InitialMovingLinearFailureCode::kRankDeficient
                  ? InitialMovingCandidateFailureCode::kRankDeficient
                  : InitialMovingCandidateFailureCode::kIllConditioned,
          .m_linear_failure   = failure,
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = std::move( gyro ),
          .m_gravity_velocity = std::move( gravity_velocity ) };
    };

    InitialMovingGyroResult gyro_result =
        solveInitialMovingGyroBiasFromZero(
            input, visual_pose_covariance );
    if ( const auto* failure =
             std::get_if<InitialMovingLinearFailure>( &gyro_result ) )
    {
      return candidateFailure( *failure );
    }
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &gyro_result ) )
    {
      return *error_value;
    }
    const InitialMovingGyroSolution gyro =
        std::get<InitialMovingGyroSolution>( gyro_result );

    InitialMovingGravityVelocityResult gv_result =
        solveInitialMovingGravityVelocity(
            input, visual_pose_covariance, gyro.m_bg_radps );
    if ( const auto* failure =
             std::get_if<InitialMovingLinearFailure>( &gv_result ) )
    {
      return candidateFailure( *failure, gyro );
    }
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &gv_result ) )
    {
      return *error_value;
    }
    const InitialMovingGravityVelocitySolution gravity_velocity =
        std::get<InitialMovingGravityVelocitySolution>( gv_result );

    InitialMovingCandidate candidate{
        .m_bg_radps = gyro.m_bg_radps,
        .m_g_W      = gravity_velocity.m_g_W,
        .m_v_W      = gravity_velocity.m_v_W };
    std::optional<InitialMovingCanonicalSystem> final_system;
    std::optional<Eigen::MatrixXd>              final_innovation;
    std::uint32_t                               refinement_rounds = 0U;
    bool                                        converged         = false;
    for ( std::uint32_t iteration = 0U;
          iteration <= limits.m_max_refinement_corrections; ++iteration )
    {
      InitialMovingCanonicalResult canonical_result =
          buildInitialMovingCanonicalSystem( input, candidate );
      if ( const auto* error_value =
               std::get_if<InitialMovingMathError>( &canonical_result ) )
      {
        return *error_value;
      }
      InitialMovingCanonicalSystem system =
          std::get<InitialMovingCanonicalSystem>(
              std::move( canonical_result ) );
      InitialMovingMatrixResult innovation_result =
          buildInitialMovingInnovationCovariance(
              system, visual_pose_covariance );
      if ( const auto* error_value =
               std::get_if<InitialMovingMathError>( &innovation_result ) )
      {
        return *error_value;
      }
      Eigen::MatrixXd innovation =
          std::get<Eigen::MatrixXd>( std::move( innovation_result ) );
      InitialMovingLinearResult linear_result =
          solveInitialMovingLinearSystem(
              system.m_residual, system.m_candidate_jacobian,
              innovation,
              static_cast<std::uint32_t>(
                  system.m_candidate_jacobian.cols() ) );
      if ( const auto* failure =
               std::get_if<InitialMovingLinearFailure>( &linear_result ) )
      {
        return candidateFailure( *failure, gyro, gravity_velocity );
      }
      if ( const auto* error_value =
               std::get_if<InitialMovingMathError>( &linear_result ) )
      {
        return *error_value;
      }
      const InitialMovingLinearSolution& linear =
          std::get<InitialMovingLinearSolution>( linear_result );
      if ( linear.m_delta.lpNorm<Eigen::Infinity>() <= 1e-8 )
      {
        final_system     = std::move( system );
        final_innovation = std::move( innovation );
        converged        = true;
        break;
      }
      if ( iteration == limits.m_max_refinement_corrections )
      {
        break;
      }

      candidate.m_bg_radps += linear.m_delta.segment<3>( 0 );
      const Eigen::Vector3d refined_gravity =
          candidate.m_g_W +
          input.m_gravity_mps2 * system.m_gravity_tangent_basis *
              linear.m_delta.segment<2>( 3 );
      if ( !refined_gravity.allFinite() ||
           !( refined_gravity.norm() > 0.0 ) )
      {
        return error( InitialMovingMathErrorCode::kNumerical,
                      "initial moving joint gravity update is invalid" );
      }
      candidate.m_g_W =
          input.m_gravity_mps2 * refined_gravity.normalized();
      for ( std::size_t index = 0U; index < candidate.m_v_W.size();
            ++index )
      {
        candidate.m_v_W[ index ] += linear.m_delta.segment<3>(
            static_cast<Eigen::Index>( 5U + 3U * index ) );
      }
      ++refinement_rounds;
    }
    if ( !converged || !final_system.has_value() ||
         !final_innovation.has_value() )
    {
      return InitialMovingCandidateFailure{
          .m_code             = InitialMovingCandidateFailureCode::kNoConvergence,
          .m_linear_failure   = std::nullopt,
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = gyro,
          .m_gravity_velocity = gravity_velocity };
    }

    InitialMovingJointResult joint_result =
        evaluateInitialMovingJointConsistency(
            final_system->m_residual,
            final_system->m_candidate_jacobian,
            *final_innovation );
    if ( const auto* error_value =
             std::get_if<InitialMovingMathError>( &joint_result ) )
    {
      return *error_value;
    }
    const InitialMovingJointFit joint =
        std::get<InitialMovingJointFit>( joint_result );
    if ( joint.m_effective_dof <= 0 )
    {
      return InitialMovingCandidateFailure{
          .m_code =
              InitialMovingCandidateFailureCode::kJointDofInsufficient,
          .m_linear_failure   = std::nullopt,
          .m_joint_fit        = joint,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = gyro,
          .m_gravity_velocity = gravity_velocity };
    }
    double joint_limit = 0.0;
    try
    {
      joint_limit = boost::math::quantile(
          boost::math::chi_squared_distribution<double>(
              static_cast<double>( joint.m_effective_dof ) ),
          0.99 );
    }
    catch ( const std::exception& exception )
    {
      return error(
          InitialMovingMathErrorCode::kNumerical,
          std::string( "initial moving chi-square limit failed: " ) +
              exception.what() );
    }
    if ( !std::isfinite( joint_limit ) || !( joint_limit > 0.0 ) )
    {
      return error( InitialMovingMathErrorCode::kNumerical,
                    "initial moving chi-square limit is invalid" );
    }
    if ( !initialMovingJointFitAccepted(
             joint.m_statistic, joint_limit ) )
    {
      return InitialMovingCandidateFailure{
          .m_code             = InitialMovingCandidateFailureCode::kJointInconsistent,
          .m_linear_failure   = std::nullopt,
          .m_joint_fit        = joint,
          .m_joint_limit      = joint_limit,
          .m_gyro             = gyro,
          .m_gravity_velocity = gravity_velocity };
    }
    return InitialMovingCandidateSolution{
        .m_candidate         = std::move( candidate ),
        .m_joint_fit         = joint,
        .m_joint_limit       = joint_limit,
        .m_refinement_rounds = refinement_rounds,
        .m_gyro              = gyro,
        .m_gravity_velocity  = gravity_velocity };
  }

  bool initialMovingJointFitAccepted( const double statistic,
                                      const double limit ) noexcept
  {
    return std::isfinite( statistic ) && statistic >= 0.0 &&
           std::isfinite( limit ) && limit > 0.0 &&
           statistic <= limit;
  }

}  // namespace phad::estimator::internal

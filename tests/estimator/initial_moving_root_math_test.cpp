#include "phad/estimator/internal/initial_moving_root_math.hpp"

#include <gtest/gtest.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <Eigen/Geometry>
#include <array>
#include <limits>
#include <memory>
#include <variant>

#include "phad/estimator/internal/imu_interval.hpp"
#include "tests/estimator/initial_moving_root_fixture.hpp"

namespace
{

  using phad::common::Timestamp;
  using phad::estimator::internal::buildInitialMovingCanonicalSystem;
  using phad::estimator::internal::buildInitialMovingInnovationCovariance;
  using phad::estimator::internal::evaluateInitialMovingJointConsistency;
  using phad::estimator::internal::InitialMovingCandidate;
  using phad::estimator::internal::InitialMovingCandidateSolution;
  using phad::estimator::internal::InitialMovingCanonicalSystem;
  using phad::estimator::internal::initialMovingConditionAccepted;
  using phad::estimator::internal::InitialMovingGravityVelocitySolution;
  using phad::estimator::internal::InitialMovingGyroSolution;
  using phad::estimator::internal::InitialMovingJointFit;
  using phad::estimator::internal::initialMovingJointFitAccepted;
  using phad::estimator::internal::InitialMovingLinearSolution;
  using phad::estimator::internal::InitialMovingMathError;
  using phad::estimator::internal::InitialMovingMathInput;
  using phad::estimator::internal::InitialMovingPimConfig;
  using phad::estimator::internal::InitialMovingPose;
  using phad::estimator::internal::InitialMovingSolverLimits;
  using phad::estimator::internal::initialMovingStrictNumericalRank;
  using phad::estimator::internal::kInitialMovingNormalizedConditionLimit;
  using phad::estimator::internal::NormalizedImuInterval;
  using phad::estimator::internal::normalizeRawImuInterval;
  using phad::estimator::internal::solveInitialMovingCandidate;
  using phad::estimator::internal::solveInitialMovingGravityVelocity;
  using phad::estimator::internal::solveInitialMovingGyroBiasFromZero;
  using phad::estimator::internal::solveInitialMovingLinearSystem;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::RawImuInterval;

  [[nodiscard]] ImuMeasurement imu( const std::int64_t timestamp_ns )
  {
    return ImuMeasurement{
        .timestamp  = Timestamp{ timestamp_ns },
        .accel_mps2 = std::array<double, 3>{ 0.0, 0.0, 9.81 },
        .gyro_radps = std::array<double, 3>{ 0.0, 0.0, 0.0 } };
  }

  [[nodiscard]] NormalizedImuInterval interval()
  {
    constexpr std::int64_t kEndNs     = 50'000'000;
    auto                   normalized = normalizeRawImuInterval(
        RawImuInterval{
                              .m_t_begin = Timestamp{ 0 },
                              .m_t_end   = Timestamp{ kEndNs },
                              .m_samples = { imu( 0 ), imu( kEndNs / 2 ), imu( kEndNs ) } },
        std::nullopt, Timestamp{ kEndNs } );
    return std::get<NormalizedImuInterval>( std::move( normalized ) );
  }

  [[nodiscard]] InitialMovingPimConfig pimConfig()
  {
    return InitialMovingPimConfig{
        .m_acc_cov         = 0.04 * Eigen::Matrix3d::Identity(),
        .m_gyr_cov         = 0.0009 * Eigen::Matrix3d::Identity(),
        .m_integration_cov = 0.006 * Eigen::Matrix3d::Identity() };
  }

}  // namespace

TEST( InitialMovingRootMath, CanonicalResidualMatchesGtsamPimError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };
  pose_i.m_T_W_B       = Eigen::Isometry3d::Identity();

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };
  pose_j.m_T_W_B       = Eigen::Isometry3d::Identity();
  pose_j.m_T_W_B.linear() =
      Eigen::AngleAxisd( 0.01, Eigen::Vector3d::UnitY() )
          .toRotationMatrix();
  pose_j.m_T_W_B.translation() = Eigen::Vector3d( 0.02, -0.01, 0.03 );

  InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { interval() },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d( 0.1, -0.2, 0.3 ),
                      Eigen::Vector3d( 0.12, -0.18, 0.28 ) } };

  auto built = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE(
      std::holds_alternative<InitialMovingCanonicalSystem>( built ) );
  const InitialMovingCanonicalSystem& system =
      std::get<InitialMovingCanonicalSystem>( built );

  auto params =
      std::make_shared<gtsam::PreintegrationParams>( candidate.m_g_W );
  params->setAccelerometerCovariance( input.m_pim.m_acc_cov );
  params->setGyroscopeCovariance( input.m_pim.m_gyr_cov );
  params->setIntegrationCovariance( input.m_pim.m_integration_cov );
  const gtsam::imuBias::ConstantBias  bias( Eigen::Vector3d::Zero(),
                                            candidate.m_bg_radps );
  gtsam::PreintegratedImuMeasurements pim( params, bias );
  for ( const auto& step : input.m_intervals.front().m_steps )
  {
    pim.integrateMeasurement( step.m_acc_mean_mps2, step.m_gyr_mean_radps,
                              step.m_dt_s );
  }
  const gtsam::Vector9 expected = pim.computeErrorAndJacobians(
      gtsam::Pose3( pose_i.m_T_W_B.matrix() ), candidate.m_v_W[ 0 ],
      gtsam::Pose3( pose_j.m_T_W_B.matrix() ), candidate.m_v_W[ 1 ], bias );

  ASSERT_EQ( system.m_residual.size(), 9 );
  EXPECT_TRUE( system.m_residual.isApprox( expected, 1e-12 ) );
}

TEST( InitialMovingRootMath, EmptyNormalizedIntervalIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  malformed.m_steps.clear();
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NormalizedStepDurationMismatchIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  ++malformed.m_steps.front().m_dt_ns;
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NormalizedIntervalClosureMismatchIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  ++malformed.m_duration_ns;
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NonfiniteNormalizedStepIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  malformed.m_steps.front().m_acc_mean_mps2.x() =
      std::numeric_limits<double>::infinity();
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, MissingNormalizedNodesIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  malformed.m_nodes.clear();
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NormalizedNodeEndpointMismatchIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed     = interval();
  malformed.m_nodes.front().timestamp = Timestamp{ 1 };
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NormalizedNodeStepMismatchIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed   = interval();
  malformed.m_nodes[ 1U ].timestamp = Timestamp{ 25'000'001 };
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NonfiniteNormalizedNodeIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  malformed.m_nodes[ 1U ].accel_mps2[ 0U ] =
      std::numeric_limits<double>::infinity();
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, NormalizedNodeMeanMismatchIsContractError )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };

  NormalizedImuInterval malformed = interval();
  malformed.m_steps.front().m_acc_mean_mps2.x() += 1.0;
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { std::move( malformed ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d::Zero(),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kContract );
}

TEST( InitialMovingRootMath, CandidateJacobianMatchesCentralDifference )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };
  pose_i.m_T_W_B       = Eigen::Isometry3d::Identity();

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };
  pose_j.m_T_W_B       = Eigen::Isometry3d::Identity();
  pose_j.m_T_W_B.linear() =
      ( Eigen::AngleAxisd( 0.008, Eigen::Vector3d::UnitX() ) *
        Eigen::AngleAxisd( -0.011, Eigen::Vector3d::UnitZ() ) )
          .toRotationMatrix();
  pose_j.m_T_W_B.translation() = Eigen::Vector3d( 0.02, -0.01, 0.03 );

  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { interval() },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d( 0.002, -0.003, 0.004 ),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d( 0.1, -0.2, 0.3 ),
                      Eigen::Vector3d( 0.12, -0.18, 0.28 ) } };
  const auto nominal_result =
      buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
      nominal_result ) );
  const InitialMovingCanonicalSystem nominal =
      std::get<InitialMovingCanonicalSystem>( nominal_result );

  constexpr double kStep = 1e-6;
  Eigen::MatrixXd  numerical =
      Eigen::MatrixXd::Zero( nominal.m_residual.size(), 11 );
  const Eigen::Matrix<double, 3, 2> independent_gravity_basis =
      ( Eigen::Matrix<double, 3, 2>() << 1.0, 0.0,
        0.0, -1.0,
        0.0, 0.0 )
          .finished();
  for ( Eigen::Index column = 0; column < numerical.cols(); ++column )
  {
    InitialMovingCandidate plus  = candidate;
    InitialMovingCandidate minus = candidate;
    if ( column < 3 )
    {
      plus.m_bg_radps( column ) += kStep;
      minus.m_bg_radps( column ) -= kStep;
    }
    else if ( column < 5 )
    {
      const Eigen::Index    tangent_column = column - 3;
      const Eigen::Vector3d plus_gravity =
          candidate.m_g_W +
          input.m_gravity_mps2 *
              independent_gravity_basis.col( tangent_column ) * kStep;
      const Eigen::Vector3d minus_gravity =
          candidate.m_g_W -
          input.m_gravity_mps2 *
              independent_gravity_basis.col( tangent_column ) * kStep;
      plus.m_g_W =
          input.m_gravity_mps2 * plus_gravity.normalized();
      minus.m_g_W =
          input.m_gravity_mps2 * minus_gravity.normalized();
    }
    else
    {
      const Eigen::Index velocity_column = column - 5;
      const std::size_t  velocity_index =
          static_cast<std::size_t>( velocity_column / 3 );
      const Eigen::Index axis = velocity_column % 3;
      plus.m_v_W[ velocity_index ]( axis ) += kStep;
      minus.m_v_W[ velocity_index ]( axis ) -= kStep;
    }
    const auto plus_result =
        buildInitialMovingCanonicalSystem( input, plus );
    const auto minus_result =
        buildInitialMovingCanonicalSystem( input, minus );
    ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
        plus_result ) );
    ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
        minus_result ) );
    numerical.col( column ) =
        ( std::get<InitialMovingCanonicalSystem>( plus_result ).m_residual -
          std::get<InitialMovingCanonicalSystem>( minus_result ).m_residual ) /
        ( 2.0 * kStep );
  }

  const double max_abs =
      ( numerical - nominal.m_candidate_jacobian ).cwiseAbs().maxCoeff();
  EXPECT_LT( max_abs, 2e-6 ) << "max_abs_error=" << max_abs;
}

TEST( InitialMovingRootMath,
      NonRootVisualPoseJacobianMatchesCentralDifference )
{
  InitialMovingPose pose_i;
  pose_i.m_frame_index = 4U;
  pose_i.m_timestamp   = Timestamp{ 0 };
  pose_i.m_T_W_B       = Eigen::Isometry3d::Identity();

  InitialMovingPose pose_j;
  pose_j.m_frame_index = 5U;
  pose_j.m_timestamp   = Timestamp{ 50'000'000 };
  pose_j.m_T_W_B.linear() =
      Eigen::AngleAxisd( 0.01, Eigen::Vector3d::UnitY() )
          .toRotationMatrix();
  pose_j.m_T_W_B.translation() = Eigen::Vector3d( 0.02, -0.01, 0.03 );

  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { interval() },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d( 0.002, -0.003, 0.004 ),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d( 0.1, -0.2, 0.3 ),
                      Eigen::Vector3d( 0.12, -0.18, 0.28 ) } };
  const auto nominal_result =
      buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
      nominal_result ) );
  const InitialMovingCanonicalSystem nominal =
      std::get<InitialMovingCanonicalSystem>( nominal_result );

  constexpr double            kStep = 1e-6;
  Eigen::Matrix<double, 9, 6> numerical;
  const gtsam::Pose3          nominal_pose_j( pose_j.m_T_W_B.matrix() );
  for ( Eigen::Index column = 0; column < 6; ++column )
  {
    gtsam::Vector6 delta         = gtsam::Vector6::Zero();
    delta( column )              = kStep;
    InitialMovingMathInput plus  = input;
    InitialMovingMathInput minus = input;
    plus.m_poses[ 1U ].m_T_W_B.matrix() =
        nominal_pose_j.retract( delta ).matrix();
    minus.m_poses[ 1U ].m_T_W_B.matrix() =
        nominal_pose_j.retract( -delta ).matrix();
    const auto plus_result =
        buildInitialMovingCanonicalSystem( plus, candidate );
    const auto minus_result =
        buildInitialMovingCanonicalSystem( minus, candidate );
    ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
        plus_result ) );
    ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
        minus_result ) );
    numerical.col( column ) =
        ( std::get<InitialMovingCanonicalSystem>( plus_result ).m_residual -
          std::get<InitialMovingCanonicalSystem>( minus_result ).m_residual ) /
        ( 2.0 * kStep );
  }

  const double max_abs =
      ( numerical - nominal.m_visual_pose_jacobian )
          .cwiseAbs()
          .maxCoeff();
  EXPECT_LT( max_abs, 2e-6 ) << "max_abs_error=" << max_abs;
}

TEST( InitialMovingRootMath, InnovationKeepsVisualCrossPoseCovariance )
{
  InitialMovingCanonicalSystem system;
  system.m_residual           = Eigen::Vector2d::Zero();
  system.m_candidate_jacobian = Eigen::Matrix<double, 2, 1>::Ones();
  system.m_visual_pose_jacobian =
      ( Eigen::Matrix2d() << 1.0, 2.0, -0.5, 1.5 ).finished();
  system.m_pim_covariance =
      ( Eigen::Matrix2d() << 0.4, 0.0, 0.0, 0.7 ).finished();
  const Eigen::Matrix2d visual_covariance =
      ( Eigen::Matrix2d() << 2.0, 0.6, 0.6, 3.0 ).finished();
  const Eigen::Matrix2d expected =
      system.m_visual_pose_jacobian * visual_covariance *
          system.m_visual_pose_jacobian.transpose() +
      system.m_pim_covariance;

  const auto result =
      buildInitialMovingInnovationCovariance( system, visual_covariance );
  ASSERT_TRUE( std::holds_alternative<Eigen::MatrixXd>( result ) );
  EXPECT_TRUE(
      std::get<Eigen::MatrixXd>( result ).isApprox( expected, 1e-15 ) );

  Eigen::Matrix2d asymmetric = visual_covariance;
  asymmetric( 0, 1 ) += 0.1;
  const auto invalid =
      buildInitialMovingInnovationCovariance( system, asymmetric );
  EXPECT_TRUE( std::holds_alternative<InitialMovingMathError>( invalid ) );
}

TEST( InitialMovingRootMath, PimCovarianceIsFreshBlockDiagonal )
{
  const auto make_interval = []( const std::int64_t begin_ns,
                                 const std::int64_t end_ns ) {
    auto normalized = normalizeRawImuInterval(
        RawImuInterval{
            .m_t_begin = Timestamp{ begin_ns },
            .m_t_end   = Timestamp{ end_ns },
            .m_samples = { imu( begin_ns ), imu( ( begin_ns + end_ns ) / 2 ),
                           imu( end_ns ) } },
        Timestamp{ begin_ns }, Timestamp{ end_ns } );
    return std::get<NormalizedImuInterval>( std::move( normalized ) );
  };
  InitialMovingPose pose_0;
  pose_0.m_frame_index = 0U;
  pose_0.m_timestamp   = Timestamp{ 0 };
  InitialMovingPose pose_1;
  pose_1.m_frame_index = 1U;
  pose_1.m_timestamp   = Timestamp{ 50'000'000 };
  InitialMovingPose pose_2;
  pose_2.m_frame_index = 2U;
  pose_2.m_timestamp   = Timestamp{ 100'000'000 };
  const InitialMovingMathInput input{
      .m_poses        = { pose_0, pose_1, pose_2 },
      .m_intervals    = { make_interval( 0, 50'000'000 ),
                          make_interval( 50'000'000, 100'000'000 ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const InitialMovingCandidate candidate{
      .m_bg_radps = Eigen::Vector3d( 0.01, -0.02, 0.03 ),
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero() } };

  const auto result = buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE(
      std::holds_alternative<InitialMovingCanonicalSystem>( result ) );
  const Eigen::MatrixXd& actual =
      std::get<InitialMovingCanonicalSystem>( result ).m_pim_covariance;
  ASSERT_EQ( actual.rows(), 18 );
  ASSERT_EQ( actual.cols(), 18 );
  EXPECT_TRUE( ( actual.block<9, 9>( 0, 9 ).isZero( 0.0 ) ) );
  EXPECT_TRUE( ( actual.block<9, 9>( 9, 0 ).isZero( 0.0 ) ) );

  auto params =
      std::make_shared<gtsam::PreintegrationParams>( candidate.m_g_W );
  params->setAccelerometerCovariance( input.m_pim.m_acc_cov );
  params->setGyroscopeCovariance( input.m_pim.m_gyr_cov );
  params->setIntegrationCovariance( input.m_pim.m_integration_cov );
  const gtsam::imuBias::ConstantBias bias( Eigen::Vector3d::Zero(),
                                           candidate.m_bg_radps );
  for ( std::size_t index = 0U; index < input.m_intervals.size(); ++index )
  {
    gtsam::PreintegratedImuMeasurements pim( params, bias );
    for ( const auto& step : input.m_intervals[ index ].m_steps )
    {
      pim.integrateMeasurement( step.m_acc_mean_mps2,
                                step.m_gyr_mean_radps, step.m_dt_s );
    }
    EXPECT_TRUE( ( actual.block<9, 9>(
                             static_cast<Eigen::Index>( 9U * index ),
                             static_cast<Eigen::Index>( 9U * index ) )
                       .isApprox( pim.preintMeasCov(), 1e-15 ) ) );
  }
}

TEST( InitialMovingRootMath, SupportedSubspaceWhiteningMatchesDenseOracle )
{
  const Eigen::Vector3d             residual( 2.0, 1.0, 100.0 );
  const Eigen::Matrix<double, 3, 1> jacobian =
      ( Eigen::Matrix<double, 3, 1>() << 1.0, 1.0, 5.0 ).finished();
  const Eigen::Matrix3d covariance =
      ( Eigen::Matrix3d() << 4.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 0.0 )
          .finished();

  const auto result = evaluateInitialMovingJointConsistency(
      residual, jacobian, covariance );
  ASSERT_TRUE( std::holds_alternative<InitialMovingJointFit>( result ) );
  const InitialMovingJointFit& fit =
      std::get<InitialMovingJointFit>( result );
  EXPECT_DOUBLE_EQ( fit.m_statistic, 2.0 );
  EXPECT_EQ( fit.m_residual_rank, 2 );
  EXPECT_EQ( fit.m_fitted_rank, 1 );
  EXPECT_EQ( fit.m_effective_dof, 1 );
}

TEST( InitialMovingRootMath, OverflowingJointStatisticIsNumericalError )
{
  const Eigen::Vector<double, 1> residual =
      Eigen::Vector<double, 1>::Constant( 1e200 );
  const Eigen::Matrix<double, 1, 1> jacobian =
      Eigen::Matrix<double, 1, 1>::Identity();
  const Eigen::Matrix<double, 1, 1> covariance =
      Eigen::Matrix<double, 1, 1>::Identity();

  const auto result = evaluateInitialMovingJointConsistency(
      residual, jacobian, covariance );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kNumerical );
}

TEST( InitialMovingRootMath, SupportedSubspaceReportsZeroEffectiveDof )
{
  const Eigen::VectorXd residual   = Eigen::VectorXd::Zero( 5 );
  const Eigen::MatrixXd jacobian   = Eigen::MatrixXd::Identity( 5, 5 );
  const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity( 5, 5 );

  const auto result = evaluateInitialMovingJointConsistency(
      residual, jacobian, covariance );
  ASSERT_TRUE( std::holds_alternative<InitialMovingJointFit>( result ) );
  const InitialMovingJointFit& fit =
      std::get<InitialMovingJointFit>( result );
  EXPECT_DOUBLE_EQ( fit.m_statistic, 0.0 );
  EXPECT_EQ( fit.m_residual_rank, 5 );
  EXPECT_EQ( fit.m_fitted_rank, 5 );
  EXPECT_EQ( fit.m_effective_dof, 0 );
}

TEST( InitialMovingRootMath, HealthySupportedSystemCanRejectJointFit )
{
  Eigen::VectorXd residual = Eigen::VectorXd::Zero( 5 );
  residual( 0 )            = 100.0;
  const Eigen::MatrixXd jacobian =
      Eigen::MatrixXd::Identity( 5, 3 );
  const Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity( 5, 5 );

  const auto result = evaluateInitialMovingJointConsistency(
      residual, jacobian, covariance );
  ASSERT_TRUE( std::holds_alternative<InitialMovingJointFit>( result ) );
  const InitialMovingJointFit& fit =
      std::get<InitialMovingJointFit>( result );
  EXPECT_DOUBLE_EQ( fit.m_statistic, 10'000.0 );
  EXPECT_EQ( fit.m_residual_rank, 5 );
  EXPECT_EQ( fit.m_fitted_rank, 3 );
  EXPECT_EQ( fit.m_effective_dof, 2 );
  constexpr double kChiSquare2At99Percent = 9.21034037197618;
  EXPECT_FALSE( initialMovingJointFitAccepted(
      fit.m_statistic, kChiSquare2At99Percent ) );
}

TEST( InitialMovingRootMath, NumericalRankUsesStrictRelativeThreshold )
{
  const double    sigma_max = 2.0;
  const double    boundary  = 1e-10 * sigma_max;
  Eigen::Vector3d singular_values(
      sigma_max, boundary,
      std::nextafter( boundary, std::numeric_limits<double>::infinity() ) );

  EXPECT_EQ( initialMovingStrictNumericalRank( singular_values ), 2 );
}

TEST( InitialMovingRootMath, WhitenedGlsMatchesDenseOracle )
{
  const Eigen::Vector4d             residual( 1.0, -2.0, 0.5, 3.0 );
  const Eigen::Matrix<double, 4, 2> jacobian =
      ( Eigen::Matrix<double, 4, 2>() << 1.0, 0.0,
        0.0, 2.0,
        1.0, 1.0,
        -1.0, 0.5 )
          .finished();
  const Eigen::Matrix4d covariance =
      ( Eigen::Matrix4d() << 2.0, 0.2, 0.0, 0.0,
        0.2, 1.5, 0.1, 0.0,
        0.0, 0.1, 1.2, 0.15,
        0.0, 0.0, 0.15, 1.8 )
          .finished();
  const Eigen::LLT<Eigen::Matrix4d> llt( covariance );
  ASSERT_EQ( llt.info(), Eigen::Success );
  const Eigen::Matrix4d lower = llt.matrixL();
  const Eigen::Vector4d rw =
      lower.triangularView<Eigen::Lower>().solve( residual );
  const Eigen::Matrix<double, 4, 2> Jw =
      lower.triangularView<Eigen::Lower>().solve( jacobian );
  const Eigen::Vector2d expected =
      -( Jw.transpose() * Jw )
           .ldlt()
           .solve( Jw.transpose() * rw );

  const auto result = solveInitialMovingLinearSystem(
      residual, jacobian, covariance, 2U );
  ASSERT_TRUE(
      std::holds_alternative<InitialMovingLinearSolution>( result ) );
  const InitialMovingLinearSolution& solved =
      std::get<InitialMovingLinearSolution>( result );
  EXPECT_TRUE( solved.m_delta.isApprox( expected, 1e-12 ) );
  EXPECT_EQ( solved.m_rank, 2U );
  EXPECT_EQ( solved.m_required_rank, 2U );
  ASSERT_TRUE( solved.m_normalized_condition.has_value() );
  EXPECT_LE( *solved.m_normalized_condition,
             solved.m_normalized_condition_limit );
}

TEST( InitialMovingRootMath, FiniteZeroColumnIsRecoverableRankDeficiency )
{
  const Eigen::Vector2d residual = Eigen::Vector2d::Zero();
  const Eigen::Matrix2d jacobian =
      ( Eigen::Matrix2d() << 1.0, 0.0, 0.0, 0.0 ).finished();
  const Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();

  const auto result = solveInitialMovingLinearSystem(
      residual, jacobian, covariance, 2U );
  ASSERT_TRUE( std::holds_alternative<
               phad::estimator::internal::InitialMovingLinearFailure>(
      result ) );
  const auto& failure = std::get<
      phad::estimator::internal::InitialMovingLinearFailure>( result );
  EXPECT_EQ(
      failure.m_code,
      phad::estimator::internal::InitialMovingLinearFailureCode::
          kRankDeficient );
  EXPECT_EQ( failure.m_rank, 0U );
  EXPECT_EQ( failure.m_required_rank, 2U );
  EXPECT_FALSE( failure.m_normalized_condition.has_value() );
}

TEST( InitialMovingRootMath, NonfiniteWhitenedResidualIsNumericalError )
{
  const Eigen::Vector<double, 1> residual =
      Eigen::Vector<double, 1>::Constant(
          std::numeric_limits<double>::max() );
  const Eigen::Matrix<double, 1, 1> jacobian =
      Eigen::Matrix<double, 1, 1>::Identity();
  const Eigen::Matrix<double, 1, 1> covariance =
      Eigen::Matrix<double, 1, 1>::Constant(
          std::numeric_limits<double>::denorm_min() );

  const auto result = solveInitialMovingLinearSystem(
      residual, jacobian, covariance, 1U );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kNumerical );
}

TEST( InitialMovingRootMath, NonfiniteColumnNormIsNumericalError )
{
  const Eigen::Vector2d             residual = Eigen::Vector2d::Zero();
  const Eigen::Matrix<double, 2, 1> jacobian =
      Eigen::Matrix<double, 2, 1>::Constant( 1e200 );
  const Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();

  const auto result = solveInitialMovingLinearSystem(
      residual, jacobian, covariance, 1U );
  ASSERT_TRUE( std::holds_alternative<InitialMovingMathError>( result ) );
  EXPECT_EQ( std::get<InitialMovingMathError>( result ).m_code,
             phad::estimator::internal::InitialMovingMathErrorCode::kNumerical );
}

TEST( InitialMovingRootMath, NormalizedConditionBoundaryIsInclusive )
{
  EXPECT_TRUE( initialMovingConditionAccepted(
      kInitialMovingNormalizedConditionLimit ) );
  EXPECT_FALSE( initialMovingConditionAccepted( std::nextafter(
      kInitialMovingNormalizedConditionLimit,
      std::numeric_limits<double>::infinity() ) ) );
}

TEST( InitialMovingRootMath, GyroBiasSolveStartsAtZero )
{
  const Eigen::Vector3d expected_bias( 0.01, -0.02, 0.03 );
  const auto            biased_imu = [ & ]( const std::int64_t timestamp_ns ) {
    return ImuMeasurement{
                   .timestamp  = Timestamp{ timestamp_ns },
                   .accel_mps2 = std::array<double, 3>{ 0.0, 0.0, 9.81 },
                   .gyro_radps = std::array<double, 3>{
            expected_bias.x(), expected_bias.y(), expected_bias.z() } };
  };
  constexpr std::int64_t kEndNs     = 50'000'000;
  auto                   normalized = normalizeRawImuInterval(
      RawImuInterval{
                            .m_t_begin = Timestamp{ 0 },
                            .m_t_end   = Timestamp{ kEndNs },
                            .m_samples = { biased_imu( 0 ), biased_imu( kEndNs / 2 ),
                                           biased_imu( kEndNs ) } },
      std::nullopt, Timestamp{ kEndNs } );
  ASSERT_TRUE( std::holds_alternative<NormalizedImuInterval>( normalized ) );

  InitialMovingPose pose_i;
  pose_i.m_frame_index = 1U;
  pose_i.m_timestamp   = Timestamp{ 0 };
  InitialMovingPose pose_j;
  pose_j.m_frame_index = 2U;
  pose_j.m_timestamp   = Timestamp{ kEndNs };
  const InitialMovingMathInput input{
      .m_poses     = { pose_i, pose_j },
      .m_intervals = {
          std::get<NormalizedImuInterval>( std::move( normalized ) ) },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const Eigen::MatrixXd visual_covariance =
      Eigen::MatrixXd::Zero( 6, 6 );

  const auto result =
      solveInitialMovingGyroBiasFromZero( input, visual_covariance );
  ASSERT_TRUE(
      std::holds_alternative<InitialMovingGyroSolution>( result ) );
  const InitialMovingGyroSolution& solved =
      std::get<InitialMovingGyroSolution>( result );
  EXPECT_TRUE( solved.m_bg_radps.isApprox( expected_bias, 1e-8 ) )
      << "actual=" << solved.m_bg_radps.transpose();
  EXPECT_EQ( solved.m_rank, 3U );
  ASSERT_TRUE( solved.m_normalized_condition.has_value() );
}

TEST( InitialMovingRootMath, FinalBiasReintegratesRawPim )
{
  constexpr std::int64_t kEndNs      = 50'000'000;
  const auto             make_sample = []( const std::int64_t           timestamp_ns,
                               const std::array<double, 3>& gyro ) {
    return ImuMeasurement{
                    .timestamp  = Timestamp{ timestamp_ns },
                    .accel_mps2 = std::array<double, 3>{ 0.0, 0.0, 9.81 },
                    .gyro_radps = gyro };
  };
  const RawImuInterval raw{
      .m_t_begin = Timestamp{ 0 },
      .m_t_end   = Timestamp{ kEndNs },
      .m_samples = {
          make_sample( 0, { 2.0, -1.0, 0.5 } ),
          make_sample( kEndNs / 2, { -1.0, 2.0, 1.0 } ),
          make_sample( kEndNs, { 0.5, -2.0, 1.5 } ) } };
  auto normalized_result = normalizeRawImuInterval(
      raw, Timestamp{ 0 }, Timestamp{ kEndNs } );
  ASSERT_TRUE( std::holds_alternative<NormalizedImuInterval>(
      normalized_result ) );
  const NormalizedImuInterval normalized =
      std::get<NormalizedImuInterval>( std::move( normalized_result ) );

  InitialMovingPose pose_i;
  pose_i.m_frame_index = 0U;
  pose_i.m_timestamp   = Timestamp{ 0 };
  InitialMovingPose pose_j;
  pose_j.m_frame_index = 1U;
  pose_j.m_timestamp   = Timestamp{ kEndNs };
  pose_j.m_T_W_B.linear() =
      ( Eigen::AngleAxisd( 0.04, Eigen::Vector3d::UnitX() ) *
        Eigen::AngleAxisd( -0.03, Eigen::Vector3d::UnitY() ) )
          .toRotationMatrix();
  pose_j.m_T_W_B.translation() = Eigen::Vector3d( 0.01, -0.02, 0.03 );
  const InitialMovingMathInput input{
      .m_poses        = { pose_i, pose_j },
      .m_intervals    = { normalized },
      .m_pim          = pimConfig(),
      .m_gravity_mps2 = 9.81 };
  const Eigen::Vector3d        final_bias( 0.5, -0.4, 0.3 );
  const InitialMovingCandidate candidate{
      .m_bg_radps = final_bias,
      .m_g_W      = Eigen::Vector3d( 0.0, 0.0, -9.81 ),
      .m_v_W      = { Eigen::Vector3d( 0.1, -0.2, 0.3 ),
                      Eigen::Vector3d( 0.12, -0.18, 0.28 ) } };
  const auto production_result =
      buildInitialMovingCanonicalSystem( input, candidate );
  ASSERT_TRUE( std::holds_alternative<InitialMovingCanonicalSystem>(
      production_result ) );
  const Eigen::VectorXd production_residual =
      std::get<InitialMovingCanonicalSystem>( production_result ).m_residual;

  auto params =
      std::make_shared<gtsam::PreintegrationParams>( candidate.m_g_W );
  params->setAccelerometerCovariance( input.m_pim.m_acc_cov );
  params->setGyroscopeCovariance( input.m_pim.m_gyr_cov );
  params->setIntegrationCovariance( input.m_pim.m_integration_cov );
  const gtsam::imuBias::ConstantBias final_gtsam_bias(
      Eigen::Vector3d::Zero(), final_bias );
  const gtsam::imuBias::ConstantBias  zero_bias;
  gtsam::PreintegratedImuMeasurements fresh_pim( params, final_gtsam_bias );
  gtsam::PreintegratedImuMeasurements first_order_pim( params, zero_bias );
  for ( const auto& step : normalized.m_steps )
  {
    fresh_pim.integrateMeasurement( step.m_acc_mean_mps2,
                                    step.m_gyr_mean_radps, step.m_dt_s );
    first_order_pim.integrateMeasurement(
        step.m_acc_mean_mps2, step.m_gyr_mean_radps, step.m_dt_s );
  }
  const gtsam::Pose3   T_W_B_i( pose_i.m_T_W_B.matrix() );
  const gtsam::Pose3   T_W_B_j( pose_j.m_T_W_B.matrix() );
  const gtsam::Vector9 fresh_residual = fresh_pim.computeErrorAndJacobians(
      T_W_B_i, candidate.m_v_W[ 0 ], T_W_B_j, candidate.m_v_W[ 1 ],
      final_gtsam_bias );
  const gtsam::Vector9 first_order_residual =
      first_order_pim.computeErrorAndJacobians(
          T_W_B_i, candidate.m_v_W[ 0 ], T_W_B_j, candidate.m_v_W[ 1 ],
          final_gtsam_bias );

  EXPECT_TRUE( production_residual.isApprox( fresh_residual, 1e-12 ) );
  EXPECT_GT( ( fresh_residual - first_order_residual ).norm(), 1e-6 );
}

TEST( InitialMovingRootMath, FixedScaleGravityVelocityClosure )
{
  namespace fixture = phad::test_support::initial_moving;
  InitialMovingMathInput input;
  input.m_pim          = pimConfig();
  input.m_gravity_mps2 = 9.81;
  std::vector<Eigen::Vector3d> expected_velocities;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    input.m_poses.push_back( InitialMovingPose{
        .m_frame_index = static_cast<std::uint64_t>( index ),
        .m_timestamp   = Timestamp{ timestamp_ns },
        .m_T_W_B       = fixture::poseWb( timestamp_ns ) } );
    expected_velocities.push_back( fixture::velocityW( timestamp_ns ) );
    if ( index > 0 )
    {
      const std::int64_t begin_ns =
          timestamp_ns - fixture::kFramePeriodNs;
      auto normalized = normalizeRawImuInterval(
          fixture::rawInterval( begin_ns, timestamp_ns ),
          Timestamp{ begin_ns }, Timestamp{ timestamp_ns } );
      ASSERT_TRUE(
          std::holds_alternative<NormalizedImuInterval>( normalized ) );
      input.m_intervals.push_back(
          std::get<NormalizedImuInterval>( std::move( normalized ) ) );
    }
  }
  const Eigen::MatrixXd visual_covariance =
      Eigen::MatrixXd::Zero( 30, 30 );

  const auto result = solveInitialMovingGravityVelocity(
      input, visual_covariance, fixture::kGyroBias );
  ASSERT_TRUE( std::holds_alternative<
               InitialMovingGravityVelocitySolution>( result ) );
  const InitialMovingGravityVelocitySolution& solved =
      std::get<InitialMovingGravityVelocitySolution>( result );
  const double gravity_angle = std::acos( std::clamp(
      solved.m_g_W.normalized().dot( fixture::kGravityW.normalized() ),
      -1.0, 1.0 ) );
  EXPECT_LT( gravity_angle, 1e-4 );
  ASSERT_EQ( solved.m_v_W.size(), expected_velocities.size() );
  for ( std::size_t index = 0U; index < solved.m_v_W.size(); ++index )
  {
    EXPECT_LT(
        ( solved.m_v_W[ index ] - expected_velocities[ index ] ).norm(),
        1e-3 )
        << "velocity_index=" << index;
  }
  EXPECT_EQ( solved.m_rank, 20U );
  ASSERT_TRUE( solved.m_normalized_condition.has_value() );
}

TEST( InitialMovingRootMath, JointRefinementConvergesWithinEightRounds )
{
  namespace fixture = phad::test_support::initial_moving;
  InitialMovingMathInput input;
  input.m_pim          = pimConfig();
  input.m_gravity_mps2 = 9.81;
  std::vector<Eigen::Vector3d> expected_velocities;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    input.m_poses.push_back( InitialMovingPose{
        .m_frame_index = static_cast<std::uint64_t>( index ),
        .m_timestamp   = Timestamp{ timestamp_ns },
        .m_T_W_B       = fixture::poseWb( timestamp_ns ) } );
    expected_velocities.push_back( fixture::velocityW( timestamp_ns ) );
    if ( index > 0 )
    {
      const std::int64_t begin_ns =
          timestamp_ns - fixture::kFramePeriodNs;
      auto normalized = normalizeRawImuInterval(
          fixture::rawInterval( begin_ns, timestamp_ns ),
          Timestamp{ begin_ns }, Timestamp{ timestamp_ns } );
      ASSERT_TRUE(
          std::holds_alternative<NormalizedImuInterval>( normalized ) );
      input.m_intervals.push_back(
          std::get<NormalizedImuInterval>( std::move( normalized ) ) );
    }
  }
  const Eigen::MatrixXd visual_covariance =
      Eigen::MatrixXd::Zero( 30, 30 );

  const auto result =
      solveInitialMovingCandidate( input, visual_covariance );
  ASSERT_TRUE(
      std::holds_alternative<InitialMovingCandidateSolution>( result ) );
  const InitialMovingCandidateSolution& solved =
      std::get<InitialMovingCandidateSolution>( result );
  EXPECT_GT( solved.m_refinement_rounds, 0U );
  EXPECT_LE( solved.m_refinement_rounds, 8U );
  EXPECT_LT(
      ( solved.m_candidate.m_bg_radps - fixture::kGyroBias ).norm(), 1e-4 );
  const double gravity_angle = std::acos( std::clamp(
      solved.m_candidate.m_g_W.normalized().dot(
          fixture::kGravityW.normalized() ),
      -1.0, 1.0 ) );
  EXPECT_LT( gravity_angle, 1e-4 );
  ASSERT_EQ( solved.m_candidate.m_v_W.size(),
             expected_velocities.size() );
  for ( std::size_t index = 0U;
        index < solved.m_candidate.m_v_W.size(); ++index )
  {
    EXPECT_LT(
        ( solved.m_candidate.m_v_W[ index ] -
          expected_velocities[ index ] )
            .norm(),
        1e-3 )
        << "velocity_index=" << index;
  }
  EXPECT_EQ( solved.m_joint_fit.m_effective_dof, 22 );
  EXPECT_LE( solved.m_joint_fit.m_statistic, solved.m_joint_limit );
}

TEST( InitialMovingRootMath, RefinementBudgetExhaustionIsNoConvergence )
{
  namespace fixture = phad::test_support::initial_moving;
  InitialMovingMathInput input;
  input.m_pim          = pimConfig();
  input.m_gravity_mps2 = 9.81;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    input.m_poses.push_back( InitialMovingPose{
        .m_frame_index = static_cast<std::uint64_t>( index ),
        .m_timestamp   = Timestamp{ timestamp_ns },
        .m_T_W_B       = fixture::poseWb( timestamp_ns ) } );
    if ( index > 0 )
    {
      const std::int64_t begin_ns =
          timestamp_ns - fixture::kFramePeriodNs;
      auto normalized = normalizeRawImuInterval(
          fixture::rawInterval( begin_ns, timestamp_ns ),
          Timestamp{ begin_ns }, Timestamp{ timestamp_ns } );
      ASSERT_TRUE(
          std::holds_alternative<NormalizedImuInterval>( normalized ) );
      input.m_intervals.push_back(
          std::get<NormalizedImuInterval>( std::move( normalized ) ) );
    }
  }
  const Eigen::MatrixXd visual_covariance =
      Eigen::MatrixXd::Zero( 30, 30 );

  const auto result = solveInitialMovingCandidate(
      input, visual_covariance,
      InitialMovingSolverLimits{ .m_max_refinement_corrections = 0U } );
  ASSERT_TRUE( std::holds_alternative<
               phad::estimator::internal::InitialMovingCandidateFailure>(
      result ) );
  const auto& failure = std::get<
      phad::estimator::internal::InitialMovingCandidateFailure>( result );
  EXPECT_EQ(
      failure.m_code,
      phad::estimator::internal::InitialMovingCandidateFailureCode::
          kNoConvergence );
  EXPECT_TRUE( failure.m_gyro.has_value() );
  EXPECT_TRUE( failure.m_gravity_velocity.has_value() );
}

TEST( InitialMovingRootMath, JointChiSquareBoundaryIsInclusive )
{
  constexpr double kLimit = 33.92443847144381;
  EXPECT_TRUE( initialMovingJointFitAccepted( kLimit, kLimit ) );
  EXPECT_FALSE( initialMovingJointFitAccepted(
      std::nextafter( kLimit, std::numeric_limits<double>::infinity() ),
      kLimit ) );
}

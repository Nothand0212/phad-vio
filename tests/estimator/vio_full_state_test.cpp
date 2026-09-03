#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/internal/vio_estimator_test_access.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"
#include "tests/estimator/initial_moving_root_fixture.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::common::Timestamp;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::estimator::VioUpdateResult;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::ImuParameters;
  using phad::sensor::ImuPayload;
  using phad::sensor::MeasurementDiscontinuity;
  using phad::sensor::RawImuInterval;
  using phad::sensor::RigidTransform;

  constexpr std::int64_t kFramePeriodNs = 50'000'000;
  const Eigen::Vector3d  kGyroBiasRadps{ 0.01, -0.02, 0.03 };

  [[nodiscard]] RectifiedStereoCalibration makeCalibration()
  {
    auto transform =
        RigidTransform::create( Eigen::Matrix4d::Identity() ).value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( transform ) )
        .value();
  }

  [[nodiscard]] ImuParameters makeImuParameters()
  {
    return ImuParameters::create( 200.0, 0.2, 0.03, 0.004, 0.005 )
        .value();
  }

  [[nodiscard]] EstimatorOptions makeOptions()
  {
    EstimatorOptions options;
    options.window_size                     = 8;
    options.min_track_observations_for_seed = 1;
    options.min_landmark_observations       = 3;
    options.min_seed_observations           = 10;
    options.min_shared_landmarks            = 3;
    options.enable_pnp_init                 = false;
    options.m_q_int                         = 0.006;
    return options;
  }

  [[nodiscard]] std::vector<StereoObservation> makeObservations()
  {
    std::vector<StereoObservation> observations;
    observations.reserve( 10U );
    for ( std::uint64_t index = 0U; index < 10U; ++index )
    {
      observations.push_back( StereoObservation{
          .id         = index + 1U,
          .left_pixel = Eigen::Vector2d(
              280.0 + 7.0 * static_cast<double>( index ),
              220.0 + 2.0 * static_cast<double>( index % 4U ) ),
          .disparity_px = 12.0 } );
    }
    return observations;
  }

  [[nodiscard]] std::vector<StereoObservation> makeUnmappedObservations()
  {
    std::vector<StereoObservation> observations = makeObservations();
    for ( StereoObservation& observation : observations )
    {
      observation.id += 1'000U;
    }
    return observations;
  }

  [[nodiscard]] std::vector<StereoObservation> makeLeftOnlyObservations(
      std::size_t count = 10U )
  {
    std::vector<StereoObservation> observations = makeObservations();
    observations.resize( count );
    for ( StereoObservation& observation : observations )
    {
      observation.disparity_px = 0.0;
    }
    return observations;
  }

  [[nodiscard]] ImuMeasurement makeImuWithAcc(
      std::int64_t timestamp_ns, const Eigen::Vector3d& acc_mps2 )
  {
    return ImuMeasurement{
        .timestamp  = Timestamp{ timestamp_ns },
        .accel_mps2 = std::array<double, 3>{ acc_mps2.x(), acc_mps2.y(),
                                             acc_mps2.z() },
        .gyro_radps =
            std::array<double, 3>{ kGyroBiasRadps.x(), kGyroBiasRadps.y(),
                                   kGyroBiasRadps.z() } };
  }

  [[nodiscard]] ImuMeasurement makeImu( std::int64_t timestamp_ns,
                                        double       acc_x_mps2 )
  {
    return makeImuWithAcc( timestamp_ns,
                           Eigen::Vector3d{ acc_x_mps2, 0.0, 9.81 } );
  }

  [[nodiscard]] RawImuInterval stationaryInterval( std::int64_t begin_ns,
                                                   std::int64_t end_ns )
  {
    return RawImuInterval{
        .m_t_begin = Timestamp{ begin_ns },
        .m_t_end   = Timestamp{ end_ns },
        .m_samples = { makeImu( begin_ns, 0.0 ),
                       makeImu( begin_ns + ( end_ns - begin_ns ) / 2, 0.0 ),
                       makeImu( end_ns, 0.0 ) } };
  }

  [[nodiscard]] RawImuInterval interpolatedRampInterval()
  {
    return RawImuInterval{
        .m_t_begin = Timestamp{ kFramePeriodNs },
        .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
        .m_samples = { makeImu( 45'000'000, -0.1 ),
                       makeImu( 60'000'000, 0.2 ),
                       makeImu( 90'000'000, 0.8 ),
                       makeImu( 105'000'000, 1.1 ) } };
  }

  [[nodiscard]] VioMeasurement makeMeasurement( std::int64_t timestamp_ns,
                                                ImuPayload   payload )
  {
    return VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = makeObservations(),
        .m_imu          = std::move( payload ) };
  }

  [[nodiscard]] VioMeasurement makeMeasurementWithObservations(
      std::int64_t timestamp_ns, ImuPayload payload,
      std::vector<StereoObservation> observations )
  {
    return VioMeasurement{ .m_timestamp    = Timestamp{ timestamp_ns },
                           .m_observations = std::move( observations ),
                           .m_imu          = std::move( payload ) };
  }

  void expectSameEstimate( const VioUpdateResult& lhs,
                           const VioUpdateResult& rhs )
  {
    ASSERT_EQ( lhs.status, UpdateStatus::kOk ) << lhs.message;
    ASSERT_EQ( rhs.status, UpdateStatus::kOk ) << rhs.message;
    ASSERT_TRUE( lhs.estimate.has_value() );
    ASSERT_TRUE( rhs.estimate.has_value() );
    EXPECT_EQ( lhs.estimate->timestamp, rhs.estimate->timestamp );
    EXPECT_TRUE( lhs.estimate->T_W_B.matrix().isApprox(
        rhs.estimate->T_W_B.matrix(), 1e-12 ) );
    EXPECT_TRUE( lhs.estimate->m_v_W_B.isApprox(
        rhs.estimate->m_v_W_B, 1e-12 ) );
    EXPECT_TRUE( lhs.estimate->m_bias.m_acc_mps2.isApprox(
        rhs.estimate->m_bias.m_acc_mps2, 1e-12 ) );
    EXPECT_TRUE( lhs.estimate->m_bias.m_gyr_radps.isApprox(
        rhs.estimate->m_bias.m_gyr_radps, 1e-12 ) );
    EXPECT_EQ( lhs.estimate->m_segment_id, rhs.estimate->m_segment_id );
  }

  void expectSameCommittedUpdate( const VioUpdateResult& lhs,
                                  const VioUpdateResult& rhs )
  {
    expectSameEstimate( lhs, rhs );
    EXPECT_EQ( lhs.diagnostics.window_size, rhs.diagnostics.window_size );
    EXPECT_EQ( lhs.diagnostics.prior_key, rhs.diagnostics.prior_key );
    EXPECT_EQ( lhs.diagnostics.num_landmarks,
               rhs.diagnostics.num_landmarks );
    EXPECT_EQ( lhs.diagnostics.num_retained_observations,
               rhs.diagnostics.num_retained_observations );
    EXPECT_EQ( lhs.diagnostics.num_seeded_landmarks,
               rhs.diagnostics.num_seeded_landmarks );
    EXPECT_EQ( lhs.diagnostics.num_current_visual_factors,
               rhs.diagnostics.num_current_visual_factors );
    EXPECT_EQ( lhs.diagnostics.num_current_mono_visual_factors,
               rhs.diagnostics.num_current_mono_visual_factors );
    EXPECT_EQ( lhs.diagnostics.unsupported_span_ns,
               rhs.diagnostics.unsupported_span_ns );
    EXPECT_EQ( lhs.diagnostics.outliers_culled,
               rhs.diagnostics.outliers_culled );
    EXPECT_EQ( lhs.diagnostics.outliers_culled_unique,
               rhs.diagnostics.outliers_culled_unique );
    EXPECT_EQ( lhs.diagnostics.outlier_reopt_rounds,
               rhs.diagnostics.outlier_reopt_rounds );
    EXPECT_EQ( lhs.diagnostics.outlier_reopt,
               rhs.diagnostics.outlier_reopt );
    EXPECT_EQ( lhs.diagnostics.outlier_reopt_failed,
               rhs.diagnostics.outlier_reopt_failed );
    const auto& lhs_vio = lhs.diagnostics.m_vio;
    const auto& rhs_vio = rhs.diagnostics.m_vio;
    EXPECT_EQ( lhs_vio.m_nav_states, rhs_vio.m_nav_states );
    EXPECT_EQ( lhs_vio.m_imu_factors, rhs_vio.m_imu_factors );
    EXPECT_EQ( lhs_vio.m_bias_rw_factors, rhs_vio.m_bias_rw_factors );
    EXPECT_EQ( lhs_vio.m_visual_factors, rhs_vio.m_visual_factors );
    EXPECT_EQ( lhs_vio.m_root_prior_sets, rhs_vio.m_root_prior_sets );
    EXPECT_EQ( lhs_vio.m_integration_steps,
               rhs_vio.m_integration_steps );
    EXPECT_EQ( lhs_vio.m_integrated_duration_ns,
               rhs_vio.m_integrated_duration_ns );
    EXPECT_EQ( lhs_vio.m_visual_coast_duration_ns,
               rhs_vio.m_visual_coast_duration_ns );
    EXPECT_EQ( lhs_vio.m_non_keyframe_evictions,
               rhs_vio.m_non_keyframe_evictions );
    EXPECT_EQ( lhs_vio.m_imu_reintegrations,
               rhs_vio.m_imu_reintegrations );
  }

  void expectZeroPacketCounters( const VioUpdateResult& result )
  {
    EXPECT_EQ( result.diagnostics.num_retained_observations, 0U );
    EXPECT_EQ( result.diagnostics.num_seeded_landmarks, 0U );
    EXPECT_EQ( result.diagnostics.num_current_visual_factors, 0U );
    EXPECT_EQ( result.diagnostics.num_current_mono_visual_factors, 0U );
  }

  [[nodiscard]] EstimatorOptions transactionOptions()
  {
    EstimatorOptions options                = makeOptions();
    options.window_size                     = 32;
    options.min_track_observations_for_seed = 2;
    options.min_landmark_observations       = 2;
    options.enable_outlier_cull             = false;
    options.enable_outlier_reopt            = false;
    return options;
  }

  void primeSupportedCoast( VioEstimator& estimator )
  {
    ASSERT_EQ( estimator
                   .update( makeMeasurement(
                       kFramePeriodNs,
                       stationaryInterval( 0, kFramePeriodNs ) ) )
                   .status,
               UpdateStatus::kOk );
    ASSERT_EQ( estimator
                   .update( makeMeasurement(
                       2 * kFramePeriodNs,
                       stationaryInterval( kFramePeriodNs,
                                           2 * kFramePeriodNs ) ) )
                   .status,
               UpdateStatus::kOk );
    const auto coast = estimator.update( makeMeasurementWithObservations(
        3 * kFramePeriodNs,
        stationaryInterval( 2 * kFramePeriodNs, 3 * kFramePeriodNs ),
        {} ) );
    ASSERT_EQ( coast.status, UpdateStatus::kOk ) << coast.message;
    ASSERT_EQ( coast.diagnostics.m_vio.m_visual_coast_duration_ns,
               kFramePeriodNs );
    ASSERT_EQ( coast.diagnostics.unsupported_span_ns, kFramePeriodNs );
  }

  [[nodiscard]] StereoObservation makeUniqueObservation(
      std::uint64_t id )
  {
    StereoObservation observation = makeObservations().front();
    observation.id                = id;
    return observation;
  }

  struct InjectedMovingAttempt
  {
    VioUpdateResult rejected;
    VioUpdateResult recovered;
  };

  [[nodiscard]] InjectedMovingAttempt runInjectedMovingCandidateResult(
      phad::estimator::internal::InitialMovingCandidateResult injected )
  {
    namespace fixture                 = phad::test_support::initial_moving;
    namespace internal                = phad::estimator::internal;
    EstimatorOptions options          = makeOptions();
    options.m_enable_moving_bootstrap = true;
    options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
    options.window_size               = 8;
    VioEstimator estimator( makeCalibration(), makeImuParameters(), options );
    internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
        estimator, std::move( injected ) );

    VioUpdateResult rejected;
    for ( std::int64_t index = 0; index < 6; ++index )
    {
      const std::int64_t timestamp_ns =
          fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
      rejected = estimator.update( VioMeasurement{
          .m_timestamp    = Timestamp{ timestamp_ns },
          .m_observations = fixture::observations( timestamp_ns ),
          .m_imu          = fixture::rawInterval(
              timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
    }

    constexpr std::int64_t kRecoveryIndex = 6;
    const std::int64_t     recovery_timestamp_ns =
        fixture::kRootTimestampNs +
        kRecoveryIndex * fixture::kFramePeriodNs;
    VioUpdateResult recovered = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ recovery_timestamp_ns },
        .m_observations = fixture::observations( recovery_timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            recovery_timestamp_ns - fixture::kFramePeriodNs,
            recovery_timestamp_ns ) } );
    return InjectedMovingAttempt{ .rejected  = std::move( rejected ),
                                  .recovered = std::move( recovered ) };
  }

}  // namespace

TEST( VioFullState, StaticRootAndInterpolatedSuccessorCommitXVB )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );

  const auto root = estimator.update( makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.estimate.has_value() );
  EXPECT_EQ( root.estimate->timestamp, Timestamp{ kFramePeriodNs } );
  EXPECT_TRUE( root.estimate->T_W_B.matrix().isApprox(
      Eigen::Isometry3d::Identity().matrix(), 1e-12 ) );
  EXPECT_TRUE( root.estimate->m_v_W_B.isZero( 1e-12 ) );
  EXPECT_TRUE( root.estimate->m_bias.m_acc_mps2.isZero( 1e-12 ) );
  EXPECT_TRUE(
      root.estimate->m_bias.m_gyr_radps.isApprox( kGyroBiasRadps, 1e-12 ) );
  EXPECT_EQ( root.estimate->m_segment_id, 0U );

  const auto& initialization = root.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kStatic );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCommit );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kNone );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kCommitted );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kValidated );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kRootCommitted );

  const auto& root_vio = root.diagnostics.m_vio;
  EXPECT_EQ( root_vio.m_nav_states, 1U );
  EXPECT_EQ( root_vio.m_root_prior_sets, 1U );
  EXPECT_EQ( root_vio.m_imu_factors, 0U );
  EXPECT_EQ( root_vio.m_bias_rw_factors, 0U );
  EXPECT_EQ( root_vio.m_integration_steps, 0U );
  EXPECT_EQ( root_vio.m_integrated_duration_ns, 0 );

  const auto successor = estimator.update( makeMeasurement(
      2 * kFramePeriodNs, interpolatedRampInterval() ) );
  ASSERT_EQ( successor.status, UpdateStatus::kOk ) << successor.message;
  ASSERT_TRUE( successor.estimate.has_value() );
  EXPECT_EQ( successor.estimate->m_segment_id, 0U );
  EXPECT_NEAR( successor.estimate->m_v_W_B.x(), 0.025, 1e-9 );
  EXPECT_NEAR( successor.estimate->m_v_W_B.y(), 0.0, 1e-9 );
  EXPECT_NEAR( successor.estimate->m_v_W_B.z(), 0.0, 1e-9 );
  EXPECT_NEAR( successor.estimate->T_W_B.translation().x(), 0.000465,
               1e-9 );
  EXPECT_NEAR( successor.estimate->T_W_B.translation().y(), 0.0, 1e-9 );
  EXPECT_NEAR( successor.estimate->T_W_B.translation().z(), 0.0, 1e-9 );
  EXPECT_TRUE( successor.estimate->T_W_B.rotation().isApprox(
      Eigen::Matrix3d::Identity(), 1e-9 ) );

  const auto& vio = successor.diagnostics.m_vio;
  EXPECT_EQ( vio.m_nav_states, 2U );
  EXPECT_EQ( vio.m_root_prior_sets, 1U );
  EXPECT_EQ( vio.m_imu_factors, 1U );
  EXPECT_EQ( vio.m_bias_rw_factors, 1U );
  EXPECT_EQ( vio.m_integration_steps, 3U );
  EXPECT_EQ( vio.m_integrated_duration_ns, kFramePeriodNs );
  EXPECT_TRUE( vio.m_acc_cov_diag.isApprox(
      Eigen::Vector3d::Constant( 0.2 * 0.2 ), 1e-15 ) );
  EXPECT_TRUE( vio.m_gyr_cov_diag.isApprox(
      Eigen::Vector3d::Constant( 0.03 * 0.03 ), 1e-15 ) );
  EXPECT_TRUE( vio.m_integration_cov_diag.isApprox(
      Eigen::Vector3d::Constant( 0.006 ), 1e-15 ) );

  const double sqrt_dt = std::sqrt( 0.05 );
  for ( Eigen::Index axis = 0; axis < 3; ++axis )
  {
    EXPECT_NEAR( vio.m_bias_rw_sigmas( axis ), 0.004 * sqrt_dt, 1e-15 );
    EXPECT_NEAR( vio.m_bias_rw_sigmas( axis + 3 ), 0.005 * sqrt_dt,
                 1e-15 );
  }
}

TEST( VioInitialization, MovingEnabledKeepsStaticReadyPrecedence )
{
  EstimatorOptions static_options          = makeOptions();
  EstimatorOptions moving_options          = static_options;
  moving_options.m_enable_moving_bootstrap = true;

  VioEstimator         static_only( makeCalibration(), makeImuParameters(),
                                    static_options );
  VioEstimator         moving_enabled( makeCalibration(), makeImuParameters(),
                                       moving_options );
  const VioMeasurement measurement = makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) );

  const VioUpdateResult control   = static_only.update( measurement );
  const VioUpdateResult candidate = moving_enabled.update( measurement );
  expectSameCommittedUpdate( candidate, control );
  EXPECT_EQ( candidate.diagnostics.m_initialization.m_path,
             phad::estimator::InitializationPath::kStatic );
  EXPECT_EQ( candidate.diagnostics.m_initialization.m_outcome,
             phad::estimator::InitializationOutcome::kCommitted );
  EXPECT_FALSE(
      candidate.diagnostics.m_initialization.m_attempt_id.has_value() );
}

TEST( VioInitialization,
      MovingEnabledRetainsStaticReadySparseEvidenceForLaterAttempt )
{
  namespace internal = phad::estimator::internal;

  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.min_pnp_inliers           = 4;
  options.window_size               = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
      estimator,
      internal::InitialMovingCandidateFailure{
          .m_code = internal::InitialMovingCandidateFailureCode::
              kRankDeficient,
          .m_linear_failure = internal::InitialMovingLinearFailure{
              .m_code =
                  internal::InitialMovingLinearFailureCode::kRankDeficient,
              .m_rank                 = 2U,
              .m_required_rank        = 3U,
              .m_normalized_condition = std::nullopt },
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = std::nullopt,
          .m_gravity_velocity = std::nullopt } );

  std::vector<StereoObservation> sparse = makeObservations();
  sparse.resize( 5U );
  for ( std::int64_t frame_index = 1; frame_index <= 2; ++frame_index )
  {
    const std::int64_t begin_ns = ( frame_index - 1 ) * kFramePeriodNs;
    const std::int64_t end_ns   = frame_index * kFramePeriodNs;
    const auto         pending  = estimator.update( makeMeasurementWithObservations(
        end_ns, stationaryInterval( begin_ns, end_ns ), sparse ) );

    ASSERT_EQ( pending.status, UpdateStatus::kInitializing )
        << "frame_index=" << frame_index << " message=" << pending.message;
    EXPECT_FALSE( pending.estimate.has_value() );
    const auto& initialization = pending.diagnostics.m_initialization;
    EXPECT_EQ( initialization.m_path,
               phad::estimator::InitializationPath::kStatic );
    EXPECT_EQ( initialization.m_phase,
               phad::estimator::InitializationPhase::kCollectingEvidence );
    EXPECT_EQ(
        initialization.m_reason,
        phad::estimator::InitializationReason::kStaticEvidenceInsufficient );
    EXPECT_EQ( initialization.m_outcome,
               phad::estimator::InitializationOutcome::kPending );
    EXPECT_FALSE( initialization.m_attempt_id.has_value() );
    EXPECT_EQ( initialization.m_current_graph,
               phad::estimator::InitializationGraphResult::kNotEvaluated );
    EXPECT_EQ(
        initialization.m_transaction,
        phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
    EXPECT_EQ( pending.diagnostics.num_disparity, 5U );
    EXPECT_EQ( estimator.observationTimestamps( 1U ).size(),
               static_cast<std::size_t>( frame_index ) );
  }

  const auto moving_interval = []( const std::int64_t frame_index ) {
    const std::int64_t begin_ns   = ( frame_index - 1 ) * kFramePeriodNs;
    const std::int64_t end_ns     = frame_index * kFramePeriodNs;
    const double       begin_acc  = frame_index % 2 == 0 ? 2.0 : 0.0;
    const double       end_acc    = frame_index % 2 == 0 ? 0.0 : 2.0;
    const double       middle_acc = frame_index % 2 == 0 ? -1.0 : 1.0;
    return RawImuInterval{
        .m_t_begin = Timestamp{ begin_ns },
        .m_t_end   = Timestamp{ end_ns },
        .m_samples = { makeImu( begin_ns, begin_acc ),
                       makeImu( begin_ns + kFramePeriodNs / 2, middle_acc ),
                       makeImu( end_ns, end_acc ) } };
  };

  VioUpdateResult attempted;
  for ( std::int64_t frame_index = 3; frame_index <= 6; ++frame_index )
  {
    attempted = estimator.update( makeMeasurement(
        frame_index * kFramePeriodNs, moving_interval( frame_index ) ) );
    ASSERT_EQ( attempted.status, UpdateStatus::kInitializing )
        << "frame_index=" << frame_index
        << " message=" << attempted.message;
    EXPECT_EQ( attempted.diagnostics.m_initialization.m_path,
               phad::estimator::InitializationPath::kMoving );
    if ( frame_index < 6 )
    {
      EXPECT_EQ(
          attempted.diagnostics.m_initialization.m_reason,
          phad::estimator::InitializationReason::kPoseCountInsufficient );
      EXPECT_EQ( attempted.diagnostics.m_initialization.m_outcome,
                 phad::estimator::InitializationOutcome::kPending );
      EXPECT_FALSE( attempted.diagnostics.m_initialization.m_attempt_id
                        .has_value() );
      EXPECT_EQ(
          attempted.diagnostics.m_initialization.m_transaction,
          phad::estimator::InitializationTransactionResult::
              kEvidenceCommitted );
    }
  }

  const auto& initialization = attempted.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kGyroAlignment );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kRankDeficient );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kRecoverableRejected );
  EXPECT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, 250'000'000 );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  EXPECT_EQ( attempted.diagnostics.num_disparity, 10U );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 6U );
}

TEST( VioFullState, TiltedStaticBootstrapMapsMeanAccelerationToWorldUp )
{
  VioEstimator          estimator( makeCalibration(), makeImuParameters(),
                                   makeOptions() );
  const double          component = 9.81 / std::sqrt( 2.0 );
  const Eigen::Vector3d acc_mean{ component, 0.0, component };
  RawImuInterval        interval{
             .m_t_begin = Timestamp{ 0 },
             .m_t_end   = Timestamp{ kFramePeriodNs },
             .m_samples = { makeImuWithAcc( 0, acc_mean ),
                            makeImuWithAcc( kFramePeriodNs / 2, acc_mean ),
                            makeImuWithAcc( kFramePeriodNs, acc_mean ) } };

  const auto result = estimator.update(
      makeMeasurement( kFramePeriodNs, std::move( interval ) ) );
  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.estimate.has_value() );
  const Eigen::Vector3d mapped =
      result.estimate->T_W_B.rotation() * acc_mean.normalized();
  EXPECT_TRUE( mapped.isApprox( Eigen::Vector3d::UnitZ(), 1e-12 ) );
  EXPECT_NEAR( Eigen::AngleAxisd( result.estimate->T_W_B.rotation() ).angle(),
               std::numbers::pi / 4.0, 1e-12 );
}

TEST( VioFullState, StaticTimeoutRollsBackCurrentPacket )
{
  EstimatorOptions options       = makeOptions();
  options.m_bootstrap_timeout_ns = 2 * kFramePeriodNs;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  const VioMeasurement first = makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } );
  for ( VioEstimator* estimator : { &subject, &control } )
  {
    const auto collecting = estimator->update( first );
    ASSERT_EQ( collecting.status, UpdateStatus::kInitializing );
    EXPECT_EQ( collecting.diagnostics.m_initialization.m_path,
               phad::estimator::InitializationPath::kStatic );
    EXPECT_EQ( collecting.diagnostics.m_initialization.m_outcome,
               phad::estimator::InitializationOutcome::kPending );
  }

  const VioMeasurement timeout_packet = makeMeasurement(
      2 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ kFramePeriodNs },
          .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
          .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                         makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 2 * kFramePeriodNs, 0.0 ) } } );
  const auto timed_out = subject.update( timeout_packet );
  ASSERT_EQ( timed_out.status, UpdateStatus::kFailed );
  EXPECT_FALSE( timed_out.estimate.has_value() );
  EXPECT_EQ( timed_out.diagnostics.num_disparity, 10U );
  const auto& initialization = timed_out.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kStatic );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCollectingEvidence );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kStaticTimeout );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kFatalError );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kRolledBack );

  const auto after_timeout = subject.update( timeout_packet );
  const auto direct        = control.update( timeout_packet );
  EXPECT_EQ( after_timeout.status, direct.status );
  EXPECT_EQ( after_timeout.message, direct.message );
  EXPECT_EQ( after_timeout.diagnostics.window_size,
             direct.diagnostics.window_size );
  EXPECT_EQ( after_timeout.diagnostics.prior_key,
             direct.diagnostics.prior_key );
  EXPECT_EQ( after_timeout.diagnostics.m_initialization.m_reason,
             direct.diagnostics.m_initialization.m_reason );
  EXPECT_EQ( after_timeout.diagnostics.m_initialization.m_transaction,
             direct.diagnostics.m_initialization.m_transaction );
}

TEST( VioFullState, StaticReadyWithoutVisualSeedRemainsTypedPending )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );
  const auto   result = estimator.update( makeMeasurementWithObservations(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ), {} ) );

  ASSERT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 0U );
  const auto& initialization = result.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kStatic );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCollectingEvidence );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kStaticEvidenceInsufficient );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kPending );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioFullState, StaticReadyWithSparseVisualSeedRemainsTypedPending )
{
  VioEstimator                   estimator( makeCalibration(), makeImuParameters(),
                                            makeOptions() );
  std::vector<StereoObservation> sparse = makeObservations();
  sparse.resize( 5U );
  const auto result = estimator.update( makeMeasurementWithObservations(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ),
      std::move( sparse ) ) );

  ASSERT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 5U );
  const auto& initialization = result.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kStatic );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kStaticEvidenceInsufficient );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kPending );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioFullState, MovingInitializationDoesNotPublishShortcutRoot )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto result = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_NE( result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 10U );
  const auto& initialization = result.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCollectingEvidence );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kPoseCountInsufficient );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kPending );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 1U );
  EXPECT_EQ( initialization.m_connected_poses.m_required, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, 0 );
  EXPECT_EQ( initialization.m_duration.m_required_ns, 250'000'000 );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 10U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_required, 10U );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioInitialization, MovingTimeoutEndsStaticCandidate )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = kFramePeriodNs;
  options.window_size               = 3;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto sample = []( const std::int64_t timestamp_ns,
                          const double       extra_gyr_x ) {
    return ImuMeasurement{
        .timestamp  = Timestamp{ timestamp_ns },
        .accel_mps2 = std::array<double, 3>{ 0.0, 0.0, 9.81 },
        .gyro_radps = std::array<double, 3>{
            kGyroBiasRadps.x() + extra_gyr_x,
            kGyroBiasRadps.y(),
            kGyroBiasRadps.z() } };
  };

  for ( std::int64_t frame_index = 1; frame_index <= 10; ++frame_index )
  {
    const std::int64_t begin_ns     = ( frame_index - 1 ) * kFramePeriodNs;
    const std::int64_t end_ns       = frame_index * kFramePeriodNs;
    const double       middle_extra = frame_index == 1 ? 0.02 : 0.0;
    const auto         update       = estimator.update( makeMeasurementWithObservations(
        end_ns,
        RawImuInterval{
                          .m_t_begin = Timestamp{ begin_ns },
                          .m_t_end   = Timestamp{ end_ns },
                          .m_samples = { sample( begin_ns, 0.0 ),
                                         sample( begin_ns + kFramePeriodNs / 2,
                                                 middle_extra ),
                                         sample( end_ns, 0.0 ) } },
        {} ) );
    ASSERT_EQ( update.status, UpdateStatus::kInitializing )
        << "frame_index=" << frame_index << " message=" << update.message;
    EXPECT_EQ( update.diagnostics.m_initialization.m_path,
               phad::estimator::InitializationPath::kMoving )
        << "frame_index=" << frame_index;
  }
}

TEST( VioInitialization, MovingEvidenceCountsKeyframeAndNonKeyframe )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.window_size               = 6;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto first = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;
  EXPECT_FALSE( first.estimate.has_value() );
  EXPECT_EQ( first.diagnostics.m_initialization.m_connected_poses.m_actual,
             1U );
  EXPECT_FALSE(
      first.diagnostics.m_initialization.m_attempt_id.has_value() );

  const auto second = estimator.update(
      makeMeasurement(
          2 * kFramePeriodNs,
          RawImuInterval{
              .m_t_begin = Timestamp{ kFramePeriodNs },
              .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
              .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                             makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                             makeImu( 2 * kFramePeriodNs, 0.0 ) } } ),
      false );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  EXPECT_FALSE( second.estimate.has_value() );
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kPoseCountInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 2U );
  EXPECT_EQ( initialization.m_connected_poses.m_required, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, kFramePeriodNs );
  EXPECT_EQ( initialization.m_duration.m_required_ns, 250'000'000 );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 10U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_required, 10U );
  ASSERT_TRUE( initialization.m_visual_support.has_value() );
  EXPECT_EQ( initialization.m_visual_support->m_actual, 10U );
  EXPECT_EQ( initialization.m_visual_support->m_required, 10U );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 2U );
}

TEST( VioInitialization, MovingVisualSeedDoesNotCountWithoutStereoFactors )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.window_size               = 6;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto first = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;
  ASSERT_EQ( first.diagnostics.m_initialization.m_connected_poses.m_actual,
             1U );

  const auto second = estimator.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ kFramePeriodNs },
          .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
          .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                         makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 2 * kFramePeriodNs, 0.0 ) } },
      makeLeftOnlyObservations() ) );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  EXPECT_FALSE( second.estimate.has_value() );
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ(
      initialization.m_phase,
      phad::estimator::InitializationPhase::kVisualSolve );
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kVisualConnectivityInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 0U );
  EXPECT_FALSE( initialization.m_visual_support.has_value() );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 2U );
}

TEST( VioInitialization, MovingEvidenceDoesNotPoolDisconnectedComponents )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.window_size               = 6;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto first = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;
  ASSERT_EQ( first.diagnostics.m_initialization.m_connected_poses.m_actual,
             1U );

  const auto second = estimator.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ kFramePeriodNs },
          .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
          .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                         makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 2 * kFramePeriodNs, 0.0 ) } },
      makeUnmappedObservations() ) );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  EXPECT_FALSE( second.estimate.has_value() );
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 1U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 10U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, 0 );
  EXPECT_FALSE( initialization.m_visual_support.has_value() );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 1U );
  EXPECT_EQ( estimator.observationTimestamps( 1'001U ).size(), 1U );

  const auto reconnected = estimator.update(
      makeMeasurement(
          3 * kFramePeriodNs,
          RawImuInterval{
              .m_t_begin = Timestamp{ 2 * kFramePeriodNs },
              .m_t_end   = Timestamp{ 3 * kFramePeriodNs },
              .m_samples = { makeImu( 2 * kFramePeriodNs, 0.0 ),
                             makeImu( 5 * kFramePeriodNs / 2, 1.0 ),
                             makeImu( 3 * kFramePeriodNs, 2.0 ) } } ),
      false );
  ASSERT_EQ( reconnected.status, UpdateStatus::kInitializing )
      << reconnected.message;
  const auto& reconnected_initialization =
      reconnected.diagnostics.m_initialization;
  EXPECT_EQ( reconnected_initialization.m_connected_poses.m_actual, 2U );
  EXPECT_EQ( reconnected_initialization.m_duration.m_actual_ns,
             2 * kFramePeriodNs );
  EXPECT_EQ( reconnected_initialization.m_connected_landmarks.m_actual,
             10U );
  ASSERT_TRUE( reconnected_initialization.m_visual_support.has_value() );
  EXPECT_EQ( reconnected_initialization.m_visual_support->m_actual, 10U );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 2U );
  EXPECT_EQ( estimator.observationTimestamps( 1'001U ).size(), 1U );
}

TEST( VioInitialization, MovingEvidenceEvictionSplicesImuIntervals )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 20 * kFramePeriodNs;
  options.window_size               = 3;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto moving_interval = []( const std::int64_t frame_index ) {
    const std::int64_t begin_ns   = ( frame_index - 1 ) * kFramePeriodNs;
    const std::int64_t end_ns     = frame_index * kFramePeriodNs;
    const double       begin_acc  = frame_index % 2 == 0 ? 2.0 : 0.0;
    const double       end_acc    = frame_index % 2 == 0 ? 0.0 : 2.0;
    const double       middle_acc = frame_index % 2 == 0 ? -1.0 : 1.0;
    return RawImuInterval{
        .m_t_begin = Timestamp{ begin_ns },
        .m_t_end   = Timestamp{ end_ns },
        .m_samples = { makeImu( begin_ns, begin_acc ),
                       makeImu( begin_ns + kFramePeriodNs / 2, middle_acc ),
                       makeImu( end_ns, end_acc ) } };
  };

  VioUpdateResult           fourth;
  const std::array<bool, 4> keyframes{ true, false, true, true };
  for ( std::int64_t frame_index = 1; frame_index <= 4; ++frame_index )
  {
    VioUpdateResult update = estimator.update(
        makeMeasurement( frame_index * kFramePeriodNs,
                         moving_interval( frame_index ) ),
        keyframes[ static_cast<std::size_t>( frame_index - 1 ) ] );
    ASSERT_EQ( update.status, UpdateStatus::kInitializing )
        << update.message;
    fourth = std::move( update );
  }

  EXPECT_EQ( fourth.diagnostics.m_initialization.m_connected_poses.m_actual,
             3U );
  EXPECT_EQ( fourth.diagnostics.m_initialization.m_duration.m_actual_ns,
             3 * kFramePeriodNs );
  EXPECT_EQ( fourth.diagnostics.m_vio.m_non_keyframe_evictions, 1U );
  EXPECT_EQ( fourth.diagnostics.m_vio.m_imu_reintegrations, 1U );
  EXPECT_EQ(
      phad::estimator::internal::VioEstimatorTestAccess::
          lastInitialVisualSolveWindowSize( estimator ),
      3U );

  const auto fifth = estimator.update(
      makeMeasurement( 5 * kFramePeriodNs, moving_interval( 5 ) ), true );
  ASSERT_EQ( fifth.status, UpdateStatus::kInitializing ) << fifth.message;
  EXPECT_EQ( fifth.diagnostics.m_initialization.m_connected_poses.m_actual,
             3U );
  EXPECT_EQ( fifth.diagnostics.m_initialization.m_duration.m_actual_ns,
             2 * kFramePeriodNs );
  EXPECT_EQ( fifth.diagnostics.m_vio.m_non_keyframe_evictions, 1U );
  EXPECT_EQ( fifth.diagnostics.m_vio.m_imu_reintegrations, 1U );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 5U );
}

TEST( VioInitialization,
      MovingEvictionPrunesProvisionalHangingLandmarkWithoutNavigation )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.window_size               = 2;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto interval = []( const std::int64_t begin_ns,
                            const std::int64_t end_ns ) {
    return RawImuInterval{
        .m_t_begin = Timestamp{ begin_ns },
        .m_t_end   = Timestamp{ end_ns },
        .m_samples = { makeImu( begin_ns, 0.0 ),
                       makeImu( begin_ns + ( end_ns - begin_ns ) / 2, 1.0 ),
                       makeImu( end_ns, 0.0 ) } };
  };
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                     kFramePeriodNs, interval( 0, kFramePeriodNs ) ) )
                 .status,
             UpdateStatus::kInitializing );
  ASSERT_EQ(
      estimator
          .update( makeMeasurementWithObservations(
              2 * kFramePeriodNs,
              interval( kFramePeriodNs, 2 * kFramePeriodNs ),
              makeUnmappedObservations() ) )
          .status,
      UpdateStatus::kInitializing );

  const auto pruned = estimator.update( makeMeasurementWithObservations(
      3 * kFramePeriodNs,
      interval( 2 * kFramePeriodNs, 3 * kFramePeriodNs ),
      makeLeftOnlyObservations() ) );
  ASSERT_EQ( pruned.status, UpdateStatus::kInitializing ) << pruned.message;
  EXPECT_FALSE( pruned.estimate.has_value() );
  EXPECT_EQ( pruned.diagnostics.window_size, 2U );
  EXPECT_EQ( pruned.diagnostics.m_initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ(
      pruned.diagnostics.m_initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioInitialization, MovingEvidenceReportsVisualPopulationInsufficient )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  VioEstimator                   estimator( makeCalibration(), makeImuParameters(), options );
  std::vector<StereoObservation> sparse = makeObservations();
  sparse.resize( 9U );

  const auto result = estimator.update( makeMeasurementWithObservations(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } },
      std::move( sparse ) ) );
  ASSERT_EQ( result.status, UpdateStatus::kInitializing ) << result.message;
  EXPECT_FALSE( result.estimate.has_value() );
  const auto& initialization = result.diagnostics.m_initialization;
  EXPECT_EQ(
      initialization.m_phase,
      phad::estimator::InitializationPhase::kVisualSolve );
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kVisualPopulationInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 0U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 9U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_required, 10U );
  EXPECT_FALSE( initialization.m_visual_support.has_value() );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioInitialization, MovingEvidenceReportsVisualSupportInsufficient )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto first = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;

  std::vector<StereoObservation> weak = makeObservations();
  weak.resize( 9U );
  const auto second = estimator.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ kFramePeriodNs },
          .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
          .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                         makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 2 * kFramePeriodNs, 0.0 ) } },
      std::move( weak ) ) );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  EXPECT_FALSE( second.estimate.has_value() );
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kVisualSupportInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 0U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 10U );
  ASSERT_TRUE( initialization.m_visual_support.has_value() );
  EXPECT_EQ( initialization.m_visual_support->m_actual, 9U );
  EXPECT_EQ( initialization.m_visual_support->m_required, 10U );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
}

TEST( VioInitialization, MovingEvidenceReportsSecondSolveVisualFit )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto result = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( result.status, UpdateStatus::kInitializing ) << result.message;
  const auto& initialization = result.diagnostics.m_initialization;
  ASSERT_TRUE( initialization.m_visual_fit.has_value() );
  EXPECT_NEAR( initialization.m_visual_fit->m_statistic, 0.0, 1e-20 );
  EXPECT_DOUBLE_EQ( initialization.m_visual_fit->m_limit,
                    11.344866730144373 );
  EXPECT_EQ( initialization.m_visual_fit->m_effective_dof, 3 );
}

TEST( VioInitialization, MovingPopulationIsComponentWideNotGaugeSupport )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  std::vector<StereoObservation> gauge_observations = makeObservations();
  gauge_observations.resize( 1U );
  const auto first = estimator.update( makeMeasurementWithObservations(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } },
      std::move( gauge_observations ) ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;

  const auto second = estimator.update( makeMeasurement(
      2 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ kFramePeriodNs },
          .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
          .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                         makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 2 * kFramePeriodNs, 0.0 ) } } ) );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kPoseCountInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 2U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 10U );
  ASSERT_TRUE( initialization.m_visual_support.has_value() );
  EXPECT_EQ( initialization.m_visual_support->m_actual, 10U );
  EXPECT_EQ( initialization.m_visual_support->m_required, 10U );
}

TEST( VioInitialization, MovingSixPosesBelowDurationReportsDurationGate )
{
  constexpr std::int64_t kShortPeriodNs = 40'000'000;
  EstimatorOptions       options        = makeOptions();
  options.m_enable_moving_bootstrap     = true;
  options.m_bootstrap_timeout_ns        = 20 * kShortPeriodNs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  VioUpdateResult sixth;
  for ( std::int64_t frame_index = 1; frame_index <= 6; ++frame_index )
  {
    const std::int64_t begin_ns   = ( frame_index - 1 ) * kShortPeriodNs;
    const std::int64_t end_ns     = frame_index * kShortPeriodNs;
    const double       begin_acc  = frame_index % 2 == 0 ? 2.0 : 0.0;
    const double       end_acc    = frame_index % 2 == 0 ? 0.0 : 2.0;
    const double       middle_acc = frame_index % 2 == 0 ? -1.0 : 1.0;
    VioUpdateResult    update     = estimator.update( makeMeasurement(
        end_ns,
        RawImuInterval{
                   .m_t_begin = Timestamp{ begin_ns },
                   .m_t_end   = Timestamp{ end_ns },
                   .m_samples = { makeImu( begin_ns, begin_acc ),
                                  makeImu( begin_ns + kShortPeriodNs / 2,
                                           middle_acc ),
                                  makeImu( end_ns, end_acc ) } } ) );
    ASSERT_EQ( update.status, UpdateStatus::kInitializing )
        << update.message;
    sixth = std::move( update );
  }

  const auto& initialization = sixth.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns,
             5 * kShortPeriodNs );
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kDurationInsufficient );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
}

TEST( VioInitialization, MovingSecondSolveRejectsFixedMaskWithoutRemasking )
{
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * kFramePeriodNs;
  options.window_size               = 2;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto first = estimator.update( makeMeasurement(
      kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ kFramePeriodNs },
          .m_samples = { makeImu( 0, 0.0 ),
                         makeImu( kFramePeriodNs / 2, 1.0 ),
                         makeImu( kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing ) << first.message;

  std::vector<StereoObservation> corrupted = makeObservations();
  for ( std::size_t index = 0U; index < corrupted.size(); ++index )
  {
    corrupted[ index ].left_pixel.x() += index % 2U == 0U ? 500.0 : -500.0;
    corrupted[ index ].left_pixel.y() += index % 3U == 0U ? 350.0 : -350.0;
  }
  const auto second = estimator.update(
      makeMeasurementWithObservations(
          2 * kFramePeriodNs,
          RawImuInterval{
              .m_t_begin = Timestamp{ kFramePeriodNs },
              .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
              .m_samples = { makeImu( kFramePeriodNs, 2.0 ),
                             makeImu( 3 * kFramePeriodNs / 2, -1.0 ),
                             makeImu( 2 * kFramePeriodNs, 0.0 ) } },
          std::move( corrupted ) ),
      false );
  ASSERT_EQ( second.status, UpdateStatus::kInitializing ) << second.message;
  EXPECT_FALSE( second.estimate.has_value() );
  const auto& initialization = second.diagnostics.m_initialization;
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kVisualFitRejected );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 0U );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 5U );
  EXPECT_FALSE( initialization.m_visual_support.has_value() );
  ASSERT_TRUE( initialization.m_visual_fit.has_value() );
  EXPECT_GT( initialization.m_visual_fit->m_statistic,
             initialization.m_visual_fit->m_limit );
  EXPECT_DOUBLE_EQ( initialization.m_visual_fit->m_limit,
                    11.344866730144373 );
  EXPECT_EQ( estimator.observationTimestamps( 1U ).size(), 2U );

  const auto after_rejection = estimator.update( makeMeasurement(
      3 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 2 * kFramePeriodNs },
          .m_t_end   = Timestamp{ 3 * kFramePeriodNs },
          .m_samples = { makeImu( 2 * kFramePeriodNs, 0.0 ),
                         makeImu( 5 * kFramePeriodNs / 2, 1.0 ),
                         makeImu( 3 * kFramePeriodNs, 2.0 ) } } ) );
  ASSERT_EQ( after_rejection.status, UpdateStatus::kInitializing )
      << after_rejection.message;
  EXPECT_EQ(
      after_rejection.diagnostics.m_vio.m_non_keyframe_evictions, 1U );

  const auto recovered = estimator.update( makeMeasurement(
      4 * kFramePeriodNs,
      RawImuInterval{
          .m_t_begin = Timestamp{ 3 * kFramePeriodNs },
          .m_t_end   = Timestamp{ 4 * kFramePeriodNs },
          .m_samples = { makeImu( 3 * kFramePeriodNs, 2.0 ),
                         makeImu( 7 * kFramePeriodNs / 2, -1.0 ),
                         makeImu( 4 * kFramePeriodNs, 0.0 ) } } ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kInitializing )
      << recovered.message;
  EXPECT_EQ(
      recovered.diagnostics.m_initialization.m_reason,
      phad::estimator::InitializationReason::kPoseCountInsufficient );
  EXPECT_EQ(
      recovered.diagnostics.m_initialization.m_connected_poses.m_actual,
      2U );
  EXPECT_EQ(
      recovered.diagnostics.m_initialization.m_connected_landmarks.m_actual,
      10U );
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_visual_support.has_value() );
  EXPECT_EQ(
      recovered.diagnostics.m_initialization.m_visual_support->m_actual,
      10U );
}

TEST( VioInitialization, InitialMovingRootCommitsExactLatestState )
{
  namespace fixture                 = phad::test_support::initial_moving;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 7;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const auto update = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
    ASSERT_EQ( update.status, UpdateStatus::kInitializing )
        << "index=" << index << " message=" << update.message;
    EXPECT_FALSE( update.estimate.has_value() );
    EXPECT_FALSE(
        update.diagnostics.m_initialization.m_attempt_id.has_value() );
    EXPECT_EQ( update.diagnostics.m_initialization.m_path,
               phad::estimator::InitializationPath::kMoving );
    if ( index == 4 )
    {
      EXPECT_EQ(
          update.diagnostics.m_initialization.m_reason,
          phad::estimator::InitializationReason::kPoseCountInsufficient );
      EXPECT_EQ(
          update.diagnostics.m_initialization.m_connected_poses.m_actual,
          5U );
      EXPECT_EQ(
          update.diagnostics.m_initialization.m_duration.m_actual_ns,
          200'000'000 );
    }
  }

  constexpr std::int64_t kLatestIndex = 5;
  const std::int64_t     latest_timestamp_ns =
      fixture::kRootTimestampNs +
      kLatestIndex * fixture::kFramePeriodNs;
  const auto root = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ latest_timestamp_ns },
      .m_observations = fixture::observations( latest_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          latest_timestamp_ns - fixture::kFramePeriodNs,
          latest_timestamp_ns ) } );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.estimate.has_value() );
  EXPECT_EQ( root.diagnostics.num_disparity, 20U );
  const auto& initialization = root.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kValidated );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kRootCommitted );
  EXPECT_EQ( root.diagnostics.window_size, 6U );
  EXPECT_EQ( root.diagnostics.num_landmarks, 20U );
  EXPECT_EQ( root.diagnostics.num_current_visual_factors, 20U );
  const auto& root_graph = root.diagnostics.m_vio;
  EXPECT_EQ( root_graph.m_nav_states, 6U );
  EXPECT_EQ( root_graph.m_root_prior_sets, 1U );
  EXPECT_EQ( root_graph.m_imu_factors, 5U );
  EXPECT_EQ( root_graph.m_bias_rw_factors, 5U );
  EXPECT_EQ( root_graph.m_visual_factors, 120U );
  EXPECT_EQ( root_graph.m_integrated_duration_ns, 250'000'000 );

  const Eigen::Isometry3d truth =
      fixture::poseWb( latest_timestamp_ns );
  EXPECT_LT(
      ( root.estimate->T_W_B.translation() - truth.translation() ).norm(),
      1e-3 );
  EXPECT_LT(
      Eigen::AngleAxisd( root.estimate->T_W_B.rotation().transpose() *
                         truth.rotation() )
          .angle(),
      1e-4 );
  EXPECT_LT(
      ( root.estimate->m_v_W_B -
        fixture::velocityW( latest_timestamp_ns ) )
          .norm(),
      1e-3 );
  EXPECT_LT(
      ( root.estimate->m_bias.m_gyr_radps - fixture::kGyroBias ).norm(),
      1e-4 );

  const std::int64_t next_timestamp_ns =
      latest_timestamp_ns + fixture::kFramePeriodNs;
  const auto continued = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ next_timestamp_ns },
      .m_observations = fixture::observations( next_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          latest_timestamp_ns, next_timestamp_ns ) } );
  EXPECT_EQ( continued.status, UpdateStatus::kOk ) << continued.message;
  EXPECT_TRUE( continued.estimate.has_value() );
}

TEST( VioInitialization,
      InitialMovingRootAlignsNonZVisualGravityToNavigationFrame )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace tilted                  = fixture::non_z_gravity;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 7;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const auto update = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = tilted::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
    ASSERT_EQ( update.status, UpdateStatus::kInitializing )
        << "index=" << index << " message=" << update.message;
    EXPECT_FALSE( update.estimate.has_value() );
    EXPECT_EQ( update.diagnostics.m_initialization.m_path,
               phad::estimator::InitializationPath::kMoving );
    EXPECT_FALSE(
        update.diagnostics.m_initialization.m_attempt_id.has_value() );
  }

  constexpr std::int64_t kRootIndex = 5;
  const std::int64_t     root_timestamp_ns =
      fixture::kRootTimestampNs + kRootIndex * fixture::kFramePeriodNs;
  const auto root = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ root_timestamp_ns },
      .m_observations = fixture::observations( root_timestamp_ns ),
      .m_imu          = tilted::rawInterval(
          root_timestamp_ns - fixture::kFramePeriodNs,
          root_timestamp_ns ) } );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.estimate.has_value() );
  const auto& initialization = root.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCommit );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kNone );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kValidated );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kRootCommitted );
  EXPECT_EQ( root.diagnostics.window_size, 6U );
  EXPECT_EQ( root.diagnostics.num_cheirality, 0U );
  EXPECT_EQ( root.diagnostics.num_current_visual_factors, 20U );
  EXPECT_EQ( root.diagnostics.m_vio.m_nav_states, 6U );
  EXPECT_EQ( root.diagnostics.m_vio.m_root_prior_sets, 1U );
  EXPECT_EQ( root.diagnostics.m_vio.m_imu_factors, 5U );
  EXPECT_EQ( root.diagnostics.m_vio.m_bias_rw_factors, 5U );
  EXPECT_EQ( root.diagnostics.m_vio.m_visual_factors, 120U );

  const Eigen::Isometry3d expected_root = tilted::poseNb( root_timestamp_ns );
  EXPECT_LT( ( root.estimate->T_W_B.translation() -
               expected_root.translation() )
                 .norm(),
             1e-3 );
  EXPECT_LT(
      Eigen::AngleAxisd( root.estimate->T_W_B.rotation().transpose() *
                         expected_root.rotation() )
          .angle(),
      1e-4 );
  EXPECT_LT(
      ( root.estimate->m_v_W_B - tilted::velocityN( root_timestamp_ns ) )
          .norm(),
      1e-3 );
  EXPECT_LT(
      ( root.estimate->m_bias.m_gyr_radps - fixture::kGyroBias ).norm(),
      1e-4 );

  constexpr std::int64_t kContinuedIndex = 6;
  const std::int64_t     continued_timestamp_ns =
      fixture::kRootTimestampNs +
      kContinuedIndex * fixture::kFramePeriodNs;
  const auto continued = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ continued_timestamp_ns },
      .m_observations = fixture::observations( continued_timestamp_ns ),
      .m_imu          = tilted::rawInterval( root_timestamp_ns,
                                             continued_timestamp_ns ) } );
  ASSERT_EQ( continued.status, UpdateStatus::kOk ) << continued.message;
  ASSERT_TRUE( continued.estimate.has_value() );
  EXPECT_EQ( continued.diagnostics.num_cheirality, 0U );
  EXPECT_EQ( continued.diagnostics.num_current_visual_factors, 20U );
  EXPECT_EQ( continued.diagnostics.m_vio.m_nav_states, 7U );
  EXPECT_EQ( continued.diagnostics.m_vio.m_imu_factors, 6U );
  EXPECT_EQ( continued.diagnostics.m_vio.m_bias_rw_factors, 6U );
  EXPECT_EQ( continued.diagnostics.m_vio.m_visual_factors, 140U );

  const Eigen::Isometry3d expected_continued =
      tilted::poseNb( continued_timestamp_ns );
  EXPECT_LT( ( continued.estimate->T_W_B.translation() -
               expected_continued.translation() )
                 .norm(),
             1e-3 );
  EXPECT_LT(
      Eigen::AngleAxisd( continued.estimate->T_W_B.rotation().transpose() *
                         expected_continued.rotation() )
          .angle(),
      1e-4 );
  EXPECT_LT( ( continued.estimate->m_v_W_B -
               tilted::velocityN( continued_timestamp_ns ) )
                 .norm(),
             1e-3 );
}

TEST( VioInitialization, FivePosesAtExactDurationRemainPending )
{
  namespace fixture                 = phad::test_support::initial_moving;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 10 * fixture::kFramePeriodNs;
  options.window_size               = 7;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  constexpr std::array<std::int64_t, 5> kTimestamps{
      50'000'000, 100'000'000, 150'000'000, 200'000'000, 300'000'000 };
  VioUpdateResult latest;
  std::int64_t    previous_timestamp_ns = 0;
  for ( const std::int64_t timestamp_ns : kTimestamps )
  {
    latest = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            previous_timestamp_ns, timestamp_ns ) } );
    ASSERT_EQ( latest.status, UpdateStatus::kInitializing )
        << latest.message;
    previous_timestamp_ns = timestamp_ns;
  }

  EXPECT_FALSE( latest.estimate.has_value() );
  const auto& initialization = latest.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kPoseCountInsufficient );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 5U );
  EXPECT_EQ( initialization.m_connected_poses.m_required, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, 250'000'000 );
  EXPECT_EQ( initialization.m_duration.m_required_ns, 250'000'000 );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
}

TEST( VioInitialization, CurrentGraphRejectsUntilVisualFactorsAreAdmitted )
{
  namespace fixture                 = phad::test_support::initial_moving;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  options.min_landmark_observations = 7;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  VioUpdateResult sixth;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    sixth = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
  }

  ASSERT_EQ( sixth.status, UpdateStatus::kInitializing ) << sixth.message;
  EXPECT_FALSE( sixth.estimate.has_value() );
  const auto& rejected = sixth.diagnostics.m_initialization;
  EXPECT_EQ( rejected.m_phase,
             phad::estimator::InitializationPhase::kCurrentGraph );
  EXPECT_EQ( rejected.m_reason,
             phad::estimator::InitializationReason::kCurrentGraphRejected );
  EXPECT_EQ( rejected.m_outcome,
             phad::estimator::InitializationOutcome::kRecoverableRejected );
  ASSERT_TRUE( rejected.m_attempt_id.has_value() );
  EXPECT_EQ( rejected.m_current_graph,
             phad::estimator::InitializationGraphResult::kRejected );
  EXPECT_EQ(
      rejected.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( rejected.m_gyro_rank.has_value() );
  EXPECT_EQ( rejected.m_gyro_rank->m_actual, 3U );
  EXPECT_EQ( rejected.m_gyro_rank->m_required, 3U );
  ASSERT_TRUE( rejected.m_gravity_velocity_rank.has_value() );
  EXPECT_EQ( rejected.m_gravity_velocity_rank->m_actual, 20U );
  EXPECT_EQ( rejected.m_gravity_velocity_rank->m_required, 20U );
  ASSERT_TRUE( rejected.m_joint_fit.has_value() );
  EXPECT_GT( rejected.m_joint_fit->m_effective_dof, 0 );
  EXPECT_LE( rejected.m_joint_fit->m_statistic,
             rejected.m_joint_fit->m_limit );

  constexpr std::int64_t kRecoveryIndex = 6;
  const std::int64_t     recovery_timestamp_ns =
      fixture::kRootTimestampNs +
      kRecoveryIndex * fixture::kFramePeriodNs;
  const auto recovered = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ recovery_timestamp_ns },
      .m_observations = fixture::observations( recovery_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          recovery_timestamp_ns - fixture::kFramePeriodNs,
          recovery_timestamp_ns ) } );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT(
      *recovered.diagnostics.m_initialization.m_attempt_id,
      *rejected.m_attempt_id );
}

TEST( VioInitialization, GyroRankDeficiencyCommitsEvidenceAndRecovers )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
      estimator,
      internal::InitialMovingCandidateFailure{
          .m_code = internal::InitialMovingCandidateFailureCode::
              kRankDeficient,
          .m_linear_failure = internal::InitialMovingLinearFailure{
              .m_code =
                  internal::InitialMovingLinearFailureCode::kRankDeficient,
              .m_rank                 = 2U,
              .m_required_rank        = 3U,
              .m_normalized_condition = std::nullopt },
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = std::nullopt,
          .m_gravity_velocity = std::nullopt } );

  VioUpdateResult rejected;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    rejected = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
  }

  ASSERT_EQ( rejected.status, UpdateStatus::kInitializing )
      << rejected.message;
  EXPECT_FALSE( rejected.estimate.has_value() );
  EXPECT_EQ( rejected.diagnostics.num_disparity, 20U );
  const auto& initialization = rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kGyroAlignment );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kRankDeficient );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kRecoverableRejected );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  EXPECT_EQ( initialization.m_gyro_rank->m_actual, 2U );
  EXPECT_EQ( initialization.m_gyro_rank->m_required, 3U );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  EXPECT_EQ( initialization.m_connected_poses.m_actual, 6U );
  EXPECT_EQ( initialization.m_duration.m_actual_ns, 250'000'000 );
  EXPECT_EQ( initialization.m_connected_landmarks.m_actual, 20U );
  ASSERT_TRUE( initialization.m_visual_fit.has_value() );
  EXPECT_LE( initialization.m_visual_fit->m_statistic,
             initialization.m_visual_fit->m_limit );

  constexpr std::int64_t kRecoveryIndex = 6;
  const std::int64_t     recovery_timestamp_ns =
      fixture::kRootTimestampNs +
      kRecoveryIndex * fixture::kFramePeriodNs;
  const auto recovered = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ recovery_timestamp_ns },
      .m_observations = fixture::observations( recovery_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          recovery_timestamp_ns - fixture::kFramePeriodNs,
          recovery_timestamp_ns ) } );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT( *recovered.diagnostics.m_initialization.m_attempt_id,
             *initialization.m_attempt_id );
}

TEST( VioInitialization,
      GravityVelocityRankDeficiencyKeepsGyroMarginAndRecovers )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
      estimator,
      internal::InitialMovingCandidateFailure{
          .m_code = internal::InitialMovingCandidateFailureCode::
              kRankDeficient,
          .m_linear_failure = internal::InitialMovingLinearFailure{
              .m_code =
                  internal::InitialMovingLinearFailureCode::kRankDeficient,
              .m_rank                 = 19U,
              .m_required_rank        = 20U,
              .m_normalized_condition = std::nullopt },
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = internal::InitialMovingGyroSolution{ .m_bg_radps = fixture::kGyroBias, .m_rank = 3U, .m_normalized_condition = 1.0 },
          .m_gravity_velocity = std::nullopt } );

  VioUpdateResult rejected;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    rejected = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
  }

  ASSERT_EQ( rejected.status, UpdateStatus::kInitializing )
      << rejected.message;
  EXPECT_FALSE( rejected.estimate.has_value() );
  const auto& initialization = rejected.diagnostics.m_initialization;
  EXPECT_EQ(
      initialization.m_phase,
      phad::estimator::InitializationPhase::kGravityVelocitySolve );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kRankDeficient );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  EXPECT_EQ( initialization.m_gyro_rank->m_actual, 3U );
  EXPECT_EQ( initialization.m_gyro_rank->m_required, 3U );
  ASSERT_TRUE( initialization.m_gravity_velocity_rank.has_value() );
  EXPECT_EQ( initialization.m_gravity_velocity_rank->m_actual, 19U );
  EXPECT_EQ( initialization.m_gravity_velocity_rank->m_required, 20U );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );

  constexpr std::int64_t kRecoveryIndex = 6;
  const std::int64_t     recovery_timestamp_ns =
      fixture::kRootTimestampNs +
      kRecoveryIndex * fixture::kFramePeriodNs;
  const auto recovered = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ recovery_timestamp_ns },
      .m_observations = fixture::observations( recovery_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          recovery_timestamp_ns - fixture::kFramePeriodNs,
          recovery_timestamp_ns ) } );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT( *recovered.diagnostics.m_initialization.m_attempt_id,
             *initialization.m_attempt_id );
}

TEST( VioInitialization, FullRankIllConditionedGyroRecoversNextRevision )
{
  namespace internal              = phad::estimator::internal;
  const double rejected_condition = std::nextafter(
      internal::kInitialMovingNormalizedConditionLimit,
      std::numeric_limits<double>::infinity() );
  const InjectedMovingAttempt attempt = runInjectedMovingCandidateResult(
      internal::InitialMovingCandidateFailure{
          .m_code =
              internal::InitialMovingCandidateFailureCode::kIllConditioned,
          .m_linear_failure = internal::InitialMovingLinearFailure{
              .m_code =
                  internal::InitialMovingLinearFailureCode::kIllConditioned,
              .m_rank                 = 3U,
              .m_required_rank        = 3U,
              .m_normalized_condition = rejected_condition },
          .m_joint_fit        = std::nullopt,
          .m_joint_limit      = std::nullopt,
          .m_gyro             = std::nullopt,
          .m_gravity_velocity = std::nullopt } );

  ASSERT_EQ( attempt.rejected.status, UpdateStatus::kInitializing )
      << attempt.rejected.message;
  EXPECT_FALSE( attempt.rejected.estimate.has_value() );
  const auto& initialization =
      attempt.rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kGyroAlignment );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kIllConditioned );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  EXPECT_EQ( initialization.m_gyro_rank->m_actual, 3U );
  EXPECT_EQ( initialization.m_gyro_rank->m_required, 3U );
  ASSERT_TRUE( initialization.m_gyro_normalized_condition.has_value() );
  EXPECT_DOUBLE_EQ(
      initialization.m_gyro_normalized_condition->m_value,
      rejected_condition );
  EXPECT_DOUBLE_EQ(
      initialization.m_gyro_normalized_condition->m_limit,
      internal::kInitialMovingNormalizedConditionLimit );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_EQ( attempt.recovered.status, UpdateStatus::kOk )
      << attempt.recovered.message;
  ASSERT_TRUE(
      attempt.recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT(
      *attempt.recovered.diagnostics.m_initialization.m_attempt_id,
      *initialization.m_attempt_id );
}

TEST( VioInitialization, JointNoConvergenceKeepsEarlierMarginsAndRecovers )
{
  namespace fixture                   = phad::test_support::initial_moving;
  namespace internal                  = phad::estimator::internal;
  const InjectedMovingAttempt attempt = runInjectedMovingCandidateResult(
      internal::InitialMovingCandidateFailure{
          .m_code =
              internal::InitialMovingCandidateFailureCode::kNoConvergence,
          .m_linear_failure = std::nullopt,
          .m_joint_fit      = std::nullopt,
          .m_joint_limit    = std::nullopt,
          .m_gyro           = internal::InitialMovingGyroSolution{
                        .m_bg_radps             = fixture::kGyroBias,
                        .m_rank                 = 3U,
                        .m_normalized_condition = 1.0 },
          .m_gravity_velocity = internal::InitialMovingGravityVelocitySolution{ .m_g_W = fixture::kGravityW, .m_v_W = {}, .m_rank = 20U, .m_normalized_condition = 1.0 } } );

  ASSERT_EQ( attempt.rejected.status, UpdateStatus::kInitializing )
      << attempt.rejected.message;
  EXPECT_FALSE( attempt.rejected.estimate.has_value() );
  const auto& initialization =
      attempt.rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kJointConsistency );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kNoConvergence );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  EXPECT_EQ( initialization.m_gyro_rank->m_actual, 3U );
  ASSERT_TRUE( initialization.m_gravity_velocity_rank.has_value() );
  EXPECT_EQ( initialization.m_gravity_velocity_rank->m_actual, 20U );
  EXPECT_FALSE( initialization.m_joint_fit.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_EQ( attempt.recovered.status, UpdateStatus::kOk )
      << attempt.recovered.message;
  ASSERT_TRUE(
      attempt.recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT(
      *attempt.recovered.diagnostics.m_initialization.m_attempt_id,
      *initialization.m_attempt_id );
}

TEST( VioInitialization, JointZeroDofCommitsEvidenceAndRecovers )
{
  namespace fixture                   = phad::test_support::initial_moving;
  namespace internal                  = phad::estimator::internal;
  const InjectedMovingAttempt attempt = runInjectedMovingCandidateResult(
      internal::InitialMovingCandidateFailure{
          .m_code = internal::InitialMovingCandidateFailureCode::
              kJointDofInsufficient,
          .m_linear_failure = std::nullopt,
          .m_joint_fit      = internal::InitialMovingJointFit{
                   .m_statistic     = 0.0,
                   .m_residual_rank = 5,
                   .m_fitted_rank   = 5,
                   .m_effective_dof = 0 },
          .m_joint_limit      = std::nullopt,
          .m_gyro             = internal::InitialMovingGyroSolution{ .m_bg_radps = fixture::kGyroBias, .m_rank = 3U, .m_normalized_condition = 1.0 },
          .m_gravity_velocity = internal::InitialMovingGravityVelocitySolution{ .m_g_W = fixture::kGravityW, .m_v_W = {}, .m_rank = 20U, .m_normalized_condition = 1.0 } } );

  ASSERT_EQ( attempt.rejected.status, UpdateStatus::kInitializing )
      << attempt.rejected.message;
  const auto& initialization =
      attempt.rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kJointConsistency );
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kJointDofInsufficient );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  ASSERT_TRUE( initialization.m_gravity_velocity_rank.has_value() );
  ASSERT_TRUE( initialization.m_joint_fit.has_value() );
  EXPECT_EQ( initialization.m_joint_fit->m_effective_dof, 0 );
  EXPECT_DOUBLE_EQ( initialization.m_joint_fit->m_statistic, 0.0 );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_EQ( attempt.recovered.status, UpdateStatus::kOk )
      << attempt.recovered.message;
  ASSERT_TRUE(
      attempt.recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT(
      *attempt.recovered.diagnostics.m_initialization.m_attempt_id,
      *initialization.m_attempt_id );
}

TEST( VioInitialization, JointInconsistencyCommitsEvidenceAndRecovers )
{
  namespace fixture                       = phad::test_support::initial_moving;
  namespace internal                      = phad::estimator::internal;
  constexpr double            kJointLimit = 9.21034037197618;
  const InjectedMovingAttempt attempt     = runInjectedMovingCandidateResult(
      internal::InitialMovingCandidateFailure{
              .m_code =
              internal::InitialMovingCandidateFailureCode::kJointInconsistent,
              .m_linear_failure = std::nullopt,
              .m_joint_fit      = internal::InitialMovingJointFit{
                       .m_statistic     = 10'000.0,
                       .m_residual_rank = 5,
                       .m_fitted_rank   = 3,
                       .m_effective_dof = 2 },
              .m_joint_limit      = kJointLimit,
              .m_gyro             = internal::InitialMovingGyroSolution{ .m_bg_radps = fixture::kGyroBias, .m_rank = 3U, .m_normalized_condition = 1.0 },
              .m_gravity_velocity = internal::InitialMovingGravityVelocitySolution{ .m_g_W = fixture::kGravityW, .m_v_W = {}, .m_rank = 20U, .m_normalized_condition = 1.0 } } );

  ASSERT_EQ( attempt.rejected.status, UpdateStatus::kInitializing )
      << attempt.rejected.message;
  EXPECT_FALSE( attempt.rejected.estimate.has_value() );
  const auto& initialization =
      attempt.rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kJointConsistency );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kJointInconsistent );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  ASSERT_TRUE( initialization.m_gravity_velocity_rank.has_value() );
  ASSERT_TRUE( initialization.m_joint_fit.has_value() );
  EXPECT_EQ( initialization.m_joint_fit->m_effective_dof, 2 );
  EXPECT_DOUBLE_EQ( initialization.m_joint_fit->m_limit, kJointLimit );
  EXPECT_GT( initialization.m_joint_fit->m_statistic,
             initialization.m_joint_fit->m_limit );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  ASSERT_EQ( attempt.recovered.status, UpdateStatus::kOk )
      << attempt.recovered.message;
  ASSERT_TRUE(
      attempt.recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT(
      *attempt.recovered.diagnostics.m_initialization.m_attempt_id,
      *initialization.m_attempt_id );
}

TEST( VioInitialization,
      CurrentGraphUnderconstraintCommitsEvidenceAndRecovers )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextCurrentGraphResult(
      estimator,
      phad::estimator::InitializationReason::kCurrentGraphUnderconstrained );

  VioUpdateResult rejected;
  for ( std::int64_t index = 0; index < 6; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    rejected = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
  }

  ASSERT_EQ( rejected.status, UpdateStatus::kInitializing )
      << rejected.message;
  EXPECT_FALSE( rejected.estimate.has_value() );
  const auto& initialization = rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCurrentGraph );
  EXPECT_EQ(
      initialization.m_reason,
      phad::estimator::InitializationReason::kCurrentGraphUnderconstrained );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kRecoverableRejected );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kRejected );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
  ASSERT_TRUE( initialization.m_gyro_rank.has_value() );
  EXPECT_EQ( initialization.m_gyro_rank->m_actual, 3U );
  ASSERT_TRUE( initialization.m_gravity_velocity_rank.has_value() );
  EXPECT_EQ( initialization.m_gravity_velocity_rank->m_actual, 20U );
  ASSERT_TRUE( initialization.m_joint_fit.has_value() );
  EXPECT_GT( initialization.m_joint_fit->m_effective_dof, 0 );

  constexpr std::int64_t kRecoveryIndex = 6;
  const std::int64_t     recovery_timestamp_ns =
      fixture::kRootTimestampNs +
      kRecoveryIndex * fixture::kFramePeriodNs;
  const auto recovered = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ recovery_timestamp_ns },
      .m_observations = fixture::observations( recovery_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          recovery_timestamp_ns - fixture::kFramePeriodNs,
          recovery_timestamp_ns ) } );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT( *recovered.diagnostics.m_initialization.m_attempt_id,
             *initialization.m_attempt_id );
}

TEST( VioInitialization, CurrentGraphContractErrorRollsBackExactly )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextCurrentGraphContractError(
      subject );
  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const VioMeasurement measurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) };
    ASSERT_EQ( subject.update( measurement ).status,
               UpdateStatus::kInitializing );
    ASSERT_EQ( control.update( measurement ).status,
               UpdateStatus::kInitializing );
  }

  constexpr std::int64_t kAttemptIndex = 5;
  const std::int64_t     attempt_timestamp_ns =
      fixture::kRootTimestampNs +
      kAttemptIndex * fixture::kFramePeriodNs;
  const VioMeasurement attempt{
      .m_timestamp    = Timestamp{ attempt_timestamp_ns },
      .m_observations = fixture::observations( attempt_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          attempt_timestamp_ns - fixture::kFramePeriodNs,
          attempt_timestamp_ns ) };
  const auto fatal           = subject.update( attempt );
  const auto control_success = control.update( attempt );

  ASSERT_EQ( fatal.status, UpdateStatus::kFailed ) << fatal.message;
  EXPECT_FALSE( fatal.estimate.has_value() );
  EXPECT_EQ( fatal.diagnostics.num_disparity, 20U );
  const auto& initialization = fatal.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kMoving );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kCurrentGraph );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kContractError );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kFatalError );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kNotEvaluated );
  EXPECT_EQ( initialization.m_transaction,
             phad::estimator::InitializationTransactionResult::kRolledBack );
  EXPECT_EQ( fatal.diagnostics.window_size, 5U );

  const auto recovered = subject.update( attempt );
  expectSameCommittedUpdate( recovered, control_success );
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT( *recovered.diagnostics.m_initialization.m_attempt_id,
             *initialization.m_attempt_id );
}

TEST( VioInitialization, CandidateMathErrorRollsBackExactly )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
      subject,
      internal::InitialMovingMathError{
          .m_code   = internal::InitialMovingMathErrorCode::kNumerical,
          .m_detail = "injected non-finite whitening" } );
  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const VioMeasurement measurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) };
    ASSERT_EQ( subject.update( measurement ).status,
               UpdateStatus::kInitializing );
    ASSERT_EQ( control.update( measurement ).status,
               UpdateStatus::kInitializing );
  }

  constexpr std::int64_t kAttemptIndex = 5;
  const std::int64_t     attempt_timestamp_ns =
      fixture::kRootTimestampNs +
      kAttemptIndex * fixture::kFramePeriodNs;
  const VioMeasurement attempt{
      .m_timestamp    = Timestamp{ attempt_timestamp_ns },
      .m_observations = fixture::observations( attempt_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          attempt_timestamp_ns - fixture::kFramePeriodNs,
          attempt_timestamp_ns ) };
  const auto fatal           = subject.update( attempt );
  const auto control_success = control.update( attempt );

  ASSERT_EQ( fatal.status, UpdateStatus::kFailed ) << fatal.message;
  EXPECT_FALSE( fatal.estimate.has_value() );
  const auto& initialization = fatal.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kJointConsistency );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kContractError );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kFatalError );
  ASSERT_TRUE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kNotEvaluated );
  EXPECT_EQ( initialization.m_transaction,
             phad::estimator::InitializationTransactionResult::kRolledBack );
  EXPECT_EQ( fatal.diagnostics.window_size, 5U );

  const auto recovered = subject.update( attempt );
  expectSameCommittedUpdate( recovered, control_success );
  ASSERT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
  EXPECT_GT( *recovered.diagnostics.m_initialization.m_attempt_id,
             *initialization.m_attempt_id );
}

TEST( VioInitialization, VisualCheiralityRejectionRecoversNextRevision )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const auto pending = estimator.update( VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) } );
    ASSERT_EQ( pending.status, UpdateStatus::kInitializing );
  }
  internal::VioEstimatorTestAccess::setNextInitialVisualDepth(
      estimator, 0.0 );

  constexpr std::int64_t kRejectedIndex = 5;
  const std::int64_t     rejected_timestamp_ns =
      fixture::kRootTimestampNs +
      kRejectedIndex * fixture::kFramePeriodNs;
  const auto rejected = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ rejected_timestamp_ns },
      .m_observations = fixture::observations( rejected_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          rejected_timestamp_ns - fixture::kFramePeriodNs,
          rejected_timestamp_ns ) } );
  ASSERT_EQ( rejected.status, UpdateStatus::kInitializing )
      << rejected.message;
  EXPECT_FALSE( rejected.estimate.has_value() );
  const auto& initialization = rejected.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kVisualSolve );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kCheiralityRejected );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kPending );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted );

  constexpr std::int64_t kRecoveryIndex = 6;
  const std::int64_t     recovery_timestamp_ns =
      fixture::kRootTimestampNs +
      kRecoveryIndex * fixture::kFramePeriodNs;
  const auto recovered = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ recovery_timestamp_ns },
      .m_observations = fixture::observations( recovery_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          recovery_timestamp_ns - fixture::kFramePeriodNs,
          recovery_timestamp_ns ) } );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_TRUE(
      recovered.diagnostics.m_initialization.m_attempt_id.has_value() );
}

TEST( VioInitialization, VisualWhiteningErrorRollsBackExactly )
{
  namespace fixture                 = phad::test_support::initial_moving;
  namespace internal                = phad::estimator::internal;
  EstimatorOptions options          = makeOptions();
  options.m_enable_moving_bootstrap = true;
  options.m_bootstrap_timeout_ns    = 2 * fixture::kFramePeriodNs;
  options.window_size               = 8;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  for ( std::int64_t index = 0; index < 5; ++index )
  {
    const std::int64_t timestamp_ns =
        fixture::kRootTimestampNs + index * fixture::kFramePeriodNs;
    const VioMeasurement measurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = fixture::observations( timestamp_ns ),
        .m_imu          = fixture::rawInterval(
            timestamp_ns - fixture::kFramePeriodNs, timestamp_ns ) };
    ASSERT_EQ( subject.update( measurement ).status,
               UpdateStatus::kInitializing );
    ASSERT_EQ( control.update( measurement ).status,
               UpdateStatus::kInitializing );
  }
  internal::VioEstimatorTestAccess::setNextInitialVisualWhiteningError(
      subject );

  constexpr std::int64_t kAttemptIndex = 5;
  const std::int64_t     attempt_timestamp_ns =
      fixture::kRootTimestampNs +
      kAttemptIndex * fixture::kFramePeriodNs;
  const VioMeasurement attempt{
      .m_timestamp    = Timestamp{ attempt_timestamp_ns },
      .m_observations = fixture::observations( attempt_timestamp_ns ),
      .m_imu          = fixture::rawInterval(
          attempt_timestamp_ns - fixture::kFramePeriodNs,
          attempt_timestamp_ns ) };
  const auto fatal           = subject.update( attempt );
  const auto control_success = control.update( attempt );

  ASSERT_EQ( fatal.status, UpdateStatus::kFailed ) << fatal.message;
  EXPECT_FALSE( fatal.estimate.has_value() );
  const auto& initialization = fatal.diagnostics.m_initialization;
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kVisualSolve );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kContractError );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kFatalError );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_transaction,
             phad::estimator::InitializationTransactionResult::kRolledBack );
  EXPECT_EQ( fatal.diagnostics.window_size, 5U );

  const auto recovered = subject.update( attempt );
  expectSameCommittedUpdate( recovered, control_success );
}

TEST( VioFullState, InvalidRawIntervalPreservesNextTransaction )
{
  VioEstimator subject( makeCalibration(), makeImuParameters(),
                        makeOptions() );
  VioEstimator control( makeCalibration(), makeImuParameters(),
                        makeOptions() );
  const auto   root_measurement = makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) );
  ASSERT_EQ( subject.update( root_measurement ).status, UpdateStatus::kOk );
  ASSERT_EQ( control.update( root_measurement ).status, UpdateStatus::kOk );

  RawImuInterval missing_end_bracket{
      .m_t_begin = Timestamp{ kFramePeriodNs },
      .m_t_end   = Timestamp{ 2 * kFramePeriodNs },
      .m_samples = { makeImu( kFramePeriodNs, 0.0 ),
                     makeImu( 75'000'000, 0.0 ),
                     makeImu( 90'000'000, 0.0 ) } };
  const auto invalid = subject.update( makeMeasurement(
      2 * kFramePeriodNs, std::move( missing_end_bracket ) ) );
  EXPECT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  EXPECT_FALSE( invalid.estimate.has_value() );

  const auto legal = makeMeasurement(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ) );
  const auto after_invalid = subject.update( legal );
  const auto direct        = control.update( legal );
  expectSameEstimate( after_invalid, direct );
  EXPECT_EQ( after_invalid.diagnostics.prior_key,
             direct.diagnostics.prior_key );
  EXPECT_EQ( after_invalid.diagnostics.window_size,
             direct.diagnostics.window_size );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_nav_states,
             direct.diagnostics.m_vio.m_nav_states );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_imu_factors,
             direct.diagnostics.m_vio.m_imu_factors );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_bias_rw_factors,
             direct.diagnostics.m_vio.m_bias_rw_factors );
}

TEST( VioFullState, InvalidObservationPreservesCoastAndTrackState )
{
  VioEstimator subject( makeCalibration(), makeImuParameters(),
                        transactionOptions() );
  VioEstimator control( makeCalibration(), makeImuParameters(),
                        transactionOptions() );
  primeSupportedCoast( subject );
  primeSupportedCoast( control );

  constexpr std::uint64_t kUniqueId = 9'001U;
  StereoObservation       malformed = makeUniqueObservation( kUniqueId );
  malformed.disparity_px            = -1.0;
  const auto invalid                = subject.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      { malformed } ) );
  ASSERT_EQ( invalid.status, UpdateStatus::kInvalidInput )
      << invalid.message;
  EXPECT_FALSE( invalid.estimate.has_value() );
  expectZeroPacketCounters( invalid );
  EXPECT_EQ( invalid.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );
  EXPECT_EQ( invalid.diagnostics.unsupported_span_ns, kFramePeriodNs );
  EXPECT_TRUE( subject.observationTimestamps( kUniqueId ).empty() );

  const auto legal = makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      { makeUniqueObservation( kUniqueId ) } );
  const auto after_invalid = subject.update( legal );
  const auto direct        = control.update( legal );
  expectSameCommittedUpdate( after_invalid, direct );
  EXPECT_EQ( after_invalid.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( after_invalid.diagnostics.unsupported_span_ns,
             2 * kFramePeriodNs );
  EXPECT_EQ( subject.observationTimestamps( kUniqueId ),
             control.observationTimestamps( kUniqueId ) );
  ASSERT_EQ( subject.observationTimestamps( kUniqueId ).size(), 1U );
}

TEST( VioFullState, DuplicateLandmarkIdIsInvalidWithoutStaging )
{
  VioEstimator                   subject( makeCalibration(), makeImuParameters(),
                                          makeOptions() );
  VioEstimator                   control( makeCalibration(), makeImuParameters(),
                                          makeOptions() );
  std::vector<StereoObservation> duplicate = makeObservations();
  duplicate.push_back( duplicate.front() );

  const auto invalid = subject.update( makeMeasurementWithObservations(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ),
      std::move( duplicate ) ) );
  EXPECT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  EXPECT_FALSE( invalid.estimate.has_value() );
  EXPECT_NE( invalid.message.find( "duplicate landmark id" ),
             std::string::npos );
  EXPECT_TRUE( subject.observationTimestamps( 1U ).empty() );

  const auto legal = makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) );
  const auto after_invalid = subject.update( legal );
  const auto direct        = control.update( legal );
  expectSameCommittedUpdate( after_invalid, direct );
}

TEST( VioFullState, NonFinitePropagationRollsBackIntakeAndSpan )
{
  VioEstimator subject( makeCalibration(), makeImuParameters(),
                        transactionOptions() );
  VioEstimator control( makeCalibration(), makeImuParameters(),
                        transactionOptions() );
  primeSupportedCoast( subject );
  primeSupportedCoast( control );

  constexpr std::uint64_t kUniqueId = 9'002U;
  constexpr std::int64_t  kHugeTimestamp =
      9'000'000'000'000'000'000LL;
  constexpr std::int64_t kMidTimestamp = kHugeTimestamp / 2;
  const double           huge_acc =
      std::numeric_limits<double>::max() / 4.0;
  RawImuInterval explosive{
      .m_t_begin = Timestamp{ 3 * kFramePeriodNs },
      .m_t_end   = Timestamp{ kHugeTimestamp },
      .m_samples = {
          makeImu( 3 * kFramePeriodNs, 0.0 ),
          makeImuWithAcc( kMidTimestamp,
                          Eigen::Vector3d{ huge_acc, 0.0, 9.81 } ),
          makeImuWithAcc( kHugeTimestamp,
                          Eigen::Vector3d{ huge_acc, 0.0, 9.81 } ) } };
  auto observations = makeObservations();
  observations.push_back( makeUniqueObservation( kUniqueId ) );
  const auto failed = subject.update( makeMeasurementWithObservations(
      kHugeTimestamp, std::move( explosive ), std::move( observations ) ) );
  ASSERT_EQ( failed.status, UpdateStatus::kFailed ) << failed.message;
  EXPECT_FALSE( failed.estimate.has_value() );
  expectZeroPacketCounters( failed );
  EXPECT_EQ( failed.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );
  EXPECT_EQ( failed.diagnostics.unsupported_span_ns, kFramePeriodNs );
  EXPECT_TRUE( subject.observationTimestamps( kUniqueId ).empty() );

  const auto legal = makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      { makeUniqueObservation( kUniqueId ) } );
  const auto after_failed = subject.update( legal );
  const auto direct       = control.update( legal );
  expectSameCommittedUpdate( after_failed, direct );
  EXPECT_EQ( after_failed.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( subject.observationTimestamps( kUniqueId ),
             control.observationTimestamps( kUniqueId ) );
  ASSERT_EQ( subject.observationTimestamps( kUniqueId ).size(), 1U );
}

TEST( VioFullState, PrimaryGraphFailureRollsBackIntakeSeedAndSpan )
{
  VioEstimator            subject( makeCalibration(), makeImuParameters(),
                                   transactionOptions() );
  VioEstimator            control( makeCalibration(), makeImuParameters(),
                                   transactionOptions() );
  constexpr std::uint64_t kUniqueId = 9'003U;
  for ( VioEstimator* estimator : { &subject, &control } )
  {
    ASSERT_EQ( estimator
                   ->update( makeMeasurement(
                       kFramePeriodNs,
                       stationaryInterval( 0, kFramePeriodNs ) ) )
                   .status,
               UpdateStatus::kOk );
    ASSERT_EQ( estimator
                   ->update( makeMeasurement(
                       2 * kFramePeriodNs,
                       stationaryInterval( kFramePeriodNs,
                                           2 * kFramePeriodNs ) ) )
                   .status,
               UpdateStatus::kOk );
    const auto retained = estimator->update( makeMeasurementWithObservations(
        3 * kFramePeriodNs,
        stationaryInterval( 2 * kFramePeriodNs, 3 * kFramePeriodNs ),
        { makeUniqueObservation( kUniqueId ) } ) );
    ASSERT_EQ( retained.status, UpdateStatus::kOk ) << retained.message;
    ASSERT_EQ( retained.diagnostics.num_seeded_landmarks, 0U );
  }

  auto conflicting = makeObservations();
  conflicting.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  conflicting.push_back( makeUniqueObservation( kUniqueId ) );
  const auto failed = subject.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      std::move( conflicting ) ) );
  ASSERT_EQ( failed.status, UpdateStatus::kFailed ) << failed.message;
  EXPECT_FALSE( failed.estimate.has_value() );
  expectZeroPacketCounters( failed );
  EXPECT_EQ( failed.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );
  EXPECT_EQ( failed.diagnostics.unsupported_span_ns, kFramePeriodNs );
  EXPECT_EQ( subject.observationTimestamps( kUniqueId ),
             control.observationTimestamps( kUniqueId ) );
  ASSERT_EQ( subject.observationTimestamps( kUniqueId ).size(), 1U );

  const auto legal = makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      { makeUniqueObservation( kUniqueId ) } );
  const auto after_failed = subject.update( legal );
  const auto direct       = control.update( legal );
  expectSameCommittedUpdate( after_failed, direct );
  EXPECT_EQ( after_failed.diagnostics.num_seeded_landmarks, 1U );
  EXPECT_EQ( after_failed.diagnostics.num_current_visual_factors, 1U );
  EXPECT_EQ( subject.observationTimestamps( kUniqueId ),
             control.observationTimestamps( kUniqueId ) );
  ASSERT_EQ( subject.observationTimestamps( kUniqueId ).size(), 2U );
}

TEST( VioFullState, BootstrapTimestampOverflowIsInvalidInputWithoutAdvancingAnchor )
{
  VioEstimator           estimator( makeCalibration(), makeImuParameters(),
                                    makeOptions() );
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();

  VioMeasurement first{
      .m_timestamp    = Timestamp{ -1 },
      .m_observations = {},
      .m_imu          = RawImuInterval{
                   .m_t_begin = Timestamp{ kMin },
                   .m_t_end   = Timestamp{ -1 },
                   .m_samples = { makeImu( kMin, 0.0 ),
                                  makeImu( -4'611'686'018'427'387'905, 0.0 ),
                                  makeImu( -1, 0.0 ) } } };
  const auto initializing = estimator.update( first );
  ASSERT_EQ( initializing.status, UpdateStatus::kInitializing );

  RawImuInterval overflowing_span{
      .m_t_begin = Timestamp{ -1 },
      .m_t_end   = Timestamp{ kMax - 1 },
      .m_samples = { makeImu( -1, 0.0 ),
                     makeImu( 4'611'686'018'427'387'902, 0.0 ),
                     makeImu( kMax - 1, 0.0 ) } };
  const auto invalid = estimator.update(
      makeMeasurement( kMax - 1, std::move( overflowing_span ) ) );
  EXPECT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  EXPECT_FALSE( invalid.estimate.has_value() );

  const auto invalid_declaration = estimator.update( makeMeasurement(
      0, MeasurementDiscontinuity{ .m_t_begin = Timestamp{ -2 },
                                   .m_t_end   = Timestamp{ 0 } } ) );
  EXPECT_EQ( invalid_declaration.status, UpdateStatus::kInvalidInput );

  const auto reset = estimator.update( makeMeasurement(
      0, MeasurementDiscontinuity{ .m_t_begin = Timestamp{ -1 },
                                   .m_t_end   = Timestamp{ 0 } } ) );
  EXPECT_EQ( reset.status, UpdateStatus::kInitializing );
}

TEST( VioFullState, DiscontinuityStartsIndependentRootSegment )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                     kFramePeriodNs,
                     stationaryInterval( 0, kFramePeriodNs ) ) )
                 .status,
             UpdateStatus::kOk );

  const auto coast = estimator.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ), {} ) );
  ASSERT_EQ( coast.status, UpdateStatus::kOk ) << coast.message;
  EXPECT_EQ( coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );

  const auto discontinuity = estimator.update( makeMeasurement(
      3 * kFramePeriodNs,
      MeasurementDiscontinuity{ .m_t_begin = Timestamp{ 2 * kFramePeriodNs },
                                .m_t_end =
                                    Timestamp{ 3 * kFramePeriodNs } } ) );
  EXPECT_EQ( discontinuity.status, UpdateStatus::kDiscontinuity );
  EXPECT_FALSE( discontinuity.estimate.has_value() );
  EXPECT_EQ( discontinuity.diagnostics.segment_id, 1U );
  ASSERT_TRUE(
      discontinuity.diagnostics.m_vio.m_completed_segment_id.has_value() );
  EXPECT_EQ(
      *discontinuity.diagnostics.m_vio.m_completed_segment_id, 0U );
  EXPECT_EQ( discontinuity.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );

  const auto new_root = estimator.update( makeMeasurement(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ) ) );
  ASSERT_EQ( new_root.status, UpdateStatus::kOk ) << new_root.message;
  ASSERT_TRUE( new_root.estimate.has_value() );
  EXPECT_EQ( new_root.estimate->m_segment_id, 1U );
  EXPECT_EQ( new_root.estimate->timestamp, Timestamp{ 4 * kFramePeriodNs } );
  EXPECT_TRUE( new_root.estimate->T_W_B.matrix().isApprox(
      Eigen::Isometry3d::Identity().matrix(), 1e-12 ) );
  EXPECT_TRUE( new_root.estimate->m_v_W_B.isZero( 1e-12 ) );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_nav_states, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_root_prior_sets, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_imu_factors, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_bias_rw_factors, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_integration_steps, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_integrated_duration_ns, 0 );
}

TEST( VioFullState, VisualCoastRetainsFactorsAndRecoversInSegment )
{
  EstimatorOptions options          = makeOptions();
  options.window_size               = 32;
  options.min_landmark_observations = 1;
  options.min_pnp_inliers           = 4;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  const auto root = estimator.update( makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_EQ( root.diagnostics.m_vio.m_visual_factors, 10U );

  const auto first_coast = estimator.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ), {} ) );
  ASSERT_EQ( first_coast.status, UpdateStatus::kOk ) << first_coast.message;
  ASSERT_TRUE( first_coast.estimate.has_value() );
  EXPECT_EQ( first_coast.estimate->m_segment_id, 0U );
  EXPECT_EQ( first_coast.diagnostics.num_shared, 0U );
  EXPECT_EQ( first_coast.diagnostics.m_vio.m_nav_states, 2U );
  EXPECT_EQ( first_coast.diagnostics.m_vio.m_imu_factors, 1U );
  EXPECT_EQ( first_coast.diagnostics.m_vio.m_bias_rw_factors, 1U );
  EXPECT_EQ( first_coast.diagnostics.m_vio.m_visual_factors, 10U );
  EXPECT_EQ( first_coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );

  const auto second_coast = estimator.update( makeMeasurementWithObservations(
      3 * kFramePeriodNs,
      stationaryInterval( 2 * kFramePeriodNs, 3 * kFramePeriodNs ),
      makeUnmappedObservations() ) );
  ASSERT_EQ( second_coast.status, UpdateStatus::kOk )
      << second_coast.message;
  ASSERT_TRUE( second_coast.estimate.has_value() );
  EXPECT_EQ( second_coast.estimate->m_segment_id, 0U );
  EXPECT_EQ( second_coast.diagnostics.num_disparity, 10U );
  EXPECT_EQ( second_coast.diagnostics.num_shared, 0U );
  EXPECT_EQ( second_coast.diagnostics.m_vio.m_nav_states, 3U );
  EXPECT_EQ( second_coast.diagnostics.m_vio.m_imu_factors, 2U );
  EXPECT_EQ( second_coast.diagnostics.m_vio.m_bias_rw_factors, 2U );
  EXPECT_EQ( second_coast.diagnostics.num_retained_observations, 10U );
  EXPECT_EQ( second_coast.diagnostics.num_seeded_landmarks, 10U );
  EXPECT_EQ( second_coast.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( second_coast.diagnostics.m_vio.m_visual_factors, 20U );
  EXPECT_EQ( second_coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             2 * kFramePeriodNs );

  const auto recovered = estimator.update( makeMeasurement(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ) ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_EQ( recovered.estimate->m_segment_id, 0U );
  EXPECT_EQ( recovered.diagnostics.num_shared, 10U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_nav_states, 4U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_imu_factors, 3U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_bias_rw_factors, 3U );
  EXPECT_EQ( recovered.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_factors, 30U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
}

TEST( VioFullState, MappedLeftOnlySupportRecoversActiveSegment )
{
  EstimatorOptions options = transactionOptions();
  VioEstimator     estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  const auto recovered = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      makeLeftOnlyObservations() ) );

  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_EQ( recovered.estimate->m_segment_id, 0U );
  EXPECT_EQ( recovered.diagnostics.num_shared, 0U );
  EXPECT_EQ( recovered.diagnostics.num_mapped_observations, 10U );
  EXPECT_EQ( recovered.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( recovered.diagnostics.num_current_mono_visual_factors, 10U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  EXPECT_EQ( recovered.diagnostics.unsupported_span_ns, 0 );
}

TEST( VioFullState, MappedOverlapClearsLowConnectivity )
{
  EstimatorOptions options     = transactionOptions();
  options.min_pnp_inliers      = 4;
  options.min_shared_landmarks = 8;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  const auto supported = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      makeLeftOnlyObservations() ) );

  ASSERT_EQ( supported.status, UpdateStatus::kOk ) << supported.message;
  EXPECT_EQ( supported.diagnostics.num_shared, 0U );
  EXPECT_EQ( supported.diagnostics.num_mapped_observations, 10U );
  EXPECT_FALSE( supported.diagnostics.low_connectivity );
}

TEST( VioFullState, MappedOverlapBelowThresholdKeepsCoastWithMixedFactors )
{
  EstimatorOptions options = transactionOptions();
  VioEstimator     estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  std::vector<StereoObservation> mixed = makeLeftOnlyObservations( 9U );
  mixed.front().disparity_px           = 12.0;
  const auto coast                     = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      std::move( mixed ) ) );

  ASSERT_EQ( coast.status, UpdateStatus::kOk ) << coast.message;
  ASSERT_TRUE( coast.estimate.has_value() );
  EXPECT_EQ( coast.estimate->m_segment_id, 0U );
  EXPECT_EQ( coast.diagnostics.num_shared, 1U );
  EXPECT_EQ( coast.diagnostics.num_mapped_observations, 9U );
  EXPECT_EQ( coast.diagnostics.num_current_visual_factors, 9U );
  EXPECT_EQ( coast.diagnostics.num_current_mono_visual_factors, 8U );
  EXPECT_EQ( coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             2 * kFramePeriodNs );
  EXPECT_EQ( coast.diagnostics.unsupported_span_ns,
             2 * kFramePeriodNs );
}

TEST( VioFullState, CurrentStereoSeedDoesNotPromotePreIntakeMappedSupport )
{
  EstimatorOptions options                = transactionOptions();
  options.min_track_observations_for_seed = 1;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  std::vector<StereoObservation> observations =
      makeLeftOnlyObservations( 9U );
  observations.push_back( makeUniqueObservation( 9'004U ) );
  const auto coast = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      std::move( observations ) ) );

  ASSERT_EQ( coast.status, UpdateStatus::kOk ) << coast.message;
  EXPECT_EQ( coast.diagnostics.num_disparity, 1U );
  EXPECT_EQ( coast.diagnostics.num_shared, 0U );
  EXPECT_EQ( coast.diagnostics.num_mapped_observations, 9U );
  EXPECT_EQ( coast.diagnostics.num_seeded_landmarks, 1U );
  EXPECT_EQ( coast.diagnostics.num_current_visual_factors, 9U );
  EXPECT_EQ( coast.diagnostics.num_current_mono_visual_factors, 9U );
  EXPECT_EQ( coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             2 * kFramePeriodNs );
  EXPECT_EQ( coast.diagnostics.unsupported_span_ns,
             2 * kFramePeriodNs );
}

TEST( VioFullState, PnpFallbackDoesNotBlockMappedSupportRecovery )
{
  EstimatorOptions options = transactionOptions();
  options.enable_pnp_init  = true;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  std::vector<StereoObservation> left_only = makeLeftOnlyObservations();
  left_only.front().left_pixel.x() += 40.0;
  const auto recovered = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      std::move( left_only ) ) );

  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_FALSE( recovered.diagnostics.pnp_success );
  EXPECT_EQ( recovered.diagnostics.pnp_inliers, 0U );
  EXPECT_EQ( recovered.diagnostics.num_shared, 0U );
  EXPECT_EQ( recovered.diagnostics.num_mapped_observations, 10U );
  EXPECT_EQ( recovered.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( recovered.diagnostics.num_current_mono_visual_factors, 10U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  EXPECT_EQ( recovered.diagnostics.unsupported_span_ns, 0 );
}

TEST( VioFullState, MappedSupportRecoveryRequiresSuccessfulPrimaryCommit )
{
  EstimatorOptions options = transactionOptions();
  VioEstimator     estimator( makeCalibration(), makeImuParameters(), options );
  primeSupportedCoast( estimator );

  std::vector<StereoObservation> explosive = makeLeftOnlyObservations();
  explosive.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  const auto failed = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      std::move( explosive ) ) );

  ASSERT_EQ( failed.status, UpdateStatus::kFailed ) << failed.message;
  EXPECT_FALSE( failed.estimate.has_value() );
  EXPECT_EQ( failed.diagnostics.num_shared, 0U );
  EXPECT_EQ( failed.diagnostics.num_mapped_observations, 10U );
  EXPECT_EQ( failed.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );
  EXPECT_EQ( failed.diagnostics.unsupported_span_ns, kFramePeriodNs );

  const auto recovered = estimator.update( makeMeasurementWithObservations(
      4 * kFramePeriodNs,
      stationaryInterval( 3 * kFramePeriodNs, 4 * kFramePeriodNs ),
      makeLeftOnlyObservations() ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  EXPECT_EQ( recovered.diagnostics.unsupported_span_ns, 0 );
}

TEST( VioFullState, ExactVisualCoastHorizonCommitsBeforeSegmentCompletion )
{
  EstimatorOptions options          = makeOptions();
  options.window_size               = 32;
  options.min_landmark_observations = 1;
  options.min_pnp_inliers           = 4;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                     kFramePeriodNs,
                     stationaryInterval( 0, kFramePeriodNs ) ) )
                 .status,
             UpdateStatus::kOk );

  VioUpdateResult at_horizon;
  for ( std::int64_t packet = 2; packet <= 11; ++packet )
  {
    at_horizon = estimator.update( makeMeasurementWithObservations(
        packet * kFramePeriodNs,
        stationaryInterval( ( packet - 1 ) * kFramePeriodNs,
                            packet * kFramePeriodNs ),
        {} ) );
    ASSERT_EQ( at_horizon.status, UpdateStatus::kOk )
        << "packet=" << packet << ": " << at_horizon.message;
  }
  ASSERT_TRUE( at_horizon.estimate.has_value() );
  EXPECT_EQ( at_horizon.estimate->m_segment_id, 0U );
  EXPECT_EQ( at_horizon.diagnostics.m_vio.m_nav_states, 11U );
  EXPECT_EQ( at_horizon.diagnostics.m_vio.m_imu_factors, 10U );
  EXPECT_EQ( at_horizon.diagnostics.m_vio.m_bias_rw_factors, 10U );
  EXPECT_EQ( at_horizon.diagnostics.m_vio.m_visual_factors, 10U );
  EXPECT_EQ( at_horizon.diagnostics.m_vio.m_visual_coast_duration_ns,
             options.m_visual_coast_horizon_ns );

  const auto outage = estimator.update( makeMeasurementWithObservations(
      12 * kFramePeriodNs,
      stationaryInterval( 11 * kFramePeriodNs, 12 * kFramePeriodNs ), {} ) );
  EXPECT_EQ( outage.status, UpdateStatus::kVisualOutage );
  EXPECT_FALSE( outage.estimate.has_value() );
  EXPECT_EQ( outage.diagnostics.segment_id, 1U );
  EXPECT_EQ( outage.diagnostics.window_size, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_nav_states, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_imu_factors, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_bias_rw_factors, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_visual_factors, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  ASSERT_TRUE( outage.diagnostics.m_vio.m_completed_segment_id.has_value() );
  EXPECT_EQ( *outage.diagnostics.m_vio.m_completed_segment_id, 0U );

  const auto new_root = estimator.update( makeMeasurement(
      13 * kFramePeriodNs,
      stationaryInterval( 12 * kFramePeriodNs, 13 * kFramePeriodNs ) ) );
  ASSERT_EQ( new_root.status, UpdateStatus::kOk ) << new_root.message;
  ASSERT_TRUE( new_root.estimate.has_value() );
  EXPECT_EQ( new_root.estimate->m_segment_id, 1U );
  EXPECT_EQ( new_root.diagnostics.prior_key, 11U );
  EXPECT_EQ( new_root.diagnostics.window_size, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_nav_states, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_root_prior_sets, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_imu_factors, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_bias_rw_factors, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_visual_factors, 10U );
}

TEST( VioFullState, FactorBearingExactHorizonPreservesSupportFirstCadence )
{
  EstimatorOptions options                = makeOptions();
  options.window_size                     = 32;
  options.min_track_observations_for_seed = 2;
  options.min_landmark_observations       = 2;
  options.min_pnp_inliers                 = 10;
  options.m_visual_coast_horizon_ns       = 4 * kFramePeriodNs;

  std::vector<StereoObservation> low_support = makeUnmappedObservations();
  low_support.pop_back();
  ASSERT_EQ( low_support.size(), 9U );

  VioEstimator recovery( makeCalibration(), makeImuParameters(), options );
  VioEstimator outage( makeCalibration(), makeImuParameters(), options );
  const auto   feed_to_horizon = [ & ]( VioEstimator& estimator ) {
    ASSERT_EQ( estimator
                     .update( makeMeasurement(
                       kFramePeriodNs,
                       stationaryInterval( 0, kFramePeriodNs ) ) )
                     .status,
                 UpdateStatus::kOk );
    const auto supported = estimator.update( makeMeasurement(
        2 * kFramePeriodNs,
        stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ) ) );
    ASSERT_EQ( supported.status, UpdateStatus::kOk ) << supported.message;
    ASSERT_EQ( supported.diagnostics.num_shared, 10U );

    VioUpdateResult at_horizon;
    for ( std::int64_t packet = 3; packet <= 6; ++packet )
    {
      at_horizon = estimator.update( makeMeasurementWithObservations(
          packet * kFramePeriodNs,
          stationaryInterval( ( packet - 1 ) * kFramePeriodNs,
                                packet * kFramePeriodNs ),
          low_support ) );
      ASSERT_EQ( at_horizon.status, UpdateStatus::kOk )
          << "packet=" << packet << ": " << at_horizon.message;
    }
    ASSERT_TRUE( at_horizon.estimate.has_value() );
    EXPECT_EQ( at_horizon.estimate->m_segment_id, 0U );
    EXPECT_EQ( at_horizon.diagnostics.num_shared, low_support.size() );
    EXPECT_EQ( at_horizon.diagnostics.num_retained_observations,
                 low_support.size() );
    EXPECT_EQ( at_horizon.diagnostics.num_current_visual_factors,
                 low_support.size() );
    EXPECT_EQ( at_horizon.diagnostics.m_vio.m_visual_coast_duration_ns,
                 options.m_visual_coast_horizon_ns );
    EXPECT_EQ( at_horizon.diagnostics.unsupported_span_ns,
                 options.m_visual_coast_horizon_ns );
  };

  feed_to_horizon( recovery );
  feed_to_horizon( outage );

  const auto recovered = recovery.update( makeMeasurement(
      7 * kFramePeriodNs,
      stationaryInterval( 6 * kFramePeriodNs, 7 * kFramePeriodNs ) ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_EQ( recovered.estimate->m_segment_id, 0U );
  EXPECT_EQ( recovered.diagnostics.num_shared, 10U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  EXPECT_EQ( recovered.diagnostics.unsupported_span_ns, 0 );

  const auto resumed_coast = recovery.update( makeMeasurementWithObservations(
      8 * kFramePeriodNs,
      stationaryInterval( 7 * kFramePeriodNs, 8 * kFramePeriodNs ),
      low_support ) );
  ASSERT_EQ( resumed_coast.status, UpdateStatus::kOk )
      << resumed_coast.message;
  EXPECT_GT( resumed_coast.diagnostics.num_current_visual_factors, 0U );
  EXPECT_EQ( resumed_coast.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );
  EXPECT_EQ( resumed_coast.diagnostics.unsupported_span_ns,
             kFramePeriodNs );

  const auto discontinuity = recovery.update( makeMeasurementWithObservations(
      9 * kFramePeriodNs,
      MeasurementDiscontinuity{
          .m_t_begin = Timestamp{ 8 * kFramePeriodNs },
          .m_t_end   = Timestamp{ 9 * kFramePeriodNs } },
      {} ) );
  ASSERT_EQ( discontinuity.status, UpdateStatus::kDiscontinuity )
      << discontinuity.message;
  EXPECT_FALSE( discontinuity.estimate.has_value() );
  EXPECT_EQ( discontinuity.diagnostics.unsupported_span_ns, 0 );
  EXPECT_EQ( discontinuity.diagnostics.num_retained_observations, 0U );
  EXPECT_EQ( discontinuity.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( discontinuity.diagnostics.num_current_visual_factors, 0U );
  EXPECT_TRUE( recovery
                   .observationTimestamps( low_support.front().id )
                   .empty() );

  const auto over_horizon = outage.update( makeMeasurementWithObservations(
      7 * kFramePeriodNs,
      stationaryInterval( 6 * kFramePeriodNs, 7 * kFramePeriodNs ),
      low_support ) );
  ASSERT_EQ( over_horizon.status, UpdateStatus::kVisualOutage )
      << over_horizon.message;
  EXPECT_FALSE( over_horizon.estimate.has_value() );
  EXPECT_EQ( over_horizon.diagnostics.segment_id, 1U );
  ASSERT_TRUE(
      over_horizon.diagnostics.m_vio.m_completed_segment_id.has_value() );
  EXPECT_EQ( *over_horizon.diagnostics.m_vio.m_completed_segment_id, 0U );
  EXPECT_EQ( over_horizon.diagnostics.unsupported_span_ns,
             5 * kFramePeriodNs );
  EXPECT_EQ( over_horizon.diagnostics.num_retained_observations, 0U );
  EXPECT_EQ( over_horizon.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( over_horizon.diagnostics.num_current_visual_factors, 0U );

  const auto initializing = outage.update( makeMeasurementWithObservations(
      8 * kFramePeriodNs,
      stationaryInterval( 7 * kFramePeriodNs, 8 * kFramePeriodNs ), {} ) );
  ASSERT_EQ( initializing.status, UpdateStatus::kInitializing )
      << initializing.message;
  EXPECT_EQ( initializing.diagnostics.unsupported_span_ns,
             6 * kFramePeriodNs );
  EXPECT_EQ( initializing.diagnostics.num_retained_observations, 0U );
  EXPECT_EQ( initializing.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( initializing.diagnostics.num_current_visual_factors, 0U );
}

TEST( VioFullState, InvalidRawIntervalPreservesVisualCoastLifecycle )
{
  EstimatorOptions options          = makeOptions();
  options.window_size               = 32;
  options.min_landmark_observations = 1;
  options.min_pnp_inliers           = 4;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  const auto root = makeMeasurement(
      kFramePeriodNs, stationaryInterval( 0, kFramePeriodNs ) );
  ASSERT_EQ( subject.update( root ).status, UpdateStatus::kOk );
  ASSERT_EQ( control.update( root ).status, UpdateStatus::kOk );

  const auto first_coast = makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ), {} );
  ASSERT_EQ( subject.update( first_coast ).status, UpdateStatus::kOk );
  ASSERT_EQ( control.update( first_coast ).status, UpdateStatus::kOk );

  RawImuInterval missing_end_bracket{
      .m_t_begin = Timestamp{ 2 * kFramePeriodNs },
      .m_t_end   = Timestamp{ 3 * kFramePeriodNs },
      .m_samples = { makeImu( 2 * kFramePeriodNs, 0.0 ),
                     makeImu( 125'000'000, 0.0 ),
                     makeImu( 140'000'000, 0.0 ) } };
  const auto invalid = subject.update( makeMeasurementWithObservations(
      3 * kFramePeriodNs, std::move( missing_end_bracket ), {} ) );
  EXPECT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  EXPECT_FALSE( invalid.estimate.has_value() );
  EXPECT_EQ( invalid.diagnostics.m_vio.m_visual_coast_duration_ns,
             kFramePeriodNs );

  const auto next_coast = makeMeasurementWithObservations(
      3 * kFramePeriodNs,
      stationaryInterval( 2 * kFramePeriodNs, 3 * kFramePeriodNs ), {} );
  const auto after_invalid = subject.update( next_coast );
  const auto direct        = control.update( next_coast );
  expectSameEstimate( after_invalid, direct );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_visual_coast_duration_ns,
             direct.diagnostics.m_vio.m_visual_coast_duration_ns );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_visual_factors,
             direct.diagnostics.m_vio.m_visual_factors );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_nav_states,
             direct.diagnostics.m_vio.m_nav_states );
  EXPECT_EQ( after_invalid.diagnostics.m_vio.m_imu_factors,
             direct.diagnostics.m_vio.m_imu_factors );
}

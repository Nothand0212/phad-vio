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
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

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

TEST( VioFullState, AccumulatedRootFailureDoesNotLeakPendingObservations )
{
  EstimatorOptions options        = transactionOptions();
  options.enable_accumulated_seed = true;
  options.min_seed_observations   = 10;
  VioEstimator subject( makeCalibration(), makeImuParameters(), options );
  VioEstimator control( makeCalibration(), makeImuParameters(), options );

  std::vector<StereoObservation> first = makeObservations();
  first.resize( 5U );
  for ( VioEstimator* estimator : { &subject, &control } )
  {
    const auto initializing = estimator->update(
        makeMeasurementWithObservations(
            kFramePeriodNs,
            stationaryInterval( 0, kFramePeriodNs ), first ) );
    ASSERT_EQ( initializing.status, UpdateStatus::kInitializing )
        << initializing.message;
  }

  std::vector<StereoObservation> rejected_only = makeObservations();
  rejected_only.erase( rejected_only.begin(),
                       rejected_only.begin() + 5 );
  for ( StereoObservation& observation : rejected_only )
  {
    observation.id += 100U;
  }
  rejected_only.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  const auto rejected = subject.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ),
      rejected_only ) );
  ASSERT_EQ( rejected.status, UpdateStatus::kRejected )
      << rejected.message;
  EXPECT_FALSE( rejected.estimate.has_value() );
  expectZeroPacketCounters( rejected );

  std::vector<StereoObservation> legal = makeObservations();
  legal.resize( 4U );
  for ( StereoObservation& observation : legal )
  {
    observation.id += 200U;
  }
  const auto after_rejected = subject.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ), legal ) );
  const auto direct         = control.update( makeMeasurementWithObservations(
      2 * kFramePeriodNs,
      stationaryInterval( kFramePeriodNs, 2 * kFramePeriodNs ), legal ) );
  EXPECT_EQ( after_rejected.status, UpdateStatus::kInitializing )
      << after_rejected.message;
  EXPECT_EQ( direct.status, UpdateStatus::kInitializing ) << direct.message;
  for ( const StereoObservation& observation : rejected_only )
  {
    EXPECT_TRUE( subject.observationTimestamps( observation.id ).empty() );
    EXPECT_TRUE( control.observationTimestamps( observation.id ).empty() );
  }
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

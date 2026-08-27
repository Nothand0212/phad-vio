#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
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
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioDiagnostics;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::estimator::VioUpdateResult;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::ImuParameters;
  using phad::sensor::MeasurementDiscontinuity;
  using phad::sensor::RawImuInterval;
  using phad::sensor::RigidTransform;

  constexpr std::int64_t kMs             = 1'000'000;
  constexpr double       kGyroBiasZRadps = 0.17;

  struct ImuNode
  {
    std::int64_t m_timestamp_ns = 0;
    double       m_acc_x_mps2   = 0.0;
    double       m_gyr_z_radps  = 0.0;
  };

  struct NavReference
  {
    Eigen::Isometry3d m_T_W_B = Eigen::Isometry3d::Identity();
    Eigen::Vector3d   m_v_W_B = Eigen::Vector3d::Zero();
  };

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
    options.window_size                     = 3;
    options.min_landmark_observations       = 1;
    options.min_track_observations_for_seed = 1;
    options.min_seed_observations           = 10;
    options.min_shared_landmarks            = 3;
    options.enable_pnp_init                 = false;
    options.enable_outlier_cull             = false;
    options.enable_outlier_reopt            = false;
    return options;
  }

  [[nodiscard]] std::vector<StereoObservation> makeObservations()
  {
    std::vector<StereoObservation> observations;
    observations.reserve( 10U );
    for ( std::uint64_t index = 0; index < 10U; ++index )
    {
      observations.push_back( StereoObservation{
          .id         = index + 1U,
          .left_pixel = Eigen::Vector2d{
              280.0 + 7.0 * static_cast<double>( index ),
              220.0 + 2.0 * static_cast<double>( index % 4U ) },
          .disparity_px = 12.0 } );
    }
    return observations;
  }

  [[nodiscard]] ImuMeasurement makeImu( const ImuNode& node )
  {
    return ImuMeasurement{
        .timestamp = Timestamp{ node.m_timestamp_ns },
        .accel_mps2 =
            std::array<double, 3>{ node.m_acc_x_mps2, 0.0, 9.81 },
        .gyro_radps =
            std::array<double, 3>{ 0.0, 0.0,
                                   node.m_gyr_z_radps + kGyroBiasZRadps } };
  }

  [[nodiscard]] RawImuInterval makeInterval(
      const std::vector<ImuNode>& nodes )
  {
    RawImuInterval interval;
    interval.m_t_begin = Timestamp{ nodes.front().m_timestamp_ns };
    interval.m_t_end   = Timestamp{ nodes.back().m_timestamp_ns };
    interval.m_samples.reserve( nodes.size() );
    for ( const ImuNode& node : nodes )
    {
      interval.m_samples.push_back( makeImu( node ) );
    }
    return interval;
  }

  [[nodiscard]] RawImuInterval makeInterval(
      const std::int64_t begin_ns, const std::int64_t end_ns,
      const std::vector<ImuNode>& nodes )
  {
    RawImuInterval interval = makeInterval( nodes );
    interval.m_t_begin      = Timestamp{ begin_ns };
    interval.m_t_end        = Timestamp{ end_ns };
    return interval;
  }

  [[nodiscard]] RawImuInterval stationaryInterval(
      const std::int64_t begin_ns, const std::int64_t end_ns )
  {
    return makeInterval(
        { ImuNode{ begin_ns, 0.0, 0.0 },
          ImuNode{ begin_ns + ( end_ns - begin_ns ) / 2, 0.0, 0.0 },
          ImuNode{ end_ns, 0.0, 0.0 } } );
  }

  [[nodiscard]] VioMeasurement makeMeasurement(
      const std::int64_t timestamp_ns, RawImuInterval interval,
      const bool visual_supported )
  {
    return VioMeasurement{
        .m_timestamp = Timestamp{ timestamp_ns },
        .m_observations =
            visual_supported ? makeObservations()
                             : std::vector<StereoObservation>{},
        .m_imu = std::move( interval ) };
  }

  [[nodiscard]] NavReference integrateReference(
      const std::vector<ImuNode>& nodes )
  {
    NavReference reference;
    for ( std::size_t index = 1U; index < nodes.size(); ++index )
    {
      const ImuNode& previous = nodes[ index - 1U ];
      const ImuNode& current  = nodes[ index ];
      const double   dt_s     = static_cast<double>( current.m_timestamp_ns -
                                                     previous.m_timestamp_ns ) *
                          1e-9;
      const Eigen::Vector3d acc_B{
          ( previous.m_acc_x_mps2 + current.m_acc_x_mps2 ) * 0.5,
          0.0, 0.0 };
      const double gyr_z =
          ( previous.m_gyr_z_radps + current.m_gyr_z_radps ) * 0.5;
      const Eigen::Vector3d acc_W = reference.m_T_W_B.rotation() * acc_B;
      reference.m_T_W_B.translation() +=
          reference.m_v_W_B * dt_s + 0.5 * acc_W * dt_s * dt_s;
      reference.m_v_W_B += acc_W * dt_s;
      reference.m_T_W_B.linear() *=
          Eigen::AngleAxisd( gyr_z * dt_s, Eigen::Vector3d::UnitZ() )
              .toRotationMatrix();
    }
    return reference;
  }

  void expectCommittedTopology( const VioUpdateResult& result,
                                const std::uint64_t    evictions,
                                const std::uint64_t    reintegrations )
  {
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.estimate.has_value() );
    EXPECT_EQ( result.diagnostics.window_size, 3U );
    EXPECT_EQ( result.diagnostics.prior_key, 0U );
    EXPECT_EQ( result.diagnostics.m_vio.m_nav_states, 3U );
    EXPECT_EQ( result.diagnostics.m_vio.m_imu_factors, 2U );
    EXPECT_EQ( result.diagnostics.m_vio.m_bias_rw_factors, 2U );
    EXPECT_EQ( result.diagnostics.m_vio.m_root_prior_sets, 1U );
    EXPECT_EQ( result.diagnostics.m_vio.m_integration_steps, 9U );
    EXPECT_EQ( result.diagnostics.m_vio.m_integrated_duration_ns,
               150 * kMs );
    EXPECT_EQ( result.diagnostics.m_vio.m_non_keyframe_evictions,
               evictions );
    EXPECT_EQ( result.diagnostics.m_vio.m_imu_reintegrations,
               reintegrations );
  }

  void expectSameCommittedCounters( const VioUpdateResult& failed,
                                    const VioUpdateResult& committed )
  {
    EXPECT_EQ( failed.diagnostics.window_size,
               committed.diagnostics.window_size );
    EXPECT_EQ( failed.diagnostics.prior_key, committed.diagnostics.prior_key );
    EXPECT_EQ( failed.diagnostics.segment_id,
               committed.diagnostics.segment_id );
    EXPECT_EQ( failed.diagnostics.unsupported_span_ns,
               committed.diagnostics.unsupported_span_ns );
    EXPECT_EQ( failed.diagnostics.num_retained_observations, 0U );
    EXPECT_EQ( failed.diagnostics.num_seeded_landmarks, 0U );
    EXPECT_EQ( failed.diagnostics.num_current_visual_factors, 0U );
    const VioDiagnostics& lhs = failed.diagnostics.m_vio;
    const VioDiagnostics& rhs = committed.diagnostics.m_vio;
    EXPECT_EQ( lhs.m_nav_states, rhs.m_nav_states );
    EXPECT_EQ( lhs.m_imu_factors, rhs.m_imu_factors );
    EXPECT_EQ( lhs.m_bias_rw_factors, rhs.m_bias_rw_factors );
    EXPECT_EQ( lhs.m_visual_factors, rhs.m_visual_factors );
    EXPECT_EQ( lhs.m_root_prior_sets, rhs.m_root_prior_sets );
    EXPECT_EQ( lhs.m_integration_steps, rhs.m_integration_steps );
    EXPECT_EQ( lhs.m_integrated_duration_ns, rhs.m_integrated_duration_ns );
    EXPECT_EQ( lhs.m_visual_coast_duration_ns,
               rhs.m_visual_coast_duration_ns );
    EXPECT_EQ( lhs.m_non_keyframe_evictions,
               rhs.m_non_keyframe_evictions );
    EXPECT_EQ( lhs.m_imu_reintegrations, rhs.m_imu_reintegrations );
    EXPECT_TRUE( lhs.m_acc_cov_diag.isApprox( rhs.m_acc_cov_diag, 0.0 ) );
    EXPECT_TRUE( lhs.m_gyr_cov_diag.isApprox( rhs.m_gyr_cov_diag, 0.0 ) );
    EXPECT_TRUE( lhs.m_integration_cov_diag.isApprox(
        rhs.m_integration_cov_diag, 0.0 ) );
    EXPECT_TRUE( lhs.m_bias_rw_sigmas.isApprox( rhs.m_bias_rw_sigmas, 0.0 ) );
    EXPECT_EQ( lhs.m_completed_segment_id, rhs.m_completed_segment_id );
  }

  const std::vector<ImuNode> kFirstInterval{
      ImuNode{ 50 * kMs, 0.2, 0.04 }, ImuNode{ 63 * kMs, 0.5, -0.02 },
      ImuNode{ 91 * kMs, -0.1, 0.07 }, ImuNode{ 100 * kMs, 0.8, 0.01 } };
  const std::vector<ImuNode> kSecondInterval{
      ImuNode{ 100 * kMs, 0.8, 0.01 }, ImuNode{ 112 * kMs, 1.1, -0.03 },
      ImuNode{ 139 * kMs, -0.4, 0.05 }, ImuNode{ 150 * kMs, 0.3, 0.02 } };
  const std::vector<ImuNode> kThirdInterval{
      ImuNode{ 150 * kMs, 0.3, 0.02 }, ImuNode{ 167 * kMs, 0.9, 0.06 },
      ImuNode{ 188 * kMs, -0.2, -0.04 }, ImuNode{ 200 * kMs, 0.7, 0.03 } };
  const std::vector<ImuNode> kFirstRawSupport{
      ImuNode{ 45 * kMs, -0.3, 0.08 }, kFirstInterval[ 0 ],
      kFirstInterval[ 1 ], kFirstInterval[ 2 ], kFirstInterval[ 3 ],
      ImuNode{ 105 * kMs, 0.6, -0.05 } };
  const std::vector<ImuNode> kSecondRawSupport{
      ImuNode{ 95 * kMs, 0.1, -0.06 }, kSecondInterval[ 0 ],
      kSecondInterval[ 1 ], kSecondInterval[ 2 ], kSecondInterval[ 3 ],
      ImuNode{ 155 * kMs, 0.5, 0.07 } };
  const std::vector<ImuNode> kThirdRawSupport{
      ImuNode{ 145 * kMs, -0.5, 0.04 }, kThirdInterval[ 0 ],
      kThirdInterval[ 1 ], kThirdInterval[ 2 ], kThirdInterval[ 3 ],
      ImuNode{ 205 * kMs, 0.2, -0.01 } };

}  // namespace

TEST( VioEviction, ReintegratesInteriorNonKeyframeFromRawIntervals )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );

  const VioUpdateResult root = estimator.update(
      makeMeasurement( 50 * kMs, stationaryInterval( 0, 50 * kMs ), true ),
      true );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.estimate.has_value() );
  EXPECT_TRUE( root.estimate->m_bias.m_gyr_radps.isApprox(
      Eigen::Vector3d{ 0.0, 0.0, kGyroBiasZRadps }, 1e-12 ) );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              100 * kMs,
                              makeInterval( 50 * kMs, 100 * kMs,
                                            kFirstRawSupport ),
                              false ),
                          false )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              150 * kMs,
                              makeInterval( 100 * kMs, 150 * kMs,
                                            kSecondRawSupport ),
                              false ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const VioUpdateResult reintegrated = estimator.update(
      makeMeasurement(
          200 * kMs,
          makeInterval( 150 * kMs, 200 * kMs, kThirdRawSupport ), false ),
      true );
  expectCommittedTopology( reintegrated, 1U, 1U );

  std::vector<ImuNode> all_nodes = kFirstInterval;
  all_nodes.insert( all_nodes.end(), kSecondInterval.begin() + 1,
                    kSecondInterval.end() );
  all_nodes.insert( all_nodes.end(), kThirdInterval.begin() + 1,
                    kThirdInterval.end() );
  const NavReference expected = integrateReference( all_nodes );
  EXPECT_TRUE( reintegrated.estimate->T_W_B.matrix().isApprox(
      expected.m_T_W_B.matrix(), 1e-8 ) );
  EXPECT_TRUE( reintegrated.estimate->m_v_W_B.isApprox( expected.m_v_W_B,
                                                        1e-8 ) );
  EXPECT_TRUE( reintegrated.estimate->m_bias.m_acc_mps2.isZero( 1e-8 ) );
  EXPECT_TRUE( reintegrated.estimate->m_bias.m_gyr_radps.isApprox(
      Eigen::Vector3d{ 0.0, 0.0, kGyroBiasZRadps }, 1e-8 ) );

  const std::vector<ImuNode> next_interval{
      ImuNode{ 200 * kMs, 0.7, 0.03 }, ImuNode{ 221 * kMs, -0.2, 0.01 },
      ImuNode{ 250 * kMs, 0.4, -0.02 } };
  const auto next = estimator.update(
      makeMeasurement( 250 * kMs, makeInterval( next_interval ), false ),
      true );
  ASSERT_EQ( next.status, UpdateStatus::kOk ) << next.message;
  ASSERT_TRUE( next.estimate.has_value() );
  EXPECT_EQ( next.estimate->timestamp, Timestamp{ 250 * kMs } );
  EXPECT_EQ( next.diagnostics.window_size, 3U );
  EXPECT_EQ( next.diagnostics.m_vio.m_nav_states, 3U );
  EXPECT_EQ( next.diagnostics.m_vio.m_imu_factors, 2U );
  EXPECT_EQ( next.diagnostics.m_vio.m_bias_rw_factors, 2U );
  EXPECT_EQ( next.diagnostics.m_vio.m_non_keyframe_evictions, 1U );
  EXPECT_EQ( next.diagnostics.m_vio.m_imu_reintegrations, 1U );
}

TEST( VioEviction, SignedZeroEndpointMismatchRollsBackAtomically )
{
  VioEstimator subject( makeCalibration(), makeImuParameters(),
                        makeOptions() );
  VioEstimator control( makeCalibration(), makeImuParameters(),
                        makeOptions() );
  for ( VioEstimator* estimator : { &subject, &control } )
  {
    ASSERT_EQ( estimator
                   ->update( makeMeasurement(
                                 50 * kMs,
                                 stationaryInterval( 0, 50 * kMs ), true ),
                             true )
                   .status,
               UpdateStatus::kOk );
  }

  const std::vector<ImuNode> predecessor_interval{
      ImuNode{ 50 * kMs, 0.0, 0.0 }, ImuNode{ 73 * kMs, 0.4, 0.02 },
      ImuNode{ 100 * kMs, +0.0, 0.0 } };
  for ( VioEstimator* estimator : { &subject, &control } )
  {
    ASSERT_EQ( estimator
                   ->update( makeMeasurement(
                                 100 * kMs,
                                 makeInterval( predecessor_interval ), true ),
                             false )
                   .status,
               UpdateStatus::kOk );
  }

  const std::vector<ImuNode> successor_interval{
      ImuNode{ 100 * kMs, -0.0, 0.0 }, ImuNode{ 127 * kMs, -0.3, -0.01 },
      ImuNode{ 150 * kMs, 0.2, 0.03 } };
  const VioUpdateResult committed = subject.update(
      makeMeasurement( 150 * kMs, makeInterval( successor_interval ), false ),
      true );
  const VioUpdateResult control_committed = control.update(
      makeMeasurement( 150 * kMs, makeInterval( successor_interval ), false ),
      true );
  ASSERT_EQ( committed.status, UpdateStatus::kOk ) << committed.message;
  ASSERT_EQ( control_committed.status, UpdateStatus::kOk )
      << control_committed.message;
  EXPECT_EQ( committed.diagnostics.window_size, 3U );
  EXPECT_EQ( committed.diagnostics.prior_key, 0U );
  EXPECT_EQ( committed.diagnostics.m_vio.m_non_keyframe_evictions, 0U );
  EXPECT_EQ( committed.diagnostics.m_vio.m_imu_reintegrations, 0U );
  EXPECT_EQ( committed.diagnostics.unsupported_span_ns, 50 * kMs );

  const std::vector<ImuNode> trigger_interval{
      ImuNode{ 150 * kMs, 0.2, 0.03 }, ImuNode{ 179 * kMs, 0.1, -0.02 },
      ImuNode{ 200 * kMs, 0.5, 0.01 } };
  constexpr LandmarkId kFailedOnlyId = 9'004U;
  VioMeasurement       failing       = makeMeasurement(
      200 * kMs, makeInterval( trigger_interval ), true );
  StereoObservation failed_only = makeObservations().front();
  failed_only.id                = kFailedOnlyId;
  failing.m_observations.push_back( failed_only );
  const VioUpdateResult failed = subject.update( failing, true );
  EXPECT_EQ( failed.status, UpdateStatus::kFailed );
  EXPECT_FALSE( failed.estimate.has_value() );
  expectSameCommittedCounters( failed, committed );
  EXPECT_TRUE( subject.observationTimestamps( kFailedOnlyId ).empty() );

  const VioMeasurement discontinuity_measurement{
      .m_timestamp    = Timestamp{ 200 * kMs },
      .m_observations = {},
      .m_imu          = MeasurementDiscontinuity{
                   .m_t_begin = Timestamp{ 150 * kMs },
                   .m_t_end   = Timestamp{ 200 * kMs } } };
  const VioUpdateResult discontinuity =
      subject.update( discontinuity_measurement );
  const VioUpdateResult control_discontinuity =
      control.update( discontinuity_measurement );
  ASSERT_EQ( discontinuity.status, UpdateStatus::kDiscontinuity )
      << discontinuity.message;
  ASSERT_EQ( control_discontinuity.status, UpdateStatus::kDiscontinuity )
      << control_discontinuity.message;
  EXPECT_EQ( discontinuity.diagnostics.unsupported_span_ns,
             control_discontinuity.diagnostics.unsupported_span_ns );

  const VioMeasurement root_measurement = makeMeasurement(
      250 * kMs, stationaryInterval( 200 * kMs, 250 * kMs ), true );
  const VioUpdateResult new_root     = subject.update( root_measurement,
                                                       true );
  const VioUpdateResult control_root = control.update( root_measurement,
                                                       true );
  ASSERT_EQ( new_root.status, UpdateStatus::kOk ) << new_root.message;
  ASSERT_EQ( control_root.status, UpdateStatus::kOk )
      << control_root.message;
  ASSERT_TRUE( new_root.estimate.has_value() );
  ASSERT_TRUE( control_root.estimate.has_value() );
  EXPECT_TRUE( new_root.estimate->T_W_B.matrix().isApprox(
      control_root.estimate->T_W_B.matrix(), 1e-12 ) );
  EXPECT_TRUE( new_root.estimate->m_v_W_B.isApprox(
      control_root.estimate->m_v_W_B, 1e-12 ) );
  EXPECT_EQ( new_root.diagnostics.prior_key, 3U );
  EXPECT_EQ( new_root.diagnostics.window_size, 1U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_non_keyframe_evictions, 0U );
  EXPECT_EQ( new_root.diagnostics.m_vio.m_imu_reintegrations, 0U );
}

TEST( VioEviction, NewSegmentDoesNotReuseCompletedSegmentProvenance )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              50 * kMs, stationaryInterval( 0, 50 * kMs ), true ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const std::vector<ImuNode> old_first{
      ImuNode{ 50 * kMs, 0.0, 0.0 }, ImuNode{ 76 * kMs, 0.2, 0.01 },
      ImuNode{ 100 * kMs, +0.0, 0.0 } };
  const std::vector<ImuNode> old_second{
      ImuNode{ 100 * kMs, -0.0, 0.0 }, ImuNode{ 124 * kMs, -0.1, 0.02 },
      ImuNode{ 150 * kMs, 0.3, -0.01 } };
  ASSERT_EQ( estimator
                 .update( makeMeasurement( 100 * kMs,
                                           makeInterval( old_first ), false ),
                          false )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement( 150 * kMs,
                                           makeInterval( old_second ), false ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const auto discontinuity = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ 200 * kMs },
      .m_observations = {},
      .m_imu          = MeasurementDiscontinuity{
                   .m_t_begin = Timestamp{ 150 * kMs },
                   .m_t_end   = Timestamp{ 200 * kMs } } } );
  ASSERT_EQ( discontinuity.status, UpdateStatus::kDiscontinuity )
      << discontinuity.message;

  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              250 * kMs,
                              stationaryInterval( 200 * kMs, 250 * kMs ), true ),
                          true )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              300 * kMs,
                              stationaryInterval( 250 * kMs, 300 * kMs ), false ),
                          false )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              350 * kMs,
                              stationaryInterval( 300 * kMs, 350 * kMs ), false ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const VioUpdateResult reintegrated = estimator.update(
      makeMeasurement( 400 * kMs,
                       stationaryInterval( 350 * kMs, 400 * kMs ), false ),
      true );
  ASSERT_EQ( reintegrated.status, UpdateStatus::kOk )
      << reintegrated.message;
  ASSERT_TRUE( reintegrated.estimate.has_value() );
  EXPECT_EQ( reintegrated.estimate->m_segment_id, 1U );
  EXPECT_EQ( reintegrated.diagnostics.segment_id, 1U );
  EXPECT_EQ( reintegrated.diagnostics.prior_key, 3U );
  EXPECT_EQ( reintegrated.diagnostics.window_size, 3U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_nav_states, 3U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_imu_factors, 2U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_bias_rw_factors, 2U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_non_keyframe_evictions, 1U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_imu_reintegrations, 1U );
}

TEST( VioEviction, VisualOutageDropsCompletedSegmentProvenance )
{
  EstimatorOptions options          = makeOptions();
  options.m_visual_coast_horizon_ns = 100 * kMs;
  VioEstimator estimator( makeCalibration(), makeImuParameters(), options );

  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              50 * kMs, stationaryInterval( 0, 50 * kMs ), true ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const std::vector<ImuNode> old_first{
      ImuNode{ 50 * kMs, 0.0, 0.0 }, ImuNode{ 76 * kMs, 0.2, 0.01 },
      ImuNode{ 100 * kMs, +0.0, 0.0 } };
  const std::vector<ImuNode> old_second{
      ImuNode{ 100 * kMs, -0.0, 0.0 }, ImuNode{ 124 * kMs, -0.1, 0.02 },
      ImuNode{ 150 * kMs, 0.3, -0.01 } };
  ASSERT_EQ( estimator
                 .update( makeMeasurement( 100 * kMs,
                                           makeInterval( old_first ), false ),
                          false )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement( 150 * kMs,
                                           makeInterval( old_second ), false ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const VioUpdateResult outage = estimator.update(
      makeMeasurement( 200 * kMs,
                       stationaryInterval( 150 * kMs, 200 * kMs ), false ),
      true );
  ASSERT_EQ( outage.status, UpdateStatus::kVisualOutage ) << outage.message;
  EXPECT_FALSE( outage.estimate.has_value() );
  EXPECT_EQ( outage.diagnostics.segment_id, 1U );
  EXPECT_EQ( outage.diagnostics.window_size, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_non_keyframe_evictions, 0U );
  EXPECT_EQ( outage.diagnostics.m_vio.m_imu_reintegrations, 0U );

  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              250 * kMs,
                              stationaryInterval( 200 * kMs, 250 * kMs ), true ),
                          true )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              300 * kMs,
                              stationaryInterval( 250 * kMs, 300 * kMs ), false ),
                          false )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeMeasurement(
                              350 * kMs,
                              stationaryInterval( 300 * kMs, 350 * kMs ), true ),
                          true )
                 .status,
             UpdateStatus::kOk );

  const VioUpdateResult reintegrated = estimator.update(
      makeMeasurement( 400 * kMs,
                       stationaryInterval( 350 * kMs, 400 * kMs ), true ),
      true );
  ASSERT_EQ( reintegrated.status, UpdateStatus::kOk )
      << reintegrated.message;
  ASSERT_TRUE( reintegrated.estimate.has_value() );
  EXPECT_EQ( reintegrated.estimate->m_segment_id, 1U );
  EXPECT_EQ( reintegrated.diagnostics.prior_key, 3U );
  EXPECT_EQ( reintegrated.diagnostics.window_size, 3U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_nav_states, 3U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_imu_factors, 2U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_bias_rw_factors, 2U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_non_keyframe_evictions, 1U );
  EXPECT_EQ( reintegrated.diagnostics.m_vio.m_imu_reintegrations, 1U );
}

TEST( VioEviction, AllKeyframeWindowRebasesFrontWithoutReintegration )
{
  VioEstimator estimator( makeCalibration(), makeImuParameters(),
                          makeOptions() );

  VioUpdateResult result;
  for ( std::int64_t frame = 1; frame <= 5; ++frame )
  {
    result = estimator.update(
        makeMeasurement(
            frame * 50 * kMs,
            stationaryInterval( ( frame - 1 ) * 50 * kMs,
                                frame * 50 * kMs ),
            true ),
        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame=" << frame << ": " << result.message;
  }

  ASSERT_TRUE( result.estimate.has_value() );
  EXPECT_EQ( result.estimate->timestamp, Timestamp{ 250 * kMs } );
  EXPECT_EQ( result.diagnostics.window_size, 3U );
  EXPECT_EQ( result.diagnostics.prior_key, 2U );
  EXPECT_EQ( result.diagnostics.m_vio.m_nav_states, 3U );
  EXPECT_EQ( result.diagnostics.m_vio.m_imu_factors, 2U );
  EXPECT_EQ( result.diagnostics.m_vio.m_bias_rw_factors, 2U );
  EXPECT_EQ( result.diagnostics.m_vio.m_non_keyframe_evictions, 0U );
  EXPECT_EQ( result.diagnostics.m_vio.m_imu_reintegrations, 0U );
}

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::GyroMode;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;

  const std::vector<Eigen::Vector3d> kLandmarks{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.25, 0.15, 4.2 },
      { -0.2, -0.15, 5.8 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
  };

  [[nodiscard]] RectifiedStereoCalibration makeCalibration()
  {
    auto rigid = phad::sensor::RigidTransform::create(
                     Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( rigid ) )
        .value();
  }

  [[nodiscard]] phad::sensor::ImuParameters makeImuParameters()
  {
    return phad::sensor::ImuParameters::create(
               200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 )
        .value();
  }

  [[nodiscard]] phad::sensor::ImuParameters makeImuParameters(
      const double gyr_nd )
  {
    return phad::sensor::ImuParameters::create(
               200.0, 0.002, gyr_nd, 0.003, 1.9393e-05 )
        .value();
  }

  [[nodiscard]] KeyframeMeasurement makeFrame(
      std::int64_t             timestamp_ns,
      const Eigen::Isometry3d& T_W_B     = Eigen::Isometry3d::Identity(),
      phad::common::LandmarkId id_offset = 0 )
  {
    const RectifiedStereoCalibration calibration = makeCalibration();
    KeyframeMeasurement              measurement;
    measurement.timestamp = phad::common::Timestamp{ timestamp_ns };
    for ( std::size_t index = 0; index < kLandmarks.size(); ++index )
    {
      const Eigen::Vector3d point = T_W_B.inverse() * kLandmarks[ index ];
      const double          u     = calibration.fxPixels() * point.x() / point.z() +
                       calibration.cxPixels();
      const double v = calibration.fyPixels() * point.y() / point.z() +
                       calibration.cyPixels();
      const double disparity =
          calibration.fxPixels() * calibration.baselineM() / point.z();
      measurement.observations.push_back( StereoObservation{
          .id = id_offset +
                static_cast<phad::common::LandmarkId>( index + 1U ),
          .left_pixel   = Eigen::Vector2d( u, v ),
          .disparity_px = disparity,
      } );
    }
    return measurement;
  }

  [[nodiscard]] Eigen::Matrix3d expRotation( const Eigen::Vector3d& tangent )
  {
    const double angle = tangent.norm();
    if ( angle == 0.0 )
    {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd( angle, tangent / angle ).toRotationMatrix();
  }

  [[nodiscard]] EstimatorOptions gyroOptions()
  {
    EstimatorOptions options;
    options.gyro_mode                       = GyroMode::kShadow;
    options.min_track_observations_for_seed = 1;
    return options;
  }

  [[nodiscard]] std::vector<phad::sensor::ImuMeasurement> makeSegment(
      std::int64_t t_i_ns, std::int64_t t_j_ns,
      const Eigen::Vector3d& omega = Eigen::Vector3d::Zero() )
  {
    return {
        { .timestamp  = phad::common::Timestamp{ t_i_ns },
          .gyro_radps = { omega.x(), omega.y(), omega.z() } },
        { .timestamp  = phad::common::Timestamp{ t_j_ns },
          .gyro_radps = { omega.x(), omega.y(), omega.z() } },
    };
  }

}  // namespace

TEST( StereoVoGyroSegment, MalformedSegmentFailsBeforeVisualStateChanges )
{
  const auto calibration = makeCalibration();
  const auto imu         = makeImuParameters();
  const auto options     = gyroOptions();

  StereoVoEstimator                         estimator( calibration, imu, options );
  KeyframeMeasurement                       malformed = makeFrame( 100'000'000 );
  std::vector<phad::sensor::ImuMeasurement> samples{
      { .timestamp  = phad::common::Timestamp{ 90'000'000 },
        .gyro_radps = { 0.0, 0.0, 0.0 } },
      { .timestamp  = phad::common::Timestamp{ 100'000'000 },
        .gyro_radps = { std::numeric_limits<double>::quiet_NaN(), 0.0,
                        0.0 } },
  };
  malformed.imu = phad::estimator::ImuSegmentView{
      .t_prev  = phad::common::Timestamp{ 90'000'000 },
      .samples = samples,
      .gap     = false,
  };

  const auto failed = estimator.update( malformed );
  EXPECT_EQ( failed.status, UpdateStatus::kFailed );
  EXPECT_EQ( failed.message.rfind( "gyro segment:", 0 ), 0U );
  ASSERT_TRUE( failed.diagnostics.gyro.has_value() );
  EXPECT_FALSE( failed.diagnostics.gyro->edge_valid );

  KeyframeMeasurement valid = makeFrame( 100'000'000 );
  valid.imu.t_prev          = valid.timestamp;
  valid.imu.gap             = true;
  const auto accepted       = estimator.update( valid );
  ASSERT_EQ( accepted.status, UpdateStatus::kOk ) << accepted.message;
  EXPECT_EQ( accepted.diagnostics.window_size, 1U );
}

TEST( StereoVoGyroSegment, RejectsMalformedContractMatrix )
{
  struct Case
  {
    const char*                               name;
    phad::common::Timestamp                   t_prev;
    std::vector<phad::sensor::ImuMeasurement> samples;
  };

  auto too_few = makeSegment( 100'000'000, 200'000'000 );
  too_few.pop_back();
  auto first_mismatch = makeSegment( 110'000'000, 200'000'000 );
  auto last_mismatch  = makeSegment( 100'000'000, 190'000'000 );
  auto duplicate      = makeSegment( 100'000'000, 200'000'000 );
  duplicate.insert( duplicate.begin() + 1, duplicate.front() );
  auto reverse = makeSegment( 100'000'000, 200'000'000 );
  reverse.insert(
      reverse.begin() + 1,
      phad::sensor::ImuMeasurement{
          .timestamp  = phad::common::Timestamp{ 210'000'000 },
          .gyro_radps = { 0.0, 0.0, 0.0 } } );
  const double nan                 = std::numeric_limits<double>::quiet_NaN();
  auto         nonfinite           = makeSegment( 100'000'000, 200'000'000 );
  nonfinite.back().gyro_radps[ 1 ] = nan;
  const std::vector<Case> cases{
      { "too few samples", phad::common::Timestamp{ 100'000'000 },
        too_few },
      { "first endpoint", phad::common::Timestamp{ 100'000'000 },
        first_mismatch },
      { "last endpoint", phad::common::Timestamp{ 100'000'000 },
        last_mismatch },
      { "duplicate timestamp", phad::common::Timestamp{ 100'000'000 },
        duplicate },
      { "reverse timestamp", phad::common::Timestamp{ 100'000'000 },
        reverse },
      { "non-finite gyro", phad::common::Timestamp{ 100'000'000 },
        nonfinite },
  };

  for ( const Case& test_case : cases )
  {
    SCOPED_TRACE( test_case.name );
    StereoVoEstimator   estimator( makeCalibration(), makeImuParameters(),
                                   gyroOptions() );
    KeyframeMeasurement first = makeFrame( 100'000'000 );
    first.imu.t_prev          = first.timestamp;
    first.imu.gap             = false;
    ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

    KeyframeMeasurement malformed = makeFrame( 200'000'000 );
    malformed.imu                 = { test_case.t_prev, test_case.samples, false };
    const auto failed             = estimator.update( malformed );
    EXPECT_EQ( failed.status, UpdateStatus::kFailed );
    EXPECT_EQ( failed.message.rfind( "gyro segment:", 0 ), 0U );

    auto                valid_samples = makeSegment( 100'000'000, 200'000'000 );
    KeyframeMeasurement valid         = makeFrame( 200'000'000 );
    valid.imu                         = { phad::common::Timestamp{ 100'000'000 }, valid_samples,
                                          false };
    EXPECT_EQ( estimator.update( valid ).status, UpdateStatus::kOk );
  }
}

TEST( StereoVoGyroSegment, IgnoresAccelAndAcceptsGapAsMissingEvidence )
{
  StereoVoEstimator   estimator( makeCalibration(), makeImuParameters(),
                                 gyroOptions() );
  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto samples = makeSegment( 100'000'000, 200'000'000 );
  samples.front().accel_mps2[ 0 ] =
      std::numeric_limits<double>::quiet_NaN();
  KeyframeMeasurement accel_nonfinite = makeFrame( 200'000'000 );
  accel_nonfinite.imu                 = { phad::common::Timestamp{ 100'000'000 }, samples,
                                          false };
  const auto accel_result             = estimator.update( accel_nonfinite );
  ASSERT_EQ( accel_result.status, UpdateStatus::kOk );
  ASSERT_TRUE( accel_result.diagnostics.gyro.has_value() );
  EXPECT_TRUE( accel_result.diagnostics.gyro->edge_valid );

  auto malformed_gap = makeSegment( 300'000'000, 250'000'000 );
  malformed_gap.front().gyro_radps[ 2 ] =
      std::numeric_limits<double>::quiet_NaN();
  KeyframeMeasurement gap = makeFrame( 300'000'000 );
  gap.imu                 = { phad::common::Timestamp{ 200'000'000 }, malformed_gap, true };
  const auto gap_result   = estimator.update( gap );
  ASSERT_EQ( gap_result.status, UpdateStatus::kOk );
  ASSERT_TRUE( gap_result.diagnostics.gyro.has_value() );
  EXPECT_TRUE( gap_result.diagnostics.gyro->imu_gap );
  EXPECT_FALSE( gap_result.diagnostics.gyro->edge_valid );
}

TEST( StereoVoGyroLedger, AcceptedPoseStoresValidIncomingEdge )
{
  StereoVoEstimator estimator( makeCalibration(), makeImuParameters(),
                               gyroOptions() );

  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  std::vector<phad::sensor::ImuMeasurement> samples =
      makeSegment( 100'000'000, 200'000'000 );
  KeyframeMeasurement second = makeFrame( 200'000'000 );
  second.imu                 = phad::estimator::ImuSegmentView{
                      .t_prev  = phad::common::Timestamp{ 100'000'000 },
                      .samples = samples,
                      .gap     = false,
  };

  const auto result = estimator.update( second );
  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.diagnostics.gyro.has_value() );
  const auto& gyro = *result.diagnostics.gyro;
  EXPECT_TRUE( gyro.edge_valid );
  EXPECT_FALSE( gyro.imu_gap );
  EXPECT_EQ( gyro.t_i_ns, 100'000'000 );
  EXPECT_EQ( gyro.t_j_ns, 200'000'000 );
  EXPECT_EQ( gyro.imu_samples, 2U );
  ASSERT_TRUE( gyro.imu_dt_s.has_value() );
  EXPECT_DOUBLE_EQ( *gyro.imu_dt_s, 0.1 );
  EXPECT_DOUBLE_EQ( gyro.align_support_s, 0.1 );
  EXPECT_EQ( gyro.align_edges, 1U );
  EXPECT_FALSE( gyro.align_ready );
}

TEST( StereoVoGyroLedger, RejectedIntervalsMergeIntoNextAcceptedEdge )
{
  StereoVoEstimator   estimator( makeCalibration(), makeImuParameters(),
                                 gyroOptions() );
  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = true;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto                samples_2 = makeSegment( 100'000'000, 200'000'000 );
  KeyframeMeasurement second    = makeFrame( 200'000'000 );
  second.imu                    = { phad::common::Timestamp{ 100'000'000 }, samples_2, false };
  ASSERT_EQ( estimator.update( second ).status, UpdateStatus::kOk );

  auto                samples_3 = makeSegment( 200'000'000, 250'000'000 );
  KeyframeMeasurement rejected  = makeFrame( 250'000'000 );
  rejected.observations.resize( 5U );
  rejected.imu               = { phad::common::Timestamp{ 200'000'000 }, samples_3, false };
  const auto rejected_result = estimator.update( rejected, false );
  ASSERT_EQ( rejected_result.status, UpdateStatus::kRejected );

  auto                samples_4 = makeSegment( 250'000'000, 300'000'000 );
  KeyframeMeasurement accepted  = makeFrame( 300'000'000 );
  accepted.imu                  = { phad::common::Timestamp{ 250'000'000 }, samples_4, false };
  const auto result             = estimator.update( accepted );
  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.diagnostics.gyro.has_value() );
  const auto& gyro = *result.diagnostics.gyro;
  EXPECT_TRUE( gyro.edge_valid );
  EXPECT_EQ( gyro.t_i_ns, 200'000'000 );
  EXPECT_EQ( gyro.t_j_ns, 300'000'000 );
  EXPECT_EQ( gyro.imu_samples, 3U );
  ASSERT_TRUE( gyro.imu_dt_s.has_value() );
  EXPECT_DOUBLE_EQ( *gyro.imu_dt_s, 0.1 );
  EXPECT_DOUBLE_EQ( gyro.align_support_s, 0.2 );
  EXPECT_EQ( gyro.align_edges, 2U );
}

TEST( StereoVoGyroLedger, GapInvalidatesOneAcceptedEdgeThenRecovers )
{
  StereoVoEstimator   estimator( makeCalibration(), makeImuParameters(),
                                 gyroOptions() );
  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = true;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto                samples_2 = makeSegment( 100'000'000, 200'000'000 );
  KeyframeMeasurement second    = makeFrame( 200'000'000 );
  second.imu                    = { phad::common::Timestamp{ 100'000'000 }, samples_2, false };
  ASSERT_EQ( estimator.update( second ).status, UpdateStatus::kOk );

  KeyframeMeasurement rejected = makeFrame( 250'000'000 );
  rejected.observations.resize( 5U );
  rejected.imu.t_prev = phad::common::Timestamp{ 200'000'000 };
  rejected.imu.gap    = true;
  ASSERT_EQ( estimator.update( rejected, false ).status,
             UpdateStatus::kRejected );

  auto                samples_4 = makeSegment( 250'000'000, 300'000'000 );
  KeyframeMeasurement after_gap = makeFrame( 300'000'000 );
  after_gap.imu                 = { phad::common::Timestamp{ 250'000'000 }, samples_4,
                                    false };
  const auto gap_result         = estimator.update( after_gap );
  ASSERT_EQ( gap_result.status, UpdateStatus::kOk );
  ASSERT_TRUE( gap_result.diagnostics.gyro.has_value() );
  EXPECT_TRUE( gap_result.diagnostics.gyro->imu_gap );
  EXPECT_FALSE( gap_result.diagnostics.gyro->edge_valid );
  EXPECT_DOUBLE_EQ( gap_result.diagnostics.gyro->align_support_s, 0.1 );

  auto                samples_5 = makeSegment( 300'000'000, 400'000'000 );
  KeyframeMeasurement recovered = makeFrame( 400'000'000 );
  recovered.imu                 = { phad::common::Timestamp{ 300'000'000 }, samples_5,
                                    false };
  const auto recovered_result   = estimator.update( recovered );
  ASSERT_EQ( recovered_result.status, UpdateStatus::kOk );
  ASSERT_TRUE( recovered_result.diagnostics.gyro.has_value() );
  EXPECT_FALSE( recovered_result.diagnostics.gyro->imu_gap );
  EXPECT_TRUE( recovered_result.diagnostics.gyro->edge_valid );
  EXPECT_DOUBLE_EQ( recovered_result.diagnostics.gyro->align_support_s,
                    0.2 );
}

TEST( StereoVoGyroLedger, ReanchorDoesNotCreateCrossSegmentEdge )
{
  StereoVoEstimator   estimator( makeCalibration(), makeImuParameters(),
                                 gyroOptions() );
  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto                samples_2 = makeSegment( 100'000'000, 200'000'000 );
  KeyframeMeasurement second    = makeFrame( 200'000'000 );
  second.imu                    = { phad::common::Timestamp{ 100'000'000 }, samples_2, false };
  ASSERT_EQ( estimator.update( second ).status, UpdateStatus::kOk );

  auto                samples_3 = makeSegment( 200'000'000, 300'000'000 );
  KeyframeMeasurement reanchor  = makeFrame(
      300'000'000, Eigen::Isometry3d::Identity(), 1'000 );
  reanchor.imu               = { phad::common::Timestamp{ 200'000'000 }, samples_3,
                                 false };
  const auto reanchor_result = estimator.update( reanchor );
  ASSERT_EQ( reanchor_result.status, UpdateStatus::kOk );
  EXPECT_EQ( reanchor_result.diagnostics.segment_id, 1U );
  ASSERT_TRUE( reanchor_result.diagnostics.gyro.has_value() );
  EXPECT_FALSE( reanchor_result.diagnostics.gyro->edge_valid );
  EXPECT_DOUBLE_EQ( reanchor_result.diagnostics.gyro->align_support_s, 0.1 );

  auto                samples_4 = makeSegment( 300'000'000, 400'000'000 );
  KeyframeMeasurement after     = makeFrame(
      400'000'000, Eigen::Isometry3d::Identity(), 1'000 );
  after.imu               = { phad::common::Timestamp{ 300'000'000 }, samples_4, false };
  const auto after_result = estimator.update( after );
  ASSERT_EQ( after_result.status, UpdateStatus::kOk );
  ASSERT_TRUE( after_result.diagnostics.gyro.has_value() );
  EXPECT_TRUE( after_result.diagnostics.gyro->edge_valid );
  EXPECT_EQ( after_result.diagnostics.segment_id, 1U );
  EXPECT_DOUBLE_EQ( after_result.diagnostics.gyro->align_support_s, 0.2 );
}

TEST( StereoVoGyroAlignment, RecoversSharedAbsoluteBiasAndFreezes )
{
  EstimatorOptions options    = gyroOptions();
  options.gyro_align_window_s = 0.3;
  StereoVoEstimator estimator( makeCalibration(), makeImuParameters(),
                               options );

  const Eigen::Vector3d              bias( 0.01, -0.02, 0.03 );
  const std::vector<Eigen::Vector3d> true_omegas{
      { 0.10, 0.20, -0.10 },
      { -0.20, 0.15, 0.05 },
      { 0.05, -0.10, 0.20 },
      { 0.12, 0.04, -0.08 },
  };

  Eigen::Isometry3d   pose  = Eigen::Isometry3d::Identity();
  KeyframeMeasurement first = makeFrame( 100'000'000, pose );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  phad::estimator::VioUpdateResult last;
  for ( std::size_t index = 0; index < true_omegas.size(); ++index )
  {
    pose.linear() = pose.linear() * expRotation( true_omegas[ index ] * 0.1 );
    const std::int64_t t_i =
        100'000'000 + static_cast<std::int64_t>( index ) * 100'000'000;
    const std::int64_t  t_j     = t_i + 100'000'000;
    auto                samples = makeSegment( t_i, t_j, true_omegas[ index ] + bias );
    KeyframeMeasurement frame   = makeFrame( t_j, pose );
    frame.imu                   = { phad::common::Timestamp{ t_i }, samples, false };
    last                        = estimator.update( frame );
    ASSERT_EQ( last.status, UpdateStatus::kOk ) << last.message;
    ASSERT_TRUE( last.diagnostics.gyro.has_value() );
    if ( index < 2U )
    {
      EXPECT_FALSE( last.diagnostics.gyro->align_ready );
    }
  }

  const auto& gyro = *last.diagnostics.gyro;
  ASSERT_TRUE( gyro.align_ready );
  ASSERT_TRUE( gyro.align_rank.has_value() );
  EXPECT_EQ( *gyro.align_rank, 3 );
  ASSERT_TRUE( gyro.bias_radps.has_value() );
  EXPECT_TRUE( gyro.bias_radps->isApprox( bias, 2e-4 ) )
      << "estimated=" << gyro.bias_radps->transpose();
  ASSERT_TRUE( gyro.align_rms_rad.has_value() );
  EXPECT_LT( *gyro.align_rms_rad, 1e-4 );
  EXPECT_EQ( gyro.align_edges, 3U );
  EXPECT_DOUBLE_EQ( gyro.align_support_s, 0.3 );
  EXPECT_TRUE( gyro.factor_eligible );
}

TEST( StereoVoGyroAlignment, RankDeficientSolveFailsAndRollsBack )
{
  EstimatorOptions options    = gyroOptions();
  options.gyro_align_window_s = 1.0;
  StereoVoEstimator estimator( makeCalibration(), makeImuParameters(),
                               options );

  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto singular_samples = makeSegment(
      100'000'000, 1'100'000'000,
      Eigen::Vector3d( 0.0, 0.0, 2.0 * std::numbers::pi ) );
  KeyframeMeasurement singular = makeFrame( 1'100'000'000 );
  singular.imu                 = { phad::common::Timestamp{ 100'000'000 }, singular_samples,
                                   false };
  const auto failed            = estimator.update( singular );
  EXPECT_EQ( failed.status, UpdateStatus::kFailed );
  EXPECT_EQ( failed.message.rfind( "gyro alignment:", 0 ), 0U );

  auto                valid_samples = makeSegment( 100'000'000, 1'100'000'000 );
  KeyframeMeasurement retry         = makeFrame( 1'100'000'000 );
  retry.imu                         = { phad::common::Timestamp{ 100'000'000 }, valid_samples,
                                        false };
  const auto accepted               = estimator.update( retry );
  ASSERT_EQ( accepted.status, UpdateStatus::kOk ) << accepted.message;
  ASSERT_TRUE( accepted.diagnostics.gyro.has_value() );
  EXPECT_TRUE( accepted.diagnostics.gyro->align_ready );
  EXPECT_EQ( accepted.diagnostics.gyro->align_rank, 3 );
}

TEST( StereoVoGyroAlignment, NonFiniteCovarianceFailsAndRollsBack )
{
  EstimatorOptions options    = gyroOptions();
  options.gyro_align_window_s = 0.1;
  StereoVoEstimator estimator( makeCalibration(), makeImuParameters( 1e200 ),
                               options );

  KeyframeMeasurement first = makeFrame( 100'000'000 );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  auto                samples = makeSegment( 100'000'000, 200'000'000 );
  KeyframeMeasurement second  = makeFrame( 200'000'000 );
  second.imu                  = { phad::common::Timestamp{ 100'000'000 }, samples, false };
  phad::estimator::VioUpdateResult failed;
  EXPECT_NO_THROW( failed = estimator.update( second ) );
  EXPECT_EQ( failed.status, UpdateStatus::kFailed );
  EXPECT_EQ( failed.message.rfind( "gyro alignment:", 0 ), 0U );

  KeyframeMeasurement retry = makeFrame( 200'000'000 );
  retry.imu.t_prev          = phad::common::Timestamp{ 100'000'000 };
  retry.imu.gap             = true;
  EXPECT_EQ( estimator.update( retry ).status, UpdateStatus::kOk );
}

TEST( StereoVoGyroShadow, SolvesAfterAlignmentWithoutWritingBackVisualState )
{
  EstimatorOptions shadow_options    = gyroOptions();
  shadow_options.gyro_align_window_s = 0.2;
  EstimatorOptions visual_options    = gyroOptions();
  visual_options.gyro_align_window_s = 100.0;
  StereoVoEstimator shadow( makeCalibration(), makeImuParameters(),
                            shadow_options );
  StereoVoEstimator visual( makeCalibration(), makeImuParameters(),
                            visual_options );

  Eigen::Isometry3d   pose  = Eigen::Isometry3d::Identity();
  KeyframeMeasurement first = makeFrame( 100'000'000, pose );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( shadow.update( first ).status, UpdateStatus::kOk );
  ASSERT_EQ( visual.update( first ).status, UpdateStatus::kOk );

  phad::estimator::VioUpdateResult shadow_result;
  phad::estimator::VioUpdateResult visual_result;
  const Eigen::Vector3d            omega( 0.1, -0.05, 0.2 );
  for ( std::size_t index = 0; index < 4U; ++index )
  {
    const Eigen::Vector3d measured_omega =
        index == 2U ? Eigen::Vector3d( 0.8, -0.4, 1.1 ) : omega;
    pose.linear() = pose.linear() * expRotation( omega * 0.1 );
    const std::int64_t t_i =
        100'000'000 + static_cast<std::int64_t>( index ) * 100'000'000;
    const std::int64_t  t_j     = t_i + 100'000'000;
    auto                samples = makeSegment( t_i, t_j, measured_omega );
    KeyframeMeasurement frame   = makeFrame( t_j, pose );
    frame.imu                   = { phad::common::Timestamp{ t_i }, samples, false };

    shadow_result = shadow.update( frame );
    visual_result = visual.update( frame );
    ASSERT_EQ( shadow_result.status, UpdateStatus::kOk )
        << shadow_result.message;
    ASSERT_EQ( visual_result.status, UpdateStatus::kOk )
        << visual_result.message;
    ASSERT_TRUE( shadow_result.estimate.has_value() );
    ASSERT_TRUE( visual_result.estimate.has_value() );
    EXPECT_TRUE( shadow_result.estimate->T_W_B.matrix().isApprox(
        visual_result.estimate->T_W_B.matrix(), 1e-12 ) );
    ASSERT_TRUE( shadow_result.diagnostics.gyro.has_value() );
    if ( index == 1U )
    {
      EXPECT_TRUE( shadow_result.diagnostics.gyro->align_ready );
      EXPECT_FALSE( shadow_result.diagnostics.gyro->factor_eligible );
      EXPECT_EQ( shadow_result.diagnostics.gyro->gyro_factor_count, 0U );
      EXPECT_FALSE(
          shadow_result.diagnostics.gyro->gyro_post_T_W_B.has_value() );
    }
    if ( index == 2U )
    {
      EXPECT_TRUE( shadow_result.diagnostics.gyro->factor_eligible );
      EXPECT_EQ( shadow_result.diagnostics.gyro->gyro_factor_count, 1U );
      EXPECT_TRUE(
          shadow_result.diagnostics.gyro->gyro_post_T_W_B.has_value() );
    }
  }

  ASSERT_TRUE( shadow_result.diagnostics.gyro.has_value() );
  const auto& gyro = *shadow_result.diagnostics.gyro;
  EXPECT_TRUE( gyro.align_ready );
  EXPECT_TRUE( gyro.factor_eligible );
  EXPECT_GT( gyro.gyro_factor_count, 0U );
  ASSERT_TRUE( gyro.visual_init_T_W_B.has_value() );
  ASSERT_TRUE( gyro.visual_post_T_W_B.has_value() );
  ASSERT_TRUE( gyro.pim_dt_s.has_value() );
  ASSERT_TRUE( gyro.pim_delta_q_Bi_Bj.has_value() );
  ASSERT_TRUE( gyro.pim_predict_q_W_Bj.has_value() );
  ASSERT_TRUE( gyro.gyro_post_T_W_B.has_value() );
  ASSERT_TRUE( gyro.visual_cost.has_value() );
  ASSERT_TRUE( gyro.gyro_cost.has_value() );
  EXPECT_TRUE( std::isfinite( *gyro.visual_cost ) );
  EXPECT_TRUE( std::isfinite( *gyro.gyro_cost ) );
  EXPECT_GT(
      ( gyro.gyro_post_T_W_B->linear() -
        gyro.visual_post_T_W_B->linear() )
          .norm(),
      1e-6 );
}

TEST( StereoVoGyroShadow, SkipsIncomingFactorWhenPredecessorWasEvicted )
{
  EstimatorOptions options    = gyroOptions();
  options.gyro_align_window_s = 0.1;
  options.window_size         = 1;
  StereoVoEstimator estimator( makeCalibration(), makeImuParameters(),
                               options );

  Eigen::Isometry3d   pose  = Eigen::Isometry3d::Identity();
  KeyframeMeasurement first = makeFrame( 100'000'000, pose );
  first.imu.t_prev          = first.timestamp;
  first.imu.gap             = false;
  ASSERT_EQ( estimator.update( first ).status, UpdateStatus::kOk );

  const Eigen::Vector3d            omega( 0.1, 0.05, -0.03 );
  phad::estimator::VioUpdateResult result;
  for ( std::size_t index = 0; index < 2U; ++index )
  {
    pose.linear() = pose.linear() * expRotation( omega * 0.1 );
    const std::int64_t t_i =
        100'000'000 + static_cast<std::int64_t>( index ) * 100'000'000;
    const std::int64_t  t_j     = t_i + 100'000'000;
    auto                samples = makeSegment( t_i, t_j, omega );
    KeyframeMeasurement frame   = makeFrame( t_j, pose );
    frame.imu                   = { phad::common::Timestamp{ t_i }, samples, false };
    result                      = estimator.update( frame );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  ASSERT_TRUE( result.diagnostics.gyro.has_value() );
  EXPECT_TRUE( result.diagnostics.gyro->align_ready );
  EXPECT_TRUE( result.diagnostics.gyro->factor_eligible );
  EXPECT_EQ( result.diagnostics.gyro->gyro_factor_count, 0U );
  EXPECT_FALSE( result.diagnostics.gyro->gyro_post_T_W_B.has_value() );
}

TEST( StereoVoGyroFused,
      MatchesShadowPosteriorThenFeedsCommittedStateToNextVisualUpdate )
{
  EstimatorOptions shadow_options    = gyroOptions();
  shadow_options.gyro_align_window_s = 0.2;
  EstimatorOptions fused_options     = shadow_options;
  fused_options.gyro_mode            = GyroMode::kFused;
  StereoVoEstimator shadow( makeCalibration(), makeImuParameters(),
                            shadow_options );
  StereoVoEstimator fused( makeCalibration(), makeImuParameters(),
                           fused_options );

  Eigen::Isometry3d   visual_pose = Eigen::Isometry3d::Identity();
  KeyframeMeasurement first       = makeFrame( 100'000'000, visual_pose );
  first.imu.t_prev                = first.timestamp;
  first.imu.gap                   = false;
  ASSERT_EQ( shadow.update( first ).status, UpdateStatus::kOk );
  ASSERT_EQ( fused.update( first ).status, UpdateStatus::kOk );

  const Eigen::Vector3d omega( 0.1, -0.05, 0.2 );
  for ( std::size_t index = 0; index < 2U; ++index )
  {
    visual_pose.linear() =
        visual_pose.linear() * expRotation( omega * 0.1 );
    const std::int64_t t_i =
        100'000'000 + static_cast<std::int64_t>( index ) * 100'000'000;
    const std::int64_t  t_j     = t_i + 100'000'000;
    auto                samples = makeSegment( t_i, t_j, omega );
    KeyframeMeasurement frame   = makeFrame( t_j, visual_pose );
    frame.imu                   = { phad::common::Timestamp{ t_i }, samples, false };
    ASSERT_EQ( shadow.update( frame ).status, UpdateStatus::kOk );
    ASSERT_EQ( fused.update( frame ).status, UpdateStatus::kOk );
  }

  const Eigen::Matrix3d previous_rotation = visual_pose.linear();
  const Eigen::Vector3d gyro_omega( 0.8, -0.4, 1.1 );
  visual_pose.linear() =
      visual_pose.linear() * expRotation( omega * 0.1 );
  auto                samples    = makeSegment( 300'000'000, 400'000'000, gyro_omega );
  KeyframeMeasurement correction = makeFrame( 400'000'000, visual_pose );
  correction.imu                 = { phad::common::Timestamp{ 300'000'000 }, samples, false };
  const auto shadow_result       = shadow.update( correction );
  const auto fused_result        = fused.update( correction );
  ASSERT_EQ( shadow_result.status, UpdateStatus::kOk )
      << shadow_result.message;
  ASSERT_EQ( fused_result.status, UpdateStatus::kOk ) << fused_result.message;
  ASSERT_TRUE( shadow_result.diagnostics.gyro.has_value() );
  ASSERT_TRUE( fused_result.diagnostics.gyro.has_value() );
  const auto& shadow_gyro = *shadow_result.diagnostics.gyro;
  const auto& fused_gyro  = *fused_result.diagnostics.gyro;
  ASSERT_TRUE( shadow_gyro.visual_post_T_W_B.has_value() );
  ASSERT_TRUE( fused_gyro.visual_post_T_W_B.has_value() );
  ASSERT_TRUE( shadow_gyro.gyro_post_T_W_B.has_value() );
  ASSERT_TRUE( fused_gyro.gyro_post_T_W_B.has_value() );
  ASSERT_TRUE( shadow_result.estimate.has_value() );
  ASSERT_TRUE( fused_result.estimate.has_value() );
  EXPECT_TRUE( shadow_gyro.visual_post_T_W_B->matrix().isApprox(
      fused_gyro.visual_post_T_W_B->matrix(), 1e-12 ) );
  EXPECT_TRUE( shadow_gyro.gyro_post_T_W_B->matrix().isApprox(
      fused_gyro.gyro_post_T_W_B->matrix(), 1e-12 ) );
  EXPECT_TRUE( shadow_result.estimate->T_W_B.matrix().isApprox(
      shadow_gyro.visual_post_T_W_B->matrix(), 1e-12 ) );
  EXPECT_TRUE( fused_result.estimate->T_W_B.matrix().isApprox(
      fused_gyro.gyro_post_T_W_B->matrix(), 1e-12 ) );

  const Eigen::Matrix3d target_rotation =
      previous_rotation * expRotation( gyro_omega * 0.1 );
  const auto rotation_error = [ &target_rotation ](
                                  const Eigen::Matrix3d& rotation ) {
    return Eigen::AngleAxisd( target_rotation.transpose() * rotation )
        .angle();
  };
  EXPECT_LT( rotation_error( fused_result.estimate->T_W_B.linear() ),
             rotation_error( shadow_result.estimate->T_W_B.linear() ) );

  KeyframeMeasurement reanchor = makeFrame(
      500'000'000, Eigen::Isometry3d::Identity(), 1'000 );
  reanchor.imu.t_prev    = phad::common::Timestamp{ 400'000'000 };
  reanchor.imu.gap       = true;
  const auto shadow_next = shadow.update( reanchor );
  const auto fused_next  = fused.update( reanchor );
  ASSERT_EQ( shadow_next.status, UpdateStatus::kOk );
  ASSERT_EQ( fused_next.status, UpdateStatus::kOk );
  ASSERT_TRUE( shadow_next.diagnostics.gyro.has_value() );
  ASSERT_TRUE( fused_next.diagnostics.gyro.has_value() );
  ASSERT_TRUE(
      shadow_next.diagnostics.gyro->visual_init_T_W_B.has_value() );
  ASSERT_TRUE( fused_next.diagnostics.gyro->visual_init_T_W_B.has_value() );
  EXPECT_GT(
      ( shadow_next.diagnostics.gyro->visual_init_T_W_B->matrix() -
        fused_next.diagnostics.gyro->visual_init_T_W_B->matrix() )
          .norm(),
      1e-6 );
}

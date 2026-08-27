#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/types.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "tests/estimator/vio_test_utils.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateDiagnostics;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration()
  {
    auto rigid = RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  [[nodiscard]] StereoObservation projectLandmark(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, LandmarkId id,
      const Eigen::Vector3d& point_W )
  {
    Eigen::Isometry3d T_B_C          = Eigen::Isometry3d::Identity();
    T_B_C.linear()                   = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()              = calibration.T_B_left_rectified().translation();
    const Eigen::Vector3d point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    const double          z          = point_left.z();
    EXPECT_GT( z, 0.0 );
    const double u_l =
        calibration.fxPixels() * point_left.x() / z + calibration.cxPixels();
    const double v =
        calibration.fyPixels() * point_left.y() / z + calibration.cyPixels();
    const double disparity =
        calibration.fxPixels() * calibration.baselineM() / z;
    return StereoObservation{ id, Eigen::Vector2d( u_l, v ), disparity };
  }

  VioMeasurement makeFrame(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, std::int64_t timestamp_ns,
      const std::vector<Eigen::Vector3d>& landmarks_W,
      const std::vector<LandmarkId>&      ids )
  {
    VioMeasurement measurement;
    measurement.m_timestamp = phad::common::Timestamp{ timestamp_ns };
    measurement.m_imu       = phad::test_support::stationaryImuPayload(
        measurement.m_timestamp );
    for ( std::size_t index = 0; index < landmarks_W.size(); ++index )
    {
      measurement.m_observations.push_back( projectLandmark(
          calibration, T_W_B, ids[ index ], landmarks_W[ index ] ) );
    }
    return measurement;
  }

  std::vector<Eigen::Isometry3d> translatingPoses( int count, double step_m )
  {
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve( static_cast<std::size_t>( count ) );
    for ( int index = 0; index < count; ++index )
    {
      Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
      T_W_B.translation() =
          Eigen::Vector3d( step_m * static_cast<double>( index ), 0.0, 0.0 );
      poses.push_back( T_W_B );
    }
    return poses;
  }

  std::vector<LandmarkId> sequentialIds( std::size_t count, LandmarkId start = 1 )
  {
    std::vector<LandmarkId> ids;
    ids.reserve( count );
    for ( std::size_t index = 0; index < count; ++index )
    {
      ids.push_back( start + static_cast<LandmarkId>( index ) );
    }
    return ids;
  }

  void offsetLeftPixel( VioMeasurement& measurement, LandmarkId id,
                        double delta_u_px )
  {
    for ( StereoObservation& observation : measurement.m_observations )
    {
      if ( observation.id == id )
      {
        observation.left_pixel.x() += delta_u_px;
        return;
      }
    }
    FAIL() << "landmark id " << id << " not found in measurement";
  }

  // Dense cloud so one toxic track cannot warp the pose window (10 landmarks
  // are under-constrained: BA absorbs ±40 px into ~2 px post-fit residuals).
  [[nodiscard]] std::vector<Eigen::Vector3d> makeDenseLandmarks()
  {
    std::vector<Eigen::Vector3d> landmarks;
    landmarks.reserve( 48 );
    for ( int iz = 0; iz < 3 && landmarks.size() < 48; ++iz )
    {
      for ( int iy = -2; iy <= 2 && landmarks.size() < 48; ++iy )
      {
        for ( int ix = -2; ix <= 2 && landmarks.size() < 48; ++ix )
        {
          landmarks.emplace_back(
              0.15 * static_cast<double>( ix ),
              0.12 * static_cast<double>( iy ),
              4.5 + 0.4 * static_cast<double>( iz ) );
        }
      }
    }
    return landmarks;
  }

  const std::vector<Eigen::Vector3d> kLandmarks = makeDenseLandmarks();

  EstimatorOptions defaultCullOptions()
  {
    EstimatorOptions options;
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size                     = 8;
    options.min_shared_landmarks            = 3;
    options.min_seed_observations           = 10;
    options.enable_pnp_init                 = false;  // keep poisoned obs in the window
    options.enable_outlier_cull             = true;
    options.outlier_avg_reproj_px           = 3.0;
    return options;
  }

  // Alternating ±80 px on a dense cloud. Constant bias / smaller offsets are
  // absorbed by BA (post-fit mean ||e|| stays below the 3 px default).
  [[nodiscard]] double poisonDeltaPx( std::size_t frame_index )
  {
    return ( frame_index % 2 == 0 ) ? 80.0 : -80.0;
  }

}  // namespace

TEST( StereoVoOutlierReoptTest, DefaultsEnableReoptAndDiagFlagsOff )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  EXPECT_TRUE( options.enable_outlier_reopt );
  UpdateDiagnostics d;
  EXPECT_FALSE( d.outlier_reopt );
  EXPECT_FALSE( d.outlier_reopt_failed );
}

TEST( StereoVoOutlierCullTest, RejectsNonPositiveOutlierAvgReproj )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.outlier_avg_reproj_px           = 0.0;
  EXPECT_THROW( ( VioEstimator{ makeCalibration(), phad::test_support::testImuParameters(), options } ),
                std::invalid_argument );
}

TEST( StereoVoOutlierCullTest, CullsPersistentHighReprojLandmark )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 10, 0.05 );
  const LandmarkId poison_id   = 1;

  VioEstimator  estimator( calibration, phad::test_support::testImuParameters(), defaultCullOptions() );
  std::uint32_t culled_on_frame = 0;
  int           cull_frame      = -1;

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids );
    if ( index >= 1 )
    {
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( result.diagnostics.outliers_culled >= 1U )
    {
      culled_on_frame = result.diagnostics.outliers_culled;
      cull_frame      = static_cast<int>( index );
      break;
    }
  }
  ASSERT_GE( culled_on_frame, 1U );
  ASSERT_GE( cull_frame, 0 );

  const auto stamps = estimator.observationTimestamps( poison_id );
  EXPECT_FALSE( stamps.empty() );

  const std::size_t probe = static_cast<std::size_t>( cull_frame + 1 );
  ASSERT_LT( probe, poses.size() );
  const std::int64_t probe_ts =
      static_cast<std::int64_t>( probe + 1 ) * 50'000'000;
  auto probe_meas =
      makeFrame( calibration, poses[ probe ], probe_ts, kLandmarks, ids );
  offsetLeftPixel( probe_meas, poison_id, poisonDeltaPx( probe ) );
  const auto probe_result = estimator.update( probe_meas );
  ASSERT_EQ( probe_result.status, UpdateStatus::kOk ) << probe_result.message;
  EXPECT_EQ( probe_result.diagnostics.num_shared, ids.size() - 1U );
}

TEST( StereoVoOutlierCullTest, LowSupportFactorsRunExistingCullPath )
{
  const auto calibration = makeCalibration();
  const auto base_ids    = sequentialIds( kLandmarks.size(), 1 );

  EstimatorOptions options     = defaultCullOptions();
  options.min_pnp_inliers      = 20;
  options.enable_outlier_reopt = false;
  VioEstimator estimator( calibration,
                          phad::test_support::testImuParameters(), options );

  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, Eigen::Isometry3d::Identity(),
                                     50'000'000, kLandmarks, base_ids ) )
                 .status,
             UpdateStatus::kOk );
  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, Eigen::Isometry3d::Identity(),
                                     100'000'000, kLandmarks, base_ids ) )
                 .status,
             UpdateStatus::kOk );

  constexpr std::size_t              kCoastLandmarks = 19U;
  const std::vector<Eigen::Vector3d> coast_landmarks(
      kLandmarks.begin(), kLandmarks.begin() + kCoastLandmarks );
  const auto       coast_ids = sequentialIds( kCoastLandmarks, 100 );
  const LandmarkId poison_id = coast_ids.front();

  phad::estimator::VioUpdateResult result;
  for ( std::size_t index = 0; index < 4U; ++index )
  {
    const std::int64_t ts_ns =
        150'000'000 + static_cast<std::int64_t>( index ) * 50'000'000;
    auto measurement = makeFrame( calibration, Eigen::Isometry3d::Identity(),
                                  ts_ns, coast_landmarks, coast_ids );
    if ( index >= 1U )
    {
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
    }
    result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  EXPECT_EQ( result.diagnostics.num_shared, kCoastLandmarks );
  EXPECT_EQ( result.diagnostics.num_current_visual_factors,
             kCoastLandmarks );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_coast_duration_ns,
             200'000'000 );
  EXPECT_EQ( result.diagnostics.unsupported_span_ns, 200'000'000 );
  EXPECT_GE( result.diagnostics.outliers_culled, 1U );
  EXPECT_NE( std::find( result.diagnostics.culled_landmark_ids.begin(),
                        result.diagnostics.culled_landmark_ids.end(),
                        poison_id ),
             result.diagnostics.culled_landmark_ids.end() );
}

TEST( StereoVoOutlierCullTest, SkipsLandmarksWithFewerThanFourObs )
{
  const auto            calibration = makeCalibration();
  const auto            base_ids    = sequentialIds( kLandmarks.size(), 1 );
  const auto            poses       = translatingPoses( 5, 0.05 );
  const LandmarkId      late_id     = 99;
  const Eigen::Vector3d late_point{ 0.2, 0.05, 5.1 };

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), defaultCullOptions() );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, base_ids );
    if ( index >= 2 )
    {
      auto obs =
          projectLandmark( calibration, poses[ index ], late_id, late_point );
      obs.left_pixel.x() += poisonDeltaPx( index );
      measurement.m_observations.push_back( obs );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U )
        << "frame " << index << " unexpectedly culled";
  }
}

TEST( StereoVoOutlierCullTest, KeepsInliersUnderThreshold )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto poses       = translatingPoses( 6, 0.05 );

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), defaultCullOptions() );
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    const auto result = estimator.update(
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
    EXPECT_EQ( result.diagnostics.outliers_culled_unique, 0U );
    EXPECT_NEAR( result.diagnostics.reproj_rms_after_cull_px,
                 result.diagnostics.reproj_rms_after_px, 1e-12 );
  }
}

TEST( StereoVoOutlierCullTest, DisabledSkipsMeanReprojCull )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 10, 0.05 );
  const LandmarkId poison_id   = 1;

  EstimatorOptions options    = defaultCullOptions();
  options.enable_outlier_cull = false;
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids );
    if ( index >= 1 )
    {
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
    EXPECT_EQ( result.diagnostics.outliers_culled_unique, 0U );
    EXPECT_NEAR( result.diagnostics.reproj_rms_after_cull_px,
                 result.diagnostics.reproj_rms_after_px, 1e-12 );
  }
}

TEST( StereoVoOutlierCullTest, PoseEqualsFirstLmWithoutReopt )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 10, 0.05 );
  const LandmarkId poison_id   = 1;

  EstimatorOptions options_cull   = defaultCullOptions();
  EstimatorOptions options_off    = defaultCullOptions();
  options_off.enable_outlier_cull = false;

  VioEstimator est_cull( calibration, phad::test_support::testImuParameters(), options_cull );
  VioEstimator est_off( calibration, phad::test_support::testImuParameters(), options_off );

  bool compared = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids );
    if ( index >= 1 )
    {
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
    }
    const auto r_cull = est_cull.update( measurement );
    const auto r_off  = est_off.update( measurement );
    ASSERT_EQ( r_cull.status, UpdateStatus::kOk ) << r_cull.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk ) << r_off.message;
    ASSERT_TRUE( r_cull.estimate.has_value() );
    ASSERT_TRUE( r_off.estimate.has_value() );

    if ( r_cull.diagnostics.outliers_culled >= 1U )
    {
      EXPECT_TRUE( r_cull.estimate->T_W_B.matrix().isApprox(
          r_off.estimate->T_W_B.matrix(), 1e-12 ) );
      compared = true;
      break;
    }
  }
  EXPECT_TRUE( compared );
}

TEST( StereoVoOutlierCullTest, RepeatCullIncrementsTotalNotUnique )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 24, 0.05 );
  const LandmarkId poison_id   = 1;

  // Requires rebirth of the same LandmarkId after mean-cull (Slice ④ semantics).
  EstimatorOptions options     = defaultCullOptions();
  options.block_culled_rebirth = false;
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  std::uint32_t total_culled      = 0;
  std::uint32_t total_unique      = 0;
  int           cull_events       = 0;
  int           frames_since_cull = 0;

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids );
    const bool poison =
        ( cull_events == 0 ) ? ( index >= 1 ) : ( frames_since_cull >= 1 );
    if ( poison )
    {
      const std::size_t phase =
          ( cull_events == 0 ) ? index
                               : static_cast<std::size_t>( frames_since_cull );
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( phase ) );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( cull_events > 0 )
    {
      ++frames_since_cull;
    }
    if ( result.diagnostics.outliers_culled > 0U )
    {
      total_culled += result.diagnostics.outliers_culled;
      total_unique += result.diagnostics.outliers_culled_unique;
      ++cull_events;
      frames_since_cull = 0;
      if ( cull_events >= 2 )
      {
        break;
      }
    }
  }
  ASSERT_GE( cull_events, 2 );
  EXPECT_EQ( total_culled, 2U );
  EXPECT_EQ( total_unique, 1U );
}

TEST( StereoVoOutlierCullTest, ConflictingNearLandmarkSequenceKeepsCullDisabled )
{
  // Cull-off diagnostics stay internally consistent throughout the sequence.
  const auto       calibration = makeCalibration();
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 8;
  options.min_shared_landmarks            = 2;
  options.min_landmark_observations       = 2;
  options.huber_k_px                      = 0.0;
  options.enable_outlier_cull             = false;

  const std::vector<Eigen::Vector3d> far_landmarks{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
      { 0.5, 0.05, 5.8 },
  };
  const Eigen::Vector3d near_landmark{ 0.15, 0.0, 1.0 };
  const auto            far_ids = sequentialIds( far_landmarks.size(), 1 );
  const LandmarkId      near_id = 99;

  VioEstimator      estimator( calibration, phad::test_support::testImuParameters(), options );
  StereoObservation near_at_first{};

  for ( int index = 0; index < 4; ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    const double      z     = ( index == 0 ) ? 0.0 : 2.5;
    T_W_B.translation()     = Eigen::Vector3d( 0.0, 0.0, z );

    VioMeasurement measurement;
    measurement.m_timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( index + 1 ) * 50'000'000 };
    measurement.m_imu = phad::test_support::stationaryImuPayload(
        measurement.m_timestamp, 50'000'000 );

    for ( std::size_t landmark_index = 0; landmark_index < far_landmarks.size();
          ++landmark_index )
    {
      measurement.m_observations.push_back( projectLandmark(
          calibration, T_W_B, far_ids[ landmark_index ],
          far_landmarks[ landmark_index ] ) );
    }

    if ( index == 0 )
    {
      near_at_first =
          projectLandmark( calibration, T_W_B, near_id, near_landmark );
      measurement.m_observations.push_back( near_at_first );
    }
    else if ( index <= 2 )
    {
      measurement.m_observations.push_back( near_at_first );
    }

    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.estimate.has_value() );
    EXPECT_TRUE( result.estimate->T_W_B.matrix().allFinite() );
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
    EXPECT_NEAR( result.diagnostics.reproj_rms_after_cull_px,
                 result.diagnostics.reproj_rms_after_px, 1e-12 );
  }
}

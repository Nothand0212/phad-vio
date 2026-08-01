#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;
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

  KeyframeMeasurement makeFrame(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, std::int64_t timestamp_ns,
      const std::vector<Eigen::Vector3d>& landmarks_W,
      const std::vector<LandmarkId>&      ids )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{ timestamp_ns };
    for ( std::size_t index = 0; index < landmarks_W.size(); ++index )
    {
      measurement.observations.push_back( projectLandmark(
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

  void offsetLeftPixel( KeyframeMeasurement& measurement, LandmarkId id,
                        const Eigen::Vector2d& delta_px )
  {
    for ( StereoObservation& observation : measurement.observations )
    {
      if ( observation.id == id )
      {
        observation.left_pixel += delta_px;
        return;
      }
    }
    FAIL() << "landmark id " << id << " not found in measurement";
  }

  [[nodiscard]] bool hasTimestamp(
      const std::vector<phad::common::Timestamp>& stamps,
      std::int64_t                                timestamp_ns )
  {
    return std::find( stamps.begin(), stamps.end(),
                      phad::common::Timestamp{ timestamp_ns } ) != stamps.end();
  }

  // Values chosen to keep every landmark well within the frustum across the
  // small translations used below (same set as stereo_vo_reanchor_test).
  const std::vector<Eigen::Vector3d> kLandmarksA{
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

  const std::vector<Eigen::Vector3d> kLandmarksB{
      { 1.4, -0.3, 5.4 },
      { 0.9, 0.35, 4.9 },
      { 1.1, -0.15, 6.2 },
      { 1.6, 0.05, 5.1 },
      { 0.65, -0.4, 4.7 },
      { 1.0, 0.2, 5.6 },
      { 1.25, -0.2, 4.4 },
      { 0.8, 0.3, 5.9 },
      { 1.35, -0.05, 5.0 },
      { 0.85, 0.15, 4.8 },
  };

  EstimatorOptions defaultPnpOptions()
  {
    EstimatorOptions options;
    options.window_size          = 5;
    options.min_shared_landmarks = 3;
    options.min_pnp_inliers      = 10;
    options.enable_pnp_init      = true;
    return options;
  }

}  // namespace

TEST( StereoVoPnpTest, RejectsNonPositivePnpReproj )
{
  EstimatorOptions options;
  options.pnp_reproj_px = 0.0;
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsInvalidPnpConfidence )
{
  EstimatorOptions options;
  options.pnp_confidence = 1.0;  // 要求 ∈ (0, 1)
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsMinPnpInliersBelowFour )
{
  EstimatorOptions options;
  options.min_pnp_inliers = 3;
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, PnpSucceedsOnCleanMotion )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 6, 0.05 );

  EstimatorOptions options_pnp = defaultPnpOptions();
  StereoVoEstimator estimator_pnp( calibration, options_pnp );

  EstimatorOptions options_cv = defaultPnpOptions();
  options_cv.enable_pnp_init  = false;
  StereoVoEstimator estimator_cv( calibration, options_cv );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    const auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarksA, ids );

    const auto result_pnp = estimator_pnp.update( measurement );
    ASSERT_EQ( result_pnp.status, UpdateStatus::kOk ) << result_pnp.message;
    ASSERT_TRUE( result_pnp.estimate.has_value() );

    const auto result_cv = estimator_cv.update( measurement );
    ASSERT_EQ( result_cv.status, UpdateStatus::kOk ) << result_cv.message;
    ASSERT_TRUE( result_cv.estimate.has_value() );

    if ( index == 0 )
    {
      EXPECT_FALSE( result_pnp.diagnostics.pnp_success );
      EXPECT_FALSE( result_cv.diagnostics.pnp_success );
      continue;
    }

    EXPECT_TRUE( result_pnp.diagnostics.pnp_success );
    EXPECT_GE( result_pnp.diagnostics.pnp_inliers, 10U );
    EXPECT_FALSE( result_cv.diagnostics.pnp_success );

    // Clean synthetic motion + BA make both paths near-truth; gate on absolute
    // translation error (brief allows <0.05 m in lieu of a CV gap).
    const double err_pnp =
        ( result_pnp.estimate->T_W_B.translation() -
          poses[ index ].translation() )
            .norm();
    EXPECT_LT( err_pnp, 0.05 );
  }
}

TEST( StereoVoPnpTest, PnpMasksSharedOutliers )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions  options = defaultPnpOptions();
  StereoVoEstimator estimator( calibration, options );

  // Need shared > min_pnp_inliers so one outlier still leaves enough inliers.
  // Public surface has no window / landmarks_W accessors: prove mask via graph
  // count (probe stays at 1 window obs) + observationTimestamps + next-frame
  // num_shared (landmarks_W retained).
  const auto seed = estimator.update(
      makeFrame( calibration, poses[ 0 ], 50'000'000, kLandmarksA, ids ) );
  ASSERT_EQ( seed.status, UpdateStatus::kOk ) << seed.message;

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 1 ], 100'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_EQ( introduced.diagnostics.num_landmarks, kLandmarksA.size() );

  auto corrupted = makeFrame( calibration, poses[ 2 ], 150'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto masked = estimator.update( corrupted );
  ASSERT_EQ( masked.status, UpdateStatus::kOk ) << masked.message;
  EXPECT_TRUE( masked.diagnostics.pnp_success );
  EXPECT_GE( masked.diagnostics.pnp_inliers, 10U );
  EXPECT_LT( masked.diagnostics.pnp_inliers,
             static_cast<std::uint32_t>( landmarks_plus.size() ) );
  // Masked current obs → probe still has one window sighting → out of graph.
  EXPECT_EQ( masked.diagnostics.num_landmarks, kLandmarksA.size() );
  EXPECT_EQ( masked.diagnostics.num_observations, landmarks_plus.size() );

  const auto stamps = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps.size(), 2U );
  EXPECT_TRUE( hasTimestamp( stamps, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( stamps, 150'000'000 ) );

  const auto recovered = estimator.update( makeFrame(
      calibration, poses[ 3 ], 200'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_EQ( recovered.diagnostics.num_shared, landmarks_plus.size() );
  EXPECT_EQ( recovered.diagnostics.num_landmarks, kLandmarksA.size() + 1U );
}

TEST( StereoVoPnpTest, FallsBackWhenTooFewShared )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.min_pnp_inliers  = 20;  // > available shared (10)
  StereoVoEstimator estimator( calibration, options );

  for ( std::size_t index = 0; index < 2; ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  // Introduce probe with one prior obs; on fallback the corrupted obs is NOT
  // culled, so probe reaches min_landmark_observations and enters the graph.
  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 2 ], 150'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_FALSE( introduced.diagnostics.pnp_success );
  EXPECT_EQ( introduced.diagnostics.num_landmarks, kLandmarksA.size() );

  auto corrupted = makeFrame( calibration, poses[ 3 ], 200'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto fallback = estimator.update( corrupted );
  ASSERT_EQ( fallback.status, UpdateStatus::kOk ) << fallback.message;
  EXPECT_FALSE( fallback.diagnostics.pnp_success );
  EXPECT_EQ( fallback.diagnostics.pnp_inliers, 0U );
  // Not culled: probe now has 2 window observations → in the graph.
  EXPECT_EQ( fallback.diagnostics.num_landmarks, kLandmarksA.size() + 1U );
}

TEST( StereoVoPnpTest, FallsBackWhenInliersBelowMin )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.min_pnp_inliers  = 10;
  StereoVoEstimator estimator( calibration, options );

  for ( std::size_t index = 0; index < 2; ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 2 ], 150'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;

  // Corrupt most shared landmarks so RANSAC inliers stay below min.
  auto corrupted = makeFrame( calibration, poses[ 3 ], 200'000'000,
                              landmarks_plus, ids_plus );
  for ( std::size_t index = 0; index < 8; ++index )
  {
    offsetLeftPixel( corrupted, ids[ index ],
                     Eigen::Vector2d( 40.0 + 5.0 * static_cast<double>( index ),
                                     0.0 ) );
  }
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );

  const auto fallback = estimator.update( corrupted );
  ASSERT_EQ( fallback.status, UpdateStatus::kOk ) << fallback.message;
  EXPECT_FALSE( fallback.diagnostics.pnp_success );
  EXPECT_EQ( fallback.diagnostics.pnp_inliers, 0U );
  // Fallback must not cull: probe reaches 2 observations and enters graph.
  EXPECT_EQ( fallback.diagnostics.num_landmarks, kLandmarksA.size() + 1U );
}

TEST( StereoVoPnpTest, DisabledMatchesConstantVelocity )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 5, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.enable_pnp_init  = false;
  StereoVoEstimator estimator( calibration, options );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_FALSE( result.diagnostics.pnp_success );
    EXPECT_EQ( result.diagnostics.pnp_inliers, 0U );
  }
}

TEST( StereoVoPnpTest, SeedAndReanchorSkipPnp )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options = defaultPnpOptions();
  StereoVoEstimator estimator( calibration, options );

  const auto poses = translatingPoses( 4, 0.05 );
  std::vector<Eigen::Isometry3d> accepted;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids_a ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( index == 0 )
    {
      EXPECT_FALSE( result.diagnostics.pnp_success );
    }
    accepted.push_back( result.estimate->T_W_B );
  }

  const Eigen::Isometry3d& T_prev = accepted[ accepted.size() - 2 ];
  const Eigen::Isometry3d& T_last = accepted.back();
  const Eigen::Isometry3d  expected_anchor =
      T_last * ( T_prev.inverse() * T_last );

  const auto turnover = estimator.update( makeFrame(
      calibration, expected_anchor, 250'000'000, kLandmarksB, ids_b ) );
  ASSERT_EQ( turnover.status, UpdateStatus::kOk ) << turnover.message;
  EXPECT_EQ( turnover.diagnostics.segment_id, 1U );
  EXPECT_EQ( turnover.diagnostics.num_shared, 0U );
  EXPECT_FALSE( turnover.diagnostics.pnp_success );
  EXPECT_EQ( turnover.diagnostics.pnp_inliers, 0U );
}

TEST( StereoVoPnpTest, MaskedObsKeepsTrackTimesButMayDropFromGraph )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 5, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  StereoVoEstimator estimator( calibration, options );

  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, poses[ 0 ], 50'000'000,
                                     kLandmarksA, ids ) )
                 .status,
             UpdateStatus::kOk );

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 1 ], 100'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_EQ( introduced.diagnostics.num_landmarks, kLandmarksA.size() );

  auto corrupted = makeFrame( calibration, poses[ 2 ], 150'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto masked = estimator.update( corrupted );
  ASSERT_EQ( masked.status, UpdateStatus::kOk ) << masked.message;
  EXPECT_TRUE( masked.diagnostics.pnp_success );
  // Masked current obs → still only one window sighting → out of graph.
  EXPECT_EQ( masked.diagnostics.num_landmarks, kLandmarksA.size() );

  const auto stamps_masked = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps_masked.size(), 2U );
  EXPECT_TRUE( hasTimestamp( stamps_masked, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( stamps_masked, 150'000'000 ) );

  const auto recovered = estimator.update( makeFrame(
      calibration, poses[ 3 ], 200'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_TRUE( recovered.diagnostics.pnp_success );
  // Prior (unmasked) + recovered → count >= 2 → back in the graph.
  EXPECT_EQ( recovered.diagnostics.num_landmarks, kLandmarksA.size() + 1U );

  const auto stamps_recovered = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps_recovered.size(), 3U );
  EXPECT_TRUE( hasTimestamp( stamps_recovered, 200'000'000 ) );
}

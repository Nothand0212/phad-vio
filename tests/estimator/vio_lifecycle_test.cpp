#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "tests/estimator/vio_test_utils.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
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

  // Projection values keep every landmark well within the frustum (positive
  // depth, moderate disparity) across the small translations used below.
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

  // Runs `frame_count` normal frames on `estimator` using ids_A/kLandmarksA
  // and returns the accepted poses for every frame (index-aligned).
  std::vector<Eigen::Isometry3d> runNormalSegment(
      VioEstimator& estimator, const RectifiedStereoCalibration& calibration,
      const std::vector<Eigen::Isometry3d>& poses,
      const std::vector<LandmarkId>&        ids_a )
  {
    std::vector<Eigen::Isometry3d> accepted;
    accepted.reserve( poses.size() );
    for ( std::size_t index = 0; index < poses.size(); ++index )
    {
      const auto result = estimator.update( makeFrame(
          calibration, poses[ index ],
          static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
          ids_a ) );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      accepted.push_back( result.estimate->T_W_B );
    }
    return accepted;
  }

}  // namespace

TEST( VisualOutageLifecycle, LandmarkIdTurnoverCoastsInActiveSegment )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;
  ASSERT_GE( kLandmarksB.size(),
             static_cast<std::size_t>( options.min_seed_observations ) );

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );
  const auto   poses    = translatingPoses( 4, 0.05 );
  const auto   accepted = runNormalSegment( estimator, calibration, poses, ids_a );

  const Eigen::Isometry3d& T_prev = accepted[ accepted.size() - 2 ];
  const Eigen::Isometry3d& T_last = accepted.back();
  const Eigen::Isometry3d  expected_anchor =
      T_last * ( T_prev.inverse() * T_last );

  // Every current id is outside the active map, so the packet is committed
  // without visual observations.
  const auto turnover = estimator.update( makeFrame(
      calibration, expected_anchor, 250'000'000, kLandmarksB, ids_b ) );
  ASSERT_EQ( turnover.status, UpdateStatus::kOk ) << turnover.message;
  ASSERT_TRUE( turnover.estimate.has_value() );
  EXPECT_EQ( turnover.diagnostics.segment_id, 0U );
  EXPECT_EQ( turnover.diagnostics.num_shared, 0U );
  EXPECT_EQ( turnover.diagnostics.m_vio.m_visual_coast_duration_ns,
             50'000'000 );
  EXPECT_TRUE( turnover.estimate->T_W_B.matrix().allFinite() );

  // Repeated unmapped ids remain unsupported; they are not silently seeded.
  Eigen::Isometry3d next_pose = expected_anchor;
  next_pose.translation() += Eigen::Vector3d( 0.05, 0.0, 0.0 );
  const auto continued = estimator.update(
      makeFrame( calibration, next_pose, 300'000'000, kLandmarksB, ids_b ) );
  EXPECT_EQ( continued.status, UpdateStatus::kOk ) << continued.message;
  EXPECT_EQ( continued.diagnostics.segment_id, 0U );
  EXPECT_EQ( continued.diagnostics.num_shared, 0U );
  EXPECT_EQ( continued.diagnostics.m_vio.m_visual_coast_duration_ns,
             100'000'000 );
}

TEST( VisualOutageLifecycle, UnsupportedIdsDoNotPoisonVisualRecovery )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;
  options.min_seed_observations           = 10;

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );
  const auto   poses = translatingPoses( 4, 0.05 );
  runNormalSegment( estimator, calibration, poses, ids_a );

  // Sparse unmapped observations are excluded from the committed coast state.
  const std::vector<Eigen::Vector3d> sparse_landmarks(
      kLandmarksB.begin(), kLandmarksB.begin() + 3 );
  const std::vector<LandmarkId> sparse_ids( ids_b.begin(), ids_b.begin() + 3 );
  const auto                    starved = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 250'000'000,
      sparse_landmarks, sparse_ids ) );
  EXPECT_EQ( starved.status, UpdateStatus::kOk ) << starved.message;
  EXPECT_TRUE( starved.estimate.has_value() );
  EXPECT_EQ( starved.diagnostics.segment_id, 0U );
  EXPECT_EQ( starved.diagnostics.m_vio.m_visual_coast_duration_ns,
             50'000'000 );

  const auto recovered = estimator.update( makeFrame(
      calibration, poses.back(), 300'000'000, kLandmarksA, ids_a ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_EQ( recovered.diagnostics.segment_id, 0U );
  EXPECT_GT( recovered.diagnostics.num_shared, 0U );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
}

TEST( VioInitialization, FirstSegmentSeedGate )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;
  options.min_seed_observations           = 10;
  options.enable_accumulated_seed         = true;  // 首段累积 (默认关, 显式开)

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  const std::vector<Eigen::Vector3d> sparse_landmarks(
      kLandmarksA.begin(), kLandmarksA.begin() + 3 );
  const std::vector<LandmarkId> sparse_ids( ids_a.begin(), ids_a.begin() + 3 );
  const auto                    starved = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 50'000'000, sparse_landmarks,
      sparse_ids ) );
  EXPECT_EQ( starved.status, UpdateStatus::kInitializing );
  EXPECT_FALSE( starved.estimate.has_value() );
  EXPECT_EQ( starved.message,
             "accumulating seed observations (first segment)" );

  const auto seeded = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 100'000'000, kLandmarksA,
      ids_a ) );
  ASSERT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
  EXPECT_EQ( seeded.diagnostics.segment_id, 0U );
}

TEST( VioInitialization, AccumulatedSeedingSeedsAfterSparseFrames )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;
  options.min_seed_observations           = 10;
  options.enable_accumulated_seed         = true;  // 首段累积 (默认关, 显式开)

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  // Three consecutive sparse frames: each contributes 4 stereo observations,
  // none reaches min_seed_observations=10 alone. Across frames the buffer
  // accumulates 10 unique tracks and the third frame seeds from the
  // accumulated evidence (SVO DepthFilter-style) before accepting the root.
  const auto sparse = [ & ]( std::int64_t t, std::size_t begin,
                             std::size_t count ) {
    std::vector<Eigen::Vector3d> landmarks;
    std::vector<LandmarkId>      ids;
    for ( std::size_t index = begin; index < begin + count; ++index )
    {
      landmarks.push_back( kLandmarksA[ index ] );
      ids.push_back( ids_a[ index ] );
    }
    return estimator.update( makeFrame(
        calibration, Eigen::Isometry3d::Identity(), t, landmarks, ids ) );
  };

  const auto first = sparse( 50'000'000, 0, 4 );  // ids 1-4
  EXPECT_EQ( first.status, UpdateStatus::kInitializing );
  EXPECT_EQ( first.message,
             "accumulating seed observations (first segment)" );

  const auto second = sparse( 100'000'000, 4, 4 );  // ids 5-8
  EXPECT_EQ( second.status, UpdateStatus::kInitializing );
  EXPECT_EQ( second.message,
             "accumulating seed observations (first segment)" );

  // ids 7-10 (two overlap the buffer) → 10 unique tracks → seeded.
  const auto seeded = sparse( 150'000'000, 6, 4 );
  ASSERT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
  EXPECT_TRUE( seeded.estimate.has_value() );
  EXPECT_EQ( seeded.diagnostics.segment_id, 0U );

  // The seeded segment must keep accepting normal frames afterward, and the
  // accumulation buffer must be drained (no stale re-seed).
  const auto continued = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 200'000'000,
      std::vector<Eigen::Vector3d>( kLandmarksA.begin(),
                                    kLandmarksA.begin() + 10 ),
      std::vector<LandmarkId>( ids_a.begin(), ids_a.begin() + 10 ) ) );
  EXPECT_EQ( continued.status, UpdateStatus::kOk ) << continued.message;
}

TEST( VisualOutageLifecycle, CoastUsesImuPropagation )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;

  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );
  runNormalSegment( estimator, calibration, poses, ids_a );

  const auto coast = estimator.update( makeFrame(
      calibration, poses.back(), 250'000'000, kLandmarksB, ids_b ) );
  ASSERT_EQ( coast.status, UpdateStatus::kOk ) << coast.message;
  ASSERT_TRUE( coast.estimate.has_value() );
  EXPECT_TRUE( coast.estimate->T_W_B.matrix().allFinite() );
  EXPECT_EQ( coast.diagnostics.m_vio.m_imu_factors, 4U );
  EXPECT_EQ( coast.diagnostics.m_vio.m_bias_rw_factors, 4U );
  EXPECT_EQ( coast.diagnostics.m_vio.m_visual_coast_duration_ns, 50'000'000 );
}

TEST( VioInitialization, CtorRejectsMinSeedObservationsBelowOne )
{
  const auto calibration = makeCalibration();

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.min_seed_observations           = 0;
  EXPECT_THROW( VioEstimator( calibration, phad::test_support::testImuParameters(), options ),
                std::invalid_argument );
}

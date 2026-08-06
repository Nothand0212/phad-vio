#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
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

  // Values chosen to keep every landmark well within the frustum (positive
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
      StereoVoEstimator& estimator, const RectifiedStereoCalibration& calibration,
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

TEST( StereoVoReanchor, RecoversAfterLandmarkIdTurnover )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size          = 5;
  options.min_shared_landmarks = 3;
  ASSERT_GE( kLandmarksB.size(),
             static_cast<std::size_t>( options.min_seed_observations ) );

  StereoVoEstimator estimator( calibration, options );
  const auto        poses    = translatingPoses( 4, 0.05 );
  const auto        accepted = runNormalSegment( estimator, calibration, poses, ids_a );

  const Eigen::Isometry3d& T_prev = accepted[ accepted.size() - 2 ];
  const Eigen::Isometry3d& T_last = accepted.back();
  const Eigen::Isometry3d  expected_anchor =
      T_last * ( T_prev.inverse() * T_last );

  // Overlap break: every observed id is brand new. Frame is generated as
  // seen from the expected anchor pose so the seeded landmarks stay in
  // front of the camera regardless of the CV extrapolation math.
  const auto turnover = estimator.update( makeFrame(
      calibration, expected_anchor, 250'000'000, kLandmarksB, ids_b ) );
  ASSERT_EQ( turnover.status, UpdateStatus::kOk ) << turnover.message;
  ASSERT_TRUE( turnover.estimate.has_value() );
  EXPECT_EQ( turnover.diagnostics.segment_id, 1U );
  EXPECT_EQ( turnover.diagnostics.num_shared, 0U );
  EXPECT_TRUE( turnover.estimate->T_W_B.matrix().isApprox(
      expected_anchor.matrix(), 1e-6 ) );

  // The new segment must keep accepting normal frames afterward.
  Eigen::Isometry3d next_pose = expected_anchor;
  next_pose.translation() += Eigen::Vector3d( 0.05, 0.0, 0.0 );
  const auto continued = estimator.update(
      makeFrame( calibration, next_pose, 300'000'000, kLandmarksB, ids_b ) );
  EXPECT_EQ( continued.status, UpdateStatus::kOk ) << continued.message;
  EXPECT_EQ( continued.diagnostics.segment_id, 1U );
  EXPECT_GT( continued.diagnostics.num_shared, 0U );
}

TEST( StereoVoReanchor, SeedGateRejectsWithoutPoisoningState )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size           = 5;
  options.min_shared_landmarks  = 3;
  options.min_seed_observations = 10;

  StereoVoEstimator estimator( calibration, options );
  const auto        poses = translatingPoses( 4, 0.05 );
  runNormalSegment( estimator, calibration, poses, ids_a );

  // Too few observations to seed a new segment (< min_seed_observations).
  const std::vector<Eigen::Vector3d> sparse_landmarks(
      kLandmarksB.begin(), kLandmarksB.begin() + 3 );
  const std::vector<LandmarkId> sparse_ids( ids_b.begin(), ids_b.begin() + 3 );
  const auto                    starved = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 250'000'000,
      sparse_landmarks, sparse_ids ) );
  EXPECT_EQ( starved.status, UpdateStatus::kRejected );
  EXPECT_FALSE( starved.estimate.has_value() );
  EXPECT_EQ( starved.diagnostics.segment_id, 0U );
  EXPECT_EQ( starved.message, "insufficient observations to seed new segment" );

  // State must be untouched: a fully-observed break frame right after
  // still seeds a fresh segment successfully.
  const auto seeded = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 300'000'000, kLandmarksB,
      ids_b ) );
  ASSERT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
  EXPECT_EQ( seeded.diagnostics.segment_id, 1U );
}

TEST( StereoVoReanchor, ReanchorDisabledReproducesLegacyReject )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size          = 5;
  options.min_shared_landmarks = 3;
  options.enable_reanchor      = false;

  StereoVoEstimator estimator( calibration, options );
  const auto        poses = translatingPoses( 4, 0.05 );
  runNormalSegment( estimator, calibration, poses, ids_a );

  const auto first_break = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 250'000'000, kLandmarksB,
      ids_b ) );
  EXPECT_EQ( first_break.status, UpdateStatus::kRejected );
  EXPECT_FALSE( first_break.estimate.has_value() );

  // Legacy behavior: the break is permanent, even with plenty of shared
  // observations on subsequent alien frames.
  const auto second_break = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 300'000'000, kLandmarksB,
      ids_b ) );
  EXPECT_EQ( second_break.status, UpdateStatus::kRejected );
  EXPECT_FALSE( second_break.estimate.has_value() );
  EXPECT_EQ( second_break.diagnostics.segment_id, 0U );
}

TEST( StereoVoReanchor, FirstSegmentSeedGate )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size           = 5;
  options.min_shared_landmarks  = 3;
  options.min_seed_observations = 10;

  StereoVoEstimator estimator( calibration, options );

  const std::vector<Eigen::Vector3d> sparse_landmarks(
      kLandmarksA.begin(), kLandmarksA.begin() + 3 );
  const std::vector<LandmarkId> sparse_ids( ids_a.begin(), ids_a.begin() + 3 );
  const auto                    starved = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 50'000'000, sparse_landmarks,
      sparse_ids ) );
  EXPECT_EQ( starved.status, UpdateStatus::kRejected );
  EXPECT_FALSE( starved.estimate.has_value() );
  EXPECT_EQ( starved.message,
             "insufficient observations to seed first segment" );

  const auto seeded = estimator.update( makeFrame(
      calibration, Eigen::Isometry3d::Identity(), 100'000'000, kLandmarksA,
      ids_a ) );
  ASSERT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
  EXPECT_EQ( seeded.diagnostics.segment_id, 0U );
}

TEST( StereoVoReanchor, AnchorFollowsConstantVelocityOption )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );
  const auto poses       = translatingPoses( 4, 0.05 );

  auto run_and_break = [ & ]( bool use_cv ) {
    EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size                = 5;
    options.min_shared_landmarks       = 3;
    options.use_constant_velocity_init = use_cv;

    StereoVoEstimator estimator( calibration, options );
    const auto        accepted = runNormalSegment( estimator, calibration, poses, ids_a );

    const Eigen::Isometry3d& T_prev = accepted[ accepted.size() - 2 ];
    const Eigen::Isometry3d& T_last = accepted.back();
    const Eigen::Isometry3d  expected_anchor =
        use_cv ? T_last * ( T_prev.inverse() * T_last ) : T_last;

    const auto turnover = estimator.update( makeFrame(
        calibration, expected_anchor, 250'000'000, kLandmarksB, ids_b ) );
    return std::make_pair( turnover, expected_anchor );
  };

  const auto [ cv_on_result, cv_on_anchor ]   = run_and_break( true );
  const auto [ cv_off_result, cv_off_anchor ] = run_and_break( false );

  ASSERT_EQ( cv_on_result.status, UpdateStatus::kOk ) << cv_on_result.message;
  ASSERT_TRUE( cv_on_result.estimate.has_value() );
  EXPECT_TRUE( cv_on_result.estimate->T_W_B.matrix().isApprox(
      cv_on_anchor.matrix(), 1e-6 ) );

  ASSERT_EQ( cv_off_result.status, UpdateStatus::kOk ) << cv_off_result.message;
  ASSERT_TRUE( cv_off_result.estimate.has_value() );
  EXPECT_TRUE( cv_off_result.estimate->T_W_B.matrix().isApprox(
      cv_off_anchor.matrix(), 1e-6 ) );

  // The two options must actually disagree here (translation-only motion
  // makes the CV-off anchor lag one step behind the CV-on anchor).
  EXPECT_GT( ( cv_on_anchor.translation() - cv_off_anchor.translation() ).norm(),
             1e-3 );
}

TEST( StereoVoReanchor, CtorRejectsMinSeedObservationsBelowOne )
{
  const auto calibration = makeCalibration();

  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.min_seed_observations = 0;
  EXPECT_THROW( StereoVoEstimator( calibration, options ),
                std::invalid_argument );
}

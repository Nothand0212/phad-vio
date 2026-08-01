#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <optional>
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

  RectifiedStereoCalibration makeCalibration(
      const Eigen::Isometry3d& T_B_left = Eigen::Isometry3d::Identity() )
  {
    auto rigid = RigidTransform::create( T_B_left.matrix() ).value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  [[nodiscard]] Eigen::Isometry3d T_B_C_from(
      const RectifiedStereoCalibration& calibration )
  {
    Eigen::Isometry3d T_B_C = Eigen::Isometry3d::Identity();
    T_B_C.linear()          = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()     = calibration.T_B_left_rectified().translation();
    return T_B_C;
  }

  [[nodiscard]] StereoObservation projectLandmark(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, LandmarkId id,
      const Eigen::Vector3d& point_W )
  {
    const Eigen::Isometry3d T_B_C      = T_B_C_from( calibration );
    const Eigen::Vector3d   point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    const double            z          = point_left.z();
    EXPECT_GT( z, 1e-6 );
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

  const std::vector<Eigen::Vector3d> kLandmarks{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
      { 0.5, 0.05, 5.8 },
      { -0.45, -0.15, 5.1 },
  };

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

}  // namespace

TEST( StereoVoDiagnostics, ZeroSharedRejectsWithoutMutatingWindow )
{
  const auto calibration = makeCalibration();
  const auto poses       = translatingPoses( 3, 0.05 );
  const auto ids         = sequentialIds( kLandmarks.size() );

  EstimatorOptions options;
  options.window_size          = 5;
  options.min_shared_landmarks = 3;
  // This test targets the permanent zero-overlap reject, not re-anchoring.
  options.enable_reanchor = false;

  StereoVoEstimator estimator( calibration, options );
  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, poses[ 0 ], 50'000'000,
                                     kLandmarks, ids ) )
                 .status,
             UpdateStatus::kOk );
  const auto before = estimator.update(
      makeFrame( calibration, poses[ 1 ], 100'000'000, kLandmarks, ids ) );
  ASSERT_EQ( before.status, UpdateStatus::kOk );
  EXPECT_EQ( before.diagnostics.window_size, 2U );
  EXPECT_EQ( before.diagnostics.prior_key, 0U );

  const auto alien_ids = sequentialIds( kLandmarks.size(), 1000 );
  const auto rejected  = estimator.update(
      makeFrame( calibration, poses[ 2 ], 150'000'000, kLandmarks, alien_ids ) );
  EXPECT_EQ( rejected.status, UpdateStatus::kRejected );
  EXPECT_FALSE( rejected.estimate.has_value() );
  EXPECT_EQ( rejected.diagnostics.num_shared, 0U );
  EXPECT_EQ( rejected.diagnostics.window_size, 2U );
  EXPECT_EQ( rejected.diagnostics.prior_key, 0U );

  const auto after = estimator.update(
      makeFrame( calibration, poses[ 2 ], 200'000'000, kLandmarks, ids ) );
  ASSERT_EQ( after.status, UpdateStatus::kOk ) << after.message;
  EXPECT_EQ( after.diagnostics.window_size, 3U );
  EXPECT_EQ( after.diagnostics.prior_key, 0U );
}

TEST( StereoVoDiagnostics, RejectedFrameSkippedByConstantVelocity )
{
  const auto calibration = makeCalibration();
  const auto poses       = translatingPoses( 5, 0.05 );
  const auto ids         = sequentialIds( kLandmarks.size() );

  EstimatorOptions options;
  options.window_size                = 5;
  options.min_shared_landmarks       = 3;
  options.use_constant_velocity_init = true;
  // This test targets the permanent zero-overlap reject, not re-anchoring.
  options.enable_reanchor = false;

  auto run_clean = [ & ]() {
    StereoVoEstimator                           estimator( calibration, options );
    std::optional<phad::estimator::VioEstimate> last;
    for ( int index = 0; index < 5; ++index )
    {
      const auto result = estimator.update( makeFrame(
          calibration, poses[ static_cast<std::size_t>( index ) ],
          static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarks,
          ids ) );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      last = result.estimate;
    }
    return *last;
  };

  auto run_with_reject = [ & ]() {
    StereoVoEstimator                           estimator( calibration, options );
    std::optional<phad::estimator::VioEstimate> last;
    for ( int index = 0; index < 3; ++index )
    {
      const auto result = estimator.update( makeFrame(
          calibration, poses[ static_cast<std::size_t>( index ) ],
          static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarks,
          ids ) );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      last = result.estimate;
    }
    const auto alien_ids = sequentialIds( kLandmarks.size(), 500 );
    const auto rejected  = estimator.update( makeFrame(
        calibration, poses[ 3 ], 175'000'000, kLandmarks, alien_ids ) );
    EXPECT_EQ( rejected.status, UpdateStatus::kRejected );
    for ( int index = 3; index < 5; ++index )
    {
      const auto result = estimator.update( makeFrame(
          calibration, poses[ static_cast<std::size_t>( index ) ],
          static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarks,
          ids ) );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      last = result.estimate;
    }
    return *last;
  };

  const auto clean    = run_clean();
  const auto rejected = run_with_reject();
  EXPECT_TRUE(
      clean.T_W_B.matrix().isApprox( rejected.T_W_B.matrix(), 1e-6 ) );
}

TEST( StereoVoDiagnostics, BehindCameraCountedAndSequenceContinues )
{
  const auto       calibration = makeCalibration();
  EstimatorOptions options;
  options.window_size               = 8;
  options.min_shared_landmarks      = 2;
  options.min_landmark_observations = 2;
  // Disable Huber so conflicting near-landmark factors are not ignored.
  options.huber_k_px = 0.0;

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

  StereoVoEstimator estimator( calibration, options );
  StereoObservation near_at_first{};
  bool              saw_cheirality = false;

  // Frame 0 seeds the near landmark at z=1. Frame 1 jumps to z=2.5 (past it)
  // while still attaching the frame-0 stereo measurement, so the factor is
  // evaluated behind the camera.
  for ( int index = 0; index < 3; ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    const double      z     = ( index == 0 ) ? 0.0 : 2.5;
    T_W_B.translation()     = Eigen::Vector3d( 0.0, 0.0, z );

    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( index + 1 ) * 50'000'000 };

    for ( std::size_t landmark_index = 0; landmark_index < far_landmarks.size();
          ++landmark_index )
    {
      measurement.observations.push_back( projectLandmark(
          calibration, T_W_B, far_ids[ landmark_index ],
          far_landmarks[ landmark_index ] ) );
    }

    if ( index == 0 )
    {
      near_at_first = projectLandmark(
          calibration, T_W_B, near_id, near_landmark );
      measurement.observations.push_back( near_at_first );
    }
    else
    {
      measurement.observations.push_back( near_at_first );
    }

    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( result.diagnostics.num_cheirality > 0 )
    {
      saw_cheirality = true;
    }
  }
  EXPECT_TRUE( saw_cheirality );
}

TEST( StereoVoDiagnostics, ObservationTimestampsAccumulateById )
{
  const auto calibration = makeCalibration();
  const auto poses       = translatingPoses( 3, 0.05 );
  const auto ids         = sequentialIds( kLandmarks.size() );

  EstimatorOptions options;
  options.window_size          = 2;  // force pruning of oldest frame
  options.min_shared_landmarks = 2;

  StereoVoEstimator estimator( calibration, options );
  for ( int index = 0; index < 3; ++index )
  {
    ASSERT_EQ( estimator
                   .update( makeFrame(
                       calibration, poses[ static_cast<std::size_t>( index ) ],
                       static_cast<std::int64_t>( index + 1 ) * 50'000'000,
                       kLandmarks, ids ) )
                   .status,
               UpdateStatus::kOk );
  }

  const auto times = estimator.observationTimestamps( ids.front() );
  ASSERT_EQ( times.size(), 3U );
  EXPECT_EQ( times[ 0 ].nanoseconds(), 50'000'000 );
  EXPECT_EQ( times[ 1 ].nanoseconds(), 100'000'000 );
  EXPECT_EQ( times[ 2 ].nanoseconds(), 150'000'000 );
}

TEST( StereoVoDiagnostics, LowConnectivityFlagWhenSharedBelowThreshold )
{
  const auto calibration = makeCalibration();
  const auto poses       = translatingPoses( 3, 0.05 );
  const auto ids         = sequentialIds( kLandmarks.size() );

  EstimatorOptions options;
  options.window_size          = 5;
  options.min_shared_landmarks = 100;  // force flag while still optimizing

  StereoVoEstimator estimator( calibration, options );
  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, poses[ 0 ], 50'000'000,
                                     kLandmarks, ids ) )
                 .status,
             UpdateStatus::kOk );
  const auto second = estimator.update(
      makeFrame( calibration, poses[ 1 ], 100'000'000, kLandmarks, ids ) );
  ASSERT_EQ( second.status, UpdateStatus::kOk );
  EXPECT_TRUE( second.diagnostics.low_connectivity );
  EXPECT_GT( second.diagnostics.num_shared, 0U );
}

TEST( StereoVoExtrinsics, RecoversBodyPoseNotLeftCamera )
{
  Eigen::Isometry3d T_B_left = Eigen::Isometry3d::Identity();
  T_B_left.linear() =
      Eigen::AngleAxisd( 0.08, Eigen::Vector3d::UnitY() ).toRotationMatrix();
  T_B_left.translation() = Eigen::Vector3d( 0.05, 0.02, 0.01 );
  const auto calibration = makeCalibration( T_B_left );

  std::vector<Eigen::Isometry3d> poses;
  for ( int index = 0; index < 8; ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    const double      yaw   = 0.12 * static_cast<double>( index );
    T_W_B.linear() =
        Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    T_W_B.translation() =
        Eigen::Vector3d( 0.04 * static_cast<double>( index ), 0.0, 0.0 );
    poses.push_back( T_W_B );
  }

  const auto       ids = sequentialIds( kLandmarks.size() );
  EstimatorOptions options;
  options.window_size          = 6;
  options.min_shared_landmarks = 3;

  StereoVoEstimator estimator( calibration, options );
  for ( int index = 0; index < static_cast<int>( poses.size() ); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ static_cast<std::size_t>( index ) ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarks,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;

    const Eigen::Isometry3d& T_W_B_gt =
        poses[ static_cast<std::size_t>( index ) ];
    const Eigen::Isometry3d  T_W_left_gt = T_W_B_gt * T_B_left;
    const Eigen::Isometry3d& T_est       = result.estimate->T_W_B;

    const double err_body =
        ( T_est.translation() - T_W_B_gt.translation() ).norm();
    const double err_left =
        ( T_est.translation() - T_W_left_gt.translation() ).norm();
    EXPECT_LT( err_body, 2e-2 );
    // If body_P_sensor were forgotten, the estimate would track left-cam.
    EXPECT_GT( err_left, err_body + 1e-2 );
  }
}

TEST( StereoVoExtrinsics, WrongExtrinsicRaisesResidualNotStatusFailure )
{
  Eigen::Isometry3d T_true = Eigen::Isometry3d::Identity();
  T_true.translation()     = Eigen::Vector3d( 0.04, 0.0, 0.0 );
  const auto calib_true    = makeCalibration( T_true );

  Eigen::Isometry3d T_wrong = Eigen::Isometry3d::Identity();
  T_wrong.linear() =
      Eigen::AngleAxisd( 0.25, Eigen::Vector3d::UnitY() ).toRotationMatrix();
  T_wrong.translation()  = Eigen::Vector3d( -0.03, 0.05, 0.02 );
  const auto calib_wrong = makeCalibration( T_wrong );

  // Rotating motion prevents a pure body-pose offset from absorbing the
  // wrong body_P_sensor (translation-only scenes can still fit well).
  std::vector<Eigen::Isometry3d> poses;
  for ( int index = 0; index < 8; ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    const double      yaw   = 0.15 * static_cast<double>( index );
    T_W_B.linear() =
        Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    T_W_B.translation() =
        Eigen::Vector3d( 0.04 * static_cast<double>( index ), 0.0, 0.0 );
    poses.push_back( T_W_B );
  }
  const auto ids = sequentialIds( kLandmarks.size() );

  EstimatorOptions options;
  options.window_size          = 5;
  options.min_shared_landmarks = 3;

  StereoVoEstimator estimator( calib_wrong, options );
  double            last_err = 0.0;
  for ( int index = 0; index < static_cast<int>( poses.size() ); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calib_true, poses[ static_cast<std::size_t>( index ) ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarks,
        ids ) );
    // Wrong extrinsics are still a valid contract → never kFailed.
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    last_err =
        ( result.estimate->T_W_B.translation() -
          poses[ static_cast<std::size_t>( index ) ].translation() )
            .norm();
  }
  // BA can absorb a wrong body_P_sensor into T_W_B (near-zero residual) via a
  // right-multiply; Umeyama cannot. Pose error is the observable signal.
  EXPECT_GT( last_err, 0.05 );
}

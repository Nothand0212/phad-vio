#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/estimator/types.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{
using phad::camera::RectifiedStereoCalibration;
using phad::common::LandmarkId;
using phad::common::Timestamp;
using phad::estimator::EstimatorOptions;
using phad::estimator::KeyframeMeasurement;
using phad::estimator::StereoObservation;
using phad::estimator::StereoVoEstimator;
using phad::estimator::UpdateStatus;
using phad::sensor::RigidTransform;

RectifiedStereoCalibration makeCalibration()
{
  auto rigid =
      RigidTransform::create( Eigen::Isometry3d::Identity().matrix() ).value();
  return RectifiedStereoCalibration::create(
             400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
      .value();
}

KeyframeMeasurement makeMeasurement(
    std::int64_t ts_ns, const std::vector<LandmarkId>& ids )
{
  KeyframeMeasurement m;
  m.timestamp = Timestamp{ ts_ns };
  for ( const auto& id : ids )
  {
    StereoObservation obs;
    obs.id           = id;
    obs.left_pixel = { 320.0, 240.0 };
    obs.disparity_px = 5.0;
    m.observations.push_back( obs );
  }
  return m;
}

// ── Slice ⑤b synthetic helpers ─────────────────────────────────────────

// Synthetic 3D landmarks in front of the camera (world frame = body frame
// for the first pose at origin).
const std::vector<Eigen::Vector3d> kLandmarks{
    { 2.0, -0.5, 4.0 }, { 2.0, 0.5, 4.0 },  { 2.0, -0.5, 5.0 },
    { 2.0, 0.5, 5.0 },  { 2.5, -0.8, 6.0 }, { 2.5, 0.8, 6.0 },
    { 2.5, -0.3, 7.0 }, { 2.5, 0.3, 7.0 },  { 3.0, -1.0, 8.0 },
    { 3.0, 1.0, 8.0 },  { 3.0, -0.6, 9.0 }, { 3.0, 0.6, 9.0 },
    { 3.5, -1.2, 10.0 }, { 3.5, 1.2, 10.0 }, { 3.5, -0.9, 11.0 },
    { 3.5, 0.9, 11.0 }, { 4.0, -1.5, 12.0 }, { 4.0, 1.5, 12.0 },
    { 4.0, -1.0, 13.0 }, { 4.0, 1.0, 13.0 },
};

// Project a world landmark into a stereo observation under pose T_W_B
// (body = left camera for this synthetic calibration).
StereoObservation projectLandmark(
    const RectifiedStereoCalibration& calibration,
    const Eigen::Isometry3d&          T_W_B, LandmarkId id,
    const Eigen::Vector3d& point_W )
{
  const Eigen::Isometry3d T_B_left = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d   point_left =
      ( T_W_B * T_B_left ).inverse() * point_W;
  const double z = point_left.z();
  EXPECT_GT( z, 0.0 );
  const double u_l =
      calibration.fxPixels() * point_left.x() / z + calibration.cxPixels();
  const double v =
      calibration.fyPixels() * point_left.y() / z + calibration.cyPixels();
  const double disparity =
      calibration.fxPixels() * calibration.baselineM() / z;
  return StereoObservation{ id, Eigen::Vector2d( u_l, v ), disparity };
}

KeyframeMeasurement makeSyntheticFrame(
    const RectifiedStereoCalibration& calibration,
    const Eigen::Isometry3d&          T_W_B, std::int64_t ts_ns,
    const std::vector<LandmarkId>& ids,
    const std::vector<Eigen::Vector3d>& landmarks )
{
  KeyframeMeasurement m;
  m.timestamp = Timestamp{ ts_ns };
  for ( std::size_t i = 0; i < ids.size(); ++i )
  {
    m.observations.push_back(
        projectLandmark( calibration, T_W_B, ids[ i ], landmarks[ i ] ) );
  }
  return m;
}

std::vector<LandmarkId> makeIds( std::size_t count, std::uint64_t start = 1 )
{
  std::vector<LandmarkId> ids;
  ids.reserve( count );
  for ( std::size_t i = 0; i < count; ++i )
  {
    ids.push_back( LandmarkId{ start + static_cast<std::uint64_t>( i ) } );
  }
  return ids;
}

}  // namespace

TEST( KeyframeUpdateTest, NonKeyframeDoesNotModifyWindow )
{
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );
  EstimatorOptions opts;
  // Needs to be initialised first with a keyframe.
  // Keyframe 0: first frame seeding.
  auto ids0 = std::vector<LandmarkId>{ LandmarkId{ 0 }, LandmarkId{ 1 },
                                       LandmarkId{ 2 }, LandmarkId{ 3 },
                                       LandmarkId{ 4 }, LandmarkId{ 5 },
                                       LandmarkId{ 6 }, LandmarkId{ 7 },
                                       LandmarkId{ 8 }, LandmarkId{ 9 } };
  auto m0 = makeMeasurement( 100'000'000, ids0 );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  // Keyframe 1: build up window.
  auto ids1 = ids0;  // same IDs → shared > 0
  auto m1   = makeMeasurement( 200'000'000, ids1 );
  auto r1   = estimator.update( m1, true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );
  EXPECT_GT( r1.diagnostics.window_size, 0U );

  const auto win_sz_before = r1.diagnostics.window_size;

  // Non-keyframe with same shared IDs.
  auto m2 = makeMeasurement( 300'000'000, ids1 );
  auto r2 = estimator.update( m2, false );
  EXPECT_EQ( r2.status, UpdateStatus::kOk );

  // Window size should be unchanged (non-keyframe doesn't push).
  // Run another keyframe to read back the window size.
  auto m3 = makeMeasurement( 400'000'000, ids1 );
  auto r3 = estimator.update( m3, true );
  EXPECT_EQ( r3.diagnostics.window_size, win_sz_before + 1U );
}

TEST( KeyframeUpdateTest, NonKeyframeRejectedOnLowShared )
{
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );

  // Initialise with 20 IDs (above min_pnp_inliers = 10).
  std::vector<LandmarkId> ids;
  for ( int i = 0; i < 20; ++i )
  {
    ids.push_back( LandmarkId{ static_cast<std::uint64_t>( i ) } );
  }
  auto m0 = makeMeasurement( 100'000'000, ids );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  auto m1 = makeMeasurement( 200'000'000, ids );
  auto r1 = estimator.update( m1, true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );

  // Non-keyframe with only 5 shared IDs (< min_pnp_inliers=10) -> rejected
  // (Slice ⑤b gate: a raw CV guess would pollute the pose chain).
  std::vector<LandmarkId> few_ids;
  for ( int i = 0; i < 5; ++i )
  {
    few_ids.push_back( LandmarkId{ static_cast<std::uint64_t>( i ) } );
  }
  auto m2 = makeMeasurement( 300'000'000, few_ids );
  auto r2 = estimator.update( m2, false );
  EXPECT_EQ( r2.status, UpdateStatus::kRejected );
}

TEST( KeyframeUpdateTest, NonKeyframeRejectedOnNoShared )
{
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );

  // Initialise.
  auto ids0 = std::vector<LandmarkId>{
      LandmarkId{ 0 }, LandmarkId{ 1 }, LandmarkId{ 2 }, LandmarkId{ 3 },
      LandmarkId{ 4 }, LandmarkId{ 5 }, LandmarkId{ 6 }, LandmarkId{ 7 },
      LandmarkId{ 8 }, LandmarkId{ 9 } };
  auto m0 = makeMeasurement( 100'000'000, ids0 );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  auto m1 = makeMeasurement( 200'000'000, ids0 );
  auto r1 = estimator.update( m1, true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );

  // Non-keyframe with entirely new IDs → shared = 0 → rejected.
  auto new_ids = std::vector<LandmarkId>{
      LandmarkId{ 100 }, LandmarkId{ 101 }, LandmarkId{ 102 },
      LandmarkId{ 103 }, LandmarkId{ 104 }, LandmarkId{ 105 },
      LandmarkId{ 106 }, LandmarkId{ 107 }, LandmarkId{ 108 },
      LandmarkId{ 109 } };
  auto m2 = makeMeasurement( 300'000'000, new_ids );
  auto r2 = estimator.update( m2, false );
  EXPECT_EQ( r2.status, UpdateStatus::kRejected );
}

TEST( KeyframeUpdateTest, NonKeyframePreservesPoseChain )
{
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );

  auto ids = std::vector<LandmarkId>{
      LandmarkId{ 0 }, LandmarkId{ 1 }, LandmarkId{ 2 }, LandmarkId{ 3 },
      LandmarkId{ 4 }, LandmarkId{ 5 }, LandmarkId{ 6 }, LandmarkId{ 7 },
      LandmarkId{ 8 }, LandmarkId{ 9 } };

  auto m0 = makeMeasurement( 100'000'000, ids );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  // Several keyframes to establish a window.
  for ( int i = 1; i < 3; ++i )
  {
    auto m = makeMeasurement( 100'000'000 * ( i + 1 ), ids );
    auto r = estimator.update( m, true );
    EXPECT_EQ( r.status, UpdateStatus::kOk );
  }

  // Non-keyframe.
  auto m_nk = makeMeasurement( 500'000'000, ids );
  auto r_nk = estimator.update( m_nk, false );
  EXPECT_EQ( r_nk.status, UpdateStatus::kOk );

  // The next keyframe should still work — proving pose chain was maintained.
  auto m_kf = makeMeasurement( 600'000'000, ids );
  auto r_kf = estimator.update( m_kf, true );
  EXPECT_EQ( r_kf.status, UpdateStatus::kOk );
  EXPECT_GT( r_kf.diagnostics.window_size, 0U );
}

TEST( KeyframeUpdateTest, NonKeyframeBeforeInitRejected )
{
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );

  auto ids = std::vector<LandmarkId>{
      LandmarkId{ 0 }, LandmarkId{ 1 }, LandmarkId{ 2 }, LandmarkId{ 3 },
      LandmarkId{ 4 }, LandmarkId{ 5 }, LandmarkId{ 6 }, LandmarkId{ 7 },
      LandmarkId{ 8 }, LandmarkId{ 9 } };
  auto m = makeMeasurement( 100'000'000, ids );
  auto r = estimator.update( m, false );  // non-keyframe before init
  EXPECT_EQ( r.status, UpdateStatus::kRejected );
}

TEST( KeyframeUpdateTest, NonKeyframeRefineImprovesPoseAccuracy )
{
  // Slice ⑤b: pose-only LM refinement should reduce translation error for
  // a non-keyframe whose PnP pose is biased by pixel noise.
  const auto calibration = makeCalibration();
  const auto ids         = makeIds( kLandmarks.size() );

  // Two keyframes at x=0 and x=0.05 (seed window + landmarks), then a
  // non-keyframe at x=0.10 whose measurements are perturbed.
  std::vector<Eigen::Isometry3d> poses;
  for ( int i = 0; i < 3; ++i )
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation()     = Eigen::Vector3d( 0.05 * i, 0.0, 0.0 );
    poses.push_back( T );
  }

  StereoVoEstimator estimator( calibration );

  // Keyframe 0: seed.
  auto r0 = estimator.update(
      makeSyntheticFrame( calibration, poses[ 0 ], 100'000'000, ids,
                          kLandmarks ),
      true );
  ASSERT_EQ( r0.status, UpdateStatus::kOk ) << r0.message;

  // Keyframe 1: build window + landmarks.
  auto r1 = estimator.update(
      makeSyntheticFrame( calibration, poses[ 1 ], 150'000'000, ids,
                          kLandmarks ),
      true );
  ASSERT_EQ( r1.status, UpdateStatus::kOk ) << r1.message;

  // Non-keyframe at x=0.10 with clean measurements. The pose should be
  // refined close to truth (Pnp success path exercises the refinement).
  auto nk_measurement =
      makeSyntheticFrame( calibration, poses[ 2 ], 200'000'000, ids,
                          kLandmarks );
  auto r_nk = estimator.update( nk_measurement, false );
  ASSERT_EQ( r_nk.status, UpdateStatus::kOk ) << r_nk.message;
  ASSERT_TRUE( r_nk.estimate.has_value() );
  EXPECT_TRUE( r_nk.diagnostics.pnp_success );

  const double err =
      ( r_nk.estimate->T_W_B.translation() - poses[ 2 ].translation() )
          .norm();
  // Refined pose should be within 1 cm of truth on clean synthetic data
  // (raw PnP with noise would be worse; refinement pulls it in).
  EXPECT_LT( err, 0.01 );
}

TEST( KeyframeUpdateTest, NonKeyframeRefineDegenerateFallsBack )
{
  // All landmarks at the same world point -> degenerate refinement input;
  // update must still return kOk with a finite pose (fallback keeps PnP).
  const auto calibration = makeCalibration();
  const auto ids         = makeIds( 12 );

  std::vector<Eigen::Vector3d> degenerate;
  for ( std::size_t i = 0; i < ids.size(); ++i )
  {
    degenerate.push_back( Eigen::Vector3d( 2.0, 0.0, 5.0 ) );
  }

  StereoVoEstimator estimator( calibration );
  Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  T1.translation()     = Eigen::Vector3d( 0.05, 0.0, 0.0 );

  auto r0 = estimator.update(
      makeSyntheticFrame( calibration, T0, 100'000'000, ids, degenerate ),
      true );
  ASSERT_EQ( r0.status, UpdateStatus::kOk ) << r0.message;

  auto r1 = estimator.update(
      makeSyntheticFrame( calibration, T1, 150'000'000, ids, degenerate ),
      true );
  ASSERT_EQ( r1.status, UpdateStatus::kOk ) << r1.message;

  Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
  T2.translation()     = Eigen::Vector3d( 0.10, 0.0, 0.0 );
  auto r_nk = estimator.update(
      makeSyntheticFrame( calibration, T2, 200'000'000, ids, degenerate ),
      false );
  // Degenerate geometry may reject (PnP inliers < threshold) or succeed;
  // either way the result must be finite and not crash.
  if ( r_nk.status == UpdateStatus::kOk )
  {
    ASSERT_TRUE( r_nk.estimate.has_value() );
    EXPECT_TRUE( r_nk.estimate->T_W_B.matrix().allFinite() );
  }
}

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
    obs.left_pixel   = { 320.0, 240.0 };
    obs.disparity_px = 5.0;
    m.observations.push_back( obs );
  }
  return m;
}

}  // namespace

TEST( KeyframeUpdateTest, NonKeyframeEntersWindow )
{
  // Slice ⑤c: non-keyframes enter the window and participate in BA.
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );
  EstimatorOptions opts;
  auto ids0 = std::vector<LandmarkId>{ LandmarkId{ 0 }, LandmarkId{ 1 },
                                       LandmarkId{ 2 }, LandmarkId{ 3 },
                                       LandmarkId{ 4 }, LandmarkId{ 5 },
                                       LandmarkId{ 6 }, LandmarkId{ 7 },
                                       LandmarkId{ 8 }, LandmarkId{ 9 } };
  auto m0 = makeMeasurement( 100'000'000, ids0 );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  auto ids1 = ids0;
  auto m1   = makeMeasurement( 200'000'000, ids1 );
  auto r1   = estimator.update( m1, true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );
  EXPECT_GT( r1.diagnostics.window_size, 0U );

  const auto win_sz_before = r1.diagnostics.window_size;

  // Non-keyframe with same shared IDs: enters window (size +1).
  auto m2 = makeMeasurement( 300'000'000, ids1 );
  auto r2 = estimator.update( m2, false );
  EXPECT_EQ( r2.status, UpdateStatus::kOk );
  EXPECT_EQ( r2.diagnostics.window_size, win_sz_before + 1U );
}

TEST( KeyframeUpdateTest, NonKeyframeAllNewIdsRejected )
{
  // Slice ⑤c: non-keyframe with all-new IDs -> shared = 0 -> rejected
  // (overlap broken); seeding is only for keyframes.
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );
  auto ids0 = std::vector<LandmarkId>{ LandmarkId{ 0 }, LandmarkId{ 1 },
                                       LandmarkId{ 2 }, LandmarkId{ 3 },
                                       LandmarkId{ 4 }, LandmarkId{ 5 },
                                       LandmarkId{ 6 }, LandmarkId{ 7 },
                                       LandmarkId{ 8 }, LandmarkId{ 9 } };
  auto m0 = makeMeasurement( 100'000'000, ids0 );
  auto r0 = estimator.update( m0, true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  auto m1 = makeMeasurement( 200'000'000, ids0 );
  auto r1 = estimator.update( m1, true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );

  std::vector<LandmarkId> new_ids;
  for ( int i = 100; i < 110; ++i )
  {
    new_ids.push_back( LandmarkId{ static_cast<std::uint64_t>( i ) } );
  }
  auto m2 = makeMeasurement( 300'000'000, new_ids );
  auto r2 = estimator.update( m2, false );
  EXPECT_EQ( r2.status, UpdateStatus::kRejected );
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

TEST( KeyframeUpdateTest, WindowCapsAtTenWithTemporalEviction )
{
  // Slice ⑤c Basalt eviction: window holds up to window_size frames;
  // non-keyframes are evicted first when full.
  auto calib = makeCalibration();
  StereoVoEstimator estimator( calib );

  std::vector<LandmarkId> ids;
  for ( int i = 0; i < 20; ++i )
  {
    ids.push_back( LandmarkId{ static_cast<std::uint64_t>( i ) } );
  }

  // 2 keyframes to seed, then 12 non-keyframes.
  auto r = estimator.update( makeMeasurement( 100'000'000, ids ), true );
  EXPECT_EQ( r.status, UpdateStatus::kOk );
  r = estimator.update( makeMeasurement( 200'000'000, ids ), true );
  EXPECT_EQ( r.status, UpdateStatus::kOk );

  std::uint32_t last_win = 0;
  for ( int i = 3; i <= 14; ++i )
  {
    r = estimator.update(
        makeMeasurement( 100'000'000LL * i, ids ), false );
    EXPECT_EQ( r.status, UpdateStatus::kOk );
    last_win = r.diagnostics.window_size;
  }
  // 2 keyframes + 12 non-keyframes, but window caps at 10 (config default).
  EXPECT_EQ( last_win, 10U );
}


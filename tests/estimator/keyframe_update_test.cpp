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

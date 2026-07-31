#include <gtest/gtest.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <cmath>
#include <memory>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration()
  {
    auto T_B_left =
        RigidTransform::create( Eigen::Matrix4d::Identity() ).value();
    return RectifiedStereoCalibration::create(
               458.654, 457.296, 367.215, 248.375, 0.110074, 752, 480,
               std::move( T_B_left ) )
        .value();
  }

  gtsam::StereoPoint2 toStereoPoint( double u_l, double v, double disparity_px )
  {
    return gtsam::StereoPoint2( u_l, u_l - disparity_px, v );
  }

}  // namespace

TEST( StereoPointMapping, ProjectBackprojectRoundTrip )
{
  const RectifiedStereoCalibration calibration = makeCalibration();
  const auto                       K           = std::make_shared<gtsam::Cal3_S2Stereo>(
      calibration.fxPixels(), calibration.fyPixels(), 0.0,
      calibration.cxPixels(), calibration.cyPixels(),
      calibration.baselineM() );

  const Eigen::Vector3d point_left( 0.35, -0.12, 4.5 );
  const double          u_l =
      calibration.fxPixels() * point_left.x() / point_left.z() +
      calibration.cxPixels();
  const double v =
      calibration.fyPixels() * point_left.y() / point_left.z() +
      calibration.cyPixels();
  const double disparity_px =
      calibration.fxPixels() * calibration.baselineM() / point_left.z();

  const gtsam::StereoCamera camera( gtsam::Pose3(), K );
  const gtsam::StereoPoint2 measured =
      toStereoPoint( u_l, v, disparity_px );
  const gtsam::Point3       recovered   = camera.backproject( measured );
  const gtsam::StereoPoint2 reprojected = camera.project( recovered );

  EXPECT_NEAR( recovered.x(), point_left.x(), 1e-9 );
  EXPECT_NEAR( recovered.y(), point_left.y(), 1e-9 );
  EXPECT_NEAR( recovered.z(), point_left.z(), 1e-9 );
  EXPECT_NEAR( reprojected.uL() - reprojected.uR(),
               calibration.fxPixels() * calibration.baselineM() /
                   point_left.z(),
               1e-9 );
  EXPECT_NEAR( measured.uL() - measured.uR(), disparity_px, 1e-12 );
}

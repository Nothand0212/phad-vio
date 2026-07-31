#include <gtsam/geometry/Pose3.h>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>

namespace
{

  gtsam::Pose3 poseFromIsometry( const Eigen::Isometry3d& T_a_b )
  {
    return gtsam::Pose3( T_a_b.matrix() );
  }

  Eigen::Isometry3d isometryFromPose( const gtsam::Pose3& pose )
  {
    Eigen::Isometry3d T_a_b = Eigen::Isometry3d::Identity();
    T_a_b.linear()          = pose.rotation().matrix();
    T_a_b.translation()     = pose.translation();
    return T_a_b;
  }

}  // namespace

TEST( GtsamSmoke, Pose3Constructs )
{
  const gtsam::Pose3 pose;
  EXPECT_TRUE( pose.rotation().matrix().isIdentity( 1e-12 ) );
  EXPECT_TRUE( pose.translation().isZero( 1e-12 ) );
}

TEST( GtsamSmoke, EigenIsometryRoundTrip )
{
  Eigen::Isometry3d T_w_b = Eigen::Isometry3d::Identity();
  T_w_b.linear() =
      Eigen::AngleAxisd( 0.3, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
  T_w_b.translation() = Eigen::Vector3d( 1.25, -0.5, 2.0 );

  const gtsam::Pose3       pose   = poseFromIsometry( T_w_b );
  const Eigen::Isometry3d  T_back = isometryFromPose( pose );

  EXPECT_TRUE( T_back.matrix().isApprox( T_w_b.matrix(), 1e-12 ) );

  // matrix() path must agree with the component-wise path (same Eigen).
  const Eigen::Matrix4d from_gtsam = pose.matrix();
  EXPECT_TRUE( from_gtsam.isApprox( T_w_b.matrix(), 1e-12 ) );
  EXPECT_FALSE( std::isnan( from_gtsam( 0, 3 ) ) );
}

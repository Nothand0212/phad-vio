#include "phad/camera/rectified_stereo_calibration.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::sensor::CalibrationErrorCode;
  using phad::sensor::CalibrationResult;
  using phad::sensor::RigidTransform;

  Eigen::Matrix4d identityTransform()
  {
    return Eigen::Matrix4d::Identity();
  }

  RectifiedStereoCalibration makeValidCalibration()
  {
    auto T_B_left = RigidTransform::create( identityTransform() );
    EXPECT_TRUE( T_B_left );
    auto calibration = RectifiedStereoCalibration::create(
        458.654, 457.296, 367.215, 248.375, 0.110074, 752, 480,
        std::move( T_B_left ).value() );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  Eigen::Vector2d projectLeft( const RectifiedStereoCalibration& calib,
                               const Eigen::Vector3d&            point_left )
  {
    const double z = point_left.z();
    return { calib.fxPixels() * point_left.x() / z + calib.cxPixels(),
             calib.fyPixels() * point_left.y() / z + calib.cyPixels() };
  }

  Eigen::Vector2d projectRight( const RectifiedStereoCalibration& calib,
                                const Eigen::Vector3d&            point_left )
  {
    const Eigen::Vector3d point_right(
        point_left.x() - calib.baselineM(), point_left.y(), point_left.z() );
    const double z = point_right.z();
    return { calib.fxPixels() * point_right.x() / z + calib.cxPixels(),
             calib.fyPixels() * point_right.y() / z + calib.cyPixels() };
  }

  TEST( RectifiedStereoCalibrationTest, AcceptsValidInputs )
  {
    const RectifiedStereoCalibration calib = makeValidCalibration();
    EXPECT_DOUBLE_EQ( calib.fxPixels(), 458.654 );
    EXPECT_DOUBLE_EQ( calib.fyPixels(), 457.296 );
    EXPECT_DOUBLE_EQ( calib.cxPixels(), 367.215 );
    EXPECT_DOUBLE_EQ( calib.cyPixels(), 248.375 );
    EXPECT_DOUBLE_EQ( calib.baselineM(), 0.110074 );
    EXPECT_EQ( calib.imageWidth(), 752 );
    EXPECT_EQ( calib.imageHeight(), 480 );
    EXPECT_TRUE( calib.T_B_left_rectified().rotation().isIdentity( 1e-12 ) );
    EXPECT_TRUE(
        calib.T_B_left_rectified().translation().isZero( 1e-12 ) );
  }

  TEST( RectifiedStereoCalibrationTest, ProjectsWithAlignedRowsAndDisparity )
  {
    const RectifiedStereoCalibration calib = makeValidCalibration();
    const std::vector<Eigen::Vector3d> points{
        { 0.0, 0.0, 2.0 },
        { 0.4, -0.2, 3.5 },
        { -0.3, 0.15, 1.2 },
        { 0.1, 0.05, 8.0 },
    };

    for ( const Eigen::Vector3d& point : points )
    {
      const Eigen::Vector2d left  = projectLeft( calib, point );
      const Eigen::Vector2d right = projectRight( calib, point );
      EXPECT_NEAR( left.y(), right.y(), 1e-12 );
      const double disparity = left.x() - right.x();
      const double expected =
          calib.fxPixels() * calib.baselineM() / point.z();
      EXPECT_NEAR( disparity, expected, 1e-12 );
    }
  }

  TEST( RectifiedStereoCalibrationTest, RejectsNonFiniteIntrinsics )
  {
    auto T_B_left = RigidTransform::create( identityTransform() );
    ASSERT_TRUE( T_B_left );
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto result      = RectifiedStereoCalibration::create(
        nan, 457.0, 367.0, 248.0, 0.11, 752, 480,
        std::move( T_B_left ).value() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, CalibrationErrorCode::kNonFiniteValue );
    EXPECT_EQ( result.error().field_path, "rectified.fx_pixels" );
  }

  TEST( RectifiedStereoCalibrationTest, RejectsNonPositiveFocalLength )
  {
    auto T_B_left = RigidTransform::create( identityTransform() );
    ASSERT_TRUE( T_B_left );
    auto result = RectifiedStereoCalibration::create(
        0.0, 457.0, 367.0, 248.0, 0.11, 752, 480,
        std::move( T_B_left ).value() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               CalibrationErrorCode::kNonPositiveValue );
    EXPECT_EQ( result.error().field_path, "rectified.fx_pixels" );
  }

  TEST( RectifiedStereoCalibrationTest, RejectsNonPositiveBaseline )
  {
    auto T_B_left = RigidTransform::create( identityTransform() );
    ASSERT_TRUE( T_B_left );
    auto result = RectifiedStereoCalibration::create(
        458.0, 457.0, 367.0, 248.0, 0.0, 752, 480,
        std::move( T_B_left ).value() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               CalibrationErrorCode::kZeroStereoBaseline );
    EXPECT_EQ( result.error().field_path, "rectified.baseline_m" );
  }

  TEST( RectifiedStereoCalibrationTest, RejectsNonPositiveImageSize )
  {
    auto T_B_left = RigidTransform::create( identityTransform() );
    ASSERT_TRUE( T_B_left );
    const RigidTransform transform = std::move( T_B_left ).value();

    auto bad_width = RectifiedStereoCalibration::create(
        458.0, 457.0, 367.0, 248.0, 0.11, 0, 480, transform );
    ASSERT_FALSE( bad_width );
    EXPECT_EQ( bad_width.error().code,
               CalibrationErrorCode::kNonPositiveValue );
    EXPECT_EQ( bad_width.error().field_path, "rectified.image_width" );

    auto bad_height = RectifiedStereoCalibration::create(
        458.0, 457.0, 367.0, 248.0, 0.11, 752, -1, transform );
    ASSERT_FALSE( bad_height );
    EXPECT_EQ( bad_height.error().code,
               CalibrationErrorCode::kNonPositiveValue );
    EXPECT_EQ( bad_height.error().field_path, "rectified.image_height" );
  }

}  // namespace

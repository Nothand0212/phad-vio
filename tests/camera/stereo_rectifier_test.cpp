#include "phad/camera/stereo_rectifier.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{

  using phad::camera::CameraModelErrorCode;
  using phad::camera::StereoRectifier;
  using phad::sensor::CameraModelParameters;
  using phad::sensor::CameraParameters;
  using phad::sensor::Image;
  using phad::sensor::ImuParameters;
  using phad::sensor::PinholeEquidistantParameters;
  using phad::sensor::PinholeRadialTangentialParameters;
  using phad::sensor::RigidTransform;
  using phad::sensor::StereoFrame;
  using phad::sensor::StereoImuCalibration;

  Eigen::Matrix4d translationX( double x_m )
  {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix( 0, 3 )         = x_m;
    return matrix;
  }

  CameraParameters makeRadtanCamera( double fx, double fy, double cx,
                                     double cy, int width, int height )
  {
    auto model =
        PinholeRadialTangentialParameters::create( fx, fy, cx, cy, 0.0, 0.0,
                                                   0.0, 0.0 );
    EXPECT_TRUE( model );
    auto camera = CameraParameters::create(
        CameraModelParameters{ std::move( model ).value() }, width, height,
        20.0 );
    EXPECT_TRUE( camera );
    return std::move( camera ).value();
  }

  CameraParameters makeEquidistantCamera( int width, int height )
  {
    auto model = PinholeEquidistantParameters::create(
        190.0, 191.0, 255.0, 256.0, 0.01, -0.02, 0.003, -0.0004 );
    EXPECT_TRUE( model );
    auto camera = CameraParameters::create(
        CameraModelParameters{ std::move( model ).value() }, width, height,
        20.0 );
    EXPECT_TRUE( camera );
    return std::move( camera ).value();
  }

  ImuParameters makeImu()
  {
    auto imu =
        ImuParameters::create( 200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 );
    EXPECT_TRUE( imu );
    return std::move( imu ).value();
  }

  RigidTransform makeTranslation( double x_m )
  {
    auto result = RigidTransform::create( translationX( x_m ) );
    EXPECT_TRUE( result );
    return std::move( result ).value();
  }

  StereoImuCalibration makeIdentityStereo( double baseline_m = 0.11 )
  {
    // fx==fy and a principal point inside the image: OpenCV stereoRectify
    // otherwise averages focals / rescales under alpha=0.
    constexpr double kF  = 458.0;
    constexpr double kCx = 367.0;
    constexpr double kCy = 248.0;
    constexpr int    kW  = 752;
    constexpr int    kH  = 480;

    auto calibration = StereoImuCalibration::create(
        makeRadtanCamera( kF, kF, kCx, kCy, kW, kH ),
        makeRadtanCamera( kF, kF, kCx, kCy, kW, kH ), makeImu(),
        makeTranslation( 0.0 ), makeTranslation( baseline_m ) );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  Image makeRampImage( int width, int height )
  {
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>( width * height ) );
    for ( int y = 0; y < height; ++y )
    {
      for ( int x = 0; x < width; ++x )
      {
        pixels[ static_cast<std::size_t>( y * width + x ) ] =
            static_cast<std::uint8_t>( ( x * 3 + y * 5 ) & 0xFF );
      }
    }
    return Image{ width, height, 1, std::move( pixels ) };
  }

  TEST( StereoRectifierTest, IdentityMappingForZeroDistortionPureTranslation )
  {
    constexpr double kBaseline = 0.11;
    const StereoImuCalibration calibration = makeIdentityStereo( kBaseline );

    auto rectifier = StereoRectifier::create( calibration );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;

    const auto& rectified = rectifier.value().calibration();
    EXPECT_NEAR( rectified.fxPixels(), 458.0, 1e-6 );
    EXPECT_NEAR( rectified.fyPixels(), 458.0, 1e-6 );
    EXPECT_NEAR( rectified.cxPixels(), 367.0, 1e-6 );
    EXPECT_NEAR( rectified.cyPixels(), 248.0, 1e-6 );
    EXPECT_NEAR( rectified.baselineM(), kBaseline, 1e-9 );
    EXPECT_EQ( rectified.imageWidth(), 752 );
    EXPECT_EQ( rectified.imageHeight(), 480 );

    const Image left  = makeRampImage( 752, 480 );
    const Image right = makeRampImage( 752, 480 );
    const StereoFrame raw{ phad::common::Timestamp{ 1 }, left, right };

    auto out = rectifier.value().rectify( raw );
    ASSERT_TRUE( out ) << out.error().detail;
    EXPECT_EQ( out.value().timestamp, raw.timestamp );

    const auto left_in   = raw.left.pixels<std::uint8_t>();
    const auto right_in  = raw.right.pixels<std::uint8_t>();
    const auto left_out  = out.value().left.pixels<std::uint8_t>();
    const auto right_out = out.value().right.pixels<std::uint8_t>();
    ASSERT_TRUE( left_in.has_value() );
    ASSERT_TRUE( right_in.has_value() );
    ASSERT_TRUE( left_out.has_value() );
    ASSERT_TRUE( right_out.has_value() );
    ASSERT_EQ( left_out->size(), left_in->size() );
    ASSERT_EQ( right_out->size(), right_in->size() );
    EXPECT_TRUE( std::equal( left_out->begin(), left_out->end(),
                             left_in->begin() ) );
    EXPECT_TRUE( std::equal( right_out->begin(), right_out->end(),
                             right_in->begin() ) );
  }

  TEST( StereoRectifierTest, RejectsEquidistantCalibration )
  {
    auto calibration = StereoImuCalibration::create(
        makeEquidistantCamera( 512, 512 ), makeEquidistantCamera( 512, 512 ),
        makeImu(), makeTranslation( 0.0 ), makeTranslation( 0.11 ) );
    ASSERT_TRUE( calibration );

    auto rectifier = StereoRectifier::create( calibration.value() );
    ASSERT_FALSE( rectifier );
    EXPECT_EQ( rectifier.error().code,
               CameraModelErrorCode::kOutsideModelDomain );
  }

  TEST( StereoRectifierTest, RejectsMismatchedFrameSize )
  {
    auto rectifier = StereoRectifier::create( makeIdentityStereo() );
    ASSERT_TRUE( rectifier );

    const StereoFrame raw{ phad::common::Timestamp{ 2 },
                           makeRampImage( 32, 24 ), makeRampImage( 32, 24 ) };
    auto out = rectifier.value().rectify( raw );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kOutsideModelDomain );
  }

}  // namespace

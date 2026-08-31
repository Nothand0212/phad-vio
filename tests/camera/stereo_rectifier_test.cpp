#include "phad/camera/stereo_rectifier.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

  using phad::camera::CameraModelErrorCode;
  using phad::camera::createCameraModel;
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

  CameraParameters makeDistortedRadtanCamera( double fx, double fy,
                                              double cx, double cy, int width,
                                              int height )
  {
    auto model = PinholeRadialTangentialParameters::create(
        fx, fy, cx, cy, -0.18, 0.03, 0.002, -0.001 );
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

  RigidTransform makeTransform( const Eigen::Matrix3d& rotation,
                                const Eigen::Vector3d& translation )
  {
    Eigen::Matrix4d matrix     = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>( 0, 0 ) = rotation;
    matrix.block<3, 1>( 0, 3 ) = translation;
    auto result                = RigidTransform::create( matrix );
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

  StereoImuCalibration makeSmallIdentityStereo()
  {
    constexpr double kF  = 10.0;
    constexpr double kCx = 3.0;
    constexpr double kCy = 0.0;
    constexpr int    kW  = 7;
    constexpr int    kH  = 1;

    auto calibration = StereoImuCalibration::create(
        makeRadtanCamera( kF, kF, kCx, kCy, kW, kH ),
        makeRadtanCamera( kF, kF, kCx, kCy, kW, kH ), makeImu(),
        makeTranslation( 0.0 ), makeTranslation( 0.11 ) );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  StereoImuCalibration makeFractionalRadtanStereo()
  {
    constexpr double kF  = 25.0;
    constexpr double kCx = 15.5;
    constexpr double kCy = 11.5;
    constexpr int    kW  = 32;
    constexpr int    kH  = 24;

    auto calibration = StereoImuCalibration::create(
        makeDistortedRadtanCamera( kF, kF, kCx, kCy, kW, kH ),
        makeDistortedRadtanCamera( kF, kF, kCx, kCy, kW, kH ), makeImu(),
        makeTranslation( 0.0 ), makeTranslation( 0.11 ) );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  StereoImuCalibration makeEquidistantStereo()
  {
    const Eigen::Matrix3d R_B_left =
        ( Eigen::AngleAxisd( 0.08, Eigen::Vector3d::UnitZ() ) *
          Eigen::AngleAxisd( -0.04, Eigen::Vector3d::UnitY() ) )
            .toRotationMatrix();
    const Eigen::Vector3d t_B_left( 0.13, -0.04, 0.07 );
    const Eigen::Matrix3d R_right_left =
        ( Eigen::AngleAxisd( 0.025, Eigen::Vector3d::UnitY() ) *
          Eigen::AngleAxisd( -0.012, Eigen::Vector3d::UnitZ() ) )
            .toRotationMatrix();
    const Eigen::Matrix3d R_B_right =
        R_B_left * R_right_left.transpose();
    const Eigen::Vector3d t_right_left( -0.11, 0.0, 0.0 );
    const Eigen::Vector3d t_B_right =
        t_B_left - R_B_right * t_right_left;

    auto calibration = StereoImuCalibration::create(
        makeEquidistantCamera( 512, 512 ), makeEquidistantCamera( 512, 512 ),
        makeImu(), makeTransform( R_B_left, t_B_left ),
        makeTransform( R_B_right, t_B_right ) );
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

  cv::Mat radtanCameraMatrix(
      const PinholeRadialTangentialParameters& model )
  {
    return ( cv::Mat_<double>( 3, 3 )
                 << model.fxPixels(),
             0.0, model.cxPixels(), 0.0,
             model.fyPixels(), model.cyPixels(), 0.0, 0.0, 1.0 );
  }

  cv::Mat radtanDistCoeffs(
      const PinholeRadialTangentialParameters& model )
  {
    return ( cv::Mat_<double>( 4, 1 ) << model.k1(), model.k2(), model.p1(),
             model.p2() );
  }

  std::pair<cv::Mat, cv::Mat> makeLeftRadtanMaps(
      const StereoImuCalibration& calibration )
  {
    const auto& left =
        std::get<PinholeRadialTangentialParameters>(
            calibration.leftCamera().modelParameters() );
    const auto& right =
        std::get<PinholeRadialTangentialParameters>(
            calibration.rightCamera().modelParameters() );
    const Eigen::Matrix3d R_B_left =
        calibration.T_B_left_camera().rotation();
    const Eigen::Matrix3d R_B_right =
        calibration.T_B_right_camera().rotation();
    const Eigen::Vector3d t_right_left =
        R_B_right.transpose() *
        ( calibration.T_B_left_camera().translation() -
          calibration.T_B_right_camera().translation() );
    const Eigen::Matrix3d R_right_left =
        R_B_right.transpose() * R_B_left;

    cv::Mat R( 3, 3, CV_64F );
    cv::Mat T( 3, 1, CV_64F );
    for ( int row = 0; row < 3; ++row )
    {
      T.at<double>( row, 0 ) = t_right_left( row );
      for ( int col = 0; col < 3; ++col )
      {
        R.at<double>( row, col ) = R_right_left( row, col );
      }
    }

    const cv::Size size(
        static_cast<int>( calibration.leftCamera().imageWidth() ),
        static_cast<int>( calibration.leftCamera().imageHeight() ) );
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    cv::stereoRectify(
        radtanCameraMatrix( left ), radtanDistCoeffs( left ),
        radtanCameraMatrix( right ), radtanDistCoeffs( right ), size, R, T, R1,
        R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0.0, size );

    cv::Mat map1;
    cv::Mat map2;
    cv::initUndistortRectifyMap( radtanCameraMatrix( left ),
                                 radtanDistCoeffs( left ), R1, P1, size,
                                 CV_16SC2, map1, map2 );
    return { map1, map2 };
  }

  std::vector<std::uint8_t> shiftHighByte( const cv::Mat& image )
  {
    std::vector<std::uint8_t> pixels( image.total() );
    for ( int row = 0; row < image.rows; ++row )
    {
      const auto* source = image.ptr<std::uint16_t>( row );
      for ( int col = 0; col < image.cols; ++col )
      {
        pixels[ static_cast<std::size_t>( row * image.cols + col ) ] =
            static_cast<std::uint8_t>( source[ col ] >> 8 );
      }
    }
    return pixels;
  }

  cv::Mat equidistantCameraMatrix(
      const PinholeEquidistantParameters& model )
  {
    return ( cv::Mat_<double>( 3, 3 )
                 << model.fxPixels(),
             0.0, model.cxPixels(), 0.0,
             model.fyPixels(), model.cyPixels(), 0.0, 0.0, 1.0 );
  }

  cv::Mat equidistantDistCoeffs(
      const PinholeEquidistantParameters& model )
  {
    return ( cv::Mat_<double>( 4, 1 ) << model.k1(), model.k2(), model.k3(),
             model.k4() );
  }

  void expectDetailContains( const std::string& detail,
                             std::string_view   expected )
  {
    EXPECT_NE( detail.find( expected ), std::string::npos ) << detail;
  }

  Image makeSubpixelMarker( int width, int height,
                            const Eigen::Vector2d& pixel )
  {
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>( width * height ), 0 );
    constexpr int    kRadius        = 5;
    constexpr double kSigma         = 1.5;
    constexpr double kTwoSigmaSqInv = 1.0 / ( 2.0 * kSigma * kSigma );
    const int        center_x       = static_cast<int>( std::floor( pixel.x() ) );
    const int        center_y       = static_cast<int>( std::floor( pixel.y() ) );
    for ( int y = center_y - kRadius; y <= center_y + kRadius; ++y )
    {
      for ( int x = center_x - kRadius; x <= center_x + kRadius; ++x )
      {
        if ( x >= 0 && x < width && y >= 0 && y < height )
        {
          const double dx = static_cast<double>( x ) - pixel.x();
          const double dy = static_cast<double>( y ) - pixel.y();
          const double value =
              255.0 * std::exp( -( dx * dx + dy * dy ) * kTwoSigmaSqInv );
          pixels[ static_cast<std::size_t>( y * width + x ) ] =
              static_cast<std::uint8_t>( std::lround( value ) );
        }
      }
    }
    return Image{ width, height, 1, std::move( pixels ) };
  }

  Eigen::Vector2d intensityCentroid( const Image& image )
  {
    const auto pixels = image.pixels<std::uint8_t>();
    EXPECT_TRUE( pixels );
    double mass  = 0.0;
    double x_sum = 0.0;
    double y_sum = 0.0;
    for ( int y = 0; y < image.height(); ++y )
    {
      for ( int x = 0; x < image.width(); ++x )
      {
        const double value = static_cast<double>(
            ( *pixels )[ static_cast<std::size_t>( y * image.width() + x ) ] );
        mass += value;
        x_sum += value * static_cast<double>( x );
        y_sum += value * static_cast<double>( y );
      }
    }
    EXPECT_GT( mass, 0.0 );
    return { x_sum / mass, y_sum / mass };
  }

  TEST( StereoRectifierTest, IdentityMappingForZeroDistortionPureTranslation )
  {
    constexpr double           kBaseline   = 0.11;
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

    const Image       left  = makeRampImage( 752, 480 );
    const Image       right = makeRampImage( 752, 480 );
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

  TEST( StereoRectifierTest, IdentityUint16ExtractsHighByte )
  {
    constexpr std::array<std::uint16_t, 7> kInput{ 0, 255, 256, 511,
                                                   512, 65280, 65535 };
    constexpr std::array<std::uint8_t, 7>  kExpected{ 0, 0, 1, 1, 2, 255,
                                                     255 };
    std::vector<std::uint16_t>             pixels( kInput.begin(), kInput.end() );
    const Image                            image{ 7, 1, 1, pixels };
    const StereoFrame                      raw{ phad::common::Timestamp{ 3 }, image, image };

    auto rectifier = StereoRectifier::create( makeSmallIdentityStereo() );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;
    auto out = rectifier.value().rectify( raw );
    ASSERT_TRUE( out ) << out.error().detail;
    EXPECT_EQ( out.value().left.pixelType(),
               phad::sensor::PixelType::kUint8 );
    EXPECT_EQ( out.value().right.pixelType(),
               phad::sensor::PixelType::kUint8 );
    const auto left_pixels = out.value().left.pixels<std::uint8_t>();
    ASSERT_TRUE( left_pixels );
    EXPECT_TRUE( std::equal( left_pixels->begin(), left_pixels->end(),
                             kExpected.begin(), kExpected.end() ) );
  }

  TEST( StereoRectifierTest, FractionalUint16RemapPrecedesHighByteExtraction )
  {
    constexpr int              kWidth  = 32;
    constexpr int              kHeight = 24;
    std::vector<std::uint16_t> pixels(
        static_cast<std::size_t>( kWidth * kHeight ) );
    for ( int y = 0; y < kHeight; ++y )
    {
      for ( int x = 0; x < kWidth; ++x )
      {
        pixels[ static_cast<std::size_t>( y * kWidth + x ) ] =
            static_cast<std::uint16_t>(
                ( x * 1531 + y * 811 + ( ( x + y ) % 3 ) * 251 ) & 0xFFFF );
      }
    }

    const StereoImuCalibration calibration = makeFractionalRadtanStereo();
    auto                       rectifier   = StereoRectifier::create( calibration );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;
    const Image image{ kWidth, kHeight, 1, pixels };
    auto        out = rectifier.value().rectify(
        StereoFrame{ phad::common::Timestamp{ 4 }, image, image } );
    ASSERT_TRUE( out ) << out.error().detail;

    const auto [ map1, map2 ] = makeLeftRadtanMaps( calibration );
    cv::Mat source16( kHeight, kWidth, CV_16UC1, pixels.data() );
    cv::Mat remapped16;
    cv::remap( source16, remapped16, map1, map2, cv::INTER_LINEAR,
               cv::BORDER_CONSTANT );
    const std::vector<std::uint8_t> expected = shiftHighByte( remapped16 );

    std::vector<std::uint8_t> shifted_source( pixels.size() );
    std::transform( pixels.begin(), pixels.end(), shifted_source.begin(),
                    []( std::uint16_t value ) {
                      return static_cast<std::uint8_t>( value >> 8 );
                    } );
    cv::Mat source8( kHeight, kWidth, CV_8UC1, shifted_source.data() );
    cv::Mat pre_shift_remapped;
    cv::remap( source8, pre_shift_remapped, map1, map2, cv::INTER_LINEAR,
               cv::BORDER_CONSTANT );
    const std::span<const std::uint8_t> pre_shift(
        pre_shift_remapped.ptr<std::uint8_t>(), pre_shift_remapped.total() );
    ASSERT_FALSE( std::equal( expected.begin(), expected.end(),
                              pre_shift.begin(), pre_shift.end() ) );

    const auto actual = out.value().left.pixels<std::uint8_t>();
    ASSERT_TRUE( actual );
    EXPECT_TRUE( std::equal( actual->begin(), actual->end(), expected.begin(),
                             expected.end() ) );
  }

  TEST( StereoRectifierTest, CreatesEquidistantCalibration )
  {
    auto rectifier = StereoRectifier::create( makeEquidistantStereo() );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;
  }

  TEST( StereoRectifierTest, RejectsMixedCameraModelsAtCreate )
  {
    auto calibration = StereoImuCalibration::create(
        makeRadtanCamera( 190.0, 191.0, 255.0, 256.0, 512, 512 ),
        makeEquidistantCamera( 512, 512 ), makeImu(), makeTranslation( 0.0 ),
        makeTranslation( 0.11 ) );
    ASSERT_TRUE( calibration );

    auto rectifier = StereoRectifier::create( calibration.value() );
    ASSERT_FALSE( rectifier );
    EXPECT_EQ( rectifier.error().code,
               CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( rectifier.error().detail, "create" );
    expectDetailContains( rectifier.error().detail, "expected" );
    expectDetailContains( rectifier.error().detail, "actual" );
    expectDetailContains( rectifier.error().detail, "radtan" );
    expectDetailContains( rectifier.error().detail, "equidistant" );
  }

  TEST( StereoRectifierTest, RejectsMismatchedCalibratedImageSizeAtCreate )
  {
    auto calibration = StereoImuCalibration::create(
        makeEquidistantCamera( 512, 512 ), makeEquidistantCamera( 640, 512 ),
        makeImu(), makeTranslation( 0.0 ), makeTranslation( 0.11 ) );
    ASSERT_TRUE( calibration );

    auto rectifier = StereoRectifier::create( calibration.value() );
    ASSERT_FALSE( rectifier );
    EXPECT_EQ( rectifier.error().code,
               CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( rectifier.error().detail, "create" );
    expectDetailContains( rectifier.error().detail, "size" );
    expectDetailContains( rectifier.error().detail, "expected" );
    expectDetailContains( rectifier.error().detail, "actual" );
    expectDetailContains( rectifier.error().detail, "512x512" );
    expectDetailContains( rectifier.error().detail, "640x512" );
  }

  TEST( StereoRectifierTest, RejectsMismatchedFrameSize )
  {
    auto rectifier = StereoRectifier::create( makeIdentityStereo() );
    ASSERT_TRUE( rectifier );

    const StereoFrame raw{ phad::common::Timestamp{ 2 },
                           makeRampImage( 32, 24 ), makeRampImage( 32, 24 ) };
    auto              out = rectifier.value().rectify( raw );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( out.error().detail, "rectify" );
    expectDetailContains( out.error().detail, "left" );
    expectDetailContains( out.error().detail, "expected" );
    expectDetailContains( out.error().detail, "actual" );
  }

  TEST( StereoRectifierTest, RejectsMultiChannelMetadata )
  {
    auto rectifier = StereoRectifier::create( makeIdentityStereo() );
    ASSERT_TRUE( rectifier );
    std::vector<std::uint8_t> pixels( 752U * 480U, 0 );
    const Image               left{ 752, 480, 3, pixels };
    const StereoFrame         raw{ phad::common::Timestamp{ 5 }, left,
                           makeRampImage( 752, 480 ) };

    auto out = rectifier.value().rectify( raw );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( out.error().detail, "rectify" );
    expectDetailContains( out.error().detail, "left" );
    expectDetailContains( out.error().detail, "channels" );
    expectDetailContains( out.error().detail, "expected" );
    expectDetailContains( out.error().detail, "actual" );
  }

  TEST( StereoRectifierTest, RejectsMixedStereoPixelTypes )
  {
    auto rectifier = StereoRectifier::create( makeIdentityStereo() );
    ASSERT_TRUE( rectifier );
    std::vector<std::uint16_t> right_pixels( 752U * 480U, 0 );
    const StereoFrame          raw{ phad::common::Timestamp{ 6 },
                           makeRampImage( 752, 480 ),
                           Image{ 752, 480, 1, std::move( right_pixels ) } };

    auto out = rectifier.value().rectify( raw );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( out.error().detail, "rectify" );
    expectDetailContains( out.error().detail, "right" );
    expectDetailContains( out.error().detail, "pixel type" );
    expectDetailContains( out.error().detail, "expected" );
    expectDetailContains( out.error().detail, "actual" );
  }

  TEST( StereoRectifierTest, RejectsTypedPixelCountMismatch )
  {
    auto rectifier = StereoRectifier::create( makeIdentityStereo() );
    ASSERT_TRUE( rectifier );
    std::vector<std::uint16_t> right_pixels( 752U * 480U - 1U, 0 );
    std::vector<std::uint16_t> left_pixels( 752U * 480U, 0 );
    const StereoFrame          raw{
        phad::common::Timestamp{ 7 },
        Image{ 752, 480, 1, std::move( left_pixels ) },
        Image{ 752, 480, 1, std::move( right_pixels ) } };

    auto out = rectifier.value().rectify( raw );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kOutsideModelDomain );
    expectDetailContains( out.error().detail, "rectify" );
    expectDetailContains( out.error().detail, "right" );
    expectDetailContains( out.error().detail, "pixel count" );
    expectDetailContains( out.error().detail, "expected" );
    expectDetailContains( out.error().detail, "actual" );
  }

  TEST( StereoRectifierTest, MapsRemapExceptionToNumericalFailure )
  {
    constexpr int kWidth      = 32767;
    auto          calibration = StereoImuCalibration::create(
        makeRadtanCamera( 100.0, 100.0, 16383.0, 0.0, kWidth, 1 ),
        makeRadtanCamera( 100.0, 100.0, 16383.0, 0.0, kWidth, 1 ), makeImu(),
        makeTranslation( 0.0 ), makeTranslation( 0.11 ) );
    ASSERT_TRUE( calibration );
    auto rectifier = StereoRectifier::create( calibration.value() );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;

    const Image image{ kWidth, 1, 1,
                       std::vector<std::uint8_t>(
                           static_cast<std::size_t>( kWidth ), 0 ) };
    auto        out = rectifier.value().rectify(
        StereoFrame{ phad::common::Timestamp{ 8 }, image, image } );
    ASSERT_FALSE( out );
    EXPECT_EQ( out.error().code, CameraModelErrorCode::kNumericalFailure );
    expectDetailContains( out.error().detail, "rectify" );
    expectDetailContains( out.error().detail, "remap" );
    expectDetailContains( out.error().detail, "left" );
    expectDetailContains( out.error().detail, "OpenCV" );
    expectDetailContains( out.error().detail, "cause" );
  }

  TEST( StereoRectifierTest, RectifiesEquidistantProjectedCorrespondences )
  {
    const StereoImuCalibration calibration = makeEquidistantStereo();
    auto                       rectifier   = StereoRectifier::create( calibration );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;

    const auto& output = rectifier.value().calibration();
    EXPECT_TRUE( std::isfinite( output.fxPixels() ) );
    EXPECT_TRUE( std::isfinite( output.fyPixels() ) );
    EXPECT_TRUE( std::isfinite( output.cxPixels() ) );
    EXPECT_TRUE( std::isfinite( output.cyPixels() ) );
    EXPECT_GT( output.fxPixels(), 0.0 );
    EXPECT_GT( output.fyPixels(), 0.0 );
    EXPECT_GT( output.baselineM(), 0.0 );
    EXPECT_EQ( output.imageWidth(), 512 );
    EXPECT_EQ( output.imageHeight(), 512 );

    const auto& left_parameters =
        std::get<PinholeEquidistantParameters>(
            calibration.leftCamera().modelParameters() );
    const auto& right_parameters =
        std::get<PinholeEquidistantParameters>(
            calibration.rightCamera().modelParameters() );
    auto left_model  = createCameraModel( left_parameters );
    auto right_model = createCameraModel( right_parameters );

    const Eigen::Matrix3d R_B_left =
        calibration.T_B_left_camera().rotation();
    const Eigen::Vector3d t_B_left =
        calibration.T_B_left_camera().translation();
    const Eigen::Matrix3d R_B_right =
        calibration.T_B_right_camera().rotation();
    const Eigen::Vector3d t_B_right =
        calibration.T_B_right_camera().translation();
    const Eigen::Matrix3d R_right_left =
        R_B_right.transpose() * R_B_left;
    const Eigen::Vector3d t_right_left =
        R_B_right.transpose() * ( t_B_left - t_B_right );

    cv::Mat R( 3, 3, CV_64F );
    cv::Mat T( 3, 1, CV_64F );
    for ( int row = 0; row < 3; ++row )
    {
      T.at<double>( row, 0 ) = t_right_left( row );
      for ( int col = 0; col < 3; ++col )
      {
        R.at<double>( row, col ) = R_right_left( row, col );
      }
    }
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    cv::fisheye::stereoRectify(
        equidistantCameraMatrix( left_parameters ),
        equidistantDistCoeffs( left_parameters ),
        equidistantCameraMatrix( right_parameters ),
        equidistantDistCoeffs( right_parameters ), cv::Size( 512, 512 ), R, T,
        R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, cv::Size( 512, 512 ),
        0.0, 1.0 );

    Eigen::Matrix3d R1_eigen;
    for ( int row = 0; row < 3; ++row )
    {
      for ( int col = 0; col < 3; ++col )
      {
        R1_eigen( row, col ) = R1.at<double>( row, col );
      }
    }
    EXPECT_FALSE( R_B_left.isApprox( Eigen::Matrix3d::Identity(), 1e-6 ) );
    EXPECT_TRUE( output.T_B_left_rectified().rotation().isApprox(
        R_B_left * R1_eigen.transpose(), 1e-9 ) );
    EXPECT_TRUE( output.T_B_left_rectified().translation().isApprox(
        t_B_left, 1e-12 ) );

    std::vector<Eigen::Vector3d> points_left;
    for ( int y_index = -2; y_index <= 2; ++y_index )
    {
      for ( int x_index = -2; x_index <= 2; ++x_index )
      {
        points_left.emplace_back( 0.38 * static_cast<double>( x_index ),
                                  0.28 * static_cast<double>( y_index ),
                                  4.5 + 0.3 * static_cast<double>(
                                                  x_index + y_index + 4 ) );
      }
    }
    ASSERT_GE( points_left.size(), 20U );

    double max_vertical_error_px = 0.0;
    double min_disparity_px      = std::numeric_limits<double>::infinity();
    for ( std::size_t index = 0; index < points_left.size(); ++index )
    {
      const Eigen::Vector3d point_right =
          R_right_left * points_left[ index ] + t_right_left;
      const auto left_pixel  = left_model->project( points_left[ index ] );
      const auto right_pixel = right_model->project( point_right );
      ASSERT_TRUE( left_pixel ) << index;
      ASSERT_TRUE( right_pixel ) << index;
      ASSERT_GT( left_pixel.value().x(), 2.0 );
      ASSERT_LT( left_pixel.value().x(), 509.0 );
      ASSERT_GT( left_pixel.value().y(), 2.0 );
      ASSERT_LT( left_pixel.value().y(), 509.0 );
      ASSERT_GT( right_pixel.value().x(), 2.0 );
      ASSERT_LT( right_pixel.value().x(), 509.0 );
      ASSERT_GT( right_pixel.value().y(), 2.0 );
      ASSERT_LT( right_pixel.value().y(), 509.0 );

      const StereoFrame raw{
          phad::common::Timestamp{ static_cast<std::int64_t>( 100 + index ) },
          makeSubpixelMarker( 512, 512, left_pixel.value() ),
          makeSubpixelMarker( 512, 512, right_pixel.value() ) };
      auto rectified = rectifier.value().rectify( raw );
      ASSERT_TRUE( rectified ) << index << ' ' << rectified.error().detail;
      const Eigen::Vector2d left_centroid =
          intensityCentroid( rectified.value().left );
      const Eigen::Vector2d right_centroid =
          intensityCentroid( rectified.value().right );
      const double vertical_error_px =
          std::abs( left_centroid.y() - right_centroid.y() );
      const double disparity_px = left_centroid.x() - right_centroid.x();
      max_vertical_error_px =
          std::max( max_vertical_error_px, vertical_error_px );
      min_disparity_px = std::min( min_disparity_px, disparity_px );
      EXPECT_LE( vertical_error_px, 1.0 )
          << index << ' ' << left_centroid.transpose() << " vs "
          << right_centroid.transpose();
      EXPECT_GT( disparity_px, 0.0 )
          << index << ' ' << left_centroid.transpose() << " vs "
          << right_centroid.transpose();
    }
    EXPECT_LE( max_vertical_error_px, 1.0 );
    EXPECT_GT( min_disparity_px, 0.0 );
    ::testing::Test::RecordProperty( "point_count", points_left.size() );
    ::testing::Test::RecordProperty( "max_vertical_error_px",
                                     max_vertical_error_px );
    ::testing::Test::RecordProperty( "min_disparity_px", min_disparity_px );
  }

}  // namespace

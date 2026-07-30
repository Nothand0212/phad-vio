#include "phad/sensor/stereo_imu_calibration.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>

namespace
{

  using phad::sensor::CalibrationErrorCode;
  using phad::sensor::CameraModelParameters;
  using phad::sensor::CameraParameters;
  using phad::sensor::ImuParameters;
  using phad::sensor::PinholeEquidistantParameters;
  using phad::sensor::PinholeRadialTangentialParameters;
  using phad::sensor::RigidTransform;
  using phad::sensor::StereoImuCalibration;

  template <typename T>
  concept HasMatrixAccessor = requires( const T& value ) {
    value.matrix();
  };

  static_assert( !std::is_default_constructible_v<RigidTransform> );
  static_assert( !std::is_aggregate_v<RigidTransform> );
  static_assert( !std::is_constructible_v<RigidTransform, Eigen::Matrix4d> );
  static_assert( !HasMatrixAccessor<RigidTransform> );
  static_assert(
      std::is_same_v<decltype( std::declval<const RigidTransform&>().rotation() ),
                     Eigen::Matrix3d> );
  static_assert(
      std::is_same_v<
          decltype( std::declval<const RigidTransform&>().translation() ),
          Eigen::Vector3d> );

  static_assert( !std::is_default_constructible_v<StereoImuCalibration> );
  static_assert( !std::is_aggregate_v<StereoImuCalibration> );
  static_assert(
      !std::is_constructible_v<
          StereoImuCalibration, CameraParameters, CameraParameters,
          ImuParameters, RigidTransform, RigidTransform> );

  Eigen::Matrix4d identityWithTranslation( double x, double y, double z )
  {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix( 0, 3 )         = x;
    matrix( 1, 3 )         = y;
    matrix( 2, 3 )         = z;
    return matrix;
  }

  CameraParameters radialCamera( int width = 752, int height = 480,
                                 double rate_hz = 20.0 )
  {
    auto model = PinholeRadialTangentialParameters::create(
        458.0, 457.0, 367.0, 248.0, -0.28, 0.07, 0.0002, -0.0001 );
    EXPECT_TRUE( model );
    auto camera = CameraParameters::create(
        CameraModelParameters{ std::move( model ).value() }, width, height,
        rate_hz );
    EXPECT_TRUE( camera );
    return std::move( camera ).value();
  }

  CameraParameters equidistantCamera( int width = 512, int height = 512,
                                      double rate_hz = 19.5 )
  {
    auto model = PinholeEquidistantParameters::create(
        190.0, 191.0, 255.0, 256.0, 0.01, -0.02, 0.003, -0.0004 );
    EXPECT_TRUE( model );
    auto camera = CameraParameters::create(
        CameraModelParameters{ std::move( model ).value() }, width, height,
        rate_hz );
    EXPECT_TRUE( camera );
    return std::move( camera ).value();
  }

  ImuParameters imuParameters()
  {
    auto imu =
        ImuParameters::create( 200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 );
    EXPECT_TRUE( imu );
    return std::move( imu ).value();
  }

  RigidTransform transform( double x, double y, double z )
  {
    auto result =
        RigidTransform::create( identityWithTranslation( x, y, z ) );
    EXPECT_TRUE( result );
    return std::move( result ).value();
  }

  TEST( RigidTransformTest, CreatesValidTransformWithoutExposingRawMatrix )
  {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>( 0, 0 ) =
        Eigen::AngleAxisd( 0.25, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    matrix.block<3, 1>( 0, 3 ) = Eigen::Vector3d( 1.0, -2.0, 3.0 );

    auto result = RigidTransform::create( matrix );

    ASSERT_TRUE( result );
    EXPECT_TRUE( result.value().rotation().isApprox(
        matrix.block<3, 3>( 0, 0 ), 0.0 ) );
    EXPECT_TRUE( result.value().translation().isApprox(
        Eigen::Vector3d( 1.0, -2.0, 3.0 ), 0.0 ) );
  }

  TEST( RigidTransformTest, RejectsEveryNonFiniteMatrixElement )
  {
    for ( Eigen::Index row = 0; row < 4; ++row )
    {
      for ( Eigen::Index column = 0; column < 4; ++column )
      {
        for ( const double invalid :
              { std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity() } )
        {
          Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
          matrix( row, column )  = invalid;

          auto result = RigidTransform::create( matrix );

          ASSERT_FALSE( result );
          EXPECT_EQ( result.error().code,
                     CalibrationErrorCode::kNonFiniteValue );
          EXPECT_EQ( result.error().field_path, "rigid_transform.matrix" );
        }
      }
    }
  }

  TEST( RigidTransformTest, EnforcesRotationAndDeterminantWithoutRepair )
  {
    Eigen::Matrix4d shear = Eigen::Matrix4d::Identity();
    shear( 0, 1 )         = 1e-3;
    auto shear_result     = RigidTransform::create( shear );
    ASSERT_FALSE( shear_result );
    EXPECT_EQ( shear_result.error().code,
               CalibrationErrorCode::kInvalidRotation );
    EXPECT_EQ( shear_result.error().field_path, "rigid_transform.rotation" );

    Eigen::Matrix4d reflection = Eigen::Matrix4d::Identity();
    reflection( 0, 0 )         = -1.0;
    auto reflection_result     = RigidTransform::create( reflection );
    ASSERT_FALSE( reflection_result );
    EXPECT_EQ( reflection_result.error().code,
               CalibrationErrorCode::kInvalidRotation );

    Eigen::Matrix4d within_tolerance = Eigen::Matrix4d::Identity();
    within_tolerance( 0, 0 ) += 0.49e-6;
    auto accepted = RigidTransform::create( within_tolerance );
    ASSERT_TRUE( accepted );
    EXPECT_DOUBLE_EQ( accepted.value().rotation()( 0, 0 ),
                      within_tolerance( 0, 0 ) );

    Eigen::Matrix4d outside_tolerance = Eigen::Matrix4d::Identity();
    outside_tolerance( 0, 0 ) += 0.51e-6;
    auto rejected = RigidTransform::create( outside_tolerance );
    ASSERT_FALSE( rejected );
    EXPECT_EQ( rejected.error().code,
               CalibrationErrorCode::kInvalidRotation );
  }

  TEST( RigidTransformTest, EnforcesHomogeneousBottomRowTolerance )
  {
    Eigen::Matrix4d within_tolerance = Eigen::Matrix4d::Identity();
    within_tolerance( 3, 0 )         = 1e-6;
    EXPECT_TRUE( RigidTransform::create( within_tolerance ) );

    Eigen::Matrix4d outside_tolerance = Eigen::Matrix4d::Identity();
    outside_tolerance( 3, 0 ) =
        std::nextafter( 1e-6, std::numeric_limits<double>::infinity() );
    auto result = RigidTransform::create( outside_tolerance );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               CalibrationErrorCode::kInvalidHomogeneousRow );
    EXPECT_EQ( result.error().field_path,
               "rigid_transform.bottom_row" );
  }

  TEST( RigidTransformTest, AccessorsReturnIndependentValues )
  {
    const RigidTransform value       = transform( 1.0, 2.0, 3.0 );
    Eigen::Matrix3d      rotation    = value.rotation();
    Eigen::Vector3d      translation = value.translation();
    rotation.setZero();
    translation.setZero();

    EXPECT_TRUE( value.rotation().isApprox( Eigen::Matrix3d::Identity() ) );
    EXPECT_TRUE(
        value.translation().isApprox( Eigen::Vector3d( 1.0, 2.0, 3.0 ) ) );
  }

  TEST( StereoImuCalibrationTest, AllowsMixedModelsSizesAndDeclaredRates )
  {
    auto result = StereoImuCalibration::create(
        radialCamera(), equidistantCamera(), imuParameters(),
        transform( 0.0, 0.0, 0.0 ), transform( 0.11, 0.0, 0.0 ) );

    ASSERT_TRUE( result );
    EXPECT_EQ( result.value().leftCamera().imageWidth(), 752U );
    EXPECT_EQ( result.value().rightCamera().imageWidth(), 512U );
    EXPECT_DOUBLE_EQ( result.value().leftCamera().rateHz(), 20.0 );
    EXPECT_DOUBLE_EQ( result.value().rightCamera().rateHz(), 19.5 );
    EXPECT_TRUE( std::holds_alternative<PinholeRadialTangentialParameters>(
        result.value().leftCamera().modelParameters() ) );
    EXPECT_TRUE( std::holds_alternative<PinholeEquidistantParameters>(
        result.value().rightCamera().modelParameters() ) );
  }

  TEST( StereoImuCalibrationTest, RejectsCoincidentCameraCenters )
  {
    auto result = StereoImuCalibration::create(
        radialCamera(), radialCamera(), imuParameters(),
        transform( 0.2, -0.1, 0.3 ), transform( 0.2, -0.1, 0.3 ) );

    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               CalibrationErrorCode::kZeroStereoBaseline );
    EXPECT_EQ( result.error().field_path,
               "stereo_imu_calibration.camera_centers" );
  }

}  // namespace

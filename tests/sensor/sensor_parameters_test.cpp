#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>

#include "phad/sensor/camera_parameters.hpp"
#include "phad/sensor/imu_parameters.hpp"

namespace
{

  using phad::sensor::CalibrationError;
  using phad::sensor::CalibrationErrorCode;
  using phad::sensor::CameraModelParameters;
  using phad::sensor::CameraParameters;
  using phad::sensor::ImuParameters;
  using phad::sensor::PinholeEquidistantParameters;
  using phad::sensor::PinholeRadialTangentialParameters;

  static_assert(
      !std::is_default_constructible_v<
          PinholeRadialTangentialParameters> );
  static_assert(
      !std::is_default_constructible_v<PinholeEquidistantParameters> );
  static_assert( !std::is_default_constructible_v<CameraModelParameters> );
  static_assert( !std::is_default_constructible_v<CameraParameters> );
  static_assert( !std::is_default_constructible_v<ImuParameters> );

  static_assert( !std::is_aggregate_v<PinholeRadialTangentialParameters> );
  static_assert( !std::is_aggregate_v<PinholeEquidistantParameters> );
  static_assert( !std::is_aggregate_v<CameraParameters> );
  static_assert( !std::is_aggregate_v<ImuParameters> );

  static_assert(
      !std::is_constructible_v<PinholeRadialTangentialParameters,
                               double, double, double, double, double, double,
                               double, double> );
  static_assert(
      !std::is_constructible_v<PinholeEquidistantParameters, double, double,
                               double, double, double, double, double,
                               double> );
  static_assert(
      !std::is_constructible_v<CameraParameters, CameraModelParameters, int,
                               int, double> );
  static_assert(
      !std::is_constructible_v<ImuParameters, double, double, double, double,
                               double> );

  template <typename T>
  void requireError(
      const phad::sensor::CalibrationResult<T>& result,
      CalibrationErrorCode expected_code, const char* expected_field_path )
  {
    EXPECT_FALSE( result.hasValue() );
    EXPECT_FALSE( static_cast<bool>( result ) );
    const CalibrationError& error = result.error();
    EXPECT_EQ( error.code, expected_code );
    EXPECT_EQ( error.field_path, expected_field_path );
    EXPECT_FALSE( error.detail.empty() );
  }

  auto makeRadial()
  {
    return PinholeRadialTangentialParameters::create(
        458.0, 457.0, 367.0, 248.0, -0.28, 0.07, 0.0002, -0.0001 );
  }

  TEST( CameraParametersTest, CreatesNamedRadialTangentialValues )
  {
    auto result = makeRadial();

    ASSERT_TRUE( result.hasValue() );
    EXPECT_TRUE( static_cast<bool>( result ) );
    const auto& value = result.value();
    EXPECT_DOUBLE_EQ( value.fxPixels(), 458.0 );
    EXPECT_DOUBLE_EQ( value.fyPixels(), 457.0 );
    EXPECT_DOUBLE_EQ( value.cxPixels(), 367.0 );
    EXPECT_DOUBLE_EQ( value.cyPixels(), 248.0 );
    EXPECT_DOUBLE_EQ( value.k1(), -0.28 );
    EXPECT_DOUBLE_EQ( value.k2(), 0.07 );
    EXPECT_DOUBLE_EQ( value.p1(), 0.0002 );
    EXPECT_DOUBLE_EQ( value.p2(), -0.0001 );
  }

  TEST( CameraParametersTest, CreatesNamedEquidistantValues )
  {
    auto result = PinholeEquidistantParameters::create(
        190.0, 191.0, 255.0, 256.0, 0.01, -0.02, 0.003, -0.0004 );

    ASSERT_TRUE( result );
    const auto& value = result.value();
    EXPECT_DOUBLE_EQ( value.fxPixels(), 190.0 );
    EXPECT_DOUBLE_EQ( value.fyPixels(), 191.0 );
    EXPECT_DOUBLE_EQ( value.cxPixels(), 255.0 );
    EXPECT_DOUBLE_EQ( value.cyPixels(), 256.0 );
    EXPECT_DOUBLE_EQ( value.k1(), 0.01 );
    EXPECT_DOUBLE_EQ( value.k2(), -0.02 );
    EXPECT_DOUBLE_EQ( value.k3(), 0.003 );
    EXPECT_DOUBLE_EQ( value.k4(), -0.0004 );
  }

  TEST( CameraParametersTest, RejectsEveryNonFiniteRadialTangentialField )
  {
    constexpr std::array<double, 8> kValid{
        458.0, 457.0, 367.0, 248.0, -0.28, 0.07, 0.0002, -0.0001 };
    constexpr std::array<const char*, 8> kFieldPaths{
        "camera.model_parameters.radial_tangential.fx_pixels",
        "camera.model_parameters.radial_tangential.fy_pixels",
        "camera.model_parameters.radial_tangential.cx_pixels",
        "camera.model_parameters.radial_tangential.cy_pixels",
        "camera.model_parameters.radial_tangential.k1",
        "camera.model_parameters.radial_tangential.k2",
        "camera.model_parameters.radial_tangential.p1",
        "camera.model_parameters.radial_tangential.p2" };
    for ( std::size_t index = 0; index < kValid.size(); ++index )
    {
      for ( const double invalid :
            { std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity() } )
      {
        auto values     = kValid;
        values[ index ] = invalid;
        requireError(
            PinholeRadialTangentialParameters::create(
                values[ 0 ], values[ 1 ], values[ 2 ], values[ 3 ],
                values[ 4 ], values[ 5 ], values[ 6 ], values[ 7 ] ),
            CalibrationErrorCode::kNonFiniteValue, kFieldPaths[ index ] );
      }
    }
  }

  TEST( CameraParametersTest, RejectsEveryNonFiniteEquidistantField )
  {
    constexpr std::array<double, 8> kValid{
        190.0, 191.0, 255.0, 256.0, 0.01, -0.02, 0.003, -0.0004 };
    constexpr std::array<const char*, 8> kFieldPaths{
        "camera.model_parameters.equidistant.fx_pixels",
        "camera.model_parameters.equidistant.fy_pixels",
        "camera.model_parameters.equidistant.cx_pixels",
        "camera.model_parameters.equidistant.cy_pixels",
        "camera.model_parameters.equidistant.k1",
        "camera.model_parameters.equidistant.k2",
        "camera.model_parameters.equidistant.k3",
        "camera.model_parameters.equidistant.k4" };
    for ( std::size_t index = 0; index < kValid.size(); ++index )
    {
      for ( const double invalid :
            { std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity() } )
      {
        auto values     = kValid;
        values[ index ] = invalid;
        requireError(
            PinholeEquidistantParameters::create(
                values[ 0 ], values[ 1 ], values[ 2 ], values[ 3 ],
                values[ 4 ], values[ 5 ], values[ 6 ], values[ 7 ] ),
            CalibrationErrorCode::kNonFiniteValue, kFieldPaths[ index ] );
      }
    }
  }

  TEST( CameraParametersTest, RejectsNonPositiveFocalLengths )
  {
    for ( const double invalid : { 0.0, -1.0 } )
    {
      requireError(
          PinholeRadialTangentialParameters::create(
              invalid, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0 ),
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fx_pixels" );
      requireError(
          PinholeRadialTangentialParameters::create(
              458.0, invalid, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0 ),
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fy_pixels" );
      requireError(
          PinholeEquidistantParameters::create(
              invalid, 191.0, 255.0, 256.0, 0.0, 0.0, 0.0, 0.0 ),
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fx_pixels" );
      requireError(
          PinholeEquidistantParameters::create(
              190.0, invalid, 255.0, 256.0, 0.0, 0.0, 0.0, 0.0 ),
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fy_pixels" );
    }
  }

  TEST( CameraParametersTest, AllowsPrincipalPointOutsideImageBounds )
  {
    auto model = PinholeRadialTangentialParameters::create(
        458.0, 457.0, -10.0, 900.0, 0.0, 0.0, 0.0, 0.0 );
    ASSERT_TRUE( model );

    auto camera = CameraParameters::create(
        CameraModelParameters{ std::move( model ).value() }, 752, 480, 20.0 );

    ASSERT_TRUE( camera );
    EXPECT_EQ( camera.value().imageWidth(), 752U );
    EXPECT_EQ( camera.value().imageHeight(), 480U );
    EXPECT_DOUBLE_EQ( camera.value().rateHz(), 20.0 );
    const auto& stored_model =
        std::get<PinholeRadialTangentialParameters>(
            camera.value().modelParameters() );
    EXPECT_DOUBLE_EQ( stored_model.cxPixels(), -10.0 );
    EXPECT_DOUBLE_EQ( stored_model.cyPixels(), 900.0 );
  }

  TEST( CameraParametersTest, RejectsNonPositiveImageSize )
  {
    for ( const int invalid : { 0, -1 } )
    {
      auto model = makeRadial();
      ASSERT_TRUE( model );
      requireError(
          CameraParameters::create( std::move( model ).value(), invalid, 480,
                                    20.0 ),
          CalibrationErrorCode::kNonPositiveValue, "camera.image_width" );

      model = makeRadial();
      ASSERT_TRUE( model );
      requireError(
          CameraParameters::create( std::move( model ).value(), 752, invalid,
                                    20.0 ),
          CalibrationErrorCode::kNonPositiveValue, "camera.image_height" );
    }
  }

  TEST( CameraParametersTest, RejectsNonFiniteAndNonPositiveRate )
  {
    for ( const double invalid :
          { std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity() } )
    {
      auto model = makeRadial();
      ASSERT_TRUE( model );
      requireError(
          CameraParameters::create( std::move( model ).value(), 752, 480,
                                    invalid ),
          CalibrationErrorCode::kNonFiniteValue, "camera.rate_hz" );
    }

    for ( const double invalid : { 0.0, -1.0 } )
    {
      auto model = makeRadial();
      ASSERT_TRUE( model );
      requireError(
          CameraParameters::create( std::move( model ).value(), 752, 480,
                                    invalid ),
          CalibrationErrorCode::kNonPositiveValue, "camera.rate_hz" );
    }
  }

  TEST( ImuParametersTest, CreatesNamedSamplingAndNoiseValues )
  {
    auto result =
        ImuParameters::create( 200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 );

    ASSERT_TRUE( result );
    const auto& value = result.value();
    EXPECT_DOUBLE_EQ( value.rateHz(), 200.0 );
    EXPECT_DOUBLE_EQ(
        value.accelerometerNoiseDensityMps2PerSqrtHz(), 0.002 );
    EXPECT_DOUBLE_EQ(
        value.gyroscopeNoiseDensityRadpsPerSqrtHz(), 0.00016968 );
    EXPECT_DOUBLE_EQ(
        value.accelerometerBiasRandomWalkMps3PerSqrtHz(), 0.003 );
    EXPECT_DOUBLE_EQ(
        value.gyroscopeBiasRandomWalkRadps2PerSqrtHz(), 1.9393e-05 );
  }

  TEST( ImuParametersTest, RejectsEveryNonFiniteValue )
  {
    constexpr std::array<double, 5> kValid{
        200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 };
    constexpr std::array<const char*, 5> kFieldPaths{
        "imu.rate_hz",
        "imu.accelerometer_noise_density_mps2_per_sqrt_hz",
        "imu.gyroscope_noise_density_radps_per_sqrt_hz",
        "imu.accelerometer_bias_random_walk_mps3_per_sqrt_hz",
        "imu.gyroscope_bias_random_walk_radps2_per_sqrt_hz" };
    for ( std::size_t index = 0; index < kValid.size(); ++index )
    {
      for ( const double invalid :
            { std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity() } )
      {
        auto values     = kValid;
        values[ index ] = invalid;
        requireError(
            ImuParameters::create( values[ 0 ], values[ 1 ], values[ 2 ],
                                   values[ 3 ], values[ 4 ] ),
            CalibrationErrorCode::kNonFiniteValue, kFieldPaths[ index ] );
      }
    }
  }

  TEST( ImuParametersTest, RejectsEveryNonPositiveValue )
  {
    constexpr std::array<double, 5> kValid{
        200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 };
    constexpr std::array<const char*, 5> kFieldPaths{
        "imu.rate_hz",
        "imu.accelerometer_noise_density_mps2_per_sqrt_hz",
        "imu.gyroscope_noise_density_radps_per_sqrt_hz",
        "imu.accelerometer_bias_random_walk_mps3_per_sqrt_hz",
        "imu.gyroscope_bias_random_walk_radps2_per_sqrt_hz" };
    for ( std::size_t index = 0; index < kValid.size(); ++index )
    {
      for ( const double invalid : { 0.0, -1.0 } )
      {
        auto values     = kValid;
        values[ index ] = invalid;
        requireError(
            ImuParameters::create( values[ 0 ], values[ 1 ], values[ 2 ],
                                   values[ 3 ], values[ 4 ] ),
            CalibrationErrorCode::kNonPositiveValue, kFieldPaths[ index ] );
      }
    }
  }

}  // namespace

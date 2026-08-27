#include "phad/estimator/imu_rotation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace
{

  using phad::common::Timestamp;
  using phad::estimator::ImuRotationError;
  using phad::estimator::integrateImuRotation;
  using phad::sensor::ImuMeasurement;

  ImuMeasurement sample( const std::int64_t     timestamp_ns,
                         const Eigen::Vector3d& gyro )
  {
    ImuMeasurement measurement;
    measurement.timestamp  = Timestamp{ timestamp_ns };
    measurement.gyro_radps = { gyro.x(), gyro.y(), gyro.z() };
    return measurement;
  }

  TEST( ImuRotationTest, StationarySegmentIsIdentity )
  {
    const std::vector<ImuMeasurement> samples{
        sample( 0, Eigen::Vector3d::Zero() ),
        sample( 500'000'000, Eigen::Vector3d::Zero() ),
        sample( 1'000'000'000, Eigen::Vector3d::Zero() ) };

    const auto result =
        integrateImuRotation( samples, Eigen::Vector3d::Zero() );
    ASSERT_TRUE( result.valid );
    EXPECT_EQ( result.error, ImuRotationError::kNone );
    EXPECT_DOUBLE_EQ( result.dt_s, 1.0 );
    EXPECT_DOUBLE_EQ( result.angle_rad, 0.0 );
    EXPECT_TRUE( result.R_Bi_Bj.isIdentity( 1e-12 ) );
  }

  TEST( ImuRotationTest, ConstantPositiveZRotationHasExpectedDirection )
  {
    const Eigen::Vector3d             gyro{ 0.0, 0.0, 1.0 };
    const std::vector<ImuMeasurement> samples{
        sample( 0, gyro ), sample( 400'000'000, gyro ),
        sample( 1'000'000'000, gyro ) };

    const auto result =
        integrateImuRotation( samples, Eigen::Vector3d::Zero() );
    ASSERT_TRUE( result.valid );
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd( 1.0, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    EXPECT_TRUE( result.R_Bi_Bj.isApprox( expected, 1e-12 ) );
    EXPECT_NEAR( result.angle_rad, 1.0, 1e-12 );
    const Eigen::Vector3d rotated =
        result.R_Bi_Bj * Eigen::Vector3d::UnitX();
    EXPECT_GT( rotated.y(), 0.0 );
  }

  TEST( ImuRotationTest, AbsoluteGyroBiasCancelsMeasurement )
  {
    const Eigen::Vector3d             bias{ 0.01, -0.02, 0.25 };
    const std::vector<ImuMeasurement> samples{
        sample( 0, bias ), sample( 500'000'000, bias ),
        sample( 1'000'000'000, bias ) };

    const auto result = integrateImuRotation( samples, bias );
    ASSERT_TRUE( result.valid );
    EXPECT_TRUE( result.R_Bi_Bj.isIdentity( 1e-12 ) );
    EXPECT_NEAR( result.angle_rad, 0.0, 1e-12 );
  }

  TEST( ImuRotationTest, RejectsInsufficientAndNonMonotonicSamples )
  {
    const std::vector<ImuMeasurement> one{
        sample( 0, Eigen::Vector3d::Zero() ) };
    EXPECT_EQ( integrateImuRotation( one, Eigen::Vector3d::Zero() ).error,
               ImuRotationError::kInsufficientSamples );

    const std::vector<ImuMeasurement> duplicate{
        sample( 1, Eigen::Vector3d::Zero() ),
        sample( 1, Eigen::Vector3d::Zero() ) };
    EXPECT_EQ(
        integrateImuRotation( duplicate, Eigen::Vector3d::Zero() ).error,
        ImuRotationError::kNonMonotonicTimestamp );

    const std::vector<ImuMeasurement> reverse{
        sample( 2, Eigen::Vector3d::Zero() ),
        sample( 1, Eigen::Vector3d::Zero() ) };
    EXPECT_EQ( integrateImuRotation( reverse, Eigen::Vector3d::Zero() ).error,
               ImuRotationError::kNonMonotonicTimestamp );
  }

  TEST( ImuRotationTest, RejectsNonFiniteGyroAndBias )
  {
    const double                      nan = std::numeric_limits<double>::quiet_NaN();
    const std::vector<ImuMeasurement> bad_gyro{
        sample( 0, { nan, 0.0, 0.0 } ),
        sample( 1, Eigen::Vector3d::Zero() ) };
    EXPECT_EQ(
        integrateImuRotation( bad_gyro, Eigen::Vector3d::Zero() ).error,
        ImuRotationError::kNonFiniteGyro );

    const std::vector<ImuMeasurement> valid{
        sample( 0, Eigen::Vector3d::Zero() ),
        sample( 1, Eigen::Vector3d::Zero() ) };
    EXPECT_EQ( integrateImuRotation( valid, { 0.0, nan, 0.0 } ).error,
               ImuRotationError::kNonFiniteBias );
  }

}  // namespace

#include "phad/estimator/imu_rotation.hpp"

#include <gtsam/navigation/AHRSFactor.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace phad::estimator
{

  ImuRotationResult integrateImuRotation(
      const std::span<const sensor::ImuMeasurement> samples,
      const Eigen::Vector3d&                        bias_gyr )
  {
    ImuRotationResult result;
    if ( samples.size() < 2U )
    {
      result.error = ImuRotationError::kInsufficientSamples;
      return result;
    }

    result.t_i = samples.front().timestamp;
    result.t_j = samples.back().timestamp;
    if ( !bias_gyr.allFinite() )
    {
      result.error = ImuRotationError::kNonFiniteBias;
      return result;
    }

    for ( const sensor::ImuMeasurement& sample : samples )
    {
      for ( const double value : sample.gyro_radps )
      {
        if ( !std::isfinite( value ) )
        {
          result.error = ImuRotationError::kNonFiniteGyro;
          return result;
        }
      }
    }

    const auto params =
        std::make_shared<gtsam::PreintegratedRotationParams>();
    gtsam::PreintegratedAhrsMeasurements preintegration( params, bias_gyr );
    for ( std::size_t i = 1; i < samples.size(); ++i )
    {
      const std::int64_t previous_ns =
          samples[ i - 1U ].timestamp.nanoseconds();
      const std::int64_t current_ns = samples[ i ].timestamp.nanoseconds();
      if ( current_ns <= previous_ns )
      {
        result.error = ImuRotationError::kNonMonotonicTimestamp;
        return result;
      }
      const std::uint64_t dt_ns =
          static_cast<std::uint64_t>( current_ns ) -
          static_cast<std::uint64_t>( previous_ns );
      const double dt_s = static_cast<double>( dt_ns ) * 1e-9;
      const auto&  gyro = samples[ i - 1U ].gyro_radps;
      preintegration.integrateMeasurement(
          Eigen::Vector3d( gyro[ 0 ], gyro[ 1 ], gyro[ 2 ] ), dt_s );
    }

    result.dt_s    = preintegration.deltaTij();
    result.R_Bi_Bj = preintegration.deltaRij().matrix();
    result.angle_rad =
        gtsam::Rot3::Logmap( preintegration.deltaRij() ).norm();
    if ( !std::isfinite( result.dt_s ) ||
         !std::isfinite( result.angle_rad ) ||
         !result.R_Bi_Bj.allFinite() )
    {
      result.dt_s      = 0.0;
      result.angle_rad = 0.0;
      result.R_Bi_Bj   = Eigen::Matrix3d::Identity();
      result.error     = ImuRotationError::kNonFiniteResult;
      return result;
    }

    result.valid = true;
    result.error = ImuRotationError::kNone;
    return result;
  }

  std::string_view imuRotationErrorName(
      const ImuRotationError error ) noexcept
  {
    switch ( error )
    {
      case ImuRotationError::kNone:
        return "none";
      case ImuRotationError::kInsufficientSamples:
        return "insufficient_samples";
      case ImuRotationError::kNonMonotonicTimestamp:
        return "non_monotonic_timestamp";
      case ImuRotationError::kNonFiniteGyro:
        return "non_finite_gyro";
      case ImuRotationError::kNonFiniteBias:
        return "non_finite_bias";
      case ImuRotationError::kNonFiniteResult:
        return "non_finite_result";
    }
    return "unknown";
  }

}  // namespace phad::estimator

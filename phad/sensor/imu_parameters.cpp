#include "phad/sensor/imu_parameters.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace phad::sensor
{

  CalibrationResult<ImuParameters> ImuParameters::create(
      double rate_hz,
      double accelerometer_noise_density_mps2_per_sqrt_hz,
      double gyroscope_noise_density_radps_per_sqrt_hz,
      double accelerometer_bias_random_walk_mps3_per_sqrt_hz,
      double gyroscope_bias_random_walk_radps2_per_sqrt_hz )
  {
    const std::array<std::pair<double, const char*>, 5> parameters{
        std::pair{ rate_hz, "imu.rate_hz" },
        std::pair{ accelerometer_noise_density_mps2_per_sqrt_hz,
                   "imu.accelerometer_noise_density_mps2_per_sqrt_hz" },
        std::pair{ gyroscope_noise_density_radps_per_sqrt_hz,
                   "imu.gyroscope_noise_density_radps_per_sqrt_hz" },
        std::pair{ accelerometer_bias_random_walk_mps3_per_sqrt_hz,
                   "imu.accelerometer_bias_random_walk_mps3_per_sqrt_hz" },
        std::pair{ gyroscope_bias_random_walk_radps2_per_sqrt_hz,
                   "imu.gyroscope_bias_random_walk_radps2_per_sqrt_hz" } };
    for ( const auto& [ value, field_path ] : parameters )
    {
      if ( !std::isfinite( value ) )
      {
        return CalibrationError{ CalibrationErrorCode::kNonFiniteValue,
                                 field_path,
                                 "IMU parameter must be finite" };
      }
      if ( value <= 0.0 )
      {
        return CalibrationError{ CalibrationErrorCode::kNonPositiveValue,
                                 field_path,
                                 "IMU parameter must be strictly positive" };
      }
    }
    return ImuParameters(
        rate_hz, accelerometer_noise_density_mps2_per_sqrt_hz,
        gyroscope_noise_density_radps_per_sqrt_hz,
        accelerometer_bias_random_walk_mps3_per_sqrt_hz,
        gyroscope_bias_random_walk_radps2_per_sqrt_hz );
  }

  ImuParameters::ImuParameters(
      double rate_hz,
      double accelerometer_noise_density_mps2_per_sqrt_hz,
      double gyroscope_noise_density_radps_per_sqrt_hz,
      double accelerometer_bias_random_walk_mps3_per_sqrt_hz,
      double gyroscope_bias_random_walk_radps2_per_sqrt_hz )
      : m_rate_hz( rate_hz ),
        m_accelerometer_noise_density_mps2_per_sqrt_hz(
            accelerometer_noise_density_mps2_per_sqrt_hz ),
        m_gyroscope_noise_density_radps_per_sqrt_hz(
            gyroscope_noise_density_radps_per_sqrt_hz ),
        m_accelerometer_bias_random_walk_mps3_per_sqrt_hz(
            accelerometer_bias_random_walk_mps3_per_sqrt_hz ),
        m_gyroscope_bias_random_walk_radps2_per_sqrt_hz(
            gyroscope_bias_random_walk_radps2_per_sqrt_hz )
  {
  }

  double ImuParameters::rateHz() const noexcept { return m_rate_hz; }

  double ImuParameters::accelerometerNoiseDensityMps2PerSqrtHz()
      const noexcept
  {
    return m_accelerometer_noise_density_mps2_per_sqrt_hz;
  }

  double ImuParameters::gyroscopeNoiseDensityRadpsPerSqrtHz() const noexcept
  {
    return m_gyroscope_noise_density_radps_per_sqrt_hz;
  }

  double ImuParameters::accelerometerBiasRandomWalkMps3PerSqrtHz()
      const noexcept
  {
    return m_accelerometer_bias_random_walk_mps3_per_sqrt_hz;
  }

  double ImuParameters::gyroscopeBiasRandomWalkRadps2PerSqrtHz()
      const noexcept
  {
    return m_gyroscope_bias_random_walk_radps2_per_sqrt_hz;
  }

}  // namespace phad::sensor

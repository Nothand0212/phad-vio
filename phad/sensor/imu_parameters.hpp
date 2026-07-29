#pragma once

#include "phad/sensor/calibration_error.hpp"

namespace phad::sensor
{

  class ImuParameters
  {
  public:
    [[nodiscard]] static CalibrationResult<ImuParameters> create(
        double rate_hz,
        double accelerometer_noise_density_mps2_per_sqrt_hz,
        double gyroscope_noise_density_radps_per_sqrt_hz,
        double accelerometer_bias_random_walk_mps3_per_sqrt_hz,
        double gyroscope_bias_random_walk_radps2_per_sqrt_hz );

    [[nodiscard]] double rateHz() const noexcept;
    [[nodiscard]] double accelerometerNoiseDensityMps2PerSqrtHz()
        const noexcept;
    [[nodiscard]] double gyroscopeNoiseDensityRadpsPerSqrtHz() const noexcept;
    [[nodiscard]] double accelerometerBiasRandomWalkMps3PerSqrtHz()
        const noexcept;
    [[nodiscard]] double gyroscopeBiasRandomWalkRadps2PerSqrtHz()
        const noexcept;

  private:
    ImuParameters(
        double rate_hz,
        double accelerometer_noise_density_mps2_per_sqrt_hz,
        double gyroscope_noise_density_radps_per_sqrt_hz,
        double accelerometer_bias_random_walk_mps3_per_sqrt_hz,
        double gyroscope_bias_random_walk_radps2_per_sqrt_hz );

    double m_rate_hz;
    double m_accelerometer_noise_density_mps2_per_sqrt_hz;
    double m_gyroscope_noise_density_radps_per_sqrt_hz;
    double m_accelerometer_bias_random_walk_mps3_per_sqrt_hz;
    double m_gyroscope_bias_random_walk_radps2_per_sqrt_hz;
  };

}  // namespace phad::sensor

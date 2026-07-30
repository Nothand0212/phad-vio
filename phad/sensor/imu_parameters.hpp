#pragma once

#include "phad/sensor/calibration_error.hpp"

namespace phad::sensor
{

  // IMU noise params: acc_nd/gyr_nd in m/s²/√Hz and rad/s/√Hz;
  // acc_rw/gyr_rw in m/s³/√Hz and rad/s²/√Hz.
  class ImuParameters
  {
  public:
    [[nodiscard]] static CalibrationResult<ImuParameters> create(
        double rate_hz, double acc_nd, double gyr_nd, double acc_rw,
        double gyr_rw );

    [[nodiscard]] double rateHz() const noexcept;
    [[nodiscard]] double accNd() const noexcept;
    [[nodiscard]] double gyrNd() const noexcept;
    [[nodiscard]] double accRw() const noexcept;
    [[nodiscard]] double gyrRw() const noexcept;

  private:
    ImuParameters( double rate_hz, double acc_nd, double gyr_nd,
                   double acc_rw, double gyr_rw );

    double m_rate_hz;
    double m_acc_nd;
    double m_gyr_nd;
    double m_acc_rw;
    double m_gyr_rw;
  };

}  // namespace phad::sensor

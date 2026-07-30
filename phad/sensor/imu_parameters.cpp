#include "phad/sensor/imu_parameters.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace phad::sensor
{

  CalibrationResult<ImuParameters> ImuParameters::create(
      double rate_hz, double acc_nd, double gyr_nd, double acc_rw,
      double gyr_rw )
  {
    const std::array<std::pair<double, const char*>, 5> parameters{
        std::pair{ rate_hz, "imu.rate_hz" },
        std::pair{ acc_nd, "imu.acc_nd" },
        std::pair{ gyr_nd, "imu.gyr_nd" },
        std::pair{ acc_rw, "imu.acc_rw" },
        std::pair{ gyr_rw, "imu.gyr_rw" } };
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
    return ImuParameters( rate_hz, acc_nd, gyr_nd, acc_rw, gyr_rw );
  }

  ImuParameters::ImuParameters( double rate_hz, double acc_nd, double gyr_nd,
                                double acc_rw, double gyr_rw )
      : m_rate_hz( rate_hz ),
        m_acc_nd( acc_nd ),
        m_gyr_nd( gyr_nd ),
        m_acc_rw( acc_rw ),
        m_gyr_rw( gyr_rw )
  {
  }

  double ImuParameters::rateHz() const noexcept { return m_rate_hz; }

  double ImuParameters::accNd() const noexcept { return m_acc_nd; }

  double ImuParameters::gyrNd() const noexcept { return m_gyr_nd; }

  double ImuParameters::accRw() const noexcept { return m_acc_rw; }

  double ImuParameters::gyrRw() const noexcept { return m_gyr_rw; }

}  // namespace phad::sensor

#include <type_traits>

#include "phad/sensor/stereo_imu_calibration.hpp"

namespace
{
  static_assert(
      !std::is_default_constructible_v<phad::sensor::StereoImuCalibration> );
}

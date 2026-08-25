#pragma once

#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::sensor
{

  struct RawImuInterval
  {
    common::Timestamp           m_t_begin;
    common::Timestamp           m_t_end;
    std::vector<ImuMeasurement> m_samples;
  };

  struct MeasurementDiscontinuity
  {
    common::Timestamp m_t_begin;
    common::Timestamp m_t_end;
  };

  using ImuPayload =
      std::variant<RawImuInterval, MeasurementDiscontinuity>;

  /** 已配对双目与 sync 分类后的 raw/discontinuity IMU payload。 */
  struct StereoImuPacket
  {
    StereoFrame m_frame;
    ImuPayload  m_imu;
  };

}  // namespace phad::sensor

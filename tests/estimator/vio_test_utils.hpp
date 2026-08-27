#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

namespace phad::test_support
{

  inline constexpr std::int64_t kFramePeriodNs = 50'000'000;

  [[nodiscard]] inline sensor::ImuParameters testImuParameters()
  {
    return sensor::ImuParameters::create( 200.0, 1'000'000.0, 1'000'000.0,
                                          1'000'000.0, 1'000'000.0 )
        .value();
  }

  [[nodiscard]] inline sensor::ImuParameters constrainingImuParameters()
  {
    return sensor::ImuParameters::create( 200.0, 0.1, 0.01, 0.001,
                                          0.0001 )
        .value();
  }

  [[nodiscard]] inline sensor::ImuMeasurement stationaryImu(
      std::int64_t timestamp_ns )
  {
    return sensor::ImuMeasurement{
        .timestamp  = common::Timestamp{ timestamp_ns },
        .accel_mps2 = std::array<double, 3>{ 0.0, 0.0, 9.81 },
        .gyro_radps = std::array<double, 3>{ 0.0, 0.0, 0.0 } };
  }

  [[nodiscard]] inline sensor::ImuPayload stationaryImuPayload(
      common::Timestamp t_end,
      std::int64_t      interval_ns = kFramePeriodNs )
  {
    const std::int64_t end_ns    = t_end.nanoseconds();
    const std::int64_t begin_ns  = end_ns - interval_ns;
    const std::int64_t middle_ns = begin_ns + interval_ns / 2;
    return sensor::RawImuInterval{
        .m_t_begin = common::Timestamp{ begin_ns },
        .m_t_end   = t_end,
        .m_samples = { stationaryImu( begin_ns ), stationaryImu( middle_ns ),
                       stationaryImu( end_ns ) } };
  }

  [[nodiscard]] inline sensor::ImuPayload stationaryImuPayload(
      common::Timestamp t_begin, common::Timestamp t_end )
  {
    const std::int64_t begin_ns  = t_begin.nanoseconds();
    const std::int64_t end_ns    = t_end.nanoseconds();
    const std::int64_t middle_ns = begin_ns + ( end_ns - begin_ns ) / 2;
    return sensor::RawImuInterval{
        .m_t_begin = t_begin,
        .m_t_end   = t_end,
        .m_samples = { stationaryImu( begin_ns ), stationaryImu( middle_ns ),
                       stationaryImu( end_ns ) } };
  }

}  // namespace phad::test_support

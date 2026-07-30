#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/sensor/stereo_imu_calibration.hpp"

namespace phad::io
{

  using SensorEvent =
      std::variant<sensor::ImuMeasurement, sensor::StereoFrame>;

  struct EndOfStream
  {
  };

  enum class SensorSourceErrorCode : std::uint8_t
  {
    kReadFailed = 0
  };

  struct SensorSourceError
  {
    SensorSourceErrorCode            code = SensorSourceErrorCode::kReadFailed;
    std::string                      source_id;
    std::optional<common::Timestamp> timestamp;
    std::string                      cause;
  };

  using SensorReadResult =
      std::variant<SensorEvent, EndOfStream, SensorSourceError>;

  class SensorSource
  {
  public:
    virtual ~SensorSource() = default;

    [[nodiscard]] virtual const sensor::StereoImuCalibration& calibration()
        const noexcept = 0;

    [[nodiscard]] virtual SensorReadResult next() = 0;
  };

}  // namespace phad::io

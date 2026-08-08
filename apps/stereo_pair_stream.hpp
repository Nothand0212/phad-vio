#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "phad/io/sensor_source.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"
#include "phad/sync/stereo_pair_synchronizer.hpp"

/**
 * @file stereo_pair_stream.hpp
 * @brief SensorSource + StereoPairSynchronizer 薄组合，产出下一对 StereoFrame
 *        或完整 StereoImuPacket（M4.1：IMU 进 sync 并切段）。
 */

namespace phad::apps
{

  struct StreamError
  {
    std::string detail;
  };

  using StereoPairReadResult =
      std::variant<sensor::StereoFrame, io::EndOfStream, StreamError>;
  // M4.1: 与 StereoPairReadResult 同源,取完整 packet(含帧间 IMU 段)。
  using StereoImuPacketReadResult =
      std::variant<sensor::StereoImuPacket, io::EndOfStream, StreamError>;

  class StereoPairStream
  {
  public:
    explicit StereoPairStream(
        io::SensorSource&                   source,
        sync::StereoPairSynchronizerOptions options = {} );

    [[nodiscard]] StereoPairReadResult next();
    /// M4.1: 与 next() 同源,IMU 进 sync(不再丢弃);session 按需消费。
    [[nodiscard]] StereoImuPacketReadResult nextPacket();

    [[nodiscard]] const sync::StereoPairDiagnostics& diagnostics()
        const noexcept;
    [[nodiscard]] const std::vector<std::string>& warnings() const noexcept;

  private:
    void noteDiagnosticsWarnings();

    io::SensorSource&            m_source;
    sync::StereoPairSynchronizer m_sync;
    std::vector<std::string>     m_warnings;
    bool                         m_warned_drop     = false;
    bool                         m_warned_overflow = false;
    bool                         m_flushed         = false;
    std::optional<StreamError>   m_terminal_error;
  };

}  // namespace phad::apps

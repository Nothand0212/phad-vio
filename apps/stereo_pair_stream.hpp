#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "phad/io/sensor_source.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/sync/stereo_pair_synchronizer.hpp"

/**
 * @file stereo_pair_stream.hpp
 * @brief SensorSource + StereoPairSynchronizer 薄组合，产出下一对 StereoFrame。
 */

namespace phad::apps
{

  struct StreamError
  {
    std::string detail;
  };

  using StereoPairReadResult =
      std::variant<sensor::StereoFrame, io::EndOfStream, StreamError>;

  class StereoPairStream
  {
  public:
    explicit StereoPairStream(
        io::SensorSource&                   source,
        sync::StereoPairSynchronizerOptions options = {} );

    [[nodiscard]] StereoPairReadResult next();

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

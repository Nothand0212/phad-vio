#include "apps/stereo_pair_stream.hpp"

#include <utility>
#include <variant>

#include "phad/sensor/camera_id.hpp"

/**
 * @file stereo_pair_stream.cpp
 * @brief StereoPairStream：拉 SensorSource 事件，push 进 sync，tryPop 配对帧。
 */

namespace phad::apps
{

  StereoPairStream::StereoPairStream(
      io::SensorSource&                   source,
      sync::StereoPairSynchronizerOptions options )
      : m_source( source ),
        m_sync( std::move( options ) )
  {
  }

  StereoImuPacketReadResult StereoPairStream::nextPacket()
  {
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }

    while ( true )
    {
      if ( auto packet = m_sync.tryPopPacket() )
      {
        return std::move( *packet );
      }

      if ( m_flushed )
      {
        return io::EndOfStream{};
      }

      io::SensorReadResult read = m_source.next();
      if ( std::holds_alternative<io::EndOfStream>( read ) )
      {
        m_sync.flush();
        m_flushed = true;
        noteDiagnosticsWarnings();
        if ( auto packet = m_sync.tryPopPacket() )
        {
          return std::move( *packet );
        }
        return io::EndOfStream{};
      }

      if ( const auto* error = std::get_if<io::SensorSourceError>( &read ) )
      {
        m_terminal_error = StreamError{ error->cause };
        return *m_terminal_error;
      }

      auto& event = std::get<io::SensorEvent>( read );
      // M4.1: IMU 进 sync(单调性校验 + 队列切段),不再丢弃。
      const sync::PushStatus status =
          std::holds_alternative<sensor::ImuMeasurement>( event )
              ? m_sync.pushImu( std::get<sensor::ImuMeasurement>( event ) )
              : m_sync.pushImage(
                    std::move( std::get<sensor::ImageFrameEvent>( event ) ) );
      noteDiagnosticsWarnings();
      if ( status != sync::PushStatus::kOk )
      {
        m_terminal_error = StreamError{ "stereo synchronizer sticky error" };
        return *m_terminal_error;
      }
    }
  }

  StereoPairReadResult StereoPairStream::next()
  {
    StereoImuPacketReadResult result = nextPacket();
    if ( const auto* packet =
             std::get_if<sensor::StereoImuPacket>( &result ) )
    {
      return std::move( packet->frame );
    }
    if ( std::holds_alternative<io::EndOfStream>( result ) )
    {
      return io::EndOfStream{};
    }
    return std::get<StreamError>( result );
  }

  const sync::StereoPairDiagnostics& StereoPairStream::diagnostics()
      const noexcept
  {
    return m_sync.diagnostics();
  }

  const std::vector<std::string>& StereoPairStream::warnings() const noexcept
  {
    return m_warnings;
  }

  void StereoPairStream::noteDiagnosticsWarnings()
  {
    const auto& d = m_sync.diagnostics();
    if ( !m_warned_drop && ( d.dropped_left > 0U || d.dropped_right > 0U ) )
    {
      m_warnings.emplace_back(
          "stereo sync dropped unpaired frames (see summary.sync)" );
      m_warned_drop = true;
    }
    if ( !m_warned_overflow &&
         ( d.dropped_left_overflow > 0U || d.dropped_right_overflow > 0U ) )
    {
      m_warnings.emplace_back(
          "stereo sync queue overflow dropped oldest frames" );
      m_warned_overflow = true;
    }
  }

}  // namespace phad::apps

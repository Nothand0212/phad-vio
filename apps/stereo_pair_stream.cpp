#include "apps/stereo_pair_stream.hpp"

#include <utility>
#include <variant>

#include "phad/sensor/camera_id.hpp"
#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "apps/internal/stereo_pair_stream_test_access.hpp"
#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/io/dataset/internal/dataset_replay_source_test_access.hpp"
#include "phad/sync/internal/stereo_pair_synchronizer_test_access.hpp"
#endif

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
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }

    while ( true )
    {
      if ( auto frame = m_sync.tryPop() )
      {
        return std::move( *frame );
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
        if ( auto frame = m_sync.tryPop() )
        {
          return std::move( *frame );
        }
        return io::EndOfStream{};
      }
      if ( const auto* error = std::get_if<io::SensorSourceError>( &read ) )
      {
        m_terminal_error = StreamError{ error->cause };
        return *m_terminal_error;
      }

      auto& event = std::get<io::SensorEvent>( read );
      if ( std::holds_alternative<sensor::ImuMeasurement>( event ) )
      {
        continue;
      }
      const sync::PushStatus status = m_sync.pushImage(
          std::move( std::get<sensor::ImageFrameEvent>( event ) ) );
      noteDiagnosticsWarnings();
      if ( status != sync::PushStatus::kOk )
      {
        m_terminal_error = StreamError{ "stereo synchronizer sticky error" };
        return *m_terminal_error;
      }
    }
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

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
  std::string internal::StereoPairStreamTestAccess::canonicalStateJson(
      const StereoPairStream& stream )
  {
    const auto* source = dynamic_cast<const io::dataset::DatasetReplaySource*>(
        &stream.m_source );
    if ( source == nullptr )
    {
      throw std::logic_error(
          "replay state requires DatasetReplaySource" );
    }

    nlohmann::ordered_json root;
    root[ "schema" ] = "phad.replay.stereo_pair_stream_state.v1";

    nlohmann::ordered_json state;
    state[ "source" ] = nlohmann::ordered_json::parse(
        io::dataset::internal::canonicalStateJson( *source ) );
    state[ "synchronizer" ] = nlohmann::ordered_json::parse(
        sync::internal::canonicalStateJson( stream.m_sync ) );
    state[ "warnings" ]        = stream.m_warnings;
    state[ "warned_drop" ]     = stream.m_warned_drop;
    state[ "warned_overflow" ] = stream.m_warned_overflow;
    state[ "flushed" ]         = stream.m_flushed;
    if ( stream.m_terminal_error.has_value() )
    {
      nlohmann::ordered_json terminal;
      terminal[ "detail" ]      = stream.m_terminal_error->detail;
      state[ "terminal_error" ] = std::move( terminal );
    }
    else
    {
      state[ "terminal_error" ] = nullptr;
    }
    root[ "state" ] = std::move( state );
    return root.dump();
  }
#endif

}  // namespace phad::apps

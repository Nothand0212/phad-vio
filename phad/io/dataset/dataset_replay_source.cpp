#include "phad/io/dataset/dataset_replay_source.hpp"

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "phad/sensor/camera_id.hpp"
#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
#include <array>
#include <bit>
#include <nlohmann/json.hpp>

#include "phad/io/dataset/internal/dataset_replay_source_test_access.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_reader_test_access.hpp"
#endif

namespace phad::io::dataset
{

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
  namespace
  {

    [[nodiscard]] std::string doubleBitsHex( double value )
    {
      constexpr std::array<char, 16> kHexDigits{
          '0', '1', '2', '3', '4', '5', '6', '7',
          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
      std::uint64_t bits = std::bit_cast<std::uint64_t>( value );
      std::string   result( 16U, '0' );
      for ( std::size_t i = result.size(); i > 0U; --i )
      {
        result[ i - 1U ] = kHexDigits[ bits & 0xFU ];
        bits >>= 4U;
      }
      return result;
    }

    [[nodiscard]] nlohmann::ordered_json imuMeasurementJson(
        const sensor::ImuMeasurement& measurement )
    {
      nlohmann::ordered_json value;
      value[ "timestamp_ns" ] = measurement.timestamp.nanoseconds();
      auto accel              = nlohmann::ordered_json::array();
      auto gyro               = nlohmann::ordered_json::array();
      for ( std::size_t axis = 0; axis < 3U; ++axis )
      {
        accel.push_back( doubleBitsHex( measurement.accel_mps2[ axis ] ) );
        gyro.push_back( doubleBitsHex( measurement.gyro_radps[ axis ] ) );
      }
      value[ "accel_mps2_f64_bits_hex" ] = std::move( accel );
      value[ "gyro_radps_f64_bits_hex" ] = std::move( gyro );
      return value;
    }

    [[nodiscard]] nlohmann::ordered_json optionalImuMeasurementJson(
        const std::optional<sensor::ImuMeasurement>& measurement )
    {
      if ( !measurement.has_value() )
      {
        return nullptr;
      }
      return imuMeasurementJson( *measurement );
    }

    [[nodiscard]] nlohmann::ordered_json sourceErrorJson(
        const io::SensorSourceError& error )
    {
      nlohmann::ordered_json value;
      value[ "kind" ]      = "sensor_source_error";
      value[ "code" ]      = "read_failed";
      value[ "source_id" ] = error.source_id;
      if ( error.timestamp.has_value() )
      {
        value[ "timestamp_ns" ] = error.timestamp->nanoseconds();
      }
      else
      {
        value[ "timestamp_ns" ] = nullptr;
      }
      value[ "cause" ] = error.cause;
      return value;
    }

  }  // namespace
#endif

  DatasetReplaySource::DatasetReplaySource( const StereoImuDataset& dataset )
      : m_calibration( dataset.calibration() ),
        m_reader( dataset.reader() )
  {
  }

  const sensor::StereoImuCalibration& DatasetReplaySource::calibration()
      const noexcept
  {
    return m_calibration;
  }

  io::SensorReadResult DatasetReplaySource::next()
  {
    if ( m_terminal_state.has_value() )
    {
      return terminalResult();
    }

    if ( !m_imu_lookahead.has_value() )
    {
      auto imu_result = m_reader.takeImu();
      if ( auto* error = std::get_if<DatasetReaderError>( &imu_result ) )
      {
        return fail( *error );
      }
      if ( auto* measurement =
               std::get_if<sensor::ImuMeasurement>( &imu_result ) )
      {
        m_imu_lookahead = std::move( *measurement );
      }
    }

    auto left_timestamp_result =
        m_reader.peekImageTimestamp( sensor::CameraId::kLeft );
    if ( auto* error =
             std::get_if<DatasetReaderError>( &left_timestamp_result ) )
    {
      return fail( *error );
    }
    auto right_timestamp_result =
        m_reader.peekImageTimestamp( sensor::CameraId::kRight );
    if ( auto* error =
             std::get_if<DatasetReaderError>( &right_timestamp_result ) )
    {
      return fail( *error );
    }

    const auto* left_timestamp =
        std::get_if<common::Timestamp>( &left_timestamp_result );
    const auto* right_timestamp =
        std::get_if<common::Timestamp>( &right_timestamp_result );

    enum class Stream : std::uint8_t
    {
      kImu = 0,
      kLeft,
      kRight
    };

    std::optional<Stream>            chosen;
    std::optional<common::Timestamp> chosen_ts;

    const auto consider = [ & ]( Stream stream, common::Timestamp ts ) {
      const auto rank = static_cast<std::uint8_t>( stream );
      if ( !chosen.has_value() || ts < *chosen_ts ||
           ( ts == *chosen_ts &&
             rank < static_cast<std::uint8_t>( *chosen ) ) )
      {
        chosen    = stream;
        chosen_ts = ts;
      }
    };

    if ( m_imu_lookahead.has_value() )
    {
      consider( Stream::kImu, m_imu_lookahead->timestamp );
    }
    if ( left_timestamp != nullptr )
    {
      consider( Stream::kLeft, *left_timestamp );
    }
    if ( right_timestamp != nullptr )
    {
      consider( Stream::kRight, *right_timestamp );
    }

    if ( !chosen.has_value() )
    {
      return finish();
    }

    switch ( *chosen )
    {
      case Stream::kImu:
      {
        sensor::ImuMeasurement measurement = std::move( *m_imu_lookahead );
        m_imu_lookahead.reset();
        return io::SensorEvent{ std::move( measurement ) };
      }
      case Stream::kLeft:
      case Stream::kRight:
      {
        const sensor::CameraId camera       = *chosen == Stream::kLeft
                                                  ? sensor::CameraId::kLeft
                                                  : sensor::CameraId::kRight;
        auto                   image_result = m_reader.takeImage( camera );
        if ( auto* error = std::get_if<DatasetReaderError>( &image_result ) )
        {
          return fail( *error );
        }
        if ( auto* image =
                 std::get_if<sensor::ImageFrameEvent>( &image_result ) )
        {
          return io::SensorEvent{ std::move( *image ) };
        }
        return finish();
      }
    }

    return finish();
  }

  io::SensorReadResult DatasetReplaySource::fail(
      const DatasetReaderError& error )
  {
    std::ostringstream cause;
    cause << "dataset reader failure, record=" << error.record_number << ": "
          << error.cause;
    m_terminal_state = io::SensorSourceError{
        io::SensorSourceErrorCode::kReadFailed,
        error.sensor_id,
        error.timestamp,
        std::move( cause ).str() };
    return terminalResult();
  }

  io::SensorReadResult DatasetReplaySource::finish()
  {
    m_terminal_state = io::EndOfStream{};
    return terminalResult();
  }

  io::SensorReadResult DatasetReplaySource::terminalResult() const
  {
    return std::visit(
        []( const auto& terminal_state ) -> io::SensorReadResult {
          return terminal_state;
        },
        *m_terminal_state );
  }

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
  std::string internal::DatasetReplaySourceTestAccess::canonicalStateJson(
      const DatasetReplaySource& source )
  {
    nlohmann::ordered_json root;
    root[ "schema" ]            = "phad.replay.dataset_replay_source_state.v1";
    root[ "immutable_dataset" ] = "content_addressed_input_manifest";

    nlohmann::ordered_json state;
    state[ "reader" ] = nlohmann::ordered_json::parse(
        internal::canonicalStateJson( source.m_reader ) );
    state[ "imu_lookahead" ] =
        optionalImuMeasurementJson( source.m_imu_lookahead );
    if ( !source.m_terminal_state.has_value() )
    {
      state[ "terminal" ] = nullptr;
    }
    else if ( std::holds_alternative<io::EndOfStream>(
                  *source.m_terminal_state ) )
    {
      state[ "terminal" ] = nlohmann::ordered_json{ { "kind",
                                                      "end_of_stream" } };
    }
    else
    {
      state[ "terminal" ] = sourceErrorJson(
          std::get<io::SensorSourceError>( *source.m_terminal_state ) );
    }
    root[ "state" ] = std::move( state );
    return root.dump();
  }
#endif

}  // namespace phad::io::dataset

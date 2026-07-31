#include "phad/io/dataset/dataset_replay_source.hpp"

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

#include "phad/sensor/camera_id.hpp"

namespace phad::io::dataset
{

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

}  // namespace phad::io::dataset

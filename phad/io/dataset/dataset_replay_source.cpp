#include "phad/io/dataset/dataset_replay_source.hpp"

#include <sstream>
#include <string>
#include <utility>
#include <variant>

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

    auto stereo_timestamp_result = m_reader.peekStereoTimestamp();
    if ( auto* error =
             std::get_if<DatasetReaderError>( &stereo_timestamp_result ) )
    {
      return fail( *error );
    }

    const auto* stereo_timestamp =
        std::get_if<common::Timestamp>( &stereo_timestamp_result );
    if ( m_imu_lookahead.has_value() &&
         ( stereo_timestamp == nullptr ||
           m_imu_lookahead->timestamp <= *stereo_timestamp ) )
    {
      sensor::ImuMeasurement measurement = std::move( *m_imu_lookahead );
      m_imu_lookahead.reset();
      return io::SensorEvent{ std::move( measurement ) };
    }

    if ( stereo_timestamp != nullptr )
    {
      auto stereo_result = m_reader.takeStereo();
      if ( auto* error = std::get_if<DatasetReaderError>( &stereo_result ) )
      {
        return fail( *error );
      }
      if ( auto* stereo = std::get_if<sensor::StereoFrame>( &stereo_result ) )
      {
        return io::SensorEvent{ std::move( *stereo ) };
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

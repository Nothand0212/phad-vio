#include "phad/io/dataset/dataset_replay_source.hpp"

#include <string>
#include <utility>

namespace phad::io::dataset
{

  DatasetReplaySource::DatasetReplaySource( StereoImuDataset dataset )
      : m_dataset( std::move( dataset ) )
  {
  }

  const sensor::StereoImuCalibration& DatasetReplaySource::calibration()
      const noexcept
  {
    return m_dataset.calibration();
  }

  io::SensorReadResult DatasetReplaySource::next()
  {
    if ( m_terminal_error.has_value() )
    {
      return *m_terminal_error;
    }

    const auto imu_measurements = m_dataset.imuMeasurements();
    const auto stereo_index     = m_dataset.stereoIndex();
    const bool has_imu          = m_next_imu_index < imu_measurements.size();
    const bool has_stereo       = m_next_stereo_index < stereo_index.size();

    if ( has_imu &&
         ( !has_stereo ||
           imu_measurements[ m_next_imu_index ].timestamp <=
               stereo_index[ m_next_stereo_index ].timestamp ) )
    {
      const sensor::ImuMeasurement measurement =
          imu_measurements[ m_next_imu_index ];
      ++m_next_imu_index;
      return io::SensorEvent{ measurement };
    }

    if ( has_stereo )
    {
      auto loaded = m_dataset.loadStereo( m_next_stereo_index );
      if ( !loaded )
      {
        return fail( loaded.error() );
      }
      ++m_next_stereo_index;
      return io::SensorEvent{ std::move( loaded ).value() };
    }

    return io::EndOfStream{};
  }

  io::SensorReadResult DatasetReplaySource::fail( const DatasetError& error )
  {
    m_terminal_error = io::SensorSourceError{
        io::SensorSourceErrorCode::kReadFailed,
        error.sensor_id.empty() ? std::string{ "dataset" } : error.sensor_id,
        error.timestamp,
        error.describe() };
    return *m_terminal_error;
  }

}  // namespace phad::io::dataset

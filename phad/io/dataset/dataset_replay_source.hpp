#pragma once

#include <cstddef>
#include <optional>

#include "phad/io/dataset/stereo_imu_dataset.hpp"
#include "phad/io/sensor_source.hpp"

namespace phad::io::dataset
{

  class DatasetReplaySource final : public io::SensorSource
  {
  public:
    explicit DatasetReplaySource( StereoImuDataset dataset );

    DatasetReplaySource( const DatasetReplaySource& )                = delete;
    DatasetReplaySource& operator=( const DatasetReplaySource& )     = delete;
    DatasetReplaySource( DatasetReplaySource&& ) noexcept            = default;
    DatasetReplaySource& operator=( DatasetReplaySource&& ) noexcept = default;

    [[nodiscard]] const sensor::StereoImuCalibration& calibration()
        const noexcept override;

    [[nodiscard]] io::SensorReadResult next() override;

  private:
    [[nodiscard]] io::SensorReadResult fail( const DatasetError& error );

    StereoImuDataset                     m_dataset;
    sensor::StereoImuCalibration         m_calibration;
    std::size_t                          m_next_imu_index    = 0;
    std::size_t                          m_next_stereo_index = 0;
    std::optional<io::SensorSourceError> m_terminal_error;
  };

}  // namespace phad::io::dataset

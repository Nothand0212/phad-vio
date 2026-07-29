#pragma once

#include <optional>
#include <variant>

#include "phad/io/dataset/stereo_imu_dataset.hpp"
#include "phad/io/sensor_source.hpp"

namespace phad::io::dataset
{

  class DatasetReplaySource final : public io::SensorSource
  {
  public:
    explicit DatasetReplaySource( const StereoImuDataset& dataset );

    DatasetReplaySource( const DatasetReplaySource& )                = delete;
    DatasetReplaySource& operator=( const DatasetReplaySource& )     = delete;
    DatasetReplaySource( DatasetReplaySource&& ) noexcept            = default;
    DatasetReplaySource& operator=( DatasetReplaySource&& ) noexcept = default;

    [[nodiscard]] const sensor::StereoImuCalibration& calibration()
        const noexcept override;

    [[nodiscard]] io::SensorReadResult next() override;

  private:
    using TerminalState = std::variant<io::EndOfStream, io::SensorSourceError>;

    [[nodiscard]] io::SensorReadResult fail(
        const DatasetReaderError& error );
    [[nodiscard]] io::SensorReadResult finish();
    [[nodiscard]] io::SensorReadResult terminalResult() const;

    sensor::StereoImuCalibration          m_calibration;
    StereoImuDatasetReader                m_reader;
    std::optional<sensor::ImuMeasurement> m_imu_lookahead;
    std::optional<TerminalState>          m_terminal_state;
  };

}  // namespace phad::io::dataset

#pragma once

#include <filesystem>
#include <utility>

#include "phad/dataset/stereo_imu_dataset.hpp"

namespace phad::dataset
{

  using EurocCalibration = StereoImuCalibration;

  namespace euroc
  {
    [[nodiscard]] DatasetResult<StereoImuDataset> open(
        const std::filesystem::path& sequence_root );
  }

  class EurocDataset
  {
  public:
    [[nodiscard]] static DatasetResult<EurocDataset> open(
        const std::filesystem::path& sequence_root );

    [[nodiscard]] const EurocCalibration& calibration() const noexcept
    {
      return m_dataset.calibration();
    }

    [[nodiscard]] std::span<const sensor::ImuMeasurement> imuMeasurements()
        const noexcept
    {
      return m_dataset.imuMeasurements();
    }

    [[nodiscard]] std::span<const StereoFrameRef> stereoIndex() const noexcept
    {
      return m_dataset.stereoIndex();
    }

    [[nodiscard]] DatasetResult<sensor::StereoFrame> loadStereo(
        std::size_t index ) const
    {
      return m_dataset.loadStereo( index );
    }

  private:
    explicit EurocDataset( StereoImuDataset dataset )
        : m_dataset( std::move( dataset ) ) {}

    StereoImuDataset m_dataset;
  };

}  // namespace phad::dataset

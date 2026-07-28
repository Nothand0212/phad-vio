#pragma once

#include <filesystem>
#include <utility>

#include "phad/io/dataset/stereo_imu_dataset.hpp"

/**
 * @file euroc_dataset.hpp
 * @brief EurocDataset 类用于表示 Euroc 数据集。
 *
 * 该文件定义了 phad::io::dataset::EurocDataset 类，
 * 封装了 Euroc 数据集的相机和 IMU 外参、IMU 测量和立体帧索引，
 * 提供对 Euroc 数据集的快速访问和引用。
 */

namespace phad::io::dataset
{

  using EurocCalibration = StereoImuCalibration;

  namespace euroc
  {
    [[nodiscard]] DatasetResult<StereoImuDataset> open(
        const std::filesystem::path& sequence_root );
  }


  /**
   * @brief EurocDataset 类用于表示 Euroc 数据集。
   *
   * 该类封装了 Euroc 数据集的相机和 IMU 外参、IMU 测量和立体帧索引，
   * 提供对 Euroc 数据集的快速访问和引用。
   */
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

}  // namespace phad::io::dataset

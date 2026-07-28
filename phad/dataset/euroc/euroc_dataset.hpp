#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <vector>

#include "phad/dataset/dataset_error.hpp"
#include "phad/sensor/calibration.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::dataset
{

  /**
   * @brief StereoFrameRef 结构体用于表示立体帧的引用。
   *
   * 该结构体封装了时间戳、左右图像路径等信息，提供对立体帧的快速访问和引用。
   */
  struct StereoFrameRef
  {
    common::Timestamp     timestamp;
    std::filesystem::path left_path;
    std::filesystem::path right_path;
  };

  /**
   * @brief EurocCalibration 结构体用于表示 Euroc 数据集的相机和 IMU 外参。
   *
   * 该结构体封装了左右相机和 IMU 的外参，提供对相机和 IMU 外参的快速访问和引用。
   */
  struct EurocCalibration
  {
    sensor::CameraCalibration left;
    sensor::CameraCalibration right;
    sensor::ImuCalibration    imu;
  };

  /**
   * @brief EurocDataset 类用于表示 Euroc 数据集。
   *
   * 该类封装了 Euroc 数据集的相机和 IMU 外参、IMU 测量和立体帧索引，
   * 提供对 Euroc 数据集的快速访问和引用。
   */
  class EurocDataset
  {
  public:
    static DatasetResult<EurocDataset> open(
        const std::filesystem::path& sequence_root );

    [[nodiscard]] const EurocCalibration& calibration() const noexcept
    {
      return m_calibration;
    }
    [[nodiscard]] std::span<const sensor::ImuMeasurement> imuMeasurements()
        const noexcept
    {
      return m_imu_measurements;
    }
    [[nodiscard]] std::span<const StereoFrameRef> stereoIndex() const noexcept
    {
      return m_stereo_index;
    }

    [[nodiscard]] DatasetResult<sensor::StereoFrame> loadStereo(
        std::size_t index ) const;

  private:
    EurocDataset( EurocCalibration                    calibration,
                  std::vector<sensor::ImuMeasurement> imu_measurements,
                  std::vector<StereoFrameRef>         stereo_index )
        : m_calibration( std::move( calibration ) ),
          m_imu_measurements( std::move( imu_measurements ) ),
          m_stereo_index( std::move( stereo_index ) ) {}

    EurocCalibration                    m_calibration;
    std::vector<sensor::ImuMeasurement> m_imu_measurements;
    std::vector<StereoFrameRef>         m_stereo_index;
  };

}  // namespace phad::dataset

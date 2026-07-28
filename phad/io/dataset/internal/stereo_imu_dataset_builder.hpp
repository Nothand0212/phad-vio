#pragma once

#include <utility>

#include "phad/io/dataset/stereo_imu_dataset.hpp"

/**
 * @file stereo_imu_dataset_builder.hpp
 * @brief StereoImuDatasetBuilder 类用于构建 StereoImuDataset 对象。
 *
 * 该文件定义了 phad::io::dataset::internal::StereoImuDatasetBuilder 类，
 * 用于构建 StereoImuDataset 对象。
 */

namespace phad::io::dataset::internal
{

  /**
   * @brief StereoImuDatasetBuilder 类用于构建 StereoImuDataset 对象。
   *
   * 该类封装了 StereoImuDataset 对象的构建过程，提供对 StereoImuDataset 对象的快速访问和引用。
   */
  class StereoImuDatasetBuilder
  {
  public:
    [[nodiscard]] static StereoImuDataset build(
        StereoImuCalibration                calibration,
        std::vector<sensor::ImuMeasurement> imu_measurements,
        std::vector<StereoFrameRef>         stereo_index,
        sensor::PixelType                   left_pixel_type,
        sensor::PixelType                   right_pixel_type )
    {
      return StereoImuDataset{
          std::move( calibration ), std::move( imu_measurements ),
          std::move( stereo_index ), left_pixel_type, right_pixel_type };
    }
  };

}  // namespace phad::io::dataset::internal

#pragma once

#include <filesystem>
#include <utility>
#include <vector>

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

  struct ImageFrameManifestEntry
  {
    common::Timestamp     timestamp;
    std::filesystem::path image_path;
  };

  class StereoImuDatasetBuilder
  {
  public:
    [[nodiscard]] static StereoImuDataset build(
        sensor::StereoImuCalibration         calibration,
        std::vector<sensor::ImuMeasurement>  imu_measurements,
        std::vector<ImageFrameManifestEntry> left_manifest,
        std::vector<ImageFrameManifestEntry> right_manifest,
        sensor::PixelType                    left_pixel_type,
        sensor::PixelType                    right_pixel_type );
  };

}  // namespace phad::io::dataset::internal

#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "phad/io/dataset/dataset_error.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_builder.hpp"
#include "phad/sensor/calibration_error.hpp"

/**
 * @file dataset_adapter_utils.hpp
 * @brief 数据集加载过程中的辅助工具函数。
 *
 * 该文件定义了 phad::io::dataset::internal 命名空间中的辅助工具函数，
 * 用于数据集加载过程中的错误处理和数据解析。
 */

namespace phad::io::dataset::internal
{

  /**
   * @brief CameraRecord 结构体用于表示相机记录。
   *
   * 该结构体封装了时间戳、图像路径和行号，提供对相机记录的快速访问和引用。
   */
  struct CameraRecord
  {
    common::Timestamp     timestamp;
    std::filesystem::path image_path;
    std::size_t           line = 0;
  };

  /**
   * @brief makeError 函数用于创建数据集加载过程中的错误信息。
   *
   * 该函数根据提供的错误代码、传感器标识、文件路径、字段名和错误原因，
   * 创建一个 DatasetError 结构体，用于表示数据集加载过程中的错误信息。
   */
  [[nodiscard]] DatasetError makeError(
      DatasetErrorCode code, std::string sensor_id,
      std::filesystem::path source_path, std::string field, std::string cause,
      std::optional<std::size_t>       line      = std::nullopt,
      std::optional<common::Timestamp> timestamp = std::nullopt );

  [[nodiscard]] DatasetResult<YAML::Node> loadYaml(
      const std::filesystem::path& path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<double> yamlScalar(
      const YAML::Node& root, std::string_view key, const std::string& sensor_id,
      const std::filesystem::path& path );

  [[nodiscard]] DatasetError mapCalibrationError(
      const sensor::CalibrationError& error, std::string sensor_id,
      std::filesystem::path source_path, std::string field );

  [[nodiscard]] sensor::CalibrationResult<sensor::RigidTransform>
  invertRigidTransform( const sensor::RigidTransform& transform );

  [[nodiscard]] bool isIdentity(
      const sensor::RigidTransform& transform ) noexcept;

  [[nodiscard]] std::optional<DatasetError> validateRequiredPath(
      const std::filesystem::path& path, bool directory );

  [[nodiscard]] DatasetResult<std::vector<CameraRecord>> parseCameraCsv(
      const std::filesystem::path& csv_path,
      const std::filesystem::path& data_path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<std::vector<sensor::ImuMeasurement>> parseImuCsv(
      const std::filesystem::path& csv_path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<std::vector<StereoFrameManifestEntry>> joinStereo(
      const std::vector<CameraRecord>& left_records,
      const std::vector<CameraRecord>& right_records,
      const std::filesystem::path&     source_path );

}  // namespace phad::io::dataset::internal

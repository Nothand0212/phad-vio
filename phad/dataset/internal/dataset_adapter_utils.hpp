#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "phad/dataset/stereo_imu_dataset.hpp"

namespace phad::dataset::internal
{

  struct CameraRecord
  {
    common::Timestamp     timestamp;
    std::filesystem::path image_path;
    std::size_t           line = 0;
  };

  [[nodiscard]] DatasetError makeError(
      DatasetErrorCode code, std::string sensor_id,
      std::filesystem::path source_path, std::string field, std::string cause,
      std::optional<std::size_t>       line      = std::nullopt,
      std::optional<common::Timestamp> timestamp = std::nullopt );

  [[nodiscard]] DatasetResult<YAML::Node> loadYaml(
      const std::filesystem::path& path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<double> positiveYamlScalar(
      const YAML::Node& root, std::string_view key, const std::string& sensor_id,
      const std::filesystem::path& path );

  [[nodiscard]] DatasetResult<sensor::RigidTransform> validateRigidTransform(
      sensor::RigidTransform transform, const std::string& sensor_id,
      const std::filesystem::path& path, std::string field );

  [[nodiscard]] sensor::RigidTransform invertRigidTransform(
      const sensor::RigidTransform& transform ) noexcept;

  [[nodiscard]] bool isIdentity(
      const sensor::RigidTransform& transform ) noexcept;

  [[nodiscard]] std::optional<DatasetError> validateRequiredPath(
      const std::filesystem::path& path, bool directory );

  [[nodiscard]] DatasetResult<std::vector<CameraRecord>> parseCameraCsv(
      const std::filesystem::path& csv_path,
      const std::filesystem::path& data_path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<std::vector<sensor::ImuMeasurement>> parseImuCsv(
      const std::filesystem::path& csv_path, const std::string& sensor_id );

  [[nodiscard]] DatasetResult<std::vector<StereoFrameRef>> joinStereo(
      const std::vector<CameraRecord>& left_records,
      const std::vector<CameraRecord>& right_records,
      const std::filesystem::path&     source_path );

}  // namespace phad::dataset::internal

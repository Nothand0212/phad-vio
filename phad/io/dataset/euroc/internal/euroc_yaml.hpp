#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

#include "phad/io/dataset/dataset_error.hpp"
#include "phad/sensor/rigid_transform.hpp"

/**
 * @file euroc_yaml.hpp
 * @brief EuRoC sensor.yaml 中 T_BS 字段的解析。
 *
 * EuRoC 的每个 sensor 目录都用同一种 4x4 T_BS 表达该 sensor 到 body frame
 * 的外参，双目/IMU 标定与真值加载都需要它，因此解析放在 EuRoC adapter
 * 内部共享，而不是复制到各个解析器中。
 */

namespace phad::io::dataset::euroc::internal
{

  /**
   * @brief 解析 sensor.yaml 的 T_BS 字段。
   *
   * 返回把 sensor 坐标系中的量转换到 body frame 的变换。字段缺失、维度
   * 错误或不构成合法刚体变换时返回带字段路径的 DatasetError。
   */
  [[nodiscard]] DatasetResult<sensor::RigidTransform> parseSensorTransform(
      const YAML::Node& root, const std::string& sensor_id,
      const std::filesystem::path& path );

}  // namespace phad::io::dataset::euroc::internal

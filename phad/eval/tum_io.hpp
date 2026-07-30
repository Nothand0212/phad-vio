#pragma once

#include <filesystem>
#include <optional>

#include "phad/common/trajectory.hpp"
#include "phad/eval/eval_error.hpp"

/**
 * @file tum_io.hpp
 * @brief TUM 轨迹格式的读写。
 *
 * 格式为无 header、空格分隔的
 * `timestamp tx ty tz qx qy qz qw`，时间戳单位为秒，与 evo 直接兼容。
 * 时间戳在写出时按整数秒加 9 位定点小数输出，读取时按字符串拆分秒与
 * 纳秒：EuRoC 量级的绝对时间（约 1.4e9 秒）无法用 double 承载纳秒精度，
 * 经由 double 会被量化到约 100 ns。
 */

namespace phad::eval
{

  [[nodiscard]] std::optional<EvalError> writeTum(
      const std::filesystem::path& path, const common::Trajectory& trajectory );

  [[nodiscard]] EvalResult<common::Trajectory> readTum(
      const std::filesystem::path& path );

}  // namespace phad::eval

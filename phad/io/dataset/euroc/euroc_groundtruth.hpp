#pragma once

#include <filesystem>

#include "phad/common/trajectory.hpp"
#include "phad/io/dataset/dataset_error.hpp"

/**
 * @file euroc_groundtruth.hpp
 * @brief EuRoC 真值轨迹加载。
 *
 * EuRoC 把真值放在 mav0/state_groundtruth_estimate0 下，位姿为 T_W_S，
 * 其中 S 由同目录 sensor.yaml 的 T_BS 关联到 body frame。本 adapter 在
 * T_BS 为 identity 时才接受该序列，从而保证产出的轨迹就是 T_W_B，
 * 与 docs/conventions.md 的状态定义一致。
 */

namespace phad::io::dataset::euroc
{

  /**
   * @brief 加载 EuRoC 序列的真值轨迹。
   *
   * 只读取 timestamp 与 pose；velocity 与 bias 列参与列数校验但不产出。
   * 时间戳为整数纳秒并严格递增，否则返回带行号的 DatasetError。
   */
  [[nodiscard]] DatasetResult<common::Trajectory> openGroundtruth(
      const std::filesystem::path& sequence_root );

}  // namespace phad::io::dataset::euroc

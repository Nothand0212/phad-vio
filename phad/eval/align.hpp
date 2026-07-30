#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "phad/eval/eval_error.hpp"

/**
 * @file align.hpp
 * @brief 固定尺度的 SE3 点集对齐。
 *
 * 双目提供可观尺度，因此对齐只求旋转与平移，不估 scale。对齐用于消除
 * 真值世界系与估计世界系之间的刚体差异：EuRoC 真值在 Vicon 系中给出，
 * 而估计的 world 由初始化时的第一个 body 状态确定。
 */

namespace phad::eval
{

  /// 判定点集是否退化的奇异值比阈值：秩小于 2 时旋转不可解。
  constexpr double kDefaultRankTolerance = 1e-8;

  /**
   * @brief 求满足 target ≈ T * source 的刚体变换（Umeyama，scale 固定为 1）。
   *
   * source 与 target 必须等长且至少包含 3 个点。点集共线或全部重合时
   * 旋转不唯一，返回 kDegenerateAlignment 而不是一个任意解。
   */
  [[nodiscard]] EvalResult<Eigen::Isometry3d> alignSe3(
      const std::vector<Eigen::Vector3d>& source,
      const std::vector<Eigen::Vector3d>& target,
      double                              rank_tolerance = kDefaultRankTolerance );

}  // namespace phad::eval

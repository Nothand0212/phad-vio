#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "phad/common/trajectory.hpp"
#include "phad/eval/eval_error.hpp"

/**
 * @file associate.hpp
 * @brief 估计轨迹与真值轨迹的时间关联。
 *
 * 对每个估计位姿取真值中时间最近的一个样本，时间差超过阈值即丢弃。
 * 丢弃本身不是错误：真值通常不覆盖序列首尾的全部图像时刻。但丢弃数量
 * 必须出现在报告里，且匹配率过低时评估失败，而不是给出一个基于少量
 * 样本的漂亮数字。
 */

namespace phad::eval
{

  /// EuRoC 真值为 200 Hz，默认阈值取其采样周期的一半。
  constexpr std::int64_t kDefaultMaxDtNs = 2'500'000;

  struct MatchedPair
  {
    std::size_t  est_index = 0;
    std::size_t  gt_index  = 0;
    std::int64_t dt_ns     = 0;  // 估计时间戳减真值时间戳
  };

  struct Association
  {
    std::vector<MatchedPair> pairs;
    std::size_t              est_total              = 0;
    std::size_t              dropped_out_of_range   = 0;
    std::size_t              dropped_over_threshold = 0;

    [[nodiscard]] double      matchRate() const noexcept;
    [[nodiscard]] std::size_t droppedTotal() const noexcept;
  };

  struct AssociationOptions
  {
    std::int64_t max_dt_ns      = kDefaultMaxDtNs;
    double       min_match_rate = 0.5;
    std::size_t  min_matches    = 3;
  };

  /**
   * @brief 关联估计与真值轨迹。
   *
   * 真值比估计稀疏时，同一个真值样本可能被多个估计位姿匹配；这不影响
   * ATE 的定义，但会体现在 dt 分布中。
   */
  [[nodiscard]] EvalResult<Association> associate(
      const common::Trajectory& est, const common::Trajectory& gt,
      const AssociationOptions& options = {} );

}  // namespace phad::eval

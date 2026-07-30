#pragma once

#include <cstddef>
#include <cstdint>

#include "phad/common/trajectory.hpp"
#include "phad/eval/associate.hpp"
#include "phad/eval/error_stats.hpp"
#include "phad/eval/eval_error.hpp"

/**
 * @file rpe.hpp
 * @brief 固定时间间隔的相对位姿误差（RPE）。
 *
 * ATE 衡量整条轨迹对齐后的绝对偏差，一次早期的偏航偏差会污染其后所有
 * 样本；RPE 只比较相隔固定时间的两个位姿之间的相对运动，因此直接给出
 * 「每秒漂移多少」。相对位姿在左乘刚体变换下不变，所以 RPE 不需要对齐，
 * 也不受估计世界系选取的影响。
 */

namespace phad::eval
{

  /// 默认间隔 1 s：漂移以「每秒多少米、多少度」阅读最直观。
  constexpr std::int64_t kDefaultRpeDeltaNs = 1'000'000'000;

  /// EuRoC 图像为 20 Hz，间隔匹配容差默认取其采样周期的一半。
  constexpr std::int64_t kDefaultRpeDeltaToleranceNs = 25'000'000;

  struct RpeOptions
  {
    AssociationOptions association;
    std::int64_t       delta_ns           = kDefaultRpeDeltaNs;  // 必须为正
    std::int64_t       delta_tolerance_ns = kDefaultRpeDeltaToleranceNs;
  };

  struct RpeReport
  {
    ErrorStats   trans_m;  // 每 delta_ns 的平移漂移
    ErrorStats   rot_deg;  // 每 delta_ns 的旋转漂移
    std::int64_t delta_ns   = kDefaultRpeDeltaNs;
    std::size_t  pair_count = 0;  // 参与统计的位姿对数
    /// 后续没有落在 delta_ns ± tolerance 内的伙伴而被跳过的位姿数。
    std::size_t dropped_no_partner = 0;
    Association association;
  };

  /**
   * @brief 计算 est 相对 gt 的固定间隔 RPE。
   *
   * 先按 options.association 做时间关联，再在匹配上的估计位姿中，为每个
   * 位姿找时间戳最接近 t + delta_ns 的后续位姿；偏离超过
   * delta_tolerance_ns 的位姿被跳过并计数，因为轨迹尾部与数据缺口处不
   * 存在相隔一个 delta 的伙伴。一对伙伴都找不到时返回 kNoDeltaPairs，
   * 而不是给出一个空统计。
   */
  [[nodiscard]] EvalResult<RpeReport> computeRpe( const common::Trajectory& est,
                                                  const common::Trajectory& gt,
                                                  const RpeOptions&         options = {} );

}  // namespace phad::eval

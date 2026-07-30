#pragma once

#include <vector>

/**
 * @file error_stats.hpp
 * @brief 误差序列的汇总统计。
 *
 * ATE 与 RPE 报告同一组统计量，因此汇总放在这里，两个指标只负责各自的
 * 误差定义。单个 RMSE 不足以判断改动好坏：median 对少量大误差不敏感，
 * max 反映最坏帧，两者一起才能区分「整体变差」与「个别帧崩掉」。
 */

namespace phad::eval
{

  struct ErrorStats
  {
    double rmse   = 0.0;
    double mean   = 0.0;
    double median = 0.0;
    double stddev = 0.0;
    double max    = 0.0;
  };

  /// 汇总非负误差序列；errors 为空时返回全零统计。
  [[nodiscard]] ErrorStats computeStats( std::vector<double> errors );

}  // namespace phad::eval

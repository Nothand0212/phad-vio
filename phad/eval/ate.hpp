#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "phad/common/trajectory.hpp"
#include "phad/eval/align.hpp"
#include "phad/eval/associate.hpp"
#include "phad/eval/eval_error.hpp"

/**
 * @file ate.hpp
 * @brief 绝对轨迹误差（ATE）。
 *
 * 流程为时间关联、固定尺度 SE3 对齐、逐样本误差统计。平移 RMSE 是主
 * 指标，旋转误差单独以度给出。逐样本误差随报告一起返回，供离线绘图与
 * 分布检查使用，避免只看到一个汇总数字。
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

  struct PoseErrorSample
  {
    common::Timestamp timestamp;                 // 估计侧时间戳
    std::int64_t      dt_ns                = 0;  // 与匹配真值的时间差
    double            trans_m              = 0.0;
    double            rot_deg              = 0.0;
    Eigen::Vector3d   aligned_est_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d   gt_position          = Eigen::Vector3d::Zero();
  };

  struct AteReport
  {
    ErrorStats                   trans_m;
    ErrorStats                   rot_deg;
    Eigen::Isometry3d            T_align = Eigen::Isometry3d::Identity();
    Association                  association;
    std::vector<PoseErrorSample> samples;
  };

  struct AteOptions
  {
    AssociationOptions association;
    double             rank_tolerance = kDefaultRankTolerance;
  };

  [[nodiscard]] EvalResult<AteReport> computeAte(
      const common::Trajectory& est, const common::Trajectory& gt,
      const AteOptions& options = {} );

}  // namespace phad::eval

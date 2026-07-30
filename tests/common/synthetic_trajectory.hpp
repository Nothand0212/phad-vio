#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "phad/common/trajectory.hpp"

/**
 * @file synthetic_trajectory.hpp
 * @brief 评估测试共用的合成轨迹构造。
 *
 * 评估模块的测试不依赖数据集，全部使用解析可控的合成轨迹。
 */

namespace phad::testing
{

  /// EuRoC MH_01 的首个图像时间戳，用于覆盖真实量级的纳秒时间。
  constexpr std::int64_t kEurocEpochNs = 1'403'636'579'763'555'584;
  constexpr std::int64_t kStepNs       = 50'000'000;

  [[nodiscard]] inline common::TimedPose makePose(
      std::int64_t timestamp_ns, const Eigen::Vector3d& position,
      const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity() )
  {
    common::TimedPose pose;
    pose.timestamp           = common::Timestamp{ timestamp_ns };
    pose.T_W_B.linear()      = rotation.normalized().toRotationMatrix();
    pose.T_W_B.translation() = position;
    return pose;
  }

  /**
   * @brief 螺旋轨迹：位置不共线、姿态随时间变化。
   *
   * 位置不共线是对齐可解的前提，姿态变化让旋转误差不会退化为常量。
   */
  [[nodiscard]] inline common::Trajectory makeHelix(
      std::size_t count, std::int64_t start_ns = kEurocEpochNs,
      std::int64_t step_ns = kStepNs )
  {
    std::vector<common::TimedPose> poses;
    poses.reserve( count );
    for ( std::size_t index = 0; index < count; ++index )
    {
      const double             angle = 0.2 * static_cast<double>( index );
      const Eigen::Vector3d    position{ std::cos( angle ), std::sin( angle ),
                                      0.1 * static_cast<double>( index ) };
      const Eigen::Quaterniond rotation{
          Eigen::AngleAxisd{ angle, Eigen::Vector3d::UnitZ() } };
      poses.push_back( makePose(
          start_ns + static_cast<std::int64_t>( index ) * step_ns, position,
          rotation ) );
    }
    return common::Trajectory::create( std::move( poses ) ).value();
  }

  [[nodiscard]] inline common::Trajectory transformed(
      const common::Trajectory& source, const Eigen::Isometry3d& transform )
  {
    std::vector<common::TimedPose> poses = source.poses();
    for ( common::TimedPose& pose : poses )
    {
      pose.T_W_B = transform * pose.T_W_B;
    }
    return common::Trajectory::create( std::move( poses ) ).value();
  }

  [[nodiscard]] inline Eigen::Isometry3d makeTransform(
      double yaw_rad, const Eigen::Vector3d& translation )
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() =
        Eigen::AngleAxisd{ yaw_rad, Eigen::Vector3d::UnitZ() }
            .toRotationMatrix();
    transform.translation() = translation;
    return transform;
  }

}  // namespace phad::testing

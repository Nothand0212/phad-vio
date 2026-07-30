#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <opencv2/core.hpp>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/common/trajectory.hpp"

/**
 * @file trajectory_panel.hpp
 * @brief 俯视 x-y 轨迹面板。
 *
 * 面板把一条轨迹的水平投影固定在画布内，并在回放时标出当前时刻的位置。
 * 它只依赖 OpenCV 的绘图能力，不打开窗口，因此在无显示环境下也能渲染与
 * 测试。构造时把轨迹一次性投影为像素折线，之后每帧只画一个标记。
 */

namespace phad::viz
{

  struct TrajectoryPanelOptions
  {
    int width_px  = 480;
    int height_px = 480;
    int margin_px = 24;  // 轨迹与画布边缘之间的留白
  };

  /**
   * @brief 固定视野的俯视轨迹面板。
   *
   * 视野在构造时由整条轨迹的 x-y 包围盒确定，回放过程中不缩放也不平移，
   * 这样面板上的位置变化只反映位姿变化。x 与 y 使用同一比例，轨迹形状
   * 不被拉伸。
   */
  class TrajectoryPanel
  {
  public:
    /**
     * @brief 按整条轨迹的水平包围盒建立面板。
     * @throws std::invalid_argument 画布尺寸或留白没有留下可用区域。
     */
    explicit TrajectoryPanel( const common::Trajectory&     trajectory,
                              const TrajectoryPanelOptions& options = {} );

    /// 世界系位置在画布上的像素位置；竖直分量不参与投影。
    [[nodiscard]] cv::Point project(
        const Eigen::Vector3d& position_W ) const noexcept;

    /// 渲染轨迹，并在不晚于 timestamp 的最后一个位姿上画出当前位置标记。
    [[nodiscard]] cv::Mat render( common::Timestamp timestamp ) const;

  private:
    [[nodiscard]] std::size_t indexAt(
        common::Timestamp timestamp ) const noexcept;

    double                         m_scale_px_per_m = 1.0;
    Eigen::Vector2d                m_origin_W       = Eigen::Vector2d::Zero();
    cv::Point2d                    m_origin_px{ 0.0, 0.0 };
    std::vector<common::Timestamp> m_timestamps;
    std::vector<cv::Point>         m_points;
    cv::Mat                        m_background;
  };

}  // namespace phad::viz

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"

namespace phad::frontend
{

  using LandmarkId = common::LandmarkId;

  /**
   * @brief 描述观测的立体匹配状态。
   *
   * 用于描述观测的立体匹配状态，包括：
   * - kValid：观测有效
   * - kNoRightMatch：右目匹配失败
   * - kInvalidDisparity：视差无效
   * - kDepthOutOfRange：深度超出范围
   */
  enum class StereoStatus : std::uint8_t
  {
    kValid            = 0,
    kNoRightMatch     = 1,
    kInvalidDisparity = 2,
    kDepthOutOfRange  = 3
  };


  /**
   * @brief 描述一个观测。
   *
   * 用于描述一个观测，包括：
   * - id：观测的特征点 ID
   * - left_pixel：观测的左目像素坐标
   * - disparity_px：观测的视差
   * - status：观测的立体匹配状态
   * - length：观测的长度
   */
  struct TrackObservation
  {
    LandmarkId      id;
    Eigen::Vector2d left_pixel;
    double          disparity_px;  // meaningful only when status == kValid
    StereoStatus    status;
    std::uint32_t   length;  // observations so far, including this frame
  };


  /**
   * @brief 描述一帧的统计信息。
   *
   * 用于描述一帧的统计信息，包括：
   * - tracked：跟踪到的特征点数
   * - detected：检测到的特征点数
   * - forward_backward_rejected：前向/后向匹配失败数
   * - epipolar_rejected：对极几何筛选失败数
   * - disparity_rejected：视差筛选失败数
   * - depth_rejected：深度筛选失败数
   * - epipolar_median_px：对极几何筛选成功的视差中位数
   * - epipolar_p95_px：对极几何筛选成功的视差第95百分位数
   * - track_length_median：跟踪长度中位数
   * - track_length_max：跟踪长度最大值
   */
  struct FrameStats
  {
    std::uint32_t tracked;   // temporal LK survivors
    std::uint32_t detected;  // new detections this frame
    std::uint32_t forward_backward_rejected;
    std::uint32_t epipolar_rejected;
    std::uint32_t disparity_rejected;
    std::uint32_t depth_rejected;
    double        epipolar_median_px;  // right-match successes only
    double        epipolar_p95_px;
    std::uint32_t track_length_median;
    std::uint32_t track_length_max;
    /// Evictable tracks removed this frame to free GFTT slots (lazy eviction).
    std::uint32_t evicted = 0;
  };


  /**
   * @brief 描述一帧的观测。
   *
   * 用于描述一帧的观测，包括：
   * - timestamp：观测的时间戳
   * - observations：观测的特征点列表
   * - stats：一帧的统计信息
   */
  struct FrameTracks
  {
    common::Timestamp             timestamp;
    std::vector<TrackObservation> observations;
    FrameStats                    stats;
  };

}  // namespace phad::frontend

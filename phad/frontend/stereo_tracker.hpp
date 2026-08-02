#pragma once

#include <memory>
#include <span>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/frontend/stereo_tracks.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file phad/frontend/stereo_tracker.hpp
 * @brief 描述立体匹配跟踪器。
 *
 * 用于描述立体匹配跟踪器，包括：
 * - StereoTracker：立体匹配跟踪器类
 * - StereoTrackerOptions：立体匹配跟踪器选项
 * - FrameTracks：一帧的观测
 * - TrackObservation：一个观测
 * - FrameStats：一帧的统计信息
 * - StereoStatus：立体匹配状态
 */

namespace phad::frontend
{

  /**
   * @brief 立体匹配跟踪器选项。
   *
   * 用于描述立体匹配跟踪器选项，包括：
   * - max_tracks：最大跟踪数
   * - quality_level：质量等级
   * - min_distance_px：最小距离
   * - mask_radius_px：掩码半径
   * - lk_window_px：LK窗口半径
   * - lk_pyramid_levels：LK金字塔层数
   * - forward_backward_px：前向/后向匹配距离
   * - max_epipolar_px：最大对极几何距离
   * - min_disparity_px：最小视差
   * - min_depth_m：最小深度
   * - max_depth_m：最大深度
   * - stereo_sad_half_win_px：立体 SAD 半窗口
   * - stereo_row_tol_px：立体行容差
   * - stereo_bidir_px：立体双向匹配距离
   * - stereo_uniq_ratio：次优/最优 SAD 相对裕度，(second-best)/best 须 ≥ 该值
   * - stereo_check_bidir：是否做同行反向一致性（合成全同 blob 可关）
   */
  struct StereoTrackerOptions
  {
    int    max_tracks             = 200;
    double quality_level          = 0.01;
    double min_distance_px        = 20.0;
    int    mask_radius_px         = 20;
    int    lk_window_px           = 21;
    int    lk_pyramid_levels      = 3;
    double forward_backward_px    = 0.5;
    double max_epipolar_px        = 1.5;
    double min_disparity_px       = 0.5;
    double min_depth_m            = 0.3;
    double max_depth_m            = 40.0;
    int    stereo_sad_half_win_px = 7;
    int    stereo_row_tol_px      = 0;
    double stereo_bidir_px        = 0.5;
    double stereo_uniq_ratio      = 0.5;
    bool   stereo_check_bidir     = true;
  };

  /**
   * @brief 左目时间戳 LK 跟踪器，带 GFTT 重填。
   *
   * 切片 M2.2-② 只填充左目观测；右目匹配状态保持 `kNoRightMatch` 直到立体匹配切片。
   */
  class StereoTracker
  {
  public:
    StereoTracker( camera::RectifiedStereoCalibration calibration,
                   StereoTrackerOptions               options = {} );
    ~StereoTracker();

    StereoTracker( const StereoTracker& )            = delete;
    StereoTracker& operator=( const StereoTracker& ) = delete;
    StereoTracker( StereoTracker&& ) noexcept;
    StereoTracker& operator=( StereoTracker&& ) noexcept;

    [[nodiscard]] FrameTracks process(
        const sensor::StereoFrame& rectified );

    /**
     * @brief 按 LandmarkId 擦除存活 tracks；未知 id 忽略；不改 next_id。
     */
    void dropTracks( std::span<const LandmarkId> ids );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::frontend

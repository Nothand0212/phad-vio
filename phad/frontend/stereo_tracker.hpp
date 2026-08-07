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
    double quality_level          = 0.003;
    double min_distance_px        = 20.0;
    int    mask_radius_px         = 20;
    int    lk_window_px           = 21;
    int    lk_pyramid_levels      = 4;
    double forward_backward_px    = 0.5;
    double max_epipolar_px        = 1.5;
    double min_disparity_px       = 2.0;
    double min_depth_m            = 0.3;
    double max_depth_m            = 25.0;
    int    stereo_sad_half_win_px = 7;
    int    stereo_row_tol_px      = 0;
    double stereo_bidir_px        = 0.5;
    double stereo_uniq_ratio      = 0.5;
    bool   stereo_check_bidir     = true;

    // pre-M4 小片: Census 变换立体匹配(5×5 窗, 24bit Hamming 距离)。
    // 实测否决(2026-08-07, cbb4505 双路径):
    //   V2_03  SAD-only ATE 3.628m(≈ORB-SLAM3 官方)/0 锚跳 vs census
    //          兜底 on 7937m/225 次 >5m 锚跳 —— census 在暗帧产出的匹配
    //          污染 PnP → 反复丢跟踪 → 重建锚错。
    //   MH_01  SAD-only 0.0810 vs census on 0.0915(+13%)。
    // 旧的"cam1 欠曝光 → SAD 全败"诊断基于 census-primary 时代观察,
    // 双路径下 SAD 主路径在 V2_03 暗段正常产视差(启动段 296 视差观测),
    // 无需兜底。默认关闭;代码与 flag 保留供 M4 复测。
    bool enable_census = false;

    // Attribution A/B switches (Slice ⑥ mechanisms, default on). Disabling
    // any of these recreates a pre-slice-⑥ frontend behaviour for
    // per-mechanism contribution measurement. Not a production knob.
    bool enable_clahe       = true;
    bool enable_median_flow = true;
    bool enable_fransac     = true;
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

    /**
     * @brief 标记 tracks 为补点时可驱逐；未知 id 忽略；不改 next_id。
     *
     * 被标记者在被挤掉前仍参与 LK 与 FrameTracks。detectNewTracks 按非
     * evictable 计占用；缺槽时按 id 升序先驱逐再 GFTT。
     */
    void markEvictable( std::span<const LandmarkId> ids );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::frontend

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "apps/candidate_pipeline.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/common/trajectory.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/sync/stereo_pair_synchronizer.hpp"

/**
 * @file offline_vo_session.hpp
 * @brief 离线双目 VO 编排：replay → rectify → track → glue → estimate。
 *
 * 不落盘、不算 ATE/RPE。probe 与 phad_vo_bench 共用，保证 diag 列合同一致。
 */

namespace phad::apps
{

  struct SessionError
  {
    std::string detail;
  };

  struct OfflineVoSessionOptions
  {
    std::filesystem::path          sequence_root;
    frontend::StereoTrackerOptions tracker;
    estimator::EstimatorOptions    estimator;
    std::optional<std::uint64_t>   max_frames;
    bool                           collect_timing = true;
    /// After estimator cull/cheirality erasures, drop matching frontend
    /// tracks. Default true (Slice ④c). False is A/B only — do not use as
    /// a production default without a MH_01 gate.
    bool drop_culled_tracks = true;
    /// When drop_culled_tracks is true, skip dropTracks when mean-cull
    /// outliers_culled >= this threshold. Default 4 (Slice ④f). 0 disables
    /// skip-by-threshold (always drop when list non-empty).
    int skip_drop_min_culled = 4;
    /// MH_05 Probe B jsonl path; empty → writer not constructed (default off).
    std::filesystem::path probe_b_path{};
    /// M4.4 fixed-production-epoch keyframe shadow CSV path. Empty disables
    /// the CLI-only probe; not part of config_hash.
    std::filesystem::path keyframe_shadow_probe_path{};
    /// M4.4 one-step gyro prediction / posterior pose+bg CSV. Empty disables
    /// the CLI-only probe; requires estimator.enable_imu and does not enter
    /// config_hash.
    std::filesystem::path vio_state_probe_path{};
    /// M4.4 one-time production static-initialization snapshot CSV. Empty
    /// disables the CLI-only probe; requires estimator.enable_imu and does
    /// not enter config_hash.
    std::filesystem::path vio_init_probe_path{};
    /// M4.4 persistent fixed-lag read-only shadow CSV. Empty disables the
    /// CLI-only probe; requires estimator.enable_imu and does not enter
    /// config_hash.
    std::filesystem::path fixed_lag_shadow_probe_path{};
    /// Probe: after skip-drop, defer dropTracks of the first K culled ids
    /// (sorted by LandmarkId) until next tracker.process. Default 0 = ④f
    /// (skip and never defer-drop). CLI-only; not in config_hash.
    int defer_drop_topk = 0;
    /// Probe: on skip-drop, mark culled ids evictable for lazy GFTT slot
    /// reclaim. Default false. CLI-only; not in config_hash.
    bool evict_skip_culled = false;
    /// Drop skip-culled ids after N consecutive frames still present in
    /// FrameTracks. Default 5 (M3.3 candidate B productized). 0 = off.
    /// In flattenConfig / config_hash; CLI --zombie-drop-age may override.
    int zombie_drop_age = 5;
    /// M4.4 P2a: run a candidate-owned batch twin after each production
    /// frame. CLI-only; not in flattenConfig / config_hash. Candidate
    /// terminal failure stops only the candidate, never production.
    bool enable_candidate = false;
    // ---- M4.3 dropout 注入 (plan 定案 G; CLI-only, 不进 config_hash) ----
    // 帧号 ∈ [start, start+frames) 时对观测做无放回子采样 (keep_ratio 0 =
    // 全清; 0.3 = 保留 ~30%) 后再传 estimator。前端 tracker 照跑不注入。
    // 三参数齐备才激活 (bench CLI 校验)。参照 enable_probe_b 模式。
    std::optional<std::uint64_t> dropout_start_frame;
    std::optional<std::uint64_t> dropout_frames;
    std::optional<double>        dropout_keep_ratio;
  };

  struct VoDiagRow
  {
    std::int64_t  timestamp_ns = 0;
    std::string   status;
    std::uint32_t num_observations         = 0;
    std::uint32_t num_landmarks            = 0;
    std::uint32_t num_shared               = 0;
    std::uint32_t num_disparity            = 0;  // Slice: obs with disparity_px > 0
    bool          low_connectivity         = false;
    std::uint32_t window_size              = 0;
    std::uint64_t prior_key                = 0;
    double        reproj_rms_before_px     = 0.0;
    double        reproj_rms_after_px      = 0.0;
    std::uint32_t num_cheirality           = 0;
    std::uint32_t lm_iterations            = 0;
    double        max_window_pose_shift_m  = 0.0;
    std::uint32_t segment_id               = 0;
    bool          pnp_success              = false;
    std::uint32_t pnp_inliers              = 0;
    std::uint32_t outliers_culled          = 0;
    double        reproj_rms_after_cull_px = 0.0;
    bool          is_keyframe              = false;  // Slice ⑤
    // ---- M4.3 bias 三轴 (Q3/C12) ----
    // 优化后 bias (接受帧 = LM 回写值; 被拒帧 = 上一接受值)。
    // IMU-off 恒 0.0。列追加在 CSV 末尾, 保证原列与 c1d3481 逐字节
    // 一致 (plan F 定案)。
    double bias_gyro_x = 0.0;
    double bias_gyro_y = 0.0;
    double bias_gyro_z = 0.0;
    double bias_acc_x  = 0.0;
    double bias_acc_y  = 0.0;
    double bias_acc_z  = 0.0;
  };

  struct FrameCounts
  {
    std::uint64_t image_frames           = 0;
    std::uint64_t ok                     = 0;
    std::uint64_t rejected               = 0;
    std::uint64_t failed                 = 0;
    std::uint64_t low_connectivity       = 0;
    std::uint64_t segments               = 0;
    std::uint64_t reanchors              = 0;
    std::uint64_t seed_rejected          = 0;
    std::uint64_t pnp_successes          = 0;
    std::uint64_t pnp_fallbacks          = 0;
    std::uint64_t outliers_culled        = 0;
    std::uint64_t outliers_culled_unique = 0;
    /// Cumulative successful reopt *rounds* (sum of outlier_reopt_rounds).
    std::uint64_t outlier_reopts = 0;
    /// Frames where dropTracks was skipped because outliers_culled >= N.
    std::uint64_t drops_skipped     = 0;
    std::uint64_t deferred_drops    = 0;  // 冲刷次数
    std::uint64_t deferred_drop_ids = 0;  // 累计 drop 的 id 个数
    /// Frames where skip marked culled ids evictable (probe).
    std::uint64_t evictable_marked = 0;
    /// Cumulative tracks evicted by frontend lazy slot reclaim.
    std::uint64_t tracks_evicted = 0;
    /// Flush count for zombie-age dropTracks (probe).
    std::uint64_t zombie_age_drops = 0;
    /// Cumulative ids dropped by zombie-age probe.
    std::uint64_t zombie_age_drop_ids = 0;
    /// Slice ⑤: keyframe counts.
    std::uint64_t total_keyframes         = 0;
    std::uint64_t total_track_only_frames = 0;
    /// M4.4: static gyro audit 未 ready 的输入帧数；视觉仍可正常接受。
    /// 进 summary.json，不进 config_hash；IMU-off 恒 0。
    std::uint64_t init_pending_frames = 0;
  };

  struct StageTiming
  {
    double mean = 0.0;
    double p95  = 0.0;
    double max  = 0.0;
  };

  struct StageTimings
  {
    StageTiming rectify;
    StageTiming frontend;
    StageTiming estimator;
    StageTiming total;
  };

  struct ReprojSummary
  {
    double median_px = 0.0;
    double p95_px    = 0.0;
  };

  struct OfflineVoSessionResult
  {
    std::optional<common::Trajectory> trajectory;     // all accepted frames (est.tum)
    std::optional<common::Trajectory> kf_trajectory;  // keyframes only (kf.tum)
    std::vector<VoDiagRow>            diag;
    FrameCounts                       counts;
    sync::StereoPairDiagnostics       sync;
    std::vector<std::string>          warnings;
    common::Timestamp                 first_image_ts;
    common::Timestamp                 last_image_ts;
    double                            wall_s = 0.0;
    StageTimings                      timings;
    ReprojSummary                     reproj;
    /// M4.4 P2a: candidate-owned twin result; present iff enable_candidate.
    /// Candidate failure never fails the session.
    std::optional<CandidateRunResult> candidate;
    /// M4.4 P2a: candidate process 累计 wall time（独立计时，不混入
    /// production rectify/frontend/estimator stage timing）。
    double                      candidate_wall_s = 0.0;
    std::optional<SessionError> error;
  };

  [[nodiscard]] OfflineVoSessionResult runOfflineVoSession(
      const OfflineVoSessionOptions& options );

  /// probe 与 bench 共用，保证 diag.csv 逐字节一致。
  [[nodiscard]] std::optional<SessionError> writeDiagCsv(
      const std::filesystem::path& path, const std::vector<VoDiagRow>& rows );

}  // namespace phad::apps

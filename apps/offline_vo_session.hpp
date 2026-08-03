#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

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
  };

  struct VoDiagRow
  {
    std::int64_t  timestamp_ns = 0;
    std::string   status;
    std::uint32_t num_observations        = 0;
    std::uint32_t num_landmarks           = 0;
    std::uint32_t num_shared              = 0;
    bool          low_connectivity        = false;
    std::uint32_t window_size             = 0;
    std::uint64_t prior_key               = 0;
    double        reproj_rms_before_px    = 0.0;
    double        reproj_rms_after_px     = 0.0;
    std::uint32_t num_cheirality          = 0;
    std::uint32_t lm_iterations           = 0;
    double        max_window_pose_shift_m = 0.0;
    std::uint32_t segment_id               = 0;
    bool          pnp_success              = false;
    std::uint32_t pnp_inliers              = 0;
    std::uint32_t outliers_culled          = 0;
    double        reproj_rms_after_cull_px = 0.0;
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
    std::uint64_t drops_skipped = 0;
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
    std::optional<common::Trajectory> trajectory;
    std::vector<VoDiagRow>            diag;
    FrameCounts                       counts;
    sync::StereoPairDiagnostics       sync;
    std::vector<std::string>          warnings;
    common::Timestamp                 first_image_ts;
    common::Timestamp                 last_image_ts;
    double                            wall_s = 0.0;
    StageTimings                      timings;
    ReprojSummary                     reproj;
    std::optional<SessionError>       error;
  };

  [[nodiscard]] OfflineVoSessionResult runOfflineVoSession(
      const OfflineVoSessionOptions& options );

  /// probe 与 bench 共用，保证 diag.csv 逐字节一致。
  [[nodiscard]] std::optional<SessionError> writeDiagCsv(
      const std::filesystem::path& path, const std::vector<VoDiagRow>& rows );

}  // namespace phad::apps

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
    std::uint32_t segment_id              = 0;
  };

  struct FrameCounts
  {
    std::uint64_t image_frames     = 0;
    std::uint64_t ok               = 0;
    std::uint64_t rejected         = 0;
    std::uint64_t failed           = 0;
    std::uint64_t low_connectivity = 0;
    std::uint64_t segments         = 0;
    std::uint64_t reanchors        = 0;
    std::uint64_t seed_rejected    = 0;
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

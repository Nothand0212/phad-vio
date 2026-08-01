#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "phad/bench/code_identity.hpp"

/**
 * @file run_summary.hpp
 * @brief meta.json / summary.json 的 schema 与 JSON 序列化入口。
 *
 * 不依赖 phad::eval；统计量用本地 StatBlock。
 */

namespace phad::bench
{

  struct StatBlock
  {
    double rmse   = 0.0;
    double mean   = 0.0;
    double median = 0.0;
    double stddev = 0.0;
    double max    = 0.0;
  };

  struct MetricReport
  {
    StatBlock trans;
    StatBlock rot_deg;
  };

  struct TrajectorySummary
  {
    std::uint64_t image_frames    = 0;
    std::uint64_t poses_written   = 0;
    std::uint64_t ok              = 0;
    std::uint64_t rejected        = 0;
    std::uint64_t failed          = 0;
    double        completion_rate = 0.0;
    double        coverage_rate   = 0.0;
    std::uint64_t segments        = 0;
  };

  struct SyncSummary
  {
    std::uint64_t pushed_left            = 0;
    std::uint64_t pushed_right           = 0;
    std::uint64_t emitted_stereo         = 0;
    std::uint64_t dropped_left           = 0;
    std::uint64_t dropped_right          = 0;
    std::uint64_t dropped_left_overflow  = 0;
    std::uint64_t dropped_right_overflow = 0;
    std::size_t   max_left_queue         = 0;
    std::size_t   max_right_queue        = 0;
  };

  struct RobustnessSummary
  {
    std::uint64_t rejected               = 0;
    std::uint64_t failed                 = 0;
    std::uint64_t low_connectivity       = 0;
    std::uint64_t cheirality             = 0;
    std::uint64_t reanchors              = 0;
    std::uint64_t pnp_successes          = 0;
    std::uint64_t pnp_fallbacks          = 0;
    std::uint64_t outliers_culled        = 0;
    std::uint64_t outliers_culled_unique = 0;
    std::uint64_t outlier_reopts         = 0;
  };

  struct StageTiming
  {
    double mean = 0.0;
    double p95  = 0.0;
    double max  = 0.0;
  };

  struct TimingSummary
  {
    double                wall_s = 0.0;
    std::optional<double> rtf;
    StageTiming           rectify;
    StageTiming           frontend;
    StageTiming           estimator;
    StageTiming           total;
  };

  enum class RunStatus : std::uint8_t
  {
    kCompleted             = 0,
    kCompletedWithFailures = 1,
    kCompletedWithWarnings = 2,
    kEvalFailed            = 3,
    kFailed                = 4,
  };

  [[nodiscard]] std::string_view runStatusToString( RunStatus status );

  struct RunMeta
  {
    int                      schema_version = 1;
    std::string              sequence;
    std::string              sequence_root;
    std::string              created_utc;
    CodeIdentity             code;
    std::string              config_json;  // ConfigSnapshot::toJson() 文本
    std::string              config_label;
    std::string              config_hash;
    std::string              config_canonical_text;
    std::string              bench_root;
    std::string              output_dir;
    std::string              layout_template;  // "template" | "override"
    std::vector<std::string> warnings;

    [[nodiscard]] std::string toJson() const;
  };

  struct RunSummary
  {
    int                         schema_version = 1;
    RunStatus                   status         = RunStatus::kFailed;
    std::string                 sequence;
    std::string                 git_commit_short = "unknown";
    bool                        git_dirty        = false;
    std::string                 config_label;
    std::string                 config_hash;
    TrajectorySummary           trajectory;
    std::optional<MetricReport> ate;
    std::optional<MetricReport> rpe;
    RobustnessSummary           robustness;
    TimingSummary               timing;
    std::optional<SyncSummary>  sync;
    std::vector<std::string>    warnings;

    [[nodiscard]] std::string toJson() const;
  };

}  // namespace phad::bench

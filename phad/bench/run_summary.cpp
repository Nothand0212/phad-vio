#include "phad/bench/run_summary.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>

/**
 * @file run_summary.cpp
 * @brief RunMeta / RunSummary JSON 序列化（nlohmann 仅此 TU）。
 */

namespace phad::bench
{
  namespace
  {

    using json = nlohmann::json;

    [[nodiscard]] json statToJson( const StatBlock& stats )
    {
      return json{ { "rmse", stats.rmse },
                   { "mean", stats.mean },
                   { "median", stats.median },
                   { "stddev", stats.stddev },
                   { "max", stats.max } };
    }

    [[nodiscard]] json metricToJson( const MetricReport& report )
    {
      return json{ { "trans", statToJson( report.trans ) },
                   { "rot_deg", statToJson( report.rot_deg ) } };
    }

    [[nodiscard]] json stageToJson( const StageTiming& stage )
    {
      return json{ { "mean", stage.mean },
                   { "p95", stage.p95 },
                   { "max", stage.max } };
    }

    [[nodiscard]] json codeToJson( const CodeIdentity& code )
    {
      json object = json::object();
      if ( code.git_commit.has_value() )
      {
        object[ "git_commit" ] = *code.git_commit;
      }
      else
      {
        object[ "git_commit" ] = nullptr;
      }
      object[ "git_commit_short" ] = code.git_commit_short;
      object[ "git_dirty" ]        = code.git_dirty;
      if ( code.git_branch.has_value() )
      {
        object[ "git_branch" ] = *code.git_branch;
      }
      else
      {
        object[ "git_branch" ] = nullptr;
      }
      return object;
    }

  }  // namespace

  std::string_view runStatusToString( RunStatus status )
  {
    switch ( status )
    {
      case RunStatus::kCompleted:
        return "completed";
      case RunStatus::kCompletedWithFailures:
        return "completed_with_failures";
      case RunStatus::kCompletedWithWarnings:
        return "completed_with_warnings";
      case RunStatus::kEvalFailed:
        return "eval_failed";
      case RunStatus::kFailed:
        return "failed";
    }
    throw std::invalid_argument( "runStatusToString: unknown RunStatus" );
  }

  std::string RunMeta::toJson() const
  {
    json root                = json::object();
    root[ "schema_version" ] = schema_version;
    root[ "sequence" ]       = sequence;
    root[ "sequence_root" ]  = sequence_root;
    root[ "created_utc" ]    = created_utc;
    root[ "code" ]           = codeToJson( code );

    if ( config_json.empty() )
    {
      root[ "config" ] = json::object();
    }
    else
    {
      root[ "config" ] = json::parse( config_json );
    }
    root[ "config_label" ]          = config_label;
    root[ "config_hash" ]           = config_hash;
    root[ "config_canonical_text" ] = config_canonical_text;
    root[ "paths" ]                 = json{
                        { "bench_root", bench_root },
                        { "output_dir", output_dir },
                        { "layout_template", layout_template },
    };
    root[ "warnings" ] = warnings;
    return root.dump( 2 );
  }

  std::string RunSummary::toJson() const
  {
    json root                = json::object();
    root[ "schema_version" ] = schema_version;
    root[ "status" ]         = runStatusToString( status );
    root[ "sequence" ]       = sequence;
    root[ "code" ]           = json{
                  { "git_commit_short", git_commit_short },
                  { "git_dirty", git_dirty },
    };
    root[ "config_label" ] = config_label;
    root[ "config_hash" ]  = config_hash;
    root[ "trajectory" ]   = json{
          { "image_frames", trajectory.image_frames },
          { "poses_written", trajectory.poses_written },
          { "ok", trajectory.ok },
          { "rejected", trajectory.rejected },
          { "failed", trajectory.failed },
          { "completion_rate", trajectory.completion_rate },
          { "coverage_rate", trajectory.coverage_rate },
          { "segments", trajectory.segments },
    };

    if ( ate.has_value() )
    {
      root[ "ate" ] = metricToJson( *ate );
    }
    else
    {
      root[ "ate" ] = nullptr;
    }
    if ( rpe.has_value() )
    {
      root[ "rpe" ] = metricToJson( *rpe );
    }
    else
    {
      root[ "rpe" ] = nullptr;
    }

    root[ "robustness" ] = json{
        { "rejected", robustness.rejected },
        { "failed", robustness.failed },
        { "low_connectivity", robustness.low_connectivity },
        { "cheirality", robustness.cheirality },
        { "reanchors", robustness.reanchors },
        { "pnp_successes", robustness.pnp_successes },
        { "pnp_fallbacks", robustness.pnp_fallbacks },
        { "outliers_culled", robustness.outliers_culled },
        { "outliers_culled_unique", robustness.outliers_culled_unique },
        { "outlier_reopts", robustness.outlier_reopts },
        { "drops_skipped", robustness.drops_skipped },
        { "deferred_drops", robustness.deferred_drops },
        { "deferred_drop_ids", robustness.deferred_drop_ids },
    };

    json timing_json = json{
        { "wall_s", timing.wall_s },
        { "rectify", stageToJson( timing.rectify ) },
        { "frontend", stageToJson( timing.frontend ) },
        { "estimator", stageToJson( timing.estimator ) },
        { "total", stageToJson( timing.total ) },
    };
    if ( timing.rtf.has_value() )
    {
      timing_json[ "rtf" ] = *timing.rtf;
    }
    else
    {
      timing_json[ "rtf" ] = nullptr;
    }
    root[ "timing" ] = timing_json;
    if ( sync.has_value() )
    {
      root[ "sync" ] = json{
          { "pushed_left", sync->pushed_left },
          { "pushed_right", sync->pushed_right },
          { "emitted_stereo", sync->emitted_stereo },
          { "dropped_left", sync->dropped_left },
          { "dropped_right", sync->dropped_right },
          { "dropped_left_overflow", sync->dropped_left_overflow },
          { "dropped_right_overflow", sync->dropped_right_overflow },
          { "max_left_queue", sync->max_left_queue },
          { "max_right_queue", sync->max_right_queue },
      };
    }
    root[ "warnings" ] = warnings;
    return root.dump( 2 );
  }

}  // namespace phad::bench

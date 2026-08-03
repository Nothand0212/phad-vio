#include "phad/bench/run_summary.hpp"

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

namespace
{

  using json = nlohmann::json;
  using phad::bench::MetricReport;
  using phad::bench::RunMeta;
  using phad::bench::RunStatus;
  using phad::bench::runStatusToString;
  using phad::bench::RunSummary;
  using phad::bench::StatBlock;

  TEST( RunSummaryTest, StatusStringMapping )
  {
    EXPECT_EQ( runStatusToString( RunStatus::kCompleted ), "completed" );
    EXPECT_EQ( runStatusToString( RunStatus::kCompletedWithFailures ),
               "completed_with_failures" );
    EXPECT_EQ( runStatusToString( RunStatus::kCompletedWithWarnings ),
               "completed_with_warnings" );
    EXPECT_EQ( runStatusToString( RunStatus::kEvalFailed ), "eval_failed" );
    EXPECT_EQ( runStatusToString( RunStatus::kFailed ), "failed" );
  }

  TEST( RunSummaryTest, NullMetricsSerializeAsNull )
  {
    RunSummary summary;
    summary.status                     = RunStatus::kEvalFailed;
    summary.sequence                   = "MH_01_easy";
    summary.git_commit_short           = "abcdef1";
    summary.config_label               = "default";
    summary.config_hash                = "9f2ab41c";
    summary.trajectory.image_frames    = 10;
    summary.trajectory.ok              = 8;
    summary.trajectory.completion_rate = 0.8;
    summary.ate                        = std::nullopt;
    summary.rpe                        = std::nullopt;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "status" ), "eval_failed" );
    EXPECT_TRUE( root.at( "ate" ).is_null() );
    EXPECT_TRUE( root.at( "rpe" ).is_null() );
    EXPECT_TRUE( root.at( "timing" ).at( "rtf" ).is_null() );
    EXPECT_EQ( root.at( "trajectory" ).at( "image_frames" ), 10 );
    EXPECT_EQ( root.at( "config_hash" ), "9f2ab41c" );
  }

  TEST( RunSummaryTest, FieldCompletenessWithMetrics )
  {
    RunSummary summary;
    summary.status               = RunStatus::kCompleted;
    summary.sequence             = "MH_01_easy";
    summary.git_commit_short     = "abcdef1";
    summary.config_label         = "default";
    summary.config_hash          = "9f2ab41c";
    summary.trajectory           = { .image_frames    = 3682,
                                     .poses_written   = 3682,
                                     .ok              = 3682,
                                     .rejected        = 0,
                                     .failed          = 0,
                                     .completion_rate = 1.0,
                                     .coverage_rate   = 1.0,
                                     .segments        = 1 };
    summary.robustness.reanchors             = 0;
    summary.robustness.pnp_successes         = 0;
    summary.robustness.pnp_fallbacks         = 0;
    summary.robustness.outliers_culled       = 0;
    summary.robustness.outliers_culled_unique = 0;
    MetricReport ate;
    ate.trans.rmse = 0.15;
    summary.ate    = ate;
    MetricReport rpe;
    rpe.trans.rmse        = 0.02;
    summary.rpe           = rpe;
    summary.timing.wall_s = 12.5;
    summary.timing.rtf    = 14.0;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "schema_version" ), 1 );
    EXPECT_EQ( root.at( "status" ), "completed" );
    EXPECT_NEAR( root.at( "ate" ).at( "trans" ).at( "rmse" ).get<double>(),
                 0.15, 1e-12 );
    EXPECT_NEAR( root.at( "rpe" ).at( "trans" ).at( "rmse" ).get<double>(),
                 0.02, 1e-12 );
    EXPECT_NEAR( root.at( "timing" ).at( "rtf" ).get<double>(), 14.0, 1e-12 );
    EXPECT_TRUE( root.contains( "robustness" ) );
    EXPECT_TRUE( root.contains( "warnings" ) );
    EXPECT_EQ( root.at( "trajectory" ).at( "segments" ), 1 );
    EXPECT_EQ( root.at( "robustness" ).at( "reanchors" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "pnp_successes" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "pnp_fallbacks" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "outliers_culled" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "outliers_culled_unique" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "outlier_reopts" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "drops_skipped" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "deferred_drops" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "deferred_drop_ids" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "evictable_marked" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "tracks_evicted" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "zombie_age_drops" ), 0 );
    EXPECT_EQ( root.at( "robustness" ).at( "zombie_age_drop_ids" ), 0 );
  }

  TEST( RunSummaryTest, SegmentsAndReanchorsSerialize )
  {
    RunSummary summary;
    summary.status               = RunStatus::kCompleted;
    summary.sequence             = "MH_04_difficult";
    summary.trajectory.segments  = 3;
    summary.robustness.reanchors = 2;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "trajectory" ).at( "segments" ), 3 );
    EXPECT_EQ( root.at( "robustness" ).at( "reanchors" ), 2 );
  }

  TEST( RunSummaryTest, PnpCountersSerialize )
  {
    RunSummary summary;
    summary.status                   = RunStatus::kCompleted;
    summary.sequence                 = "MH_01_easy";
    summary.robustness.pnp_successes = 3600;
    summary.robustness.pnp_fallbacks = 12;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "pnp_successes" ), 3600 );
    EXPECT_EQ( root.at( "robustness" ).at( "pnp_fallbacks" ), 12 );
  }

  TEST( RunSummaryTest, OutlierCullCountersSerialize )
  {
    RunSummary summary;
    summary.status                            = RunStatus::kCompleted;
    summary.sequence                          = "MH_05_difficult";
    summary.robustness.outliers_culled        = 42;
    summary.robustness.outliers_culled_unique = 30;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "outliers_culled" ), 42 );
    EXPECT_EQ( root.at( "robustness" ).at( "outliers_culled_unique" ), 30 );
  }

  TEST( RunSummaryTest, OutlierReoptCounterSerialize )
  {
    // Slice ④e: outlier_reopts is a successful reopt *round* count
    // (sum of UpdateDiagnostics.outlier_reopt_rounds), not frames-with-reopt.
    RunSummary summary;
    summary.status                    = RunStatus::kCompleted;
    summary.sequence                  = "MH_05_difficult";
    summary.robustness.outlier_reopts = 7;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "outlier_reopts" ), 7 );
    EXPECT_EQ( root.at( "schema_version" ), 1 );
  }

  TEST( RunSummaryTest, DropsSkippedCounterSerialize )
  {
    RunSummary summary;
    summary.status                   = RunStatus::kCompleted;
    summary.sequence                 = "MH_05_difficult";
    summary.robustness.drops_skipped = 3;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "drops_skipped" ), 3 );
  }

  TEST( RunSummaryTest, DeferredDropCountersSerialize )
  {
    RunSummary summary;
    summary.status                        = RunStatus::kCompleted;
    summary.sequence                      = "MH_05_difficult";
    summary.robustness.deferred_drops     = 2;
    summary.robustness.deferred_drop_ids  = 32;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "deferred_drops" ), 2 );
    EXPECT_EQ( root.at( "robustness" ).at( "deferred_drop_ids" ), 32 );
  }

  TEST( RunSummaryTest, EvictSkipCulledCountersSerialize )
  {
    RunSummary summary;
    summary.status                       = RunStatus::kCompleted;
    summary.sequence                     = "MH_05_difficult";
    summary.robustness.evictable_marked  = 1;
    summary.robustness.tracks_evicted    = 12;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "evictable_marked" ), 1 );
    EXPECT_EQ( root.at( "robustness" ).at( "tracks_evicted" ), 12 );
  }

  TEST( RunSummaryTest, ZombieAgeDropCountersSerialize )
  {
    RunSummary summary;
    summary.status                          = RunStatus::kCompleted;
    summary.sequence                        = "MH_05_difficult";
    summary.robustness.zombie_age_drops     = 2;
    summary.robustness.zombie_age_drop_ids  = 18;

    const json root = json::parse( summary.toJson() );
    EXPECT_EQ( root.at( "robustness" ).at( "zombie_age_drops" ), 2 );
    EXPECT_EQ( root.at( "robustness" ).at( "zombie_age_drop_ids" ), 18 );
  }

  TEST( RunMetaTest, SerializesCodeConfigAndPaths )
  {
    RunMeta meta;
    meta.sequence              = "MH_01_easy";
    meta.sequence_root         = "/data/MH_01_easy";
    meta.created_utc           = "2026-07-31T00:00:00Z";
    meta.code.git_commit       = "0123456789abcdef0123456789abcdef01234567";
    meta.code.git_commit_short = "0123456";
    meta.code.git_dirty        = false;
    meta.code.git_branch       = "main";
    meta.config_json           = R"({"session.max_frames":100})";
    meta.config_label          = "default";
    meta.config_hash           = "deadbeef";
    meta.config_canonical_text = "session.max_frames=100\n";
    meta.bench_root            = "/tmp/bench";
    meta.output_dir            = "/tmp/bench/MH_01_easy/0123456/default_deadbeef";
    meta.layout_template       = "template";
    meta.warnings.emplace_back( "example" );

    const json root = json::parse( meta.toJson() );
    EXPECT_EQ( root.at( "schema_version" ), 1 );
    EXPECT_EQ( root.at( "code" ).at( "git_commit_short" ), "0123456" );
    EXPECT_EQ( root.at( "code" ).at( "git_branch" ), "main" );
    EXPECT_EQ( root.at( "config" ).at( "session.max_frames" ), 100 );
    EXPECT_EQ( root.at( "config_hash" ), "deadbeef" );
    EXPECT_EQ( root.at( "paths" ).at( "layout_template" ), "template" );
    EXPECT_EQ( root.at( "warnings" ).at( 0 ), "example" );
  }

  TEST( RunMetaTest, MissingGitCommitIsNull )
  {
    RunMeta meta;
    meta.sequence   = "MH_01_easy";
    const json root = json::parse( meta.toJson() );
    EXPECT_TRUE( root.at( "code" ).at( "git_commit" ).is_null() );
    EXPECT_EQ( root.at( "code" ).at( "git_commit_short" ), "unknown" );
  }

}  // namespace

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "apps/offline_vo_session.hpp"
#include "phad/bench/code_identity.hpp"
#include "phad/bench/config_snapshot.hpp"
#include "phad/bench/run_paths.hpp"
#include "phad/bench/run_summary.hpp"
#include "phad/eval/ate.hpp"
#include "phad/eval/error_stats.hpp"
#include "phad/eval/rpe.hpp"
#include "phad/eval/tum_io.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

/**
 * @file phad_vo_bench.cpp
 * @brief VO 回归 composition root：session + eval + bench 落盘。
 */

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_vo_bench <sequence-root>\n"
      "                     [--bench-root <dir> | --out <dir>]\n"
      "                     [--sequence-name <name>] [--config-label <name>]\n"
      "                     [--gt-euroc <sequence-root>] [--max-dt-ms <v>]\n"
      "                     [--min-match-rate <v>] [--rpe-delta-s <v>]\n"
      "                     [--errors-csv] [--repo <dir>]\n"
      "                     [--max-frames <n>] [--force]\n"
      "                     [--no-outlier-cull] [--no-outlier-reopt]\n"
      "                     [--allow-culled-rebirth]\n"
      "                     [--no-drop-culled-tracks]\n"
      "                     [--skip-drop-min-culled <n>]\n"
      "                     [--outlier-avg-reproj-px <v>]\n"
      "                     [--max-outlier-reopts <n>]\n"
      "                     [--probe-b <path>]\n"
      "                     [--defer-drop-topk <k>]\n"
      "                     [--evict-skip-culled]\n"
      "                     [--zombie-drop-age <n>]\n";

#ifndef PHAD_SOURCE_DIR
#define PHAD_SOURCE_DIR ""
#endif

  struct Arguments
  {
    std::filesystem::path        sequence_root;
    std::filesystem::path        bench_root;
    std::filesystem::path        out_dir;
    std::filesystem::path        gt_euroc;
    std::filesystem::path        repo = PHAD_SOURCE_DIR;
    std::string                  sequence_name;
    std::string                  config_label   = "default";
    double                       max_dt_ms      = 2.5;
    double                       min_match_rate = 0.5;
    double                       rpe_delta_s    = 1.0;
    std::optional<std::uint64_t> max_frames;
    bool                         write_errors_csv   = false;
    bool                         force              = false;
    bool                         no_outlier_cull       = false;
    bool                         no_outlier_reopt      = false;
    bool                         allow_culled_rebirth  = false;
    bool                         no_drop_culled_tracks = false;
    std::optional<double>        outlier_avg_reproj_px;
    std::optional<int>           max_outlier_reopts;
    std::optional<int>           skip_drop_min_culled;
    std::filesystem::path        probe_b_path;
    std::optional<int>           defer_drop_topk;
    bool                         evict_skip_culled = false;
    std::optional<int>           zombie_drop_age;
  };

  [[nodiscard]] bool parseDouble( std::string_view text, double& value )
  {
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    return parsed.ec == std::errc{} &&
           parsed.ptr == text.data() + text.size() && std::isfinite( value );
  }

  [[nodiscard]] bool parseUint64( std::string_view text, std::uint64_t& value )
  {
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    return parsed.ec == std::errc{} &&
           parsed.ptr == text.data() + text.size();
  }

  [[nodiscard]] bool parseArguments( int argc, char** argv,
                                     Arguments& arguments )
  {
    if ( argc < 2 )
    {
      return false;
    }
    arguments.sequence_root = argv[ 1 ];
    for ( int index = 2; index < argc; ++index )
    {
      const std::string_view flag{ argv[ index ] };
      if ( flag == "--force" )
      {
        arguments.force = true;
        continue;
      }
      if ( flag == "--errors-csv" )
      {
        arguments.write_errors_csv = true;
        continue;
      }
      if ( flag == "--no-outlier-cull" )
      {
        arguments.no_outlier_cull = true;
        continue;
      }
      if ( flag == "--no-outlier-reopt" )
      {
        arguments.no_outlier_reopt = true;
        continue;
      }
      if ( flag == "--allow-culled-rebirth" )
      {
        arguments.allow_culled_rebirth = true;
        continue;
      }
      if ( flag == "--no-drop-culled-tracks" )
      {
        arguments.no_drop_culled_tracks = true;
        continue;
      }
      if ( flag == "--evict-skip-culled" )
      {
        arguments.evict_skip_culled = true;
        continue;
      }
      if ( index + 1 >= argc )
      {
        std::cerr << "missing value for " << flag << '\n';
        return false;
      }
      const std::string_view value{ argv[ index + 1 ] };
      ++index;
      if ( flag == "--bench-root" )
      {
        arguments.bench_root = value;
      }
      else if ( flag == "--out" )
      {
        arguments.out_dir = value;
      }
      else if ( flag == "--sequence-name" )
      {
        arguments.sequence_name = std::string( value );
      }
      else if ( flag == "--config-label" )
      {
        arguments.config_label = std::string( value );
      }
      else if ( flag == "--gt-euroc" )
      {
        arguments.gt_euroc = value;
      }
      else if ( flag == "--repo" )
      {
        arguments.repo = value;
      }
      else if ( flag == "--max-dt-ms" )
      {
        if ( !parseDouble( value, arguments.max_dt_ms ) ||
             arguments.max_dt_ms <= 0.0 )
        {
          std::cerr << "--max-dt-ms expects a positive number\n";
          return false;
        }
      }
      else if ( flag == "--min-match-rate" )
      {
        if ( !parseDouble( value, arguments.min_match_rate ) ||
             arguments.min_match_rate < 0.0 ||
             arguments.min_match_rate > 1.0 )
        {
          std::cerr << "--min-match-rate expects a value in [0, 1]\n";
          return false;
        }
      }
      else if ( flag == "--rpe-delta-s" )
      {
        constexpr double kMinRpeDeltaS =
            static_cast<double>( phad::eval::kDefaultRpeDeltaToleranceNs ) *
            1e-9;
        if ( !parseDouble( value, arguments.rpe_delta_s ) ||
             arguments.rpe_delta_s <= kMinRpeDeltaS )
        {
          std::cerr << "--rpe-delta-s expects a number greater than "
                    << kMinRpeDeltaS << " s\n";
          return false;
        }
      }
      else if ( flag == "--max-frames" )
      {
        std::uint64_t max_frames = 0;
        if ( !parseUint64( value, max_frames ) || max_frames == 0U )
        {
          std::cerr << "--max-frames expects a positive integer\n";
          return false;
        }
        arguments.max_frames = max_frames;
      }
      else if ( flag == "--outlier-avg-reproj-px" )
      {
        double px = 0.0;
        if ( !parseDouble( value, px ) || !( px > 0.0 ) )
        {
          std::cerr
              << "--outlier-avg-reproj-px expects a number greater than 0\n";
          return false;
        }
        arguments.outlier_avg_reproj_px = px;
      }
      else if ( flag == "--max-outlier-reopts" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--max-outlier-reopts expects a non-negative integer\n";
          return false;
        }
        arguments.max_outlier_reopts = static_cast<int>( parsed );
      }
      else if ( flag == "--skip-drop-min-culled" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--skip-drop-min-culled expects a non-negative integer\n";
          return false;
        }
        arguments.skip_drop_min_culled = static_cast<int>( parsed );
      }
      else if ( flag == "--probe-b" )
      {
        arguments.probe_b_path = value;
      }
      else if ( flag == "--defer-drop-topk" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--defer-drop-topk expects a non-negative integer\n";
          return false;
        }
        arguments.defer_drop_topk = static_cast<int>( parsed );
      }
      else if ( flag == "--zombie-drop-age" )
      {
        std::uint64_t parsed = 0;
        if ( !parseUint64( value, parsed ) ||
             parsed > static_cast<std::uint64_t>(
                          std::numeric_limits<int>::max() ) )
        {
          std::cerr
              << "--zombie-drop-age expects a non-negative integer\n";
          return false;
        }
        arguments.zombie_drop_age = static_cast<int>( parsed );
      }
      else
      {
        std::cerr << "unknown flag " << flag << '\n';
        return false;
      }
    }

    if ( arguments.sequence_root.empty() )
    {
      return false;
    }
    if ( arguments.bench_root.empty() )
    {
      if ( const char* env = std::getenv( "PHAD_BENCH_ROOT" );
           env != nullptr && env[ 0 ] != '\0' )
      {
        arguments.bench_root = env;
      }
    }
    if ( arguments.out_dir.empty() && arguments.bench_root.empty() )
    {
      std::cerr << "need --bench-root, PHAD_BENCH_ROOT, or --out\n";
      return false;
    }
    if ( arguments.config_label.empty() )
    {
      std::cerr << "--config-label must be non-empty\n";
      return false;
    }
    if ( arguments.gt_euroc.empty() )
    {
      arguments.gt_euroc = arguments.sequence_root;
    }
    return true;
  }

  [[nodiscard]] std::string resolveSequenceName( const Arguments& arguments )
  {
    if ( !arguments.sequence_name.empty() )
    {
      return arguments.sequence_name;
    }
    std::error_code             ec;
    const std::filesystem::path canonical =
        std::filesystem::weakly_canonical( arguments.sequence_root, ec );
    const std::filesystem::path root =
        ec ? arguments.sequence_root.lexically_normal() : canonical;
    return root.filename().string();
  }

  [[nodiscard]] std::string utcNowIso8601()
  {
    const std::time_t now = std::time( nullptr );
    std::tm           tm{};
#if defined( _WIN32 )
    gmtime_s( &tm, &now );
#else
    gmtime_r( &now, &tm );
#endif
    char buffer[ 32 ];
    if ( std::strftime( buffer, sizeof( buffer ), "%Y-%m-%dT%H:%M:%SZ",
                        &tm ) == 0U )
    {
      return "unknown";
    }
    return buffer;
  }

  [[nodiscard]] bool writeTextFile( const std::filesystem::path& path,
                                    std::string_view             text )
  {
    std::ofstream out( path, std::ios::trunc );
    if ( !out )
    {
      return false;
    }
    out << text;
    out.flush();
    return static_cast<bool>( out );
  }

  [[nodiscard]] phad::bench::ConfigSnapshot flattenConfig(
      const Arguments&                           arguments,
      const phad::apps::OfflineVoSessionOptions& session )
  {
    phad::bench::ConfigSnapshot snap;
    const auto&                 tracker   = session.tracker;
    const auto&                 estimator = session.estimator;

    snap.set( "tracker.max_tracks",
              static_cast<std::int64_t>( tracker.max_tracks ) );
    snap.set( "tracker.quality_level", tracker.quality_level );
    snap.set( "tracker.min_distance_px", tracker.min_distance_px );
    snap.set( "tracker.mask_radius_px",
              static_cast<std::int64_t>( tracker.mask_radius_px ) );
    snap.set( "tracker.lk_window_px",
              static_cast<std::int64_t>( tracker.lk_window_px ) );
    snap.set( "tracker.lk_pyramid_levels",
              static_cast<std::int64_t>( tracker.lk_pyramid_levels ) );
    snap.set( "tracker.forward_backward_px", tracker.forward_backward_px );
    snap.set( "tracker.max_epipolar_px", tracker.max_epipolar_px );
    snap.set( "tracker.min_disparity_px", tracker.min_disparity_px );
    snap.set( "tracker.min_depth_m", tracker.min_depth_m );
    snap.set( "tracker.max_depth_m", tracker.max_depth_m );
    snap.set( "tracker.stereo_sad_half_win_px",
              static_cast<std::int64_t>( tracker.stereo_sad_half_win_px ) );
    snap.set( "tracker.stereo_row_tol_px",
              static_cast<std::int64_t>( tracker.stereo_row_tol_px ) );
    snap.set( "tracker.stereo_bidir_px", tracker.stereo_bidir_px );
    snap.set( "tracker.stereo_uniq_ratio", tracker.stereo_uniq_ratio );
    snap.set( "tracker.stereo_check_bidir", tracker.stereo_check_bidir );

    snap.set( "estimator.window_size",
              static_cast<std::int64_t>( estimator.window_size ) );
    snap.set( "estimator.min_landmark_observations",
              static_cast<std::int64_t>( estimator.min_landmark_observations ) );
    snap.set( "estimator.min_shared_landmarks",
              static_cast<std::int64_t>( estimator.min_shared_landmarks ) );
    snap.set( "estimator.stereo_sigma_px", estimator.stereo_sigma_px );
    snap.set( "estimator.huber_k_px", estimator.huber_k_px );
    snap.set( "estimator.prior_rotation_sigma_rad",
              estimator.prior_rotation_sigma_rad );
    snap.set( "estimator.prior_translation_sigma_m",
              estimator.prior_translation_sigma_m );
    snap.set( "estimator.use_constant_velocity_init",
              estimator.use_constant_velocity_init );
    snap.set( "estimator.min_seed_observations",
              static_cast<std::int64_t>( estimator.min_seed_observations ) );
    snap.set( "estimator.min_track_observations_for_seed",
              static_cast<std::int64_t>(
                  estimator.min_track_observations_for_seed ) );
    snap.set( "estimator.enable_reanchor", estimator.enable_reanchor );
    snap.set( "estimator.enable_pnp_init", estimator.enable_pnp_init );
    snap.set( "estimator.pnp_reproj_px", estimator.pnp_reproj_px );
    snap.set( "estimator.pnp_confidence", estimator.pnp_confidence );
    snap.set( "estimator.min_pnp_inliers",
              static_cast<std::int64_t>( estimator.min_pnp_inliers ) );
    snap.set( "estimator.enable_outlier_cull", estimator.enable_outlier_cull );
    snap.set( "estimator.outlier_avg_reproj_px",
              estimator.outlier_avg_reproj_px );
    snap.set( "estimator.enable_outlier_reopt",
              estimator.enable_outlier_reopt );
    snap.set( "estimator.max_outlier_reopts",
              static_cast<std::int64_t>( estimator.max_outlier_reopts ) );
    snap.set( "estimator.block_culled_rebirth",
              estimator.block_culled_rebirth );

    snap.set( "session.dataset_format", std::string( "euroc" ) );
    snap.set( "session.drop_culled_tracks", session.drop_culled_tracks );
    snap.set( "session.skip_drop_min_culled",
              static_cast<std::int64_t>( session.skip_drop_min_culled ) );
    snap.set( "session.zombie_drop_age",
              static_cast<std::int64_t>( session.zombie_drop_age ) );
    if ( session.max_frames.has_value() )
    {
      snap.set( "session.max_frames",
                static_cast<std::int64_t>( *session.max_frames ) );
    }

    snap.set( "eval.max_dt_ms", arguments.max_dt_ms );
    snap.set( "eval.min_match_rate", arguments.min_match_rate );
    snap.set( "eval.rpe_delta_s", arguments.rpe_delta_s );
    return snap;
  }

  [[nodiscard]] phad::bench::StatBlock toStatBlock(
      const phad::eval::ErrorStats& stats )
  {
    return phad::bench::StatBlock{ .rmse   = stats.rmse,
                                   .mean   = stats.mean,
                                   .median = stats.median,
                                   .stddev = stats.stddev,
                                   .max    = stats.max };
  }

  [[nodiscard]] phad::bench::StageTiming toBenchTiming(
      const phad::apps::StageTiming& timing )
  {
    return phad::bench::StageTiming{ .mean = timing.mean,
                                     .p95  = timing.p95,
                                     .max  = timing.max };
  }

  [[nodiscard]] bool writeErrorsCsv( const std::filesystem::path& path,
                                     const phad::eval::AteReport& report )
  {
    std::ofstream stream( path, std::ios::trunc );
    if ( !stream )
    {
      return false;
    }
    stream << std::setprecision( std::numeric_limits<double>::max_digits10 );
    stream << "timestamp_ns,dt_ns,err_trans_m,err_rot_deg,est_x,est_y,est_z,"
              "gt_x,gt_y,gt_z\n";
    for ( const phad::eval::PoseErrorSample& sample : report.samples )
    {
      stream << sample.timestamp.nanoseconds() << ',' << sample.dt_ns << ','
             << sample.trans_m << ',' << sample.rot_deg << ','
             << sample.aligned_est_position.x() << ','
             << sample.aligned_est_position.y() << ','
             << sample.aligned_est_position.z() << ','
             << sample.gt_position.x() << ',' << sample.gt_position.y() << ','
             << sample.gt_position.z() << '\n';
    }
    stream.flush();
    return static_cast<bool>( stream );
  }

  [[nodiscard]] double coverageRate(
      const phad::apps::OfflineVoSessionResult& session )
  {
    if ( !session.trajectory.has_value() || session.trajectory->size() == 0U )
    {
      return 0.0;
    }
    const auto&  poses = session.trajectory->poses();
    const double pose_span_s =
        static_cast<double>( poses.back().timestamp.nanoseconds() -
                             poses.front().timestamp.nanoseconds() ) *
        1e-9;
    const double image_span_s =
        static_cast<double>( session.last_image_ts.nanoseconds() -
                             session.first_image_ts.nanoseconds() ) *
        1e-9;
    if ( image_span_s <= 0.0 )
    {
      return 0.0;
    }
    return pose_span_s / image_span_s;
  }

  [[nodiscard]] double imageSpanSeconds(
      const phad::apps::OfflineVoSessionResult& session )
  {
    if ( session.counts.image_frames == 0U )
    {
      return 0.0;
    }
    return static_cast<double>( session.last_image_ts.nanoseconds() -
                                session.first_image_ts.nanoseconds() ) *
           1e-9;
  }

  [[nodiscard]] int run( const Arguments& arguments )
  {
    const std::string sequence = resolveSequenceName( arguments );
    if ( sequence.empty() || sequence == "mav0" )
    {
      std::cerr << "sequence name is empty or 'mav0'; pass --sequence-name\n";
      return 2;
    }

    phad::apps::OfflineVoSessionOptions session_options;
    session_options.sequence_root = arguments.sequence_root;
    session_options.max_frames    = arguments.max_frames;
    if ( arguments.no_outlier_cull )
    {
      session_options.estimator.enable_outlier_cull = false;
    }
    if ( arguments.no_outlier_reopt )
    {
      session_options.estimator.enable_outlier_reopt = false;
    }
    // Default leaves estimator.block_culled_rebirth=true. Flag only for
    // A/B that re-enables same-id backproject; session still dropTracks
    // unless --no-drop-culled-tracks.
    if ( arguments.allow_culled_rebirth )
    {
      session_options.estimator.block_culled_rebirth = false;
    }
    if ( arguments.no_drop_culled_tracks )
    {
      session_options.drop_culled_tracks = false;
    }
    if ( arguments.skip_drop_min_culled.has_value() )
    {
      session_options.skip_drop_min_culled = *arguments.skip_drop_min_culled;
    }
    if ( arguments.outlier_avg_reproj_px.has_value() )
    {
      session_options.estimator.outlier_avg_reproj_px =
          *arguments.outlier_avg_reproj_px;
    }
    if ( arguments.max_outlier_reopts.has_value() )
    {
      session_options.estimator.max_outlier_reopts =
          *arguments.max_outlier_reopts;
    }
    // Probe B path is CLI-only: not in flattenConfig / config_hash.
    session_options.probe_b_path = arguments.probe_b_path;
    // defer-drop-topk is CLI-only probe: not in flattenConfig / config_hash.
    if ( arguments.defer_drop_topk.has_value() )
    {
      session_options.defer_drop_topk = *arguments.defer_drop_topk;
    }
    // evict-skip-culled is CLI-only probe: not in flattenConfig / config_hash.
    session_options.evict_skip_culled = arguments.evict_skip_culled;
    // zombie-drop-age defaults to 5 and enters flattenConfig / config_hash.
    if ( arguments.zombie_drop_age.has_value() )
    {
      session_options.zombie_drop_age = *arguments.zombie_drop_age;
    }

    phad::bench::ConfigSnapshot config =
        flattenConfig( arguments, session_options );
    const std::string config_hash = config.hash8();
    const std::string config_text = config.canonicalText();
    const std::string config_json = config.toJson();

    std::vector<std::string>        warnings;
    const phad::bench::CodeIdentity code =
        phad::bench::queryGitIdentity( arguments.repo, warnings );
    if ( code.git_dirty )
    {
      warnings.emplace_back(
          "git working tree is dirty; not suitable as a formal baseline" );
    }

    const bool            use_override = !arguments.out_dir.empty();
    std::filesystem::path output_dir =
        use_override ? arguments.out_dir
                     : phad::bench::composeRunDir( arguments.bench_root,
                                                   sequence, code,
                                                   arguments.config_label,
                                                   config_hash );

    const phad::bench::OverwriteDecision overwrite =
        phad::bench::decideOverwrite( output_dir, code.git_dirty,
                                      arguments.force );
    if ( overwrite == phad::bench::OverwriteDecision::kRefuse )
    {
      std::cerr << "refusing to overwrite existing run at " << output_dir
                << " (pass --force)\n";
      return 3;
    }
    if ( overwrite == phad::bench::OverwriteDecision::kOverwriteWithWarning )
    {
      warnings.emplace_back(
          arguments.force
              ? "overwriting existing summary.json due to --force"
              : "overwriting existing summary.json on dirty tree" );
    }

    std::error_code ec;
    std::filesystem::create_directories( output_dir, ec );
    if ( ec )
    {
      std::cerr << "failed to create output dir: " << ec.message() << '\n';
      return 1;
    }

    phad::bench::RunMeta meta;
    meta.sequence              = sequence;
    meta.sequence_root         = arguments.sequence_root.string();
    meta.created_utc           = utcNowIso8601();
    meta.code                  = code;
    meta.config_json           = config_json;
    meta.config_label          = arguments.config_label;
    meta.config_hash           = config_hash;
    meta.config_canonical_text = config_text;
    meta.bench_root            = arguments.bench_root.string();
    meta.output_dir            = output_dir.string();
    meta.layout_template       = use_override ? "override" : "template";
    meta.warnings              = warnings;
    if ( !writeTextFile( output_dir / "meta.json", meta.toJson() ) )
    {
      std::cerr << "failed to write meta.json\n";
      return 1;
    }

    phad::apps::OfflineVoSessionResult session =
        phad::apps::runOfflineVoSession( session_options );

    phad::bench::RunSummary summary;
    summary.sequence         = sequence;
    summary.git_commit_short = code.git_commit_short;
    summary.git_dirty        = code.git_dirty;
    summary.config_label     = arguments.config_label;
    summary.config_hash      = config_hash;
    summary.warnings         = warnings;
    summary.warnings.insert( summary.warnings.end(), session.warnings.begin(),
                             session.warnings.end() );
    summary.sync = phad::bench::SyncSummary{
        session.sync.pushed_left,
        session.sync.pushed_right,
        session.sync.emitted_stereo,
        session.sync.dropped_left,
        session.sync.dropped_right,
        session.sync.dropped_left_overflow,
        session.sync.dropped_right_overflow,
        session.sync.max_left_queue,
        session.sync.max_right_queue,
    };
    summary.trajectory.image_frames = session.counts.image_frames;
    summary.trajectory.ok           = session.counts.ok;
    summary.trajectory.rejected     = session.counts.rejected;
    summary.trajectory.failed       = session.counts.failed;
    summary.trajectory.poses_written =
        session.trajectory.has_value() ? session.trajectory->size() : 0U;
    summary.trajectory.completion_rate =
        session.counts.image_frames == 0U
            ? 0.0
            : static_cast<double>( session.counts.ok ) /
                  static_cast<double>( session.counts.image_frames );
    summary.trajectory.coverage_rate    = coverageRate( session );
    summary.trajectory.segments               = session.counts.segments;
    summary.trajectory.total_keyframes       = session.counts.total_keyframes;
    summary.trajectory.total_track_only_frames =
        session.counts.total_track_only_frames;
    summary.robustness.rejected         = session.counts.rejected;
    summary.robustness.failed           = session.counts.failed;
    summary.robustness.low_connectivity = session.counts.low_connectivity;
    summary.robustness.reanchors        = session.counts.reanchors;
    summary.robustness.pnp_successes          = session.counts.pnp_successes;
    summary.robustness.pnp_fallbacks          = session.counts.pnp_fallbacks;
    summary.robustness.outliers_culled        = session.counts.outliers_culled;
    summary.robustness.outliers_culled_unique =
        session.counts.outliers_culled_unique;
    summary.robustness.outlier_reopts = session.counts.outlier_reopts;
    summary.robustness.drops_skipped     = session.counts.drops_skipped;
    summary.robustness.deferred_drops    = session.counts.deferred_drops;
    summary.robustness.deferred_drop_ids = session.counts.deferred_drop_ids;
    summary.robustness.evictable_marked   = session.counts.evictable_marked;
    summary.robustness.tracks_evicted     = session.counts.tracks_evicted;
    summary.robustness.zombie_age_drops   = session.counts.zombie_age_drops;
    summary.robustness.zombie_age_drop_ids =
        session.counts.zombie_age_drop_ids;
    for ( const auto& row : session.diag )
    {
      summary.robustness.cheirality += row.num_cheirality;
    }
    summary.timing.wall_s     = session.wall_s;
    summary.timing.rectify    = toBenchTiming( session.timings.rectify );
    summary.timing.frontend   = toBenchTiming( session.timings.frontend );
    summary.timing.estimator  = toBenchTiming( session.timings.estimator );
    summary.timing.total      = toBenchTiming( session.timings.total );
    const double image_span_s = imageSpanSeconds( session );
    if ( session.wall_s > 0.0 && image_span_s > 0.0 )
    {
      summary.timing.rtf = image_span_s / session.wall_s;
    }

    int exit_code = 0;
    if ( session.error.has_value() || !session.trajectory.has_value() )
    {
      summary.status = phad::bench::RunStatus::kFailed;
      if ( session.error.has_value() )
      {
        summary.warnings.push_back( session.error->detail );
        std::cerr << session.error->detail << '\n';
      }
      exit_code = 1;
    }
    else
    {
      if ( const auto write_error = phad::eval::writeTum(
               output_dir / "est.tum", *session.trajectory ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( write_error->describe() );
        std::cerr << "write tum failed: " << write_error->describe() << '\n';
        exit_code = 1;
      }
      else if ( const auto diag_error = phad::apps::writeDiagCsv(
                    output_dir / "diag.csv", session.diag ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( diag_error->detail );
        std::cerr << diag_error->detail << '\n';
        exit_code = 1;
      }
      else
      {
        auto gt = phad::io::dataset::euroc::openGroundtruth(
            arguments.gt_euroc );
        if ( !gt )
        {
          summary.status = phad::bench::RunStatus::kEvalFailed;
          summary.warnings.push_back( gt.error().describe() );
          std::cerr << gt.error().describe() << '\n';
          exit_code = 1;
        }
        else
        {
          phad::eval::AssociationOptions association_options;
          association_options.max_dt_ns = static_cast<std::int64_t>(
              std::llround( arguments.max_dt_ms * 1'000'000.0 ) );
          association_options.min_match_rate = arguments.min_match_rate;

          phad::eval::AteOptions ate_options;
          ate_options.association = association_options;
          auto ate                = phad::eval::computeAte( *session.trajectory, gt.value(),
                                                            ate_options );

          phad::eval::RpeOptions rpe_options;
          rpe_options.association = association_options;
          rpe_options.delta_ns    = static_cast<std::int64_t>(
              std::llround( arguments.rpe_delta_s * 1'000'000'000.0 ) );
          auto rpe = phad::eval::computeRpe( *session.trajectory, gt.value(),
                                             rpe_options );

          if ( !ate || !rpe )
          {
            summary.status = phad::bench::RunStatus::kEvalFailed;
            if ( !ate )
            {
              summary.warnings.push_back( ate.error().describe() );
              std::cerr << ate.error().describe() << '\n';
            }
            if ( !rpe )
            {
              summary.warnings.push_back( rpe.error().describe() );
              std::cerr << rpe.error().describe() << '\n';
            }
            exit_code = 1;
          }
          else
          {
            summary.ate = phad::bench::MetricReport{
                .trans   = toStatBlock( ate.value().trans_m ),
                .rot_deg = toStatBlock( ate.value().rot_deg ),
            };
            summary.rpe = phad::bench::MetricReport{
                .trans   = toStatBlock( rpe.value().trans_m ),
                .rot_deg = toStatBlock( rpe.value().rot_deg ),
            };
            if ( arguments.write_errors_csv &&
                 !writeErrorsCsv( output_dir / "errors.csv", ate.value() ) )
            {
              summary.warnings.emplace_back( "failed to write errors.csv" );
            }

            if ( session.counts.failed > 0U )
            {
              summary.status =
                  phad::bench::RunStatus::kCompletedWithFailures;
            }
            else if ( !summary.warnings.empty() )
            {
              summary.status =
                  phad::bench::RunStatus::kCompletedWithWarnings;
            }
            else
            {
              summary.status = phad::bench::RunStatus::kCompleted;
            }
            exit_code = 0;
          }
        }
      }
    }

    // kf.tum: independent of the est.tum/diag/eval chain above.
    if ( exit_code == 0 && session.kf_trajectory.has_value() )
    {
      if ( const auto kf_error = phad::eval::writeTum(
               output_dir / "kf.tum", *session.kf_trajectory ) )
      {
        summary.status = phad::bench::RunStatus::kFailed;
        summary.warnings.push_back( kf_error->describe() );
        std::cerr << "write kf tum failed: " << kf_error->describe() << '\n';
        exit_code = 1;
      }
    }

    // failed / eval_failed still write summary.json for the regression table.
    if ( !writeTextFile( output_dir / "summary.json", summary.toJson() ) )
    {
      std::cerr << "failed to write summary.json\n";
      return 1;
    }

    std::cout << "status=" << phad::bench::runStatusToString( summary.status )
              << '\n'
              << "sequence=" << sequence << '\n'
              << "output_dir=" << output_dir.string() << '\n'
              << "config_hash=" << config_hash << '\n'
              << "completion_rate=" << summary.trajectory.completion_rate
              << '\n'
              << "coverage_rate=" << summary.trajectory.coverage_rate << '\n';
    if ( summary.ate.has_value() )
    {
      std::cout << "ate_trans_rmse=" << summary.ate->trans.rmse << '\n';
    }
    if ( summary.rpe.has_value() )
    {
      std::cout << "rpe_trans_rmse=" << summary.rpe->trans.rmse << '\n';
    }
    if ( summary.timing.rtf.has_value() )
    {
      std::cout << "rtf=" << *summary.timing.rtf << '\n';
    }
    std::cout << "wall_s=" << summary.timing.wall_s << '\n';
    return exit_code;
  }

}  // namespace

int main( int argc, char** argv )
{
  Arguments arguments;
  if ( !parseArguments( argc, argv, arguments ) )
  {
    std::cerr << kUsage;
    return 2;
  }

  try
  {
    return run( arguments );
  }
  catch ( const std::exception& exception )
  {
    std::cerr << "phad_vo_bench error: " << exception.what() << '\n';
    return 1;
  }
}

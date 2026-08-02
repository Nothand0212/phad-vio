#include "apps/offline_vo_session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "apps/probe_b_writer.hpp"
#include "apps/stereo_pair_stream.hpp"
#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file offline_vo_session.cpp
 * @brief 离线 VO session：只跑 pipeline，错误经 result 返回。
 */

namespace phad::apps
{
  namespace
  {

    // Must match phad::estimator::StereoVoEstimator::update reject messages.
    constexpr std::string_view kSeedRejectedNewSegment =
        "insufficient observations to seed new segment";
    constexpr std::string_view kSeedRejectedFirstSegment =
        "insufficient observations to seed first segment";

    [[nodiscard]] double percentile( std::vector<double> values, double q )
    {
      if ( values.empty() )
      {
        return 0.0;
      }
      std::sort( values.begin(), values.end() );
      const double position = q * static_cast<double>( values.size() - 1U );
      const auto   lower    = static_cast<std::size_t>( std::floor( position ) );
      const auto   upper    = static_cast<std::size_t>( std::ceil( position ) );
      if ( lower == upper )
      {
        return values[ lower ];
      }
      const double weight = position - static_cast<double>( lower );
      return values[ lower ] * ( 1.0 - weight ) + values[ upper ] * weight;
    }

    [[nodiscard]] StageTiming summarizeStage(
        std::vector<double> samples )
    {
      StageTiming timing;
      if ( samples.empty() )
      {
        return timing;
      }
      double sum = 0.0;
      double max = samples.front();
      for ( const double sample : samples )
      {
        sum += sample;
        max = std::max( max, sample );
      }
      timing.mean = sum / static_cast<double>( samples.size() );
      timing.p95  = percentile( samples, 0.95 );
      timing.max  = max;
      return timing;
    }

  }  // namespace

  OfflineVoSessionResult runOfflineVoSession(
      const OfflineVoSessionOptions& options )
  {
    OfflineVoSessionResult result;

    auto opened = io::dataset::euroc::open( options.sequence_root );
    if ( !opened )
    {
      result.error = SessionError{ opened.error().describe() };
      return result;
    }

    auto rectifier =
        camera::StereoRectifier::create( opened.value().calibration() );
    if ( !rectifier )
    {
      result.error =
          SessionError{ "stereo rectifier: " + rectifier.error().detail };
      return result;
    }

    const auto&             rectified_cal = rectifier.value().calibration();
    frontend::StereoTracker tracker( rectified_cal, options.tracker );

    // Probe B: CLI path only. Enable estimator side-channel when writing.
    std::unique_ptr<ProbeBWriter>              probe_b_writer;
    std::unordered_set<common::LandmarkId>     lifetime_culled;
    estimator::EstimatorOptions                estimator_options = options.estimator;
    if ( !options.probe_b_path.empty() )
    {
      try
      {
        probe_b_writer =
            std::make_unique<ProbeBWriter>( options.probe_b_path );
      }
      catch ( const std::runtime_error& exception )
      {
        result.error = SessionError{ exception.what() };
        return result;
      }
      estimator_options.enable_probe_b = true;
    }
    estimator::StereoVoEstimator estimator( rectified_cal, estimator_options );

    std::vector<common::TimedPose> poses;
    std::vector<double>            rms_after;
    std::vector<double>            rectify_s;
    std::vector<double>            frontend_s;
    std::vector<double>            estimator_s;
    std::vector<double>            total_s;

    std::optional<std::uint32_t> last_segment_id;
    bool                         any_segment_established = false;
    bool                         warned_reanchor         = false;
    std::vector<std::string>     segment_warnings;

    const auto                       wall_begin = std::chrono::steady_clock::now();
    io::dataset::DatasetReplaySource source{ opened.value() };
    StereoPairStream                 stream{ source };

    // Shared by the success path and every mid-loop error return so the
    // "vo segments summary: ..." line and segment_warnings are never lost.
    const auto finalizeSegmentsAndWarnings =
        [ &result, &stream, &segment_warnings, &any_segment_established ]() {
          result.counts.segments =
              any_segment_established ? result.counts.reanchors + 1U : 0U;
          result.warnings = stream.warnings();
          result.warnings.insert( result.warnings.end(),
                                  segment_warnings.begin(),
                                  segment_warnings.end() );
          // Only worth a warning when something actually happened to the
          // segment lifecycle or PnP fell back. Cull totals stay in
          // FrameCounts / summary.json robustness only — healthy runs with
          // normal cull must remain eligible for kCompleted.
          if ( result.counts.reanchors > 0U ||
               result.counts.seed_rejected > 0U )
          {
            result.warnings.push_back(
                "vo segments summary: segments=" +
                std::to_string( result.counts.segments ) +
                " reanchors=" + std::to_string( result.counts.reanchors ) +
                " seed_rejected=" +
                std::to_string( result.counts.seed_rejected ) );
          }
          if ( result.counts.pnp_fallbacks > 0U )
          {
            result.warnings.push_back(
                "vo pnp summary: pnp_successes=" +
                std::to_string( result.counts.pnp_successes ) +
                " pnp_fallbacks=" +
                std::to_string( result.counts.pnp_fallbacks ) );
          }
        };

    while ( true )
    {
      if ( options.max_frames.has_value() &&
           result.counts.image_frames >= *options.max_frames )
      {
        break;
      }

      auto loaded = stream.next();
      if ( std::holds_alternative<io::EndOfStream>( loaded ) )
      {
        break;
      }
      if ( const auto* error = std::get_if<StreamError>( &loaded ) )
      {
        result.error = SessionError{ error->detail };
        result.sync  = stream.diagnostics();
        finalizeSegmentsAndWarnings();
        return result;
      }

      const auto  frame_begin = std::chrono::steady_clock::now();
      const auto& raw         = std::get<sensor::StereoFrame>( loaded );

      const auto rectify_begin = std::chrono::steady_clock::now();
      auto       rectified     = rectifier.value().rectify( raw );
      const auto rectify_end   = std::chrono::steady_clock::now();
      if ( !rectified )
      {
        result.error =
            SessionError{ "rectify failed: " + rectified.error().detail };
        result.sync = stream.diagnostics();
        finalizeSegmentsAndWarnings();
        return result;
      }

      const auto                  frontend_begin = std::chrono::steady_clock::now();
      const frontend::FrameTracks tracks =
          tracker.process( rectified.value() );
      const auto frontend_end = std::chrono::steady_clock::now();

      // Zombie = frontend obs whose id was permanently culled in a prior frame.
      std::uint32_t zombie_track_n = 0;
      if ( probe_b_writer )
      {
        for ( const auto& obs : tracks.observations )
        {
          if ( lifetime_culled.count( obs.id ) != 0U )
          {
            ++zombie_track_n;
          }
        }
      }

      const auto                           estimator_begin = std::chrono::steady_clock::now();
      const estimator::KeyframeMeasurement measurement =
          toKeyframeMeasurement( tracks );
      const estimator::VioUpdateResult update =
          estimator.update( measurement );
      const auto estimator_end = std::chrono::steady_clock::now();
      const auto frame_end     = estimator_end;

      // Composition-root feedback: drop frontend tracks for ids the
      // estimator permanently removed this frame. Default on (④c); two-level
      // gate (④f): drop_culled_tracks then skip when outliers_culled >= N.
      // Does not enter warnings.
      bool drops_skipped_this_frame = false;
      if ( options.drop_culled_tracks &&
           !update.diagnostics.culled_landmark_ids.empty() )
      {
        const bool skip =
            options.skip_drop_min_culled > 0 &&
            update.diagnostics.outliers_culled >=
                static_cast<std::uint32_t>( options.skip_drop_min_culled );
        if ( skip )
        {
          ++result.counts.drops_skipped;
          drops_skipped_this_frame = true;
        }
        else
        {
          tracker.dropTracks( update.diagnostics.culled_landmark_ids );
        }
      }

      // Probe B jsonl: i matches diag row order (0-based before increment).
      if ( probe_b_writer )
      {
        const std::uint64_t frame_i = result.counts.image_frames;
        const auto&         d       = update.diagnostics;
        for ( const common::LandmarkId id : d.culled_landmark_ids )
        {
          lifetime_culled.insert( id );
        }

        const bool heavy =
            ( frame_i >= 420U && frame_i <= 450U ) ||
            !d.culled_landmark_ids.empty() || d.lm_iterations >= 100U ||
            d.max_window_pose_shift_m > 0.5;

        ProbeBFrame probe_frame;
        probe_frame.i     = frame_i;
        probe_frame.ts_ns = tracks.timestamp.nanoseconds();
        if ( heavy )
        {
          probe_frame.culled_ids = std::vector<std::uint64_t>(
              d.culled_landmark_ids.begin(), d.culled_landmark_ids.end() );
          probe_frame.zombie_track_n    = zombie_track_n;
          probe_frame.rejected_block_n  = d.probe_rejected_block_n;
          probe_frame.new_lm            = d.probe_new_lm_n;
          probe_frame.shared            = d.num_shared;
          probe_frame.num_obs           = d.num_observations;
          probe_frame.lm_iterations     = d.lm_iterations;
          probe_frame.shift_m           = d.max_window_pose_shift_m;
          std::vector<ProbeBShiftTop> shift_top;
          shift_top.reserve( d.probe_shift_top.size() );
          for ( const auto& entry : d.probe_shift_top )
          {
            shift_top.push_back(
                ProbeBShiftTop{ .key = entry.first, .dt_m = entry.second } );
          }
          probe_frame.shift_top = std::move( shift_top );
          if ( d.probe_detail_valid )
          {
            probe_frame.res_mean_px = d.probe_res_mean_px;
            probe_frame.res_max_px  = d.probe_res_max_px;
            probe_frame.res_max_id  = static_cast<std::uint64_t>(
                d.probe_res_max_id );
          }
          probe_frame.drops_skipped_this_frame = drops_skipped_this_frame;
        }
        else
        {
          // Light frame (design §4): keep index continuity without heavy fields.
          probe_frame.shared  = d.num_shared;
          probe_frame.num_obs = d.num_observations;
        }

        try
        {
          probe_b_writer->write( probe_frame );
        }
        catch ( const std::runtime_error& exception )
        {
          result.error = SessionError{ exception.what() };
          result.sync  = stream.diagnostics();
          finalizeSegmentsAndWarnings();
          return result;
        }
      }

      if ( result.counts.image_frames == 0U )
      {
        result.first_image_ts = tracks.timestamp;
      }
      result.last_image_ts = tracks.timestamp;
      ++result.counts.image_frames;

      switch ( update.status )
      {
        case estimator::UpdateStatus::kOk:
          ++result.counts.ok;
          break;
        case estimator::UpdateStatus::kRejected:
          ++result.counts.rejected;
          if ( update.message == kSeedRejectedNewSegment ||
               update.message == kSeedRejectedFirstSegment )
          {
            ++result.counts.seed_rejected;
          }
          break;
        case estimator::UpdateStatus::kFailed:
          ++result.counts.failed;
          break;
      }
      if ( update.diagnostics.low_connectivity )
      {
        ++result.counts.low_connectivity;
      }

      const auto& d = update.diagnostics;
      result.counts.outliers_culled += d.outliers_culled;
      result.counts.outliers_culled_unique += d.outliers_culled_unique;
      // Slice ④e: accumulate successful reopt *rounds*, not frames-with-reopt.
      result.counts.outlier_reopts += d.outlier_reopt_rounds;
      if ( update.status == estimator::UpdateStatus::kOk )
      {
        rms_after.push_back( d.reproj_rms_after_px );
        common::TimedPose pose;
        pose.timestamp = update.estimate->timestamp;
        pose.T_W_B     = update.estimate->T_W_B;
        poses.push_back( pose );

        const std::uint32_t segment_id  = d.segment_id;
        const bool          is_reanchor = last_segment_id.has_value() &&
                                 segment_id > *last_segment_id;
        if ( is_reanchor )
        {
          ++result.counts.reanchors;
          if ( !warned_reanchor )
          {
            segment_warnings.push_back(
                "vo re-anchored: ts=" +
                std::to_string( tracks.timestamp.nanoseconds() ) +
                " segment_id=" + std::to_string( segment_id ) );
            warned_reanchor = true;
          }
        }

        // Normal-path PnP counters: seed / re-anchor frames are excluded
        // (num_shared==0 or segment jump). enable_pnp_init=false is not a
        // fallback — that path never attempted PnP.
        if ( d.pnp_success )
        {
          ++result.counts.pnp_successes;
        }
        if ( options.estimator.enable_pnp_init && d.num_shared > 0U &&
             !d.pnp_success && !is_reanchor )
        {
          ++result.counts.pnp_fallbacks;
        }

        any_segment_established = true;
        last_segment_id         = segment_id;
      }

      result.diag.push_back( VoDiagRow{
          .timestamp_ns             = tracks.timestamp.nanoseconds(),
          .status                   = updateStatusName( update.status ),
          .num_observations         = d.num_observations,
          .num_landmarks            = d.num_landmarks,
          .num_shared               = d.num_shared,
          .low_connectivity         = d.low_connectivity,
          .window_size              = d.window_size,
          .prior_key                = d.prior_key,
          .reproj_rms_before_px     = d.reproj_rms_before_px,
          .reproj_rms_after_px      = d.reproj_rms_after_px,
          .num_cheirality           = d.num_cheirality,
          .lm_iterations            = d.lm_iterations,
          .max_window_pose_shift_m  = d.max_window_pose_shift_m,
          .segment_id               = d.segment_id,
          .pnp_success              = d.pnp_success,
          .pnp_inliers              = d.pnp_inliers,
          .outliers_culled          = d.outliers_culled,
          .reproj_rms_after_cull_px = d.reproj_rms_after_cull_px,
      } );

      if ( options.collect_timing )
      {
        const auto seconds = []( auto begin, auto end ) {
          return std::chrono::duration<double>( end - begin ).count();
        };
        rectify_s.push_back( seconds( rectify_begin, rectify_end ) );
        frontend_s.push_back( seconds( frontend_begin, frontend_end ) );
        estimator_s.push_back( seconds( estimator_begin, estimator_end ) );
        total_s.push_back( seconds( frame_begin, frame_end ) );
      }
    }

    const auto wall_end = std::chrono::steady_clock::now();
    result.wall_s =
        std::chrono::duration<double>( wall_end - wall_begin ).count();

    if ( options.collect_timing )
    {
      result.timings.rectify   = summarizeStage( std::move( rectify_s ) );
      result.timings.frontend  = summarizeStage( std::move( frontend_s ) );
      result.timings.estimator = summarizeStage( std::move( estimator_s ) );
      result.timings.total     = summarizeStage( std::move( total_s ) );
    }

    result.reproj.median_px = percentile( rms_after, 0.5 );
    result.reproj.p95_px    = percentile( rms_after, 0.95 );
    result.sync             = stream.diagnostics();
    finalizeSegmentsAndWarnings();

    if ( poses.empty() )
    {
      result.error = SessionError{ "no accepted poses to write" };
      return result;
    }

    auto trajectory = common::Trajectory::create( std::move( poses ) );
    if ( !trajectory )
    {
      result.error = SessionError{ "trajectory create failed: " +
                                   trajectory.error().detail };
      return result;
    }
    result.trajectory = std::move( trajectory.value() );
    return result;
  }

  std::optional<SessionError> writeDiagCsv(
      const std::filesystem::path& path, const std::vector<VoDiagRow>& rows )
  {
    std::ofstream out( path );
    if ( !out )
    {
      return SessionError{ "failed to open diag csv: " + path.string() };
    }

    out << "timestamp_ns,status,num_obs,num_landmarks,num_shared,"
           "low_connectivity,window_size,prior_key,"
           "reproj_rms_before_px,reproj_rms_after_px,num_cheirality,"
           "lm_iterations,max_window_pose_shift_m,segment_id,"
           "pnp_success,pnp_inliers,outliers_culled,"
           "reproj_rms_after_cull_px\n";

    for ( const VoDiagRow& row : rows )
    {
      // Keep manipulator placement identical to the pre-migration probe so
      // MH_01 diag.csv stays byte-identical under std::fixed / setprecision.
      out << row.timestamp_ns << ',' << row.status << ','
          << row.num_observations << ',' << row.num_landmarks << ','
          << row.num_shared << ',' << ( row.low_connectivity ? 1 : 0 ) << ','
          << row.window_size << ',' << row.prior_key << ',' << std::fixed
          << std::setprecision( 6 ) << row.reproj_rms_before_px << ','
          << row.reproj_rms_after_px << ',' << row.num_cheirality << ','
          << row.lm_iterations << ',' << row.max_window_pose_shift_m << ','
          << row.segment_id << ',' << ( row.pnp_success ? 1 : 0 ) << ','
          << row.pnp_inliers << ',' << row.outliers_culled << ','
          << row.reproj_rms_after_cull_px << '\n';
    }

    if ( !out )
    {
      return SessionError{ "failed to write diag csv: " + path.string() };
    }
    return std::nullopt;
  }

}  // namespace phad::apps

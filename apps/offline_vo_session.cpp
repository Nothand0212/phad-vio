#include "apps/offline_vo_session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "apps/stereo_pair_stream.hpp"
#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/stereo_rectifier.hpp"
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

    const auto&                  rectified_cal = rectifier.value().calibration();
    frontend::StereoTracker      tracker( rectified_cal, options.tracker );
    estimator::StereoVoEstimator estimator( rectified_cal, options.estimator );

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
          // segment lifecycle; a clean single-segment run stays kCompleted.
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

      const auto                           estimator_begin = std::chrono::steady_clock::now();
      const estimator::KeyframeMeasurement measurement =
          toKeyframeMeasurement( tracks );
      const estimator::VioUpdateResult update =
          estimator.update( measurement );
      const auto estimator_end = std::chrono::steady_clock::now();
      const auto frame_end     = estimator_end;

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

      if ( update.status == estimator::UpdateStatus::kOk )
      {
        rms_after.push_back( update.diagnostics.reproj_rms_after_px );
        common::TimedPose pose;
        pose.timestamp = update.estimate->timestamp;
        pose.T_W_B     = update.estimate->T_W_B;
        poses.push_back( pose );

        const std::uint32_t segment_id = update.diagnostics.segment_id;
        if ( last_segment_id.has_value() && segment_id > *last_segment_id )
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
        any_segment_established = true;
        last_segment_id         = segment_id;
      }

      const auto& d = update.diagnostics;
      result.diag.push_back( VoDiagRow{
          .timestamp_ns            = tracks.timestamp.nanoseconds(),
          .status                  = updateStatusName( update.status ),
          .num_observations        = d.num_observations,
          .num_landmarks           = d.num_landmarks,
          .num_shared              = d.num_shared,
          .low_connectivity        = d.low_connectivity,
          .window_size             = d.window_size,
          .prior_key               = d.prior_key,
          .reproj_rms_before_px    = d.reproj_rms_before_px,
          .reproj_rms_after_px     = d.reproj_rms_after_px,
          .num_cheirality          = d.num_cheirality,
          .lm_iterations           = d.lm_iterations,
          .max_window_pose_shift_m = d.max_window_pose_shift_m,
          .segment_id              = d.segment_id,
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
           "lm_iterations,max_window_pose_shift_m,segment_id\n";

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
          << row.segment_id << '\n';
    }

    if ( !out )
    {
      return SessionError{ "failed to write diag csv: " + path.string() };
    }
    return std::nullopt;
  }

}  // namespace phad::apps

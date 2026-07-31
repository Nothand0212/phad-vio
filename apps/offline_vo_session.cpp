#include "apps/offline_vo_session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
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

    const auto wall_begin = std::chrono::steady_clock::now();
    auto       reader     = opened.value().reader();
    while ( true )
    {
      if ( options.max_frames.has_value() &&
           result.counts.image_frames >= *options.max_frames )
      {
        break;
      }

      auto loaded = reader.takeStereo();
      if ( std::holds_alternative<io::dataset::DatasetReaderEnd>( loaded ) )
      {
        break;
      }
      if ( const auto* error =
               std::get_if<io::dataset::DatasetReaderError>( &loaded ) )
      {
        result.error = SessionError{ "reader error at record " +
                                     std::to_string( error->record_number ) };
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
           "lm_iterations,max_window_pose_shift_m\n";

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
          << row.lm_iterations << ',' << row.max_window_pose_shift_m << '\n';
    }

    if ( !out )
    {
      return SessionError{ "failed to write diag csv: " + path.string() };
    }
    return std::nullopt;
  }

}  // namespace phad::apps

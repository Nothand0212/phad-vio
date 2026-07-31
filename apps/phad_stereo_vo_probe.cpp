#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/common/trajectory.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/eval/tum_io.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"

/**
 * @file phad_stereo_vo_probe.cpp
 * @brief 无窗口跑完整序列，导出 VO 轨迹 TUM 与诊断 CSV。
 */

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_stereo_vo_probe <sequence-root> --tum <path> "
      "[--diag-csv <path>]\n";

  struct Arguments
  {
    std::filesystem::path sequence_root;
    std::filesystem::path tum_path;
    std::filesystem::path diag_csv;
  };

  [[nodiscard]] bool parseArguments( int argc, char** argv,
                                     Arguments& arguments )
  {
    if ( argc < 4 )
    {
      return false;
    }
    arguments.sequence_root = argv[ 1 ];
    for ( int index = 2; index < argc; ++index )
    {
      const std::string_view flag{ argv[ index ] };
      if ( index + 1 >= argc )
      {
        std::cerr << "missing value for " << flag << '\n';
        return false;
      }
      const std::string_view value{ argv[ index + 1 ] };
      ++index;
      if ( flag == "--tum" )
      {
        arguments.tum_path = value;
      }
      else if ( flag == "--diag-csv" )
      {
        arguments.diag_csv = value;
      }
      else
      {
        std::cerr << "unknown flag " << flag << '\n';
        return false;
      }
    }
    return !arguments.sequence_root.empty() && !arguments.tum_path.empty();
  }

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

  [[nodiscard]] int run( const Arguments& arguments )
  {
    auto opened = phad::io::dataset::euroc::open( arguments.sequence_root );
    if ( !opened )
    {
      std::cerr << opened.error().describe() << '\n';
      return 1;
    }

    auto rectifier =
        phad::camera::StereoRectifier::create( opened.value().calibration() );
    if ( !rectifier )
    {
      std::cerr << "stereo rectifier: " << rectifier.error().detail << '\n';
      return 1;
    }

    const auto&                        rectified_cal = rectifier.value().calibration();
    phad::frontend::StereoTracker      tracker( rectified_cal );
    phad::estimator::StereoVoEstimator estimator( rectified_cal );

    std::ofstream diag_out;
    const bool    write_diag = !arguments.diag_csv.empty();
    if ( write_diag )
    {
      diag_out.open( arguments.diag_csv );
      if ( !diag_out )
      {
        std::cerr << "failed to open --diag-csv\n";
        return 1;
      }
      diag_out << "timestamp_ns,status,num_obs,num_landmarks,num_shared,"
                  "low_connectivity,window_size,prior_key,"
                  "reproj_rms_before_px,reproj_rms_after_px,num_cheirality,"
                  "lm_iterations,max_window_pose_shift_m\n";
    }

    std::vector<phad::common::TimedPose> poses;
    std::uint64_t                        frame_count      = 0;
    std::uint64_t                        ok_count         = 0;
    std::uint64_t                        rejected_count   = 0;
    std::uint64_t                        failed_count     = 0;
    std::uint64_t                        low_connectivity = 0;
    std::vector<double>                  rms_after;

    auto reader = opened.value().reader();
    while ( true )
    {
      auto loaded = reader.takeStereo();
      if ( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
               loaded ) )
      {
        break;
      }
      if ( const auto* error =
               std::get_if<phad::io::dataset::DatasetReaderError>( &loaded ) )
      {
        std::cerr << "reader error at record " << error->record_number << '\n';
        return 1;
      }

      const auto& raw       = std::get<phad::sensor::StereoFrame>( loaded );
      auto        rectified = rectifier.value().rectify( raw );
      if ( !rectified )
      {
        std::cerr << "rectify failed: " << rectified.error().detail << '\n';
        return 1;
      }

      const phad::frontend::FrameTracks tracks =
          tracker.process( rectified.value() );
      const phad::estimator::KeyframeMeasurement measurement =
          phad::apps::toKeyframeMeasurement( tracks );
      const phad::estimator::VioUpdateResult result =
          estimator.update( measurement );

      ++frame_count;
      switch ( result.status )
      {
        case phad::estimator::UpdateStatus::kOk:
          ++ok_count;
          break;
        case phad::estimator::UpdateStatus::kRejected:
          ++rejected_count;
          break;
        case phad::estimator::UpdateStatus::kFailed:
          ++failed_count;
          break;
      }
      if ( result.diagnostics.low_connectivity )
      {
        ++low_connectivity;
      }
      if ( result.status == phad::estimator::UpdateStatus::kOk )
      {
        rms_after.push_back( result.diagnostics.reproj_rms_after_px );
        phad::common::TimedPose pose;
        pose.timestamp = result.estimate->timestamp;
        pose.T_W_B     = result.estimate->T_W_B;
        poses.push_back( pose );
      }

      if ( write_diag )
      {
        const auto& d = result.diagnostics;
        diag_out << tracks.timestamp.nanoseconds() << ','
                 << phad::apps::updateStatusName( result.status ) << ','
                 << d.num_observations << ',' << d.num_landmarks << ','
                 << d.num_shared << ',' << ( d.low_connectivity ? 1 : 0 ) << ','
                 << d.window_size << ',' << d.prior_key << ',' << std::fixed
                 << std::setprecision( 6 ) << d.reproj_rms_before_px << ','
                 << d.reproj_rms_after_px << ',' << d.num_cheirality << ','
                 << d.lm_iterations << ',' << d.max_window_pose_shift_m << '\n';
      }
    }

    if ( poses.empty() )
    {
      std::cerr << "no accepted poses to write\n";
      return 1;
    }

    auto trajectory = phad::common::Trajectory::create( std::move( poses ) );
    if ( !trajectory )
    {
      std::cerr << "trajectory create failed: " << trajectory.error().detail
                << '\n';
      return 1;
    }
    if ( const auto write_error =
             phad::eval::writeTum( arguments.tum_path, trajectory.value() ) )
    {
      std::cerr << "write tum failed: " << write_error->describe() << '\n';
      return 1;
    }

    const double reject_rate =
        frame_count == 0
            ? 0.0
            : static_cast<double>( rejected_count + failed_count ) /
                  static_cast<double>( frame_count );

    std::cout << "frames=" << frame_count << '\n'
              << "ok=" << ok_count << '\n'
              << "rejected=" << rejected_count << '\n'
              << "failed=" << failed_count << '\n'
              << "reject_rate=" << reject_rate << '\n'
              << "low_connectivity_frames=" << low_connectivity << '\n'
              << "reproj_rms_after_median_px=" << percentile( rms_after, 0.5 )
              << '\n'
              << "reproj_rms_after_p95_px=" << percentile( rms_after, 0.95 )
              << '\n'
              << "tum=" << arguments.tum_path << '\n';
    return 0;
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
    std::cerr << "probe error: " << exception.what() << '\n';
    return 1;
  }
}

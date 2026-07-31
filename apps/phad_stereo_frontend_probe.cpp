#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "phad/camera/stereo_rectifier.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/stereo_imu_dataset.hpp"

/**
 * @file phad_stereo_frontend_probe.cpp
 * @brief 无窗口跑完整序列，导出 frontend 帧级与 track 生命表 CSV。
 */

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_stereo_frontend_probe <sequence-root> "
      "--frames-csv <path> [--tracks-csv <path>]\n";

  struct Arguments
  {
    std::filesystem::path sequence_root;
    std::filesystem::path frames_csv;
    std::filesystem::path tracks_csv;
  };

  struct LiveRecord
  {
    std::int64_t    first_timestamp_ns = 0;
    std::int64_t    last_timestamp_ns  = 0;
    std::uint32_t   length             = 0;
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
      if ( flag == "--frames-csv" )
      {
        arguments.frames_csv = value;
      }
      else if ( flag == "--tracks-csv" )
      {
        arguments.tracks_csv = value;
      }
      else
      {
        std::cerr << "unknown flag " << flag << '\n';
        return false;
      }
    }
    return !arguments.sequence_root.empty() && !arguments.frames_csv.empty();
  }

  void writeDeadTracks(
      std::ofstream&                                           tracks_out,
      std::unordered_map<phad::frontend::LandmarkId, LiveRecord>& live,
      const phad::frontend::FrameTracks&                       current )
  {
    std::unordered_map<phad::frontend::LandmarkId, LiveRecord> next;
    next.reserve( current.observations.size() );
    for ( const auto& observation : current.observations )
    {
      LiveRecord record;
      const auto it = live.find( observation.id );
      if ( it == live.end() )
      {
        record.first_timestamp_ns = current.timestamp.nanoseconds();
      }
      else
      {
        record.first_timestamp_ns = it->second.first_timestamp_ns;
      }
      record.last_timestamp_ns = current.timestamp.nanoseconds();
      record.length            = observation.length;
      next.emplace( observation.id, record );
    }

    for ( const auto& [ id, record ] : live )
    {
      if ( next.contains( id ) )
      {
        continue;
      }
      tracks_out << id << ',' << record.first_timestamp_ns << ','
                 << record.last_timestamp_ns << ',' << record.length << '\n';
    }
    live.swap( next );
  }

  void flushLiveTracks(
      std::ofstream&                                                 tracks_out,
      const std::unordered_map<phad::frontend::LandmarkId, LiveRecord>& live )
  {
    for ( const auto& [ id, record ] : live )
    {
      tracks_out << id << ',' << record.first_timestamp_ns << ','
                 << record.last_timestamp_ns << ',' << record.length << '\n';
    }
  }

  [[nodiscard]] int run( const Arguments& arguments )
  {
    auto opened =
        phad::io::dataset::euroc::open( arguments.sequence_root );
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

    phad::frontend::StereoTracker tracker( rectifier.value().calibration() );

    std::ofstream frames_out( arguments.frames_csv );
    if ( !frames_out )
    {
      std::cerr << "failed to open --frames-csv\n";
      return 1;
    }
    frames_out
        << "timestamp_ns,tracked,detected,valid,no_right_match,"
           "invalid_disparity,depth_out_of_range,fb_rejected,"
           "epipolar_median_px,epipolar_p95_px\n";

    std::ofstream tracks_out;
    const bool    write_tracks = !arguments.tracks_csv.empty();
    if ( write_tracks )
    {
      tracks_out.open( arguments.tracks_csv );
      if ( !tracks_out )
      {
        std::cerr << "failed to open --tracks-csv\n";
        return 1;
      }
      tracks_out << "id,first_timestamp_ns,last_timestamp_ns,length\n";
    }

    auto reader = opened.value().reader();
    std::unordered_map<phad::frontend::LandmarkId, LiveRecord> live;

    std::uint64_t frame_count      = 0;
    std::uint64_t min_tracks       = 0;
    bool          have_min_tracks  = false;
    std::uint64_t sum_obs          = 0;
    std::uint64_t sum_valid        = 0;
    std::uint64_t sum_no_right     = 0;
    std::uint64_t sum_invalid_disp = 0;
    std::uint64_t sum_depth        = 0;
    std::uint64_t sum_fb           = 0;
    std::uint64_t sum_tracked      = 0;
    std::uint64_t sum_detected     = 0;
    std::vector<double> epipolar_medians;
    std::vector<double> epipolar_p95s;

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

      const auto& raw = std::get<phad::sensor::StereoFrame>( loaded );
      auto rectified  = rectifier.value().rectify( raw );
      if ( !rectified )
      {
        std::cerr << "rectify failed: " << rectified.error().detail << '\n';
        return 1;
      }

      const phad::frontend::FrameTracks tracks =
          tracker.process( rectified.value() );

      std::uint32_t valid = 0;
      std::uint32_t no_right = 0;
      std::uint32_t invalid_disp = 0;
      std::uint32_t depth = 0;
      for ( const auto& observation : tracks.observations )
      {
        switch ( observation.status )
        {
          case phad::frontend::StereoStatus::kValid:
            ++valid;
            break;
          case phad::frontend::StereoStatus::kNoRightMatch:
            ++no_right;
            break;
          case phad::frontend::StereoStatus::kInvalidDisparity:
            ++invalid_disp;
            break;
          case phad::frontend::StereoStatus::kDepthOutOfRange:
            ++depth;
            break;
        }
      }

      const std::uint64_t track_count = tracks.observations.size();
      if ( !have_min_tracks || track_count < min_tracks )
      {
        min_tracks      = track_count;
        have_min_tracks = true;
      }

      frames_out << tracks.timestamp.nanoseconds() << ','
                 << tracks.stats.tracked << ',' << tracks.stats.detected << ','
                 << valid << ',' << no_right << ',' << invalid_disp << ','
                 << depth << ',' << tracks.stats.forward_backward_rejected
                 << ',' << std::setprecision( 6 ) << std::fixed
                 << tracks.stats.epipolar_median_px << ','
                 << tracks.stats.epipolar_p95_px << '\n';

      if ( write_tracks )
      {
        writeDeadTracks( tracks_out, live, tracks );
      }

      ++frame_count;
      sum_obs += track_count;
      sum_valid += valid;
      sum_no_right += no_right;
      sum_invalid_disp += invalid_disp;
      sum_depth += depth;
      sum_fb += tracks.stats.forward_backward_rejected;
      sum_tracked += tracks.stats.tracked;
      sum_detected += tracks.stats.detected;
      epipolar_medians.push_back( tracks.stats.epipolar_median_px );
      epipolar_p95s.push_back( tracks.stats.epipolar_p95_px );
    }

    if ( write_tracks )
    {
      flushLiveTracks( tracks_out, live );
    }

    const auto percentile = []( std::vector<double> values, double q )
    {
      if ( values.empty() )
      {
        return 0.0;
      }
      std::sort( values.begin(), values.end() );
      const double position =
          q * static_cast<double>( values.size() - 1U );
      const auto lower = static_cast<std::size_t>( std::floor( position ) );
      const auto upper = static_cast<std::size_t>( std::ceil( position ) );
      if ( lower == upper )
      {
        return values[ lower ];
      }
      const double weight = position - static_cast<double>( lower );
      return values[ lower ] * ( 1.0 - weight ) + values[ upper ] * weight;
    };

    const double denom =
        sum_obs == 0 ? 1.0 : static_cast<double>( sum_obs );
    std::cout << "frames=" << frame_count << '\n'
              << "min_tracks=" << ( have_min_tracks ? min_tracks : 0 ) << '\n'
              << "mean_tracks="
              << ( frame_count == 0
                       ? 0.0
                       : static_cast<double>( sum_obs ) /
                             static_cast<double>( frame_count ) )
              << '\n'
              << "valid_rate=" << static_cast<double>( sum_valid ) / denom
              << '\n'
              << "no_right_match_rate="
              << static_cast<double>( sum_no_right ) / denom << '\n'
              << "invalid_disparity_rate="
              << static_cast<double>( sum_invalid_disp ) / denom << '\n'
              << "depth_out_of_range_rate="
              << static_cast<double>( sum_depth ) / denom << '\n'
              << "fb_rejected_total=" << sum_fb << '\n'
              << "tracked_total=" << sum_tracked << '\n'
              << "detected_total=" << sum_detected << '\n'
              << "epipolar_median_px="
              << percentile( epipolar_medians, 0.5 ) << '\n'
              << "epipolar_p95_px=" << percentile( epipolar_p95s, 0.95 )
              << '\n';
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

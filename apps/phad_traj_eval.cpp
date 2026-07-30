#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

#include "phad/common/trajectory.hpp"
#include "phad/eval/ate.hpp"
#include "phad/eval/rpe.hpp"
#include "phad/eval/tum_io.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

/**
 * @file phad_traj_eval.cpp
 * @brief 比较估计轨迹与真值轨迹并输出 ATE。
 */

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_traj_eval --est <est.tum>\n"
      "                      (--gt <gt.tum> | --gt-euroc <sequence-root>)\n"
      "                      [--max-dt-ms <value>] [--min-match-rate <value>]\n"
      "                      [--rpe-delta-s <value>] [--errors-csv <path>]\n";

  struct Arguments
  {
    std::filesystem::path est_path;
    std::filesystem::path gt_path;
    std::filesystem::path gt_sequence_root;
    std::filesystem::path errors_csv_path;
    double                max_dt_ms      = 2.5;
    double                min_match_rate = 0.5;
    double                rpe_delta_s    = 1.0;
  };

  [[nodiscard]] bool parseDouble( std::string_view text, double& value )
  {
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    return parsed.ec == std::errc{} &&
           parsed.ptr == text.data() + text.size() && std::isfinite( value );
  }

  [[nodiscard]] bool parseArguments( int argc, char** argv,
                                     Arguments& arguments )
  {
    for ( int index = 1; index < argc; ++index )
    {
      const std::string_view flag{ argv[ index ] };
      if ( index + 1 >= argc )
      {
        std::cerr << "missing value for " << flag << '\n';
        return false;
      }
      const std::string_view value{ argv[ index + 1 ] };
      ++index;
      if ( flag == "--est" )
      {
        arguments.est_path = value;
      }
      else if ( flag == "--gt" )
      {
        arguments.gt_path = value;
      }
      else if ( flag == "--gt-euroc" )
      {
        arguments.gt_sequence_root = value;
      }
      else if ( flag == "--errors-csv" )
      {
        arguments.errors_csv_path = value;
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
             arguments.min_match_rate < 0.0 || arguments.min_match_rate > 1.0 )
        {
          std::cerr << "--min-match-rate expects a value in [0, 1]\n";
          return false;
        }
      }
      else if ( flag == "--rpe-delta-s" )
      {
        // 间隔匹配容差固定在库内，间隔必须大于它才有意义。
        constexpr double kMinRpeDeltaS =
            static_cast<double>( phad::eval::kDefaultRpeDeltaToleranceNs ) *
            1e-9;
        if ( !parseDouble( value, arguments.rpe_delta_s ) ||
             arguments.rpe_delta_s <= kMinRpeDeltaS )
        {
          std::cerr << "--rpe-delta-s expects a number greater than the "
                    << kMinRpeDeltaS << " s pairing tolerance\n";
          return false;
        }
      }
      else
      {
        std::cerr << "unknown flag " << flag << '\n';
        return false;
      }
    }

    if ( arguments.est_path.empty() )
    {
      std::cerr << "--est is required\n";
      return false;
    }
    if ( arguments.gt_path.empty() == arguments.gt_sequence_root.empty() )
    {
      std::cerr << "exactly one of --gt and --gt-euroc is required\n";
      return false;
    }
    return true;
  }

  [[nodiscard]] std::optional<phad::common::Trajectory> loadGroundtruth(
      const Arguments& arguments )
  {
    if ( !arguments.gt_sequence_root.empty() )
    {
      auto trajectory = phad::io::dataset::euroc::openGroundtruth(
          arguments.gt_sequence_root );
      if ( !trajectory )
      {
        std::cerr << trajectory.error().describe() << '\n';
        return std::nullopt;
      }
      return std::move( trajectory ).value();
    }
    auto trajectory = phad::eval::readTum( arguments.gt_path );
    if ( !trajectory )
    {
      std::cerr << trajectory.error().describe() << '\n';
      return std::nullopt;
    }
    return std::move( trajectory ).value();
  }

  void printStats( std::string_view label, const phad::eval::ErrorStats& stats )
  {
    std::cout << label << "  rmse " << stats.rmse << "  mean " << stats.mean
              << "  median " << stats.median << "  std " << stats.stddev
              << "  max " << stats.max << '\n';
  }

  [[nodiscard]] bool writeErrorsCsv( const std::filesystem::path& path,
                                     const phad::eval::AteReport& report )
  {
    std::ofstream stream( path, std::ios::trunc );
    if ( !stream )
    {
      std::cerr << "failed to open " << path.string() << " for writing\n";
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
    if ( !stream )
    {
      std::cerr << "failed while writing " << path.string() << '\n';
      return false;
    }
    return true;
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

  auto est = phad::eval::readTum( arguments.est_path );
  if ( !est )
  {
    std::cerr << est.error().describe() << '\n';
    return 1;
  }
  auto gt = loadGroundtruth( arguments );
  if ( !gt.has_value() )
  {
    return 1;
  }

  phad::eval::AssociationOptions association_options;
  association_options.max_dt_ns = static_cast<std::int64_t>(
      std::llround( arguments.max_dt_ms * 1'000'000.0 ) );
  association_options.min_match_rate = arguments.min_match_rate;

  phad::eval::AteOptions ate_options;
  ate_options.association = association_options;

  auto report = phad::eval::computeAte( est.value(), *gt, ate_options );
  if ( !report )
  {
    std::cerr << report.error().describe() << '\n';
    return 1;
  }

  const phad::eval::Association& association = report.value().association;
  std::cout << "estimate poses " << est.value().size() << ", groundtruth poses "
            << gt->size() << '\n';
  std::cout << "matched " << association.pairs.size() << " of "
            << association.est_total << " (rate " << association.matchRate()
            << "), dropped " << association.dropped_out_of_range
            << " outside groundtruth span and "
            << association.dropped_over_threshold << " beyond "
            << arguments.max_dt_ms << " ms\n";
  printStats( "ATE translation [m] ", report.value().trans_m );
  printStats( "ATE rotation [deg]  ", report.value().rot_deg );

  phad::eval::RpeOptions rpe_options;
  rpe_options.association = association_options;
  rpe_options.delta_ns    = static_cast<std::int64_t>(
      std::llround( arguments.rpe_delta_s * 1'000'000'000.0 ) );

  auto rpe = phad::eval::computeRpe( est.value(), *gt, rpe_options );
  if ( !rpe )
  {
    std::cerr << rpe.error().describe() << '\n';
    return 1;
  }

  std::cout << "RPE over " << arguments.rpe_delta_s << " s from "
            << rpe.value().pair_count << " pose pairs, skipped "
            << rpe.value().dropped_no_partner << " poses without a partner\n";
  printStats( "RPE translation [m] ", rpe.value().trans_m );
  printStats( "RPE rotation [deg]  ", rpe.value().rot_deg );

  if ( !arguments.errors_csv_path.empty() &&
       !writeErrorsCsv( arguments.errors_csv_path, report.value() ) )
  {
    return 1;
  }
  return 0;
}

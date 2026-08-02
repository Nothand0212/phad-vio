#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include "apps/offline_vo_session.hpp"
#include "phad/eval/tum_io.hpp"

/**
 * @file phad_stereo_vo_probe.cpp
 * @brief 无窗口跑完整序列，导出 VO 轨迹 TUM 与诊断 CSV。
 *
 * pipeline 编排在 OfflineVoSession；本文件只做 CLI、落盘与人读摘要。
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

  [[nodiscard]] int run( const Arguments& arguments )
  {
    phad::apps::OfflineVoSessionOptions options;
    options.sequence_root = arguments.sequence_root;

    phad::apps::OfflineVoSessionResult result =
        phad::apps::runOfflineVoSession( options );
    if ( result.error.has_value() )
    {
      std::cerr << result.error->detail << '\n';
      return 1;
    }
    if ( !result.trajectory.has_value() )
    {
      std::cerr << "no accepted poses to write\n";
      return 1;
    }

    if ( !arguments.diag_csv.empty() )
    {
      if ( const auto write_error =
               phad::apps::writeDiagCsv( arguments.diag_csv, result.diag ) )
      {
        std::cerr << write_error->detail << '\n';
        return 1;
      }
    }

    if ( const auto write_error =
             phad::eval::writeTum( arguments.tum_path, *result.trajectory ) )
    {
      std::cerr << "write tum failed: " << write_error->describe() << '\n';
      return 1;
    }

    const double reject_rate =
        result.counts.image_frames == 0
            ? 0.0
            : static_cast<double>( result.counts.rejected +
                                   result.counts.failed ) /
                  static_cast<double>( result.counts.image_frames );

    std::cout << "frames=" << result.counts.image_frames << '\n'
              << "ok=" << result.counts.ok << '\n'
              << "rejected=" << result.counts.rejected << '\n'
              << "failed=" << result.counts.failed << '\n'
              << "reject_rate=" << reject_rate << '\n'
              << "low_connectivity_frames=" << result.counts.low_connectivity
              << '\n'
              << "segments=" << result.counts.segments << '\n'
              << "reanchors=" << result.counts.reanchors << '\n'
              << "seed_rejected=" << result.counts.seed_rejected << '\n'
              << "pnp_successes=" << result.counts.pnp_successes << '\n'
              << "pnp_fallbacks=" << result.counts.pnp_fallbacks << '\n'
              << "outliers_culled=" << result.counts.outliers_culled << '\n'
              << "outliers_culled_unique="
              << result.counts.outliers_culled_unique << '\n'
              << "outlier_reopts=" << result.counts.outlier_reopts << '\n'
              << "drops_skipped=" << result.counts.drops_skipped << '\n'
              << "reproj_rms_after_median_px=" << result.reproj.median_px
              << '\n'
              << "reproj_rms_after_p95_px=" << result.reproj.p95_px << '\n'
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

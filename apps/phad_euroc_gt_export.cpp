#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include "phad/eval/tum_io.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

/**
 * @file phad_euroc_gt_export.cpp
 * @brief 把 EuRoC 真值轨迹导出为 TUM 格式。
 *
 * 导出文件用于 evo 交叉验证，以及把真值当作估计输入的自比检查。
 */

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_euroc_gt_export --seq <sequence-root> --out <trajectory.tum>\n";

  struct Arguments
  {
    std::filesystem::path sequence_root;
    std::filesystem::path output_path;
  };

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
      const std::string value{ argv[ index + 1 ] };
      ++index;
      if ( flag == "--seq" )
      {
        arguments.sequence_root = value;
      }
      else if ( flag == "--out" )
      {
        arguments.output_path = value;
      }
      else
      {
        std::cerr << "unknown flag " << flag << '\n';
        return false;
      }
    }
    return !arguments.sequence_root.empty() && !arguments.output_path.empty();
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

  auto trajectory =
      phad::io::dataset::euroc::openGroundtruth( arguments.sequence_root );
  if ( !trajectory )
  {
    std::cerr << trajectory.error().describe() << '\n';
    return 1;
  }

  if ( auto error =
           phad::eval::writeTum( arguments.output_path, trajectory.value() ) )
  {
    std::cerr << error->describe() << '\n';
    return 1;
  }

  std::cout << "wrote " << trajectory.value().size() << " poses to "
            << arguments.output_path.string() << ", timestamps ["
            << trajectory.value().firstTimestamp().nanoseconds() << ", "
            << trajectory.value().lastTimestamp().nanoseconds() << "] ns\n";
  return 0;
}

#include <filesystem>
#include <iostream>
#include <string_view>

#include "apps/gyro_alignment_runner.hpp"

namespace
{

  constexpr std::string_view kUsage =
      "usage: phad_gyro_align <q1-run-dir> --out <new-output-dir>\n";

}  // namespace

int main( int argc, char** argv )
{
  if ( argc != 4 || std::string_view{ argv[ 2 ] } != "--out" ||
       std::string_view{ argv[ 1 ] }.empty() ||
       std::string_view{ argv[ 3 ] }.empty() )
  {
    std::cerr << kUsage;
    return 64;
  }

  const auto result = phad::apps::runGyroAlignment(
      phad::apps::makeGyroAlignmentV1ProtocolDescriptor(),
      std::filesystem::path{ argv[ 1 ] },
      std::filesystem::path{ argv[ 3 ] } );
  if ( !result.verdict.detail.empty() )
  {
    std::cerr << result.verdict.detail << '\n';
  }
  return result.verdict.exit_code;
}

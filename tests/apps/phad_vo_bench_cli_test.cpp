#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

#ifdef __linux__
#include <sys/wait.h>
#endif

namespace
{

#ifndef PHAD_VO_BENCH_PATH
#define PHAD_VO_BENCH_PATH ""
#endif

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream in( path );
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
  }

  [[nodiscard]] int runBench( std::string_view args,
                              const std::filesystem::path& stdout_path,
                              const std::filesystem::path& stderr_path )
  {
    const std::string command =
        std::string{ "\"" PHAD_VO_BENCH_PATH "\" " } + std::string( args ) +
        " > \"" + stdout_path.string() + "\" 2> \"" + stderr_path.string() +
        "\"";
    const int status = std::system( command.c_str() );
#ifdef __linux__
    if ( !WIFEXITED( status ) )
    {
      return -1;
    }
    return WEXITSTATUS( status );
#else
    return status;
#endif
  }

  [[nodiscard]] std::string extractConfigHash( const std::string& stdout_text )
  {
    constexpr std::string_view kPrefix = "config_hash=";
    const auto                 pos     = stdout_text.find( kPrefix );
    if ( pos == std::string::npos )
    {
      return {};
    }
    const auto start = pos + kPrefix.size();
    const auto end   = stdout_text.find( '\n', start );
    if ( end == std::string::npos )
    {
      return stdout_text.substr( start );
    }
    return stdout_text.substr( start, end - start );
  }

  TEST( VoBenchCliTest, RejectsNegativeMaxOutlierReopts )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_reject_neg";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --out \"" + ( root / "out" ).string() +
            "\" --sequence-name cli_reject --max-outlier-reopts -1",
        stdout_path, stderr_path );

    EXPECT_NE( exit_code, 0 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--max-outlier-reopts" ), std::string::npos );
    EXPECT_NE( err.find( "non-negative" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, MaxOutlierReoptsOverrideChangesConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_hash_probe";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_max1    = root / "out_max1";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_max1    = root / "stdout_max1.txt";
    const auto stderr_max1    = root / "stderr_max1.txt";

    // meta.json / config_hash are written before the session opens the
    // dataset, so a missing sequence still exercises flattenConfig.
    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_max1.string() +
                        "\" --sequence-name hash_probe --force "
                        "--max-outlier-reopts 1",
                    stdout_max1, stderr_max1 );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_max1 = extractConfigHash( readFile( stdout_max1 ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_max1.empty() ) << readFile( stderr_max1 );
    EXPECT_NE( hash_default, hash_max1 );

    const std::string meta_default = readFile( out_default / "meta.json" );
    const std::string meta_max1    = readFile( out_max1 / "meta.json" );
    EXPECT_NE( meta_default.find( "estimator.max_outlier_reopts=3" ),
               std::string::npos );
    EXPECT_NE( meta_max1.find( "estimator.max_outlier_reopts=1" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

}  // namespace

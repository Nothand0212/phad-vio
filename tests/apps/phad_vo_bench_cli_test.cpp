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

  TEST( VoBenchCliTest, UsageMentionsProbeB )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root =
        std::filesystem::temp_directory_path() / "phad_vo_bench_cli_usage";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    // No sequence-root → parseArguments fails and prints usage.
    const int exit_code = runBench( "", stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--probe-b" ), std::string::npos );
    EXPECT_NE( err.find( "--defer-drop-topk" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, ProbeBPathDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_probe_b_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_probe   = root / "out_probe";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_probe   = root / "stdout_probe.txt";
    const auto stderr_probe   = root / "stderr_probe.txt";
    const auto probe_path     = root / "probe_b.jsonl";

    // meta.json / config_hash are written before the session opens the
    // dataset; Probe B is CLI-only and must not enter flattenConfig.
    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_probe.string() +
                        "\" --sequence-name hash_probe --force --probe-b \"" +
                        probe_path.string() + "\"",
                    stdout_probe, stderr_probe );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_probe = extractConfigHash( readFile( stdout_probe ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_probe.empty() ) << readFile( stderr_probe );
    EXPECT_EQ( hash_default, hash_probe );

    std::filesystem::remove_all( root );
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

  TEST( VoBenchCliTest, RejectsNegativeSkipDropMinCulled )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_reject_skip_neg";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --out \"" + ( root / "out" ).string() +
            "\" --sequence-name cli_reject --skip-drop-min-culled -1",
        stdout_path, stderr_path );

    EXPECT_NE( exit_code, 0 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--skip-drop-min-culled" ), std::string::npos );
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

  TEST( VoBenchCliTest, DeferDropTopkDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_defer_topk_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_topk    = root / "out_topk";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_topk    = root / "stdout_topk.txt";
    const auto stderr_topk    = root / "stderr_topk.txt";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_topk.string() +
                        "\" --sequence-name hash_probe --force "
                        "--defer-drop-topk 8",
                    stdout_topk, stderr_topk );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_topk = extractConfigHash( readFile( stdout_topk ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_topk.empty() ) << readFile( stderr_topk );
    EXPECT_EQ( hash_default, hash_topk );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, RejectsNegativeDeferDropTopk )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_reject_topk_neg";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --out \"" + ( root / "out" ).string() +
            "\" --sequence-name cli_reject --defer-drop-topk -1",
        stdout_path, stderr_path );

    EXPECT_NE( exit_code, 0 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--defer-drop-topk" ), std::string::npos );
    EXPECT_NE( err.find( "non-negative" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, SkipDropMinCulledOverrideChangesConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_skip_drop_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_zero    = root / "out_zero";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_zero    = root / "stdout_zero.txt";
    const auto stderr_zero    = root / "stderr_zero.txt";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_zero.string() +
                        "\" --sequence-name hash_probe --force "
                        "--skip-drop-min-culled 0",
                    stdout_zero, stderr_zero );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_zero = extractConfigHash( readFile( stdout_zero ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_zero.empty() ) << readFile( stderr_zero );
    EXPECT_NE( hash_default, hash_zero );

    const std::string meta_default = readFile( out_default / "meta.json" );
    const std::string meta_zero    = readFile( out_zero / "meta.json" );
    EXPECT_NE( meta_default.find( "session.skip_drop_min_culled=4" ),
               std::string::npos );
    EXPECT_NE( meta_zero.find( "session.skip_drop_min_culled=0" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

}  // namespace

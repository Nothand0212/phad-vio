#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

#include "tests/apps/synthetic_euroc_fixture.hpp"

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
    std::ifstream      in( path );
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
  }

  [[nodiscard]] int runBench( std::string_view             args,
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
    EXPECT_NE( err.find( "--keyframe-shadow-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--vio-state-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--vio-init-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--fixed-lag-shadow-probe" ),
               std::string::npos );
    EXPECT_NE( err.find( "--defer-drop-topk" ), std::string::npos );
    EXPECT_NE( err.find( "--evict-skip-culled" ), std::string::npos );
    EXPECT_NE( err.find( "--zombie-drop-age" ), std::string::npos );

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

  TEST( VoBenchCliTest, KeyframeShadowPathDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_keyframe_shadow_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_probe   = root / "out_probe";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_probe   = root / "stdout_probe.txt";
    const auto stderr_probe   = root / "stderr_probe.txt";
    const auto probe_path     = root / "keyframe_shadow.csv";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_shadow --force",
                    stdout_default, stderr_default );
    (void)runBench(
        "/nonexistent/sequence --out \"" + out_probe.string() +
            "\" --sequence-name hash_shadow --force "
            "--keyframe-shadow-probe \"" +
            probe_path.string() + "\"",
        stdout_probe, stderr_probe );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_probe =
        extractConfigHash( readFile( stdout_probe ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_probe.empty() ) << readFile( stderr_probe );
    EXPECT_EQ( hash_default, hash_probe );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioStateProbePathDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_state_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_probe   = root / "out_probe";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_probe   = root / "stdout_probe.txt";
    const auto stderr_probe   = root / "stderr_probe.txt";
    const auto probe_path     = root / "vio_state.csv";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_vio_state --force",
                    stdout_default, stderr_default );
    (void)runBench(
        "/nonexistent/sequence --out \"" + out_probe.string() +
            "\" --sequence-name hash_vio_state --force "
            "--vio-state-probe \"" +
            probe_path.string() + "\"",
        stdout_probe, stderr_probe );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_probe =
        extractConfigHash( readFile( stdout_probe ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_probe.empty() ) << readFile( stderr_probe );
    EXPECT_EQ( hash_default, hash_probe );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioStateProbeRequiresPath )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_state_missing";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name missing_vio_state "
        "--vio-state-probe",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "missing value for --vio-state-probe" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioStateProbeRejectsNoImu )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_state_no_imu";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name conflict_vio_state "
        "--out \"" +
            ( root / "out" ).string() +
            "\" --no-imu --vio-state-probe \"" +
            ( root / "state.csv" ).string() + "\"",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--vio-state-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--no-imu" ), std::string::npos );
    EXPECT_NE( err.find( "cannot be used with" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioInitProbePathDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_init_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_probe   = root / "out_probe";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_probe   = root / "stdout_probe.txt";
    const auto stderr_probe   = root / "stderr_probe.txt";
    const auto probe_path     = root / "vio_init.csv";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_vio_init --force",
                    stdout_default, stderr_default );
    (void)runBench(
        "/nonexistent/sequence --out \"" + out_probe.string() +
            "\" --sequence-name hash_vio_init --force "
            "--vio-init-probe \"" +
            probe_path.string() + "\"",
        stdout_probe, stderr_probe );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_probe =
        extractConfigHash( readFile( stdout_probe ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_probe.empty() ) << readFile( stderr_probe );
    EXPECT_EQ( hash_default, hash_probe );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioInitProbeRequiresPath )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_init_missing";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name missing_vio_init "
        "--vio-init-probe",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "missing value for --vio-init-probe" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, VioInitProbeRejectsNoImu )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_vio_init_no_imu";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name conflict_vio_init "
        "--out \"" +
            ( root / "out" ).string() +
            "\" --no-imu --vio-init-probe \"" +
            ( root / "init.csv" ).string() + "\"",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--vio-init-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--no-imu" ), std::string::npos );
    EXPECT_NE( err.find( "cannot be used with" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, FixedLagShadowProbePathDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_fixed_lag_shadow_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_probe   = root / "out_probe";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_probe   = root / "stdout_probe.txt";
    const auto stderr_probe   = root / "stderr_probe.txt";
    const auto probe_path     = root / "fixed_lag_shadow.csv";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_fixed_lag_shadow --force",
                    stdout_default, stderr_default );
    (void)runBench(
        "/nonexistent/sequence --out \"" + out_probe.string() +
            "\" --sequence-name hash_fixed_lag_shadow --force "
            "--fixed-lag-shadow-probe \"" +
            probe_path.string() + "\"",
        stdout_probe, stderr_probe );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_probe =
        extractConfigHash( readFile( stdout_probe ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_probe.empty() ) << readFile( stderr_probe );
    EXPECT_EQ( hash_default, hash_probe );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, FixedLagShadowProbeRequiresPath )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_fixed_lag_shadow_missing";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name missing_fixed_lag_shadow "
        "--fixed-lag-shadow-probe",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "missing value for --fixed-lag-shadow-probe" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, FixedLagShadowProbeRejectsNoImu )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_fixed_lag_shadow_no_imu";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --sequence-name conflict_fixed_lag_shadow "
        "--out \"" +
            ( root / "out" ).string() +
            "\" --no-imu --fixed-lag-shadow-probe \"" +
            ( root / "shadow.csv" ).string() + "\"",
        stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--fixed-lag-shadow-probe" ), std::string::npos );
    EXPECT_NE( err.find( "--no-imu" ), std::string::npos );
    EXPECT_NE( err.find( "cannot be used with" ), std::string::npos );

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

  TEST( VoBenchCliTest, EvictSkipCulledDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_evict_skip_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_evict   = root / "out_evict";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_evict   = root / "stdout_evict.txt";
    const auto stderr_evict   = root / "stderr_evict.txt";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_evict.string() +
                        "\" --sequence-name hash_probe --force "
                        "--evict-skip-culled",
                    stdout_evict, stderr_evict );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_evict = extractConfigHash( readFile( stdout_evict ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_evict.empty() ) << readFile( stderr_evict );
    EXPECT_EQ( hash_default, hash_evict );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, ZombieDropAgeOverrideChangesConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_zombie_age_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_age5    = root / "out_age5";
    const auto out_age8    = root / "out_age8";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_age5    = root / "stdout_age5.txt";
    const auto stderr_age5    = root / "stderr_age5.txt";
    const auto stdout_age8    = root / "stdout_age8.txt";
    const auto stderr_age8    = root / "stderr_age8.txt";

    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_age5.string() +
                        "\" --sequence-name hash_probe --force "
                        "--zombie-drop-age 5",
                    stdout_age5, stderr_age5 );
    (void)runBench( "/nonexistent/sequence --out \"" + out_age8.string() +
                        "\" --sequence-name hash_probe --force "
                        "--zombie-drop-age 8",
                    stdout_age8, stderr_age8 );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_age5 = extractConfigHash( readFile( stdout_age5 ) );
    const std::string hash_age8 = extractConfigHash( readFile( stdout_age8 ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_age5.empty() ) << readFile( stderr_age5 );
    ASSERT_FALSE( hash_age8.empty() ) << readFile( stderr_age8 );
    EXPECT_EQ( hash_default, hash_age5 );
    EXPECT_NE( hash_default, hash_age8 );

    const std::string meta_default = readFile( out_default / "meta.json" );
    EXPECT_NE( meta_default.find( "session.zombie_drop_age=5" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, RejectsNegativeZombieDropAge )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_reject_zombie_age_neg";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --out \"" + ( root / "out" ).string() +
            "\" --sequence-name cli_reject --zombie-drop-age -1",
        stdout_path, stderr_path );

    EXPECT_NE( exit_code, 0 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--zombie-drop-age" ), std::string::npos );
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

  TEST( VoBenchCliTest, UsageMentionsCandidate )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root =
        std::filesystem::temp_directory_path() / "phad_vo_bench_cli_candidate";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench( "", stdout_path, stderr_path );
    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE( err.find( "--candidate" ), std::string::npos );
    EXPECT_NE( err.find( "--tracker-stereo-uniq-ratio" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, CandidateFlagDoesNotChangeConfigHash )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_candidate_hash";
    std::filesystem::remove_all( root );
    const auto out_default = root / "out_default";
    const auto out_cand    = root / "out_cand";
    std::filesystem::create_directories( root );

    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_cand    = root / "stdout_cand.txt";
    const auto stderr_cand    = root / "stderr_cand.txt";

    // meta.json / config_hash 在打开 dataset 前写；--candidate 是 CLI-only，
    // 不得进入 flattenConfig。
    (void)runBench( "/nonexistent/sequence --out \"" + out_default.string() +
                        "\" --sequence-name hash_probe --force",
                    stdout_default, stderr_default );
    (void)runBench( "/nonexistent/sequence --out \"" + out_cand.string() +
                        "\" --sequence-name hash_probe --force --candidate",
                    stdout_cand, stderr_cand );

    const std::string hash_default =
        extractConfigHash( readFile( stdout_default ) );
    const std::string hash_cand = extractConfigHash( readFile( stdout_cand ) );
    ASSERT_FALSE( hash_default.empty() ) << readFile( stderr_default );
    ASSERT_FALSE( hash_cand.empty() ) << readFile( stderr_cand );
    EXPECT_EQ( hash_default, hash_cand );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, CandidateWritesIndependentArtifacts )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    phad::testing::SyntheticEurocFixture fixture;

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_candidate_artifacts";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto out_plain = root / "out_plain";
    const auto out_cand  = root / "out_cand";

    // 合成全同 blob 需要关闭唯一性/双向检查（与 session 测试一致）。
    const std::string base =
        " \"" + fixture.root().string() +
        "\" --sequence-name synthetic"
        " --gt-euroc \"" +
        fixture.root().string() +
        "\" --rpe-delta-s 0.05 --force --tracker-stereo-uniq-ratio 0"
        " --tracker-disable-stereo-check-bidir";

    const auto stdout_plain = root / "stdout_plain.txt";
    const auto stderr_plain = root / "stderr_plain.txt";
    const auto stdout_cand  = root / "stdout_cand.txt";
    const auto stderr_cand  = root / "stderr_cand.txt";

    const int plain_code = runBench(
        base + " --out \"" + out_plain.string() + "\"", stdout_plain,
        stderr_plain );
    const int cand_code = runBench(
        base + " --out \"" + out_cand.string() + "\" --candidate",
        stdout_cand, stderr_cand );

    ASSERT_EQ( plain_code, 0 ) << readFile( stderr_plain );
    ASSERT_EQ( cand_code, 0 ) << readFile( stderr_cand );

    // production 主产物在 --candidate 开关下不受影响（meta.json 的
    // created_utc 每次 run 不同，用 config_hash 代替逐字节比较）。
    EXPECT_EQ( readFile( out_plain / "est.tum" ),
               readFile( out_cand / "est.tum" ) );
    EXPECT_EQ( readFile( out_plain / "diag.csv" ),
               readFile( out_cand / "diag.csv" ) );
    EXPECT_EQ( extractConfigHash( readFile( stdout_plain ) ),
               extractConfigHash( readFile( stdout_cand ) ) );

    // candidate artifacts 只出现在 --candidate run。P2b: candidate 是
    // 独立 fixed-lag 估计器（非 twin）——结构断言：candidate est 行数
    // 与 production 相同（同一序列帧数）、无 nan/inf、无 terminal。
    EXPECT_FALSE( std::filesystem::exists( out_plain / "candidate" ) );
    const auto candidate_dir = out_cand / "candidate";
    ASSERT_TRUE( std::filesystem::exists( candidate_dir / "est.tum" ) );
    const std::string cand_est    = readFile( candidate_dir / "est.tum" );
    const auto        count_lines = []( const std::string& text ) {
      return static_cast<std::size_t>(
          std::count( text.begin(), text.end(), '\n' ) );
    };
    EXPECT_EQ( count_lines( cand_est ),
               count_lines( readFile( out_cand / "est.tum" ) ) );
    EXPECT_EQ( cand_est.find( "nan" ), std::string::npos );
    EXPECT_EQ( cand_est.find( "inf" ), std::string::npos );
    ASSERT_TRUE( std::filesystem::exists( candidate_dir / "diag.csv" ) );
    ASSERT_TRUE( std::filesystem::exists( candidate_dir / "meta.json" ) );
    const std::string meta = readFile( candidate_dir / "meta.json" );
    EXPECT_NE( meta.find( "\"ownership\":\"full_pipeline\"" ),
               std::string::npos );
    EXPECT_NE( meta.find( "\"terminal\":0" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

}  // namespace

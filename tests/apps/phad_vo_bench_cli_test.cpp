#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

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

  TEST( VoBenchCliTest, UsageMentionsProbeBAndGyroObserve )
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
    EXPECT_NE( err.find( "--evict-skip-culled" ), std::string::npos );
    EXPECT_NE( err.find( "--zombie-drop-age" ), std::string::npos );
    EXPECT_NE( err.find( "--gyro-observe" ), std::string::npos );
    EXPECT_NE( err.find( "--estimator-enable-pnp-cheirality-retry" ),
               std::string::npos );
    EXPECT_NE( err.find( "--dataset-format" ), std::string::npos );
    EXPECT_NE( err.find( "euroc|tum-vi" ), std::string::npos );
    EXPECT_NE( err.find( "--gt-tum-vi" ), std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest,
        DatasetFormatIsExplicitAndPreservesFormalEurocIdentity )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_dataset_format";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );

    struct RunPaths
    {
      std::filesystem::path out;
      std::filesystem::path stdout_path;
      std::filesystem::path stderr_path;
    };
    const RunPaths default_euroc{ root / "default-euroc",
                                  root / "default-euroc.stdout",
                                  root / "default-euroc.stderr" };
    const RunPaths explicit_euroc{ root / "explicit-euroc",
                                   root / "explicit-euroc.stdout",
                                   root / "explicit-euroc.stderr" };
    const RunPaths tum_vi{ root / "tum-vi", root / "tum-vi.stdout",
                           root / "tum-vi.stderr" };
    const auto     tum_vi_sequence = root / "tum-vi-sequence";
    for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
    {
      const auto sensor_root = tum_vi_sequence / "mav0" / sensor;
      std::filesystem::create_directories( sensor_root );
      std::ofstream( sensor_root / "data.csv" );
    }

    const auto base_args = []( const RunPaths&  paths,
                               std::string_view sequence_name ) {
      return "/nonexistent/sequence --out \"" + paths.out.string() +
             "\" --sequence-name " + std::string{ sequence_name } +
             " --force --estimator-enable-moving-bootstrap";
    };

    EXPECT_NE( runBench( base_args( default_euroc, "default_euroc" ),
                         default_euroc.stdout_path,
                         default_euroc.stderr_path ),
               0 );
    EXPECT_NE( runBench( base_args( explicit_euroc, "explicit_euroc" ) +
                             " --dataset-format euroc",
                         explicit_euroc.stdout_path,
                         explicit_euroc.stderr_path ),
               0 );
    EXPECT_NE( runBench( "\"" + tum_vi_sequence.string() + "\" --out \"" +
                             tum_vi.out.string() +
                             "\" --sequence-name tum_vi --force "
                             "--estimator-enable-moving-bootstrap "
                             "--dataset-format tum-vi --gt-tum-vi \"" +
                             tum_vi_sequence.string() + "\"",
                         tum_vi.stdout_path, tum_vi.stderr_path ),
               0 );

    ASSERT_TRUE( std::filesystem::exists( default_euroc.out / "meta.json" ) )
        << readFile( default_euroc.stderr_path );
    ASSERT_TRUE( std::filesystem::exists( explicit_euroc.out / "meta.json" ) )
        << readFile( explicit_euroc.stderr_path );
    ASSERT_TRUE( std::filesystem::exists( tum_vi.out / "meta.json" ) )
        << readFile( tum_vi.stderr_path );
    const auto default_meta =
        nlohmann::json::parse( readFile( default_euroc.out / "meta.json" ) );
    const auto explicit_meta =
        nlohmann::json::parse( readFile( explicit_euroc.out / "meta.json" ) );
    const auto tum_vi_meta =
        nlohmann::json::parse( readFile( tum_vi.out / "meta.json" ) );

    EXPECT_EQ( default_meta.at( "paths" ),
               nlohmann::json( { { "bench_root", "" },
                                 { "output_dir",
                                   default_euroc.out.string() },
                                 { "layout_template", "override" } } ) );
    EXPECT_EQ( default_meta.at( "config_hash" ), "0337287b" );
    EXPECT_EQ( explicit_meta.at( "config_hash" ),
               default_meta.at( "config_hash" ) );
    EXPECT_EQ( explicit_meta.at( "config_canonical_text" ),
               default_meta.at( "config_canonical_text" ) );
    EXPECT_NE( tum_vi_meta.at( "config_hash" ),
               default_meta.at( "config_hash" ) );
    EXPECT_NE( tum_vi_meta.at( "config_canonical_text" )
                   .get<std::string>()
                   .find( "session.dataset_format=tum_vi" ),
               std::string::npos );
    EXPECT_NE( readFile( tum_vi.stderr_path ).find( "dso/camchain.yaml" ),
               std::string::npos );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, RejectsInvalidOrConflictingDatasetOptions )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_dataset_format_reject";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );

    const auto expect_parse_failure =
        [ &root ]( std::string_view name, const std::string& arguments,
                   std::string_view expected_message ) {
          const auto stdout_path = root / ( std::string{ name } + ".stdout" );
          const auto stderr_path = root / ( std::string{ name } + ".stderr" );
          EXPECT_EQ( runBench( arguments, stdout_path, stderr_path ), 2 );
          EXPECT_NE( readFile( stderr_path ).find( expected_message ),
                     std::string::npos );
        };

    expect_parse_failure(
        "invalid", "/nonexistent/sequence --dataset-format inferred",
        "--dataset-format" );
    expect_parse_failure(
        "tum-with-euroc-gt",
        "/nonexistent/sequence --dataset-format tum-vi --gt-euroc "
        "/nonexistent/gt",
        "--gt-euroc" );
    expect_parse_failure(
        "euroc-with-tum-gt",
        "/nonexistent/sequence --dataset-format euroc --gt-tum-vi "
        "/nonexistent/gt",
        "--gt-tum-vi" );
    expect_parse_failure(
        "both-gt",
        "/nonexistent/sequence --dataset-format tum-vi --gt-euroc "
        "/nonexistent/euroc --gt-tum-vi /nonexistent/tum",
        "groundtruth" );

    std::filesystem::remove_all( root );
  }

  TEST( VoBenchCliTest, GyroObserveDoesNotChangeConfigHashOnSessionFailure )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_gyro_observe_hash";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto out_default    = root / "out_default";
    const auto out_observe    = root / "out_observe";
    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_observe = root / "stdout_observe.txt";
    const auto stderr_observe = root / "stderr_observe.txt";

    const int default_exit = runBench(
        "/nonexistent/sequence --out \"" + out_default.string() +
            "\" --sequence-name gyro_observe_default --force",
        stdout_default, stderr_default );
    const int observe_exit = runBench(
        "/nonexistent/sequence --out \"" + out_observe.string() +
            "\" --sequence-name gyro_observe_on --force --gyro-observe",
        stdout_observe, stderr_observe );

    EXPECT_NE( default_exit, 0 );
    EXPECT_NE( observe_exit, 0 );
    const auto default_meta =
        nlohmann::json::parse( readFile( out_default / "meta.json" ) );
    const auto observe_meta =
        nlohmann::json::parse( readFile( out_observe / "meta.json" ) );
    EXPECT_EQ( default_meta.at( "config" ), observe_meta.at( "config" ) );
    EXPECT_EQ( default_meta.at( "config_canonical_text" ),
               observe_meta.at( "config_canonical_text" ) );
    EXPECT_EQ( default_meta.at( "config_hash" ),
               observe_meta.at( "config_hash" ) );
    EXPECT_EQ( extractConfigHash( readFile( stdout_default ) ),
               extractConfigHash( readFile( stdout_observe ) ) );
    const auto default_summary =
        nlohmann::json::parse( readFile( out_default / "summary.json" ) );
    const auto observe_summary =
        nlohmann::json::parse( readFile( out_observe / "summary.json" ) );
    EXPECT_EQ( default_summary.at( "status" ), "failed" );
    EXPECT_EQ( observe_summary.at( "status" ), "failed" );
    EXPECT_FALSE( std::filesystem::exists( out_observe /
                                           "gyro_packets.csv" ) );
    EXPECT_FALSE( std::filesystem::exists( out_observe /
                                           "gyro_samples.csv" ) );
    EXPECT_FALSE( std::filesystem::exists( out_default / "est.tum" ) );
    EXPECT_FALSE( std::filesystem::exists( out_default / "diag.csv" ) );
    EXPECT_FALSE( std::filesystem::exists( out_observe / "est.tum" ) );
    EXPECT_FALSE( std::filesystem::exists( out_observe / "diag.csv" ) );
    const auto default_stderr = readFile( stderr_default );
    const auto observe_stderr = readFile( stderr_observe );
    const auto default_warnings =
        default_summary.at( "warnings" ).get<std::vector<std::string>>();
    const auto observe_warnings =
        observe_summary.at( "warnings" ).get<std::vector<std::string>>();
    EXPECT_NE( default_stderr.find( "dataset error" ), std::string::npos );
    EXPECT_NE( default_stderr.find( "/nonexistent/sequence" ),
               std::string::npos );
    EXPECT_NE( observe_stderr.find( "dataset error" ), std::string::npos );
    EXPECT_NE( observe_stderr.find( "/nonexistent/sequence" ),
               std::string::npos );
    const auto has_dataset_error = []( const std::vector<std::string>& warnings ) {
      return std::any_of(
          warnings.begin(), warnings.end(), []( const std::string& warning ) {
            return warning.find( "dataset error" ) != std::string::npos &&
                   warning.find( "/nonexistent/sequence" ) !=
                       std::string::npos;
          } );
    };
    EXPECT_TRUE( has_dataset_error( default_warnings ) );
    EXPECT_TRUE( has_dataset_error( observe_warnings ) );

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

  TEST( VoBenchCliTest,
        PnpCheiralityRetryGateChangesConfigIdentityOnlyWhenEnabled )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_pnp_cheirality_retry_hash";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto out_default    = root / "out_default";
    const auto out_retry      = root / "out_retry";
    const auto stdout_default = root / "stdout_default.txt";
    const auto stderr_default = root / "stderr_default.txt";
    const auto stdout_retry   = root / "stdout_retry.txt";
    const auto stderr_retry   = root / "stderr_retry.txt";

    const int default_exit = runBench(
        "/nonexistent/sequence --out \"" + out_default.string() +
            "\" --sequence-name pnp_retry_default --force "
            "--estimator-enable-moving-bootstrap",
        stdout_default, stderr_default );
    const int retry_exit = runBench(
        "/nonexistent/sequence --out \"" + out_retry.string() +
            "\" --sequence-name pnp_retry_enabled --force "
            "--estimator-enable-moving-bootstrap "
            "--estimator-enable-pnp-cheirality-retry",
        stdout_retry, stderr_retry );

    EXPECT_NE( default_exit, 0 );
    ASSERT_NE( retry_exit, 2 ) << readFile( stderr_retry );
    EXPECT_NE( retry_exit, 0 );
    ASSERT_TRUE( std::filesystem::exists( out_default / "meta.json" ) )
        << readFile( stderr_default );
    ASSERT_TRUE( std::filesystem::exists( out_retry / "meta.json" ) )
        << readFile( stderr_retry );

    const auto default_meta =
        nlohmann::json::parse( readFile( out_default / "meta.json" ) );
    const auto retry_meta =
        nlohmann::json::parse( readFile( out_retry / "meta.json" ) );
    const std::string default_hash =
        default_meta.at( "config_hash" ).get<std::string>();
    const std::string retry_hash =
        retry_meta.at( "config_hash" ).get<std::string>();
    const std::string default_config =
        default_meta.at( "config_canonical_text" ).get<std::string>();
    const std::string retry_config =
        retry_meta.at( "config_canonical_text" ).get<std::string>();

    constexpr std::string_view kRetryKey =
        "estimator.enable_pnp_cheirality_retry=true";
    EXPECT_EQ( default_hash, "0337287b" );
    EXPECT_EQ( extractConfigHash( readFile( stdout_default ) ), default_hash );
    EXPECT_EQ( default_config.find( kRetryKey ), std::string::npos );
    ASSERT_FALSE( retry_hash.empty() );
    EXPECT_NE( retry_hash, default_hash );
    EXPECT_EQ( extractConfigHash( readFile( stdout_retry ) ), retry_hash );
    const auto key_position = retry_config.find( kRetryKey );
    ASSERT_NE( key_position, std::string::npos );
    EXPECT_EQ( retry_config.find( kRetryKey, key_position + kRetryKey.size() ),
               std::string::npos );

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

  TEST( VoBenchCliTest, RejectsRetiredAccumulatedSeedFlag )
  {
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vo_bench_cli_retired_seed";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        "/nonexistent/sequence --estimator-enable-accumulated-seed",
        stdout_path, stderr_path );

    EXPECT_EQ( exit_code, 2 );
    const std::string err = readFile( stderr_path );
    EXPECT_NE(
        err.find( "unknown option: --estimator-enable-accumulated-seed" ),
        std::string::npos );
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

}  // namespace

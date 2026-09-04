#include <gtest/gtest.h>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "tests/support/scoped_temp_directory.hpp"

#ifdef __linux__
#include <sys/wait.h>
#endif

namespace
{

  TEST( M551ReplayDiagnosticInfrastructureTest,
        ScopedTemporaryDirectoriesAreUniqueAndCleaned )
  {
    std::filesystem::path first_path;
    std::filesystem::path second_path;
    {
      phad::testing::ScopedTempDirectory first{ "m5-51-diagnostic" };
      phad::testing::ScopedTempDirectory second{ "m5-51-diagnostic" };
      first_path  = first.path();
      second_path = second.path();

      EXPECT_NE( first_path, second_path );
      EXPECT_TRUE( std::filesystem::is_directory( first_path ) );
      EXPECT_TRUE( std::filesystem::is_directory( second_path ) );
    }

    EXPECT_FALSE( std::filesystem::exists( first_path ) );
    EXPECT_FALSE( std::filesystem::exists( second_path ) );
  }

#ifndef PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH
#define PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH ""
#endif

#ifndef PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH
#define PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH ""
#endif

#ifndef PHAD_SOURCE_DIR
#define PHAD_SOURCE_DIR ""
#endif

#if !defined( PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH ) ||    \
    !defined( PHAD_M5_51_EXPECTED_CXX_COMPILER_VERSION ) || \
    !defined( PHAD_M5_51_EXPECTED_CMAKE_PATH ) ||           \
    !defined( PHAD_M5_51_EXPECTED_CMAKE_VERSION ) ||        \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH ) ||      \
    !defined( PHAD_M5_51_EXPECTED_GENERATOR ) ||            \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TYPE ) ||           \
    !defined( PHAD_M5_51_EXPECTED_BUILD_TESTS ) ||          \
    !defined( PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH )
#error "M5 #51 diagnostic test requires trusted build identity definitions"
#endif

  constexpr std::string_view kFormalConfigSha =
      "246033bd867ec01acad1ad8ba1ae5dab24f9c79321af862e19dc1660d99b24a7";

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream      input( path, std::ios::binary );
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
  }

  [[nodiscard]] int runExecutable(
      const std::filesystem::path& executable,
      const std::string&           args,
      const std::filesystem::path& stdout_path,
      const std::filesystem::path& stderr_path )
  {
    const std::string command =
        "OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \"" +
        executable.string() + "\" " + args + " > \"" +
        stdout_path.string() + "\" 2> \"" +
        stderr_path.string() + "\"";
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

  [[nodiscard]] int runDiagnostic(
      const std::string&           args,
      const std::filesystem::path& stdout_path,
      const std::filesystem::path& stderr_path )
  {
    return runExecutable( PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH, args,
                          stdout_path, stderr_path );
  }

  [[nodiscard]] std::string sha256( const std::string_view bytes )
  {
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    std::size_t                                digest_size = digest.size();
    if ( EVP_Q_digest( nullptr, "SHA256", nullptr, bytes.data(),
                       bytes.size(), digest.data(), &digest_size ) != 1 ||
         digest_size != 32U )
    {
      throw std::runtime_error( "cannot compute test SHA-256" );
    }

    constexpr char kHex[] = "0123456789abcdef";
    std::string    encoded;
    encoded.reserve( digest_size * 2U );
    for ( std::size_t index = 0U; index < digest_size; ++index )
    {
      const unsigned char value = digest[ index ];
      encoded.push_back( kHex[ value >> 4U ] );
      encoded.push_back( kHex[ value & 0x0fU ] );
    }
    return encoded;
  }

  void writeFile( const std::filesystem::path& path,
                  const std::string_view       bytes )
  {
    std::ofstream output( path, std::ios::binary );
    ASSERT_TRUE( output ) << path;
    output << bytes;
    ASSERT_TRUE( output ) << path;
  }

  [[nodiscard]] std::string fileSha256(
      const std::filesystem::path& path )
  {
    return sha256( readFile( path ) );
  }

  [[nodiscard]] std::string repeatedHex( const char value )
  {
    return std::string( 64U, value );
  }

  [[nodiscard]] std::string runtimeLibraryManifest(
      const std::filesystem::path& root,
      const std::filesystem::path& executable )
  {
    const auto        output_path = root / "ldd.txt";
    const std::string command     = "ldd \"" + executable.string() +
                                "\" > \"" + output_path.string() + "\"";
    if ( std::system( command.c_str() ) != 0 )
    {
      throw std::runtime_error( "cannot inspect diagnostic runtime libraries" );
    }

    std::map<std::string, std::string> libraries;
    std::istringstream                 lines{ readFile( output_path ) };
    std::string                        line;
    while ( std::getline( lines, line ) )
    {
      const std::size_t first = line.find_first_not_of( " \t" );
      if ( first == std::string::npos ||
           line.compare( first, 10U, "linux-vdso" ) == 0 )
      {
        continue;
      }

      std::size_t path_begin = first;
      const auto  arrow      = line.find( "=>", first );
      if ( arrow != std::string::npos )
      {
        path_begin = line.find_first_not_of( " \t", arrow + 2U );
      }
      if ( path_begin == std::string::npos || line[ path_begin ] != '/' )
      {
        throw std::runtime_error( "ldd did not resolve a runtime library" );
      }
      const std::size_t           path_end = line.find_first_of( " \t", path_begin );
      const std::filesystem::path path     = std::filesystem::canonical(
          line.substr( path_begin, path_end - path_begin ) );
      libraries.emplace( path.string(), fileSha256( path ) );
    }
    std::filesystem::remove( output_path );

    std::string manifest;
    for ( const auto& [ path, digest ] : libraries )
    {
      manifest += digest + "  " + path + '\n';
    }
    return manifest;
  }

  [[nodiscard]] nlohmann::json qualificationTemplate(
      const std::filesystem::path& root )
  {
    const std::filesystem::path source_root{ PHAD_SOURCE_DIR };
    const std::filesystem::path executable{
        PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH };
    const std::string executable_sha           = fileSha256( executable );
    const auto        source_manifest_path     = root / "source-manifest.json";
    const auto        source_archive_path      = root / "source.tar.zst";
    const auto        base_bundle_path         = root / "base.bundle";
    const auto        source_verification_path = root / "source-verification.json";
    const auto        input_manifest_path      = root / "input.sha256";
    const auto        dynamic_libraries_path   = root / "dynamic-libraries.sha256";
    const std::string dynamic_libraries_bytes =
        runtimeLibraryManifest( root, executable );
    writeFile( dynamic_libraries_path, dynamic_libraries_bytes );

    nlohmann::json qualification{
        { "schema", "phad.m5_51.baseline_qualification.v1" },
        { "status", "qualified" },
        { "source_id", "sha256:" + repeatedHex( '1' ) },
        { "source_manifest_path", source_manifest_path.string() },
        { "source_manifest_sha256", repeatedHex( '2' ) },
        { "source_archive_path", source_archive_path.string() },
        { "source_archive_sha256", repeatedHex( '3' ) },
        { "base_bundle_path", base_bundle_path.string() },
        { "base_bundle_sha256", repeatedHex( '4' ) },
        { "source_verification_receipt_path",
          source_verification_path.string() },
        { "source_verification_receipt_sha256", repeatedHex( '5' ) },
        { "binary_sha256", executable_sha },
        { "config_hash", "0337287b" },
        { "config_canonical_text_sha256", kFormalConfigSha },
        { "input_manifest_path", input_manifest_path.string() },
        { "input_manifest_sha256", repeatedHex( '6' ) },
        { "input_entry_count", 7376U },
        { "rpe_evaluator_sha256",
          fileSha256( source_root / "phad/eval/rpe.cpp" ) } };

    const auto           preflight_path = root / "preflight.json";
    const nlohmann::json preflight{
        { "schema", "phad.m5_51.preflight_identity.v1" },
        { "source",
          { { "source_id", qualification.at( "source_id" ) },
            { "manifest_path", source_manifest_path.string() },
            { "manifest_sha256",
              qualification.at( "source_manifest_sha256" ) },
            { "archive_path", source_archive_path.string() },
            { "archive_sha256",
              qualification.at( "source_archive_sha256" ) },
            { "base_bundle_path", base_bundle_path.string() },
            { "base_bundle_sha256",
              qualification.at( "base_bundle_sha256" ) },
            { "verification_receipt_path",
              source_verification_path.string() },
            { "verification_receipt_sha256",
              qualification.at( "source_verification_receipt_sha256" ) },
            { "entry_count", 1U },
            { "deleted_tracked_count", 0U },
            { "git_head", repeatedHex( '7' ) },
            { "git_tree", std::string( 40U, '8' ) } } },
        { "binaries",
          { { "phad_vo_bench",
              { { "path", executable.string() },
                { "sha256", executable_sha } } },
            { "phad_traj_eval",
              { { "path", PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH ) } } } } },
        { "build",
          { { "type", PHAD_M5_51_EXPECTED_BUILD_TYPE },
            { "generator", PHAD_M5_51_EXPECTED_GENERATOR },
            { "phad_build_tests",
              static_cast<bool>( PHAD_M5_51_EXPECTED_BUILD_TESTS ) },
            { "compiler",
              { { "path", PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH },
                { "version", PHAD_M5_51_EXPECTED_CXX_COMPILER_VERSION },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH ) } } },
            { "cmake",
              { { "path", PHAD_M5_51_EXPECTED_CMAKE_PATH },
                { "version", PHAD_M5_51_EXPECTED_CMAKE_VERSION },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_CMAKE_PATH ) } } },
            { "ninja",
              { { "path", PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH },
                { "version", "test" },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH ) } } } } },
        { "configuration",
          { { "label", "default" },
            { "canonical_hash", "0337287b" },
            { "canonical_text_sha256", kFormalConfigSha },
            { "moving_bootstrap", true },
            { "pnp_cheirality_retry", false },
            { "collect_timing", false },
            { "execution_bound_max_frames", 43U },
            { "reference_meta_path", executable.string() },
            { "reference_meta_sha256", executable_sha } } },
        { "input",
          { { "root", "/nonexistent/sequence" },
            { "sequence", "MH_01_easy" },
            { "entry_count", 7376U },
            { "verified_entry_count", 7376U },
            { "ordered_manifest_path", input_manifest_path.string() },
            { "ordered_manifest_sha256",
              qualification.at( "input_manifest_sha256" ) } } },
        { "evaluator",
          { { "bench_source_sha256",
              fileSha256( source_root / "apps/phad_vo_bench.cpp" ) },
            { "rpe_cpp_sha256",
              qualification.at( "rpe_evaluator_sha256" ) },
            { "rpe_hpp_sha256",
              fileSha256( source_root / "phad/eval/rpe.hpp" ) },
            { "max_dt_ms", 2.5 },
            { "min_match_rate", 0.5 },
            { "rpe_delta_s", 1.0 } } },
        { "runtime",
          { { "determinism_environment",
              { { "MKL_NUM_THREADS", "1" },
                { "OMP_NUM_THREADS", "1" },
                { "OPENBLAS_NUM_THREADS", "1" } } },
            { "dynamic_libraries_path", dynamic_libraries_path.string() },
            { "dynamic_libraries_sha256",
              sha256( dynamic_libraries_bytes ) },
            { "dynamic_library_count",
              static_cast<std::size_t>( std::count(
                  dynamic_libraries_bytes.begin(),
                  dynamic_libraries_bytes.end(), '\n' ) ) },
            { "gtsam_version", "test" },
            { "opencv_version", "test" },
            { "openssl_version", "test" } } } };
    const std::string preflight_bytes = preflight.dump( 2 ) + '\n';
    writeFile( preflight_path, preflight_bytes );

    qualification[ "preflight_identity_path" ]   = preflight_path.string();
    qualification[ "preflight_identity_sha256" ] = sha256( preflight_bytes );
    return qualification;
  }

  void rewritePreflight( nlohmann::json&       qualification,
                         const nlohmann::json& preflight )
  {
    const std::filesystem::path path =
        qualification.at( "preflight_identity_path" ).get<std::string>();
    const std::string bytes = preflight.dump( 2 ) + '\n';
    writeFile( path, bytes );
    qualification[ "preflight_identity_sha256" ] = sha256( bytes );
  }

  struct QualificationRun
  {
    int                   exit_code = -1;
    std::filesystem::path output_path;
    std::filesystem::path state_path;
    std::filesystem::path comparison_path;
    std::string           stderr_text;
  };

  [[nodiscard]] QualificationRun runQualification(
      const std::filesystem::path& root, std::string_view name,
      const nlohmann::json& qualification )
  {
    const auto qualification_path = root / ( std::string{ name } + ".json" );
    const auto output_path        = root / ( std::string{ name } + ".out" );
    const auto state_path         = root / ( std::string{ name } + ".state.json" );
    const auto comparison_path =
        root / ( std::string{ name } + ".comparison.json" );
    const auto stdout_path = root / ( std::string{ name } + ".stdout" );
    const auto stderr_path = root / ( std::string{ name } + ".stderr" );
    writeFile( qualification_path, qualification.dump( 2 ) + '\n' );

    const std::string args =
        "/nonexistent/sequence --out \"" + output_path.string() +
        "\" --sequence-name MH_01_easy --config-label default "
        "--gt-euroc /nonexistent/sequence --repo \"" +
        std::string{ PHAD_SOURCE_DIR } +
        "\" --max-dt-ms 2.5 --min-match-rate 0.5 --rpe-delta-s 1.0 "
        "--max-frames 43 --estimator-enable-moving-bootstrap "
        "--m5-51-state-receipt \"" +
        state_path.string() +
        "\" --m5-51-selector-warmup --m5-51-baseline-qualification \"" +
        qualification_path.string() +
        "\" --m5-51-comparison-receipt \"" + comparison_path.string() +
        "\"";
    const int exit_code =
        runDiagnostic( args, stdout_path, stderr_path );
    return QualificationRun{ .exit_code       = exit_code,
                             .output_path     = output_path,
                             .state_path      = state_path,
                             .comparison_path = comparison_path,
                             .stderr_text     = readFile( stderr_path ) };
  }

  TEST( M551ReplayDiagnosticCliTest,
        EnforcesFormalConfigAndBaselineGate )
  {
    ASSERT_FALSE(
        std::string_view{ PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH }.empty() );

    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_replay_diagnostic_gate" };
    const auto& root = temp_directory.path();

    const auto run = [ &root ]( std::string_view   name,
                                const std::string& suffix ) {
      const auto        stdout_path  = root / ( std::string{ name } + ".stdout" );
      const auto        stderr_path  = root / ( std::string{ name } + ".stderr" );
      const auto        output_path  = root / ( std::string{ name } + ".out" );
      const auto        receipt_path = root / ( std::string{ name } + ".json" );
      const std::string base =
          "/nonexistent/sequence --out \"" + output_path.string() +
          "\" --sequence-name MH_01_easy --config-label default "
          "--gt-euroc /nonexistent/sequence --repo \"" +
          std::filesystem::current_path().string() +
          "\" --max-dt-ms 2.5 --min-match-rate 0.5 "
          "--rpe-delta-s 1.0 --estimator-enable-moving-bootstrap "
          "--m5-51-state-receipt \"" +
          receipt_path.string() + "\" ";
      const int exit_code =
          runDiagnostic( base + suffix, stdout_path, stderr_path );
      return std::tuple{ exit_code, output_path, stderr_path };
    };

    const auto [ bounded_exit, bounded_output, bounded_stderr ] =
        run( "bounded", "--max-frames 43" );
    EXPECT_NE( bounded_exit, 0 );
    ASSERT_TRUE( std::filesystem::exists( bounded_output / "meta.json" ) )
        << readFile( bounded_stderr );
    const auto bounded_meta = nlohmann::json::parse(
        readFile( bounded_output / "meta.json" ) );
    EXPECT_EQ( bounded_meta.at( "config_hash" ), "0337287b" );
    EXPECT_EQ( bounded_meta.at( "config_canonical_text" )
                   .get<std::string>()
                   .find( "session.max_frames" ),
               std::string::npos );

    const auto [ missing_bound_exit, missing_bound_output,
                 missing_bound_stderr ] = run( "missing-bound", "" );
    static_cast<void>( missing_bound_output );
    EXPECT_EQ( missing_bound_exit, 2 );
    EXPECT_NE( readFile( missing_bound_stderr ).find( "--max-frames 43" ),
               std::string::npos );

    const auto [ direct_exit, direct_output, direct_stderr ] =
        run( "direct-intervention",
             "--max-frames 43 --m5-51-selector-warmup" );
    static_cast<void>( direct_output );
    EXPECT_EQ( direct_exit, 2 );
    EXPECT_NE(
        readFile( direct_stderr ).find( "--m5-51-baseline-qualification" ),
        std::string::npos );

    const auto [ invalid_qualification_exit, invalid_qualification_output,
                 invalid_qualification_stderr ] =
        run( "invalid-qualification",
             "--max-frames 43 --m5-51-selector-warmup "
             "--m5-51-baseline-qualification /nonexistent/qualification.json "
             "--m5-51-comparison-receipt /nonexistent/comparison.json" );
    static_cast<void>( invalid_qualification_output );
    EXPECT_EQ( invalid_qualification_exit, 2 );
    EXPECT_NE( readFile( invalid_qualification_stderr )
                   .find( "baseline qualification invalid" ),
               std::string::npos );

    const auto [ retry_exit, retry_output, retry_stderr ] =
        run( "pnp-retry",
             "--max-frames 43 --estimator-enable-pnp-cheirality-retry" );
    static_cast<void>( retry_output );
    EXPECT_EQ( retry_exit, 2 );
    EXPECT_NE( readFile( retry_stderr ).find( "retry" ),
               std::string::npos );
  }

  TEST( M551ReplayDiagnosticCliTest,
        RejectsMissingOrTamperedPreflightReceipt )
  {
    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_preflight_receipt_gate" };
    const auto& root = temp_directory.path();

    auto missing = qualificationTemplate( root );
    missing.erase( "preflight_identity_path" );
    const QualificationRun missing_run =
        runQualification( root, "missing-preflight", missing );
    EXPECT_EQ( missing_run.exit_code, 2 );
    EXPECT_NE( missing_run.stderr_text.find( "preflight identity" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( missing_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( missing_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( missing_run.comparison_path ) );

    auto unreadable = qualificationTemplate( root );
    unreadable[ "preflight_identity_path" ] =
        ( root / "absent-preflight-receipt.json" ).string();
    const QualificationRun unreadable_run =
        runQualification( root, "unreadable-preflight", unreadable );
    EXPECT_EQ( unreadable_run.exit_code, 2 );
    EXPECT_NE( unreadable_run.stderr_text.find( "preflight identity unreadable" ),
               std::string::npos )
        << unreadable_run.stderr_text;
    EXPECT_FALSE( std::filesystem::exists( unreadable_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( unreadable_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( unreadable_run.comparison_path ) );

    auto tampered = qualificationTemplate( root );
    writeFile( tampered.at( "preflight_identity_path" ).get<std::string>(),
               "{}\n" );
    const QualificationRun tampered_run =
        runQualification( root, "tampered-preflight", tampered );
    EXPECT_EQ( tampered_run.exit_code, 2 );
    EXPECT_NE( tampered_run.stderr_text.find( "preflight identity digest" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( tampered_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( tampered_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( tampered_run.comparison_path ) );

    auto missing_sha = qualificationTemplate( root );
    missing_sha.erase( "preflight_identity_sha256" );
    const QualificationRun missing_sha_run =
        runQualification( root, "missing-preflight-sha", missing_sha );
    EXPECT_EQ( missing_sha_run.exit_code, 2 );
    EXPECT_NE( missing_sha_run.stderr_text.find( "preflight identity" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( missing_sha_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( missing_sha_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( missing_sha_run.comparison_path ) );
  }

  TEST( M551ReplayDiagnosticCliTest,
        RejectsDifferentCanonicalConfigWithSameShortHash )
  {
    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_full_config_identity_gate" };
    const auto& root = temp_directory.path();

    auto              qualification = qualificationTemplate( root );
    const std::string different_config_sha =
        sha256( "different canonical config\n" );
    qualification[ "config_canonical_text_sha256" ] = different_config_sha;
    auto preflight                                  = nlohmann::json::parse( readFile(
        qualification.at( "preflight_identity_path" ).get<std::string>() ) );
    preflight[ "configuration" ][ "canonical_text_sha256" ] =
        different_config_sha;
    rewritePreflight( qualification, preflight );
    ASSERT_EQ( qualification.at( "config_hash" ), "0337287b" );

    const QualificationRun run =
        runQualification( root, "same-short-hash", qualification );
    EXPECT_EQ( run.exit_code, 2 );
    EXPECT_NE( run.stderr_text.find( "canonical config SHA-256" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( run.comparison_path ) );
  }

  TEST( M551ReplayDiagnosticCliTest,
        RejectsCrossReceiptIdentityMismatch )
  {
    struct IdentityCase
    {
      std::string                  name;
      nlohmann::json::json_pointer field;
      std::string                  expected_error;
    };
    const std::vector<IdentityCase> cases{
        { "source", nlohmann::json::json_pointer{ "/source/source_id" },
          "preflight source identity" },
        { "binary",
          nlohmann::json::json_pointer{
              "/binaries/phad_vo_bench/sha256" },
          "preflight binary identity" },
        { "input",
          nlohmann::json::json_pointer{
              "/input/ordered_manifest_sha256" },
          "preflight input identity" },
        { "evaluator",
          nlohmann::json::json_pointer{ "/evaluator/rpe_cpp_sha256" },
          "preflight evaluator identity" } };

    for ( const auto& identity_case : cases )
    {
      phad::testing::ScopedTempDirectory temp_directory{
          "phad_m5_51_cross_identity_" + identity_case.name };
      const auto& root                 = temp_directory.path();
      auto        qualification        = qualificationTemplate( root );
      auto        preflight            = nlohmann::json::parse( readFile(
          qualification.at( "preflight_identity_path" ).get<std::string>() ) );
      preflight[ identity_case.field ] = repeatedHex( '0' );
      rewritePreflight( qualification, preflight );

      const QualificationRun run =
          runQualification( root, identity_case.name, qualification );
      EXPECT_EQ( run.exit_code, 2 ) << identity_case.name;
      EXPECT_NE( run.stderr_text.find( identity_case.expected_error ),
                 std::string::npos )
          << identity_case.name << ": " << run.stderr_text;
      EXPECT_FALSE( std::filesystem::exists( run.output_path ) );
      EXPECT_FALSE( std::filesystem::exists( run.state_path ) );
      EXPECT_FALSE( std::filesystem::exists( run.comparison_path ) );
    }
  }

  TEST( M551ReplayDiagnosticCliTest,
        RejectsToolchainAndRuntimeLibraryIdentityMismatch )
  {
    phad::testing::ScopedTempDirectory evaluator_directory{
        "phad_m5_51_evaluator_binary_identity" };
    const auto&                 evaluator_root          = evaluator_directory.path();
    auto                        evaluator_qualification = qualificationTemplate( evaluator_root );
    auto                        evaluator_preflight     = nlohmann::json::parse( readFile(
        evaluator_qualification.at( "preflight_identity_path" )
            .get<std::string>() ) );
    const std::filesystem::path diagnostic_executable{
        PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH };
    evaluator_preflight[ "binaries" ][ "phad_traj_eval" ][ "path" ] =
        diagnostic_executable.string();
    evaluator_preflight[ "binaries" ][ "phad_traj_eval" ][ "sha256" ] =
        fileSha256( diagnostic_executable );
    rewritePreflight( evaluator_qualification, evaluator_preflight );
    const QualificationRun evaluator_run = runQualification(
        evaluator_root, "evaluator", evaluator_qualification );
    EXPECT_EQ( evaluator_run.exit_code, 2 );
    EXPECT_NE( evaluator_run.stderr_text.find( "preflight evaluator identity" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( evaluator_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( evaluator_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( evaluator_run.comparison_path ) );

    phad::testing::ScopedTempDirectory toolchain_directory{
        "phad_m5_51_toolchain_identity" };
    const auto& toolchain_root          = toolchain_directory.path();
    auto        toolchain_qualification = qualificationTemplate( toolchain_root );
    auto        toolchain_preflight     = nlohmann::json::parse( readFile(
        toolchain_qualification.at( "preflight_identity_path" )
            .get<std::string>() ) );
    toolchain_preflight[ "build" ][ "compiler" ][ "path" ] =
        diagnostic_executable.string();
    toolchain_preflight[ "build" ][ "compiler" ][ "sha256" ] =
        fileSha256( diagnostic_executable );
    rewritePreflight( toolchain_qualification, toolchain_preflight );
    const QualificationRun toolchain_run = runQualification(
        toolchain_root, "toolchain", toolchain_qualification );
    EXPECT_EQ( toolchain_run.exit_code, 2 );
    EXPECT_NE( toolchain_run.stderr_text.find( "preflight compiler identity" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( toolchain_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( toolchain_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( toolchain_run.comparison_path ) );

    phad::testing::ScopedTempDirectory runtime_directory{
        "phad_m5_51_runtime_identity" };
    const auto&                 runtime_root          = runtime_directory.path();
    auto                        runtime_qualification = qualificationTemplate( runtime_root );
    auto                        runtime_preflight     = nlohmann::json::parse( readFile(
        runtime_qualification.at( "preflight_identity_path" )
            .get<std::string>() ) );
    const std::filesystem::path manifest_path =
        runtime_preflight.at( "runtime" )
            .at( "dynamic_libraries_path" )
            .get<std::string>();
    const std::string manifest_bytes = fileSha256( diagnostic_executable ) +
                                       "  " +
                                       diagnostic_executable.string() + '\n';
    writeFile( manifest_path, manifest_bytes );
    runtime_preflight[ "runtime" ][ "dynamic_libraries_sha256" ] =
        sha256( manifest_bytes );
    runtime_preflight[ "runtime" ][ "dynamic_library_count" ] = 1U;
    rewritePreflight( runtime_qualification, runtime_preflight );
    const QualificationRun runtime_run =
        runQualification( runtime_root, "runtime", runtime_qualification );
    EXPECT_EQ( runtime_run.exit_code, 2 );
    EXPECT_NE(
        runtime_run.stderr_text.find( "preflight runtime library identity" ),
        std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( runtime_run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( runtime_run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( runtime_run.comparison_path ) );
  }

  TEST( M551ReplayDiagnosticCliTest,
        AcceptsBoundPreflightIdentityUntilSourceManifestGate )
  {
    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_bound_preflight_identity" };
    const auto& root = temp_directory.path();

    const QualificationRun run = runQualification(
        root, "bound-preflight", qualificationTemplate( root ) );
    EXPECT_EQ( run.exit_code, 2 );
    EXPECT_NE( run.stderr_text.find( "source-manifest.json" ),
               std::string::npos )
        << run.stderr_text;
    EXPECT_FALSE( std::filesystem::exists( run.output_path ) );
    EXPECT_FALSE( std::filesystem::exists( run.state_path ) );
    EXPECT_FALSE( std::filesystem::exists( run.comparison_path ) );
  }

  TEST( M551ReplayDiagnosticCliTest,
        CliFixtureRequiresSelectorIntervention )
  {
    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_cli_fixture_arm_gate" };
    const auto& root = temp_directory.path();

    const auto output_path = root / "output";
    const auto state_path  = root / "state.json";
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";
    const int  exit_code   = runExecutable(
        PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH,
        "/nonexistent/sequence --out \"" + output_path.string() +
            "\" --max-frames 43 --estimator-enable-moving-bootstrap "
               "--m5-51-state-receipt \"" +
            state_path.string() + "\"",
        stdout_path, stderr_path );

    EXPECT_EQ( exit_code, 2 );
    EXPECT_NE( readFile( stderr_path )
                   .find( "CLI fixture requires --m5-51-selector-warmup" ),
               std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( output_path ) );
    EXPECT_FALSE( std::filesystem::exists( state_path ) );
  }

  TEST( M551ReplayDiagnosticCliTest,
        WritesBoundStateAndComparisonReceiptsThroughCliFixture )
  {
    ASSERT_FALSE( std::string_view{
        PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH }
                      .empty() );

    phad::testing::ScopedTempDirectory temp_directory{
        "phad_m5_51_positive_cli_receipt_chain" };
    const auto& root = temp_directory.path();

    const auto canonical_state = []( const bool          i7_is_keyframe,
                                     const std::uint64_t total_keyframes ) {
      nlohmann::ordered_json state;
      state[ "schema" ]                                 = "phad.m5_51.future_runtime_state.v1";
      state[ "keyframe_selector" ][ "total_keyframes" ] = total_keyframes;
      nlohmann::ordered_json diag                       = nlohmann::ordered_json::array();
      for ( std::uint64_t index = 0U; index < 43U; ++index )
      {
        diag.push_back( { { "frame_index", index },
                          { "is_keyframe",
                            index == 7U ? i7_is_keyframe : false } } );
      }
      state[ "session" ][ "result" ][ "diag" ] = std::move( diag );
      return state;
    };
    const auto wrapped_state = []( const nlohmann::ordered_json& state ) {
      return nlohmann::ordered_json{
          { "sha256", sha256( state.dump() ) },
          { "canonical_json", state } };
    };

    const auto             baseline_pre = wrapped_state( canonical_state( true, 1U ) );
    nlohmann::ordered_json baseline_receipt;
    baseline_receipt[ "schema" ] =
        "m5_51_prefix_diagnostic_receipt_v1";
    baseline_receipt[ "hook_executed" ]      = true;
    baseline_receipt[ "hook_frame_index" ]   = 6U;
    baseline_receipt[ "pre_hook_state_v1" ]  = baseline_pre;
    baseline_receipt[ "post_hook_state_v1" ] = baseline_pre;
    baseline_receipt[ "post_state_v1" ]      = baseline_pre;
    baseline_receipt[ "hook_diff" ]          = nlohmann::json::array();
    baseline_receipt[ "failure_reason" ]     = "";
    const std::string baseline_bytes         = baseline_receipt.dump();
    const auto        baseline_path          = root / "baseline-state.json";
    writeFile( baseline_path, baseline_bytes );

    const auto source_root  = root / "source";
    const auto bench_source = source_root / "apps/phad_vo_bench.cpp";
    const auto rpe_cpp      = source_root / "phad/eval/rpe.cpp";
    const auto rpe_hpp      = source_root / "phad/eval/rpe.hpp";
    std::filesystem::create_directories( bench_source.parent_path() );
    std::filesystem::create_directories( rpe_cpp.parent_path() );
    writeFile( bench_source, "synthetic bench source\n" );
    writeFile( rpe_cpp, "synthetic RPE implementation\n" );
    writeFile( rpe_hpp, "synthetic RPE contract\n" );
    for ( const auto& path : { bench_source, rpe_cpp, rpe_hpp } )
    {
      std::filesystem::permissions(
          path,
          std::filesystem::perms::owner_read |
              std::filesystem::perms::owner_write |
              std::filesystem::perms::group_read |
              std::filesystem::perms::others_read,
          std::filesystem::perm_options::replace );
    }

    nlohmann::json source_manifest{
        { "deleted_tracked", nlohmann::json::array() },
        { "entries", nlohmann::json::array() },
        { "git",
          { { "branch", "synthetic-fixture" },
            { "head", std::string( 40U, 'a' ) },
            { "tree", std::string( 40U, 'b' ) } } },
        { "schema_version", 1 } };
    for ( const auto& [ relative, path ] :
          std::array<std::pair<std::string, std::filesystem::path>, 3>{
              std::pair{ "apps/phad_vo_bench.cpp", bench_source },
              std::pair{ "phad/eval/rpe.cpp", rpe_cpp },
              std::pair{ "phad/eval/rpe.hpp", rpe_hpp } } )
    {
      source_manifest[ "entries" ].push_back(
          { { "mode", "0644" },
            { "path", relative },
            { "sha256", fileSha256( path ) },
            { "size", std::filesystem::file_size( path ) },
            { "type", "file" } } );
    }
    const std::string source_id =
        "sha256:" + sha256( source_manifest.dump() );
    source_manifest[ "source_id" ]          = source_id;
    const auto        source_manifest_path  = root / "source-manifest.json";
    const std::string source_manifest_bytes = source_manifest.dump( 2 ) + '\n';
    writeFile( source_manifest_path, source_manifest_bytes );

    const auto source_archive_path = root / "source.tar.zst";
    const auto base_bundle_path    = root / "base.bundle";
    writeFile( source_archive_path, "synthetic source archive\n" );
    writeFile( base_bundle_path, "synthetic base bundle\n" );

    const auto sequence_root = root / "sequence";
    std::filesystem::create_directories( sequence_root / "cam0" );
    std::filesystem::create_directories( sequence_root / "imu0" );
    const auto camera_input = sequence_root / "cam0/data.bin";
    const auto imu_input    = sequence_root / "imu0/data.bin";
    writeFile( camera_input, "fixture-camera\n" );
    writeFile( imu_input, "fixture-imu\n" );
    const std::string input_manifest_bytes =
        fileSha256( camera_input ) + "  ./cam0/data.bin\n" +
        fileSha256( imu_input ) + "  ./imu0/data.bin\n";
    const auto input_manifest_path = root / "input.sha256";
    writeFile( input_manifest_path, input_manifest_bytes );
    ASSERT_EQ( sha256( input_manifest_bytes ),
               "2b982fd1a2c814d077e0ee4361ca45d8b56ca23aabc65c5c14509d40f4763ea4" );

    const auto baseline_artifact_dir = root / "baseline";
    std::filesystem::create_directories( baseline_artifact_dir );
    std::string diag_bytes = "timestamp,status,fixture\n";
    for ( std::uint64_t index = 0U; index < 43U; ++index )
    {
      diag_bytes += std::to_string( index ) + ',' +
                    ( index < 6U ? "rejected" : "ok" ) + ',' +
                    std::to_string( index ) + '\n';
    }
    constexpr std::string_view kEstBytes =
        "0 0 0 0 0 0 0 1\n1000000000 1 0 0 0 0 0 1\n";
    constexpr std::string_view kKeyframeBytes = "0 0 0 0 0 0 0 1\n";
    writeFile( baseline_artifact_dir / "diag.csv", diag_bytes );
    writeFile( baseline_artifact_dir / "est.tum", kEstBytes );
    writeFile( baseline_artifact_dir / "kf.tum", kKeyframeBytes );
    ASSERT_EQ( sha256( diag_bytes ),
               "ef783a731d0946936e0640065a215138471b58d81f0b55d156ed61228331674d" );
    ASSERT_EQ( sha256( kEstBytes ),
               "ff390472ab5a66e5d4db829511c305d4155110cb8516e0a32bf4c191770527fe" );
    ASSERT_EQ( sha256( kKeyframeBytes ),
               "cfa32b9311b7ca9f4e56b1fdc88d32287e0d655d73cb34adcddf7bac4bb195aa" );
    const nlohmann::json baseline_meta{
        { "config_hash", "0337287b" },
        { "sequence_root", sequence_root.string() } };
    const std::string baseline_meta_bytes = baseline_meta.dump( 2 ) + '\n';
    const auto        baseline_meta_path  = baseline_artifact_dir / "meta.json";
    writeFile( baseline_meta_path, baseline_meta_bytes );

    const auto           source_verification_path = root / "source-verification.json";
    const nlohmann::json source_verification{
        { "base_bundle_sha256", fileSha256( base_bundle_path ) },
        { "schema", "phad.m5_51.source_verification.v1" },
        { "source_archive_sha256", fileSha256( source_archive_path ) },
        { "source_id", source_id },
        { "source_manifest_sha256", sha256( source_manifest_bytes ) },
        { "source_root", source_root.string() },
        { "verified",
          { { "deleted_tracked_count", 0U },
            { "entry_count", 3U },
            { "git_branch", "synthetic-fixture" },
            { "git_head", std::string( 40U, 'a' ) },
            { "git_tree", std::string( 40U, 'b' ) },
            { "schema_version", 1 },
            { "source_id", source_id } } } };
    const std::string source_verification_bytes =
        source_verification.dump( 2 ) + '\n';
    writeFile( source_verification_path, source_verification_bytes );

    const std::filesystem::path fixture_executable{
        PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH };
    const auto        dynamic_libraries_path = root / "dynamic-libraries.sha256";
    const std::string dynamic_libraries_bytes =
        runtimeLibraryManifest( root, fixture_executable );
    writeFile( dynamic_libraries_path, dynamic_libraries_bytes );

    nlohmann::json qualification{
        { "schema", "phad.m5_51.baseline_qualification.v1" },
        { "status", "qualified" },
        { "source_id", source_id },
        { "source_manifest_path", source_manifest_path.string() },
        { "source_manifest_sha256", sha256( source_manifest_bytes ) },
        { "source_archive_path", source_archive_path.string() },
        { "source_archive_sha256", fileSha256( source_archive_path ) },
        { "base_bundle_path", base_bundle_path.string() },
        { "base_bundle_sha256", fileSha256( base_bundle_path ) },
        { "source_verification_receipt_path",
          source_verification_path.string() },
        { "source_verification_receipt_sha256",
          sha256( source_verification_bytes ) },
        { "binary_sha256", fileSha256( fixture_executable ) },
        { "config_hash", "0337287b" },
        { "config_canonical_text_sha256", kFormalConfigSha },
        { "input_manifest_path", input_manifest_path.string() },
        { "input_manifest_sha256", sha256( input_manifest_bytes ) },
        { "input_entry_count", 2U },
        { "rpe_evaluator_sha256", fileSha256( rpe_cpp ) },
        { "baseline_artifact_dir", baseline_artifact_dir.string() },
        { "baseline_state_receipt", baseline_path.string() },
        { "baseline_state_receipt_sha256", sha256( baseline_bytes ) } };

    const auto           preflight_path = root / "preflight.json";
    const nlohmann::json preflight{
        { "schema", "phad.m5_51.preflight_identity.v1" },
        { "source",
          { { "source_id", source_id },
            { "manifest_path", source_manifest_path.string() },
            { "manifest_sha256", sha256( source_manifest_bytes ) },
            { "archive_path", source_archive_path.string() },
            { "archive_sha256", fileSha256( source_archive_path ) },
            { "base_bundle_path", base_bundle_path.string() },
            { "base_bundle_sha256", fileSha256( base_bundle_path ) },
            { "verification_receipt_path",
              source_verification_path.string() },
            { "verification_receipt_sha256",
              sha256( source_verification_bytes ) },
            { "entry_count", 3U },
            { "deleted_tracked_count", 0U },
            { "git_head", std::string( 40U, 'a' ) },
            { "git_tree", std::string( 40U, 'b' ) } } },
        { "binaries",
          { { "phad_vo_bench",
              { { "path", fixture_executable.string() },
                { "sha256", fileSha256( fixture_executable ) } } },
            { "phad_traj_eval",
              { { "path", PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_TRAJ_EVAL_PATH ) } } } } },
        { "build",
          { { "type", PHAD_M5_51_EXPECTED_BUILD_TYPE },
            { "generator", PHAD_M5_51_EXPECTED_GENERATOR },
            { "phad_build_tests",
              static_cast<bool>( PHAD_M5_51_EXPECTED_BUILD_TESTS ) },
            { "compiler",
              { { "path", PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH },
                { "version", PHAD_M5_51_EXPECTED_CXX_COMPILER_VERSION },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_CXX_COMPILER_PATH ) } } },
            { "cmake",
              { { "path", PHAD_M5_51_EXPECTED_CMAKE_PATH },
                { "version", PHAD_M5_51_EXPECTED_CMAKE_VERSION },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_CMAKE_PATH ) } } },
            { "ninja",
              { { "path", PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH },
                { "version", "synthetic-fixture" },
                { "sha256",
                  fileSha256( PHAD_M5_51_EXPECTED_BUILD_TOOL_PATH ) } } } } },
        { "configuration",
          { { "label", "default" },
            { "canonical_hash", "0337287b" },
            { "canonical_text_sha256", kFormalConfigSha },
            { "moving_bootstrap", true },
            { "pnp_cheirality_retry", false },
            { "collect_timing", false },
            { "execution_bound_max_frames", 43U },
            { "reference_meta_path", baseline_meta_path.string() },
            { "reference_meta_sha256", sha256( baseline_meta_bytes ) } } },
        { "input",
          { { "root", sequence_root.string() },
            { "sequence", "MH_01_easy" },
            { "entry_count", 2U },
            { "verified_entry_count", 2U },
            { "ordered_manifest_path", input_manifest_path.string() },
            { "ordered_manifest_sha256", sha256( input_manifest_bytes ) } } },
        { "evaluator",
          { { "bench_source_sha256", fileSha256( bench_source ) },
            { "rpe_cpp_sha256", fileSha256( rpe_cpp ) },
            { "rpe_hpp_sha256", fileSha256( rpe_hpp ) },
            { "max_dt_ms", 2.5 },
            { "min_match_rate", 0.5 },
            { "rpe_delta_s", 1.0 } } },
        { "runtime",
          { { "determinism_environment",
              { { "MKL_NUM_THREADS", "1" },
                { "OMP_NUM_THREADS", "1" },
                { "OPENBLAS_NUM_THREADS", "1" } } },
            { "dynamic_libraries_path", dynamic_libraries_path.string() },
            { "dynamic_libraries_sha256",
              sha256( dynamic_libraries_bytes ) },
            { "dynamic_library_count",
              static_cast<std::size_t>( std::count(
                  dynamic_libraries_bytes.begin(),
                  dynamic_libraries_bytes.end(), '\n' ) ) },
            { "gtsam_version", "synthetic-fixture" },
            { "opencv_version", "synthetic-fixture" },
            { "openssl_version", "synthetic-fixture" } } } };
    const std::string preflight_bytes = preflight.dump( 2 ) + '\n';
    writeFile( preflight_path, preflight_bytes );
    qualification[ "preflight_identity_path" ] = preflight_path.string();
    qualification[ "preflight_identity_sha256" ] =
        sha256( preflight_bytes );
    const std::string qualification_bytes = qualification.dump( 2 ) + '\n';
    const auto        qualification_path  = root / "qualification.json";
    writeFile( qualification_path, qualification_bytes );

    const auto        output_path     = root / "output";
    const auto        state_path      = root / "intervention-state.json";
    const auto        comparison_path = root / "comparison.json";
    const auto        stdout_path     = root / "stdout.txt";
    const auto        stderr_path     = root / "stderr.txt";
    const std::string command =
        std::string{
            "OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \"" } +
        PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH +
        "\" \"" + sequence_root.string() + "\" --out \"" +
        output_path.string() +
        "\" --sequence-name MH_01_easy --config-label default "
        "--gt-euroc \"" +
        sequence_root.string() + "\" --repo \"" +
        source_root.string() +
        "\" --max-dt-ms 2.5 --min-match-rate 0.5 --rpe-delta-s 1.0 "
        "--max-frames 43 --estimator-enable-moving-bootstrap "
        "--m5-51-state-receipt \"" +
        state_path.string() +
        "\" --m5-51-selector-warmup --m5-51-baseline-qualification \"" +
        qualification_path.string() +
        "\" --m5-51-comparison-receipt \"" + comparison_path.string() +
        "\" > \"" + stdout_path.string() + "\" 2> \"" +
        stderr_path.string() + "\"";
    const int status = std::system( command.c_str() );
#ifdef __linux__
    ASSERT_TRUE( WIFEXITED( status ) );
    ASSERT_EQ( WEXITSTATUS( status ), 0 ) << readFile( stderr_path );
#else
    ASSERT_EQ( status, 0 ) << readFile( stderr_path );
#endif

    ASSERT_TRUE( std::filesystem::is_regular_file( state_path ) );
    ASSERT_TRUE( std::filesystem::is_regular_file( comparison_path ) );
    const auto state = nlohmann::ordered_json::parse( readFile( state_path ) );
    EXPECT_EQ( state.at( "schema" ),
               "m5_51_prefix_diagnostic_receipt_v1" );
    EXPECT_TRUE( state.at( "hook_executed" ).get<bool>() );
    EXPECT_EQ( state.at( "hook_frame_index" ), 6U );
    EXPECT_TRUE( state.at( "failure_reason" ).get<std::string>().empty() );
    EXPECT_EQ( state.at( "pre_hook_state_v1" ), baseline_pre );
    EXPECT_EQ( state.at( "hook_diff" ),
               nlohmann::json::array(
                   { { { "op", "replace" },
                       { "path", "/keyframe_selector/total_keyframes" },
                       { "value", 2U } } } ) );
    for ( const std::string_view field :
          { "pre_hook_state_v1", "post_hook_state_v1", "post_state_v1" } )
    {
      const auto& canonical = state.at( field ).at( "canonical_json" );
      EXPECT_EQ( state.at( field ).at( "sha256" ),
                 sha256( canonical.dump() ) );
    }
    EXPECT_FALSE( state.at( "post_state_v1" )
                      .at( "canonical_json" )
                      .at( "session" )
                      .at( "result" )
                      .at( "diag" )
                      .at( 7 )
                      .at( "is_keyframe" )
                      .get<bool>() );

    const auto comparison =
        nlohmann::ordered_json::parse( readFile( comparison_path ) );
    EXPECT_EQ( comparison.at( "schema" ),
               "phad.m5_51.prefix_comparison.v1" );
    EXPECT_EQ( comparison.at( "baseline_qualification_sha256" ),
               sha256( qualification_bytes ) );
    EXPECT_EQ( comparison.at( "baseline_state_receipt_sha256" ),
               sha256( baseline_bytes ) );
    EXPECT_EQ( comparison.at( "intervention_state_receipt_sha256" ),
               fileSha256( state_path ) );
    EXPECT_EQ( comparison.at( "pre_hook_state_sha256" ),
               state.at( "pre_hook_state_v1" ).at( "sha256" ) );
    EXPECT_EQ( comparison.at( "baseline_post_state_sha256" ),
               baseline_pre.at( "sha256" ) );
    EXPECT_EQ( comparison.at( "intervention_post_state_sha256" ),
               state.at( "post_state_v1" ).at( "sha256" ) );
    EXPECT_FALSE( comparison.at( "post_state_diff" ).empty() );
    EXPECT_FALSE(
        comparison.at( "intervention_i7_is_keyframe" ).get<bool>() );

    const std::string fixture_marker = "phad.m5_51.cli_fixture.v1";
    EXPECT_NE( readFile( PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH )
                   .find( fixture_marker ),
               std::string::npos );
    EXPECT_EQ( readFile( PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH )
                   .find( fixture_marker ),
               std::string::npos );

    constexpr std::string_view kSyntheticInputAnchor =
        "2b982fd1a2c814d077e0ee4361ca45d8b56ca23aabc65c5c14509d40f4763ea4";
    constexpr std::string_view kFormalInputAnchor =
        "edd7053e1152e1cd167cec3564e694696d7d5f25fddb0f4f90b1453a310992db";
    constexpr std::string_view kSyntheticDiagAnchor =
        "ef783a731d0946936e0640065a215138471b58d81f0b55d156ed61228331674d";
    constexpr std::string_view kFormalDiagAnchor =
        "dde502ec23ef8401d30024aa02a14e1854e5c05d1db054d9a3d82bd307696f7c";
    constexpr std::string_view kSyntheticEstAnchor =
        "ff390472ab5a66e5d4db829511c305d4155110cb8516e0a32bf4c191770527fe";
    constexpr std::string_view kFormalEstAnchor =
        "1f032910d0e2c4464c30d861d2d18d73439a904bbead148c09627c24d35b16b6";
    constexpr std::string_view kSyntheticKeyframeAnchor =
        "cfa32b9311b7ca9f4e56b1fdc88d32287e0d655d73cb34adcddf7bac4bb195aa";
    constexpr std::string_view kFormalKeyframeAnchor =
        "a5f9c6a423d6fc854cebe98a57d773faa1edbb7fee9fd15199dd941b13adcb7c";
    const std::string fixture_binary =
        readFile( PHAD_M5_51_REPLAY_DIAGNOSTIC_FIXTURE_PATH );
    const std::string diagnostic_binary =
        readFile( PHAD_M5_51_REPLAY_DIAGNOSTIC_PATH );
    const auto expect_isolated_anchor =
        [ &fixture_binary, &diagnostic_binary ](
            const std::string_view synthetic_anchor,
            const std::string_view formal_anchor ) {
          EXPECT_NE( fixture_binary.find( synthetic_anchor ),
                     std::string::npos );
          EXPECT_EQ( fixture_binary.find( formal_anchor ), std::string::npos );
          EXPECT_NE( diagnostic_binary.find( formal_anchor ),
                     std::string::npos );
          EXPECT_EQ( diagnostic_binary.find( synthetic_anchor ),
                     std::string::npos );
        };
    expect_isolated_anchor( kSyntheticInputAnchor, kFormalInputAnchor );
    expect_isolated_anchor( kSyntheticDiagAnchor, kFormalDiagAnchor );
    expect_isolated_anchor( kSyntheticEstAnchor, kFormalEstAnchor );
    expect_isolated_anchor( kSyntheticKeyframeAnchor,
                            kFormalKeyframeAnchor );
  }

}  // namespace

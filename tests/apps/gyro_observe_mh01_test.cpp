#include <gtest/gtest.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "apps/offline_vo_session.hpp"

#ifdef __linux__
#include <sys/wait.h>
#endif

namespace
{

#ifndef PHAD_VO_BENCH_PATH
#define PHAD_VO_BENCH_PATH ""
#endif

  constexpr std::string_view kPacketsHeader =
      "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,"
      "sum_dt_ns,interval_ns,status";
  constexpr std::string_view kSamplesHeader =
      "packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,"
      "gyr_z_radps";

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream      in( path, std::ios::binary );
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
  }

  [[nodiscard]] std::vector<std::string> splitCsv( std::string_view line )
  {
    std::vector<std::string> fields;
    std::size_t              begin = 0;
    while ( true )
    {
      const std::size_t end = line.find( ',', begin );
      if ( end == std::string_view::npos )
      {
        fields.emplace_back( line.substr( begin ) );
        return fields;
      }
      fields.emplace_back( line.substr( begin, end - begin ) );
      begin = end + 1U;
    }
  }

  template <typename Value>
  [[nodiscard]] Value parseNumber( const std::string& text )
  {
    Value      value{};
    const auto parsed =
        std::from_chars( text.data(), text.data() + text.size(), value );
    EXPECT_EQ( parsed.ec, std::errc{} );
    EXPECT_EQ( parsed.ptr, text.data() + text.size() );
    return value;
  }

  [[nodiscard]] phad::apps::GyroPacketStatus parseStatus(
      std::string_view status )
  {
    if ( status == "first_zero" )
    {
      return phad::apps::GyroPacketStatus::kFirstZero;
    }
    if ( status == "gap" )
    {
      return phad::apps::GyroPacketStatus::kGap;
    }
    if ( status == "empty_nonfirst" )
    {
      return phad::apps::GyroPacketStatus::kEmptyNonfirst;
    }
    EXPECT_EQ( status, "valid" );
    return phad::apps::GyroPacketStatus::kValid;
  }

  void readArtifacts(
      const std::filesystem::path&      packets_path,
      const std::filesystem::path&      samples_path,
      phad::apps::GyroObserveArtifacts& artifacts )
  {
    std::ifstream packets( packets_path, std::ios::binary );
    EXPECT_TRUE( packets );
    std::string line;
    ASSERT_TRUE( std::getline( packets, line ) );
    EXPECT_EQ( line, kPacketsHeader );
    while ( std::getline( packets, line ) )
    {
      const auto fields = splitCsv( line );
      ASSERT_EQ( fields.size(), 9U );
      artifacts.packets.push_back( phad::apps::GyroPacketRow{
          .packet_index  = parseNumber<std::uint64_t>( fields[ 0 ] ),
          .t_prev_ns     = parseNumber<std::int64_t>( fields[ 1 ] ),
          .t_cur_ns      = parseNumber<std::int64_t>( fields[ 2 ] ),
          .vo_segment_id = parseNumber<std::uint32_t>( fields[ 3 ] ),
          .imu_gap       = parseNumber<int>( fields[ 4 ] ) == 1,
          .sample_count  = parseNumber<std::uint64_t>( fields[ 5 ] ),
          .sum_dt_ns     = parseNumber<std::int64_t>( fields[ 6 ] ),
          .interval_ns   = parseNumber<std::int64_t>( fields[ 7 ] ),
          .status        = parseStatus( fields[ 8 ] ),
      } );
    }
    EXPECT_TRUE( packets.eof() );

    std::ifstream samples( samples_path, std::ios::binary );
    EXPECT_TRUE( samples );
    ASSERT_TRUE( std::getline( samples, line ) );
    EXPECT_EQ( line, kSamplesHeader );
    while ( std::getline( samples, line ) )
    {
      const auto fields = splitCsv( line );
      ASSERT_EQ( fields.size(), 6U );
      artifacts.samples.push_back( phad::apps::GyroSampleRow{
          .packet_index = parseNumber<std::uint64_t>( fields[ 0 ] ),
          .sample_index = parseNumber<std::uint64_t>( fields[ 1 ] ),
          .timestamp_ns = parseNumber<std::int64_t>( fields[ 2 ] ),
          .gyr_x_radps  = parseNumber<double>( fields[ 3 ] ),
          .gyr_y_radps  = parseNumber<double>( fields[ 4 ] ),
          .gyr_z_radps  = parseNumber<double>( fields[ 5 ] ),
      } );
    }
    EXPECT_TRUE( samples.eof() );
  }

  void readDiagJoin(
      const std::filesystem::path&                     path,
      std::unordered_map<std::int64_t, std::uint32_t>& join )
  {
    std::ifstream diag( path );
    std::string   line;
    EXPECT_TRUE( diag );
    ASSERT_TRUE( std::getline( diag, line ) );
    const auto header = splitCsv( line );
    ASSERT_EQ( header.size(), 20U );
    EXPECT_EQ( header[ 0 ], "timestamp_ns" );
    EXPECT_EQ( header[ 13 ], "segment_id" );
    while ( std::getline( diag, line ) )
    {
      const auto fields = splitCsv( line );
      ASSERT_EQ( fields.size(), header.size() );
      join.emplace( parseNumber<std::int64_t>( fields[ 0 ] ),
                    parseNumber<std::uint32_t>( fields[ 13 ] ) );
    }
  }

  [[nodiscard]] int runBench( const std::filesystem::path& dataset,
                              const std::filesystem::path& output,
                              const std::filesystem::path& stdout_path,
                              const std::filesystem::path& stderr_path,
                              std::string_view             extra_args )
  {
    const std::string command =
        std::string{ "\"" PHAD_VO_BENCH_PATH "\" \"" } + dataset.string() +
        "\" --gt-euroc \"" + dataset.string() + "\" --out \"" +
        output.string() + "\" " + std::string{ extra_args } + " > \"" +
        stdout_path.string() + "\" 2> \"" + stderr_path.string() + "\"";
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

  TEST( GyroObserveMh01Test, ObservePreservesControlAndPublishesReplayableCsvs )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const std::filesystem::path dataset = configured_path;
    const std::filesystem::path control =
        "/home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/"
        "m4_minimal_gyro_control";
    ASSERT_TRUE( std::filesystem::exists( control / "meta.json" ) );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_mh01_qualification";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto output      = root / "run";
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench( dataset, output, stdout_path, stderr_path,
                                    "--errors-csv --gyro-observe" );
    ASSERT_EQ( exit_code, 0 ) << readFile( stderr_path );

    EXPECT_TRUE( readFile( output / "est.tum" ) ==
                 readFile( control / "est.tum" ) );
    EXPECT_TRUE( readFile( output / "kf.tum" ) ==
                 readFile( control / "kf.tum" ) );
    EXPECT_TRUE( readFile( output / "diag.csv" ) ==
                 readFile( control / "diag.csv" ) );

    const auto output_meta =
        nlohmann::json::parse( readFile( output / "meta.json" ) );
    const auto control_meta =
        nlohmann::json::parse( readFile( control / "meta.json" ) );
    EXPECT_EQ( output_meta.at( "config_hash" ), "402d1925" );
    EXPECT_EQ( output_meta.at( "config" ), control_meta.at( "config" ) );
    EXPECT_EQ( output_meta.at( "config_canonical_text" ),
               control_meta.at( "config_canonical_text" ) );

    const std::string packets_text = readFile( output / "gyro_packets.csv" );
    const std::string samples_text = readFile( output / "gyro_samples.csv" );
    ASSERT_FALSE( packets_text.empty() );
    ASSERT_FALSE( samples_text.empty() );
    EXPECT_EQ( packets_text.back(), '\n' );
    EXPECT_EQ( samples_text.back(), '\n' );
    EXPECT_EQ( packets_text.find( '\r' ), std::string::npos );
    EXPECT_EQ( samples_text.find( '\r' ), std::string::npos );

    const auto replay_dir = root / "replay_without_dataset";
    std::filesystem::create_directories( replay_dir );
    std::filesystem::copy_file( output / "gyro_packets.csv",
                                replay_dir / "gyro_packets.csv" );
    std::filesystem::copy_file( output / "gyro_samples.csv",
                                replay_dir / "gyro_samples.csv" );
    std::filesystem::copy_file( output / "diag.csv",
                                replay_dir / "diag.csv" );
    phad::apps::GyroObserveArtifacts artifacts;
    ASSERT_NO_FATAL_FAILURE(
        readArtifacts( replay_dir / "gyro_packets.csv",
                       replay_dir / "gyro_samples.csv", artifacts ) );
    ASSERT_FALSE( artifacts.packets.empty() );
    EXPECT_FALSE( phad::apps::validateGyroObserveArtifacts( artifacts ) );
    std::unordered_map<std::int64_t, std::uint32_t> diag_join;
    ASSERT_NO_FATAL_FAILURE(
        readDiagJoin( replay_dir / "diag.csv", diag_join ) );
    ASSERT_EQ( artifacts.packets.size(), diag_join.size() );
    for ( const auto& packet : artifacts.packets )
    {
      const auto it = diag_join.find( packet.t_cur_ns );
      ASSERT_NE( it, diag_join.end() );
      EXPECT_EQ( packet.vo_segment_id, it->second );
    }

    std::filesystem::remove_all( root );
  }

  TEST( GyroObserveMh01Test, SuccessfulDefaultCliDoesNotPublishObserveCsvs )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const std::filesystem::path dataset = configured_path;
    const auto                  root    = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_mh01_default_off";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto output      = root / "run";
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        dataset, output, stdout_path, stderr_path,
        // EuRoC MH_01 groundtruth starts about 1.075 s after the first image;
        // 30 frames keep this integration short while retaining enough true
        // overlap for non-degenerate ATE/RPE evaluation.
        "--max-frames 30 --min-match-rate 0.25 --rpe-delta-s 0.05" );
    ASSERT_EQ( exit_code, 0 ) << readFile( stderr_path );
    EXPECT_TRUE( std::filesystem::exists( output / "summary.json" ) );
    EXPECT_FALSE( std::filesystem::exists( output / "gyro_packets.csv" ) );
    EXPECT_FALSE( std::filesystem::exists( output / "gyro_samples.csv" ) );
    EXPECT_FALSE(
        std::filesystem::exists( output / "gyro_packets.csv.tmp" ) );
    EXPECT_FALSE(
        std::filesystem::exists( output / "gyro_samples.csv.tmp" ) );

    std::filesystem::remove_all( root );
  }

  TEST( GyroObserveMh01Test,
        WriterFailureAfterSuccessfulSessionFailsRunAndSummary )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }
    ASSERT_FALSE( std::string_view{ PHAD_VO_BENCH_PATH }.empty() );

    const std::filesystem::path dataset = configured_path;
    const auto                  root    = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_mh01_writer_failure";
    std::filesystem::remove_all( root );
    const auto output = root / "run";
    std::filesystem::create_directories( output / "gyro_samples.csv" );
    const auto stdout_path = root / "stdout.txt";
    const auto stderr_path = root / "stderr.txt";

    const int exit_code = runBench(
        dataset, output, stdout_path, stderr_path,
        "--max-frames 30 --min-match-rate 0.25 --rpe-delta-s 0.05 "
        "--gyro-observe" );
    EXPECT_NE( exit_code, 0 );
    EXPECT_NE( readFile( stderr_path ).find( "gyro observe:samples_publish" ),
               std::string::npos );
    EXPECT_TRUE( std::filesystem::exists( output / "est.tum" ) );
    EXPECT_TRUE( std::filesystem::exists( output / "kf.tum" ) );
    EXPECT_TRUE( std::filesystem::exists( output / "diag.csv" ) );
    EXPECT_TRUE( std::filesystem::exists( output / "gyro_packets.csv" ) );
    EXPECT_TRUE( std::filesystem::is_directory( output /
                                                "gyro_samples.csv" ) );

    const auto summary =
        nlohmann::json::parse( readFile( output / "summary.json" ) );
    EXPECT_EQ( summary.at( "status" ), "failed" );
    const auto& warnings = summary.at( "warnings" );
    EXPECT_NE( std::find( warnings.begin(), warnings.end(),
                          "gyro observe:samples_publish" ),
               warnings.end() );

    std::filesystem::remove_all( root );
  }

}  // namespace

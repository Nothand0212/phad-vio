#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "apps/gyro_observe_writer.hpp"
#include "apps/offline_vo_session.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

namespace
{

  using phad::apps::collectGyroObservePacket;
  using phad::apps::GyroObserveArtifacts;
  using phad::apps::GyroPacketStatus;
  using phad::apps::gyroPacketStatusName;
  using phad::apps::validateGyroObserveArtifacts;
  using phad::apps::writeGyroObserveCsvs;
  using phad::common::Timestamp;
  using phad::sensor::Image;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::MeasurementDiscontinuity;
  using phad::sensor::RawImuInterval;
  using phad::sensor::StereoFrame;
  using phad::sensor::StereoImuPacket;

  [[nodiscard]] ImuMeasurement sample(
      std::int64_t          timestamp_ns,
      std::array<double, 3> gyro = { 0.0, 0.0, 0.0 } )
  {
    return ImuMeasurement{ .timestamp  = Timestamp{ timestamp_ns },
                           .accel_mps2 = {},
                           .gyro_radps = gyro };
  }

  [[nodiscard]] StereoImuPacket packet(
      std::int64_t                t_prev_ns,
      std::int64_t                t_cur_ns,
      std::vector<ImuMeasurement> samples = {},
      bool                        imu_gap = false )
  {
    const Image image{ 1, 1, 1, std::vector<std::uint8_t>{ 0 } };
    StereoFrame frame{ .timestamp = Timestamp{ t_cur_ns },
                       .left      = image,
                       .right     = image };
    if ( imu_gap )
    {
      return StereoImuPacket{
          .m_frame = std::move( frame ),
          .m_imu   = MeasurementDiscontinuity{
                .m_t_begin = Timestamp{ t_prev_ns },
                .m_t_end   = Timestamp{ t_cur_ns } } };
    }
    return StereoImuPacket{
        .m_frame = std::move( frame ),
        .m_imu   = RawImuInterval{
              .m_t_begin = Timestamp{ t_prev_ns },
              .m_t_end   = Timestamp{ t_cur_ns },
              .m_samples = std::move( samples ) } };
  }

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream      in( path, std::ios::binary );
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
  }

  [[nodiscard]] std::vector<std::string> splitLines( std::string_view text )
  {
    std::vector<std::string> lines;
    std::size_t              begin = 0;
    while ( begin < text.size() )
    {
      const std::size_t end = text.find( '\n', begin );
      if ( end == std::string_view::npos )
      {
        lines.emplace_back( text.substr( begin ) );
        break;
      }
      lines.emplace_back( text.substr( begin, end - begin ) );
      begin = end + 1U;
    }
    return lines;
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

  void expectErrorCode( const StereoImuPacket& packet,
                        GyroObserveArtifacts&  artifacts,
                        std::string_view       code )
  {
    const auto error = collectGyroObservePacket( packet, 0U, artifacts );
    ASSERT_TRUE( error.has_value() );
    EXPECT_EQ( error->detail, std::string{ "gyro observe:" } +
                                  std::string{ code } );
  }

  TEST( GyroObserveArtifactTest, CollectsConsecutiveRawPackets )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 90, 100, { sample( 90 ), sample( 100 ) } ), 7U,
        artifacts ) );
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 100, 110,
                { sample( 100, { 1.0, -2.0, 3.0 } ),
                  sample( 104, { 1.5, -2.5, 3.5 } ),
                  sample( 110, { 2.0, -3.0, 4.0 } ) } ),
        8U, artifacts ) );

    ASSERT_EQ( artifacts.packets.size(), 2U );
    EXPECT_EQ( artifacts.packets[ 0 ].status, GyroPacketStatus::kValid );
    EXPECT_EQ( artifacts.packets[ 1 ].status, GyroPacketStatus::kValid );
    EXPECT_EQ( gyroPacketStatusName( artifacts.packets[ 0 ].status ),
               "valid" );
    EXPECT_EQ( gyroPacketStatusName( artifacts.packets[ 1 ].status ),
               "valid" );
    EXPECT_EQ( artifacts.packets[ 1 ].packet_index, 1U );
    EXPECT_EQ( artifacts.packets[ 1 ].vo_segment_id, 8U );
    EXPECT_EQ( artifacts.packets[ 1 ].sample_count, 3U );
    EXPECT_EQ( artifacts.packets[ 1 ].sum_dt_ns, 10 );
    EXPECT_EQ( artifacts.packets[ 1 ].interval_ns, 10 );
    ASSERT_EQ( artifacts.samples.size(), 5U );
    EXPECT_EQ( artifacts.samples[ 4 ].sample_index, 2U );
    EXPECT_EQ( artifacts.samples[ 4 ].timestamp_ns, 110 );
    EXPECT_DOUBLE_EQ( artifacts.samples[ 4 ].gyr_z_radps, 4.0 );
    EXPECT_FALSE( validateGyroObserveArtifacts( artifacts ) );
  }

  TEST( GyroObserveArtifactTest, ClassifiesGapBeforeEmptyNonfirst )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 5, { sample( 0 ), sample( 5 ) } ), 0U,
        artifacts ) );
    ASSERT_FALSE( collectGyroObservePacket( packet( 5, 10, {}, true ), 0U,
                                            artifacts ) );
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 10, 15, { sample( 11 ), sample( 14 ) }, true ), 0U,
        artifacts ) );
    ASSERT_FALSE( collectGyroObservePacket( packet( 15, 20 ), 0U,
                                            artifacts ) );

    ASSERT_EQ( artifacts.packets.size(), 4U );
    EXPECT_EQ( artifacts.packets[ 1 ].status, GyroPacketStatus::kGap );
    EXPECT_EQ( artifacts.packets[ 2 ].status, GyroPacketStatus::kGap );
    EXPECT_EQ( artifacts.packets[ 3 ].status,
               GyroPacketStatus::kEmptyNonfirst );
  }

  TEST( GyroObserveArtifactTest, PreservesSharedPacketEndpointSamples )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 10, { sample( 0 ), sample( 10 ) } ), 0U, artifacts ) );
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 10, 20, { sample( 10 ), sample( 20 ) } ), 0U, artifacts ) );

    ASSERT_EQ( artifacts.samples.size(), 4U );
    EXPECT_EQ( artifacts.samples[ 1 ].timestamp_ns, 10 );
    EXPECT_EQ( artifacts.samples[ 2 ].timestamp_ns, 10 );
    EXPECT_EQ( artifacts.samples[ 1 ].packet_index, 0U );
    EXPECT_EQ( artifacts.samples[ 2 ].packet_index, 1U );
    EXPECT_EQ( artifacts.samples[ 2 ].sample_index, 0U );
  }

  TEST( GyroObserveArtifactTest, AcceptsFirstPositiveRawInterval )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 1, { sample( 0 ), sample( 1 ) } ), 9U,
        artifacts ) );
    ASSERT_EQ( artifacts.packets.size(), 1U );
    EXPECT_EQ( artifacts.packets.front().status, GyroPacketStatus::kValid );
    EXPECT_EQ( artifacts.packets.front().vo_segment_id, 9U );
  }

  TEST( GyroObserveArtifactTest, RejectsNonpositiveAndOverflowingIntervals )
  {
    GyroObserveArtifacts artifacts;
    expectErrorCode( packet( 2, 1, {}, true ), artifacts,
                     "interval_nonpositive" );
    expectErrorCode(
        packet( std::numeric_limits<std::int64_t>::min(),
                std::numeric_limits<std::int64_t>::max(), {}, true ),
        artifacts, "interval_overflow" );
  }

  TEST( GyroObserveArtifactTest, RejectsDuplicateAndOutOfOrderSamples )
  {
    GyroObserveArtifacts artifacts;
    expectErrorCode(
        packet( 0, 2, { sample( 0 ), sample( 1 ), sample( 1 ) } ),
        artifacts, "sample_duplicate" );
    expectErrorCode(
        packet( 0, 2, { sample( 0 ), sample( 2 ), sample( 1 ) } ),
        artifacts, "sample_out_of_order" );
  }

  TEST( GyroObserveArtifactTest, RejectsNonfiniteGyroOnEveryAxis )
  {
    constexpr std::array<double, 3> values = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    for ( std::size_t axis = 0; axis < values.size(); ++axis )
    {
      GyroObserveArtifacts  artifacts;
      std::array<double, 3> gyro{};
      gyro[ axis ] = values[ axis ];
      expectErrorCode(
          packet( 0, 1, { sample( 0, gyro ), sample( 1 ) } ), artifacts,
          "sample_nonfinite" );
    }
  }

  TEST( GyroObserveArtifactTest, PreservesBracketSpanAndValidatesRecordedSum )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 10, { sample( -1 ), sample( 11 ) } ), 0U,
        artifacts ) );
    ASSERT_EQ( artifacts.packets.size(), 1U );
    EXPECT_EQ( artifacts.packets.front().interval_ns, 10 );
    EXPECT_EQ( artifacts.packets.front().sum_dt_ns, 12 );

    artifacts.packets.front().sum_dt_ns = 11;
    const auto error                    = validateGyroObserveArtifacts( artifacts );
    ASSERT_TRUE( error.has_value() );
    EXPECT_EQ( error->detail, "gyro observe:interval_unclosed" );
  }

  TEST( GyroObserveArtifactTest, RejectsCheckedSampleDeltaAndSumOverflow )
  {
    GyroObserveArtifacts artifacts;
    expectErrorCode(
        packet( 0, 1,
                { sample( std::numeric_limits<std::int64_t>::min() ),
                  sample( std::numeric_limits<std::int64_t>::max() ) },
                false ),
        artifacts, "sample_delta_overflow" );
    expectErrorCode(
        packet( 0, 1,
                { sample( std::numeric_limits<std::int64_t>::min() + 1 ),
                  sample( 0 ),
                  sample( std::numeric_limits<std::int64_t>::max() ) },
                false ),
        artifacts, "sum_dt_overflow" );
  }

  TEST( GyroObserveArtifactTest, WriterUsesExactSchemaAndRoundTripsDoubles )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_writer";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto packets_path = root / "gyro_packets.csv";
    const auto samples_path = root / "gyro_samples.csv";

    GyroObserveArtifacts        artifacts;
    const std::array<double, 3> gyro = {
        -0.0,
        std::numeric_limits<double>::denorm_min(),
        -1.25,
    };
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 1, { sample( 0, gyro ), sample( 1, gyro ) } ), 4U,
        artifacts ) );

    ASSERT_FALSE(
        writeGyroObserveCsvs( packets_path, samples_path, artifacts ) );
    const auto packet_lines = splitLines( readFile( packets_path ) );
    const auto sample_lines = splitLines( readFile( samples_path ) );
    ASSERT_EQ( packet_lines.size(), 2U );
    ASSERT_EQ( sample_lines.size(), 3U );
    EXPECT_EQ( packet_lines[ 0 ],
               "packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,"
               "sample_count,sum_dt_ns,interval_ns,status" );
    EXPECT_EQ( sample_lines[ 0 ],
               "packet_index,sample_index,timestamp_ns,gyr_x_radps,"
               "gyr_y_radps,gyr_z_radps" );
    EXPECT_EQ( packet_lines[ 1 ], "0,0,1,4,0,2,1,1,valid" );

    const auto fields = splitCsv( sample_lines[ 1 ] );
    ASSERT_EQ( fields.size(), 6U );
    for ( std::size_t axis = 0; axis < gyro.size(); ++axis )
    {
      double             parsed = 0.0;
      const std::string& text   = fields[ axis + 3U ];
      const auto         result = std::from_chars(
          text.data(), text.data() + text.size(), parsed );
      ASSERT_EQ( result.ec, std::errc{} );
      ASSERT_EQ( result.ptr, text.data() + text.size() );
      EXPECT_EQ( std::bit_cast<std::uint64_t>( parsed ),
                 std::bit_cast<std::uint64_t>( gyro[ axis ] ) );
    }
    EXPECT_FALSE( std::filesystem::exists( packets_path.string() + ".tmp" ) );
    EXPECT_FALSE( std::filesystem::exists( samples_path.string() + ".tmp" ) );

    std::filesystem::remove_all( root );
  }

  TEST( GyroObserveArtifactTest, WriterReplacesExistingRegularTargets )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_writer_replace";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto packets_path = root / "gyro_packets.csv";
    const auto samples_path = root / "gyro_samples.csv";

    std::ofstream( packets_path ) << "stale packets contents that must vanish\n";
    std::ofstream( samples_path ) << "stale samples contents that must vanish\n";

    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 1,
                { sample( 0, { 1.0, 2.0, 3.0 } ),
                  sample( 1, { 4.0, 5.0, 6.0 } ) } ),
        4U, artifacts ) );

    ASSERT_FALSE(
        writeGyroObserveCsvs( packets_path, samples_path, artifacts ) );
    const std::string packets_first = readFile( packets_path );
    const std::string samples_first = readFile( samples_path );
    EXPECT_EQ( packets_first.find( "stale" ), std::string::npos );
    EXPECT_EQ( samples_first.find( "stale" ), std::string::npos );

    std::ofstream( packets_path, std::ios::binary | std::ios::trunc )
        << "different stale packets\n";
    std::ofstream( samples_path, std::ios::binary | std::ios::trunc )
        << "different stale samples\n";
    ASSERT_FALSE(
        writeGyroObserveCsvs( packets_path, samples_path, artifacts ) );
    EXPECT_EQ( readFile( packets_path ), packets_first );
    EXPECT_EQ( readFile( samples_path ), samples_first );
    EXPECT_FALSE( std::filesystem::exists( packets_path.string() + ".tmp" ) );
    EXPECT_FALSE( std::filesystem::exists( samples_path.string() + ".tmp" ) );

    std::filesystem::remove_all( root );
  }

  TEST( GyroObserveArtifactTest, WriterReportsOpenAndRenameFailures )
  {
    GyroObserveArtifacts artifacts;
    ASSERT_FALSE( collectGyroObservePacket(
        packet( 0, 1, { sample( 0 ), sample( 1 ) } ), 0U,
        artifacts ) );

    const auto root = std::filesystem::temp_directory_path() /
                      "phad_gyro_observe_writer_failures";
    std::filesystem::remove_all( root );
    const auto missing_packets = root / "missing" / "gyro_packets.csv";
    const auto missing_samples = root / "missing" / "gyro_samples.csv";
    const auto open_error =
        writeGyroObserveCsvs( missing_packets, missing_samples, artifacts );
    ASSERT_TRUE( open_error.has_value() );
    EXPECT_EQ( open_error->detail, "gyro observe:packets_open" );

    std::filesystem::create_directories( root );
    const auto packets_dir = root / "gyro_packets.csv";
    std::filesystem::create_directories( packets_dir );
    const auto rename_error = writeGyroObserveCsvs(
        packets_dir, root / "gyro_samples.csv", artifacts );
    ASSERT_TRUE( rename_error.has_value() );
    EXPECT_EQ( rename_error->detail, "gyro observe:packets_publish" );
    EXPECT_TRUE( std::filesystem::exists( packets_dir.string() + ".tmp" ) );
    EXPECT_FALSE( std::filesystem::exists( root / "gyro_samples.csv" ) );

    const auto packets_path = root / "packets_ok.csv";
    const auto samples_dir  = root / "samples_dir.csv";
    std::filesystem::create_directories( samples_dir );
    const auto samples_rename_error =
        writeGyroObserveCsvs( packets_path, samples_dir, artifacts );
    ASSERT_TRUE( samples_rename_error.has_value() );
    EXPECT_EQ( samples_rename_error->detail,
               "gyro observe:samples_publish" );
    EXPECT_TRUE( std::filesystem::exists( packets_path ) );
    EXPECT_TRUE( std::filesystem::exists( samples_dir.string() + ".tmp" ) );

    std::filesystem::remove_all( root );
  }

}  // namespace

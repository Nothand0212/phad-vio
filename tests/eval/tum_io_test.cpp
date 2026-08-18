#include "phad/eval/tum_io.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "tests/common/synthetic_trajectory.hpp"

namespace
{

  namespace fs = std::filesystem;

  using phad::common::TimedPose;
  using phad::common::Trajectory;
  using phad::eval::EvalErrorCode;
  using phad::eval::readTum;
  using phad::eval::readTumBytes;
  using phad::eval::writeTum;
  using phad::testing::kEurocEpochNs;
  using phad::testing::kStepNs;
  using phad::testing::makeHelix;
  using phad::testing::makePose;

  class TumFileFixture
  {
  public:
    TumFileFixture()
    {
      std::random_device random;
      m_path = fs::temp_directory_path() /
               ( "phad_tum_test_" + std::to_string( random() ) + ".tum" );
    }

    ~TumFileFixture()
    {
      std::error_code error;
      fs::remove( m_path, error );
    }

    TumFileFixture( const TumFileFixture& )            = delete;
    TumFileFixture& operator=( const TumFileFixture& ) = delete;

    [[nodiscard]] const fs::path& path() const { return m_path; }

    void write( const std::string& contents ) const
    {
      std::ofstream( m_path, std::ios::trunc ) << contents;
    }

    [[nodiscard]] std::string read() const
    {
      std::ifstream     stream( m_path );
      std::stringstream buffer;
      buffer << stream.rdbuf();
      return buffer.str();
    }

  private:
    fs::path m_path;
  };

  [[nodiscard]] std::string tumRow( std::string_view timestamp )
  {
    return std::string{ timestamp } + " 0 0 0 0 0 0 1\n";
  }

  void expectTimestampOverflow( std::string_view timestamp )
  {
    const fs::path source_label = "opaque-timestamp-overflow.tum";
    const auto     read         = readTumBytes( tumRow( timestamp ), source_label );

    ASSERT_FALSE( read ) << timestamp;
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    EXPECT_EQ( read.error().source_path, source_label );
    ASSERT_TRUE( read.error().line.has_value() );
    EXPECT_EQ( *read.error().line, 1U );
    EXPECT_EQ( read.error().field, "timestamp" );
    EXPECT_EQ( read.error().cause,
               "timestamp is outside the int64 nanosecond range" );
  }

  TEST( TumIoTest, ReadTumBytesParsesMaximumInt64Timestamp )
  {
    const auto read = readTumBytes(
        tumRow( "9223372036.854775807" ), "opaque-max-timestamp.tum" );

    ASSERT_TRUE( read ) << read.error().describe();
    EXPECT_EQ( read.value().firstTimestamp().nanoseconds(),
               std::numeric_limits<std::int64_t>::max() );
  }

  TEST( TumIoTest, ReadTumBytesParsesMinimumInt64Timestamp )
  {
    const auto read = readTumBytes(
        tumRow( "-9223372036.854775808" ), "opaque-min-timestamp.tum" );

    ASSERT_TRUE( read ) << read.error().describe();
    EXPECT_EQ( read.value().firstTimestamp().nanoseconds(),
               std::numeric_limits<std::int64_t>::min() );
  }

  TEST( TumIoTest, ReadTumBytesPreservesLexicalSignAcrossZero )
  {
    const auto read = readTumBytes(
        tumRow( "-1.5" ) + tumRow( "-0.5" ) + tumRow( "0.5" ) +
            tumRow( "1.5" ),
        "opaque-signed-timestamps.tum" );

    ASSERT_TRUE( read ) << read.error().describe();
    ASSERT_EQ( read.value().size(), 4U );
    EXPECT_EQ( read.value().poses()[ 0 ].timestamp.nanoseconds(),
               -1'500'000'000 );
    EXPECT_EQ( read.value().poses()[ 1 ].timestamp.nanoseconds(),
               -500'000'000 );
    EXPECT_EQ( read.value().poses()[ 2 ].timestamp.nanoseconds(),
               500'000'000 );
    EXPECT_EQ( read.value().poses()[ 3 ].timestamp.nanoseconds(),
               1'500'000'000 );
  }

  TEST( TumIoTest, ReadTumBytesRejectsSecondsMultiplicationOverflow )
  {
    expectTimestampOverflow( "9223372037" );
    expectTimestampOverflow( "-9223372037" );
  }

  TEST( TumIoTest, ReadTumBytesRejectsFractionCombinationOverflow )
  {
    expectTimestampOverflow( "9223372036.854775808" );
    expectTimestampOverflow( "-9223372036.854775809" );
  }

  TEST( TumIoTest, ReadTumBytesDoesNotOpenOpaqueSourceLabel )
  {
    const fs::path source_label =
        "/phad/nonexistent/read_tum_bytes/opaque-source.tum";
    ASSERT_FALSE( fs::exists( source_label ) );

    const auto read = readTumBytes(
        "# generated in memory\r\n"
        "\r\n"
        "0.000000000 0 0 0 0 0 0 1\r\n"
        "   \r\n"
        "1.000000000 1 0 0 0 0 0 1\r\n",
        source_label );

    ASSERT_TRUE( read ) << read.error().describe();
    ASSERT_EQ( read.value().size(), 2U );
    EXPECT_EQ( read.value().firstTimestamp().nanoseconds(), 0 );
    EXPECT_EQ( read.value().lastTimestamp().nanoseconds(), 1'000'000'000 );
  }

  TEST( TumIoTest, ReadTumBytesReportsExactSourceLabelAndPhysicalLine )
  {
    const fs::path source_label = "opaque://joined-estimate.tum";
    const auto     read         = readTumBytes(
        "# in-memory source\n"
                    "0.000000000 0 0 0 0 0 0 1\n"
                    "invalid third line\n",
        source_label );

    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    EXPECT_EQ( read.error().source_path, source_label );
    ASSERT_TRUE( read.error().line.has_value() );
    EXPECT_EQ( *read.error().line, 3U );
  }

  TEST( TumIoTest, RoundTripPreservesNanosecondTimestamps )
  {
    const TumFileFixture file;
    const Trajectory     written = makeHelix( 5 );
    ASSERT_FALSE( writeTum( file.path(), written ).has_value() );

    auto read = readTum( file.path() );
    ASSERT_TRUE( read ) << read.error().describe();
    const Trajectory restored = std::move( read ).value();
    ASSERT_EQ( restored.size(), written.size() );
    for ( std::size_t index = 0; index < written.size(); ++index )
    {
      EXPECT_EQ( restored.poses()[ index ].timestamp.nanoseconds(),
                 written.poses()[ index ].timestamp.nanoseconds() );
    }
  }

  TEST( TumIoTest, RoundTripPreservesPoses )
  {
    const TumFileFixture file;
    const Trajectory     written = makeHelix( 8 );
    ASSERT_FALSE( writeTum( file.path(), written ).has_value() );

    auto read = readTum( file.path() );
    ASSERT_TRUE( read ) << read.error().describe();
    const Trajectory restored = std::move( read ).value();
    for ( std::size_t index = 0; index < written.size(); ++index )
    {
      EXPECT_TRUE( restored.poses()[ index ].T_W_B.isApprox(
          written.poses()[ index ].T_W_B, 1e-12 ) );
    }
  }

  TEST( TumIoTest, WritesTimestampAsFixedNanosecondFraction )
  {
    const TumFileFixture file;
    auto                 trajectory = Trajectory::create(
        { makePose( kEurocEpochNs, Eigen::Vector3d::Zero() ) } );
    ASSERT_TRUE( trajectory );
    ASSERT_FALSE(
        writeTum( file.path(), std::move( trajectory ).value() ).has_value() );

    const std::string contents = file.read();
    EXPECT_EQ( contents.substr( 0, contents.find( ' ' ) ),
               "1403636579.763555584" );
  }

  TEST( TumIoTest, WritesQuaternionWithScalarLast )
  {
    const TumFileFixture     file;
    const Eigen::Quaterniond rotation{ Eigen::AngleAxisd{
        std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ() } };
    auto                     trajectory = Trajectory::create( { makePose(
        kEurocEpochNs, Eigen::Vector3d{ 1.0, 2.0, 3.0 }, rotation ) } );
    ASSERT_TRUE( trajectory );
    ASSERT_FALSE(
        writeTum( file.path(), std::move( trajectory ).value() ).has_value() );

    std::istringstream  stream( file.read() );
    std::string         timestamp;
    std::vector<double> values( 7 );
    stream >> timestamp >> values[ 0 ] >> values[ 1 ] >> values[ 2 ] >>
        values[ 3 ] >> values[ 4 ] >> values[ 5 ] >> values[ 6 ];
    ASSERT_FALSE( stream.fail() );
    EXPECT_DOUBLE_EQ( values[ 0 ], 1.0 );
    EXPECT_DOUBLE_EQ( values[ 1 ], 2.0 );
    EXPECT_DOUBLE_EQ( values[ 2 ], 3.0 );
    EXPECT_NEAR( values[ 3 ], 0.0, 1e-15 );
    EXPECT_NEAR( values[ 4 ], 0.0, 1e-15 );
    EXPECT_NEAR( values[ 5 ], std::sin( std::numbers::pi / 4.0 ), 1e-15 );
    EXPECT_NEAR( values[ 6 ], std::cos( std::numbers::pi / 4.0 ), 1e-15 );
  }

  TEST( TumIoTest, SkipsCommentsAndBlankLines )
  {
    const TumFileFixture file;
    file.write(
        "# generated by test\n"
        "\n"
        "1403636579.763555584 0 0 0 0 0 0 1\n"
        "   \n"
        "1403636579.813555584 1 0 0 0 0 0 1\n" );

    auto read = readTum( file.path() );
    ASSERT_TRUE( read ) << read.error().describe();
    EXPECT_EQ( read.value().size(), 2U );
    EXPECT_EQ( read.value().firstTimestamp().nanoseconds(), kEurocEpochNs );
    EXPECT_EQ( read.value().lastTimestamp().nanoseconds(),
               kEurocEpochNs + kStepNs );
  }

  TEST( TumIoTest, ParsesIntegerSecondsWithoutFraction )
  {
    const TumFileFixture file;
    file.write( "1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n" );

    auto read = readTum( file.path() );
    ASSERT_TRUE( read ) << read.error().describe();
    EXPECT_EQ( read.value().firstTimestamp().nanoseconds(), 1'000'000'000 );
  }

  TEST( TumIoTest, RejectsWrongFieldCount )
  {
    const TumFileFixture file;
    file.write( "1403636579.763555584 0 0 0 0 0 1\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    ASSERT_TRUE( read.error().line.has_value() );
    EXPECT_EQ( *read.error().line, 1U );
  }

  TEST( TumIoTest, RejectsNonNumericField )
  {
    const TumFileFixture file;
    file.write( "1403636579.763555584 0 0 nan 0 0 0 1\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    EXPECT_EQ( read.error().field, "tz" );
  }

  TEST( TumIoTest, RejectsNonUnitQuaternion )
  {
    const TumFileFixture file;
    file.write( "1403636579.763555584 0 0 0 0 0 0 0.5\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    EXPECT_EQ( read.error().field, "qx qy qz qw" );
  }

  TEST( TumIoTest, RejectsSubNanosecondTimestamp )
  {
    const TumFileFixture file;
    file.write( "1403636579.7635555841 0 0 0 0 0 0 1\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidRecord );
    EXPECT_EQ( read.error().field, "timestamp" );
  }

  TEST( TumIoTest, RejectsOutOfOrderTimestamps )
  {
    const TumFileFixture file;
    file.write(
        "1403636579.813555584 0 0 0 0 0 0 1\n"
        "1403636579.763555584 1 0 0 0 0 0 1\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidTrajectory );
    ASSERT_TRUE( read.error().line.has_value() );
    EXPECT_EQ( *read.error().line, 2U );
  }

  TEST( TumIoTest, RejectsEmptyFile )
  {
    const TumFileFixture file;
    file.write( "# only a comment\n" );

    const auto read = readTum( file.path() );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kInvalidTrajectory );
  }

  TEST( TumIoTest, ReportsMissingFile )
  {
    const auto read =
        readTum( fs::temp_directory_path() / "phad_tum_missing.tum" );
    ASSERT_FALSE( read );
    EXPECT_EQ( read.error().code, EvalErrorCode::kIoError );
  }

}  // namespace

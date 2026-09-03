#include "phad/io/dataset/tum_vi/tum_vi_groundtruth.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <string_view>

namespace
{

  namespace fs = std::filesystem;

  using phad::io::dataset::DatasetErrorCode;
  using phad::io::dataset::tum_vi::openGroundtruth;

  constexpr std::string_view kHeader =
      "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz\n";

  constexpr std::string_view kRows =
      "1520531829243709516,0.5,-1.5,2.5,1,0,0,0\n"
      "1520531829252042516,1.5,-1.5,2.5,0,0,0,1\n";

  class TumViGroundtruthFixture
  {
  public:
    TumViGroundtruthFixture()
    {
      const auto nonce =
          std::chrono::steady_clock::now().time_since_epoch().count();
      m_root = fs::temp_directory_path() /
               ( "phad-tum-vi-groundtruth-test-" +
                 std::to_string( nonce ) );
      fs::create_directories( m_root / "dso" );
      writeCsv( std::string{ kHeader } + std::string{ kRows } );
    }

    ~TumViGroundtruthFixture()
    {
      std::error_code error;
      fs::remove_all( m_root, error );
    }

    TumViGroundtruthFixture( const TumViGroundtruthFixture& ) = delete;
    TumViGroundtruthFixture& operator=(
        const TumViGroundtruthFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const noexcept { return m_root; }

    void writeCsv( const std::string& contents ) const
    {
      std::ofstream( m_root / "dso" / "gt_imu.csv", std::ios::trunc )
          << contents;
    }

  private:
    fs::path m_root;
  };

  TEST( TumViGroundtruthTest, LoadsBodyPosesFromNativeGtImuCsv )
  {
    const TumViGroundtruthFixture fixture;
    const auto                    result = openGroundtruth( fixture.root() );
    ASSERT_TRUE( result ) << result.error().describe();

    const auto& poses = result.value().poses();
    ASSERT_EQ( poses.size(), 2U );
    EXPECT_EQ( poses[ 0 ].timestamp.nanoseconds(),
               1'520'531'829'243'709'516 );
    EXPECT_TRUE( poses[ 0 ].T_W_B.translation().isApprox(
        Eigen::Vector3d{ 0.5, -1.5, 2.5 }, 1e-12 ) );
    EXPECT_TRUE( poses[ 0 ].T_W_B.linear().isApprox(
        Eigen::Matrix3d::Identity(), 1e-12 ) );
    EXPECT_TRUE( poses[ 1 ].T_W_B.linear().isApprox(
        Eigen::AngleAxisd{ std::numbers::pi, Eigen::Vector3d::UnitZ() }
            .toRotationMatrix(),
        1e-12 ) );
  }

  TEST( TumViGroundtruthTest, AcceptsNativeQuaternionRoundoffWithinOneEminus6 )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv(
        std::string{ kHeader } +
        "1520531829243709516,0.5,-1.5,2.5,1.0000005,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_TRUE( result.value().poses().front().T_W_B.linear().isApprox(
        Eigen::Matrix3d::Identity(), 1e-12 ) );
  }

  TEST( TumViGroundtruthTest, RejectsMissingSequenceRoot )
  {
    const auto result = openGroundtruth(
        fs::temp_directory_path() / "phad-tum-vi-groundtruth-missing" );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kRootNotFound );
  }

  TEST( TumViGroundtruthTest, RejectsMissingGtImuCsv )
  {
    const TumViGroundtruthFixture fixture;
    fs::remove( fixture.root() / "dso" / "gt_imu.csv" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kRequiredFileMissing );
  }

  TEST( TumViGroundtruthTest, RejectsUnexpectedHeader )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv( "# timestamp,tx,ty,tz,qw,qx,qy,qz\n" +
                      std::string{ kRows } );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidCsvHeader );
  }

  TEST( TumViGroundtruthTest, RejectsWrongColumnCount )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv( std::string{ kHeader } +
                      "1520531829243709516,0.5,-1.5,2.5,1,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidColumnCount );
    EXPECT_EQ( result.error().line_or_record_index, 2U );
  }

  TEST( TumViGroundtruthTest, RejectsNonFinitePoseField )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv( std::string{ kHeader } +
                      "1520531829243709516,nan,-1.5,2.5,1,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               DatasetErrorCode::kNonFiniteMeasurement );
    EXPECT_EQ( result.error().field, "tx" );
  }

  TEST( TumViGroundtruthTest, RejectsZeroQuaternion )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv( std::string{ kHeader } +
                      "1520531829243709516,0.5,-1.5,2.5,0,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidField );
    EXPECT_EQ( result.error().field, "q_W_B" );
  }

  TEST( TumViGroundtruthTest,
        RejectsQuaternionNormErrorBeyondOneEminus6 )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv(
        std::string{ kHeader } +
        "1520531829243709516,0.5,-1.5,2.5,1.000002,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidField );
  }

  TEST( TumViGroundtruthTest, RejectsDuplicateTimestamps )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv(
        std::string{ kHeader } +
        "1520531829243709516,0.5,-1.5,2.5,1,0,0,0\n"
        "1520531829243709516,1.5,-1.5,2.5,1,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kDuplicateTimestamp );
  }

  TEST( TumViGroundtruthTest, RejectsOutOfOrderTimestamps )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv(
        std::string{ kHeader } +
        "1520531829252042516,0.5,-1.5,2.5,1,0,0,0\n"
        "1520531829243709516,1.5,-1.5,2.5,1,0,0,0\n" );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               DatasetErrorCode::kOutOfOrderTimestamp );
  }

  TEST( TumViGroundtruthTest, RejectsHeaderOnlyCsv )
  {
    const TumViGroundtruthFixture fixture;
    fixture.writeCsv( std::string{ kHeader } );

    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kEmptyStream );
  }

}  // namespace

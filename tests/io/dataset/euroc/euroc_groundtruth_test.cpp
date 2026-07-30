#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <random>
#include <string>

namespace
{

  namespace fs = std::filesystem;

  using phad::io::dataset::DatasetErrorCode;
  using phad::io::dataset::euroc::openGroundtruth;

  constexpr const char* kSensorDirectory = "state_groundtruth_estimate0";

  const std::string kHeader =
      "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], "
      "q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], "
      "v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], "
      "b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], "
      "b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n";

  /// 两条真值记录，位姿为 identity 与绕 z 轴 180 度，velocity/bias 非零。
  const std::string kRows =
      "1403636580838555648,1.5,-2.5,0.5,1,0,0,0,"
      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n"
      "1403636580843555328,2.5,-2.5,0.5,0,0,0,1,"
      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n";

  class GroundtruthFixture
  {
  public:
    GroundtruthFixture()
    {
      std::random_device random;
      m_root = fs::temp_directory_path() /
               ( "phad_euroc_gt_test_" + std::to_string( random() ) );
      fs::create_directories( sensorRoot() );
      writeYaml( identityYaml() );
      writeCsv( kHeader + kRows );
    }

    ~GroundtruthFixture()
    {
      std::error_code error;
      fs::remove_all( m_root, error );
    }

    GroundtruthFixture( const GroundtruthFixture& )            = delete;
    GroundtruthFixture& operator=( const GroundtruthFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const { return m_root; }
    [[nodiscard]] fs::path        sensorRoot() const
    {
      return m_root / "mav0" / kSensorDirectory;
    }

    void writeYaml( const std::string& contents ) const
    {
      std::ofstream( sensorRoot() / "sensor.yaml", std::ios::trunc )
          << contents;
    }

    void writeCsv( const std::string& contents ) const
    {
      std::ofstream( sensorRoot() / "data.csv", std::ios::trunc ) << contents;
    }

    static std::string identityYaml()
    {
      return "sensor_type: visual-inertial\n"
             "T_BS:\n"
             "  rows: 4\n"
             "  cols: 4\n"
             "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
    }

  private:
    fs::path m_root;
  };

  TEST( EurocGroundtruthTest, LoadsPosesAndIgnoresVelocityAndBias )
  {
    const GroundtruthFixture fixture;
    const auto               result = openGroundtruth( fixture.root() );
    ASSERT_TRUE( result ) << result.error().describe();

    const auto& poses = result.value().poses();
    ASSERT_EQ( poses.size(), 2U );
    EXPECT_EQ( poses[ 0 ].timestamp.nanoseconds(), 1'403'636'580'838'555'648 );
    EXPECT_TRUE( poses[ 0 ].T_W_B.translation().isApprox(
        Eigen::Vector3d{ 1.5, -2.5, 0.5 } ) );
    EXPECT_TRUE( poses[ 0 ].T_W_B.linear().isApprox(
        Eigen::Matrix3d::Identity(), 1e-12 ) );
    EXPECT_TRUE( poses[ 1 ].T_W_B.linear().isApprox(
        Eigen::AngleAxisd{ std::numbers::pi, Eigen::Vector3d::UnitZ() }
            .toRotationMatrix(),
        1e-12 ) );
  }

  TEST( EurocGroundtruthTest, RejectsMissingSequenceRoot )
  {
    const auto result = openGroundtruth( fs::temp_directory_path() /
                                         "phad_euroc_gt_missing" );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kRootNotFound );
  }

  TEST( EurocGroundtruthTest, RejectsMissingCsv )
  {
    const GroundtruthFixture fixture;
    fs::remove( fixture.sensorRoot() / "data.csv" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kRequiredFileMissing );
    EXPECT_EQ( result.error().sensor_id, kSensorDirectory );
  }

  TEST( EurocGroundtruthTest, RejectsNonIdentityExtrinsics )
  {
    const GroundtruthFixture fixture;
    fixture.writeYaml(
        "sensor_type: visual-inertial\n"
        "T_BS:\n"
        "  rows: 4\n"
        "  cols: 4\n"
        "  data: [1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               DatasetErrorCode::kUnsupportedImuExtrinsics );
    EXPECT_EQ( result.error().field, "T_BS" );
  }

  TEST( EurocGroundtruthTest, RejectsUnexpectedHeader )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( "#timestamp,p_x,p_y,p_z\n" + kRows );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidCsvHeader );
  }

  TEST( EurocGroundtruthTest, RejectsWrongColumnCount )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( kHeader + "1403636580838555648,1.5,-2.5,0.5,1,0,0,0\n" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidColumnCount );
    ASSERT_TRUE( result.error().line_or_record_index.has_value() );
    EXPECT_EQ( *result.error().line_or_record_index, 2U );
  }

  TEST( EurocGroundtruthTest, RejectsNonUnitQuaternion )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( kHeader +
                      "1403636580838555648,1.5,-2.5,0.5,0.5,0,0,0,"
                      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kInvalidField );
    EXPECT_EQ( result.error().field, "q_RS" );
  }

  TEST( EurocGroundtruthTest, RejectsNonFinitePosition )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( kHeader +
                      "1403636580838555648,nan,-2.5,0.5,1,0,0,0,"
                      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().field, "p_RS_R_x" );
  }

  TEST( EurocGroundtruthTest, RejectsOutOfOrderTimestamps )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( kHeader +
                      "1403636580843555328,1.5,-2.5,0.5,1,0,0,0,"
                      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n"
                      "1403636580838555648,2.5,-2.5,0.5,1,0,0,0,"
                      "0.1,0.2,0.3,0.01,0.02,0.03,0.04,0.05,0.06\n" );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kOutOfOrderTimestamp );
  }

  TEST( EurocGroundtruthTest, RejectsHeaderOnlyCsv )
  {
    const GroundtruthFixture fixture;
    fixture.writeCsv( kHeader );
    const auto result = openGroundtruth( fixture.root() );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, DatasetErrorCode::kEmptyStream );
  }

}  // namespace

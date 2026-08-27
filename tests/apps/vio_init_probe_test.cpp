#include "apps/vio_init_probe.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "phad/estimator/types.hpp"

namespace
{

  using phad::apps::VioInitProbe;
  using phad::estimator::ImuInitDiagnostics;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioUpdateResult;

  struct CsvTable
  {
    std::unordered_map<std::string, std::size_t> columns;
    std::vector<std::vector<std::string>>        rows;
  };

  std::vector<std::string> split( const std::string& line )
  {
    std::vector<std::string> cells;
    std::stringstream        stream( line );
    std::string              cell;
    while ( std::getline( stream, cell, ',' ) )
    {
      cells.push_back( cell );
    }
    return cells;
  }

  CsvTable readCsv( const std::filesystem::path& path )
  {
    std::ifstream input( path );
    EXPECT_TRUE( input );
    std::string header_line;
    std::getline( input, header_line );
    const std::vector<std::string> header = split( header_line );

    CsvTable table;
    for ( std::size_t index = 0; index < header.size(); ++index )
    {
      table.columns.emplace( header[ index ], index );
    }
    std::string line;
    while ( std::getline( input, line ) )
    {
      table.rows.push_back( split( line ) );
    }
    return table;
  }

  const std::string& cell( const CsvTable& table, const std::string& name )
  {
    return table.rows.at( 0 ).at( table.columns.at( name ) );
  }

  VioUpdateResult makeInitEvent()
  {
    VioUpdateResult update;
    update.status = UpdateStatus::kRejected;

    ImuInitDiagnostics init;
    init.imu_sample_count = 101U;
    init.imu_t_i_ns       = 1'000'000'000;
    init.imu_t_j_ns       = 1'500'000'000;
    init.imu_dt_s         = 0.5;
    init.gyro_mean        = { 0.01, -0.02, 0.03 };
    init.gyro_std         = { 0.001, 0.002, 0.003 };
    init.acc_mean         = { 0.1, -0.2, 9.85 };
    init.acc_std          = { 0.01, 0.02, 0.03 };
    init.T_W_B0.linear() =
        Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitY() ).toRotationMatrix();
    init.velocity_W              = { 0.0, 0.0, 0.0 };
    init.bias_gyro               = { 0.011, -0.022, 0.033 };
    init.bias_acc                = { -0.11, 0.22, -0.33 };
    init.acc_mean_norm           = 9.85;
    init.gravity_model_magnitude = 9.81007;
    init.gyro_std_limit          = 0.01;
    init.acc_std_limit           = 0.2;
    update.diagnostics.imu_init  = init;
    return update;
  }

  TEST( VioInitProbeTest, WritesRejectedInitEventAndSkipsMissingEvent )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_init_probe_row";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "init.csv";
    {
      VioInitProbe    probe( path );
      VioUpdateResult missing;
      missing.status = UpdateStatus::kOk;
      probe.write( missing );
      probe.write( makeInitEvent() );
    }

    const CsvTable table = readCsv( path );
    EXPECT_EQ( table.columns.size(), 33U );
    ASSERT_EQ( table.rows.size(), 1U );
    EXPECT_EQ( table.rows[ 0 ].size(), table.columns.size() );
    EXPECT_EQ( cell( table, "imu_t_i_ns" ), "1000000000" );
    EXPECT_EQ( cell( table, "imu_t_j_ns" ), "1500000000" );
    EXPECT_EQ( cell( table, "imu_sample_n" ), "101" );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, "gyro_mean_y" ) ), -0.02 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, "acc_std_z" ) ), 0.03 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, "init_bg_z" ) ), 0.033 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, "init_ba_y" ) ), 0.22 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, "acc_mean_norm" ) ),
                      9.85 );

    std::filesystem::remove_all( root );
  }

  TEST( VioInitProbeTest, RejectsDuplicateEvent )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_init_probe_duplicate";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    VioInitProbe probe( root / "init.csv" );
    probe.write( makeInitEvent() );
    EXPECT_THROW( probe.write( makeInitEvent() ), std::logic_error );
    std::filesystem::remove_all( root );
  }

  TEST( VioInitProbeTest, RejectsInvalidSnapshotWithoutAdvancingTransaction )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_init_probe_invalid";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "init.csv";
    {
      VioInitProbe    probe( path );
      VioUpdateResult invalid = makeInitEvent();
      invalid.diagnostics.imu_init->gyro_mean.x() =
          std::numeric_limits<double>::quiet_NaN();
      EXPECT_THROW( probe.write( invalid ), std::invalid_argument );
      probe.write( makeInitEvent() );
    }
    EXPECT_EQ( readCsv( path ).rows.size(), 1U );
    std::filesystem::remove_all( root );
  }

  TEST( VioInitProbeTest, RejectsInvalidIntervalAndPose )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_init_probe_contract";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    VioInitProbe probe( root / "init.csv" );

    VioUpdateResult interval = makeInitEvent();
    interval.diagnostics.imu_init->imu_t_j_ns =
        interval.diagnostics.imu_init->imu_t_i_ns;
    EXPECT_THROW( probe.write( interval ), std::invalid_argument );

    VioUpdateResult pose                               = makeInitEvent();
    pose.diagnostics.imu_init->T_W_B0.linear()( 0, 0 ) = 2.0;
    EXPECT_THROW( probe.write( pose ), std::invalid_argument );

    std::filesystem::remove_all( root );
  }

  TEST( VioInitProbeTest, ReportsOpenFailure )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_init_probe_open_failure";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    EXPECT_THROW( VioInitProbe probe( root ), std::runtime_error );
    std::filesystem::remove_all( root );
  }

}  // namespace

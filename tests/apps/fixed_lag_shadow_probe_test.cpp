#include "apps/fixed_lag_shadow_probe.hpp"

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

#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"

namespace
{

  using phad::apps::FixedLagShadowProbe;
  using phad::common::Timestamp;
  using phad::estimator::FixedLagShadowDiagnostics;
  using phad::estimator::FixedLagShadowReset;
  using phad::estimator::FusionMode;
  using phad::estimator::GyroStateDiagnostics;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimate;
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

  const std::string& cell( const CsvTable& table, const std::size_t row,
                           const std::string& name )
  {
    return table.rows.at( row ).at( table.columns.at( name ) );
  }

  VioUpdateResult makeAccepted( const std::int64_t  timestamp_ns,
                                const std::uint64_t state_key,
                                const bool          active )
  {
    VioUpdateResult update;
    update.status   = UpdateStatus::kOk;
    update.estimate = VioEstimate{
        Timestamp{ timestamp_ns }, Eigen::Isometry3d::Identity() };
    update.diagnostics.window_size = 7U;
    GyroStateDiagnostics state;
    state.state_key               = state_key;
    update.diagnostics.gyro_state = state;

    FixedLagShadowDiagnostics shadow;
    if ( active )
    {
      update.diagnostics.fusion_mode       = FusionMode::kGyroVisual;
      shadow.active                        = true;
      shadow.reset                         = true;
      shadow.reset_reason                  = FixedLagShadowReset::kBootstrap;
      shadow.current_epoch                 = state_key;
      shadow.cutoff_epoch                  = state_key - 6U;
      shadow.batch_window_size             = 7U;
      shadow.smoother_pose_count           = 7U;
      shadow.smoother_landmark_count       = 12U;
      shadow.bias_present                  = true;
      shadow.bias_delta_norm               = 0.001;
      shadow.user_factor_count             = 44U;
      shadow.new_landmark_generation_count = 2U;
      shadow.newest_T_W_B.translation()    = Eigen::Vector3d( 1.0, 2.0, 3.0 );
      shadow.newest_T_W_B.linear() =
          Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitY() )
              .toRotationMatrix();
      shadow.newest_rotation_delta_rad  = 0.01;
      shadow.newest_translation_delta_m = 0.02;
    }
    update.diagnostics.fixed_lag_shadow = shadow;
    return update;
  }

  TEST( FixedLagShadowProbeTest, WritesAcceptedRowsAndSkipsRejected )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_fixed_lag_shadow_probe_rows";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "shadow.csv";
    {
      FixedLagShadowProbe probe( path );
      probe.write( makeAccepted( 100'000'000, 7U, false ) );

      VioUpdateResult rejected;
      rejected.status = UpdateStatus::kRejected;
      probe.write( rejected );

      probe.write( makeAccepted( 200'000'000, 8U, true ) );
    }

    const CsvTable table = readCsv( path );
    EXPECT_EQ( table.columns.size(), 30U );
    ASSERT_EQ( table.rows.size(), 2U );
    EXPECT_EQ( table.rows[ 0 ].size(), table.columns.size() );
    EXPECT_EQ( cell( table, 0U, "fusion_mode" ), "vision_only" );
    EXPECT_EQ( cell( table, 0U, "active" ), "0" );
    EXPECT_EQ( cell( table, 0U, "reset_reason" ), "none" );
    EXPECT_EQ( cell( table, 1U, "state_key" ), "8" );
    EXPECT_EQ( cell( table, 1U, "fusion_mode" ), "gyro_visual" );
    EXPECT_EQ( cell( table, 1U, "reset_reason" ), "bootstrap" );
    EXPECT_EQ( cell( table, 1U, "smoother_pose_n" ), "7" );
    EXPECT_EQ( cell( table, 1U, "smoother_landmark_n" ), "12" );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, 1U, "shadow_p_z" ) ),
                      3.0 );
    EXPECT_DOUBLE_EQ(
        std::stod( cell( table, 1U, "newest_rot_delta_rad" ) ), 0.01 );

    std::filesystem::remove_all( root );
  }

  TEST( FixedLagShadowProbeTest,
        RejectsInvalidDiagnosticsWithoutAdvancingTransaction )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_fixed_lag_shadow_probe_invalid";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "shadow.csv";
    {
      FixedLagShadowProbe probe( path );
      probe.write( makeAccepted( 100'000'000, 7U, false ) );

      VioUpdateResult invalid                                   = makeAccepted( 150'000'000, 8U, true );
      invalid.diagnostics.fixed_lag_shadow->smoother_pose_count = 6U;
      EXPECT_THROW( probe.write( invalid ), std::invalid_argument );

      VioUpdateResult non_finite = makeAccepted( 150'000'000, 8U, true );
      non_finite.diagnostics.fixed_lag_shadow->bias_delta_norm =
          std::numeric_limits<double>::quiet_NaN();
      EXPECT_THROW( probe.write( non_finite ), std::invalid_argument );

      probe.write( makeAccepted( 200'000'000, 8U, true ) );
    }
    EXPECT_EQ( readCsv( path ).rows.size(), 2U );
    std::filesystem::remove_all( root );
  }

  TEST( FixedLagShadowProbeTest, AllowsLandmarkFreeSegmentResetFrame )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_fixed_lag_shadow_probe_segment";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "shadow.csv";
    {
      FixedLagShadowProbe probe( path );
      probe.write( makeAccepted( 100'000'000, 7U, true ) );

      VioUpdateResult segment              = makeAccepted( 200'000'000, 8U, true );
      segment.diagnostics.window_size      = 1U;
      auto& shadow                         = *segment.diagnostics.fixed_lag_shadow;
      shadow.reset                         = true;
      shadow.reset_reason                  = FixedLagShadowReset::kSegment;
      shadow.cutoff_epoch                  = shadow.current_epoch;
      shadow.batch_window_size             = 1U;
      shadow.smoother_pose_count           = 1U;
      shadow.smoother_landmark_count       = 0U;
      shadow.user_factor_count             = 2U;
      shadow.new_landmark_generation_count = 0U;
      probe.write( segment );
    }
    const CsvTable table = readCsv( path );
    ASSERT_EQ( table.rows.size(), 2U );
    EXPECT_EQ( cell( table, 1U, "reset_reason" ), "segment" );
    EXPECT_EQ( cell( table, 1U, "smoother_landmark_n" ), "0" );
    std::filesystem::remove_all( root );
  }

  TEST( FixedLagShadowProbeTest, RejectsMissingSnapshotAndOpenFailure )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_fixed_lag_shadow_probe_missing";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    FixedLagShadowProbe probe( root / "shadow.csv" );

    VioUpdateResult missing = makeAccepted( 100'000'000, 7U, false );
    missing.diagnostics.fixed_lag_shadow.reset();
    EXPECT_THROW( probe.write( missing ), std::invalid_argument );
    EXPECT_THROW( FixedLagShadowProbe invalid_probe( root ),
                  std::runtime_error );

    std::filesystem::remove_all( root );
  }

}  // namespace

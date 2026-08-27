#include "apps/vio_state_probe.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
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

  using phad::apps::VioStateProbe;
  using phad::common::Timestamp;
  using phad::estimator::FusionMode;
  using phad::estimator::GyroGraphCostDiagnostics;
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
                                const bool          prediction_valid = true )
  {
    VioUpdateResult update;
    update.status = UpdateStatus::kOk;

    Eigen::Isometry3d posterior = Eigen::Isometry3d::Identity();
    posterior.translation()     = Eigen::Vector3d( 1.25, -2.5, 3.75 );
    posterior.linear() =
        Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitY() ).toRotationMatrix();
    update.estimate                                    = VioEstimate{ Timestamp{ timestamp_ns }, posterior };
    update.diagnostics.prior_key                       = state_key - 2U;
    update.diagnostics.window_size                     = 7U;
    update.diagnostics.bias_gyro                       = { 0.01, -0.02, 0.03 };
    update.diagnostics.fusion_mode                     = FusionMode::kGyroVisual;
    update.diagnostics.gyro_factor_count               = 3U;
    update.diagnostics.gyro_alignment_residual_rms_rad = 5e-4;
    GyroGraphCostDiagnostics gyro_cost;
    gyro_cost.gyro_factor_count                     = 3U;
    gyro_cost.interior_factor_count                 = 2U;
    gyro_cost.stereo_factor_count                   = 42U;
    gyro_cost.total_initial_cost                    = 90.0;
    gyro_cost.total_posterior_cost                  = 45.0;
    gyro_cost.gyro_initial_cost                     = 30.0;
    gyro_cost.gyro_posterior_cost                   = 12.0;
    gyro_cost.boundary_initial_cost                 = 8.0;
    gyro_cost.boundary_posterior_cost               = 3.0;
    gyro_cost.boundary_initial_residual_norm_rad    = 0.004;
    gyro_cost.boundary_posterior_residual_norm_rad  = 0.002;
    gyro_cost.boundary_initial_whitened_norm        = 4.0;
    gyro_cost.boundary_posterior_whitened_norm      = 2.0;
    gyro_cost.interior_initial_cost                 = 22.0;
    gyro_cost.interior_posterior_cost               = 9.0;
    gyro_cost.newest_initial_cost                   = 7.0;
    gyro_cost.newest_posterior_cost                 = 2.5;
    gyro_cost.newest_initial_residual_norm_rad      = 0.0035;
    gyro_cost.newest_posterior_residual_norm_rad    = 0.0015;
    gyro_cost.newest_initial_whitened_norm          = 3.5;
    gyro_cost.newest_posterior_whitened_norm        = 1.5;
    gyro_cost.stereo_initial_cost                   = 50.0;
    gyro_cost.stereo_posterior_cost                 = 25.0;
    gyro_cost.pose_prior_posterior_rotation_norm    = 0.006;
    gyro_cost.pose_prior_posterior_translation_norm = 0.007;
    gyro_cost.pose_prior_posterior_cost             = 4.0;
    gyro_cost.bias_prior_posterior_norm             = 0.008;
    gyro_cost.bias_prior_posterior_cost             = 4.0;
    update.diagnostics.gyro_graph_cost              = gyro_cost;

    GyroStateDiagnostics state;
    state.state_key        = state_key;
    state.prediction_valid = prediction_valid;
    if ( prediction_valid )
    {
      state.imu_sample_count = 6U;
      state.imu_t_i_ns       = timestamp_ns - 50'000'000;
      state.imu_t_j_ns       = timestamp_ns;
      state.imu_dt_s         = 0.05;
      state.predicted_T_W_B.translation() =
          Eigen::Vector3d( 1.125, -2.25, 3.5 );
      state.predicted_T_W_B.linear() =
          Eigen::AngleAxisd( 0.1, Eigen::Vector3d::UnitX() )
              .toRotationMatrix();
      state.prediction_bias_gyro = { 0.011, -0.022, 0.033 };
    }
    state.graph_initial_T_W_B.translation() =
        Eigen::Vector3d( 0.625, -1.25, 1.875 );
    state.graph_initial_T_W_B.linear() =
        Eigen::AngleAxisd( 0.15, Eigen::Vector3d::UnitZ() )
            .toRotationMatrix();
    update.diagnostics.gyro_state = state;
    return update;
  }

  TEST( VioStateProbeTest, WritesAcceptedStatesAndSkipsRejected )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_state_probe_rows";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "state.csv";
    {
      VioStateProbe probe( path );
      probe.write( true, makeAccepted( 100'000'000, 7U ) );

      VioUpdateResult rejected;
      rejected.status = UpdateStatus::kRejected;
      probe.write( true, rejected );

      probe.write( false, makeAccepted( 200'000'000, 8U, false ) );
    }

    const CsvTable table = readCsv( path );
    EXPECT_EQ( table.columns.size(), 69U );
    ASSERT_EQ( table.rows.size(), 2U );
    EXPECT_EQ( table.rows[ 0 ].size(), table.columns.size() );
    EXPECT_EQ( cell( table, 0U, "timestamp_ns" ), "100000000" );
    EXPECT_EQ( cell( table, 0U, "state_key" ), "7" );
    EXPECT_EQ( cell( table, 0U, "prior_key" ), "5" );
    EXPECT_EQ( cell( table, 0U, "is_keyframe" ), "1" );
    EXPECT_EQ( cell( table, 0U, "fusion_mode" ), "gyro_visual" );
    EXPECT_EQ( cell( table, 0U, "gyro_factor_n" ), "3" );
    EXPECT_DOUBLE_EQ(
        std::stod(
            cell( table, 0U, "gyro_alignment_residual_rms_rad" ) ),
        5e-4 );
    EXPECT_EQ( cell( table, 0U, "prediction_valid" ), "1" );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, 0U, "pred_p_x" ) ), 1.125 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, 0U, "pred_bg_y" ) ), -0.022 );
    EXPECT_DOUBLE_EQ(
        std::stod( cell( table, 0U, "graph_init_p_z" ) ), 1.875 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, 0U, "post_p_z" ) ), 3.75 );
    EXPECT_DOUBLE_EQ( std::stod( cell( table, 0U, "post_bg_z" ) ), 0.03 );
    EXPECT_EQ( cell( table, 0U, "gyro_obj_valid" ), "1" );
    EXPECT_EQ( cell( table, 0U, "gyro_obj_factor_n" ), "3" );
    EXPECT_EQ( cell( table, 0U, "gyro_obj_interior_n" ), "2" );
    EXPECT_DOUBLE_EQ(
        std::stod( cell( table, 0U, "gyro_obj_boundary_w_post" ) ), 2.0 );
    EXPECT_DOUBLE_EQ(
        std::stod( cell( table, 0U, "gyro_obj_new_r_init_rad" ) ), 0.0035 );
    EXPECT_DOUBLE_EQ(
        std::stod( cell( table, 0U, "gyro_obj_bias_prior_post" ) ), 4.0 );
    EXPECT_EQ( cell( table, 1U, "is_keyframe" ), "0" );
    EXPECT_EQ( cell( table, 1U, "prediction_valid" ), "0" );
    EXPECT_EQ( cell( table, 1U, "imu_sample_n" ), "0" );

    std::filesystem::remove_all( root );
  }

  TEST( VioStateProbeTest, RejectsMissingAcceptedState )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_state_probe_missing";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    VioStateProbe probe( root / "state.csv" );

    VioUpdateResult missing_estimate = makeAccepted( 100'000'000, 7U );
    missing_estimate.estimate.reset();
    EXPECT_THROW( probe.write( true, missing_estimate ), std::invalid_argument );

    VioUpdateResult missing_state = makeAccepted( 100'000'000, 7U );
    missing_state.diagnostics.gyro_state.reset();
    EXPECT_THROW( probe.write( true, missing_state ), std::invalid_argument );

    std::filesystem::remove_all( root );
  }

  TEST( VioStateProbeTest, RejectsNonFiniteAndInvalidInterval )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_state_probe_invalid";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    VioStateProbe probe( root / "state.csv" );

    VioUpdateResult non_finite = makeAccepted( 100'000'000, 7U );
    non_finite.diagnostics.gyro_state->prediction_bias_gyro.x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW( probe.write( false, non_finite ), std::invalid_argument );

    VioUpdateResult invalid_interval = makeAccepted( 100'000'000, 7U );
    invalid_interval.diagnostics.gyro_state->imu_t_j_ns =
        invalid_interval.diagnostics.gyro_state->imu_t_i_ns;
    EXPECT_THROW( probe.write( false, invalid_interval ),
                  std::invalid_argument );

    VioUpdateResult missing_gyro_cost = makeAccepted( 100'000'000, 7U );
    missing_gyro_cost.diagnostics.gyro_graph_cost.reset();
    EXPECT_THROW( probe.write( false, missing_gyro_cost ),
                  std::invalid_argument );

    VioUpdateResult non_finite_gyro_cost = makeAccepted( 100'000'000, 7U );
    non_finite_gyro_cost.diagnostics.gyro_graph_cost->boundary_initial_cost =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW( probe.write( false, non_finite_gyro_cost ),
                  std::invalid_argument );

    VioUpdateResult vision_with_gyro         = makeAccepted( 100'000'000, 7U );
    vision_with_gyro.diagnostics.fusion_mode = FusionMode::kVisionOnly;
    EXPECT_THROW( probe.write( false, vision_with_gyro ),
                  std::invalid_argument );

    std::filesystem::remove_all( root );
  }

  TEST( VioStateProbeTest, FailedValidationDoesNotAdvanceMonotonicState )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_state_probe_transaction";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    const auto path = root / "state.csv";
    {
      VioStateProbe probe( path );
      probe.write( true, makeAccepted( 100'000'000, 7U ) );
      EXPECT_THROW( probe.write( false, makeAccepted( 100'000'000, 8U ) ),
                    std::invalid_argument );
      probe.write( false, makeAccepted( 200'000'000, 8U ) );
    }
    EXPECT_EQ( readCsv( path ).rows.size(), 2U );
    std::filesystem::remove_all( root );
  }

  TEST( VioStateProbeTest, ReportsOpenFailure )
  {
    const auto root = std::filesystem::temp_directory_path() /
                      "phad_vio_state_probe_open_failure";
    std::filesystem::remove_all( root );
    std::filesystem::create_directories( root );
    EXPECT_THROW( VioStateProbe probe( root ), std::runtime_error );
    std::filesystem::remove_all( root );
  }

}  // namespace

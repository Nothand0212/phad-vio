#include "apps/offline_vo_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace
{

  using phad::apps::FrameCounts;
  using phad::apps::OfflineVoSessionOptions;
  using phad::apps::runOfflineVoSession;
  using phad::apps::VoDiagRow;
  using phad::apps::writeDiagCsv;

  [[nodiscard]] std::vector<std::string> splitCsvLine(
      std::string_view line )
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

  // Minimal two-frame EuRoC-shaped sequence used to drive
  // runOfflineVoSession() past the dataset-open step and into its per-frame
  // loop, so the mid-loop error paths (stream error / rectify failure) can
  // be exercised without a real dataset.
  class TinyEurocFixture
  {
  public:
    static constexpr std::int64_t kFirstTimestampNs =
        1'403'636'579'763'555'584LL;
    static constexpr std::int64_t kSecondTimestampNs =
        kFirstTimestampNs + 50'000'000LL;

    explicit TinyEurocFixture( std::string_view suffix )
    {
      m_root = std::filesystem::temp_directory_path() /
               ( "phad_offline_vo_session_tiny_seq_" +
                 std::string{ suffix } );
      std::filesystem::remove_all( m_root );
      for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
      {
        std::filesystem::create_directories( m_root / "mav0" / sensor /
                                             "data" );
      }
      writeCalibration();
      writeCsv( "imu0", imuCsv() );
      writeImage( "cam0", "left-a.png" );
      writeImage( "cam0", "left-b.png" );
      writeImage( "cam1", "right-a.png" );
      writeImage( "cam1", "right-b.png" );
      writeCsv( "cam0", "#timestamp [ns],filename\n" +
                            std::to_string( kFirstTimestampNs ) +
                            ",left-a.png\n" +
                            std::to_string( kSecondTimestampNs ) +
                            ",left-b.png\n" );
      writeCsv( "cam1", "#timestamp [ns],filename\n" +
                            std::to_string( kFirstTimestampNs ) +
                            ",right-a.png\n" +
                            std::to_string( kSecondTimestampNs ) +
                            ",right-b.png\n" );
    }

    ~TinyEurocFixture() { std::filesystem::remove_all( m_root ); }

    TinyEurocFixture( const TinyEurocFixture& )            = delete;
    TinyEurocFixture& operator=( const TinyEurocFixture& ) = delete;

    [[nodiscard]] const std::filesystem::path& root() const { return m_root; }

    void corruptSecondRightImage()
    {
      std::ofstream corrupt( m_root / "mav0" / "cam1" / "data" /
                                 "right-b.png",
                             std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }

  private:
    static std::string imuHeader()
    {
      return "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
             "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
             "a_RS_S_z [m s^-2]\n";
    }

    static std::string imuCsv()
    {
      const auto row = []( std::int64_t timestamp_ns ) {
        return std::to_string( timestamp_ns ) +
               ",0,0,0,0,0,9.81\n";
      };
      return imuHeader() + row( kFirstTimestampNs - 25'000'000LL ) +
             row( kFirstTimestampNs - 12'500'000LL ) +
             row( kFirstTimestampNs ) +
             row( kFirstTimestampNs + 25'000'000LL ) +
             row( kSecondTimestampNs ) +
             row( kSecondTimestampNs + 25'000'000LL );
    }

    void writeCsv( const std::string& sensor, const std::string& contents )
    {
      std::ofstream( m_root / "mav0" / sensor / "data.csv" ) << contents;
    }

    void writeImage( const std::string& sensor, const std::string& filename )
    {
      cv::Mat image( 48, 64, CV_8UC1, cv::Scalar( 30 ) );
      cv::imwrite(
          ( m_root / "mav0" / sensor / "data" / filename ).string(), image );
    }

    void writeCalibration()
    {
      const std::string cam0_yaml =
          "sensor_type: camera\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 20\n"
          "resolution: [64, 48]\n"
          "camera_model: pinhole\n"
          "intrinsics: [100, 100, 32, 24]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n";
      const std::string cam1_yaml =
          "sensor_type: camera\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0.11, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 20\n"
          "resolution: [64, 48]\n"
          "camera_model: pinhole\n"
          "intrinsics: [100, 100, 32, 24]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n";
      const std::string imu_yaml =
          "sensor_type: imu\n"
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
          "rate_hz: 200\n"
          "gyroscope_noise_density: 0.0001\n"
          "gyroscope_random_walk: 0.00001\n"
          "accelerometer_noise_density: 0.002\n"
          "accelerometer_random_walk: 0.003\n";
      std::ofstream( m_root / "mav0" / "cam0" / "sensor.yaml" ) << cam0_yaml;
      std::ofstream( m_root / "mav0" / "cam1" / "sensor.yaml" ) << cam1_yaml;
      std::ofstream( m_root / "mav0" / "imu0" / "sensor.yaml" ) << imu_yaml;
    }

    std::filesystem::path m_root;
  };

  TEST( OfflineVoSessionTest, MissingSequenceReturnsError )
  {
    OfflineVoSessionOptions options;
    options.sequence_root =
        std::filesystem::temp_directory_path() / "phad_missing_euroc_seq";
    std::filesystem::remove_all( options.sequence_root );

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 0U );
  }

  TEST( OfflineVoSessionTest, FrameCountsDefaultsIncludeSegmentFields )
  {
    FrameCounts counts;
    EXPECT_EQ( counts.segments, 0U );
    EXPECT_EQ( counts.reanchors, 0U );
    EXPECT_EQ( counts.seed_rejected, 0U );
    EXPECT_EQ( counts.pnp_successes, 0U );
    EXPECT_EQ( counts.pnp_fallbacks, 0U );
    EXPECT_EQ( counts.outliers_culled, 0U );
    EXPECT_EQ( counts.outliers_culled_unique, 0U );
    EXPECT_EQ( counts.outlier_reopts, 0U );
    EXPECT_EQ( counts.non_keyframe_evictions, 0U );
    EXPECT_EQ( counts.imu_reintegrations, 0U );
  }

  TEST( OfflineVoSessionTest, SkipDropMinCulledDefaultsFour )
  {
    OfflineVoSessionOptions options;
    EXPECT_FALSE( options.collect_gyro_observe );
    EXPECT_EQ( options.skip_drop_min_culled, 4 );
    EXPECT_TRUE( options.drop_culled_tracks );
    EXPECT_EQ( options.defer_drop_topk, 0 );
    FrameCounts counts;
    EXPECT_EQ( counts.drops_skipped, 0U );
    EXPECT_EQ( counts.deferred_drops, 0U );
    EXPECT_EQ( counts.deferred_drop_ids, 0U );
  }

  TEST( OfflineVoSessionTest,
        StreamErrorMidLoopFinalizesCountsWithoutSpuriousSummaryWarning )
  {
    TinyEurocFixture fixture{ "stream_error" };
    fixture.corruptSecondRightImage();

    OfflineVoSessionOptions options;
    options.sequence_root = fixture.root();

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.counts.image_frames, 1U );
    ASSERT_EQ( result.diag.size(), 1U );
    const VoDiagRow& row = result.diag.front();
    EXPECT_EQ( row.num_retained_observations, 0U );
    EXPECT_EQ( row.num_seeded_landmarks, 0U );
    EXPECT_EQ( row.num_current_visual_factors, 0U );
    EXPECT_EQ( row.num_current_mono_visual_factors, 0U );
    EXPECT_EQ( row.unsupported_span_ns, 0 );
    EXPECT_EQ( row.m_cold_root_phase,
               phad::estimator::ColdRootPhase::kStereoPopulation );
    EXPECT_EQ( row.m_cold_root_reason,
               phad::estimator::ColdRootReason::kPopulationInsufficient );
    EXPECT_EQ( row.m_cold_root_bootstrap_path,
               phad::estimator::ColdRootBootstrapPath::kStatic );
    EXPECT_EQ( row.m_cold_root_seed_input_origin,
               phad::estimator::ColdRootSeedInputOrigin::kCurrentPacket );
    EXPECT_EQ( row.m_cold_root_bootstrap_gate,
               phad::estimator::ColdRootGateState::kPassed );
    EXPECT_EQ( row.m_cold_root_stereo_population_gate,
               phad::estimator::ColdRootGateState::kFailed );
    EXPECT_FALSE( row.m_cold_root_attempt_id.has_value() );
    ASSERT_TRUE( row.m_cold_root_current_positive_disparity_count.has_value() );
    EXPECT_EQ( *row.m_cold_root_current_positive_disparity_count, 0U );
    ASSERT_TRUE( row.m_cold_root_accumulated_seed_enabled.has_value() );
    EXPECT_FALSE( *row.m_cold_root_accumulated_seed_enabled );
    ASSERT_TRUE( row.m_cold_root_effective_seed_count.has_value() );
    EXPECT_EQ( *row.m_cold_root_effective_seed_count, 0U );
    EXPECT_FALSE( row.m_cold_root_pending_unique_seed_count.has_value() );

    // Mid-loop stream errors still run segment finalization, but a run
    // With no initialization rejection, no initialization summary warning is
    // emitted. Cull totals never
    // enter warnings either.
    EXPECT_EQ( result.counts.reanchors, 0U );
    EXPECT_EQ( result.counts.seed_rejected, 0U );
    const bool has_summary_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo initialization summary:", 0 ) ==
                              0;
                     } );
    EXPECT_FALSE( has_summary_warning );
    const bool has_cull_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo outlier cull summary:", 0 ) ==
                              0;
                     } );
    EXPECT_FALSE( has_cull_warning );
    EXPECT_TRUE( result.gyro_observe.packets.empty() );
    EXPECT_TRUE( result.gyro_observe.samples.empty() );
  }

  TEST( OfflineVoSessionTest, EstimatorHardFailureDoesNotAppendDiagRow )
  {
    TinyEurocFixture fixture{ "estimator_hard_failure" };

    OfflineVoSessionOptions options;
    options.sequence_root                     = fixture.root();
    options.estimator.m_bootstrap_min_samples = 100U;
    options.estimator.m_bootstrap_timeout_ns  = 20'000'000;

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_NE( result.error->detail.find( "static bootstrap timed out" ),
               std::string::npos );
    EXPECT_EQ( result.counts.image_frames, 1U );
    EXPECT_EQ( result.counts.failed, 1U );
    EXPECT_TRUE( result.diag.empty() );
    EXPECT_FALSE( result.trajectory.has_value() );
  }

  TEST( OfflineVoSessionTest, GyroObserveCollectsPacketBeforeLaterStreamError )
  {
    TinyEurocFixture fixture{ "gyro_observe_stream_error" };
    fixture.corruptSecondRightImage();

    OfflineVoSessionOptions options;
    options.sequence_root        = fixture.root();
    options.collect_gyro_observe = true;

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    ASSERT_EQ( result.gyro_observe.packets.size(), 1U );
    EXPECT_EQ( result.gyro_observe.packets.front().status,
               phad::apps::GyroPacketStatus::kValid );
    EXPECT_EQ( result.gyro_observe.packets.front().vo_segment_id,
               result.diag.empty() ? 0U : result.diag.front().segment_id );
    EXPECT_EQ( result.gyro_observe.samples.size(),
               result.gyro_observe.packets.front().sample_count );
    EXPECT_EQ( result.gyro_observe.samples.front().timestamp_ns,
               TinyEurocFixture::kFirstTimestampNs - 25'000'000LL );
  }

  TEST( OfflineVoSessionTest, WriteDiagCsvMatchesProbeContract )
  {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_session_diag.csv";
    std::filesystem::remove( path );

    VoDiagRow row;
    row.timestamp_ns                    = 1403636579763555584LL;
    row.status                          = "ok";
    row.num_observations                = 136;
    row.num_landmarks                   = 0;
    row.num_shared                      = 0;
    row.num_disparity                   = 91;
    row.low_connectivity                = false;
    row.window_size                     = 1;
    row.prior_key                       = 0;
    row.reproj_rms_before_px            = 0.0;
    row.reproj_rms_after_px             = 0.0;
    row.num_cheirality                  = 0;
    row.lm_iterations                   = 0;
    row.max_window_pose_shift_m         = 0.0;
    row.segment_id                      = 0;
    row.pnp_success                     = false;
    row.pnp_inliers                     = 0;
    row.outliers_culled                 = 0;
    row.reproj_rms_after_cull_px        = 0.0;
    row.unsupported_span_ns             = 250'000'000;
    row.num_retained_observations       = 73;
    row.num_seeded_landmarks            = 11;
    row.num_current_visual_factors      = 9;
    row.num_mapped_observations         = 2;
    row.num_current_mono_visual_factors = 4;
    row.m_cold_root_phase               = phad::estimator::ColdRootPhase::kCommit;
    row.m_cold_root_reason              = phad::estimator::ColdRootReason::kCommitted;
    row.m_cold_root_bootstrap_path =
        phad::estimator::ColdRootBootstrapPath::kMoving;
    row.m_cold_root_seed_input_origin =
        phad::estimator::ColdRootSeedInputOrigin::kCurrentPacket;
    row.m_cold_root_geometry_result =
        phad::estimator::ColdRootGeometryResult::kAccepted;
    row.m_cold_root_attempt_id     = 7U;
    row.m_cold_root_bootstrap_gate = phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_keyframe_gate  = phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_stereo_population_gate =
        phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_geometry_gate = phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_current_graph_gate =
        phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_commit_gate                       = phad::estimator::ColdRootGateState::kPassed;
    row.m_cold_root_bootstrap_sample_count            = 5U;
    row.m_cold_root_bootstrap_min_samples             = 3U;
    row.m_cold_root_bootstrap_duration_ns             = 50'000'000;
    row.m_cold_root_bootstrap_min_duration_ns         = 20'000'000;
    row.m_cold_root_bootstrap_acc_std_max_mps2        = 0.0;
    row.m_cold_root_bootstrap_acc_std_limit_mps2      = 0.05;
    row.m_cold_root_bootstrap_gyr_std_max_radps       = 0.0;
    row.m_cold_root_bootstrap_gyr_std_limit_radps     = 0.005;
    row.m_cold_root_bootstrap_acc_norm_error_mps2     = 0.0;
    row.m_cold_root_bootstrap_acc_norm_tolerance_mps2 = 0.25;
    row.m_cold_root_bootstrap_timeout_ns              = 1'000'000'000;
    row.m_cold_root_moving_bootstrap_enabled          = true;
    row.m_cold_root_moving_suffix_sample_count        = 3U;
    row.m_cold_root_moving_suffix_duration_ns         = 20'000'000;
    row.m_cold_root_moving_acc_mean_norm_mps2         = 9.81;
    row.m_cold_root_moving_acc_mean_norm_min_mps2 =
        std::numeric_limits<double>::epsilon();
    row.m_cold_root_current_positive_disparity_count = 10U;
    row.m_cold_root_accumulated_seed_enabled         = false;
    row.m_cold_root_effective_seed_count             = 10U;
    row.m_cold_root_min_seed_observations            = 10U;
    row.m_cold_root_geometry_accepted_landmarks      = 10U;
    row.m_cold_root_geometry_min_landmarks           = 1U;

    VoDiagRow outage_row;
    outage_row.timestamp_ns        = row.timestamp_ns + 1;
    outage_row.status              = "visual_outage";
    outage_row.unsupported_span_ns = 300'000'000;
    VoDiagRow discontinuity_row;
    discontinuity_row.timestamp_ns = row.timestamp_ns + 2;
    discontinuity_row.status       = "discontinuity";

    ASSERT_FALSE( writeDiagCsv(
                      path, { row, outage_row, discontinuity_row } )
                      .has_value() );

    std::ifstream in( path );
    ASSERT_TRUE( in );
    std::string header_line;
    std::string row_line;
    std::string outage_line;
    std::string discontinuity_line;
    std::string extra_line;
    ASSERT_TRUE( std::getline( in, header_line ) );
    ASSERT_TRUE( std::getline( in, row_line ) );
    ASSERT_TRUE( std::getline( in, outage_line ) );
    ASSERT_TRUE( std::getline( in, discontinuity_line ) );
    EXPECT_FALSE( std::getline( in, extra_line ) );

    const std::vector<std::string> expected_header{
        "timestamp_ns",
        "status",
        "num_obs",
        "num_landmarks",
        "num_shared",
        "low_connectivity",
        "window_size",
        "prior_key",
        "reproj_rms_before_px",
        "reproj_rms_after_px",
        "num_cheirality",
        "lm_iterations",
        "max_window_pose_shift_m",
        "segment_id",
        "pnp_success",
        "pnp_inliers",
        "outliers_culled",
        "reproj_rms_after_cull_px",
        "is_keyframe",
        "num_disparity",
        "unsupported_span_ns",
        "num_retained_observations",
        "num_seeded_landmarks",
        "num_current_visual_factors",
        "num_mapped_observations",
        "num_current_mono_visual_factors",
        "cold_root_phase",
        "cold_root_reason",
        "cold_root_bootstrap_path",
        "cold_root_seed_input_origin",
        "cold_root_geometry_result",
        "cold_root_attempt_id",
        "cold_root_bootstrap_gate",
        "cold_root_keyframe_gate",
        "cold_root_stereo_population_gate",
        "cold_root_geometry_gate",
        "cold_root_imu_excitation_gate",
        "cold_root_conditioning_gate",
        "cold_root_initialization_solve_gate",
        "cold_root_current_graph_gate",
        "cold_root_commit_gate",
        "cold_root_bootstrap_sample_count",
        "cold_root_bootstrap_min_samples",
        "cold_root_bootstrap_duration_ns",
        "cold_root_bootstrap_min_duration_ns",
        "cold_root_bootstrap_acc_std_max_mps2",
        "cold_root_bootstrap_acc_std_limit_mps2",
        "cold_root_bootstrap_gyr_std_max_radps",
        "cold_root_bootstrap_gyr_std_limit_radps",
        "cold_root_bootstrap_acc_norm_error_mps2",
        "cold_root_bootstrap_acc_norm_tolerance_mps2",
        "cold_root_bootstrap_timeout_ns",
        "cold_root_moving_bootstrap_enabled",
        "cold_root_moving_suffix_sample_count",
        "cold_root_moving_suffix_duration_ns",
        "cold_root_moving_acc_mean_norm_mps2",
        "cold_root_moving_acc_mean_norm_min_mps2",
        "cold_root_current_positive_disparity_count",
        "cold_root_accumulated_seed_enabled",
        "cold_root_pending_unique_seed_count",
        "cold_root_effective_seed_count",
        "cold_root_min_seed_observations",
        "cold_root_geometry_accepted_landmarks",
        "cold_root_geometry_min_landmarks",
    };

    const std::vector<std::string> header_fields = splitCsvLine( header_line );
    const std::vector<std::string> row_fields    = splitCsvLine( row_line );
    const std::vector<std::string> outage_fields = splitCsvLine( outage_line );
    const std::vector<std::string> discontinuity_fields =
        splitCsvLine( discontinuity_line );
    EXPECT_EQ( header_fields, expected_header );
    ASSERT_EQ( row_fields.size(), 64U );
    ASSERT_EQ( outage_fields.size(), 64U );
    ASSERT_EQ( discontinuity_fields.size(), 64U );

    const std::vector<std::string> expected_legacy_row{
        "1403636579763555584",
        "ok",
        "136",
        "0",
        "0",
        "0",
        "1",
        "0",
        "0.000000",
        "0.000000",
        "0",
        "0",
        "0.000000",
        "0",
        "0",
        "0",
        "0",
        "0.000000",
        "0",
        "91",
        "250000000",
        "73",
        "11",
        "9",
        "2",
        "4",
    };
    EXPECT_EQ( std::vector<std::string>( row_fields.begin(),
                                         row_fields.begin() + 26 ),
               expected_legacy_row );

    EXPECT_EQ( row_fields[ 26 ], "commit" );
    EXPECT_EQ( row_fields[ 27 ], "committed" );
    EXPECT_EQ( row_fields[ 28 ], "moving" );
    EXPECT_EQ( row_fields[ 29 ], "current_packet" );
    EXPECT_EQ( row_fields[ 30 ], "accepted" );
    EXPECT_EQ( row_fields[ 31 ], "7" );
    for ( const std::size_t index : { 32U, 33U, 34U, 35U, 39U, 40U } )
    {
      EXPECT_EQ( row_fields[ index ], "passed" );
    }
    for ( const std::size_t index : { 36U, 37U, 38U } )
    {
      EXPECT_EQ( row_fields[ index ], "not_evaluated" );
    }
    EXPECT_EQ( row_fields[ 41 ], "5" );
    EXPECT_EQ( row_fields[ 42 ], "3" );
    EXPECT_EQ( row_fields[ 43 ], "50000000" );
    EXPECT_EQ( row_fields[ 44 ], "20000000" );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 45 ] ), 0.0 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 46 ] ), 0.05 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 47 ] ), 0.0 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 48 ] ), 0.005 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 49 ] ), 0.0 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 50 ] ), 0.25 );
    EXPECT_EQ( row_fields[ 51 ], "1000000000" );
    EXPECT_EQ( row_fields[ 52 ], "1" );
    EXPECT_EQ( row_fields[ 53 ], "3" );
    EXPECT_EQ( row_fields[ 54 ], "20000000" );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 55 ] ), 9.81 );
    EXPECT_DOUBLE_EQ( std::stod( row_fields[ 56 ] ),
                      std::numeric_limits<double>::epsilon() );
    EXPECT_EQ( row_fields[ 57 ], "10" );
    EXPECT_EQ( row_fields[ 58 ], "0" );
    EXPECT_TRUE( row_fields[ 59 ].empty() );
    EXPECT_EQ( row_fields[ 60 ], "10" );
    EXPECT_EQ( row_fields[ 61 ], "10" );
    EXPECT_EQ( row_fields[ 62 ], "10" );
    EXPECT_EQ( row_fields[ 63 ], "1" );

    const std::vector<std::string> expected_outage_legacy{
        "1403636579763555585",
        "visual_outage",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0.000000",
        "0.000000",
        "0",
        "0",
        "0.000000",
        "0",
        "0",
        "0",
        "0",
        "0.000000",
        "0",
        "0",
        "300000000",
        "0",
        "0",
        "0",
        "0",
        "0",
    };
    const std::vector<std::string> expected_discontinuity_legacy{
        "1403636579763555586",
        "discontinuity",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0.000000",
        "0.000000",
        "0",
        "0",
        "0.000000",
        "0",
        "0",
        "0",
        "0",
        "0.000000",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
        "0",
    };
    EXPECT_EQ( std::vector<std::string>( outage_fields.begin(),
                                         outage_fields.begin() + 26 ),
               expected_outage_legacy );
    EXPECT_EQ( std::vector<std::string>( discontinuity_fields.begin(),
                                         discontinuity_fields.begin() + 26 ),
               expected_discontinuity_legacy );

    std::vector<std::string> expected_default_observe( 38U );
    std::fill_n( expected_default_observe.begin(), 5U, "not_evaluated" );
    std::fill_n( expected_default_observe.begin() + 6, 9U,
                 "not_evaluated" );
    EXPECT_EQ( std::vector<std::string>( outage_fields.begin() + 26,
                                         outage_fields.end() ),
               expected_default_observe );
    EXPECT_EQ( std::vector<std::string>( discontinuity_fields.begin() + 26,
                                         discontinuity_fields.end() ),
               expected_default_observe );
    std::filesystem::remove( path );
  }

  TEST( OfflineVoSessionTest, EmptyProbeBPathDoesNotCreateFile )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    const auto probe_dir = std::filesystem::temp_directory_path() /
                           "phad_offline_vo_session_probe_b_off";
    std::filesystem::remove_all( probe_dir );
    std::filesystem::create_directories( probe_dir );
    const auto probe_path = probe_dir / "probe_b.jsonl";

    OfflineVoSessionOptions options;
    options.sequence_root = configured_path;
    options.max_frames    = 5;
    // Default: probe_b_path empty → writer not constructed.
    ASSERT_TRUE( options.probe_b_path.empty() );

    const auto result = runOfflineVoSession( options );
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    EXPECT_EQ( result.counts.image_frames, 5U );
    EXPECT_FALSE( std::filesystem::exists( probe_path ) );
    EXPECT_TRUE( std::filesystem::is_empty( probe_dir ) );

    std::filesystem::remove_all( probe_dir );
  }

  TEST( OfflineVoSessionTest, ValidProbeBPathWritesJsonlLines )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    const auto probe_dir = std::filesystem::temp_directory_path() /
                           "phad_offline_vo_session_probe_b_on";
    std::filesystem::remove_all( probe_dir );
    std::filesystem::create_directories( probe_dir );
    const auto probe_path = probe_dir / "probe_b.jsonl";

    OfflineVoSessionOptions options;
    options.sequence_root = configured_path;
    options.max_frames    = 5;
    options.probe_b_path  = probe_path;

    const auto result = runOfflineVoSession( options );
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    EXPECT_EQ( result.counts.image_frames, 5U );
    ASSERT_TRUE( std::filesystem::exists( probe_path ) );

    std::ifstream in( probe_path );
    ASSERT_TRUE( in );
    std::size_t valid_lines = 0;
    std::string line;
    while ( std::getline( in, line ) )
    {
      if ( line.empty() )
      {
        continue;
      }
      EXPECT_EQ( line.front(), '{' );
      EXPECT_EQ( line.back(), '}' );
      EXPECT_NE( line.find( "\"i\":" ), std::string::npos );
      EXPECT_NE( line.find( "\"ts_ns\":" ), std::string::npos );
      ++valid_lines;
    }
    EXPECT_GE( valid_lines, 1U );
    EXPECT_EQ( valid_lines, result.counts.image_frames );

    std::filesystem::remove_all( probe_dir );
  }

  TEST( OfflineVoSessionTest, IllegalProbeBParentDirFailsSession )
  {
    // Writer open fails before the frame loop; TinyEuroc is enough.
    TinyEurocFixture fixture{ "probe_b_open_error" };
    const auto       probe_path =
        std::filesystem::temp_directory_path() /
        "phad_offline_vo_session_probe_b_missing_parent" / "nested" /
        "probe_b.jsonl";
    std::filesystem::remove_all( probe_path.parent_path().parent_path() );

    OfflineVoSessionOptions options;
    options.sequence_root = fixture.root();
    options.max_frames    = 5;
    options.probe_b_path  = probe_path;

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_NE( result.error->detail.find( "probe_b" ), std::string::npos );
    EXPECT_FALSE( std::filesystem::exists( probe_path ) );
  }

  TEST( OfflineVoSessionTest, MaxFramesLimitsCountsAndCoverageSpan )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    OfflineVoSessionOptions options;
    options.sequence_root = configured_path;
    options.max_frames    = 5;

    const auto result = runOfflineVoSession( options );
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 5U );
    EXPECT_EQ( result.counts.ok, result.trajectory->size() );
    EXPECT_EQ( result.diag.size(), 5U );
    EXPECT_LT( result.first_image_ts, result.last_image_ts );
    EXPECT_GT( result.wall_s, 0.0 );

    // A clean prefix has no initialization summary warning.
    // Cull counters may be non-zero but must not enter warnings.
    EXPECT_EQ( result.counts.reanchors, 0U );
    EXPECT_EQ( result.counts.seed_rejected, 0U );
    const bool has_summary_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo initialization summary:", 0 ) ==
                              0;
                     } );
    EXPECT_FALSE( has_summary_warning );
    const bool has_cull_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo outlier cull summary:", 0 ) ==
                              0;
                     } );
    EXPECT_FALSE( has_cull_warning );
  }

}  // namespace

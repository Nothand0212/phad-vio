#include "apps/offline_vo_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <string>

namespace
{

  using phad::apps::FrameCounts;
  using phad::apps::GyroStateRow;
  using phad::apps::OfflineVoSessionOptions;
  using phad::apps::runOfflineVoSession;
  using phad::apps::VoDiagRow;
  using phad::apps::writeDiagCsv;
  using phad::apps::writeGyroStateCsv;

  [[nodiscard]] std::string readFile( const std::filesystem::path& path )
  {
    std::ifstream      in( path );
    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
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

    TinyEurocFixture()
    {
      const ::testing::TestInfo* test_info =
          ::testing::UnitTest::GetInstance()->current_test_info();
      const std::string test_name =
          test_info == nullptr ? "unknown" : test_info->name();
      m_root = std::filesystem::temp_directory_path() /
               ( "phad_offline_vo_session_tiny_seq_" + test_name );
      std::filesystem::remove_all( m_root );
      for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
      {
        std::filesystem::create_directories( m_root / "mav0" / sensor /
                                             "data" );
      }
      writeCalibration();
      writeCsv( "imu0", imuHeader() );
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
  }

  TEST( OfflineVoSessionTest, SkipDropMinCulledDefaultsFour )
  {
    OfflineVoSessionOptions options;
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
    TinyEurocFixture fixture;
    fixture.corruptSecondRightImage();

    OfflineVoSessionOptions options;
    options.sequence_root = fixture.root();

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.counts.image_frames, 1U );

    // Mid-loop stream errors still run segment finalization, but a run
    // with no re-anchors and no seed-gate rejections is clean: no
    // "vo segments summary" warning should be emitted. Cull totals never
    // enter warnings either.
    EXPECT_EQ( result.counts.reanchors, 0U );
    EXPECT_EQ( result.counts.seed_rejected, 0U );
    const bool has_summary_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo segments summary:", 0 ) == 0;
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

  TEST( OfflineVoSessionTest, WriteDiagCsvMatchesProbeContract )
  {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_session_diag.csv";
    std::filesystem::remove( path );

    VoDiagRow row;
    row.timestamp_ns             = 1403636579763555584LL;
    row.status                   = "ok";
    row.num_observations         = 136;
    row.num_landmarks            = 0;
    row.num_shared               = 0;
    row.low_connectivity         = false;
    row.window_size              = 1;
    row.prior_key                = 0;
    row.reproj_rms_before_px     = 0.0;
    row.reproj_rms_after_px      = 0.0;
    row.num_cheirality           = 0;
    row.lm_iterations            = 0;
    row.max_window_pose_shift_m  = 0.0;
    row.segment_id               = 0;
    row.pnp_success              = false;
    row.pnp_inliers              = 0;
    row.outliers_culled          = 0;
    row.reproj_rms_after_cull_px = 0.0;

    ASSERT_FALSE( writeDiagCsv( path, { row } ).has_value() );

    std::ifstream in( path );
    ASSERT_TRUE( in );
    std::ostringstream oss;
    oss << in.rdbuf();
    const std::string text = oss.str();
    EXPECT_NE( text.find( "timestamp_ns,status,num_obs," ), std::string::npos );
    EXPECT_NE( text.find( "pnp_success,pnp_inliers,outliers_culled,"
                          "reproj_rms_after_cull_px" ),
               std::string::npos );
    // Slice ④e / Probe B keep the 19-column contract; Probe B is a
    // separate jsonl side-channel. outlier_reopt_rounds stay off diag
    // (session/summary only). culled_landmark_ids remain in-memory only
    // (session dropTracks; not a diag column). Slice ⑦'s
    // num_triangulated_seed column was dropped again post-gate (the
    // diagnostic never increments: triangulation seeding is disabled).
    // pre-M4 小片 (2026-08-07): num_disparity appended after is_keyframe →
    // 20-column contract.
    const auto header_end = text.find( '\n' );
    ASSERT_NE( header_end, std::string::npos );
    const std::string header = text.substr( 0, header_end );
    EXPECT_EQ( std::count( header.begin(), header.end(), ',' ), 19 );
    EXPECT_NE(
        text.find( "1403636579763555584,ok,136,0,0,0,1,0,0.000000,0.000000,0,"
                   "0,0.000000,0,0,0,0,0.000000,0" ),
        std::string::npos );
    std::filesystem::remove( path );
  }

  TEST( OfflineVoSessionTest, WriteGyroStateCsvUsesValidityFlagsAndWxyz )
  {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_gyro_state.csv";
    std::filesystem::remove( path );

    GyroStateRow row;
    row.timestamp_ns           = TinyEurocFixture::kFirstTimestampNs;
    row.status                 = "ok";
    row.segment_id             = 2;
    row.gyro.mode              = phad::estimator::GyroMode::kShadow;
    row.gyro.t_i_ns            = TinyEurocFixture::kFirstTimestampNs - 5'000'000;
    row.gyro.t_j_ns            = TinyEurocFixture::kFirstTimestampNs;
    row.gyro.visual_post_T_W_B = Eigen::Isometry3d::Identity();
    row.gyro.pim_delta_q_Bi_Bj =
        Eigen::Quaterniond( Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitZ() ) );
    row.gyro.gyro_factor_count = 3;
    row.gyro.visual_cost       = 1.25;

    ASSERT_FALSE( writeGyroStateCsv( path, { row } ).has_value() );
    const std::string text = readFile( path );
    EXPECT_NE( text.find( "ts_ns,mode,status,segment_id," ),
               std::string::npos );
    EXPECT_NE( text.find( "visual_post_valid,visual_post_tx," ),
               std::string::npos );
    EXPECT_NE( text.find( "visual_post_qw,visual_post_qx," ),
               std::string::npos );
    EXPECT_NE( text.find( "pim_delta_valid,pim_delta_qw," ),
               std::string::npos );
    EXPECT_NE( text.find( "gyro_cost_valid,gyro_cost" ),
               std::string::npos );
    EXPECT_NE( text.find( ",shadow,ok,2," ), std::string::npos );
    const auto split = []( const std::string& line ) {
      std::vector<std::string> fields;
      std::istringstream       stream( line );
      std::string              field;
      while ( std::getline( stream, field, ',' ) )
      {
        fields.push_back( field );
      }
      if ( !line.empty() && line.back() == ',' )
      {
        fields.emplace_back();
      }
      return fields;
    };
    std::istringstream lines( text );
    std::string        header_line;
    std::string        data_line;
    ASSERT_TRUE( std::getline( lines, header_line ) );
    ASSERT_TRUE( std::getline( lines, data_line ) );
    const auto header = split( header_line );
    const auto data   = split( data_line );
    ASSERT_EQ( header.size(), data.size() );
    const auto value = [ &header, &data ]( const std::string& name )
        -> const std::string& {
      const auto it = std::find( header.begin(), header.end(), name );
      EXPECT_NE( it, header.end() );
      return data[ static_cast<std::size_t>( it - header.begin() ) ];
    };
    EXPECT_EQ( value( "visual_init_valid" ), "0" );
    EXPECT_TRUE( value( "visual_init_tx" ).empty() );
    EXPECT_EQ( value( "visual_post_valid" ), "1" );
    EXPECT_EQ( value( "visual_post_qw" ), "1" );
    EXPECT_EQ( value( "pim_delta_valid" ), "1" );
    EXPECT_FALSE( value( "pim_delta_qw" ).empty() );
    EXPECT_EQ( value( "gyro_post_valid" ), "0" );
    EXPECT_TRUE( value( "gyro_post_tx" ).empty() );
    EXPECT_EQ( value( "visual_cost_valid" ), "1" );
    EXPECT_EQ( value( "gyro_cost_valid" ), "0" );
    std::filesystem::remove( path );
  }

  TEST( OfflineVoSessionTest, OffDoesNotCollectGyroRows )
  {
    TinyEurocFixture        fixture;
    OfflineVoSessionOptions options;
    options.sequence_root = fixture.root();
    options.max_frames    = 2;

    const auto result = runOfflineVoSession( options );
    EXPECT_TRUE( result.gyro_state.empty() );
  }

  TEST( OfflineVoSessionTest, ShadowCollectsOneGyroRowPerDiagRow )
  {
    TinyEurocFixture        fixture;
    OfflineVoSessionOptions options;
    options.sequence_root       = fixture.root();
    options.max_frames          = 2;
    options.estimator.gyro_mode = phad::estimator::GyroMode::kShadow;

    const auto result = runOfflineVoSession( options );
    ASSERT_EQ( result.gyro_state.size(), result.diag.size() );
    ASSERT_FALSE( result.gyro_state.empty() );
    for ( std::size_t index = 0; index < result.diag.size(); ++index )
    {
      EXPECT_EQ( result.gyro_state[ index ].timestamp_ns,
                 result.diag[ index ].timestamp_ns );
      EXPECT_EQ( result.gyro_state[ index ].status,
                 result.diag[ index ].status );
      EXPECT_EQ( result.gyro_state[ index ].segment_id,
                 result.diag[ index ].segment_id );
      EXPECT_EQ( result.gyro_state[ index ].gyro.mode,
                 phad::estimator::GyroMode::kShadow );
    }
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
    TinyEurocFixture fixture;
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

    // MH_01 is the clean-run reference: no re-anchors, no seed-gate
    // rejections, so no "vo segments summary" warning is expected.
    // Cull counters may be non-zero but must not enter warnings.
    EXPECT_EQ( result.counts.reanchors, 0U );
    EXPECT_EQ( result.counts.seed_rejected, 0U );
    const bool has_summary_warning =
        std::any_of( result.warnings.begin(), result.warnings.end(),
                     []( const std::string& warning ) {
                       return warning.rfind( "vo segments summary:", 0 ) == 0;
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

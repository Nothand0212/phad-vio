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
#include <string_view>
#include <utility>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  using phad::apps::FrameCounts;
  using phad::apps::OfflineVoSessionOptions;
  using phad::apps::runOfflineVoSession;
  using phad::apps::VoDiagRow;
  using phad::apps::writeDiagCsv;

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
        InitializingPrefixCompletesWithoutTrajectoryOrSessionError )
  {
    TinyEurocFixture fixture{ "initializing_prefix" };

    auto opened = phad::io::dataset::euroc::open( fixture.root() );
    if ( !opened )
    {
      FAIL() << "failed to open tiny euroc fixture: "
             << opened.error().describe();
    }

    OfflineVoSessionOptions options;
    options.max_frames = 2U;

    const auto result =
        runOfflineVoSession( std::move( opened ).value(), options );
    EXPECT_FALSE( result.error.has_value() );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 2U );
    EXPECT_EQ( result.diag.size(), 2U );
    EXPECT_EQ( result.counts.failed, 0U );
    EXPECT_EQ( result.sync.emitted_stereo, 2U );
    EXPECT_EQ( result.diag.at( 0 ).status, "initializing" );
    EXPECT_EQ( result.diag.at( 1 ).status, "initializing" );
  }

  TEST( OfflineVoSessionTest,
        StreamErrorMidLoopFinalizesCountsWithoutSpuriousSummaryWarning )
  {
    TinyEurocFixture fixture{ "stream_error" };
    fixture.corruptSecondRightImage();

    OfflineVoSessionOptions options;
    auto                    opened = phad::io::dataset::euroc::open( fixture.root() );
    if ( !opened )
    {
      FAIL() << "failed to open tiny euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.counts.image_frames, 1U );
    ASSERT_EQ( result.diag.size(), 1U );
    EXPECT_EQ( result.diag.front().num_retained_observations, 0U );
    EXPECT_EQ( result.diag.front().num_seeded_landmarks, 0U );
    EXPECT_EQ( result.diag.front().num_current_visual_factors, 0U );
    EXPECT_EQ( result.diag.front().num_current_mono_visual_factors, 0U );
    EXPECT_EQ( result.diag.front().unsupported_span_ns, 0 );

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

  TEST( OfflineVoSessionTest, GyroObserveCollectsPacketBeforeLaterStreamError )
  {
    TinyEurocFixture fixture{ "gyro_observe_stream_error" };
    fixture.corruptSecondRightImage();

    OfflineVoSessionOptions options;
    options.collect_gyro_observe = true;
    auto opened                  = phad::io::dataset::euroc::open( fixture.root() );
    if ( !opened )
    {
      FAIL() << "failed to open tiny euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
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
    std::ostringstream oss;
    oss << in.rdbuf();
    const std::string text = oss.str();
    EXPECT_NE( text.find( "timestamp_ns,status,num_obs," ), std::string::npos );
    EXPECT_NE( text.find( "pnp_success,pnp_inliers,outliers_culled,"
                          "reproj_rms_after_cull_px" ),
               std::string::npos );
    // Slice ④e / Probe B keep their existing contract; Probe B is a
    // separate jsonl side-channel. outlier_reopt_rounds stay off diag
    // (session/summary only). culled_landmark_ids remain in-memory only
    // (session dropTracks; not a diag column). Slice ⑦'s
    // num_triangulated_seed column was dropped again post-gate (the
    // diagnostic never increments: triangulation seeding is disabled).
    // Slice 1b appends continuity and committed-intake diagnostics after the
    // existing num_disparity column. Mapped-bearing diagnostics append the
    // mapped population and current mono-factor breakdown.
    const auto header_end = text.find( '\n' );
    ASSERT_NE( header_end, std::string::npos );
    const std::string header = text.substr( 0, header_end );
    EXPECT_EQ( std::count( header.begin(), header.end(), ',' ), 25 );
    EXPECT_NE(
        header.find(
            "num_disparity,unsupported_span_ns,num_retained_observations,"
            "num_seeded_landmarks,num_current_visual_factors,"
            "num_mapped_observations,num_current_mono_visual_factors" ),
        std::string::npos );
    EXPECT_NE(
        text.find( "1403636579763555584,ok,136,0,0,0,1,0,0.000000,0.000000,0,"
                   "0,0.000000,0,0,0,0,0.000000,0,91,250000000,73,11,9,2,4" ),
        std::string::npos );
    const auto outage_begin = text.find( ",visual_outage," );
    ASSERT_NE( outage_begin, std::string::npos );
    const auto outage_end = text.find( '\n', outage_begin );
    ASSERT_NE( outage_end, std::string::npos );
    const std::string outage_line =
        text.substr( outage_begin, outage_end - outage_begin );
    EXPECT_EQ( std::count( outage_line.begin(), outage_line.end(), ',' ),
               25 );
    EXPECT_EQ(
        outage_line.rfind( ",300000000,0,0,0,0,0" ),
        outage_line.size() -
            std::string{ ",300000000,0,0,0,0,0" }.size() );
    const auto discontinuity_begin = text.find( ",discontinuity," );
    ASSERT_NE( discontinuity_begin, std::string::npos );
    const auto discontinuity_end = text.find( '\n', discontinuity_begin );
    ASSERT_NE( discontinuity_end, std::string::npos );
    const std::string discontinuity_line =
        text.substr( discontinuity_begin,
                     discontinuity_end - discontinuity_begin );
    EXPECT_EQ( discontinuity_line.rfind( ",0,0,0,0,0,0" ),
               discontinuity_line.size() -
                   std::string{ ",0,0,0,0,0,0" }.size() );
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
    options.max_frames                          = 5;
    options.estimator.m_enable_moving_bootstrap = true;
    // Default: probe_b_path empty → writer not constructed.
    ASSERT_TRUE( options.probe_b_path.empty() );
    auto opened = phad::io::dataset::euroc::open( configured_path );
    if ( !opened )
    {
      FAIL() << "failed to open Euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
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
    options.max_frames                          = 5;
    options.probe_b_path                        = probe_path;
    options.estimator.m_enable_moving_bootstrap = true;
    auto opened                                 = phad::io::dataset::euroc::open( configured_path );
    if ( !opened )
    {
      FAIL() << "failed to open Euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
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
    options.max_frames   = 5;
    options.probe_b_path = probe_path;
    auto opened          = phad::io::dataset::euroc::open( fixture.root() );
    if ( !opened )
    {
      FAIL() << "failed to open tiny euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
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
    options.max_frames                          = 5;
    options.estimator.m_enable_moving_bootstrap = true;
    auto opened                                 = phad::io::dataset::euroc::open( configured_path );
    if ( !opened )
    {
      FAIL() << "failed to open Euroc fixture: "
             << opened.error().describe();
    }

    const auto result = runOfflineVoSession( std::move( opened ).value(), options );
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

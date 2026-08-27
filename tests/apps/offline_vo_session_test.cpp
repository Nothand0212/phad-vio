#include "apps/offline_vo_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "phad/sensor/stereo_frame.hpp"
#include "tests/apps/synthetic_euroc_fixture.hpp"
#include "tests/frontend/synthetic_stereo.hpp"

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

    TinyEurocFixture()
    {
      m_root = std::filesystem::temp_directory_path() /
               "phad_offline_vo_session_tiny_seq";
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

  TEST( OfflineVoSessionTest, VioStateProbeRejectsDisabledImuBeforeOpen )
  {
    OfflineVoSessionOptions options;
    options.sequence_root =
        std::filesystem::temp_directory_path() / "phad_missing_euroc_seq";
    options.estimator.enable_imu = false;
    options.vio_state_probe_path =
        std::filesystem::temp_directory_path() / "phad_vio_state.csv";

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_NE( result.error->detail.find( "requires estimator.enable_imu" ),
               std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 0U );
  }

  TEST( OfflineVoSessionTest, VioInitProbeRejectsDisabledImuBeforeOpen )
  {
    OfflineVoSessionOptions options;
    options.sequence_root =
        std::filesystem::temp_directory_path() / "phad_missing_euroc_seq";
    options.estimator.enable_imu = false;
    options.vio_init_probe_path =
        std::filesystem::temp_directory_path() / "phad_vio_init.csv";

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_NE( result.error->detail.find( "VIO init probe" ),
               std::string::npos );
    EXPECT_NE( result.error->detail.find( "requires estimator.enable_imu" ),
               std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 0U );
  }

  TEST( OfflineVoSessionTest,
        FixedLagShadowProbeRejectsDisabledImuBeforeOpen )
  {
    OfflineVoSessionOptions options;
    options.sequence_root =
        std::filesystem::temp_directory_path() / "phad_missing_euroc_seq";
    options.estimator.enable_imu = false;
    options.fixed_lag_shadow_probe_path =
        std::filesystem::temp_directory_path() /
        "phad_fixed_lag_shadow.csv";

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_NE( result.error->detail.find( "fixed-lag shadow probe" ),
               std::string::npos );
    EXPECT_NE( result.error->detail.find( "requires estimator.enable_imu" ),
               std::string::npos );
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
    // M4.3 (Q3/C12, plan F): bias 三轴 6 列追加在末尾 → 26-column contract。
    // 原列顺序与写入格式不动 (IMU-off 时新列恒 0.0, 原列逐字节回归)。
    const auto header_end = text.find( '\n' );
    ASSERT_NE( header_end, std::string::npos );
    const std::string header = text.substr( 0, header_end );
    EXPECT_EQ( std::count( header.begin(), header.end(), ',' ), 25 );
    EXPECT_NE(
        text.find( "1403636579763555584,ok,136,0,0,0,1,0,0.000000,0.000000,0,"
                   "0,0.000000,0,0,0,0,0.000000,0,0,0.000000,0.000000,"
                   "0.000000,0.000000,0.000000,0.000000" ),
        std::string::npos );
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

  void expectSameTrajectory(
      const std::optional<phad::common::Trajectory>& a,
      const std::optional<phad::common::Trajectory>& b )
  {
    EXPECT_EQ( a.has_value(), b.has_value() );
    if ( !a.has_value() )
    {
      return;
    }
    ASSERT_TRUE( b.has_value() );
    const auto& poses_a = a->poses();
    const auto& poses_b = b->poses();
    ASSERT_EQ( poses_a.size(), poses_b.size() );
    for ( std::size_t i = 0; i < poses_a.size(); ++i )
    {
      EXPECT_EQ( poses_a[ i ].timestamp, poses_b[ i ].timestamp );
      EXPECT_TRUE( poses_a[ i ].T_W_B.matrix().isApprox(
          poses_b[ i ].T_W_B.matrix(), 1e-12 ) );
    }
  }

  void expectSameDiagRow( const VoDiagRow& a, const VoDiagRow& b )
  {
    EXPECT_EQ( a.timestamp_ns, b.timestamp_ns );
    EXPECT_EQ( a.status, b.status );
    EXPECT_EQ( a.num_observations, b.num_observations );
    EXPECT_EQ( a.num_landmarks, b.num_landmarks );
    EXPECT_EQ( a.num_shared, b.num_shared );
    EXPECT_EQ( a.num_disparity, b.num_disparity );
    EXPECT_EQ( a.low_connectivity, b.low_connectivity );
    EXPECT_EQ( a.window_size, b.window_size );
    EXPECT_EQ( a.prior_key, b.prior_key );
    EXPECT_EQ( a.reproj_rms_before_px, b.reproj_rms_before_px );
    EXPECT_EQ( a.reproj_rms_after_px, b.reproj_rms_after_px );
    EXPECT_EQ( a.num_cheirality, b.num_cheirality );
    EXPECT_EQ( a.lm_iterations, b.lm_iterations );
    EXPECT_EQ( a.max_window_pose_shift_m, b.max_window_pose_shift_m );
    EXPECT_EQ( a.segment_id, b.segment_id );
    EXPECT_EQ( a.pnp_success, b.pnp_success );
    EXPECT_EQ( a.pnp_inliers, b.pnp_inliers );
    EXPECT_EQ( a.outliers_culled, b.outliers_culled );
    EXPECT_EQ( a.reproj_rms_after_cull_px, b.reproj_rms_after_cull_px );
    EXPECT_EQ( a.is_keyframe, b.is_keyframe );
    EXPECT_EQ( a.bias_gyro_x, b.bias_gyro_x );
    EXPECT_EQ( a.bias_gyro_y, b.bias_gyro_y );
    EXPECT_EQ( a.bias_gyro_z, b.bias_gyro_z );
    EXPECT_EQ( a.bias_acc_x, b.bias_acc_x );
    EXPECT_EQ( a.bias_acc_y, b.bias_acc_y );
    EXPECT_EQ( a.bias_acc_z, b.bias_acc_z );
  }

  TEST( OfflineVoSessionTest, CandidateSwitchKeepsProductionArtifactsIdentical )
  {
    phad::testing::SyntheticEurocFixture fixture;

    OfflineVoSessionOptions baseline_options;
    baseline_options.sequence_root = fixture.root();
    baseline_options.max_frames    = 5;
    // 合成全同 Gaussian blob：唯一性/双向检查无区分度，与
    // candidate_pipeline_test 的 trackerOptions 一致（synthetic 专属）。
    baseline_options.tracker.stereo_uniq_ratio  = 0.0;
    baseline_options.tracker.stereo_check_bidir = false;

    const auto baseline = runOfflineVoSession( baseline_options );
    ASSERT_FALSE( baseline.error.has_value() ) << baseline.error->detail;

    OfflineVoSessionOptions candidate_options = baseline_options;
    candidate_options.enable_candidate        = true;
    const auto with_candidate                 = runOfflineVoSession( candidate_options );
    ASSERT_FALSE( with_candidate.error.has_value() )
        << with_candidate.error->detail;

    // candidate 开关绝不改变 production 主产物或错误。
    expectSameTrajectory( baseline.trajectory, with_candidate.trajectory );
    expectSameTrajectory( baseline.kf_trajectory,
                          with_candidate.kf_trajectory );
    ASSERT_EQ( baseline.diag.size(), with_candidate.diag.size() );
    for ( std::size_t i = 0; i < baseline.diag.size(); ++i )
    {
      expectSameDiagRow( baseline.diag[ i ], with_candidate.diag[ i ] );
    }
    EXPECT_EQ( baseline.counts.image_frames,
               with_candidate.counts.image_frames );
    EXPECT_EQ( baseline.counts.ok, with_candidate.counts.ok );
    EXPECT_EQ( baseline.counts.rejected, with_candidate.counts.rejected );
    EXPECT_EQ( baseline.counts.failed, with_candidate.counts.failed );
    EXPECT_EQ( baseline.counts.total_keyframes,
               with_candidate.counts.total_keyframes );
    EXPECT_EQ( baseline.warnings.size(), with_candidate.warnings.size() );
    for ( std::size_t i = 0; i < baseline.warnings.size(); ++i )
    {
      EXPECT_EQ( baseline.warnings[ i ], with_candidate.warnings[ i ] );
    }

    // P2b: candidate 不再是逐位 twin——fixed-lag 是独立估计器，精度与
    // production 允许不同。结构断言：candidate 无 terminal error、
    // trajectory 与送入帧数一致、timestamp 严格递增、estimate 全 finite；
    // 精度对比不属于 P2b structural gate。
    EXPECT_FALSE( baseline.candidate.has_value() );
    EXPECT_EQ( baseline.candidate_wall_s, 0.0 );
    ASSERT_TRUE( with_candidate.candidate.has_value() );
    ASSERT_FALSE( with_candidate.candidate->error.has_value() )
        << with_candidate.candidate->error->detail;
    EXPECT_GT( with_candidate.candidate_wall_s, 0.0 );
    ASSERT_TRUE( with_candidate.candidate->trajectory.has_value() );
    EXPECT_EQ( with_candidate.candidate->trajectory->size(),
               with_candidate.counts.image_frames );
    const auto& candidate_poses =
        with_candidate.candidate->trajectory->poses();
    for ( std::size_t i = 1; i < candidate_poses.size(); ++i )
    {
      EXPECT_GT( candidate_poses[ i ].timestamp,
                 candidate_poses[ i - 1 ].timestamp );
      EXPECT_TRUE( candidate_poses[ i ].T_W_B.matrix().allFinite() );
    }
    ASSERT_EQ( with_candidate.candidate->diagnostics.size(),
               with_candidate.counts.image_frames );
    EXPECT_EQ( with_candidate.candidate->counts.failed, 0U );
  }

}  // namespace

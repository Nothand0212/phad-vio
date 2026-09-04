#include "apps/offline_vo_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <numbers>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "apps/internal/offline_vo_session_keyframe.hpp"
#include "apps/probe_b_writer.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::apps::FrameCounts;
  using phad::apps::OfflineVoSessionOptions;
  using phad::apps::ProbeBFrame;
  using phad::apps::ProbeBWriter;
  using phad::apps::runOfflineVoSession;
  using phad::apps::VoDiagRow;
  using phad::apps::writeDiagCsv;

  [[nodiscard]] std::string readTextFile(
      const std::filesystem::path& path )
  {
    std::ifstream input( path, std::ios::binary );
    EXPECT_TRUE( input ) << path;
    return { std::istreambuf_iterator<char>( input ),
             std::istreambuf_iterator<char>() };
  }

  void expectFrameCountsEqual( const FrameCounts& lhs,
                               const FrameCounts& rhs )
  {
    EXPECT_EQ( lhs.image_frames, rhs.image_frames );
    EXPECT_EQ( lhs.ok, rhs.ok );
    EXPECT_EQ( lhs.rejected, rhs.rejected );
    EXPECT_EQ( lhs.failed, rhs.failed );
    EXPECT_EQ( lhs.low_connectivity, rhs.low_connectivity );
    EXPECT_EQ( lhs.segments, rhs.segments );
    EXPECT_EQ( lhs.reanchors, rhs.reanchors );
    EXPECT_EQ( lhs.seed_rejected, rhs.seed_rejected );
    EXPECT_EQ( lhs.pnp_successes, rhs.pnp_successes );
    EXPECT_EQ( lhs.pnp_fallbacks, rhs.pnp_fallbacks );
    EXPECT_EQ( lhs.outliers_culled, rhs.outliers_culled );
    EXPECT_EQ( lhs.outliers_culled_unique, rhs.outliers_culled_unique );
    EXPECT_EQ( lhs.outlier_reopts, rhs.outlier_reopts );
    EXPECT_EQ( lhs.drops_skipped, rhs.drops_skipped );
    EXPECT_EQ( lhs.deferred_drops, rhs.deferred_drops );
    EXPECT_EQ( lhs.deferred_drop_ids, rhs.deferred_drop_ids );
    EXPECT_EQ( lhs.evictable_marked, rhs.evictable_marked );
    EXPECT_EQ( lhs.tracks_evicted, rhs.tracks_evicted );
    EXPECT_EQ( lhs.zombie_age_drops, rhs.zombie_age_drops );
    EXPECT_EQ( lhs.zombie_age_drop_ids, rhs.zombie_age_drop_ids );
    EXPECT_EQ( lhs.total_keyframes, rhs.total_keyframes );
    EXPECT_EQ( lhs.total_track_only_frames, rhs.total_track_only_frames );
    EXPECT_EQ( lhs.non_keyframe_evictions, rhs.non_keyframe_evictions );
    EXPECT_EQ( lhs.imu_reintegrations, rhs.imu_reintegrations );
  }

  void expectTrajectoryEqual(
      const std::optional<phad::common::Trajectory>& lhs,
      const std::optional<phad::common::Trajectory>& rhs )
  {
    ASSERT_EQ( lhs.has_value(), rhs.has_value() );
    if ( !lhs.has_value() )
    {
      return;
    }
    ASSERT_EQ( lhs->size(), rhs->size() );
    for ( std::size_t index = 0U; index < lhs->size(); ++index )
    {
      const auto& lhs_pose = lhs->poses()[ index ];
      const auto& rhs_pose = rhs->poses()[ index ];
      EXPECT_EQ( lhs_pose.timestamp, rhs_pose.timestamp ) << "index=" << index;
      EXPECT_TRUE( ( lhs_pose.T_W_B.matrix().array() ==
                     rhs_pose.T_W_B.matrix().array() )
                       .all() )
          << "index=" << index;
    }
  }

  [[nodiscard]] phad::frontend::FrameTracks makeAcceptedTracks(
      const std::int64_t timestamp_ns )
  {
    phad::frontend::FrameTracks tracks;
    tracks.timestamp = phad::common::Timestamp{ timestamp_ns };
    for ( std::uint64_t index = 0U; index < 10U; ++index )
    {
      tracks.observations.push_back( phad::frontend::TrackObservation{
          .id         = index + 1U,
          .left_pixel = Eigen::Vector2d(
              120.0 + 8.0 * static_cast<double>( index ),
              100.0 + 4.0 * static_cast<double>( index ) ),
          .disparity_px = 8.0,
          .status       = phad::frontend::StereoStatus::kValid,
          .length       = 1U } );
    }
    return tracks;
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

  TEST( OfflineVoSessionTest, AcceptedSegmentTransitionsCountReanchorsExactly )
  {
    FrameCounts                            counts;
    std::optional<std::uint32_t>           last_segment_id;
    constexpr std::array<std::uint32_t, 5> kAcceptedSegmentIds{ 0U, 0U, 1U,
                                                                1U, 2U };

    for ( const std::uint32_t segment_id : kAcceptedSegmentIds )
    {
      phad::apps::internal::recordAcceptedSegment(
          segment_id, last_segment_id, counts );
    }

    EXPECT_EQ( counts.segments, 3U );
    EXPECT_EQ( counts.reanchors, 2U );
    ASSERT_TRUE( last_segment_id.has_value() );
    EXPECT_EQ( *last_segment_id, 2U );
  }

  TEST( OfflineVoSessionTest, RepeatedAcceptedPosesInOneSegmentDoNotReanchor )
  {
    FrameCounts                  counts;
    std::optional<std::uint32_t> last_segment_id;

    phad::apps::internal::recordAcceptedSegment( 0U, last_segment_id,
                                                 counts );
    phad::apps::internal::recordAcceptedSegment( 0U, last_segment_id,
                                                 counts );

    EXPECT_EQ( counts.segments, 1U );
    EXPECT_EQ( counts.reanchors, 0U );
  }

  TEST( OfflineVoSessionTest, ReanchorAppendsCanonicalSegmentSummaryWarning )
  {
    FrameCounts              counts;
    std::vector<std::string> warnings{ "stream warning" };
    counts.segments  = 3U;
    counts.reanchors = 2U;

    phad::apps::internal::appendSegmentSummaryWarning( counts, warnings );

    ASSERT_EQ( warnings.size(), 2U );
    EXPECT_EQ( warnings.front(), "stream warning" );
    EXPECT_EQ( warnings.back(),
               "vo segments summary: segments=3 reanchors=2 "
               "seed_rejected=0" );
  }

  TEST( OfflineVoSessionTest, SeedRejectionAppendsSegmentSummaryWarning )
  {
    FrameCounts              counts;
    std::vector<std::string> warnings;
    counts.segments      = 1U;
    counts.seed_rejected = 1U;

    phad::apps::internal::appendSegmentSummaryWarning( counts, warnings );

    ASSERT_EQ( warnings.size(), 1U );
    EXPECT_EQ( warnings.front(),
               "vo segments summary: segments=1 reanchors=0 "
               "seed_rejected=1" );
  }

  TEST( OfflineVoSessionTest,
        CleanSingleSegmentDoesNotAppendSegmentSummaryWarning )
  {
    FrameCounts              counts;
    std::vector<std::string> warnings;
    counts.segments = 1U;

    phad::apps::internal::appendSegmentSummaryWarning( counts, warnings );

    EXPECT_TRUE( warnings.empty() );
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
    const auto& first_initialization = result.diag.at( 0 ).initialization;
    EXPECT_EQ( first_initialization.m_path,
               phad::estimator::InitializationPath::kStatic );
    EXPECT_EQ( first_initialization.m_phase,
               phad::estimator::InitializationPhase::kCollectingEvidence );
    EXPECT_EQ(
        first_initialization.m_reason,
        phad::estimator::InitializationReason::kStaticEvidenceInsufficient );
    EXPECT_EQ( first_initialization.m_outcome,
               phad::estimator::InitializationOutcome::kPending );
    EXPECT_FALSE( first_initialization.m_attempt_id.has_value() );
    EXPECT_EQ(
        first_initialization.m_transaction,
        phad::estimator::InitializationTransactionResult::kEvidenceCommitted );

    const auto& initialization = result.diag.at( 1 ).initialization;
    EXPECT_EQ( initialization.m_path,
               phad::estimator::InitializationPath::kNone );
    EXPECT_EQ( initialization.m_phase,
               phad::estimator::InitializationPhase::kInactive );
    EXPECT_EQ( initialization.m_reason,
               phad::estimator::InitializationReason::kNone );
    EXPECT_EQ( initialization.m_outcome,
               phad::estimator::InitializationOutcome::kNotActive );
    EXPECT_FALSE( initialization.m_attempt_id.has_value() );
    EXPECT_EQ( initialization.m_current_graph,
               phad::estimator::InitializationGraphResult::kNotEvaluated );
    EXPECT_EQ(
        initialization.m_transaction,
        phad::estimator::InitializationTransactionResult::kNotEvaluated );
  }

  TEST( OfflineVoSessionTest,
        EstimatorFatalStatusPreservesTypedPartialSessionEvidence )
  {
    for ( const auto status : { phad::estimator::UpdateStatus::kInvalidInput,
                                phad::estimator::UpdateStatus::kFailed } )
    {
      const std::string status_name =
          status == phad::estimator::UpdateStatus::kInvalidInput
              ? "invalid_input"
              : "failed";
      TinyEurocFixture fixture{ "estimator_" + status_name };
      const auto       probe_path = fixture.root() / "probe-b.jsonl";

      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      if ( !opened )
      {
        FAIL() << "failed to open tiny euroc fixture: "
               << opened.error().describe();
      }

      OfflineVoSessionOptions options;
      options.max_frames         = 2U;
      options.probe_b_path       = probe_path;
      std::uint32_t update_count = 0U;
      const auto    result =
          phad::apps::internal::runOfflineVoSessionWithEstimatorUpdate(
              std::move( opened ).value(), options,
              [ &update_count, status ](
                  const phad::estimator::VioMeasurement& measurement,
                  bool ) {
                phad::estimator::VioUpdateResult update;
                if ( update_count++ == 0U )
                {
                  update.status = phad::estimator::UpdateStatus::kOk;
                  phad::estimator::VioEstimate estimate;
                  estimate.timestamp = measurement.m_timestamp;
                  estimate.T_W_B     = Eigen::Isometry3d::Identity();
                  update.estimate    = estimate;
                }
                else
                {
                  update.status  = status;
                  update.message = "injected terminal estimator result";
                }
                return update;
              } );

      ASSERT_TRUE( result.error.has_value() );
      ASSERT_TRUE( result.error->estimator_status.has_value() );
      EXPECT_EQ( *result.error->estimator_status, status );
      ASSERT_TRUE( result.error->timestamp.has_value() );
      EXPECT_EQ( result.error->timestamp->nanoseconds(),
                 TinyEurocFixture::kSecondTimestampNs );
      EXPECT_EQ( result.counts.image_frames, 2U );
      EXPECT_EQ( result.counts.ok, 1U );
      EXPECT_EQ( result.counts.failed, 1U );
      ASSERT_EQ( result.diag.size(), 2U );
      EXPECT_EQ( result.diag.front().status, "ok" );
      EXPECT_EQ( result.diag.back().status, status_name );
      ASSERT_TRUE( result.trajectory.has_value() );
      ASSERT_EQ( result.trajectory->size(), 1U );
      EXPECT_EQ( result.trajectory->firstTimestamp().nanoseconds(),
                 TinyEurocFixture::kFirstTimestampNs );
      EXPECT_EQ( result.sync.emitted_stereo, 2U );

      std::istringstream probe_stream( readTextFile( probe_path ) );
      std::string        probe_line;
      std::uint32_t      probe_rows = 0U;
      while ( std::getline( probe_stream, probe_line ) )
      {
        const auto probe = nlohmann::json::parse( probe_line );
        EXPECT_EQ( probe.at( "i" ), probe_rows );
        EXPECT_EQ( probe.at( "ts_ns" ),
                   result.diag.at( probe_rows ).timestamp_ns );
        ++probe_rows;
      }
      EXPECT_EQ( probe_rows, 2U );
    }
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
    row.initialization.m_path =
        phad::estimator::InitializationPath::kMoving;
    row.initialization.m_phase =
        phad::estimator::InitializationPhase::kCommit;
    row.initialization.m_outcome =
        phad::estimator::InitializationOutcome::kCommitted;

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
    EXPECT_EQ( header.find( "initialization" ), std::string::npos );
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

  TEST( OfflineVoSessionTest, ProbeBWriterSerializesPnpAndGraphDiagnostics )
  {
    const auto probe_dir = std::filesystem::temp_directory_path() /
                           "phad_offline_vo_session_probe_b_diagnostics";
    std::filesystem::remove_all( probe_dir );
    std::filesystem::create_directories( probe_dir );
    const auto probe_path = probe_dir / "probe_b.jsonl";

    ProbeBFrame frame;
    frame.i             = 17U;
    frame.ts_ns         = 123'456'789;
    frame.pnp_attempted = true;
    frame.pnp_result =
        phad::estimator::PnpProbeResult::kAccepted;
    frame.pnp_correspondences                    = 23U;
    frame.pnp_inliers                            = 19U;
    frame.pnp_proposal_rms_px                    = 0.125;
    frame.pnp_guess_rms_px                       = 0.75;
    frame.pnp_proposal_inlier_count              = 19U;
    frame.pnp_proposal_cheiral_inlier_count      = 2U;
    frame.pnp_proposal_min_inlier_depth_m        = 1.25;
    frame.pnp_proposal_min_inlier_landmark_id    = 42U;
    frame.pnp_proposal_guess_translation_delta_m = 0.03125;
    frame.pnp_proposal_guess_rotation_delta_rad  = 0.0625;
    frame.pnp_guess_retry_outcome =
        phad::estimator::PnpGuessRetryOutcome::kWouldPassExistingPnpGates;
    frame.pnp_guess_retry_proposal_rms_px      = 0.15;
    frame.pnp_guess_retry_guess_rms_px         = 0.25;
    frame.pnp_guess_retry_cheiral_inlier_count = 0U;
    frame.pnp_guess_retry_min_inlier_depth_m   = 1.75;
    frame.pnp_guess_retry_translation_delta_m  = 0.04;
    frame.pnp_guess_retry_rotation_delta_rad   = 0.05;
    frame.current_pose_translation_delta_m     = 0.01;
    frame.current_pose_rotation_delta_rad      = 0.02;
    frame.current_velocity_delta_mps           = 0.03;
    frame.current_acc_bias_delta_mps2          = 0.04;
    frame.current_gyr_bias_delta_radps         = 0.05;
    frame.primary_graph_error_seed             = 12.5;
    frame.primary_graph_error_optimized        = 3.25;
    frame.primary_imu_factor_error_seed        = 2.5;
    frame.primary_imu_factor_error_optimized   = 0.625;
    {
      ProbeBWriter writer( probe_path );
      writer.write( frame );
    }

    std::ifstream input( probe_path );
    ASSERT_TRUE( input );
    std::string line;
    ASSERT_TRUE( std::getline( input, line ) );
    const nlohmann::json json = nlohmann::json::parse( line );
    EXPECT_EQ( json.at( "i" ).get<std::uint64_t>(), 17U );
    EXPECT_EQ( json.at( "ts_ns" ).get<std::int64_t>(), 123'456'789 );
    EXPECT_TRUE( json.at( "pnp_attempted" ).get<bool>() );
    EXPECT_EQ( json.at( "pnp_result" ).get<std::string>(), "accepted" );
    EXPECT_EQ(
        json.at( "pnp_correspondences" ).get<std::uint32_t>(), 23U );
    EXPECT_EQ( json.at( "pnp_inliers" ).get<std::uint32_t>(), 19U );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_proposal_rms_px" ).get<double>(), 0.125 );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_rms_px" ).get<double>(), 0.75 );
    EXPECT_EQ(
        json.at( "pnp_proposal_inlier_count" ).get<std::uint32_t>(),
        19U );
    EXPECT_EQ(
        json.at( "pnp_proposal_cheiral_inlier_count" )
            .get<std::uint32_t>(),
        2U );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_proposal_min_inlier_depth_m" ).get<double>(),
        1.25 );
    EXPECT_EQ(
        json.at( "pnp_proposal_min_inlier_landmark_id" )
            .get<std::uint64_t>(),
        42U );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_proposal_guess_translation_delta_m" ).get<double>(),
        0.03125 );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_proposal_guess_rotation_delta_rad" ).get<double>(),
        0.0625 );
    EXPECT_EQ(
        json.at( "pnp_guess_retry_outcome" ).get<std::string>(),
        "would_pass_existing_pnp_gates" );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_retry_proposal_rms_px" ).get<double>(),
        0.15 );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_retry_guess_rms_px" ).get<double>(), 0.25 );
    EXPECT_EQ(
        json.at( "pnp_guess_retry_cheiral_inlier_count" )
            .get<std::uint32_t>(),
        0U );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_retry_min_inlier_depth_m" ).get<double>(),
        1.75 );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_retry_translation_delta_m" ).get<double>(),
        0.04 );
    EXPECT_DOUBLE_EQ(
        json.at( "pnp_guess_retry_rotation_delta_rad" ).get<double>(),
        0.05 );
    EXPECT_DOUBLE_EQ(
        json.at( "current_pose_translation_delta_m" ).get<double>(), 0.01 );
    EXPECT_DOUBLE_EQ(
        json.at( "current_pose_rotation_delta_rad" ).get<double>(), 0.02 );
    EXPECT_DOUBLE_EQ(
        json.at( "current_velocity_delta_mps" ).get<double>(), 0.03 );
    EXPECT_DOUBLE_EQ(
        json.at( "current_acc_bias_delta_mps2" ).get<double>(), 0.04 );
    EXPECT_DOUBLE_EQ(
        json.at( "current_gyr_bias_delta_radps" ).get<double>(), 0.05 );
    EXPECT_DOUBLE_EQ(
        json.at( "primary_graph_error_seed" ).get<double>(), 12.5 );
    EXPECT_DOUBLE_EQ(
        json.at( "primary_graph_error_optimized" ).get<double>(), 3.25 );
    EXPECT_DOUBLE_EQ(
        json.at( "primary_imu_factor_error_seed" ).get<double>(), 2.5 );
    EXPECT_DOUBLE_EQ(
        json.at( "primary_imu_factor_error_optimized" ).get<double>(),
        0.625 );

    std::filesystem::remove_all( probe_dir );
  }

  TEST( OfflineVoSessionTest, ProbeBWriterSerializesEveryPnpResultToken )
  {
    using phad::estimator::PnpProbeResult;
    constexpr std::array<std::pair<PnpProbeResult, std::string_view>, 10>
        kExpected{
            std::pair{ PnpProbeResult::kNotAttempted, "not_attempted" },
            std::pair{ PnpProbeResult::kRansacFailed, "ransac_failed" },
            std::pair{ PnpProbeResult::kInsufficientInliers,
                       "insufficient_inliers" },
            std::pair{ PnpProbeResult::kProposalPoseNonFinite,
                       "proposal_pose_nonfinite" },
            std::pair{
                PnpProbeResult::kProposalRmsInvalidCorrespondence,
                "proposal_rms_invalid_correspondence" },
            std::pair{ PnpProbeResult::kProposalRmsCheirality,
                       "proposal_rms_cheirality" },
            std::pair{ PnpProbeResult::kProposalRmsNonFinite,
                       "proposal_rms_nonfinite" },
            std::pair{ PnpProbeResult::kProposalRmsRejected,
                       "proposal_rms_rejected" },
            std::pair{ PnpProbeResult::kAccepted, "accepted" },
            std::pair{ PnpProbeResult::kAcceptedAfterGuessRetry,
                       "accepted_after_guess_retry" },
        };
    const auto path = std::filesystem::temp_directory_path() /
                      "phad_probe_b_pnp_result_tokens.jsonl";
    std::filesystem::remove( path );
    {
      ProbeBWriter writer( path );
      for ( std::size_t index = 0U; index < kExpected.size(); ++index )
      {
        ProbeBFrame frame;
        frame.i          = static_cast<std::uint64_t>( index );
        frame.ts_ns      = static_cast<std::int64_t>( index );
        frame.pnp_result = kExpected[ index ].first;
        writer.write( frame );
      }
    }

    std::ifstream input( path );
    ASSERT_TRUE( input );
    std::string line;
    for ( const auto& [ result, token ] : kExpected )
    {
      (void)result;
      ASSERT_TRUE( std::getline( input, line ) );
      const nlohmann::json json = nlohmann::json::parse( line );
      EXPECT_EQ( json.at( "pnp_result" ), token );
    }
    EXPECT_FALSE( std::getline( input, line ) );
    std::filesystem::remove( path );
  }

  TEST( OfflineVoSessionTest,
        ProbeBWriterSerializesEveryPnpGuessRetryOutcomeToken )
  {
    using phad::estimator::PnpGuessRetryOutcome;
    constexpr std::array<
        std::pair<PnpGuessRetryOutcome, std::string_view>, 8>
        kExpected{
            std::pair{ PnpGuessRetryOutcome::kNotAttempted,
                       "not_attempted" },
            std::pair{ PnpGuessRetryOutcome::kSolveFailed,
                       "solve_failed" },
            std::pair{ PnpGuessRetryOutcome::kProposalPoseNonFinite,
                       "proposal_pose_nonfinite" },
            std::pair{
                PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence,
                "proposal_rms_invalid_correspondence" },
            std::pair{ PnpGuessRetryOutcome::kProposalRmsCheirality,
                       "proposal_rms_cheirality" },
            std::pair{ PnpGuessRetryOutcome::kProposalRmsNonFinite,
                       "proposal_rms_nonfinite" },
            std::pair{ PnpGuessRetryOutcome::kProposalRmsRejected,
                       "proposal_rms_rejected" },
            std::pair{
                PnpGuessRetryOutcome::kWouldPassExistingPnpGates,
                "would_pass_existing_pnp_gates" },
        };
    const auto path = std::filesystem::temp_directory_path() /
                      "phad_probe_b_pnp_guess_retry_outcome_tokens.jsonl";
    std::filesystem::remove( path );
    {
      ProbeBWriter writer( path );
      for ( std::size_t index = 0U; index < kExpected.size(); ++index )
      {
        ProbeBFrame frame;
        frame.i                       = static_cast<std::uint64_t>( index );
        frame.ts_ns                   = static_cast<std::int64_t>( index );
        frame.pnp_guess_retry_outcome = kExpected[ index ].first;
        writer.write( frame );
      }
    }

    std::ifstream input( path );
    ASSERT_TRUE( input );
    std::string line;
    for ( const auto& [ outcome, token ] : kExpected )
    {
      (void)outcome;
      ASSERT_TRUE( std::getline( input, line ) );
      const nlohmann::json json = nlohmann::json::parse( line );
      EXPECT_EQ( json.at( "pnp_guess_retry_outcome" ), token );
    }
    EXPECT_FALSE( std::getline( input, line ) );
    std::filesystem::remove( path );
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

  TEST( OfflineVoSessionTest, Mh01FormalMovingRootCommitsOnSeventhPacket )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    OfflineVoSessionOptions options;
    options.max_frames                          = 7;
    options.estimator.m_enable_moving_bootstrap = true;
    auto opened                                 = phad::io::dataset::euroc::open( configured_path );
    if ( !opened )
    {
      FAIL() << "failed to open Euroc fixture: "
             << opened.error().describe();
    }

    const auto         result = runOfflineVoSession( std::move( opened ).value(), options );
    std::ostringstream trace;
    trace << "error="
          << ( result.error.has_value() ? result.error->detail : "none" )
          << "; image_frames=" << result.counts.image_frames
          << "; ok=" << result.counts.ok
          << "; rejected=" << result.counts.rejected
          << "; failed=" << result.counts.failed;
    for ( std::size_t index = 0U; index < result.diag.size(); ++index )
    {
      const VoDiagRow& row = result.diag[ index ];
      trace << "; row[" << index << "]=" << row.status
            << "/path=" << static_cast<int>( row.initialization.m_path )
            << "/phase=" << static_cast<int>( row.initialization.m_phase )
            << "/reason=" << static_cast<int>( row.initialization.m_reason )
            << "/outcome=" << static_cast<int>( row.initialization.m_outcome )
            << "/attempt="
            << ( row.initialization.m_attempt_id.has_value() ? 1 : 0 )
            << "/transaction="
            << static_cast<int>( row.initialization.m_transaction )
            << "/poses=" << row.initialization.m_connected_poses.m_actual
            << "/duration_ns=" << row.initialization.m_duration.m_actual_ns
            << "/disparity=" << row.num_disparity;
    }
    SCOPED_TRACE( trace.str() );
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 7U );
    EXPECT_EQ( result.counts.ok, result.trajectory->size() );
    ASSERT_EQ( result.diag.size(), 7U );
    EXPECT_LT( result.first_image_ts, result.last_image_ts );
    EXPECT_GT( result.wall_s, 0.0 );

    for ( std::size_t index = 0U; index < 5U; ++index )
    {
      const VoDiagRow& row = result.diag[ index ];
      EXPECT_EQ( row.status, "initializing" ) << "diag_index=" << index;
      EXPECT_EQ( row.initialization.m_path,
                 phad::estimator::InitializationPath::kMoving )
          << "diag_index=" << index;
      EXPECT_EQ( row.initialization.m_phase,
                 phad::estimator::InitializationPhase::kCollectingEvidence )
          << "diag_index=" << index;
      EXPECT_EQ( row.initialization.m_reason,
                 phad::estimator::InitializationReason::kPoseCountInsufficient )
          << "diag_index=" << index;
      EXPECT_EQ( row.initialization.m_outcome,
                 phad::estimator::InitializationOutcome::kPending )
          << "diag_index=" << index;
      EXPECT_FALSE( row.initialization.m_attempt_id.has_value() )
          << "diag_index=" << index;
      EXPECT_EQ( row.initialization.m_current_graph,
                 phad::estimator::InitializationGraphResult::kNotEvaluated )
          << "diag_index=" << index;
      EXPECT_EQ(
          row.initialization.m_transaction,
          phad::estimator::InitializationTransactionResult::kEvidenceCommitted )
          << "diag_index=" << index;
      EXPECT_GT( row.num_disparity, 0U ) << "diag_index=" << index;
    }

    const VoDiagRow& duration_pending = result.diag[ 5 ];
    EXPECT_EQ( duration_pending.status, "initializing" );
    EXPECT_EQ( duration_pending.initialization.m_path,
               phad::estimator::InitializationPath::kMoving );
    EXPECT_EQ( duration_pending.initialization.m_phase,
               phad::estimator::InitializationPhase::kCollectingEvidence );
    EXPECT_EQ( duration_pending.initialization.m_reason,
               phad::estimator::InitializationReason::kDurationInsufficient );
    EXPECT_EQ( duration_pending.initialization.m_outcome,
               phad::estimator::InitializationOutcome::kPending );
    EXPECT_FALSE( duration_pending.initialization.m_attempt_id.has_value() );
    EXPECT_EQ( duration_pending.initialization.m_connected_poses.m_actual, 6U );
    EXPECT_EQ( duration_pending.initialization.m_connected_poses.m_required, 6U );
    EXPECT_EQ( duration_pending.initialization.m_duration.m_actual_ns,
               249'999'872 );
    EXPECT_EQ( duration_pending.initialization.m_duration.m_required_ns,
               250'000'000 );
    EXPECT_EQ(
        duration_pending.initialization.m_current_graph,
        phad::estimator::InitializationGraphResult::kNotEvaluated );
    EXPECT_EQ(
        duration_pending.initialization.m_transaction,
        phad::estimator::InitializationTransactionResult::kEvidenceCommitted );
    EXPECT_GT( duration_pending.num_disparity, 0U );

    const VoDiagRow& root = result.diag[ 6 ];
    EXPECT_EQ( root.status, "ok" );
    EXPECT_EQ( root.initialization.m_path,
               phad::estimator::InitializationPath::kMoving );
    EXPECT_EQ( root.initialization.m_phase,
               phad::estimator::InitializationPhase::kCommit );
    EXPECT_EQ( root.initialization.m_reason,
               phad::estimator::InitializationReason::kNone );
    EXPECT_EQ( root.initialization.m_outcome,
               phad::estimator::InitializationOutcome::kCommitted );
    EXPECT_TRUE( root.initialization.m_attempt_id.has_value() );
    EXPECT_EQ( root.initialization.m_connected_poses.m_required, 6U );
    EXPECT_EQ( root.initialization.m_connected_poses.m_actual, 7U );
    EXPECT_EQ( root.initialization.m_duration.m_required_ns, 250'000'000 );
    EXPECT_EQ( root.initialization.m_duration.m_actual_ns, 300'000'000 );
    EXPECT_EQ( root.timestamp_ns - result.diag.front().timestamp_ns,
               300'000'000 );
    EXPECT_GE( root.initialization.m_connected_landmarks.m_actual,
               root.initialization.m_connected_landmarks.m_required );
    ASSERT_TRUE( root.initialization.m_visual_support.has_value() );
    EXPECT_GE( root.initialization.m_visual_support->m_actual,
               root.initialization.m_visual_support->m_required );
    EXPECT_EQ( root.initialization.m_current_graph,
               phad::estimator::InitializationGraphResult::kValidated );
    EXPECT_EQ(
        root.initialization.m_transaction,
        phad::estimator::InitializationTransactionResult::kRootCommitted );
    ASSERT_EQ( result.trajectory->size(), 1U );
    EXPECT_EQ( result.trajectory->poses().front().timestamp.nanoseconds(),
               root.timestamp_ns );

    // A clean formal-root prefix has no initialization summary warning.
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

  TEST( OfflineVoSessionTest, Mh01SevenFrameProbeBIsNonPerturbing )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    const auto output_dir = std::filesystem::temp_directory_path() /
                            "phad_offline_vo_session_probe_b_nonperturbing";
    std::filesystem::remove_all( output_dir );
    std::filesystem::create_directories( output_dir );
    const auto probe_path    = output_dir / "probe_b.jsonl";
    const auto diag_off_path = output_dir / "diag-off.csv";
    const auto diag_on_path  = output_dir / "diag-on.csv";

    OfflineVoSessionOptions options_off;
    options_off.max_frames                          = 7;
    options_off.collect_timing                      = false;
    options_off.estimator.m_enable_moving_bootstrap = true;
    OfflineVoSessionOptions options_on              = options_off;
    options_on.probe_b_path                         = probe_path;

    auto opened_off = phad::io::dataset::euroc::open( configured_path );
    ASSERT_TRUE( opened_off ) << opened_off.error().describe();
    const auto result_off =
        runOfflineVoSession( std::move( opened_off ).value(), options_off );
    auto opened_on = phad::io::dataset::euroc::open( configured_path );
    ASSERT_TRUE( opened_on ) << opened_on.error().describe();
    const auto result_on =
        runOfflineVoSession( std::move( opened_on ).value(), options_on );

    ASSERT_EQ( result_off.error.has_value(), result_on.error.has_value() );
    if ( result_off.error.has_value() )
    {
      EXPECT_EQ( result_off.error->detail, result_on.error->detail );
    }
    ASSERT_FALSE( result_off.error.has_value() ) << result_off.error->detail;
    expectFrameCountsEqual( result_off.counts, result_on.counts );
    expectTrajectoryEqual( result_off.trajectory, result_on.trajectory );
    expectTrajectoryEqual( result_off.kf_trajectory, result_on.kf_trajectory );

    const auto off_write = writeDiagCsv( diag_off_path, result_off.diag );
    ASSERT_FALSE( off_write.has_value() ) << off_write->detail;
    const auto on_write = writeDiagCsv( diag_on_path, result_on.diag );
    ASSERT_FALSE( on_write.has_value() ) << on_write->detail;
    EXPECT_EQ( readTextFile( diag_off_path ), readTextFile( diag_on_path ) );

    std::ifstream probe_input( probe_path );
    ASSERT_TRUE( probe_input );
    std::size_t probe_rows = 0U;
    std::string line;
    while ( std::getline( probe_input, line ) )
    {
      if ( line.empty() )
      {
        continue;
      }
      EXPECT_NO_THROW( {
        const auto json = nlohmann::json::parse( line );
        EXPECT_TRUE( json.is_object() );
      } );
      ++probe_rows;
    }
    EXPECT_EQ( probe_rows, result_on.diag.size() );

    std::filesystem::remove_all( output_dir );
  }

}  // namespace

TEST( OfflineVoSessionTest,
      AcceptedKeyframeUsesSameFrameRectifiedCameraRotation )
{
  namespace internal          = phad::apps::internal;
  Eigen::Matrix4d       T_B_C = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R_B_C =
      Eigen::AngleAxisd( std::numbers::pi / 12.0,
                         Eigen::Vector3d::UnitX() )
          .toRotationMatrix();
  T_B_C.topLeftCorner<3, 3>() = R_B_C;
  auto       transform        = phad::sensor::RigidTransform::create( T_B_C ).value();
  const auto calibration =
      phad::camera::RectifiedStereoCalibration::create(
          400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
          std::move( transform ) )
          .value();

  internal::KeyframeSelectorState state;
  state.m_total_keyframes                 = 2U;
  const auto                       tracks = makeAcceptedTracks( 100'000'000 );
  phad::estimator::VioUpdateResult update;
  update.status = phad::estimator::UpdateStatus::kOk;
  phad::estimator::VioEstimate estimate;
  estimate.timestamp = tracks.timestamp;
  const Eigen::Matrix3d R_W_B =
      Eigen::AngleAxisd( 5.0 * std::numbers::pi / 36.0,
                         Eigen::Vector3d::UnitZ() )
          .toRotationMatrix();
  estimate.T_W_B          = Eigen::Isometry3d::Identity();
  estimate.T_W_B.linear() = R_W_B;
  update.estimate         = estimate;

  internal::applyAcceptedCameraState(
      true, tracks, tracks.timestamp, update, calibration, state );

  const Eigen::Matrix3d expected = R_W_B * R_B_C;
  EXPECT_TRUE( state.m_last_kf_rotation.isApprox( expected, 1e-12 ) );
  EXPECT_TRUE( state.m_last_accepted_rotation.isApprox( expected, 1e-12 ) );
  const auto next_tracks = makeAcceptedTracks( 150'000'000 );
  EXPECT_FALSE( internal::isKeyframe(
      next_tracks, next_tracks.timestamp, state, calibration ) );
}

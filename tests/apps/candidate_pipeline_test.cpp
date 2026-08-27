#include "apps/candidate_pipeline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "apps/candidate_fixed_lag_estimator.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "tests/frontend/synthetic_stereo.hpp"

namespace
{

  using phad::apps::CandidateErrorCode;
  using phad::apps::CandidateFrameInput;
  using phad::apps::CandidatePipeline;
  using phad::apps::CandidatePipelineOptions;
  using phad::apps::CandidateProgress;
  using phad::estimator::UpdateStatus;
  using phad::frontend::StereoTrackerOptions;
  using phad::sensor::Image;
  using phad::testing::kStereoEpochNs;
  using phad::testing::kStereoStepNs;
  using phad::testing::makePointGrid;
  using phad::testing::makeRectifiedCalibration;
  using phad::testing::renderStereo;

  [[nodiscard]] StereoTrackerOptions trackerOptions( int max_tracks )
  {
    StereoTrackerOptions options;
    options.max_tracks          = max_tracks;
    options.quality_level       = 0.01;
    options.min_distance_px     = 15.0;
    options.mask_radius_px      = 12;
    options.lk_window_px        = 21;
    options.lk_pyramid_levels   = 2;
    options.forward_backward_px = 1.0;
    options.max_epipolar_px     = 1.5;
    options.min_disparity_px    = 0.5;
    options.min_depth_m         = 0.3;
    options.max_depth_m         = 40.0;
    options.stereo_uniq_ratio   = 0.0;
    options.stereo_check_bidir  = false;
    return options;
  }

  [[nodiscard]] std::vector<std::uint8_t> imageBytes( const Image& image )
  {
    const std::optional<std::span<const std::uint8_t>> pixels =
        image.pixels<std::uint8_t>();
    EXPECT_TRUE( pixels.has_value() );
    if ( !pixels.has_value() )
    {
      return {};
    }
    return { pixels->begin(), pixels->end() };
  }

  [[nodiscard]] CandidatePipelineOptions candidateOptions( int max_tracks )
  {
    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( max_tracks );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;
    return options;
  }

  void expectSameRun( const phad::apps::CandidateRunResult& a,
                      const phad::apps::CandidateRunResult& b )
  {
    EXPECT_EQ( a.error.has_value(), b.error.has_value() );
    EXPECT_EQ( a.trajectory.has_value(), b.trajectory.has_value() );
    if ( a.trajectory.has_value() )
    {
      ASSERT_TRUE( b.trajectory.has_value() );
      ASSERT_EQ( a.trajectory->size(), b.trajectory->size() );
      const auto& poses_a = a.trajectory->poses();
      const auto& poses_b = b.trajectory->poses();
      for ( std::size_t i = 0; i < poses_a.size(); ++i )
      {
        EXPECT_EQ( poses_a[ i ].timestamp, poses_b[ i ].timestamp );
        EXPECT_TRUE( poses_a[ i ].T_W_B.matrix().isApprox(
            poses_b[ i ].T_W_B.matrix(), 1e-9 ) );
      }
    }
    ASSERT_EQ( a.diagnostics.size(), b.diagnostics.size() );
    for ( std::size_t i = 0; i < a.diagnostics.size(); ++i )
    {
      EXPECT_EQ( a.diagnostics[ i ].frame_index, b.diagnostics[ i ].frame_index );
      EXPECT_EQ( a.diagnostics[ i ].timestamp, b.diagnostics[ i ].timestamp );
      EXPECT_EQ( a.diagnostics[ i ].status, b.diagnostics[ i ].status );
      EXPECT_EQ( a.diagnostics[ i ].state.has_value(),
                 b.diagnostics[ i ].state.has_value() );
      if ( a.diagnostics[ i ].state.has_value() )
      {
        ASSERT_TRUE( b.diagnostics[ i ].state.has_value() );
        EXPECT_TRUE( a.diagnostics[ i ].state->T_W_B.matrix().isApprox(
            b.diagnostics[ i ].state->T_W_B.matrix(), 1e-9 ) );
      }
    }
    EXPECT_EQ( a.counts.frames, b.counts.frames );
    EXPECT_EQ( a.counts.ok, b.counts.ok );
    EXPECT_EQ( a.counts.rejected, b.counts.rejected );
    EXPECT_EQ( a.counts.failed, b.counts.failed );
    EXPECT_EQ( a.counts.keyframes, b.counts.keyframes );
    EXPECT_EQ( a.counts.track_only_frames, b.counts.track_only_frames );
  }

  TEST( CandidatePipelineTest,
        ProcessesImmutableInputIntoAcceptedTrajectory )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto frame       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto left_before  = imageBytes( frame.left );
    const auto right_before = imageBytes( frame.right );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    const CandidateProgress progress = candidate.process( CandidateFrameInput{
        .rectified   = frame,
        .imu_samples = {},
        .t_prev      = frame.timestamp,
        .imu_gap     = false,
    } );
    EXPECT_EQ( progress, CandidateProgress::kRunning );

    auto result = std::move( candidate ).finish();
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.trajectory->size(), 1U );
    ASSERT_EQ( result.diagnostics.size(), 1U );
    EXPECT_EQ( result.diagnostics.front().status, UpdateStatus::kOk );
    ASSERT_TRUE( result.diagnostics.front().state.has_value() );
    EXPECT_TRUE( result.diagnostics.front()
                     .state->T_W_B.matrix()
                     .isApprox( Eigen::Matrix4d::Identity(), 1e-9 ) );

    EXPECT_EQ( imageBytes( frame.left ), left_before );
    EXPECT_EQ( imageBytes( frame.right ), right_before );
  }

  TEST( CandidatePipelineTest,
        ReturnsTypedTerminalForNonIncreasingFrameTimestamp )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto frame       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = frame,
                   .imu_samples = {},
                   .t_prev      = frame.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = frame,
                   .imu_samples = {},
                   .t_prev      = frame.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    ASSERT_TRUE( result.error->frame_index.has_value() );
    EXPECT_EQ( *result.error->frame_index, 1U );
    ASSERT_TRUE( result.error->timestamp.has_value() );
    EXPECT_EQ( *result.error->timestamp, frame.timestamp );
    EXPECT_NE( result.error->detail.find( "strictly increasing" ),
               std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_FALSE( result.keyframe_trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
    EXPECT_EQ( result.diagnostics.back().status, UpdateStatus::kFailed );
  }

  TEST( CandidatePipelineTest, ReturnsTypedTerminalForMismatchedPreviousTime )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = {},
                   .t_prev      = second.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    EXPECT_NE( result.error->detail.find( "t_prev" ), std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_FALSE( result.keyframe_trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
    EXPECT_EQ( result.diagnostics.back().timestamp, second.timestamp );
    EXPECT_EQ( result.diagnostics.back().status, UpdateStatus::kFailed );
  }

  TEST( CandidatePipelineTest, ReturnsTypedTerminalForEmptyNonGapImuInterval )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    EXPECT_NE( result.error->detail.find( "IMU" ), std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_FALSE( result.keyframe_trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
    EXPECT_EQ( result.diagnostics.back().timestamp, second.timestamp );
    EXPECT_EQ( result.diagnostics.back().status, UpdateStatus::kFailed );
  }

  TEST( CandidatePipelineTest, ReturnsTypedTerminalForNonIncreasingImuTimestamps )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );

    using phad::sensor::ImuMeasurement;
    const std::vector<ImuMeasurement> imu_samples{
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = second.timestamp },
    };

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = imu_samples,
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    EXPECT_NE( result.error->detail.find( "IMU" ), std::string::npos );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_FALSE( result.keyframe_trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
    EXPECT_EQ( result.diagnostics.back().status, UpdateStatus::kFailed );
  }

  TEST( CandidatePipelineTest, AllowsIncompleteImuIntervalAcrossGap )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = true,
               } ),
               CandidateProgress::kRunning );

    auto result = std::move( candidate ).finish();
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
  }

  TEST( CandidatePipelineTest, TerminalFailureIsStickyWithoutNewDiagnostics )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    const auto third = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = {},
                   .t_prev      = second.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = third,
                   .imu_samples = {},
                   .t_prev      = second.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    ASSERT_TRUE( result.error->frame_index.has_value() );
    EXPECT_EQ( *result.error->frame_index, 1U );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_FALSE( result.keyframe_trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
  }

  // P2b slice ②: 骨架期 fixed-lag 无 IMU init 概念，无帧级 kFailed 源；
  // 该测试依赖 batch `StereoVoEstimator` 的 imu_init_timeout 行为，暂时
  // DISABLED，slice ④（per-KF bias 链 + fixed-lag init 语义）恢复。
  TEST( CandidatePipelineTest,
        DISABLED_ContinuesAfterEstimatorFrameFailure )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    const auto third = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );

    using phad::sensor::ImuMeasurement;
    const std::vector<ImuMeasurement> imu_12{
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = second.timestamp },
    };
    const std::vector<ImuMeasurement> imu_23{
        ImuMeasurement{ .timestamp = second.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + 3 * kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = third.timestamp },
    };

    CandidatePipelineOptions options;
    options.tracker                                   = trackerOptions( static_cast<int>( points.size() ) );
    options.estimator.enable_imu                      = true;
    options.estimator.imu_init_timeout_s              = 0.001;
    options.estimator.min_seed_observations           = 10;
    options.estimator.min_track_observations_for_seed = 1;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = imu_12,
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = third,
                   .imu_samples = imu_23,
                   .t_prev      = second.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );

    auto result = std::move( candidate ).finish();
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 3U );
    EXPECT_EQ( result.diagnostics[ 0 ].status, UpdateStatus::kOk );
    EXPECT_EQ( result.diagnostics[ 1 ].status, UpdateStatus::kFailed );
    EXPECT_EQ( result.diagnostics[ 2 ].status, UpdateStatus::kFailed );
    EXPECT_EQ( result.counts.failed, 2U );
    EXPECT_EQ( result.trajectory->size(), 1U );
  }

  TEST( CandidatePipelineTest, IndependentCandidatesProduceIdenticalResults )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    const auto third = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );

    using phad::sensor::ImuMeasurement;
    const std::vector<ImuMeasurement> imu_12{
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = second.timestamp },
    };
    const std::vector<ImuMeasurement> imu_23{
        ImuMeasurement{ .timestamp = second.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + 3 * kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = third.timestamp },
    };

    const CandidatePipelineOptions options =
        candidateOptions( static_cast<int>( points.size() ) );

    const auto run_three_frames = [ & ]() -> phad::apps::CandidateRunResult {
      auto created = CandidatePipeline::create( calibration, options );
      EXPECT_TRUE( created ) << created.error().detail;
      CandidatePipeline candidate = std::move( created ).value();
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = first,
                     .imu_samples = {},
                     .t_prev      = first.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = second,
                     .imu_samples = imu_12,
                     .t_prev      = first.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = third,
                     .imu_samples = imu_23,
                     .t_prev      = second.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      return std::move( candidate ).finish();
    };

    const phad::apps::CandidateRunResult run_a = run_three_frames();
    const phad::apps::CandidateRunResult run_b = run_three_frames();
    expectSameRun( run_a, run_b );
  }

  TEST( CandidatePipelineTest, CandidateOnlyPerturbationDoesNotAffectControl )
  {
    const auto calibration    = makeRectifiedCalibration();
    const auto grid           = makePointGrid( calibration, 4, 5 );
    const auto perturbed_grid = makePointGrid( calibration, 3, 4 );
    // z 非线性抖动（sin/cos）：无 Between 后共面观测会让 pose 深度
    // 方向欠约束（奇异），线性斜面仍共面。
    const auto spread = []( std::vector<Eigen::Vector3d> points ) {
      for ( Eigen::Vector3d& point : points )
      {
        point.z() += 0.35 * std::sin( 5.0 * point.x() ) +
                     0.35 * std::cos( 4.0 * point.y() );
      }
      return points;
    };
    const auto points           = spread( grid );
    const auto perturbed_points = spread( perturbed_grid );
    const auto first            = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    const auto third = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );
    const auto third_perturbed = renderStereo(
        calibration, perturbed_points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );

    using phad::sensor::ImuMeasurement;
    const std::vector<ImuMeasurement> imu_12{
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = second.timestamp },
    };
    const std::vector<ImuMeasurement> imu_23{
        ImuMeasurement{ .timestamp = second.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + 3 * kStereoStepNs / 2 } },
        ImuMeasurement{ .timestamp = third.timestamp },
    };

    const CandidatePipelineOptions options =
        candidateOptions( static_cast<int>( points.size() ) );

    const auto run_with_third = [ & ]( const phad::sensor::StereoFrame& third_frame )
        -> phad::apps::CandidateRunResult {
      auto created = CandidatePipeline::create( calibration, options );
      EXPECT_TRUE( created ) << created.error().detail;
      CandidatePipeline candidate = std::move( created ).value();
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = first,
                     .imu_samples = {},
                     .t_prev      = first.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = second,
                     .imu_samples = imu_12,
                     .t_prev      = first.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      EXPECT_EQ( candidate.process( CandidateFrameInput{
                     .rectified   = third_frame,
                     .imu_samples = imu_23,
                     .t_prev      = second.timestamp,
                     .imu_gap     = false,
                 } ),
                 CandidateProgress::kRunning );
      return std::move( candidate ).finish();
    };

    const phad::apps::CandidateRunResult perturbed = run_with_third( third_perturbed );
    const phad::apps::CandidateRunResult control   = run_with_third( third );
    const phad::apps::CandidateRunResult reference = run_with_third( third );
    // candidate-only 输入扰动只影响被扰动的实例，control 必须与
    // 从未见过扰动的 reference 完全一致。
    expectSameRun( control, reference );
    (void)perturbed;
  }

  TEST( CandidatePipelineTest, ReturnsTypedTerminalForImuSampleOutsideInterval )
  {
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );
    const auto first       = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const auto second = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );

    using phad::sensor::ImuMeasurement;
    const std::vector<ImuMeasurement> imu_samples{
        ImuMeasurement{ .timestamp = first.timestamp },
        ImuMeasurement{ .timestamp = phad::common::Timestamp{
                            kStereoEpochNs + kStereoStepNs + 1 } },
        ImuMeasurement{ .timestamp = second.timestamp },
    };

    const CandidatePipelineOptions options =
        candidateOptions( static_cast<int>( points.size() ) );

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = first,
                   .imu_samples = {},
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kRunning );
    EXPECT_EQ( candidate.process( CandidateFrameInput{
                   .rectified   = second,
                   .imu_samples = imu_samples,
                   .t_prev      = first.timestamp,
                   .imu_gap     = false,
               } ),
               CandidateProgress::kTerminalFailure );

    auto result = std::move( candidate ).finish();
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_EQ( result.error->code, CandidateErrorCode::kInputContract );
    EXPECT_FALSE( result.trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 2U );
    EXPECT_EQ( result.diagnostics.back().status, UpdateStatus::kFailed );
  }

  TEST( CandidatePipelineTest, FixedLagSkeletonBoundsGraphAndDeclaresReset )
  {
    // P2b slice ②：fixed-lag 骨架的结构门。synthetic 序列 40 帧
    // （50 ms/帧；gate bootstrap epoch<2 使 0、1 帧都是 KF，之后每 10 帧
    // timeout KF），断言：全程 kRunning/kOk、无 error、graph 计数真实且
    // active pose 有界、出现边缘化、reset 只声明在首帧（kBootstrap）。
    const auto calibration = makeRectifiedCalibration();
    const auto points      = makePointGrid( calibration, 4, 5 );

    std::vector<phad::sensor::StereoFrame> frames;
    frames.reserve( 40 );
    for ( int i = 0; i < 40; ++i )
    {
      frames.push_back( renderStereo(
          calibration, points,
          phad::common::Timestamp{ kStereoEpochNs +
                                   static_cast<std::int64_t>( i ) *
                                       kStereoStepNs },
          2.5 ) );
    }

    const CandidatePipelineOptions options =
        candidateOptions( static_cast<int>( points.size() ) );

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    for ( int i = 0; i < 40; ++i )
    {
      const auto&                               frame = frames[ static_cast<std::size_t>( i ) ];
      std::vector<phad::sensor::ImuMeasurement> imu_samples;
      if ( i > 0 )
      {
        imu_samples = {
            phad::sensor::ImuMeasurement{ .timestamp = frames[ static_cast<std::size_t>( i - 1 ) ].timestamp },
            phad::sensor::ImuMeasurement{ .timestamp = frame.timestamp },
        };
      }
      const phad::common::Timestamp t_prev =
          i == 0 ? frame.timestamp
                 : frames[ static_cast<std::size_t>( i - 1 ) ].timestamp;
      const CandidateProgress progress = candidate.process(
          CandidateFrameInput{
              .rectified   = frame,
              .imu_samples = imu_samples,
              .t_prev      = t_prev,
              .imu_gap     = false,
          } );
      if ( progress != CandidateProgress::kRunning )
      {
        auto debug_result = std::move( candidate ).finish();
        ADD_FAILURE() << "DBG terminal at frame " << i << ": "
                      << ( debug_result.error.has_value()
                               ? debug_result.error->detail
                               : "no error" );
        return;
      }
    }

    auto result = std::move( candidate ).finish();
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    ASSERT_EQ( result.diagnostics.size(), 40U );
    EXPECT_EQ( result.counts.failed, 0U );

    std::uint32_t total_marginalized = 0;
    for ( std::size_t i = 0; i < result.diagnostics.size(); ++i )
    {
      const auto& diagnostics = result.diagnostics[ i ];
      // kOk 或 kRejected（非 KF 帧 PnP 失败合法）；kFailed 禁止。
      EXPECT_NE( diagnostics.status, UpdateStatus::kFailed ) << i;
      EXPECT_EQ( diagnostics.reset_reason,
                 i == 0U ? phad::apps::CandidateResetReason::kBootstrap
                         : phad::apps::CandidateResetReason::kNone )
          << i;
      ASSERT_TRUE( diagnostics.graph.has_value() ) << i;
      EXPECT_LE( diagnostics.graph->active_pose_count,
                 static_cast<std::uint32_t>( options.estimator.window_size ) +
                     2U )
          << i;
      total_marginalized += diagnostics.graph->marginalized_pose_count;
    }
    EXPECT_GT( total_marginalized, 0U );
  }

  TEST( CandidatePipelineTest, FixedLagLandmarksRetireAndStayBounded )
  {
    // P2b slice ③：landmark 生命周期结构门。40 帧合成序列，点云每帧绕
    // Y 轴旋转 0.02 rad → 点逐渐移出视野 → tracker 丢 track → landmark
    // 最后观测出窗后自然退休。断言：全程 kRunning/kOk、无 error、
    // active pose 有界、退休累计 > 0。
    const auto calibration = makeRectifiedCalibration();
    const auto grid        = makePointGrid( calibration, 4, 5 );
    // z 非线性抖动（sin/cos）：无 Between 后共面观测会让 pose 深度
    // 方向欠约束（奇异），线性斜面仍共面。
    std::vector<Eigen::Vector3d> spread_grid = grid;
    for ( Eigen::Vector3d& point : spread_grid )
    {
      point.z() += 0.35 * std::sin( 5.0 * point.x() ) +
                   0.35 * std::cos( 4.0 * point.y() );
    }

    std::vector<phad::sensor::StereoFrame> frames;
    frames.reserve( 40 );
    for ( int i = 0; i < 40; ++i )
    {
      const double          angle = static_cast<double>( i ) * 0.02;
      const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd( angle, Eigen::Vector3d::UnitY() )
              .toRotationMatrix();
      std::vector<Eigen::Vector3d> rotated;
      rotated.reserve( grid.size() );
      for ( const Eigen::Vector3d& point : spread_grid )
      {
        rotated.push_back( rotation * point );
      }
      frames.push_back( renderStereo(
          calibration, rotated,
          phad::common::Timestamp{ kStereoEpochNs +
                                   static_cast<std::int64_t>( i ) *
                                       kStereoStepNs },
          2.5 ) );
    }

    CandidatePipelineOptions options =
        candidateOptions( static_cast<int>( spread_grid.size() ) );
    // 旋转场景帧间投影位移 ~5-12 px > 默认 gating 阈值 4 px：正常观测
    // 会被误拒收；放宽阈值让运动观测进图（测试专属）。
    options.estimator.outlier_avg_reproj_px = 20.0;

    auto created = CandidatePipeline::create( calibration, options );
    ASSERT_TRUE( created ) << created.error().detail;
    CandidatePipeline candidate = std::move( created ).value();

    for ( int i = 0; i < 40; ++i )
    {
      const auto&                               frame = frames[ static_cast<std::size_t>( i ) ];
      std::vector<phad::sensor::ImuMeasurement> imu_samples;
      if ( i > 0 )
      {
        imu_samples = {
            phad::sensor::ImuMeasurement{ .timestamp = frames[ static_cast<std::size_t>( i - 1 ) ].timestamp },
            phad::sensor::ImuMeasurement{ .timestamp = frame.timestamp },
        };
      }
      const phad::common::Timestamp t_prev =
          i == 0 ? frame.timestamp
                 : frames[ static_cast<std::size_t>( i - 1 ) ].timestamp;
      const CandidateProgress progress = candidate.process(
          CandidateFrameInput{
              .rectified   = frame,
              .imu_samples = imu_samples,
              .t_prev      = t_prev,
              .imu_gap     = false,
          } );
      if ( progress != CandidateProgress::kRunning )
      {
        auto debug_result = std::move( candidate ).finish();
        ADD_FAILURE() << "DBG terminal at frame " << i << ": "
                      << ( debug_result.error.has_value()
                               ? debug_result.error->detail
                               : "no error" );
        return;
      }
    }

    auto result = std::move( candidate ).finish();
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_EQ( result.diagnostics.size(), 40U );
    EXPECT_EQ( result.counts.failed, 0U );

    std::uint32_t total_retired = 0;
    for ( std::size_t i = 0; i < result.diagnostics.size(); ++i )
    {
      const auto& diagnostics = result.diagnostics[ i ];
      // kOk 或 kRejected（非 KF 帧 PnP 失败合法）；kFailed 禁止。
      EXPECT_NE( diagnostics.status, UpdateStatus::kFailed ) << i;
      ASSERT_TRUE( diagnostics.graph.has_value() ) << i;
      EXPECT_LE( diagnostics.graph->active_pose_count,
                 static_cast<std::uint32_t>( options.estimator.window_size ) +
                     2U )
          << i;
      total_retired += diagnostics.graph->retired_landmark_count;
    }
    // 点逐渐移出视野：landmark 最后观测必然出窗（窗口 10 帧），退休
    // 必须发生（retired_landmark_count 是每帧增量，累计 > 0）。
    EXPECT_GT( total_retired, 0U );
  }

  TEST( CandidatePipelineTest, FixedLagEstimatorGatesOutlierObservations )
  {
    // P2b slice ③：estimator 级进图前 gating。手造 stereo 观测：Z=2 m、
    // 像素 (159.5, 119.5)、视差 13.75（= 0.11 * 250 / 2）。KF2 同观测
    // 残差≈0 通过；KF3 观测左移 +50 px → 残差超阈值 → 观测不入图
    // （active_factor_count 只加 between，不加 stereo）。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    // 不同 landmark 用不同深度与像素方向（disparity 13.75/11.0/9.1667
    // ≈ z=2/2.5/3 m，v 也不同），避免共面/近中心观测导致 pose 欠约束
    // （无 Between 时奇异）。
    const auto observation_at = []( double u, double v, std::uint64_t id,
                                    double disparity ) {
      return phad::estimator::StereoObservation{
          .id           = id,
          .left_pixel   = Eigen::Vector2d{ u, v },
          .disparity_px = disparity,
      };
    };
    const auto l1 = [ & ]( double u, double v ) {
      return observation_at( u, v, 1, 13.75 );
    };
    const auto l2 = [ & ]( double u, double v ) {
      return observation_at( u, v, 2, 11.0 );
    };
    const auto l3 = [ & ]( double u, double v ) {
      return observation_at( u, v, 3, 9.1667 );
    };
    const auto l4 = [ & ]( double u, double v ) {
      return observation_at( u, v, 4, 8.3333 );
    };
    const auto make_measurement = [ & ]( std::int64_t epoch_offset,
                                         std::vector<phad::estimator::
                                                         StereoObservation>
                                             observations ) {
      phad::estimator::KeyframeMeasurement measurement;
      measurement.timestamp = phad::common::Timestamp{
          kStereoEpochNs + epoch_offset };
      measurement.observations = std::move( observations );
      return measurement;
    };

    // r1: 插入 l1-l4（root 良定）；r2: 全部通过；
    // r3: l1 被 gating 拒收（+150 px > 100 px 阈值），l2-l4 通过；
    // r4: l1 zero-disparity 跳过，l2-l4 通过。
    const phad::apps::FixedLagUpdateResult r1 = estimator.update(
        make_measurement( 0, { l1( 139.5, 99.5 ), l2( 179.5, 139.5 ),
                               l3( 159.5, 119.5 ), l4( 119.5, 179.5 ) } ),
        true );
    ASSERT_EQ( r1.vio.status, UpdateStatus::kOk );
    // pose_prior + bias_prior + 4 stereo = 6
    EXPECT_EQ( r1.graph.active_factor_count, 6U );

    const phad::apps::FixedLagUpdateResult r2 = estimator.update(
        make_measurement( kStereoStepNs,
                          { l1( 139.5, 99.5 ), l2( 179.5, 139.5 ),
                            l3( 159.5, 119.5 ), l4( 119.5, 179.5 ) } ),
        true );
    ASSERT_EQ( r2.vio.status, UpdateStatus::kOk );
    // + bias_rw + 4 stereo = 11（无 IMU 样本 → 无 ahrs；无占位 Between）
    EXPECT_EQ( r2.graph.active_factor_count, 11U );

    const phad::apps::FixedLagUpdateResult r3 = estimator.update(
        make_measurement( 2 * kStereoStepNs,
                          { l1( 289.5, 99.5 ), l2( 179.5, 139.5 ),
                            l3( 159.5, 119.5 ), l4( 119.5, 179.5 ) } ),
        true );
    ASSERT_EQ( r3.vio.status, UpdateStatus::kOk );
    // gating 拒收 l1：+ bias_rw + 3 stereo = 15
    EXPECT_EQ( r3.graph.active_factor_count, 15U );

    // zero-disparity 观测：不建 stereo factor、不刷 timestamp。
    phad::estimator::StereoObservation zero_disparity = l1( 139.5, 99.5 );
    zero_disparity.disparity_px                       = 0.0;
    phad::estimator::KeyframeMeasurement r4_measurement =
        make_measurement( 3 * kStereoStepNs,
                          { l1( 139.5, 99.5 ), l2( 179.5, 139.5 ),
                            l3( 159.5, 119.5 ), l4( 119.5, 179.5 ) } );
    r4_measurement.observations[ 0 ] = zero_disparity;
    const phad::apps::FixedLagUpdateResult r4 =
        estimator.update( r4_measurement, true );
    ASSERT_EQ( r4.vio.status, UpdateStatus::kOk );
    // + bias_rw + 3 stereo = 19
    EXPECT_EQ( r4.graph.active_factor_count, 19U );
    EXPECT_EQ( r4.graph.retired_landmark_count, 0U );
  }

  TEST( CandidatePipelineTest, FixedLagEstimatorRollsBackAfterDuplicateId )
  {
    // P2b slice ③：异常回滚回归。同一帧内同一 landmark id 出现两次：
    // 第一次插入新 key，第二次走 gated() 对尚未进入 smoother 的 key 调
    // calculateEstimate → 抛异常；snapshot 必须完整恢复（smoother +
    // ledger + active_landmarks + 计数），后续干净 update 成功且账本
    // 一致（否则永久 terminal）。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    const auto observation_at = []( double u ) {
      return phad::estimator::StereoObservation{
          .id           = 1,
          .left_pixel   = Eigen::Vector2d{ u, 119.5 },
          .disparity_px = 13.75,
      };
    };

    phad::estimator::KeyframeMeasurement bad;
    bad.timestamp    = phad::common::Timestamp{ kStereoEpochNs };
    bad.observations = { observation_at( 159.5 ), observation_at( 159.5 ) };
    EXPECT_THROW( estimator.update( bad, true ), std::exception );

    phad::estimator::KeyframeMeasurement clean;
    clean.timestamp = phad::common::Timestamp{ kStereoEpochNs };
    clean.observations.push_back( observation_at( 159.5 ) );
    phad::apps::FixedLagUpdateResult r;
    try
    {
      r = estimator.update( clean, true );
    }
    catch ( const std::exception& debug_exception )
    {
      FAIL() << "clean update threw: " << debug_exception.what();
    }
    ASSERT_EQ( r.vio.status, UpdateStatus::kOk );
    // 回滚后本帧是首个 KF：pose_prior + bias_prior + 1 stereo = 3
    // （ledger/计数一致）。
    EXPECT_EQ( r.graph.active_factor_count, 3U );
    EXPECT_EQ( r.graph.active_pose_count, 1U );
  }

  TEST( CandidatePipelineTest, FixedLagBiasChainFusesGyroAndStaysBounded )
  {
    // P2b slice ④：per-KF bias 链 + AHRS gyro factor。3 帧 KF，每帧带
    // 两样本零 gyro 段（预积分 deltaR=identity，与视觉 identity 一致）。
    // 首帧无 gyro factor（kVisionOnly），其后 kGyroVisual；factor 计数
    // 验证 bias prior/RW edge/AHRS factor 都进图；全程 kOk、active 有界。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    estimator_options.enable_imu  = true;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    // 两个 landmark 分散在不同方向与深度（避免近中心同向观测的
    // Jacobian 秩亏导致 pose 欠约束）。
    const auto observation_at = []( double u, double v, std::uint64_t id,
                                    double disparity ) {
      return phad::estimator::StereoObservation{
          .id           = id,
          .left_pixel   = Eigen::Vector2d{ u, v },
          .disparity_px = disparity,
      };
    };
    const auto make_measurement = [ & ]( std::int64_t epoch_offset ) {
      using phad::sensor::ImuMeasurement;
      phad::estimator::KeyframeMeasurement measurement;
      measurement.timestamp    = phad::common::Timestamp{ kStereoEpochNs +
                                                       epoch_offset };
      measurement.observations = { observation_at( 139.5, 99.5, 1, 13.75 ),
                                   observation_at( 179.5, 139.5, 2, 11.0 ) };
      if ( epoch_offset > 0 )
      {
        measurement.imu_samples = {
            ImuMeasurement{ .timestamp = phad::common::Timestamp{
                                kStereoEpochNs + epoch_offset - kStereoStepNs } },
            ImuMeasurement{ .timestamp = phad::common::Timestamp{ kStereoEpochNs + epoch_offset } },
        };
      }
      return measurement;
    };

    const phad::apps::FixedLagUpdateResult r1 =
        estimator.update( make_measurement( 0 ), true );
    ASSERT_EQ( r1.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r1.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kVisionOnly );
    // pose_prior + bias_prior + 2 stereo = 4
    EXPECT_EQ( r1.graph.active_factor_count, 4U );

    const phad::apps::FixedLagUpdateResult r2 = estimator.update(
        make_measurement( kStereoStepNs ), true );
    ASSERT_EQ( r2.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r2.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kGyroVisual );
    // + bias_rw + ahrs + 2 stereo = 8（无占位 Between）
    EXPECT_EQ( r2.graph.active_factor_count, 8U );

    const phad::apps::FixedLagUpdateResult r3 = estimator.update(
        make_measurement( 2 * kStereoStepNs ), true );
    ASSERT_EQ( r3.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r3.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kGyroVisual );
    EXPECT_EQ( r3.graph.active_factor_count, 12U );
    EXPECT_LE( r3.graph.active_pose_count, 3U );
    EXPECT_EQ( r3.graph.retired_landmark_count, 0U );
  }

  TEST( CandidatePipelineTest,
        FixedLagBiasChainAccumulatesInterKeyframeImu )
  {
    // P2b slice ④：AHRS 预积分覆盖完整 KF→KF 区间。KF1(ts=0) → 非 KF
    // 帧(ts=1，带 IMU 段) → KF2(ts=2，带 IMU 段)：pending_imu 去重累积
    // [0,1]∪[1,2]，KF2 的 gyro factor 用完整区间（fusion kGyroVisual），
    // 非 KF 帧不更新图（factor 数不变）。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    estimator_options.enable_imu  = true;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    using phad::sensor::ImuMeasurement;
    // 两个 landmark 分散在不同方向与深度（避免近中心同向观测的
    // Jacobian 秩亏导致 pose 欠约束）。
    const auto observation_at = []( double u, double v, std::uint64_t id,
                                    double disparity ) {
      return phad::estimator::StereoObservation{
          .id           = id,
          .left_pixel   = Eigen::Vector2d{ u, v },
          .disparity_px = disparity,
      };
    };
    const auto segment = []( std::int64_t from_ns, std::int64_t to_ns ) {
      return std::vector<ImuMeasurement>{
          ImuMeasurement{ .timestamp = phad::common::Timestamp{ from_ns } },
          ImuMeasurement{ .timestamp = phad::common::Timestamp{ to_ns } },
      };
    };
    const auto make_measurement = [ & ]( std::int64_t epoch_offset ) {
      phad::estimator::KeyframeMeasurement measurement;
      measurement.timestamp    = phad::common::Timestamp{ kStereoEpochNs +
                                                       epoch_offset };
      measurement.observations = { observation_at( 139.5, 99.5, 1, 13.75 ),
                                   observation_at( 179.5, 139.5, 2, 11.0 ) };
      if ( epoch_offset > 0 )
      {
        measurement.imu_samples = segment(
            kStereoEpochNs + epoch_offset - kStereoStepNs,
            kStereoEpochNs + epoch_offset );
      }
      return measurement;
    };

    const phad::apps::FixedLagUpdateResult r1 =
        estimator.update( make_measurement( 0 ), true );
    ASSERT_EQ( r1.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r1.graph.active_factor_count, 4U );

    // 非 KF 帧不进图（只老化）：factor 数不变；PnP 失败（2 观测
    // < 4 匹配）→ kRejected。
    const phad::apps::FixedLagUpdateResult r2 =
        estimator.update( make_measurement( kStereoStepNs ), false );
    ASSERT_EQ( r2.vio.status, UpdateStatus::kRejected );
    EXPECT_EQ( r2.graph.active_factor_count, 4U );
    EXPECT_EQ( r2.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kVisionOnly );

    // KF2：pending [0,1]∪[1,2] 累积 → gyro factor（完整区间）。
    const phad::apps::FixedLagUpdateResult r3 = estimator.update(
        make_measurement( 2 * kStereoStepNs ), true );
    ASSERT_EQ( r3.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r3.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kGyroVisual );
    // 4 + bias_rw + ahrs + 2 stereo = 8（无占位 Between）
    EXPECT_EQ( r3.graph.active_factor_count, 8U );
  }

  TEST( CandidatePipelineTest, FixedLagBiasChainSkipsGyroAfterGap )
  {
    // P2b slice ④：gap 打断 KF→KF 区间（即使发生在非 KF 帧）时，下一
    // KF 不能拿 gap 后的部分 IMU 段建 gyro factor（sticky broken flag）。
    // KF1(ts=0) → 非 KF(ts=1, imu_gap=true) → KF2(ts=2, 带 [1,2] 段)：
    // KF2 保持 kVisionOnly，factor 只 + between_pose + bias_rw + stereo。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    estimator_options.enable_imu  = true;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    using phad::sensor::ImuMeasurement;
    // 两个 landmark 分散在不同方向与深度（避免近中心同向观测的
    // Jacobian 秩亏导致 pose 欠约束）。
    const auto observation_at = []( double u, double v, std::uint64_t id,
                                    double disparity ) {
      return phad::estimator::StereoObservation{
          .id           = id,
          .left_pixel   = Eigen::Vector2d{ u, v },
          .disparity_px = disparity,
      };
    };
    const auto make_measurement = [ & ]( std::int64_t epoch_offset,
                                         bool         gap ) {
      phad::estimator::KeyframeMeasurement measurement;
      measurement.timestamp    = phad::common::Timestamp{ kStereoEpochNs +
                                                       epoch_offset };
      measurement.observations = { observation_at( 139.5, 99.5, 1, 13.75 ),
                                   observation_at( 179.5, 139.5, 2, 11.0 ),
                                   observation_at( 159.5, 179.5, 3, 9.1667 ) };
      measurement.imu_gap      = gap;
      if ( epoch_offset > 0 && !gap )
      {
        measurement.imu_samples = {
            ImuMeasurement{ .timestamp = phad::common::Timestamp{
                                kStereoEpochNs + epoch_offset - kStereoStepNs } },
            ImuMeasurement{ .timestamp = phad::common::Timestamp{ kStereoEpochNs + epoch_offset } },
        };
      }
      return measurement;
    };

    const phad::apps::FixedLagUpdateResult r1 =
        estimator.update( make_measurement( 0, false ), true );
    ASSERT_EQ( r1.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r1.graph.active_factor_count, 5U );

    // gap 帧不进图（只老化）：factor 数不变；PnP 失败 → kRejected。
    const phad::apps::FixedLagUpdateResult r2 =
        estimator.update( make_measurement( kStereoStepNs, true ), false );
    ASSERT_EQ( r2.vio.status, UpdateStatus::kRejected );
    EXPECT_EQ( r2.graph.active_factor_count, 5U );

    const phad::apps::FixedLagUpdateResult r3 = estimator.update(
        make_measurement( 2 * kStereoStepNs, false ), true );
    ASSERT_EQ( r3.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r3.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kVisionOnly );
    // 5 + bias_rw + 3 stereo = 9（无 ahrs、无占位 Between）
    EXPECT_EQ( r3.graph.active_factor_count, 9U );

    // gap 后的第一个完整区间（[2,3] 从 KF2 重新累积）恢复融合。
    const phad::apps::FixedLagUpdateResult r4 = estimator.update(
        make_measurement( 3 * kStereoStepNs, false ), true );
    ASSERT_EQ( r4.vio.status, UpdateStatus::kOk );
    EXPECT_EQ( r4.vio.diagnostics.fusion_mode,
               phad::estimator::FusionMode::kGyroVisual );
    // 9 + bias_rw + ahrs + 3 stereo = 14
    EXPECT_EQ( r4.graph.active_factor_count, 14U );
  }

  TEST( CandidatePipelineTest, FixedLagNonKeyframePnpRecoversPose )
  {
    // P2b slice ⑤：非 KF 帧 PnP 成功路径。KF1（identity pose）插入
    // 12 个 landmark（世界系 = 相机系）；非 KF 帧相机运动到
    // T_W_B_true = Rz(0.05)·t(0.05, -0.02, 0.1)，观测为
    // T_W_B_true⁻¹ 投影 → solvePnPRansac 应恢复 ≈ 真值。
    const auto                        calibration = makeRectifiedCalibration();
    phad::estimator::EstimatorOptions estimator_options;
    estimator_options.window_size = 10;
    // 测试 landmark 数 <10：锚定门槛与测试构造匹配。
    estimator_options.min_shared_landmarks = 2;
    estimator_options.enable_imu           = false;

    phad::apps::CandidateFixedLagEstimator estimator(
        calibration, estimator_options );

    // 12 个点：z≈2.0 平面 4×3 网格（每列 z 加小 spread，避免退化采样）。
    const double                 fx = 250.0, cx = 159.5, cy = 119.5, baseline = 0.11;
    std::vector<Eigen::Vector3d> points_W;
    for ( int row = 0; row < 3; ++row )
    {
      for ( int col = 0; col < 4; ++col )
      {
        points_W.emplace_back( -0.3 + 0.2 * col, -0.2 + 0.2 * row,
                               2.0 + 0.05 * col );
      }
    }
    const auto make_observation = [ & ]( std::uint64_t          id,
                                         const Eigen::Vector3d& point_left ) {
      const double disparity = baseline * fx / point_left.z();
      return phad::estimator::StereoObservation{
          .id = id,
          .left_pixel =
              Eigen::Vector2d{ fx * point_left.x() / point_left.z() + cx,
                               calibration.fyPixels() * point_left.y() /
                                       point_left.z() +
                                   cy },
          .disparity_px = disparity,
      };
    };

    // KF1（identity）：观测即世界系点的相机系投影。
    phad::estimator::KeyframeMeasurement kf1;
    kf1.timestamp = phad::common::Timestamp{ kStereoEpochNs };
    for ( std::size_t i = 0; i < points_W.size(); ++i )
    {
      kf1.observations.push_back( make_observation( i + 1, points_W[ i ] ) );
    }
    const phad::apps::FixedLagUpdateResult r1 =
        estimator.update( kf1, true );
    ASSERT_EQ( r1.vio.status, UpdateStatus::kOk );

    // 非 KF 帧：真实位姿 T_W_B_true。
    const Eigen::Isometry3d T_W_B_true =
        Eigen::Translation3d( 0.05, -0.02, 0.1 ) *
        Eigen::AngleAxisd( 0.05, Eigen::Vector3d::UnitZ() );
    const Eigen::Isometry3d T_B_W_true = T_W_B_true.inverse();

    phad::estimator::KeyframeMeasurement moving;
    moving.timestamp =
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs };
    for ( std::size_t i = 0; i < points_W.size(); ++i )
    {
      const Eigen::Vector3d point_left = T_B_W_true * points_W[ i ];
      moving.observations.push_back( make_observation( i + 1, point_left ) );
    }
    const phad::apps::FixedLagUpdateResult r2 =
        estimator.update( moving, false );
    ASSERT_EQ( r2.vio.status, UpdateStatus::kOk ) << r2.vio.message;
    ASSERT_TRUE( r2.vio.estimate.has_value() );
    const Eigen::Matrix4d recovered = r2.vio.estimate->T_W_B.matrix();
    const Eigen::Matrix4d truth     = T_W_B_true.matrix();
    EXPECT_LT( ( recovered - truth ).norm(), 1e-3 )
        << "recovered:\n"
        << recovered << "\ntruth:\n"
        << truth;
  }

}  // namespace

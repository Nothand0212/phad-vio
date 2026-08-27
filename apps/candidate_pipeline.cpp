#include "apps/candidate_pipeline.hpp"

#include <exception>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "apps/candidate_fixed_lag_estimator.hpp"
#include "apps/stereo_vo_glue.hpp"
#include "phad/common/landmark_id.hpp"

namespace phad::apps
{

  struct CandidatePipeline::Impl
  {
    Impl( camera::RectifiedStereoCalibration calibration,
          CandidatePipelineOptions           candidate_options )
        : options( std::move( candidate_options ) ),
          tracker( calibration, options.tracker ),
          keyframe_gate( calibration, options.keyframe_timeout_ns ),
          estimator( std::move( calibration ), options.estimator )
    {
    }

    CandidateProgress terminate( CandidateErrorCode code,
                                 common::Timestamp  timestamp,
                                 std::string        detail )
    {
      const std::uint64_t frame_index = counts.frames;
      error                           = CandidateError{
                                    .code        = code,
                                    .frame_index = frame_index,
                                    .timestamp   = timestamp,
                                    .detail      = detail,
      };

      CandidateFrameDiagnostics frame;
      frame.frame_index = frame_index;
      frame.timestamp   = timestamp;
      frame.status      = estimator::UpdateStatus::kFailed;
      frame.message     = std::move( detail );
      diagnostics.push_back( std::move( frame ) );
      ++counts.frames;
      ++counts.failed;
      return CandidateProgress::kTerminalFailure;
    }

    CandidatePipelineOptions               options;
    frontend::StereoTracker                tracker;
    KeyframeEpochGate                      keyframe_gate;
    CandidateFixedLagEstimator             estimator;
    std::vector<common::TimedPose>         poses;
    std::vector<common::TimedPose>         keyframe_poses;
    std::vector<CandidateFrameDiagnostics> diagnostics;
    CandidateCounts                        counts;
    std::optional<CandidateError>          error;
    std::optional<common::Timestamp>       last_timestamp;
    /// skip-culled ids 的连续存活年龄（与 production zombie_drop_age
    /// 语义一致，见 offline_vo_session.cpp）。
    std::unordered_map<common::LandmarkId, int> zombie_age;
  };

  CandidateCreateResult CandidatePipeline::create(
      camera::RectifiedStereoCalibration calibration,
      CandidatePipelineOptions           options )
  {
    try
    {
      return CandidateCreateResult{ CandidatePipeline{
          std::make_unique<Impl>( std::move( calibration ),
                                  std::move( options ) ) } };
    }
    catch ( const std::exception& exception )
    {
      return CandidateCreateResult{ CandidateError{
          .code        = CandidateErrorCode::kInvalidConfig,
          .frame_index = std::nullopt,
          .timestamp   = std::nullopt,
          .detail      = exception.what(),
      } };
    }
  }

  CandidatePipeline::CandidatePipeline( std::unique_ptr<Impl> impl )
      : m_impl( std::move( impl ) )
  {
  }

  CandidatePipeline::~CandidatePipeline() = default;

  CandidatePipeline::CandidatePipeline( CandidatePipeline&& ) noexcept =
      default;

  CandidatePipeline& CandidatePipeline::operator=(
      CandidatePipeline&& ) noexcept = default;

  CandidateProgress CandidatePipeline::process(
      const CandidateFrameInput& input )
  {
    if ( m_impl->error.has_value() )
    {
      return CandidateProgress::kTerminalFailure;
    }

    if ( m_impl->last_timestamp.has_value() &&
         input.rectified.timestamp <= *m_impl->last_timestamp )
    {
      return m_impl->terminate(
          CandidateErrorCode::kInputContract, input.rectified.timestamp,
          "frame timestamp must be strictly increasing" );
    }

    const common::Timestamp expected_t_prev =
        m_impl->last_timestamp.value_or( input.rectified.timestamp );
    if ( input.t_prev != expected_t_prev )
    {
      return m_impl->terminate(
          CandidateErrorCode::kInputContract, input.rectified.timestamp,
          "t_prev does not match previous frame timestamp: expected " +
              std::to_string( expected_t_prev.nanoseconds() ) + ", got " +
              std::to_string( input.t_prev.nanoseconds() ) );
    }
    m_impl->last_timestamp = input.rectified.timestamp;

    if ( !input.imu_gap )
    {
      const bool first_frame = m_impl->counts.frames == 0U;
      if ( input.imu_samples.size() >= 2 )
      {
        if ( input.imu_samples.front().timestamp != input.t_prev ||
             input.imu_samples.back().timestamp !=
                 input.rectified.timestamp )
        {
          return m_impl->terminate(
              CandidateErrorCode::kInputContract, input.rectified.timestamp,
              "non-gap IMU interval endpoints must match [t_prev, t_cur]" );
        }
        for ( std::size_t i = 1; i < input.imu_samples.size(); ++i )
        {
          if ( input.imu_samples[ i ].timestamp <=
               input.imu_samples[ i - 1 ].timestamp )
          {
            return m_impl->terminate(
                CandidateErrorCode::kInputContract,
                input.rectified.timestamp,
                "non-gap IMU sample timestamps must be strictly increasing" );
          }
        }
      }
      else if ( !first_frame )
      {
        return m_impl->terminate(
            CandidateErrorCode::kInputContract, input.rectified.timestamp,
            "non-gap IMU interval must contain at least two samples" );
      }
    }

    const frontend::FrameTracks tracks =
        m_impl->tracker.process( input.rectified );

    // candidate-owned zombie-age 推进（与 production 时序一致：在
    // tracker.process 之后、gate/estimator 之前推进 age 并按阈值 drop）。
    std::uint32_t zombie_dropped = 0;
    if ( m_impl->options.tracks.zombie_drop_age > 0 )
    {
      std::unordered_set<common::LandmarkId> present;
      present.reserve( tracks.observations.size() );
      for ( const auto& obs : tracks.observations )
      {
        present.insert( obs.id );
      }
      for ( auto it = m_impl->zombie_age.begin();
            it != m_impl->zombie_age.end(); )
      {
        if ( present.count( it->first ) != 0U )
        {
          ++it->second;
          ++it;
        }
        else
        {
          it = m_impl->zombie_age.erase( it );
        }
      }
      std::vector<common::LandmarkId> aged_drop;
      for ( const auto& [ id, age ] : m_impl->zombie_age )
      {
        if ( age >= m_impl->options.tracks.zombie_drop_age )
        {
          aged_drop.push_back( id );
        }
      }
      if ( !aged_drop.empty() )
      {
        m_impl->tracker.dropTracks( aged_drop );
        for ( const common::LandmarkId id : aged_drop )
        {
          m_impl->zombie_age.erase( id );
        }
        zombie_dropped = static_cast<std::uint32_t>( aged_drop.size() );
      }
    }

    const KeyframeDecision decision = m_impl->keyframe_gate.decide( tracks );

    estimator::KeyframeMeasurement measurement =
        toKeyframeMeasurement( tracks );
    // 帧间 IMU 段随测量一并送入（slice ④：AHRS 预积分 + bias 链）。
    measurement.imu_samples.assign( input.imu_samples.begin(),
                                    input.imu_samples.end() );
    measurement.t_prev  = input.t_prev;
    measurement.imu_gap = input.imu_gap;
    FixedLagUpdateResult update_result;
    try
    {
      update_result = m_impl->estimator.update(
          measurement, decision.selected );
    }
    catch ( const std::exception& exception )
    {
      return m_impl->terminate(
          CandidateErrorCode::kEstimatorInvariant,
          input.rectified.timestamp,
          std::string( "fixed-lag estimator update failed: " ) +
              exception.what() );
    }
    const estimator::VioUpdateResult& update = update_result.vio;

    // candidate-owned cull 反馈：estimator 永久移除的 landmark 同步从
    // candidate tracker 删除，与 production 的 drop/skip/zombie 语义
    // 一致（P2a twin 必需，见 mh01 分叉根因）。
    std::uint32_t dropped_track_count = 0;
    if ( m_impl->options.tracks.drop_culled_tracks &&
         !update.diagnostics.culled_landmark_ids.empty() )
    {
      const bool skip =
          m_impl->options.tracks.skip_drop_min_culled > 0 &&
          update.diagnostics.outliers_culled >=
              static_cast<std::uint32_t>(
                  m_impl->options.tracks.skip_drop_min_culled );
      if ( skip )
      {
        if ( m_impl->options.tracks.zombie_drop_age > 0 )
        {
          for ( const common::LandmarkId id :
                update.diagnostics.culled_landmark_ids )
          {
            m_impl->zombie_age.try_emplace( id, 1 );
          }
        }
      }
      else
      {
        m_impl->tracker.dropTracks(
            update.diagnostics.culled_landmark_ids );
        dropped_track_count = static_cast<std::uint32_t>(
            update.diagnostics.culled_landmark_ids.size() );
        for ( const common::LandmarkId id :
              update.diagnostics.culled_landmark_ids )
        {
          m_impl->zombie_age.erase( id );
        }
      }
    }
    dropped_track_count += zombie_dropped;

    KeyframeFeedback feedback;
    feedback.status = update.status;
    if ( update.status == estimator::UpdateStatus::kOk &&
         update.estimate.has_value() )
    {
      feedback.R_W_B = update.estimate->T_W_B.linear();
    }
    const KeyframeEvent event =
        m_impl->keyframe_gate.resolve( decision.ticket, feedback );

    CandidateFrameDiagnostics diagnostics;
    diagnostics.frame_index       = m_impl->counts.frames;
    diagnostics.timestamp         = tracks.timestamp;
    diagnostics.status            = update.status;
    diagnostics.message           = update.message;
    diagnostics.selected_keyframe = decision.selected;
    diagnostics.keyframe_rule     = decision.rule;
    diagnostics.epoch_committed   = event.epoch_committed;
    diagnostics.keyframe_epoch    = event.epoch;
    diagnostics.segment_id        = update.diagnostics.segment_id;
    diagnostics.reset_reason =
        m_impl->counts.frames == 0U && update.status == estimator::UpdateStatus::kOk
            ? CandidateResetReason::kBootstrap
            : CandidateResetReason::kNone;
    diagnostics.track_count =
        static_cast<std::uint32_t>( tracks.observations.size() );
    diagnostics.observation_count = update.diagnostics.num_observations;
    diagnostics.disparity_count   = update.diagnostics.num_disparity;
    diagnostics.shared_count      = update.diagnostics.num_shared;
    diagnostics.landmark_count    = update.diagnostics.num_landmarks;
    diagnostics.culled_count      = static_cast<std::uint32_t>(
        update.diagnostics.culled_landmark_ids.size() );
    diagnostics.dropped_track_count = dropped_track_count;
    diagnostics.fusion_mode         = update.diagnostics.fusion_mode;
    CandidateGraphDiagnostics graph;
    graph.active_pose_count       = update_result.graph.active_pose_count;
    graph.active_factor_count     = update_result.graph.active_factor_count;
    graph.marginalized_pose_count = update_result.graph.marginalized_pose_count;
    graph.retired_landmark_count  = update_result.graph.retired_landmark_count;
    diagnostics.graph             = graph;

    ++m_impl->counts.frames;
    if ( decision.selected )
    {
      ++m_impl->counts.keyframes;
    }
    else
    {
      ++m_impl->counts.track_only_frames;
    }

    switch ( update.status )
    {
      case estimator::UpdateStatus::kOk:
      {
        ++m_impl->counts.ok;
        CandidateState state;
        state.T_W_B       = update.estimate->T_W_B;
        state.bias_gyro   = update.diagnostics.bias_gyro;
        diagnostics.state = state;

        const common::TimedPose pose{
            .timestamp = update.estimate->timestamp,
            .T_W_B     = update.estimate->T_W_B,
        };
        m_impl->poses.push_back( pose );
        if ( event.epoch_committed )
        {
          m_impl->keyframe_poses.push_back( pose );
        }
        break;
      }
      case estimator::UpdateStatus::kRejected:
        ++m_impl->counts.rejected;
        break;
      case estimator::UpdateStatus::kFailed:
        ++m_impl->counts.failed;
        break;
    }

    m_impl->diagnostics.push_back( std::move( diagnostics ) );
    return CandidateProgress::kRunning;
  }

  CandidateRunResult CandidatePipeline::finish() &&
  {
    CandidateRunResult result;
    result.diagnostics = std::move( m_impl->diagnostics );
    result.counts      = m_impl->counts;
    result.error       = std::move( m_impl->error );

    if ( result.error.has_value() )
    {
      return result;
    }

    auto trajectory = common::Trajectory::create( std::move( m_impl->poses ) );
    if ( !trajectory )
    {
      result.error = CandidateError{
          .code        = CandidateErrorCode::kTrajectoryInvariant,
          .frame_index = std::nullopt,
          .timestamp   = std::nullopt,
          .detail      = trajectory.error().detail,
      };
      return result;
    }
    result.trajectory = std::move( trajectory ).value();

    if ( !m_impl->keyframe_poses.empty() )
    {
      auto keyframe_trajectory =
          common::Trajectory::create( std::move( m_impl->keyframe_poses ) );
      if ( !keyframe_trajectory )
      {
        result.trajectory.reset();
        result.error = CandidateError{
            .code        = CandidateErrorCode::kTrajectoryInvariant,
            .frame_index = std::nullopt,
            .timestamp   = std::nullopt,
            .detail      = keyframe_trajectory.error().detail,
        };
        return result;
      }
      result.keyframe_trajectory =
          std::move( keyframe_trajectory ).value();
    }
    return result;
  }

  CandidateCreateResult::CandidateCreateResult( CandidatePipeline value )
      : m_storage( std::move( value ) )
  {
  }

  CandidateCreateResult::CandidateCreateResult( CandidateError error )
      : m_storage( std::move( error ) )
  {
  }

  bool CandidateCreateResult::hasValue() const noexcept
  {
    return std::holds_alternative<CandidatePipeline>( m_storage );
  }

  CandidateCreateResult::operator bool() const noexcept
  {
    return hasValue();
  }

  CandidatePipeline&& CandidateCreateResult::value() &&
  {
    return std::get<CandidatePipeline>( std::move( m_storage ) );
  }

  const CandidateError& CandidateCreateResult::error() const&
  {
    return std::get<CandidateError>( m_storage );
  }

}  // namespace phad::apps

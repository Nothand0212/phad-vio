#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "apps/keyframe_epoch_gate.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/common/trajectory.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::apps
{

  struct CandidateTrackPolicy
  {
    bool drop_culled_tracks   = true;
    int  skip_drop_min_culled = 4;
    int  zombie_drop_age      = 5;
  };

  struct CandidatePipelineOptions
  {
    frontend::StereoTrackerOptions tracker;
    estimator::EstimatorOptions    estimator;
    CandidateTrackPolicy           tracks;
    /// KF timeout 间隔（candidate 用短间隔保证 KF 与旧图共享 landmark；
    /// MH_01 20 fps 下 150 ms = 3 帧，track 存活 5-10 帧内）。
    std::int64_t keyframe_timeout_ns = 150'000'000;
  };

  struct CandidateFrameInput
  {
    const sensor::StereoFrame&              rectified;
    std::span<const sensor::ImuMeasurement> imu_samples;
    common::Timestamp                       t_prev{ 0 };
    bool                                    imu_gap = false;
  };

  enum class CandidateProgress : std::uint8_t
  {
    kRunning         = 0,
    kTerminalFailure = 1
  };

  enum class CandidateErrorCode : std::uint8_t
  {
    kInvalidConfig       = 0,
    kInputContract       = 1,
    kPipelineInvariant   = 2,
    kEstimatorInvariant  = 3,
    kTrajectoryInvariant = 4
  };

  struct CandidateError
  {
    CandidateErrorCode               code = CandidateErrorCode::kPipelineInvariant;
    std::optional<std::uint64_t>     frame_index;
    std::optional<common::Timestamp> timestamp;
    std::string                      detail;
  };

  enum class CandidateResetReason : std::uint8_t
  {
    kNone      = 0,
    kBootstrap = 1,
    kSegment   = 2
  };

  struct CandidateState
  {
    Eigen::Isometry3d              T_W_B = Eigen::Isometry3d::Identity();
    std::optional<Eigen::Vector3d> velocity_W;
    Eigen::Vector3d                bias_gyro = Eigen::Vector3d::Zero();
    std::optional<Eigen::Vector3d> bias_acc;
  };

  struct CandidateGraphDiagnostics
  {
    std::uint32_t active_pose_count       = 0;
    std::uint32_t active_factor_count     = 0;
    std::uint32_t marginalized_pose_count = 0;
    std::uint32_t retired_landmark_count  = 0;
  };

  struct CandidateFrameDiagnostics
  {
    std::uint64_t           frame_index = 0;
    common::Timestamp       timestamp{ 0 };
    estimator::UpdateStatus status = estimator::UpdateStatus::kFailed;
    std::string             message;

    bool                 selected_keyframe = false;
    KeyframeRule         keyframe_rule     = KeyframeRule::kNone;
    bool                 epoch_committed   = false;
    std::uint64_t        keyframe_epoch    = 0;
    std::uint32_t        segment_id        = 0;
    CandidateResetReason reset_reason      = CandidateResetReason::kNone;

    std::uint32_t track_count         = 0;
    std::uint32_t observation_count   = 0;
    std::uint32_t disparity_count     = 0;
    std::uint32_t shared_count        = 0;
    std::uint32_t landmark_count      = 0;
    std::uint32_t culled_count        = 0;
    std::uint32_t dropped_track_count = 0;

    estimator::FusionMode                    fusion_mode = estimator::FusionMode::kVisionOnly;
    std::optional<CandidateState>            state;
    std::optional<CandidateGraphDiagnostics> graph;
  };

  struct CandidateCounts
  {
    std::uint64_t frames            = 0;
    std::uint64_t ok                = 0;
    std::uint64_t rejected          = 0;
    std::uint64_t failed            = 0;
    std::uint64_t keyframes         = 0;
    std::uint64_t track_only_frames = 0;
  };

  struct CandidateRunResult
  {
    std::optional<common::Trajectory>      trajectory;
    std::optional<common::Trajectory>      keyframe_trajectory;
    std::vector<CandidateFrameDiagnostics> diagnostics;
    CandidateCounts                        counts;
    std::optional<CandidateError>          error;
  };

  class CandidateCreateResult;

  class CandidatePipeline
  {
  public:
    [[nodiscard]] static CandidateCreateResult create(
        camera::RectifiedStereoCalibration calibration,
        CandidatePipelineOptions           options );

    ~CandidatePipeline();

    CandidatePipeline( const CandidatePipeline& )            = delete;
    CandidatePipeline& operator=( const CandidatePipeline& ) = delete;
    CandidatePipeline( CandidatePipeline&& ) noexcept;
    CandidatePipeline& operator=( CandidatePipeline&& ) noexcept;

    [[nodiscard]] CandidateProgress process(
        const CandidateFrameInput& input );

    [[nodiscard]] CandidateRunResult finish() &&;

  private:
    struct Impl;
    explicit CandidatePipeline( std::unique_ptr<Impl> impl );

    std::unique_ptr<Impl> m_impl;
  };

  class CandidateCreateResult
  {
  public:
    CandidateCreateResult( const CandidateCreateResult& )                = delete;
    CandidateCreateResult& operator=( const CandidateCreateResult& )     = delete;
    CandidateCreateResult( CandidateCreateResult&& ) noexcept            = default;
    CandidateCreateResult& operator=( CandidateCreateResult&& ) noexcept = default;

    [[nodiscard]] bool hasValue() const noexcept;
    explicit           operator bool() const noexcept;

    [[nodiscard]] CandidatePipeline&&   value() &&;
    [[nodiscard]] const CandidateError& error() const&;

  private:
    friend class CandidatePipeline;

    explicit CandidateCreateResult( CandidatePipeline value );
    explicit CandidateCreateResult( CandidateError error );

    std::variant<CandidatePipeline, CandidateError> m_storage;
  };

}  // namespace phad::apps

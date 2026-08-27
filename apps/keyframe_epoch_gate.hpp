#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"

namespace phad::apps
{

  enum class KeyframeRule : std::uint8_t
  {
    kNone        = 0,
    kBootstrap   = 1,
    kLowTracks   = 2,
    kTimeout     = 3,
    kLowSurvival = 4,
    kParallax    = 5
  };

  struct KeyframeTriggers
  {
    bool empty        = false;
    bool bootstrap    = false;
    bool low_tracks   = false;
    bool timeout      = false;
    bool low_survival = false;
    bool parallax     = false;
  };

  struct KeyframeEvidence
  {
    common::Timestamp timestamp{ 0 };
    std::uint64_t     epoch                   = 0;
    std::size_t       observation_count       = 0;
    std::size_t       common_count            = 0;
    std::size_t       parallax_count          = 0;
    std::int64_t      since_keyframe_ns       = 0;
    double            survival_ratio          = 0.0;
    double            raw_parallax_px         = 0.0;
    double            compensated_parallax_px = 0.0;
    KeyframeTriggers  triggers;
  };

  struct KeyframeDecision
  {
    std::uint64_t    ticket   = 0;
    bool             selected = false;
    KeyframeRule     rule     = KeyframeRule::kNone;
    KeyframeEvidence evidence;
  };

  struct KeyframeRotationEvidence
  {
    std::size_t common_count            = 0;
    std::size_t parallax_count          = 0;
    double      raw_parallax_px         = 0.0;
    double      compensated_parallax_px = 0.0;
  };

  struct KeyframeFeedback
  {
    estimator::UpdateStatus        status = estimator::UpdateStatus::kFailed;
    std::optional<Eigen::Matrix3d> R_W_B;
  };

  struct KeyframeEvent
  {
    std::uint64_t           ticket          = 0;
    bool                    selected        = false;
    estimator::UpdateStatus status          = estimator::UpdateStatus::kFailed;
    bool                    epoch_committed = false;
    std::uint64_t           epoch           = 0;
  };

  class KeyframeEpochGate
  {
  public:
    /// timeout_ns：KF timeout 间隔（candidate 用短间隔保证与旧图共享
    /// landmark；production 用默认 500 ms）。
    explicit KeyframeEpochGate(
        camera::RectifiedStereoCalibration calibration,
        std::int64_t                       timeout_ns = 500'000'000 );
    ~KeyframeEpochGate();

    KeyframeEpochGate( const KeyframeEpochGate& )            = delete;
    KeyframeEpochGate& operator=( const KeyframeEpochGate& ) = delete;
    KeyframeEpochGate( KeyframeEpochGate&& ) noexcept;
    KeyframeEpochGate& operator=( KeyframeEpochGate&& ) noexcept;

    [[nodiscard]] KeyframeDecision decide(
        const frontend::FrameTracks& tracks );

    [[nodiscard]] KeyframeRotationEvidence evaluateRotation(
        std::uint64_t ticket, const frontend::FrameTracks& tracks,
        const Eigen::Matrix3d& R_kf_to_cur ) const;

    [[nodiscard]] KeyframeEvent resolve(
        std::uint64_t ticket, const KeyframeFeedback& feedback );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps

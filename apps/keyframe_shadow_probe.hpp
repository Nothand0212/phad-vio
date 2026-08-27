#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>

#include "apps/keyframe_epoch_gate.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace phad::apps
{

  class KeyframeShadowProbe
  {
  public:
    KeyframeShadowProbe(
        const std::filesystem::path&       path,
        camera::RectifiedStereoCalibration calibration, bool imu_enabled );
    ~KeyframeShadowProbe();

    KeyframeShadowProbe( const KeyframeShadowProbe& )            = delete;
    KeyframeShadowProbe& operator=( const KeyframeShadowProbe& ) = delete;
    KeyframeShadowProbe( KeyframeShadowProbe&& )                 = delete;
    KeyframeShadowProbe& operator=( KeyframeShadowProbe&& )      = delete;

    void observe(
        std::uint64_t                           frame,
        std::span<const sensor::ImuMeasurement> imu_samples, bool imu_gap,
        const frontend::FrameTracks& tracks,
        const KeyframeDecision& decision, const KeyframeEpochGate& gate );

    void resolve( const KeyframeEvent&   event,
                  const Eigen::Vector3d& bias_gyr );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps

#pragma once

#include <memory>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"

namespace phad::estimator
{

  class StereoVoEstimator
  {
  public:
    explicit StereoVoEstimator( camera::RectifiedStereoCalibration calibration,
                                EstimatorOptions                   options = {} );
    ~StereoVoEstimator();

    StereoVoEstimator( const StereoVoEstimator& )            = delete;
    StereoVoEstimator& operator=( const StereoVoEstimator& ) = delete;
    StereoVoEstimator( StereoVoEstimator&& ) noexcept;
    StereoVoEstimator& operator=( StereoVoEstimator&& ) noexcept;

    [[nodiscard]] VioUpdateResult update(
        const KeyframeMeasurement& measurement,
        bool                       keyframe = true );

    /// Timestamps of accepted frames that observed `id` (diagnostic; survives
    /// window/landmark pruning).
    [[nodiscard]] std::vector<common::Timestamp> observationTimestamps(
        LandmarkId id ) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::estimator

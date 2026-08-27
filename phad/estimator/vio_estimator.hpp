#pragma once

#include <memory>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/sensor/imu_parameters.hpp"

namespace phad::estimator
{

  class VioEstimator
  {
  public:
    VioEstimator( camera::RectifiedStereoCalibration calibration,
                  sensor::ImuParameters              imu,
                  EstimatorOptions                   options = {} );
    ~VioEstimator();

    VioEstimator( const VioEstimator& )            = delete;
    VioEstimator& operator=( const VioEstimator& ) = delete;
    VioEstimator( VioEstimator&& ) noexcept;
    VioEstimator& operator=( VioEstimator&& ) noexcept;

    [[nodiscard]] VioUpdateResult update(
        const VioMeasurement& measurement,
        bool                  keyframe = true );

    /// Timestamps of accepted frames that observed `id` (diagnostic; survives
    /// window/landmark pruning).
    [[nodiscard]] std::vector<common::Timestamp> observationTimestamps(
        LandmarkId id ) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::estimator

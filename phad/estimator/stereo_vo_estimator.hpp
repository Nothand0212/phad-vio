#pragma once

#include <memory>

#include "phad/camera/rectified_stereo_calibration.hpp"
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
        const KeyframeMeasurement& measurement );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::estimator

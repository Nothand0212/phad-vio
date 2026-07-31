#pragma once

#include <memory>

#include "phad/camera/camera_model.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/sensor/stereo_imu_calibration.hpp"

namespace phad::camera
{

  /**
   * @brief Whole-image stereo rectification for radtan calibrations.
   *
   * Public headers stay OpenCV-free; remap tables live in the PIMPL.
   * Equidistant models are rejected with `kOutsideModelDomain`.
   */
  class StereoRectifier
  {
  public:
    [[nodiscard]] static CameraModelResult<StereoRectifier> create(
        const sensor::StereoImuCalibration& calibration );

    ~StereoRectifier();
    StereoRectifier( const StereoRectifier& )            = delete;
    StereoRectifier& operator=( const StereoRectifier& ) = delete;
    StereoRectifier( StereoRectifier&& ) noexcept;
    StereoRectifier& operator=( StereoRectifier&& ) noexcept;

    [[nodiscard]] const RectifiedStereoCalibration& calibration()
        const noexcept;

    [[nodiscard]] CameraModelResult<sensor::StereoFrame> rectify(
        const sensor::StereoFrame& raw_frame ) const;

  private:
    struct Impl;

    explicit StereoRectifier( std::unique_ptr<Impl> impl );

    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::camera

#pragma once

#include "phad/sensor/calibration_error.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace phad::camera
{

  /**
   * @brief Post-rectification stereo calibration shared by both cameras.
   *
   * Intrinsics are zero-distortion and shared; baseline is along the rectified
   * x axis. `T_B_left_rectified` maps points from the rectified left camera
   * frame into the body frame.
   */
  class RectifiedStereoCalibration
  {
  public:
    [[nodiscard]] static sensor::CalibrationResult<RectifiedStereoCalibration>
    create( double fx_pixels, double fy_pixels, double cx_pixels,
            double cy_pixels, double baseline_m, int image_width,
            int image_height, sensor::RigidTransform T_B_left_rectified );

    [[nodiscard]] double fxPixels() const noexcept;
    [[nodiscard]] double fyPixels() const noexcept;
    [[nodiscard]] double cxPixels() const noexcept;
    [[nodiscard]] double cyPixels() const noexcept;
    [[nodiscard]] double baselineM() const noexcept;  // meters
    [[nodiscard]] int    imageWidth() const noexcept;
    [[nodiscard]] int    imageHeight() const noexcept;
    [[nodiscard]] const sensor::RigidTransform& T_B_left_rectified()
        const noexcept;

  private:
    RectifiedStereoCalibration(
        double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
        double baseline_m, int image_width, int image_height,
        sensor::RigidTransform T_B_left_rectified );

    double                 m_fx_pixels;
    double                 m_fy_pixels;
    double                 m_cx_pixels;
    double                 m_cy_pixels;
    double                 m_baseline_m;
    int                    m_image_width;
    int                    m_image_height;
    sensor::RigidTransform m_T_B_left_rectified;
  };

}  // namespace phad::camera

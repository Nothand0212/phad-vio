#include "phad/camera/rectified_stereo_calibration.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace phad::camera
{

  sensor::CalibrationResult<RectifiedStereoCalibration>
  RectifiedStereoCalibration::create(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double baseline_m, int image_width, int image_height,
      sensor::RigidTransform T_B_left_rectified )
  {
    const std::array<std::pair<double, const char*>, 5> finite_params{
        std::pair{ fx_pixels, "rectified.fx_pixels" },
        std::pair{ fy_pixels, "rectified.fy_pixels" },
        std::pair{ cx_pixels, "rectified.cx_pixels" },
        std::pair{ cy_pixels, "rectified.cy_pixels" },
        std::pair{ baseline_m, "rectified.baseline_m" } };
    for ( const auto& [ value, field_path ] : finite_params )
    {
      if ( !std::isfinite( value ) )
      {
        return sensor::CalibrationError{
            sensor::CalibrationErrorCode::kNonFiniteValue, field_path,
            "rectified stereo parameter must be finite" };
      }
    }
    if ( fx_pixels <= 0.0 )
    {
      return sensor::CalibrationError{
          sensor::CalibrationErrorCode::kNonPositiveValue,
          "rectified.fx_pixels", "fx_pixels must be strictly positive" };
    }
    if ( fy_pixels <= 0.0 )
    {
      return sensor::CalibrationError{
          sensor::CalibrationErrorCode::kNonPositiveValue,
          "rectified.fy_pixels", "fy_pixels must be strictly positive" };
    }
    if ( baseline_m <= 0.0 )
    {
      return sensor::CalibrationError{
          sensor::CalibrationErrorCode::kZeroStereoBaseline,
          "rectified.baseline_m",
          "baseline_m must be strictly positive" };
    }
    if ( image_width <= 0 )
    {
      return sensor::CalibrationError{
          sensor::CalibrationErrorCode::kNonPositiveValue,
          "rectified.image_width",
          "image_width must be strictly positive" };
    }
    if ( image_height <= 0 )
    {
      return sensor::CalibrationError{
          sensor::CalibrationErrorCode::kNonPositiveValue,
          "rectified.image_height",
          "image_height must be strictly positive" };
    }
    return RectifiedStereoCalibration(
        fx_pixels, fy_pixels, cx_pixels, cy_pixels, baseline_m, image_width,
        image_height, std::move( T_B_left_rectified ) );
  }

  RectifiedStereoCalibration::RectifiedStereoCalibration(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double baseline_m, int image_width, int image_height,
      sensor::RigidTransform T_B_left_rectified )
      : m_fx_pixels( fx_pixels ),
        m_fy_pixels( fy_pixels ),
        m_cx_pixels( cx_pixels ),
        m_cy_pixels( cy_pixels ),
        m_baseline_m( baseline_m ),
        m_image_width( image_width ),
        m_image_height( image_height ),
        m_T_B_left_rectified( std::move( T_B_left_rectified ) )
  {
  }

  double RectifiedStereoCalibration::fxPixels() const noexcept
  {
    return m_fx_pixels;
  }

  double RectifiedStereoCalibration::fyPixels() const noexcept
  {
    return m_fy_pixels;
  }

  double RectifiedStereoCalibration::cxPixels() const noexcept
  {
    return m_cx_pixels;
  }

  double RectifiedStereoCalibration::cyPixels() const noexcept
  {
    return m_cy_pixels;
  }

  double RectifiedStereoCalibration::baselineM() const noexcept
  {
    return m_baseline_m;
  }

  int RectifiedStereoCalibration::imageWidth() const noexcept
  {
    return m_image_width;
  }

  int RectifiedStereoCalibration::imageHeight() const noexcept
  {
    return m_image_height;
  }

  const sensor::RigidTransform&
  RectifiedStereoCalibration::T_B_left_rectified() const noexcept
  {
    return m_T_B_left_rectified;
  }

}  // namespace phad::camera

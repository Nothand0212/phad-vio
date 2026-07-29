#include "phad/sensor/camera_parameters.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace phad::sensor
{
  CalibrationResult<PinholeRadialTangentialParameters>
  PinholeRadialTangentialParameters::create(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double k1, double k2, double p1, double p2 )
  {
    const std::array<std::pair<double, const char*>, 8> parameters{
        std::pair{
            fx_pixels,
            "camera.model_parameters.radial_tangential.fx_pixels" },
        std::pair{
            fy_pixels,
            "camera.model_parameters.radial_tangential.fy_pixels" },
        std::pair{
            cx_pixels,
            "camera.model_parameters.radial_tangential.cx_pixels" },
        std::pair{
            cy_pixels,
            "camera.model_parameters.radial_tangential.cy_pixels" },
        std::pair{ k1, "camera.model_parameters.radial_tangential.k1" },
        std::pair{ k2, "camera.model_parameters.radial_tangential.k2" },
        std::pair{ p1, "camera.model_parameters.radial_tangential.p1" },
        std::pair{ p2, "camera.model_parameters.radial_tangential.p2" } };
    for ( const auto& [ value, field_path ] : parameters )
    {
      if ( !std::isfinite( value ) )
      {
        return CalibrationError{ CalibrationErrorCode::kNonFiniteValue,
                                 field_path,
                                 "camera model parameter must be finite" };
      }
    }
    if ( fx_pixels <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fx_pixels",
          "fx_pixels must be strictly positive" };
    }
    if ( fy_pixels <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fy_pixels",
          "fy_pixels must be strictly positive" };
    }
    return PinholeRadialTangentialParameters(
        fx_pixels, fy_pixels, cx_pixels, cy_pixels, k1, k2, p1, p2 );
  }

  PinholeRadialTangentialParameters::PinholeRadialTangentialParameters(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double k1, double k2, double p1, double p2 )
      : m_fx_pixels( fx_pixels ),
        m_fy_pixels( fy_pixels ),
        m_cx_pixels( cx_pixels ),
        m_cy_pixels( cy_pixels ),
        m_k1( k1 ),
        m_k2( k2 ),
        m_p1( p1 ),
        m_p2( p2 )
  {
  }

  double PinholeRadialTangentialParameters::fxPixels() const noexcept
  {
    return m_fx_pixels;
  }

  double PinholeRadialTangentialParameters::fyPixels() const noexcept
  {
    return m_fy_pixels;
  }

  double PinholeRadialTangentialParameters::cxPixels() const noexcept
  {
    return m_cx_pixels;
  }

  double PinholeRadialTangentialParameters::cyPixels() const noexcept
  {
    return m_cy_pixels;
  }

  double PinholeRadialTangentialParameters::k1() const noexcept
  {
    return m_k1;
  }

  double PinholeRadialTangentialParameters::k2() const noexcept
  {
    return m_k2;
  }

  double PinholeRadialTangentialParameters::p1() const noexcept
  {
    return m_p1;
  }

  double PinholeRadialTangentialParameters::p2() const noexcept
  {
    return m_p2;
  }

  CalibrationResult<PinholeEquidistantParameters>
  PinholeEquidistantParameters::create(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double k1, double k2, double k3, double k4 )
  {
    const std::array<std::pair<double, const char*>, 8> parameters{
        std::pair{
            fx_pixels,
            "camera.model_parameters.equidistant.fx_pixels" },
        std::pair{
            fy_pixels,
            "camera.model_parameters.equidistant.fy_pixels" },
        std::pair{
            cx_pixels,
            "camera.model_parameters.equidistant.cx_pixels" },
        std::pair{
            cy_pixels,
            "camera.model_parameters.equidistant.cy_pixels" },
        std::pair{ k1, "camera.model_parameters.equidistant.k1" },
        std::pair{ k2, "camera.model_parameters.equidistant.k2" },
        std::pair{ k3, "camera.model_parameters.equidistant.k3" },
        std::pair{ k4, "camera.model_parameters.equidistant.k4" } };
    for ( const auto& [ value, field_path ] : parameters )
    {
      if ( !std::isfinite( value ) )
      {
        return CalibrationError{ CalibrationErrorCode::kNonFiniteValue,
                                 field_path,
                                 "camera model parameter must be finite" };
      }
    }
    if ( fx_pixels <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fx_pixels",
          "fx_pixels must be strictly positive" };
    }
    if ( fy_pixels <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fy_pixels",
          "fy_pixels must be strictly positive" };
    }
    return PinholeEquidistantParameters(
        fx_pixels, fy_pixels, cx_pixels, cy_pixels, k1, k2, k3, k4 );
  }

  PinholeEquidistantParameters::PinholeEquidistantParameters(
      double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
      double k1, double k2, double k3, double k4 )
      : m_fx_pixels( fx_pixels ),
        m_fy_pixels( fy_pixels ),
        m_cx_pixels( cx_pixels ),
        m_cy_pixels( cy_pixels ),
        m_k1( k1 ),
        m_k2( k2 ),
        m_k3( k3 ),
        m_k4( k4 )
  {
  }

  double PinholeEquidistantParameters::fxPixels() const noexcept
  {
    return m_fx_pixels;
  }

  double PinholeEquidistantParameters::fyPixels() const noexcept
  {
    return m_fy_pixels;
  }

  double PinholeEquidistantParameters::cxPixels() const noexcept
  {
    return m_cx_pixels;
  }

  double PinholeEquidistantParameters::cyPixels() const noexcept
  {
    return m_cy_pixels;
  }

  double PinholeEquidistantParameters::k1() const noexcept { return m_k1; }

  double PinholeEquidistantParameters::k2() const noexcept { return m_k2; }

  double PinholeEquidistantParameters::k3() const noexcept { return m_k3; }

  double PinholeEquidistantParameters::k4() const noexcept { return m_k4; }

  CalibrationResult<CameraParameters> CameraParameters::create(
      CameraModelParameters model_parameters, int image_width,
      int image_height, double rate_hz )
  {
    if ( image_width <= 0 )
    {
      return CalibrationError{ CalibrationErrorCode::kNonPositiveValue,
                               "camera.image_width",
                               "image width must be positive" };
    }
    if ( image_height <= 0 )
    {
      return CalibrationError{ CalibrationErrorCode::kNonPositiveValue,
                               "camera.image_height",
                               "image height must be positive" };
    }
    if ( !std::isfinite( rate_hz ) )
    {
      return CalibrationError{ CalibrationErrorCode::kNonFiniteValue,
                               "camera.rate_hz",
                               "declared sample rate must be finite" };
    }
    if ( rate_hz <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kNonPositiveValue,
          "camera.rate_hz",
          "declared sample rate must be strictly positive" };
    }
    return CameraParameters( std::move( model_parameters ),
                             static_cast<std::uint32_t>( image_width ),
                             static_cast<std::uint32_t>( image_height ),
                             rate_hz );
  }

  CameraParameters::CameraParameters(
      CameraModelParameters model_parameters, std::uint32_t image_width,
      std::uint32_t image_height, double rate_hz )
      : m_model_parameters( std::move( model_parameters ) ),
        m_image_width( image_width ),
        m_image_height( image_height ),
        m_rate_hz( rate_hz )
  {
  }

  const CameraModelParameters& CameraParameters::modelParameters()
      const noexcept
  {
    return m_model_parameters;
  }

  std::uint32_t CameraParameters::imageWidth() const noexcept
  {
    return m_image_width;
  }

  std::uint32_t CameraParameters::imageHeight() const noexcept
  {
    return m_image_height;
  }

  double CameraParameters::rateHz() const noexcept { return m_rate_hz; }

}  // namespace phad::sensor

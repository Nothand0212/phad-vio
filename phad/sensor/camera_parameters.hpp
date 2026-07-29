#pragma once

#include <cstdint>
#include <variant>

#include "phad/sensor/calibration_error.hpp"

namespace phad::sensor
{

  class PinholeRadialTangentialParameters
  {
  public:
    [[nodiscard]] static CalibrationResult<
        PinholeRadialTangentialParameters>
    create( double fx_pixels, double fy_pixels, double cx_pixels,
            double cy_pixels, double k1, double k2, double p1, double p2 );

    [[nodiscard]] double fxPixels() const noexcept;
    [[nodiscard]] double fyPixels() const noexcept;
    [[nodiscard]] double cxPixels() const noexcept;
    [[nodiscard]] double cyPixels() const noexcept;
    [[nodiscard]] double k1() const noexcept;
    [[nodiscard]] double k2() const noexcept;
    [[nodiscard]] double p1() const noexcept;
    [[nodiscard]] double p2() const noexcept;

  private:
    PinholeRadialTangentialParameters(
        double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
        double k1, double k2, double p1, double p2 );

    double m_fx_pixels;
    double m_fy_pixels;
    double m_cx_pixels;
    double m_cy_pixels;
    double m_k1;
    double m_k2;
    double m_p1;
    double m_p2;
  };

  class PinholeEquidistantParameters
  {
  public:
    [[nodiscard]] static CalibrationResult<PinholeEquidistantParameters>
    create( double fx_pixels, double fy_pixels, double cx_pixels,
            double cy_pixels, double k1, double k2, double k3, double k4 );

    [[nodiscard]] double fxPixels() const noexcept;
    [[nodiscard]] double fyPixels() const noexcept;
    [[nodiscard]] double cxPixels() const noexcept;
    [[nodiscard]] double cyPixels() const noexcept;
    [[nodiscard]] double k1() const noexcept;
    [[nodiscard]] double k2() const noexcept;
    [[nodiscard]] double k3() const noexcept;
    [[nodiscard]] double k4() const noexcept;

  private:
    PinholeEquidistantParameters(
        double fx_pixels, double fy_pixels, double cx_pixels, double cy_pixels,
        double k1, double k2, double k3, double k4 );

    double m_fx_pixels;
    double m_fy_pixels;
    double m_cx_pixels;
    double m_cy_pixels;
    double m_k1;
    double m_k2;
    double m_k3;
    double m_k4;
  };

  using CameraModelParameters =
      std::variant<PinholeRadialTangentialParameters,
                   PinholeEquidistantParameters>;

  class CameraParameters
  {
  public:
    [[nodiscard]] static CalibrationResult<CameraParameters> create(
        CameraModelParameters model_parameters, int image_width,
        int image_height, double rate_hz );

    [[nodiscard]] const CameraModelParameters& modelParameters()
        const noexcept;
    [[nodiscard]] std::uint32_t imageWidth() const noexcept;
    [[nodiscard]] std::uint32_t imageHeight() const noexcept;
    [[nodiscard]] double        rateHz() const noexcept;

  private:
    CameraParameters( CameraModelParameters model_parameters,
                      std::uint32_t image_width, std::uint32_t image_height,
                      double rate_hz );

    CameraModelParameters m_model_parameters;
    std::uint32_t         m_image_width;
    std::uint32_t         m_image_height;
    double                m_rate_hz;
  };

}  // namespace phad::sensor

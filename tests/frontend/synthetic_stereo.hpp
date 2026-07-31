#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file synthetic_stereo.hpp
 * @brief Frontend tests: project known 3D points and paint Gaussian blobs.
 *
 * No OpenCV dependency — blobs are written into `sensor::Image` pixel buffers.
 */

namespace phad::testing
{

  inline constexpr std::int64_t kStereoEpochNs = 1'403'636'579'763'555'584;
  inline constexpr std::int64_t kStereoStepNs  = 50'000'000;

  [[nodiscard]] inline camera::RectifiedStereoCalibration
  makeRectifiedCalibration( int width = 320, int height = 240,
                            double fx = 250.0, double baseline_m = 0.11 )
  {
    auto T_B_left =
        sensor::RigidTransform::create( Eigen::Matrix4d::Identity() );
    auto calibration = camera::RectifiedStereoCalibration::create(
        fx, fx, 0.5 * static_cast<double>( width - 1 ),
        0.5 * static_cast<double>( height - 1 ), baseline_m, width, height,
        std::move( T_B_left ).value() );
    return std::move( calibration ).value();
  }

  [[nodiscard]] inline Eigen::Vector2d projectLeft(
      const camera::RectifiedStereoCalibration& calibration,
      const Eigen::Vector3d&                    point_left )
  {
    return { calibration.fxPixels() * point_left.x() / point_left.z() +
                 calibration.cxPixels(),
             calibration.fyPixels() * point_left.y() / point_left.z() +
                 calibration.cyPixels() };
  }

  [[nodiscard]] inline Eigen::Vector2d projectRight(
      const camera::RectifiedStereoCalibration& calibration,
      const Eigen::Vector3d&                    point_left )
  {
    const Eigen::Vector3d point_right( point_left.x() - calibration.baselineM(),
                                       point_left.y(), point_left.z() );
    return { calibration.fxPixels() * point_right.x() / point_right.z() +
                 calibration.cxPixels(),
             calibration.fyPixels() * point_right.y() / point_right.z() +
                 calibration.cyPixels() };
  }

  inline void paintGaussianBlob( std::vector<std::uint8_t>& pixels, int width,
                                 int height, double u, double v,
                                 double sigma_px = 2.0,
                                 double peak     = 255.0 )
  {
    const int radius =
        std::max( 1, static_cast<int>( std::ceil( 3.0 * sigma_px ) ) );
    const int u0 = static_cast<int>( std::floor( u ) );
    const int v0 = static_cast<int>( std::floor( v ) );
    const double inv_2s2 = 1.0 / ( 2.0 * sigma_px * sigma_px );

    for ( int dy = -radius; dy <= radius; ++dy )
    {
      const int y = v0 + dy;
      if ( y < 0 || y >= height )
      {
        continue;
      }
      for ( int dx = -radius; dx <= radius; ++dx )
      {
        const int x = u0 + dx;
        if ( x < 0 || x >= width )
        {
          continue;
        }
        const double xu = static_cast<double>( x ) + 0.5 - u;
        const double yv = static_cast<double>( y ) + 0.5 - v;
        const double weight = std::exp( -( xu * xu + yv * yv ) * inv_2s2 );
        const auto   index =
            static_cast<std::size_t>( y * width + x );
        const double value =
            static_cast<double>( pixels[ index ] ) + peak * weight;
        pixels[ index ] = static_cast<std::uint8_t>(
            std::clamp( value, 0.0, 255.0 ) );
      }
    }
  }

  [[nodiscard]] inline sensor::Image makeBlankImage( int width, int height )
  {
    return sensor::Image{
        width, height, 1,
        std::vector<std::uint8_t>(
            static_cast<std::size_t>( width * height ), 0U ) };
  }

  /**
   * @brief Render a rectified stereo pair from points in the left camera frame.
   */
  [[nodiscard]] inline sensor::StereoFrame renderStereo(
      const camera::RectifiedStereoCalibration& calibration,
      const std::vector<Eigen::Vector3d>&       points_left,
      common::Timestamp                         timestamp,
      double                                    sigma_px = 2.0 )
  {
    const int width  = calibration.imageWidth();
    const int height = calibration.imageHeight();
    std::vector<std::uint8_t> left_pixels(
        static_cast<std::size_t>( width * height ), 0U );
    std::vector<std::uint8_t> right_pixels(
        static_cast<std::size_t>( width * height ), 0U );

    for ( const Eigen::Vector3d& point : points_left )
    {
      if ( !( point.z() > 0.0 ) || !point.allFinite() )
      {
        continue;
      }
      const Eigen::Vector2d left_uv  = projectLeft( calibration, point );
      const Eigen::Vector2d right_uv = projectRight( calibration, point );
      paintGaussianBlob( left_pixels, width, height, left_uv.x(), left_uv.y(),
                         sigma_px );
      paintGaussianBlob( right_pixels, width, height, right_uv.x(),
                         right_uv.y(), sigma_px );
    }

    return sensor::StereoFrame{
        timestamp,
        sensor::Image{ width, height, 1, std::move( left_pixels ) },
        sensor::Image{ width, height, 1, std::move( right_pixels ) } };
  }

  /**
   * @brief Grid of points in front of the camera, spaced for GFTT separation.
   */
  [[nodiscard]] inline std::vector<Eigen::Vector3d> makePointGrid(
      const camera::RectifiedStereoCalibration& calibration, int rows = 4,
      int cols = 5, double depth_m = 3.0 )
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve( static_cast<std::size_t>( rows * cols ) );
    const double margin_u = 40.0;
    const double margin_v = 40.0;
    for ( int row = 0; row < rows; ++row )
    {
      for ( int col = 0; col < cols; ++col )
      {
        const double u =
            margin_u +
            ( static_cast<double>( calibration.imageWidth() ) - 2.0 * margin_u ) *
                ( static_cast<double>( col ) + 0.5 ) /
                static_cast<double>( cols );
        const double v =
            margin_v +
            ( static_cast<double>( calibration.imageHeight() ) -
              2.0 * margin_v ) *
                ( static_cast<double>( row ) + 0.5 ) /
                static_cast<double>( rows );
        const double x =
            ( u - calibration.cxPixels() ) * depth_m / calibration.fxPixels();
        const double y =
            ( v - calibration.cyPixels() ) * depth_m / calibration.fyPixels();
        points.emplace_back( x, y, depth_m );
      }
    }
    return points;
  }

}  // namespace phad::testing

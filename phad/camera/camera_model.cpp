#include "phad/camera/camera_model.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <type_traits>
#include <utility>
#include <variant>

namespace phad::camera
{
  namespace
  {

    constexpr int    kMaximumIterations          = 50;
    constexpr double kSingularTolerance          = 1e-14;
    constexpr double kUnitBearingTolerance       = 1e-12;
    constexpr double kPixelReprojectionTolerance = 1e-8;
    constexpr double kHalfPi                     = 1.57079632679489661923;

    CameraModelError nonFiniteInputError()
    {
      return CameraModelError{
          CameraModelErrorCode::kNonFiniteInput,
          "camera geometry input must contain only finite values" };
    }

    CameraModelError outsideDomainError( const char* detail )
    {
      return CameraModelError{ CameraModelErrorCode::kOutsideModelDomain,
                               detail };
    }

    CameraModelError numericalFailureError( const char* detail )
    {
      return CameraModelError{ CameraModelErrorCode::kNumericalFailure,
                               detail };
    }

    bool hasAccurateReprojection( const CameraModel&     model,
                                  const Eigen::Vector3d& bearing,
                                  const Eigen::Vector2d& pixel )
    {
      if ( !bearing.allFinite() ||
           std::abs( bearing.norm() - 1.0 ) >
               kUnitBearingTolerance )
      {
        return false;
      }

      const auto reprojected = model.project( bearing );
      if ( !reprojected.hasValue() )
      {
        return false;
      }

      return std::hypot( reprojected.value().x() - pixel.x(),
                         reprojected.value().y() - pixel.y() ) <=
             kPixelReprojectionTolerance;
    }

    struct DistortedPoint
    {
      double x;
      double y;
    };

    class RadialTangentialCameraModel final : public CameraModel
    {
    public:
      explicit RadialTangentialCameraModel(
          sensor::PinholeRadialTangentialParameters parameters )
          : m_parameters( std::move( parameters ) )
      {
      }

      CameraModelResult<Eigen::Vector2d> project(
          const Eigen::Vector3d& point_camera ) const override
      {
        if ( !point_camera.allFinite() )
        {
          return nonFiniteInputError();
        }
        if ( point_camera.z() <= 0.0 )
        {
          return outsideDomainError(
              "project requires strictly positive camera-frame depth" );
        }

        const double x         = point_camera.x() / point_camera.z();
        const double y         = point_camera.y() / point_camera.z();
        const auto   distorted = distort( x, y );
        if ( !distorted.hasValue() )
        {
          return distorted.error();
        }

        const Eigen::Vector2d pixel(
            m_parameters.fxPixels() * distorted.value().x +
                m_parameters.cxPixels(),
            m_parameters.fyPixels() * distorted.value().y +
                m_parameters.cyPixels() );
        if ( !pixel.allFinite() )
        {
          return numericalFailureError(
              "radial-tangential projection produced a non-finite pixel" );
        }
        return pixel;
      }

      CameraModelResult<Eigen::Vector3d> backProject(
          const Eigen::Vector2d& pixel ) const override
      {
        if ( !pixel.allFinite() )
        {
          return nonFiniteInputError();
        }

        const double distorted_x =
            ( pixel.x() - m_parameters.cxPixels() ) /
            m_parameters.fxPixels();
        const double distorted_y =
            ( pixel.y() - m_parameters.cyPixels() ) /
            m_parameters.fyPixels();
        if ( !std::isfinite( distorted_x ) ||
             !std::isfinite( distorted_y ) )
        {
          return numericalFailureError(
              "radial-tangential normalization overflowed" );
        }

        double x = distorted_x;
        double y = distorted_y;
        for ( int iteration = 0; iteration < kMaximumIterations;
              ++iteration )
        {
          const auto distorted = distort( x, y );
          if ( !distorted.hasValue() )
          {
            return distorted.error();
          }

          const double scale =
              std::max( { std::abs( x ), std::abs( y ), 1.0 } );
          const Eigen::Vector3d bearing =
              Eigen::Vector3d( x / scale, y / scale, 1.0 / scale )
                  .normalized();
          if ( hasAccurateReprojection( *this, bearing, pixel ) )
          {
            return bearing;
          }

          const double residual_x = distorted.value().x - distorted_x;
          const double residual_y = distorted.value().y - distorted_y;

          const double radius_squared = x * x + y * y;
          const double radial_derivative_factor =
              2.0 *
              ( m_parameters.k1() +
                2.0 * m_parameters.k2() * radius_squared );
          const double radial =
              1.0 +
              radius_squared *
                  ( m_parameters.k1() +
                    m_parameters.k2() * radius_squared );
          const double radial_x = radial_derivative_factor * x;
          const double radial_y = radial_derivative_factor * y;
          const double j00 =
              radial + x * radial_x + 2.0 * m_parameters.p1() * y +
              6.0 * m_parameters.p2() * x;
          const double j01 =
              x * radial_y + 2.0 * m_parameters.p1() * x +
              2.0 * m_parameters.p2() * y;
          const double j10 =
              y * radial_x + 2.0 * m_parameters.p2() * y +
              2.0 * m_parameters.p1() * x;
          const double j11 =
              radial + y * radial_y + 6.0 * m_parameters.p1() * y +
              2.0 * m_parameters.p2() * x;
          const double determinant = j00 * j11 - j01 * j10;
          if ( !std::isfinite( determinant ) )
          {
            return numericalFailureError(
                "radial-tangential inverse Jacobian is non-finite" );
          }
          if ( std::abs( determinant ) <= kSingularTolerance )
          {
            return outsideDomainError(
                "pixel is outside the invertible radial-tangential domain" );
          }

          const double step_x =
              ( j11 * residual_x - j01 * residual_y ) / determinant;
          const double step_y =
              ( -j10 * residual_x + j00 * residual_y ) / determinant;
          x -= step_x;
          y -= step_y;
          if ( !std::isfinite( x ) || !std::isfinite( y ) )
          {
            return numericalFailureError(
                "radial-tangential inverse iteration diverged" );
          }
        }

        return numericalFailureError(
            "radial-tangential inverse iteration did not converge" );
      }

    private:
      CameraModelResult<DistortedPoint> distort( double x, double y ) const
      {
        if ( !std::isfinite( x ) || !std::isfinite( y ) )
        {
          return numericalFailureError(
              "radial-tangential normalized point is non-finite" );
        }

        const double radius_squared = x * x + y * y;
        const double radial =
            1.0 +
            radius_squared *
                ( m_parameters.k1() +
                  m_parameters.k2() * radius_squared );
        const DistortedPoint distorted{
            x * radial + 2.0 * m_parameters.p1() * x * y +
                m_parameters.p2() *
                    ( radius_squared + 2.0 * x * x ),
            y * radial +
                m_parameters.p1() *
                    ( radius_squared + 2.0 * y * y ) +
                2.0 * m_parameters.p2() * x * y };
        if ( !std::isfinite( distorted.x ) ||
             !std::isfinite( distorted.y ) )
        {
          return numericalFailureError(
              "radial-tangential distortion overflowed" );
        }
        return distorted;
      }

      sensor::PinholeRadialTangentialParameters m_parameters;
    };

    class EquidistantCameraModel final : public CameraModel
    {
    public:
      explicit EquidistantCameraModel(
          sensor::PinholeEquidistantParameters parameters )
          : m_parameters( std::move( parameters ) )
      {
      }

      CameraModelResult<Eigen::Vector2d> project(
          const Eigen::Vector3d& point_camera ) const override
      {
        if ( !point_camera.allFinite() )
        {
          return nonFiniteInputError();
        }
        if ( point_camera.z() <= 0.0 )
        {
          return outsideDomainError(
              "project requires strictly positive camera-frame depth" );
        }

        const double x      = point_camera.x() / point_camera.z();
        const double y      = point_camera.y() / point_camera.z();
        const double radius = std::hypot( x, y );
        if ( !std::isfinite( radius ) )
        {
          return numericalFailureError(
              "equidistant normalized point overflowed" );
        }

        double distorted_x = 0.0;
        double distorted_y = 0.0;
        if ( radius > 0.0 )
        {
          const double theta           = std::atan( radius );
          const double distorted_theta = distortTheta( theta );
          if ( !std::isfinite( distorted_theta ) )
          {
            return numericalFailureError(
                "equidistant angular distortion overflowed" );
          }
          const double scale = distorted_theta / radius;
          distorted_x        = scale * x;
          distorted_y        = scale * y;
        }

        const Eigen::Vector2d pixel(
            m_parameters.fxPixels() * distorted_x +
                m_parameters.cxPixels(),
            m_parameters.fyPixels() * distorted_y +
                m_parameters.cyPixels() );
        if ( !pixel.allFinite() )
        {
          return numericalFailureError(
              "equidistant projection produced a non-finite pixel" );
        }
        return pixel;
      }

      CameraModelResult<Eigen::Vector3d> backProject(
          const Eigen::Vector2d& pixel ) const override
      {
        if ( !pixel.allFinite() )
        {
          return nonFiniteInputError();
        }

        const double distorted_x =
            ( pixel.x() - m_parameters.cxPixels() ) /
            m_parameters.fxPixels();
        const double distorted_y =
            ( pixel.y() - m_parameters.cyPixels() ) /
            m_parameters.fyPixels();
        const double distorted_radius =
            std::hypot( distorted_x, distorted_y );
        if ( !std::isfinite( distorted_radius ) )
        {
          return numericalFailureError(
              "equidistant pixel normalization overflowed" );
        }
        if ( distorted_radius == 0.0 )
        {
          return Eigen::Vector3d( 0.0, 0.0, 1.0 );
        }

        double theta = std::min( distorted_radius, kHalfPi * 0.5 );
        for ( int iteration = 0; iteration < kMaximumIterations;
              ++iteration )
        {
          const double residual =
              distortTheta( theta ) - distorted_radius;
          if ( !std::isfinite( residual ) )
          {
            return numericalFailureError(
                "equidistant inverse residual is non-finite" );
          }

          const double          direction_x = distorted_x / distorted_radius;
          const double          direction_y = distorted_y / distorted_radius;
          const double          sine_theta  = std::sin( theta );
          const Eigen::Vector3d bearing(
              sine_theta * direction_x, sine_theta * direction_y,
              std::cos( theta ) );
          if ( hasAccurateReprojection( *this, bearing, pixel ) )
          {
            return bearing;
          }

          const double derivative = distortThetaDerivative( theta );
          if ( !std::isfinite( derivative ) )
          {
            return numericalFailureError(
                "equidistant inverse derivative is non-finite" );
          }
          if ( std::abs( derivative ) <= kSingularTolerance )
          {
            return outsideDomainError(
                "pixel is outside the invertible equidistant domain" );
          }

          theta -= residual / derivative;
          if ( !std::isfinite( theta ) )
          {
            return numericalFailureError(
                "equidistant inverse iteration diverged" );
          }
          if ( theta < 0.0 || theta >= kHalfPi )
          {
            return outsideDomainError(
                "pixel is outside the positive-depth equidistant domain" );
          }
        }

        return numericalFailureError(
            "equidistant inverse iteration did not converge" );
      }

    private:
      double distortTheta( double theta ) const
      {
        const double theta_squared = theta * theta;
        return theta *
               ( 1.0 +
                 theta_squared *
                     ( m_parameters.k1() +
                       theta_squared *
                           ( m_parameters.k2() +
                             theta_squared *
                                 ( m_parameters.k3() +
                                   theta_squared * m_parameters.k4() ) ) ) );
      }

      double distortThetaDerivative( double theta ) const
      {
        const double theta_squared = theta * theta;
        return 1.0 +
               theta_squared *
                   ( 3.0 * m_parameters.k1() +
                     theta_squared *
                         ( 5.0 * m_parameters.k2() +
                           theta_squared *
                               ( 7.0 * m_parameters.k3() +
                                 theta_squared *
                                     9.0 * m_parameters.k4() ) ) );
      }

      sensor::PinholeEquidistantParameters m_parameters;
    };

  }  // namespace

  std::unique_ptr<CameraModel> createCameraModel(
      sensor::CameraModelParameters model_parameters )
  {
    return std::visit(
        []( auto parameters ) -> std::unique_ptr<CameraModel> {
          using Parameters = decltype( parameters );
          if constexpr ( std::is_same_v<
                             Parameters,
                             sensor::PinholeRadialTangentialParameters> )
          {
            return std::make_unique<RadialTangentialCameraModel>(
                std::move( parameters ) );
          }
          else
          {
            static_assert(
                std::is_same_v<
                    Parameters, sensor::PinholeEquidistantParameters> );
            return std::make_unique<EquidistantCameraModel>(
                std::move( parameters ) );
          }
        },
        std::move( model_parameters ) );
  }

}  // namespace phad::camera

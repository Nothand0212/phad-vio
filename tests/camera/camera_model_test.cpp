#include "phad/camera/camera_model.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

namespace
{

  using phad::camera::CameraModel;
  using phad::camera::CameraModelError;
  using phad::camera::CameraModelErrorCode;
  using phad::camera::CameraModelResult;
  using phad::camera::createCameraModel;
  using phad::sensor::PinholeEquidistantParameters;
  using phad::sensor::PinholeRadialTangentialParameters;

  static_assert( std::has_virtual_destructor_v<CameraModel> );
  static_assert( std::is_abstract_v<CameraModel> );
  static_assert( !std::is_copy_constructible_v<CameraModel> );
  static_assert( !std::is_copy_assignable_v<CameraModel> );
  static_assert( !std::is_move_constructible_v<CameraModel> );
  static_assert( !std::is_move_assignable_v<CameraModel> );
  static_assert( std::is_same_v<
                 decltype( createCameraModel(
                     std::declval<phad::sensor::CameraModelParameters>() ) ),
                 std::unique_ptr<CameraModel>> );

  template <typename T>
  void requireError( const CameraModelResult<T>& result,
                     CameraModelErrorCode        expected_code )
  {
    ASSERT_FALSE( result.hasValue() );
    EXPECT_FALSE( static_cast<bool>( result ) );
    const CameraModelError& error = result.error();
    EXPECT_EQ( error.code, expected_code );
    EXPECT_FALSE( error.detail.empty() );
  }

  std::unique_ptr<CameraModel> makeRadialTangentialModel()
  {
    auto parameters = PinholeRadialTangentialParameters::create(
        400.0, 420.0, 320.0, 240.0, 0.1, -0.02, 0.01, -0.005 );
    return createCameraModel( std::move( parameters ).value() );
  }

  std::unique_ptr<CameraModel> makeEquidistantModel()
  {
    auto parameters = PinholeEquidistantParameters::create(
        300.0, 310.0, 321.0, 239.0, 0.01, -0.001, 0.0001, -0.00001 );
    return createCameraModel( std::move( parameters ).value() );
  }

  double angularError( const Eigen::Vector3d& lhs,
                       const Eigen::Vector3d& rhs )
  {
    const Eigen::Vector3d cross(
        lhs.y() * rhs.z() - lhs.z() * rhs.y(),
        lhs.z() * rhs.x() - lhs.x() * rhs.z(),
        lhs.x() * rhs.y() - lhs.y() * rhs.x() );
    return std::atan2( cross.norm(), lhs.dot( rhs ) );
  }

  void expectAccurateBackProjection( CameraModel&           model,
                                     const Eigen::Vector2d& pixel )
  {
    auto bearing = model.backProject( pixel );
    ASSERT_TRUE( bearing );
    ASSERT_TRUE( bearing.value().allFinite() );
    EXPECT_NEAR( bearing.value().norm(), 1.0, 1e-12 );
    auto reprojected = model.project( bearing.value() );
    ASSERT_TRUE( reprojected );
    EXPECT_LE( std::hypot( reprojected.value().x() - pixel.x(),
                           reprojected.value().y() - pixel.y() ),
               1e-8 );
  }

  TEST( CameraModelResultTest, ExposesValueAndStructuredErrorBranches )
  {
    CameraModelResult<int> value( 7 );
    ASSERT_TRUE( value );
    EXPECT_EQ( value.value(), 7 );

    CameraModelResult<int> error( CameraModelError{
        CameraModelErrorCode::kOutsideModelDomain, "outside domain" } );
    requireError( error, CameraModelErrorCode::kOutsideModelDomain );
  }

  TEST( CameraModelTest, ProjectsOpticalAxisToPrincipalPoint )
  {
    const std::array<std::pair<std::unique_ptr<CameraModel>, Eigen::Vector2d>,
                     2>
        models{
            std::pair{ makeRadialTangentialModel(),
                       Eigen::Vector2d( 320.0, 240.0 ) },
            std::pair{ makeEquidistantModel(),
                       Eigen::Vector2d( 321.0, 239.0 ) } };

    for ( const auto& [ model, principal_point ] : models )
    {
      auto projected = model->project( Eigen::Vector3d( 0.0, 0.0, 2.0 ) );
      ASSERT_TRUE( projected );
      EXPECT_NEAR( projected.value().x(), principal_point.x(), 1e-12 );
      EXPECT_NEAR( projected.value().y(), principal_point.y(), 1e-12 );

      auto bearing = model->backProject( principal_point );
      ASSERT_TRUE( bearing );
      EXPECT_NEAR( bearing.value().x(), 0.0, 1e-12 );
      EXPECT_NEAR( bearing.value().y(), 0.0, 1e-12 );
      EXPECT_NEAR( bearing.value().z(), 1.0, 1e-12 );
      EXPECT_NEAR( bearing.value().norm(), 1.0, 1e-12 );
    }
  }

  TEST( CameraModelTest, AppliesRadialTangentialDistortionGolden )
  {
    auto projected = makeRadialTangentialModel()->project(
        Eigen::Vector3d( 0.2, -0.1, 1.0 ) );

    ASSERT_TRUE( projected );
    EXPECT_NEAR( projected.value().x(), 399.976, 1e-12 );
    EXPECT_NEAR( projected.value().y(), 198.1701, 1e-12 );
  }

  TEST( CameraModelTest, AppliesEquidistantDistortionGolden )
  {
    auto projected = makeEquidistantModel()->project(
        Eigen::Vector3d( 0.3, -0.2, 1.0 ) );

    ASSERT_TRUE( projected );
    EXPECT_NEAR( projected.value().x(), 407.4807482047052, 1e-12 );
    EXPECT_NEAR( projected.value().y(), 179.42437345898085, 1e-12 );
  }

  TEST( CameraModelTest, ProjectionIsInvariantAcrossPositiveScales )
  {
    auto radial      = makeRadialTangentialModel();
    auto equidistant = makeEquidistantModel();
    for ( const CameraModel* model : { radial.get(), equidistant.get() } )
    {
      auto reference =
          model->project( Eigen::Vector3d( 0.2, -0.1, 1.0 ) );
      ASSERT_TRUE( reference );
      for ( const double scale : { 1e-9, 1e-3, 1.0, 1e3, 1e9 } )
      {
        auto scaled =
            model->project( scale * Eigen::Vector3d( 0.2, -0.1, 1.0 ) );
        ASSERT_TRUE( scaled );
        EXPECT_NEAR( scaled.value().x(), reference.value().x(), 1e-12 );
        EXPECT_NEAR( scaled.value().y(), reference.value().y(), 1e-12 );
      }
    }
  }

  TEST( CameraModelTest, RoundTripsPointsAndPixelsForBothModels )
  {
    auto radial      = makeRadialTangentialModel();
    auto equidistant = makeEquidistantModel();
    for ( const CameraModel* model : { radial.get(), equidistant.get() } )
    {
      const Eigen::Vector3d point( 0.3, -0.2, 1.4 );
      auto                  pixel = model->project( point );
      ASSERT_TRUE( pixel );
      auto bearing = model->backProject( pixel.value() );
      ASSERT_TRUE( bearing );
      EXPECT_LE( angularError( bearing.value(), point.normalized() ), 1e-9 );
      EXPECT_NEAR( bearing.value().norm(), 1.0, 1e-12 );

      const Eigen::Vector2d outside_image_pixel( -20.0, 300.0 );
      auto                  outside_bearing = model->backProject( outside_image_pixel );
      ASSERT_TRUE( outside_bearing );
      auto reprojected = model->project( outside_bearing.value() );
      ASSERT_TRUE( reprojected );
      EXPECT_LE(
          ( reprojected.value() - outside_image_pixel ).norm(), 1e-8 );
    }
  }

  TEST( CameraModelTest,
        RadialBackProjectionDoesNotAcceptPixelAmplifiedResidual )
  {
    auto parameters = PinholeRadialTangentialParameters::create(
        1e308, 1.0, 0.0, 0.0, 1e-15, 0.0, 0.0, 0.0 );
    auto radial = createCameraModel( std::move( parameters ).value() );

    expectAccurateBackProjection( *radial,
                                  Eigen::Vector2d( 1e308, 0.0 ) );
  }

  TEST( CameraModelTest,
        EquidistantBackProjectionDoesNotAcceptPixelAmplifiedResidual )
  {
    auto parameters = PinholeEquidistantParameters::create(
        1e308, 1.0, 0.0, 0.0, 1e-12, 0.0, 0.0, 0.0 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );

    requireError(
        equidistant->backProject( Eigen::Vector2d( 1e307, 0.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, RejectsNonFiniteInputsForBothModels )
  {
    auto         radial      = makeRadialTangentialModel();
    auto         equidistant = makeEquidistantModel();
    const double nan         = std::numeric_limits<double>::quiet_NaN();
    const double inf         = std::numeric_limits<double>::infinity();
    for ( const CameraModel* model : { radial.get(), equidistant.get() } )
    {
      for ( const Eigen::Vector3d& point :
            { Eigen::Vector3d( nan, 0.0, 1.0 ),
              Eigen::Vector3d( 0.0, inf, 1.0 ),
              Eigen::Vector3d( 0.0, 0.0, -inf ) } )
      {
        requireError( model->project( point ),
                      CameraModelErrorCode::kNonFiniteInput );
      }
      for ( const Eigen::Vector2d& pixel :
            { Eigen::Vector2d( nan, 0.0 ),
              Eigen::Vector2d( 0.0, inf ) } )
      {
        requireError( model->backProject( pixel ),
                      CameraModelErrorCode::kNonFiniteInput );
      }
    }
  }

  TEST( CameraModelTest, RejectsNonPositiveDepthOutsideModelDomain )
  {
    auto radial      = makeRadialTangentialModel();
    auto equidistant = makeEquidistantModel();
    for ( const CameraModel* model : { radial.get(), equidistant.get() } )
    {
      requireError( model->project( Eigen::Vector3d( 1.0, 2.0, 0.0 ) ),
                    CameraModelErrorCode::kOutsideModelDomain );
      requireError( model->project( Eigen::Vector3d( 1.0, 2.0, -1.0 ) ),
                    CameraModelErrorCode::kOutsideModelDomain );
    }
  }

  TEST( CameraModelTest, ReportsNonInvertibleBackProjectionDomain )
  {
    auto radial_parameters = PinholeRadialTangentialParameters::create(
        1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0 );
    auto radial =
        createCameraModel( std::move( radial_parameters ).value() );
    requireError(
        radial->backProject(
            Eigen::Vector2d( 1.0 / std::sqrt( 3.0 ), 0.0 ) ),
        CameraModelErrorCode::kOutsideModelDomain );

    auto equidistant_parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0 );
    auto equidistant =
        createCameraModel( std::move( equidistant_parameters ).value() );
    requireError( equidistant->backProject( Eigen::Vector2d( 1.0, 0.0 ) ),
                  CameraModelErrorCode::kOutsideModelDomain );
  }

  TEST( CameraModelTest,
        ReportsRadialProjectionNormalizedCoordinateOverflow )
  {
    const double maximum = std::numeric_limits<double>::max();
    auto         radial  = makeRadialTangentialModel();
    requireError(
        radial->project( Eigen::Vector3d(
            maximum, maximum, std::numeric_limits<double>::min() ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialProjectionDistortionOverflow )
  {
    auto radial = makeRadialTangentialModel();
    requireError(
        radial->project( Eigen::Vector3d( 1e200, 0.0, 1.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialProjectionPixelAssemblyOverflow )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeRadialTangentialParameters::create(
        maximum, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
    auto radial = createCameraModel( std::move( parameters ).value() );
    requireError( radial->project( Eigen::Vector3d( 2.0, 0.0, 1.0 ) ),
                  CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialBackProjectionNormalizationOverflow )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeRadialTangentialParameters::create(
        1.0, 1.0, maximum, 0.0, 0.0, 0.0, 0.0, 0.0 );
    auto radial = createCameraModel( std::move( parameters ).value() );
    requireError( radial->backProject( Eigen::Vector2d( -maximum, 0.0 ) ),
                  CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialBackProjectionDistortionOverflow )
  {
    const double maximum = std::numeric_limits<double>::max();
    auto         radial  = makeRadialTangentialModel();
    requireError( radial->backProject( Eigen::Vector2d( maximum, maximum ) ),
                  CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialBackProjectionNonFiniteJacobian )
  {
    auto parameters = PinholeRadialTangentialParameters::create(
        1.0, 1.0, 0.0, 0.0, -1e200, -0.1, 100.0, -0.1 );
    auto radial = createCameraModel( std::move( parameters ).value() );
    requireError( radial->backProject( Eigen::Vector2d( -10.0, -100.0 ) ),
                  CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialBackProjectionIterationDivergence )
  {
    auto parameters = PinholeRadialTangentialParameters::create(
        1.0, 1.0, 0.0, 0.0, 0.0, 0.0, -10.0, -1e10 );
    auto radial = createCameraModel( std::move( parameters ).value() );
    requireError(
        radial->backProject( Eigen::Vector2d( 1e100, 0.3 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsRadialBackProjectionNonConvergence )
  {
    auto parameters = PinholeRadialTangentialParameters::create(
        1.0, 1.0, 0.0, 0.0, -10.0, 0.3, 0.0, -1e100 );
    auto radial = createCameraModel( std::move( parameters ).value() );
    requireError( radial->backProject( Eigen::Vector2d( 3.0, 10.0 ) ),
                  CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest,
        ReportsEquidistantProjectionNormalizedCoordinateOverflow )
  {
    const double maximum     = std::numeric_limits<double>::max();
    auto         equidistant = makeEquidistantModel();
    requireError(
        equidistant->project( Eigen::Vector3d(
            maximum, maximum, std::numeric_limits<double>::min() ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantProjectionAngularOverflow )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, maximum, 0.0, 0.0, 0.0 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->project( Eigen::Vector3d( maximum, 0.0, 1.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantProjectionPixelAssemblyOverflow )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeEquidistantParameters::create(
        maximum, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->project( Eigen::Vector3d( 2.0, 0.0, 1.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantBackProjectionNormalizationOverflow )
  {
    const double maximum                = std::numeric_limits<double>::max();
    auto         equidistant_parameters = PinholeEquidistantParameters::create(
        300.0, 310.0, maximum, 239.0, 0.01, -0.001, 0.0001,
        -0.00001 );
    auto equidistant =
        createCameraModel( std::move( equidistant_parameters ).value() );
    requireError(
        equidistant->backProject( Eigen::Vector2d( -maximum, 239.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantBackProjectionNonFiniteResidual )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, 100.0, -maximum, 0.0, -1.0 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->backProject( Eigen::Vector2d( -maximum, 1.0 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantBackProjectionNonFiniteDerivative )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, 0.0, 1e100, -1.0, -maximum );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->backProject( Eigen::Vector2d( 100.0, -0.1 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantBackProjectionIterationDivergence )
  {
    const double maximum    = std::numeric_limits<double>::max();
    auto         parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, -0.3, 0.0, -0.3, 0.3 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->backProject( Eigen::Vector2d( 0.0, maximum ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

  TEST( CameraModelTest, ReportsEquidistantBackProjectionNonConvergence )
  {
    auto parameters = PinholeEquidistantParameters::create(
        1.0, 1.0, 0.0, 0.0, 0.0, -100.0, 100.0, 1e300 );
    auto equidistant =
        createCameraModel( std::move( parameters ).value() );
    requireError(
        equidistant->backProject( Eigen::Vector2d( -100.0, -0.1 ) ),
        CameraModelErrorCode::kNumericalFailure );
  }

}  // namespace

#include "phad/sensor/rigid_transform.hpp"

#include <Eigen/LU>
#include <cmath>
#include <utility>

namespace phad::sensor
{

  CalibrationResult<RigidTransform> RigidTransform::create(
      Eigen::Matrix4d matrix )
  {
    if ( !matrix.allFinite() )
    {
      return CalibrationError{ CalibrationErrorCode::kNonFiniteValue,
                               "rigid_transform.matrix",
                               "all matrix elements must be finite" };
    }

    const Eigen::Matrix3d rotation = matrix.block<3, 3>( 0, 0 );
    const double          orthogonality_error =
        ( rotation.transpose() * rotation - Eigen::Matrix3d::Identity() )
            .norm();
    const double determinant_error =
        std::abs( rotation.determinant() - 1.0 );
    if ( orthogonality_error > kValidationTolerance ||
         determinant_error > kValidationTolerance )
    {
      return CalibrationError{
          CalibrationErrorCode::kInvalidRotation,
          "rigid_transform.rotation",
          "rotation must be orthogonal with determinant +1" };
    }

    const Eigen::RowVector4d expected_bottom_row( 0.0, 0.0, 0.0, 1.0 );
    if ( ( matrix.row( 3 ) - expected_bottom_row ).norm() >
         kValidationTolerance )
    {
      return CalibrationError{
          CalibrationErrorCode::kInvalidHomogeneousRow,
          "rigid_transform.bottom_row",
          "bottom row must be [0, 0, 0, 1]" };
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear()          = rotation;
    transform.translation()     = matrix.block<3, 1>( 0, 3 );
    return RigidTransform( std::move( transform ) );
  }

  Eigen::Matrix3d RigidTransform::rotation() const
  {
    return m_transform.rotation();
  }

  Eigen::Vector3d RigidTransform::translation() const
  {
    return m_transform.translation();
  }

  RigidTransform::RigidTransform( Eigen::Isometry3d transform )
      : m_transform( std::move( transform ) )
  {
  }

}  // namespace phad::sensor

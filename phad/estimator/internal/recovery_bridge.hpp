#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <variant>

#include "phad/estimator/types.hpp"

namespace phad::estimator::internal
{
  inline constexpr double kRecoveryYawDefinitionLimit =
      64.0 * std::numeric_limits<double>::epsilon();

  [[nodiscard]] inline std::variant<double, RecoveryBridgeReason> recoveryYaw(
      const double a, const double b )
  {
    if ( !std::isfinite( a ) || !std::isfinite( b ) )
    {
      return RecoveryBridgeReason::kInvalidTransform;
    }
    if ( std::hypot( a, b ) <= kRecoveryYawDefinitionLimit )
    {
      return RecoveryBridgeReason::kYawUndefined;
    }
    return std::atan2( b, a );
  }

  [[nodiscard]] inline bool isRecoveryPoseValid( const Eigen::Isometry3d& pose )
  {
    constexpr double kRotationTolerance = 1e-9;
    return pose.matrix().allFinite() &&
           ( pose.rotation().transpose() * pose.rotation() ).isApprox( Eigen::Matrix3d::Identity(), kRotationTolerance ) &&
           std::abs( pose.rotation().determinant() - 1.0 ) <= kRotationTolerance;
  }

  [[nodiscard]] inline std::variant<Eigen::Isometry3d, RecoveryBridgeReason>
  makeRecoveryBridgeProposal( const Eigen::Isometry3d& anchor,
                              const Eigen::Isometry3d& local )
  {
    if ( !isRecoveryPoseValid( anchor ) )
    {
      return RecoveryBridgeReason::kInvalidAnchor;
    }
    if ( !isRecoveryPoseValid( local ) )
    {
      return RecoveryBridgeReason::kInvalidTransform;
    }
    const Eigen::Matrix3d delta = anchor.rotation() * local.rotation().transpose();
    const auto            yaw   = recoveryYaw( delta( 0, 0 ) + delta( 1, 1 ),
                                               delta( 1, 0 ) - delta( 0, 1 ) );
    if ( const auto* reason = std::get_if<RecoveryBridgeReason>( &yaw ) )
    {
      return *reason;
    }
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear()          = Eigen::AngleAxisd( std::get<double>( yaw ),
                                                     Eigen::Vector3d::UnitZ() )
                             .toRotationMatrix();
    transform.translation() = anchor.translation() - transform.rotation() * local.translation();
    if ( !isRecoveryPoseValid( transform ) )
    {
      return RecoveryBridgeReason::kInvalidTransform;
    }
    return transform;
  }
}  // namespace phad::estimator::internal

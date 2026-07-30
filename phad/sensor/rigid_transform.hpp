#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "phad/sensor/calibration_error.hpp"

namespace phad::sensor
{

  class RigidTransform
  {
  public:
    static constexpr double kValidationTolerance = 1e-6;

    [[nodiscard]] static CalibrationResult<RigidTransform> create(
        Eigen::Matrix4d matrix );

    [[nodiscard]] Eigen::Matrix3d rotation() const;
    [[nodiscard]] Eigen::Vector3d translation() const;

  private:
    explicit RigidTransform( Eigen::Isometry3d transform );

    Eigen::Isometry3d m_transform;
  };

}  // namespace phad::sensor

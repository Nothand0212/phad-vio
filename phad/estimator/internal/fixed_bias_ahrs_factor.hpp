#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace phad::estimator::internal
{

  class FixedBiasAhrsFactor final
      : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base =
        gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  public:
    using Base::evaluateError;

    FixedBiasAhrsFactor(
        gtsam::Key key_i, gtsam::Key key_j,
        const gtsam::PreintegratedAhrsMeasurements& pim,
        const gtsam::Vector3& bias, const gtsam::Matrix3& align_cov );

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

    [[nodiscard]] gtsam::Vector evaluateError(
        const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
        gtsam::OptionalMatrixType H_i,
        gtsam::OptionalMatrixType H_j ) const override;

  private:
    gtsam::PreintegratedAhrsMeasurements m_pim;
    gtsam::Vector3                       m_bias;
  };

}  // namespace phad::estimator::internal

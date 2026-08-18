#pragma once

#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>

#include "phad/estimator/internal/gyro_interval_reducer.hpp"

namespace phad::estimator::internal
{

  [[nodiscard]] gtsam::PreintegratedAhrsMeasurements
  preintegrateGyroInterval( const ValidatedGyroInterval& interval,
                            const Eigen::Vector3d& bias_hat, double gyr_nd );

  class GyroRotationFactor final
      : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                        gtsam::Vector3>
  {
  public:
    using Base =
        gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>;
    using Base::evaluateError;

    GyroRotationFactor(
        gtsam::Key pose_i_key, gtsam::Key pose_j_key, gtsam::Key bias_i_key,
        const gtsam::PreintegratedAhrsMeasurements& pim );

    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override;

    [[nodiscard]] gtsam::Vector evaluateError(
        const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
        const gtsam::Vector3& bias_i, gtsam::OptionalMatrixType h_i,
        gtsam::OptionalMatrixType h_j,
        gtsam::OptionalMatrixType h_g ) const override;

    [[nodiscard]] const gtsam::PreintegratedAhrsMeasurements&
    preintegratedMeasurements() const noexcept;

  private:
    gtsam::AHRSFactor m_ahrs_factor;
  };

}  // namespace phad::estimator::internal

#include "phad/estimator/internal/gyro_rotation_factor.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace phad::estimator::internal
{

  gtsam::PreintegratedAhrsMeasurements preintegrateGyroInterval(
      const ValidatedGyroInterval& interval, const Eigen::Vector3d& bias_hat,
      double gyr_nd )
  {
    const double gyro_variance = gyr_nd * gyr_nd;
    if ( interval.m_steps.empty() || interval.m_duration_ns <= 0 ||
         !bias_hat.allFinite() || !std::isfinite( gyro_variance ) ||
         gyro_variance <= 0.0 )
    {
      throw std::invalid_argument(
          "gyro preintegration requires finite bias, samples, and noise" );
    }

    auto params =
        std::make_shared<gtsam::PreintegratedAhrsMeasurements::Params>();
    params->setGyroscopeCovariance(
        gyro_variance * Eigen::Matrix3d::Identity() );
    gtsam::PreintegratedAhrsMeasurements pim( params, bias_hat );
    for ( const GyroIntervalStep& step : interval.m_steps )
    {
      pim.integrateMeasurement( step.m_omega_mean_radps, step.m_dt_s );
    }
    return pim;
  }

  GyroRotationFactor::GyroRotationFactor(
      gtsam::Key pose_i_key, gtsam::Key pose_j_key, gtsam::Key bias_i_key,
      const gtsam::PreintegratedAhrsMeasurements& pim )
      : Base( gtsam::noiseModel::Gaussian::Covariance(
                  pim.preintMeasCov() ),
              pose_i_key, pose_j_key, bias_i_key ),
        m_ahrs_factor( pose_i_key, pose_j_key, bias_i_key, pim )
  {
  }

  gtsam::NonlinearFactor::shared_ptr GyroRotationFactor::clone() const
  {
    return std::make_shared<GyroRotationFactor>( *this );
  }

  gtsam::Vector GyroRotationFactor::evaluateError(
      const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
      const gtsam::Vector3& bias_i, gtsam::OptionalMatrixType h_i,
      gtsam::OptionalMatrixType h_j, gtsam::OptionalMatrixType h_g ) const
  {
    gtsam::Matrix36    d_rotation_i_pose_i;
    gtsam::Matrix36    d_rotation_j_pose_j;
    const gtsam::Rot3& rotation_i = pose_i.rotation(
        h_i != nullptr ? &d_rotation_i_pose_i : nullptr );
    const gtsam::Rot3& rotation_j = pose_j.rotation(
        h_j != nullptr ? &d_rotation_j_pose_j : nullptr );

    gtsam::Matrix       h_rotation_i;
    gtsam::Matrix       h_rotation_j;
    const gtsam::Vector residual = m_ahrs_factor.evaluateError(
        rotation_i, rotation_j, bias_i,
        h_i != nullptr ? &h_rotation_i : nullptr,
        h_j != nullptr ? &h_rotation_j : nullptr, h_g );
    if ( h_i != nullptr )
    {
      *h_i = h_rotation_i * d_rotation_i_pose_i;
    }
    if ( h_j != nullptr )
    {
      *h_j = h_rotation_j * d_rotation_j_pose_j;
    }
    return residual;
  }

  const gtsam::PreintegratedAhrsMeasurements&
  GyroRotationFactor::preintegratedMeasurements() const noexcept
  {
    return m_ahrs_factor.preintegratedMeasurements();
  }

}  // namespace phad::estimator::internal

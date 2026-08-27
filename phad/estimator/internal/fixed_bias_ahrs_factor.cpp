#include "phad/estimator/internal/fixed_bias_ahrs_factor.hpp"

#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <stdexcept>

namespace phad::estimator::internal
{
  namespace
  {

    [[nodiscard]] gtsam::SharedNoiseModel makeNoise(
        const gtsam::PreintegratedAhrsMeasurements& pim,
        const gtsam::Matrix3&                       align_cov )
    {
      const gtsam::Matrix3 covariance =
          pim.preintMeasCov() + align_cov;
      if ( !covariance.allFinite() )
      {
        throw std::invalid_argument(
            "fixed-bias AHRS covariance must be finite" );
      }
      return gtsam::noiseModel::Gaussian::Covariance( covariance );
    }

  }  // namespace

  FixedBiasAhrsFactor::FixedBiasAhrsFactor(
      const gtsam::Key key_i, const gtsam::Key key_j,
      const gtsam::PreintegratedAhrsMeasurements& pim,
      const gtsam::Vector3& bias, const gtsam::Matrix3& align_cov )
      : Base( makeNoise( pim, align_cov ), key_i, key_j ), m_pim( pim ), m_bias( bias )
  {
  }

  gtsam::NonlinearFactor::shared_ptr FixedBiasAhrsFactor::clone() const
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        std::make_shared<FixedBiasAhrsFactor>( *this ) );
  }

  gtsam::Vector FixedBiasAhrsFactor::evaluateError(
      const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
      gtsam::OptionalMatrixType H_i,
      gtsam::OptionalMatrixType H_j ) const
  {
    const gtsam::AHRSFactor factor( 0, 1, 2, m_pim );
    gtsam::Matrix           H_R_i;
    gtsam::Matrix           H_R_j;
    const gtsam::Vector     residual = factor.evaluateError(
        pose_i.rotation(), pose_j.rotation(), m_bias,
        H_i == nullptr ? nullptr : &H_R_i,
        H_j == nullptr ? nullptr : &H_R_j, nullptr );

    if ( H_i != nullptr )
    {
      *H_i               = gtsam::Matrix::Zero( 3, 6 );
      H_i->leftCols<3>() = H_R_i;
    }
    if ( H_j != nullptr )
    {
      *H_j               = gtsam::Matrix::Zero( 3, 6 );
      H_j->leftCols<3>() = H_R_j;
    }
    return residual;
  }

}  // namespace phad::estimator::internal

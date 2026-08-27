#include "phad/estimator/internal/fixed_bias_ahrs_factor.hpp"

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>
#include <functional>

namespace
{

  using phad::estimator::internal::FixedBiasAhrsFactor;

  [[nodiscard]] gtsam::PreintegratedAhrsMeasurements preintegrate(
      const gtsam::Vector3& measured_omega, const double dt_s,
      const double gyr_nd = 0.002 )
  {
    auto params =
        std::make_shared<gtsam::PreintegratedRotationParams>();
    params->gyroscopeCovariance =
        gtsam::Matrix3::Identity() * gyr_nd * gyr_nd;
    gtsam::PreintegratedAhrsMeasurements pim( params,
                                              gtsam::Vector3::Zero() );
    pim.integrateMeasurement( measured_omega, dt_s );
    return pim;
  }

  [[nodiscard]] gtsam::Pose3 poseWithRotation( const gtsam::Rot3& R )
  {
    return gtsam::Pose3( R, gtsam::Point3( 1.0, -2.0, 0.5 ) );
  }

}  // namespace

TEST( FixedBiasAhrsFactor, ZeroRotationHasZeroResidual )
{
  const auto                pim = preintegrate( gtsam::Vector3::Zero(), 0.1 );
  const FixedBiasAhrsFactor factor(
      1, 2, pim, gtsam::Vector3::Zero(), gtsam::Matrix3::Zero() );

  const gtsam::Pose3 pose = poseWithRotation( gtsam::Rot3() );
  EXPECT_TRUE( factor.evaluateError( pose, pose ).isZero( 1e-12 ) );
}

TEST( FixedBiasAhrsFactor, CorrectsKnownBiasForMultipleDurations )
{
  const gtsam::Vector3 bias( 0.03, -0.02, 0.01 );
  const gtsam::Vector3 omega( 0.2, -0.1, 0.15 );
  const gtsam::Pose3   pose_i = poseWithRotation(
      gtsam::Rot3::RzRyRx( 0.1, -0.2, 0.3 ) );

  for ( const double dt_s : std::array<double, 3>{ 0.05, 0.1, 0.25 } )
  {
    SCOPED_TRACE( dt_s );
    const auto                pim = preintegrate( omega + bias, dt_s );
    const FixedBiasAhrsFactor factor(
        1, 2, pim, bias, gtsam::Matrix3::Identity() * 1e-7 );
    const gtsam::Rot3 R_j =
        pose_i.rotation().compose( gtsam::Rot3::Expmap( omega * dt_s ) );
    const gtsam::Pose3 pose_j(
        R_j, gtsam::Point3( -4.0, 3.0, 2.0 ) );

    // The GTSAM AHRS bias correction is first-order when the PIM biasHat is
    // zero, so the residual grows slightly with dt but remains near zero.
    EXPECT_LT( factor.evaluateError( pose_i, pose_j ).norm(), 1e-6 );
  }
}

TEST( FixedBiasAhrsFactor, RejectsReverseRotationDirection )
{
  const gtsam::Vector3      omega( 0.3, -0.2, 0.1 );
  constexpr double          kDtS = 0.2;
  const auto                pim  = preintegrate( omega, kDtS );
  const FixedBiasAhrsFactor factor(
      1, 2, pim, gtsam::Vector3::Zero(), gtsam::Matrix3::Zero() );
  const gtsam::Pose3 pose_i = poseWithRotation( gtsam::Rot3() );
  const gtsam::Pose3 pose_j(
      gtsam::Rot3::Expmap( -omega * kDtS ), gtsam::Point3::Zero() );

  EXPECT_GT( factor.evaluateError( pose_i, pose_j ).norm(), 0.1 );
}

TEST( FixedBiasAhrsFactor, AnalyticPoseJacobiansMatchNumericalDerivatives )
{
  const gtsam::Vector3      bias( -0.01, 0.02, 0.015 );
  const auto                pim = preintegrate( gtsam::Vector3( 0.21, -0.08, 0.13 ) + bias,
                                                0.17 );
  const FixedBiasAhrsFactor factor(
      1, 2, pim, bias, gtsam::Matrix3::Identity() * 2e-6 );
  const gtsam::Pose3 pose_i(
      gtsam::Rot3::RzRyRx( -0.12, 0.07, 0.21 ),
      gtsam::Point3( 0.5, -0.1, 0.2 ) );
  const gtsam::Pose3 pose_j(
      pose_i.rotation().compose(
          gtsam::Rot3::Expmap( gtsam::Vector3( 0.2, -0.1, 0.12 ) *
                               0.17 ) ),
      gtsam::Point3( -0.4, 0.7, 1.1 ) );

  gtsam::Matrix H_i;
  gtsam::Matrix H_j;
  (void)factor.evaluateError( pose_i, pose_j, H_i, H_j );
  const std::function<gtsam::Vector( const gtsam::Pose3&,
                                     const gtsam::Pose3& )>
      error = [ &factor ]( const gtsam::Pose3& left,
                           const gtsam::Pose3& right ) {
        return factor.evaluateError( left, right );
      };
  const gtsam::Matrix numerical_i =
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3,
                                   gtsam::Pose3>( error, pose_i, pose_j,
                                                  1e-6 );
  const gtsam::Matrix numerical_j =
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3,
                                   gtsam::Pose3>( error, pose_i, pose_j,
                                                  1e-6 );

  ASSERT_EQ( H_i.rows(), 3 );
  ASSERT_EQ( H_i.cols(), 6 );
  ASSERT_EQ( H_j.rows(), 3 );
  ASSERT_EQ( H_j.cols(), 6 );
  EXPECT_TRUE( H_i.isApprox( numerical_i, 1e-6 ) );
  EXPECT_TRUE( H_j.isApprox( numerical_j, 1e-6 ) );
  EXPECT_TRUE( H_i.rightCols<3>().isZero( 0.0 ) );
  EXPECT_TRUE( H_j.rightCols<3>().isZero( 0.0 ) );
}

TEST( FixedBiasAhrsFactor, NoiseAddsAlignmentCovarianceToPimCovariance )
{
  const auto           pim = preintegrate( gtsam::Vector3( 0.1, 0.2, 0.3 ),
                                           0.2, 0.004 );
  const gtsam::Matrix3 align_cov =
      gtsam::Matrix3::Identity() * 3e-5;
  const FixedBiasAhrsFactor factor(
      1, 2, pim, gtsam::Vector3::Zero(), align_cov );

  const auto gaussian =
      std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
          factor.noiseModel() );
  ASSERT_NE( gaussian, nullptr );
  EXPECT_TRUE( gaussian->covariance().isApprox(
      pim.preintMeasCov() + align_cov, 1e-12 ) );
}

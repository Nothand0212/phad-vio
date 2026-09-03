#include "phad/estimator/internal/initial_visual_evidence.hpp"

#include <gtest/gtest.h>
#include <gtsam/linear/NoiseModel.h>

#include <cmath>
#include <limits>

#include "phad/estimator/internal/vio_update_transaction.hpp"

namespace
{

  using phad::estimator::internal::classifyInitialVisualFactorFinal;
  using phad::estimator::internal::classifyInitialVisualFactorMask;
  using phad::estimator::internal::InitialVisualFactorFinalDecision;
  using phad::estimator::internal::InitialVisualFactorMaskDecision;
  using phad::estimator::internal::initialVisualUnderlyingGaussianQ;

  constexpr double kExpectedLimit = 11.344866730144373;

}  // namespace

TEST( InitialVisualEvidence, FactorMaskAndFinalBoundariesAreExact )
{
  EXPECT_DOUBLE_EQ(
      phad::estimator::internal::kInitialVisualFactorChiSquareLimit,
      kExpectedLimit );
  EXPECT_EQ( classifyInitialVisualFactorMask( kExpectedLimit ),
             InitialVisualFactorMaskDecision::kRetained );
  EXPECT_EQ(
      classifyInitialVisualFactorMask(
          std::nextafter( kExpectedLimit,
                          std::numeric_limits<double>::infinity() ) ),
      InitialVisualFactorMaskDecision::kMasked );
  EXPECT_EQ(
      classifyInitialVisualFactorMask(
          std::numeric_limits<double>::quiet_NaN() ),
      InitialVisualFactorMaskDecision::kContractError );

  EXPECT_EQ( classifyInitialVisualFactorFinal(
                 kExpectedLimit, std::numeric_limits<double>::min() ),
             InitialVisualFactorFinalDecision::kAccepted );
  EXPECT_EQ(
      classifyInitialVisualFactorFinal(
          std::nextafter( kExpectedLimit,
                          std::numeric_limits<double>::infinity() ),
          1.0 ),
      InitialVisualFactorFinalDecision::kVisualFitRejected );
  EXPECT_EQ( classifyInitialVisualFactorFinal( 0.0, 0.0 ),
             InitialVisualFactorFinalDecision::kCheiralityRejected );
  EXPECT_EQ(
      classifyInitialVisualFactorFinal(
          0.0, std::numeric_limits<double>::infinity() ),
      InitialVisualFactorFinalDecision::kContractError );
}

TEST( InitialVisualEvidence, VerifiedPoseKeepsRetainedFactorIdentity )
{
  phad::estimator::internal::VerifiedVisualPose verified;
  verified.m_stereo_factors = {
      { .m_landmark_id = 7U, .m_observation_index = 2U },
      { .m_landmark_id = 7U, .m_observation_index = 5U },
  };

  ASSERT_EQ( verified.m_stereo_factors.size(), 2U );
  EXPECT_EQ( verified.m_stereo_factors[ 0 ].m_landmark_id, 7U );
  EXPECT_EQ( verified.m_stereo_factors[ 0 ].m_observation_index, 2U );
  EXPECT_EQ( verified.m_stereo_factors[ 1 ].m_observation_index, 5U );
}

TEST( InitialVisualEvidence, UsesUnderlyingGaussianWithoutRobustWeight )
{
  const gtsam::Vector3          residual( 1.0, 2.0, 4.0 );
  const gtsam::Vector3          sigmas( 1.0, 2.0, 4.0 );
  const gtsam::SharedNoiseModel gaussian =
      gtsam::noiseModel::Diagonal::Sigmas( sigmas );
  const gtsam::SharedNoiseModel robust =
      gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create( 0.5 ), gaussian );

  EXPECT_DOUBLE_EQ(
      initialVisualUnderlyingGaussianQ( residual, gaussian ), 3.0 );
  EXPECT_DOUBLE_EQ(
      initialVisualUnderlyingGaussianQ( residual, robust ), 3.0 );
}

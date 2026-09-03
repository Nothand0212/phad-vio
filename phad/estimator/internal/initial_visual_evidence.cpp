#include "phad/estimator/internal/initial_visual_evidence.hpp"

#include <cmath>
#include <stdexcept>

namespace phad::estimator::internal
{

  InitialVisualFactorMaskDecision classifyInitialVisualFactorMask(
      const double statistic ) noexcept
  {
    if ( !std::isfinite( statistic ) || statistic < 0.0 )
    {
      return InitialVisualFactorMaskDecision::kContractError;
    }
    if ( statistic > kInitialVisualFactorChiSquareLimit )
    {
      return InitialVisualFactorMaskDecision::kMasked;
    }
    return InitialVisualFactorMaskDecision::kRetained;
  }

  InitialVisualFactorFinalDecision classifyInitialVisualFactorFinal(
      const double statistic, const double depth ) noexcept
  {
    if ( !std::isfinite( statistic ) || statistic < 0.0 ||
         !std::isfinite( depth ) )
    {
      return InitialVisualFactorFinalDecision::kContractError;
    }
    if ( statistic > kInitialVisualFactorChiSquareLimit )
    {
      return InitialVisualFactorFinalDecision::kVisualFitRejected;
    }
    if ( !( depth > 0.0 ) )
    {
      return InitialVisualFactorFinalDecision::kCheiralityRejected;
    }
    return InitialVisualFactorFinalDecision::kAccepted;
  }

  double initialVisualUnderlyingGaussianQ(
      const gtsam::Vector&           residual,
      const gtsam::SharedNoiseModel& noise_model )
  {
    constexpr Eigen::Index kStereoDimension = 3;
    if ( noise_model == nullptr || residual.size() != kStereoDimension ||
         noise_model->dim() !=
             static_cast<std::size_t>( kStereoDimension ) ||
         !residual.allFinite() )
    {
      throw std::invalid_argument(
          "initial visual factor residual/noise contract is invalid" );
    }
    const gtsam::Vector whitened = noise_model->unweightedWhiten( residual );
    if ( whitened.size() != kStereoDimension || !whitened.allFinite() )
    {
      throw std::runtime_error(
          "initial visual factor whitening is non-finite" );
    }
    const double statistic = whitened.squaredNorm();
    if ( !std::isfinite( statistic ) )
    {
      throw std::runtime_error(
          "initial visual factor statistic is non-finite" );
    }
    return statistic;
  }

}  // namespace phad::estimator::internal

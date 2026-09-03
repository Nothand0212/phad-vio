#pragma once

#include <gtsam/linear/NoiseModel.h>

#include <cstdint>

namespace phad::estimator::internal
{

  inline constexpr double kInitialVisualFactorChiSquareLimit =
      11.344866730144373;

  enum class InitialVisualFactorMaskDecision : std::uint8_t
  {
    kRetained      = 0,
    kMasked        = 1,
    kContractError = 2
  };

  enum class InitialVisualFactorFinalDecision : std::uint8_t
  {
    kAccepted           = 0,
    kVisualFitRejected  = 1,
    kCheiralityRejected = 2,
    kContractError      = 3
  };

  [[nodiscard]] InitialVisualFactorMaskDecision
  classifyInitialVisualFactorMask( double statistic ) noexcept;

  [[nodiscard]] InitialVisualFactorFinalDecision
  classifyInitialVisualFactorFinal( double statistic,
                                    double depth ) noexcept;

  [[nodiscard]] double initialVisualUnderlyingGaussianQ(
      const gtsam::Vector&           residual,
      const gtsam::SharedNoiseModel& noise_model );

}  // namespace phad::estimator::internal

#pragma once

#ifndef PHAD_ESTIMATOR_TESTING
#error "vio_estimator_test_access.hpp is available only to estimator tests"
#endif

#include <cstddef>

#include "phad/estimator/internal/initial_moving_root_math.hpp"
#include "phad/estimator/types.hpp"

namespace phad::estimator
{
  class VioEstimator;

  namespace internal
  {

    class VioEstimatorTestAccess
    {
    public:
      static void setNextMovingCandidateResult(
          VioEstimator& estimator, InitialMovingCandidateResult result );

      static void setNextCurrentGraphResult(
          VioEstimator& estimator, InitializationReason reason );

      static void setNextCurrentGraphContractError(
          VioEstimator& estimator );

      static void setNextInitialVisualDepth(
          VioEstimator& estimator, double depth );

      static void setNextInitialVisualWhiteningError(
          VioEstimator& estimator );

      static void setNextPnpProposalFailure(
          VioEstimator& estimator, PnpProbeResult failure );

      static void setNextPnpGuessRetryOutcome(
          VioEstimator& estimator, PnpGuessRetryOutcome outcome );

      [[nodiscard]] static std::size_t lastInitialVisualSolveWindowSize(
          const VioEstimator& estimator );
    };

  }  // namespace internal
}  // namespace phad::estimator

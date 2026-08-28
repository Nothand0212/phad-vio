#pragma once

#include "phad/estimator/types.hpp"

namespace phad::estimator::internal
{

  [[nodiscard]] bool hasValidColdRootObserveResult(
      const VioUpdateResult& result ) noexcept;

}  // namespace phad::estimator::internal

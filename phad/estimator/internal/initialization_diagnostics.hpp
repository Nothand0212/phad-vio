#pragma once

#include "phad/estimator/types.hpp"

namespace phad::estimator::internal
{

  [[nodiscard]] bool isValidInitializationDiagnostics(
      const InitializationDiagnostics& diagnostics ) noexcept;

  [[nodiscard]] VioUpdateResult finalizeInitializationResult(
      const InitializationDiagnostics& diagnostics,
      std::optional<VioEstimate>       estimate,
      std::string                      message );

  [[nodiscard]] VioUpdateResult finalizeInitializationResult(
      VioUpdateResult                  result,
      const InitializationDiagnostics& diagnostics,
      std::optional<VioEstimate>       estimate,
      std::string                      message );

}  // namespace phad::estimator::internal

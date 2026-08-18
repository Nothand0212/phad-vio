#pragma once

#include <Eigen/Core>
#include <optional>
#include <variant>

namespace phad::estimator::internal
{

  enum class GyroBiasInitialKind
  {
    kExactLink,
    kComponentRoot
  };

  enum class GyroBiasInitialErrorCode
  {
    kMissingPredecessor,
    kNonFiniteSelectedValue
  };

  using GyroBiasInitialResult =
      std::variant<Eigen::Vector3d, GyroBiasInitialErrorCode>;

  [[nodiscard]] inline GyroBiasInitialResult selectGyroBiasInitialValue(
      GyroBiasInitialKind                   kind,
      const std::optional<Eigen::Vector3d>& provisional,
      const std::optional<Eigen::Vector3d>& committed,
      const Eigen::Vector3d&                prior_mean_radps )
  {
    if ( kind == GyroBiasInitialKind::kComponentRoot )
    {
      if ( !prior_mean_radps.allFinite() )
      {
        return GyroBiasInitialErrorCode::kNonFiniteSelectedValue;
      }
      return prior_mean_radps;
    }

    const Eigen::Vector3d* selected = nullptr;
    if ( provisional.has_value() )
    {
      selected = &*provisional;
    }
    else if ( committed.has_value() )
    {
      selected = &*committed;
    }
    else
    {
      return GyroBiasInitialErrorCode::kMissingPredecessor;
    }

    if ( !selected->allFinite() )
    {
      return GyroBiasInitialErrorCode::kNonFiniteSelectedValue;
    }
    return *selected;
  }

}  // namespace phad::estimator::internal

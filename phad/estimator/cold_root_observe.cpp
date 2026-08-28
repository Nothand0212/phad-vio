#include "phad/estimator/internal/cold_root_observe.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace phad::estimator::internal
{
  namespace
  {

    template <typename... T>
    [[nodiscard]] bool allPresent( const std::optional<T>&... values ) noexcept
    {
      return ( values.has_value() && ... );
    }

    template <typename... T>
    [[nodiscard]] bool allAbsent( const std::optional<T>&... values ) noexcept
    {
      return ( !values.has_value() && ... );
    }

    [[nodiscard]] bool isFiniteNonnegative( const double value ) noexcept
    {
      return std::isfinite( value ) && value >= 0.0;
    }

    [[nodiscard]] bool allGatesNotEvaluated(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      return diagnostics.m_bootstrap_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_keyframe_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_stereo_population_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_root_geometry_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_imu_excitation_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_conditioning_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_initialization_solve_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_current_graph_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_commit_gate ==
                 ColdRootGateState::kNotEvaluated;
    }

    [[nodiscard]] bool allOptionalsAbsent(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      return allAbsent(
          diagnostics.m_attempt_id,
          diagnostics.m_bootstrap_sample_count,
          diagnostics.m_bootstrap_min_samples,
          diagnostics.m_bootstrap_duration_ns,
          diagnostics.m_bootstrap_min_duration_ns,
          diagnostics.m_bootstrap_acc_std_max_mps2,
          diagnostics.m_bootstrap_acc_std_limit_mps2,
          diagnostics.m_bootstrap_gyr_std_max_radps,
          diagnostics.m_bootstrap_gyr_std_limit_radps,
          diagnostics.m_bootstrap_acc_norm_error_mps2,
          diagnostics.m_bootstrap_acc_norm_tolerance_mps2,
          diagnostics.m_bootstrap_timeout_ns,
          diagnostics.m_moving_bootstrap_enabled,
          diagnostics.m_moving_suffix_sample_count,
          diagnostics.m_moving_suffix_duration_ns,
          diagnostics.m_moving_acc_mean_norm_mps2,
          diagnostics.m_moving_acc_mean_norm_min_mps2,
          diagnostics.m_current_positive_disparity_count,
          diagnostics.m_accumulated_seed_enabled,
          diagnostics.m_pending_unique_seed_count,
          diagnostics.m_effective_seed_count,
          diagnostics.m_min_seed_observations,
          diagnostics.m_geometry_accepted_landmarks,
          diagnostics.m_geometry_min_landmarks );
    }

    [[nodiscard]] bool hasValidBootstrapPredicate(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      if ( !allPresent(
               diagnostics.m_bootstrap_sample_count,
               diagnostics.m_bootstrap_min_samples,
               diagnostics.m_bootstrap_duration_ns,
               diagnostics.m_bootstrap_min_duration_ns,
               diagnostics.m_bootstrap_acc_std_max_mps2,
               diagnostics.m_bootstrap_acc_std_limit_mps2,
               diagnostics.m_bootstrap_gyr_std_max_radps,
               diagnostics.m_bootstrap_gyr_std_limit_radps,
               diagnostics.m_bootstrap_acc_norm_error_mps2,
               diagnostics.m_bootstrap_acc_norm_tolerance_mps2,
               diagnostics.m_bootstrap_timeout_ns ) )
      {
        return false;
      }

      if ( *diagnostics.m_bootstrap_sample_count < 2U ||
           *diagnostics.m_bootstrap_min_samples < 2U ||
           *diagnostics.m_bootstrap_duration_ns <= 0 ||
           *diagnostics.m_bootstrap_min_duration_ns <= 0 ||
           *diagnostics.m_bootstrap_timeout_ns <
               *diagnostics.m_bootstrap_min_duration_ns ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_acc_std_max_mps2 ) ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_acc_std_limit_mps2 ) ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_gyr_std_max_radps ) ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_gyr_std_limit_radps ) ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_acc_norm_error_mps2 ) ||
           !isFiniteNonnegative(
               *diagnostics.m_bootstrap_acc_norm_tolerance_mps2 ) )
      {
        return false;
      }

      const bool static_ready =
          *diagnostics.m_bootstrap_sample_count >=
              *diagnostics.m_bootstrap_min_samples &&
          *diagnostics.m_bootstrap_duration_ns >=
              *diagnostics.m_bootstrap_min_duration_ns &&
          *diagnostics.m_bootstrap_acc_std_max_mps2 <=
              *diagnostics.m_bootstrap_acc_std_limit_mps2 &&
          *diagnostics.m_bootstrap_gyr_std_max_radps <=
              *diagnostics.m_bootstrap_gyr_std_limit_radps &&
          *diagnostics.m_bootstrap_acc_norm_error_mps2 <=
              *diagnostics.m_bootstrap_acc_norm_tolerance_mps2;

      const bool suffix_present = allPresent(
          diagnostics.m_moving_suffix_sample_count,
          diagnostics.m_moving_suffix_duration_ns,
          diagnostics.m_moving_acc_mean_norm_mps2,
          diagnostics.m_moving_acc_mean_norm_min_mps2 );
      const bool suffix_absent = allAbsent(
          diagnostics.m_moving_suffix_sample_count,
          diagnostics.m_moving_suffix_duration_ns,
          diagnostics.m_moving_acc_mean_norm_mps2,
          diagnostics.m_moving_acc_mean_norm_min_mps2 );
      if ( !suffix_present && !suffix_absent )
      {
        return false;
      }

      bool suffix_valid = false;
      if ( suffix_present )
      {
        suffix_valid =
            diagnostics.m_moving_bootstrap_enabled == true &&
            *diagnostics.m_moving_suffix_sample_count >=
                *diagnostics.m_bootstrap_min_samples &&
            *diagnostics.m_moving_suffix_duration_ns >=
                *diagnostics.m_bootstrap_min_duration_ns &&
            isFiniteNonnegative(
                *diagnostics.m_moving_acc_mean_norm_mps2 ) &&
            *diagnostics.m_moving_acc_mean_norm_min_mps2 ==
                std::numeric_limits<double>::epsilon();
        if ( !suffix_valid )
        {
          return false;
        }
      }

      switch ( diagnostics.m_bootstrap_path )
      {
        case ColdRootBootstrapPath::kCollecting:
          if ( static_ready || diagnostics.m_bootstrap_gate != ColdRootGateState::kFailed ||
               !diagnostics.m_moving_bootstrap_enabled.has_value() )
          {
            return false;
          }
          if ( !*diagnostics.m_moving_bootstrap_enabled )
          {
            return suffix_absent;
          }
          return suffix_absent ||
                 ( suffix_valid &&
                   *diagnostics.m_moving_acc_mean_norm_mps2 <=
                       *diagnostics.m_moving_acc_mean_norm_min_mps2 );
        case ColdRootBootstrapPath::kStatic:
          return static_ready &&
                 diagnostics.m_bootstrap_gate ==
                     ColdRootGateState::kPassed &&
                 !diagnostics.m_moving_bootstrap_enabled.has_value() &&
                 suffix_absent;
        case ColdRootBootstrapPath::kMoving:
          return !static_ready &&
                 diagnostics.m_bootstrap_gate ==
                     ColdRootGateState::kPassed &&
                 diagnostics.m_moving_bootstrap_enabled == true &&
                 suffix_valid &&
                 *diagnostics.m_moving_acc_mean_norm_mps2 >
                     *diagnostics.m_moving_acc_mean_norm_min_mps2;
        case ColdRootBootstrapPath::kNotEvaluated:
          return false;
      }
      return false;
    }

    [[nodiscard]] bool hasNoPopulationFields(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      return diagnostics.m_seed_input_origin ==
                 ColdRootSeedInputOrigin::kNotEvaluated &&
             allAbsent( diagnostics.m_pending_unique_seed_count,
                        diagnostics.m_effective_seed_count,
                        diagnostics.m_min_seed_observations );
    }

    [[nodiscard]] bool hasValidPopulationFields(
        const ColdRootObserveDiagnostics& diagnostics,
        const ColdRootGateState           expected_gate ) noexcept
    {
      if ( diagnostics.m_stereo_population_gate != expected_gate ||
           !allPresent( diagnostics.m_effective_seed_count,
                        diagnostics.m_min_seed_observations ) ||
           *diagnostics.m_min_seed_observations == 0U )
      {
        return false;
      }

      switch ( diagnostics.m_seed_input_origin )
      {
        case ColdRootSeedInputOrigin::kCurrentPacket:
          if ( diagnostics.m_pending_unique_seed_count.has_value() ||
               *diagnostics.m_effective_seed_count !=
                   *diagnostics.m_current_positive_disparity_count )
          {
            return false;
          }
          break;
        case ColdRootSeedInputOrigin::kAccumulated:
          if ( diagnostics.m_accumulated_seed_enabled != true ||
               !diagnostics.m_pending_unique_seed_count.has_value() ||
               *diagnostics.m_effective_seed_count !=
                   *diagnostics.m_pending_unique_seed_count )
          {
            return false;
          }
          break;
        case ColdRootSeedInputOrigin::kNotEvaluated:
          return false;
      }

      const bool population_passed =
          *diagnostics.m_effective_seed_count >=
          *diagnostics.m_min_seed_observations;
      return ( expected_gate == ColdRootGateState::kPassed &&
               population_passed ) ||
             ( expected_gate == ColdRootGateState::kFailed &&
               !population_passed );
    }

    [[nodiscard]] bool hasNoGeometryFields(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      return diagnostics.m_geometry_result ==
                 ColdRootGeometryResult::kNotEvaluated &&
             allAbsent( diagnostics.m_attempt_id,
                        diagnostics.m_geometry_accepted_landmarks,
                        diagnostics.m_geometry_min_landmarks );
    }

    [[nodiscard]] bool hasValidGeometryFields(
        const ColdRootObserveDiagnostics& diagnostics,
        const ColdRootGateState           expected_gate ) noexcept
    {
      if ( diagnostics.m_root_geometry_gate != expected_gate ||
           !allPresent( diagnostics.m_attempt_id,
                        diagnostics.m_geometry_accepted_landmarks,
                        diagnostics.m_geometry_min_landmarks,
                        diagnostics.m_effective_seed_count ) ||
           *diagnostics.m_attempt_id == 0U ||
           *diagnostics.m_geometry_min_landmarks != 1U ||
           *diagnostics.m_geometry_accepted_landmarks >
               *diagnostics.m_effective_seed_count )
      {
        return false;
      }

      if ( expected_gate == ColdRootGateState::kPassed )
      {
        return diagnostics.m_geometry_result ==
                   ColdRootGeometryResult::kAccepted &&
               *diagnostics.m_geometry_accepted_landmarks >=
                   *diagnostics.m_geometry_min_landmarks;
      }

      if ( diagnostics.m_geometry_result ==
           ColdRootGeometryResult::kEmptyAfterFilter )
      {
        return *diagnostics.m_geometry_accepted_landmarks == 0U;
      }
      return diagnostics.m_geometry_result ==
                 ColdRootGeometryResult::kNonfiniteBackprojection ||
             diagnostics.m_geometry_result ==
                 ColdRootGeometryResult::kBehindCamera;
    }

    [[nodiscard]] bool formalGatesNotEvaluated(
        const ColdRootObserveDiagnostics& diagnostics ) noexcept
    {
      return diagnostics.m_imu_excitation_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_conditioning_gate ==
                 ColdRootGateState::kNotEvaluated &&
             diagnostics.m_initialization_solve_gate ==
                 ColdRootGateState::kNotEvaluated;
    }

  }  // namespace

  bool hasValidColdRootObserveResult(
      const VioUpdateResult& result ) noexcept
  {
    const ColdRootObserveDiagnostics& diagnostics =
        result.diagnostics.m_cold_root;
    if ( !formalGatesNotEvaluated( diagnostics ) )
    {
      return false;
    }

    if ( diagnostics.m_phase == ColdRootPhase::kNotEvaluated )
    {
      return diagnostics.m_reason == ColdRootReason::kNotEvaluated &&
             diagnostics.m_bootstrap_path ==
                 ColdRootBootstrapPath::kNotEvaluated &&
             diagnostics.m_seed_input_origin ==
                 ColdRootSeedInputOrigin::kNotEvaluated &&
             diagnostics.m_geometry_result ==
                 ColdRootGeometryResult::kNotEvaluated &&
             allGatesNotEvaluated( diagnostics ) &&
             allOptionalsAbsent( diagnostics );
    }

    if ( diagnostics.m_reason == ColdRootReason::kNotEvaluated ||
         !allPresent(
             diagnostics.m_current_positive_disparity_count,
             diagnostics.m_accumulated_seed_enabled ) ||
         !hasValidBootstrapPredicate( diagnostics ) )
    {
      return false;
    }

    const bool no_estimate = !result.estimate.has_value();
    switch ( diagnostics.m_phase )
    {
      case ColdRootPhase::kBootstrap:
      {
        const bool evidence_insufficient =
            diagnostics.m_reason == ColdRootReason::kEvidenceInsufficient &&
            result.status == UpdateStatus::kInitializing &&
            *diagnostics.m_bootstrap_duration_ns <
                *diagnostics.m_bootstrap_timeout_ns;
        const bool timed_out =
            diagnostics.m_reason == ColdRootReason::kTimedOut &&
            result.status == UpdateStatus::kFailed &&
            *diagnostics.m_bootstrap_duration_ns >=
                *diagnostics.m_bootstrap_timeout_ns;
        return no_estimate &&
               diagnostics.m_bootstrap_path ==
                   ColdRootBootstrapPath::kCollecting &&
               diagnostics.m_keyframe_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_stereo_population_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_root_geometry_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_commit_gate ==
                   ColdRootGateState::kNotEvaluated &&
               hasNoPopulationFields( diagnostics ) &&
               hasNoGeometryFields( diagnostics ) &&
               ( evidence_insufficient || timed_out );
      }
      case ColdRootPhase::kKeyframe:
        return no_estimate &&
               result.status == UpdateStatus::kInitializing &&
               diagnostics.m_reason == ColdRootReason::kKeyframeRequired &&
               diagnostics.m_bootstrap_gate ==
                   ColdRootGateState::kPassed &&
               diagnostics.m_keyframe_gate == ColdRootGateState::kFailed &&
               diagnostics.m_stereo_population_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_root_geometry_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_commit_gate ==
                   ColdRootGateState::kNotEvaluated &&
               hasNoPopulationFields( diagnostics ) &&
               hasNoGeometryFields( diagnostics );
      case ColdRootPhase::kStereoPopulation:
      {
        const bool current_insufficient =
            diagnostics.m_reason ==
                ColdRootReason::kPopulationInsufficient &&
            diagnostics.m_seed_input_origin ==
                ColdRootSeedInputOrigin::kCurrentPacket;
        const bool accumulating =
            diagnostics.m_reason ==
                ColdRootReason::kPopulationAccumulating &&
            diagnostics.m_keyframe_gate == ColdRootGateState::kPassed &&
            diagnostics.m_seed_input_origin ==
                ColdRootSeedInputOrigin::kAccumulated;
        const bool keyframe_trace_valid =
            diagnostics.m_keyframe_gate == ColdRootGateState::kPassed ||
            ( diagnostics.m_keyframe_gate ==
                  ColdRootGateState::kNotEvaluated &&
              current_insufficient &&
              *diagnostics.m_current_positive_disparity_count == 0U &&
              *diagnostics.m_effective_seed_count == 0U );
        return no_estimate &&
               result.status == UpdateStatus::kInitializing &&
               diagnostics.m_bootstrap_gate ==
                   ColdRootGateState::kPassed &&
               keyframe_trace_valid &&
               diagnostics.m_root_geometry_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_commit_gate ==
                   ColdRootGateState::kNotEvaluated &&
               hasValidPopulationFields(
                   diagnostics, ColdRootGateState::kFailed ) &&
               hasNoGeometryFields( diagnostics ) &&
               ( current_insufficient || accumulating );
      }
      case ColdRootPhase::kRootGeometry:
        return no_estimate && result.status == UpdateStatus::kRejected &&
               diagnostics.m_reason == ColdRootReason::kGeometryRejected &&
               diagnostics.m_bootstrap_gate ==
                   ColdRootGateState::kPassed &&
               diagnostics.m_keyframe_gate == ColdRootGateState::kPassed &&
               hasValidPopulationFields(
                   diagnostics, ColdRootGateState::kPassed ) &&
               hasValidGeometryFields(
                   diagnostics, ColdRootGateState::kFailed ) &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kNotEvaluated &&
               diagnostics.m_commit_gate ==
                   ColdRootGateState::kNotEvaluated;
      case ColdRootPhase::kCurrentGraph:
      {
        const bool graph_reason =
            diagnostics.m_reason == ColdRootReason::kGraphBuildFailed ||
            diagnostics.m_reason == ColdRootReason::kGraphSolveFailed ||
            diagnostics.m_reason == ColdRootReason::kGraphValidationFailed;
        return no_estimate && result.status == UpdateStatus::kFailed &&
               graph_reason &&
               diagnostics.m_bootstrap_gate ==
                   ColdRootGateState::kPassed &&
               diagnostics.m_keyframe_gate == ColdRootGateState::kPassed &&
               hasValidPopulationFields(
                   diagnostics, ColdRootGateState::kPassed ) &&
               hasValidGeometryFields(
                   diagnostics, ColdRootGateState::kPassed ) &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kFailed &&
               diagnostics.m_commit_gate ==
                   ColdRootGateState::kNotEvaluated;
      }
      case ColdRootPhase::kCommit:
        return result.status == UpdateStatus::kOk &&
               result.estimate.has_value() &&
               diagnostics.m_reason == ColdRootReason::kCommitted &&
               diagnostics.m_bootstrap_gate ==
                   ColdRootGateState::kPassed &&
               diagnostics.m_keyframe_gate == ColdRootGateState::kPassed &&
               hasValidPopulationFields(
                   diagnostics, ColdRootGateState::kPassed ) &&
               hasValidGeometryFields(
                   diagnostics, ColdRootGateState::kPassed ) &&
               diagnostics.m_current_graph_gate ==
                   ColdRootGateState::kPassed &&
               diagnostics.m_commit_gate == ColdRootGateState::kPassed;
      case ColdRootPhase::kNotEvaluated:
        return false;
    }
    return false;
  }

}  // namespace phad::estimator::internal

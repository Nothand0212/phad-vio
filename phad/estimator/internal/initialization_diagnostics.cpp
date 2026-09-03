#include "phad/estimator/internal/initialization_diagnostics.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace phad::estimator::internal
{
  namespace
  {

    [[nodiscard]] bool isMovingPendingReason(
        const InitializationReason reason ) noexcept
    {
      switch ( reason )
      {
        case InitializationReason::kPoseCountInsufficient:
        case InitializationReason::kDurationInsufficient:
        case InitializationReason::kVisualConnectivityInsufficient:
        case InitializationReason::kVisualPopulationInsufficient:
        case InitializationReason::kVisualSupportInsufficient:
        case InitializationReason::kVisualFitRejected:
        case InitializationReason::kCheiralityRejected:
          return true;
        default:
          return false;
      }
    }

    [[nodiscard]] bool isMovingPendingPhase(
        const InitializationReason reason,
        const InitializationPhase  phase ) noexcept
    {
      switch ( reason )
      {
        case InitializationReason::kPoseCountInsufficient:
        case InitializationReason::kDurationInsufficient:
          return phase == InitializationPhase::kCollectingEvidence;
        case InitializationReason::kVisualConnectivityInsufficient:
        case InitializationReason::kVisualPopulationInsufficient:
        case InitializationReason::kVisualSupportInsufficient:
        case InitializationReason::kVisualFitRejected:
        case InitializationReason::kCheiralityRejected:
          return phase == InitializationPhase::kVisualSolve;
        default:
          return false;
      }
    }

    [[nodiscard]] bool isMovingRecoverableReason(
        const InitializationReason reason ) noexcept
    {
      switch ( reason )
      {
        case InitializationReason::kRankDeficient:
        case InitializationReason::kIllConditioned:
        case InitializationReason::kNoConvergence:
        case InitializationReason::kJointDofInsufficient:
        case InitializationReason::kJointInconsistent:
        case InitializationReason::kCurrentGraphUnderconstrained:
        case InitializationReason::kCurrentGraphRejected:
          return true;
        default:
          return false;
      }
    }

    [[nodiscard]] bool isCurrentGraphReason(
        const InitializationReason reason ) noexcept
    {
      return reason == InitializationReason::kCurrentGraphUnderconstrained ||
             reason == InitializationReason::kCurrentGraphRejected;
    }

    [[nodiscard]] bool isMovingRecoverablePhase(
        const InitializationReason reason,
        const InitializationPhase  phase ) noexcept
    {
      switch ( reason )
      {
        case InitializationReason::kRankDeficient:
        case InitializationReason::kIllConditioned:
          return phase == InitializationPhase::kGyroAlignment ||
                 phase == InitializationPhase::kGravityVelocitySolve;
        case InitializationReason::kNoConvergence:
          return phase == InitializationPhase::kJointConsistency;
        case InitializationReason::kJointDofInsufficient:
        case InitializationReason::kJointInconsistent:
          return phase == InitializationPhase::kJointConsistency;
        case InitializationReason::kCurrentGraphUnderconstrained:
        case InitializationReason::kCurrentGraphRejected:
          return phase == InitializationPhase::kCurrentGraph;
        default:
          return false;
      }
    }

    [[nodiscard]] bool isFiniteFit(
        const std::optional<InitializationFitSummary>& fit ) noexcept
    {
      return !fit.has_value() ||
             ( std::isfinite( fit->m_statistic ) &&
               std::isfinite( fit->m_limit ) );
    }

    [[nodiscard]] bool isFiniteThreshold(
        const std::optional<InitializationThresholdSummary>& threshold ) noexcept
    {
      return !threshold.has_value() ||
             ( std::isfinite( threshold->m_value ) &&
               std::isfinite( threshold->m_limit ) );
    }

    [[nodiscard]] bool hasCanonicalNonMovingSummaries(
        const InitializationDiagnostics& diagnostics ) noexcept
    {
      return diagnostics.m_connected_poses.m_actual == 0U &&
             diagnostics.m_connected_poses.m_required == 0U &&
             diagnostics.m_duration.m_actual_ns == 0 &&
             diagnostics.m_duration.m_required_ns == 0 &&
             diagnostics.m_connected_landmarks.m_actual == 0U &&
             diagnostics.m_connected_landmarks.m_required == 0U &&
             !diagnostics.m_visual_support.has_value() &&
             !diagnostics.m_visual_fit.has_value() &&
             !diagnostics.m_gyro_rank.has_value() &&
             !diagnostics.m_gyro_normalized_condition.has_value() &&
             !diagnostics.m_gravity_velocity_rank.has_value() &&
             !diagnostics.m_gravity_velocity_normalized_condition.has_value() &&
             !diagnostics.m_joint_fit.has_value();
    }

    [[nodiscard]] bool hasValidMovingSummaries(
        const InitializationDiagnostics& diagnostics ) noexcept
    {
      if ( diagnostics.m_duration.m_actual_ns < 0 ||
           diagnostics.m_duration.m_required_ns < 0 ||
           !isFiniteFit( diagnostics.m_visual_fit ) ||
           !isFiniteFit( diagnostics.m_joint_fit ) ||
           !isFiniteThreshold( diagnostics.m_gyro_normalized_condition ) ||
           !isFiniteThreshold(
               diagnostics.m_gravity_velocity_normalized_condition ) )
      {
        return false;
      }
      if ( diagnostics.m_gyro_normalized_condition.has_value() &&
           !diagnostics.m_gyro_rank.has_value() )
      {
        return false;
      }
      if ( diagnostics.m_gravity_velocity_normalized_condition.has_value() &&
           !diagnostics.m_gravity_velocity_rank.has_value() )
      {
        return false;
      }
      if ( !diagnostics.m_attempt_id.has_value() &&
           ( diagnostics.m_gyro_rank.has_value() ||
             diagnostics.m_gyro_normalized_condition.has_value() ||
             diagnostics.m_gravity_velocity_rank.has_value() ||
             diagnostics.m_gravity_velocity_normalized_condition.has_value() ||
             diagnostics.m_joint_fit.has_value() ) )
      {
        return false;
      }
      return true;
    }

  }  // namespace

  bool isValidInitializationDiagnostics(
      const InitializationDiagnostics& diagnostics ) noexcept
  {
    const bool has_attempt = diagnostics.m_attempt_id.has_value();
    switch ( diagnostics.m_path )
    {
      case InitializationPath::kNone:
        return hasCanonicalNonMovingSummaries( diagnostics ) &&
               diagnostics.m_phase == InitializationPhase::kInactive &&
               diagnostics.m_reason == InitializationReason::kNone &&
               diagnostics.m_outcome == InitializationOutcome::kNotActive &&
               !has_attempt &&
               diagnostics.m_current_graph ==
                   InitializationGraphResult::kNotEvaluated &&
               diagnostics.m_transaction ==
                   InitializationTransactionResult::kNotEvaluated;
      case InitializationPath::kStatic:
        if ( has_attempt ||
             !hasCanonicalNonMovingSummaries( diagnostics ) )
        {
          return false;
        }
        if ( diagnostics.m_outcome == InitializationOutcome::kPending )
        {
          return diagnostics.m_phase ==
                     InitializationPhase::kCollectingEvidence &&
                 diagnostics.m_reason ==
                     InitializationReason::kStaticEvidenceInsufficient &&
                 diagnostics.m_current_graph ==
                     InitializationGraphResult::kNotEvaluated &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kEvidenceCommitted;
        }
        if ( diagnostics.m_outcome == InitializationOutcome::kFatalError )
        {
          return diagnostics.m_phase ==
                     InitializationPhase::kCollectingEvidence &&
                 diagnostics.m_reason == InitializationReason::kStaticTimeout &&
                 diagnostics.m_current_graph ==
                     InitializationGraphResult::kNotEvaluated &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kRolledBack;
        }
        return diagnostics.m_outcome == InitializationOutcome::kCommitted &&
               diagnostics.m_phase == InitializationPhase::kCommit &&
               diagnostics.m_reason == InitializationReason::kNone &&
               diagnostics.m_current_graph ==
                   InitializationGraphResult::kValidated &&
               diagnostics.m_transaction ==
                   InitializationTransactionResult::kRootCommitted;
      case InitializationPath::kMoving:
        if ( !hasValidMovingSummaries( diagnostics ) )
        {
          return false;
        }
        if ( diagnostics.m_outcome == InitializationOutcome::kPending )
        {
          return !has_attempt &&
                 isMovingPendingReason( diagnostics.m_reason ) &&
                 isMovingPendingPhase( diagnostics.m_reason,
                                       diagnostics.m_phase ) &&
                 diagnostics.m_current_graph ==
                     InitializationGraphResult::kNotEvaluated &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kEvidenceCommitted;
        }
        if ( diagnostics.m_outcome ==
             InitializationOutcome::kRecoverableRejected )
        {
          const bool current_graph_reason =
              isCurrentGraphReason( diagnostics.m_reason );
          return has_attempt &&
                 isMovingRecoverableReason( diagnostics.m_reason ) &&
                 isMovingRecoverablePhase( diagnostics.m_reason,
                                           diagnostics.m_phase ) &&
                 diagnostics.m_current_graph ==
                     ( current_graph_reason
                           ? InitializationGraphResult::kRejected
                           : InitializationGraphResult::kNotEvaluated ) &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kEvidenceCommitted;
        }
        if ( diagnostics.m_outcome == InitializationOutcome::kCommitted )
        {
          return has_attempt &&
                 diagnostics.m_phase == InitializationPhase::kCommit &&
                 diagnostics.m_reason == InitializationReason::kNone &&
                 diagnostics.m_current_graph ==
                     InitializationGraphResult::kValidated &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kRootCommitted;
        }
        if ( diagnostics.m_outcome == InitializationOutcome::kFatalError )
        {
          return diagnostics.m_phase != InitializationPhase::kInactive &&
                 diagnostics.m_reason == InitializationReason::kContractError &&
                 diagnostics.m_current_graph !=
                     InitializationGraphResult::kValidated &&
                 diagnostics.m_transaction ==
                     InitializationTransactionResult::kRolledBack;
        }
        return false;
    }
    return false;
  }

  VioUpdateResult finalizeInitializationResult(
      const InitializationDiagnostics& diagnostics,
      std::optional<VioEstimate>       estimate,
      std::string                      message )
  {
    return finalizeInitializationResult(
        VioUpdateResult{}, diagnostics, std::move( estimate ),
        std::move( message ) );
  }

  VioUpdateResult finalizeInitializationResult(
      VioUpdateResult                  result,
      const InitializationDiagnostics& diagnostics,
      std::optional<VioEstimate>       estimate,
      std::string                      message )
  {
    if ( !isValidInitializationDiagnostics( diagnostics ) )
    {
      throw std::invalid_argument(
          "initialization diagnostics lifecycle is invalid" );
    }
    if ( diagnostics.m_outcome == InitializationOutcome::kPending ||
         diagnostics.m_outcome ==
             InitializationOutcome::kRecoverableRejected )
    {
      if ( estimate.has_value() )
      {
        throw std::invalid_argument(
            "pending initialization must not carry an estimate" );
      }
      result.status = UpdateStatus::kInitializing;
      result.estimate.reset();
      result.diagnostics.m_initialization = diagnostics;
      result.message                      = std::move( message );
      return result;
    }
    if ( diagnostics.m_outcome == InitializationOutcome::kCommitted )
    {
      if ( !estimate.has_value() )
      {
        throw std::invalid_argument(
            "committed initialization requires an estimate" );
      }
      result.status                       = UpdateStatus::kOk;
      result.estimate                     = std::move( estimate );
      result.diagnostics.m_initialization = diagnostics;
      result.message.clear();
      return result;
    }
    if ( diagnostics.m_outcome == InitializationOutcome::kFatalError )
    {
      if ( estimate.has_value() )
      {
        throw std::invalid_argument(
            "fatal initialization must not carry an estimate" );
      }
      result.status = UpdateStatus::kFailed;
      result.estimate.reset();
      result.diagnostics.m_initialization = diagnostics;
      result.message                      = std::move( message );
      return result;
    }
    throw std::invalid_argument(
        "initialization outcome is not supported by the result finalizer" );
  }

}  // namespace phad::estimator::internal

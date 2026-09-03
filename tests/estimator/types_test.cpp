#include "phad/estimator/types.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/estimator/internal/initialization_diagnostics.hpp"
#include "tests/estimator/vio_test_utils.hpp"

TEST( EstimatorTypes, LandmarkIdAliasesCommon )
{
  static_assert( std::is_same_v<phad::common::LandmarkId, std::uint64_t> );
  static_assert(
      std::is_same_v<phad::estimator::LandmarkId, phad::common::LandmarkId> );
}

TEST( EstimatorTypes, OutlierAvgReprojDefault )
{
  phad::estimator::EstimatorOptions options;
  EXPECT_DOUBLE_EQ( options.outlier_avg_reproj_px, 4.0 );
  EXPECT_TRUE( options.enable_outlier_cull );
  EXPECT_TRUE( options.block_culled_rebirth );
}

TEST( EstimatorTypes, InitializationDiagnosticsInactiveDefaults )
{
  const phad::estimator::UpdateDiagnostics diagnostics;
  const auto&                              initialization = diagnostics.m_initialization;

  EXPECT_EQ( initialization.m_path,
             phad::estimator::InitializationPath::kNone );
  EXPECT_EQ( initialization.m_phase,
             phad::estimator::InitializationPhase::kInactive );
  EXPECT_EQ( initialization.m_reason,
             phad::estimator::InitializationReason::kNone );
  EXPECT_EQ( initialization.m_outcome,
             phad::estimator::InitializationOutcome::kNotActive );
  EXPECT_FALSE( initialization.m_attempt_id.has_value() );
  EXPECT_EQ( initialization.m_current_graph,
             phad::estimator::InitializationGraphResult::kNotEvaluated );
  EXPECT_EQ(
      initialization.m_transaction,
      phad::estimator::InitializationTransactionResult::kNotEvaluated );
}

TEST( EstimatorTypes, InitializationDiagnosticsLegalCombinations )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationReason;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::internal::isValidInitializationDiagnostics;

  const auto makeDiagnostics = [](
                                   const InitializationPath        path,
                                   const InitializationPhase       phase,
                                   const InitializationReason      reason,
                                   const InitializationOutcome     outcome,
                                   std::optional<std::uint64_t>    attempt_id,
                                   const InitializationGraphResult graph,
                                   const InitializationTransactionResult
                                       transaction ) {
    InitializationDiagnostics diagnostics;
    diagnostics.m_path          = path;
    diagnostics.m_phase         = phase;
    diagnostics.m_reason        = reason;
    diagnostics.m_outcome       = outcome;
    diagnostics.m_attempt_id    = attempt_id;
    diagnostics.m_current_graph = graph;
    diagnostics.m_transaction   = transaction;
    return diagnostics;
  };

  const std::vector<InitializationDiagnostics> legal{
      {},
      makeDiagnostics(
          InitializationPath::kStatic,
          InitializationPhase::kCollectingEvidence,
          InitializationReason::kStaticEvidenceInsufficient,
          InitializationOutcome::kPending, std::nullopt,
          InitializationGraphResult::kNotEvaluated,
          InitializationTransactionResult::kEvidenceCommitted ),
      makeDiagnostics(
          InitializationPath::kStatic,
          InitializationPhase::kCollectingEvidence,
          InitializationReason::kStaticTimeout,
          InitializationOutcome::kFatalError, std::nullopt,
          InitializationGraphResult::kNotEvaluated,
          InitializationTransactionResult::kRolledBack ),
      makeDiagnostics(
          InitializationPath::kStatic, InitializationPhase::kCommit,
          InitializationReason::kNone, InitializationOutcome::kCommitted,
          std::nullopt, InitializationGraphResult::kValidated,
          InitializationTransactionResult::kRootCommitted ),
      makeDiagnostics(
          InitializationPath::kMoving,
          InitializationPhase::kCollectingEvidence,
          InitializationReason::kPoseCountInsufficient,
          InitializationOutcome::kPending, std::nullopt,
          InitializationGraphResult::kNotEvaluated,
          InitializationTransactionResult::kEvidenceCommitted ),
      makeDiagnostics(
          InitializationPath::kMoving,
          InitializationPhase::kGyroAlignment,
          InitializationReason::kRankDeficient,
          InitializationOutcome::kRecoverableRejected, 1U,
          InitializationGraphResult::kNotEvaluated,
          InitializationTransactionResult::kEvidenceCommitted ),
      makeDiagnostics(
          InitializationPath::kMoving, InitializationPhase::kCommit,
          InitializationReason::kNone, InitializationOutcome::kCommitted, 2U,
          InitializationGraphResult::kValidated,
          InitializationTransactionResult::kRootCommitted ),
      makeDiagnostics(
          InitializationPath::kMoving,
          InitializationPhase::kJointConsistency,
          InitializationReason::kContractError,
          InitializationOutcome::kFatalError, 3U,
          InitializationGraphResult::kNotEvaluated,
          InitializationTransactionResult::kRolledBack ),
  };
  for ( const InitializationDiagnostics& diagnostics : legal )
  {
    EXPECT_TRUE( isValidInitializationDiagnostics( diagnostics ) );
  }

  InitializationDiagnostics invalid = legal.front();
  invalid.m_outcome                 = InitializationOutcome::kCommitted;
  EXPECT_FALSE( isValidInitializationDiagnostics( invalid ) );

  invalid              = legal[ 4 ];
  invalid.m_attempt_id = 7U;
  EXPECT_FALSE( isValidInitializationDiagnostics( invalid ) );

  invalid = legal[ 6 ];
  invalid.m_attempt_id.reset();
  EXPECT_FALSE( isValidInitializationDiagnostics( invalid ) );

  invalid         = legal[ 4 ];
  invalid.m_phase = InitializationPhase::kCommit;
  EXPECT_FALSE( isValidInitializationDiagnostics( invalid ) );

  invalid         = legal[ 5 ];
  invalid.m_phase = InitializationPhase::kVisualSolve;
  EXPECT_FALSE( isValidInitializationDiagnostics( invalid ) );
}

TEST( EstimatorTypes, InitializationDiagnosticsOptionalGroupPresence )
{
  using phad::estimator::InitializationCountSummary;
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationFitSummary;
  using phad::estimator::InitializationRankSummary;
  using phad::estimator::InitializationThresholdSummary;
  using phad::estimator::internal::isValidInitializationDiagnostics;

  InitializationDiagnostics diagnostics;
  EXPECT_EQ( diagnostics.m_connected_poses.m_actual, 0U );
  EXPECT_EQ( diagnostics.m_connected_poses.m_required, 0U );
  EXPECT_EQ( diagnostics.m_duration.m_actual_ns, 0 );
  EXPECT_EQ( diagnostics.m_duration.m_required_ns, 0 );
  EXPECT_EQ( diagnostics.m_connected_landmarks.m_actual, 0U );
  EXPECT_EQ( diagnostics.m_connected_landmarks.m_required, 0U );
  EXPECT_FALSE( diagnostics.m_visual_support.has_value() );
  EXPECT_FALSE( diagnostics.m_visual_fit.has_value() );
  EXPECT_FALSE( diagnostics.m_gyro_rank.has_value() );
  EXPECT_FALSE( diagnostics.m_gyro_normalized_condition.has_value() );
  EXPECT_FALSE( diagnostics.m_gravity_velocity_rank.has_value() );
  EXPECT_FALSE(
      diagnostics.m_gravity_velocity_normalized_condition.has_value() );
  EXPECT_FALSE( diagnostics.m_joint_fit.has_value() );

  diagnostics.m_visual_support =
      InitializationCountSummary{ .m_actual = 12U, .m_required = 10U };
  diagnostics.m_visual_fit = InitializationFitSummary{
      .m_statistic = 2.0, .m_limit = 3.0, .m_effective_dof = 3 };
  diagnostics.m_gyro_rank =
      InitializationRankSummary{ .m_actual = 3U, .m_required = 3U };
  diagnostics.m_gyro_normalized_condition =
      InitializationThresholdSummary{ .m_value = 10.0, .m_limit = 20.0 };
  diagnostics.m_gravity_velocity_rank =
      InitializationRankSummary{ .m_actual = 20U, .m_required = 20U };
  diagnostics.m_gravity_velocity_normalized_condition =
      InitializationThresholdSummary{ .m_value = 11.0, .m_limit = 20.0 };
  diagnostics.m_joint_fit = InitializationFitSummary{
      .m_statistic = 21.0, .m_limit = 33.0, .m_effective_dof = 22 };

  EXPECT_EQ( diagnostics.m_visual_support->m_actual, 12U );
  EXPECT_DOUBLE_EQ( diagnostics.m_visual_fit->m_statistic, 2.0 );
  EXPECT_EQ( diagnostics.m_gyro_rank->m_required, 3U );
  EXPECT_DOUBLE_EQ(
      diagnostics.m_gyro_normalized_condition->m_value, 10.0 );
  EXPECT_EQ( diagnostics.m_gravity_velocity_rank->m_actual, 20U );
  EXPECT_DOUBLE_EQ(
      diagnostics.m_gravity_velocity_normalized_condition->m_limit, 20.0 );
  EXPECT_EQ( diagnostics.m_joint_fit->m_effective_dof, 22 );
  EXPECT_FALSE( isValidInitializationDiagnostics( diagnostics ) );

  InitializationDiagnostics static_with_moving_summary;
  static_with_moving_summary.m_path =
      phad::estimator::InitializationPath::kStatic;
  static_with_moving_summary.m_phase =
      phad::estimator::InitializationPhase::kCollectingEvidence;
  static_with_moving_summary.m_reason =
      phad::estimator::InitializationReason::kStaticEvidenceInsufficient;
  static_with_moving_summary.m_outcome =
      phad::estimator::InitializationOutcome::kPending;
  static_with_moving_summary.m_connected_poses.m_required = 6U;
  static_with_moving_summary.m_transaction =
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted;
  EXPECT_FALSE(
      isValidInitializationDiagnostics( static_with_moving_summary ) );

  InitializationDiagnostics moving_preattempt;
  moving_preattempt.m_path =
      phad::estimator::InitializationPath::kMoving;
  moving_preattempt.m_phase =
      phad::estimator::InitializationPhase::kCollectingEvidence;
  moving_preattempt.m_reason =
      phad::estimator::InitializationReason::kPoseCountInsufficient;
  moving_preattempt.m_outcome =
      phad::estimator::InitializationOutcome::kPending;
  moving_preattempt.m_connected_poses.m_required = 6U;
  moving_preattempt.m_duration.m_required_ns     = 250'000'000;
  moving_preattempt.m_connected_landmarks.m_required = 10U;
  moving_preattempt.m_joint_fit = InitializationFitSummary{
    .m_statistic = 1.0, .m_limit = 2.0, .m_effective_dof = 1 };
  moving_preattempt.m_transaction =
      phad::estimator::InitializationTransactionResult::kEvidenceCommitted;
  EXPECT_FALSE( isValidInitializationDiagnostics( moving_preattempt ) );

  moving_preattempt.m_joint_fit.reset();
  moving_preattempt.m_visual_fit = InitializationFitSummary{
    .m_statistic = std::numeric_limits<double>::quiet_NaN(),
    .m_limit = 11.344866730144373,
    .m_effective_dof = 3 };
  EXPECT_FALSE( isValidInitializationDiagnostics( moving_preattempt ) );
}

TEST( EstimatorTypes, InitializationPendingAndRecoverableAreNotEstimates )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationReason;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::UpdateStatus;
  using phad::estimator::internal::finalizeInitializationResult;

  InitializationDiagnostics pending;
  pending.m_path          = InitializationPath::kMoving;
  pending.m_phase         = InitializationPhase::kCollectingEvidence;
  pending.m_reason        = InitializationReason::kPoseCountInsufficient;
  pending.m_outcome       = InitializationOutcome::kPending;
  pending.m_current_graph = InitializationGraphResult::kNotEvaluated;
  pending.m_transaction =
      InitializationTransactionResult::kEvidenceCommitted;
  const auto pending_result = finalizeInitializationResult(
      pending, std::nullopt, "collecting moving evidence" );
  EXPECT_EQ( pending_result.status, UpdateStatus::kInitializing );
  EXPECT_NE( pending_result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( pending_result.estimate.has_value() );

  InitializationDiagnostics rejected = pending;
  rejected.m_phase                   = InitializationPhase::kGyroAlignment;
  rejected.m_reason                  = InitializationReason::kRankDeficient;
  rejected.m_outcome                 = InitializationOutcome::kRecoverableRejected;
  rejected.m_attempt_id              = 1U;
  const auto rejected_result         = finalizeInitializationResult(
      rejected, std::nullopt, "moving candidate rejected" );
  EXPECT_EQ( rejected_result.status, UpdateStatus::kInitializing );
  EXPECT_NE( rejected_result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( rejected_result.estimate.has_value() );
}

TEST( EstimatorTypes, NoConvergenceRejectsAtJointConsistencyPhase )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationReason;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::internal::isValidInitializationDiagnostics;

  InitializationDiagnostics diagnostics;
  diagnostics.m_path       = InitializationPath::kMoving;
  diagnostics.m_phase      = InitializationPhase::kJointConsistency;
  diagnostics.m_reason     = InitializationReason::kNoConvergence;
  diagnostics.m_outcome    = InitializationOutcome::kRecoverableRejected;
  diagnostics.m_attempt_id = 1U;
  diagnostics.m_current_graph = InitializationGraphResult::kNotEvaluated;
  diagnostics.m_transaction =
      InitializationTransactionResult::kEvidenceCommitted;

  EXPECT_TRUE( isValidInitializationDiagnostics( diagnostics ) );
  diagnostics.m_phase = InitializationPhase::kGravityVelocitySolve;
  EXPECT_FALSE( isValidInitializationDiagnostics( diagnostics ) );
}

TEST( EstimatorTypes, InitializationCommittedRequiresEstimate )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimate;
  using phad::estimator::internal::finalizeInitializationResult;

  InitializationDiagnostics committed;
  committed.m_path          = InitializationPath::kMoving;
  committed.m_phase         = InitializationPhase::kCommit;
  committed.m_outcome       = InitializationOutcome::kCommitted;
  committed.m_attempt_id    = 1U;
  committed.m_current_graph = InitializationGraphResult::kValidated;
  committed.m_transaction =
      InitializationTransactionResult::kRootCommitted;

  const VioEstimate estimate{
      .timestamp    = phad::common::Timestamp{ 250'000'000 },
      .T_W_B        = Eigen::Isometry3d::Identity(),
      .m_v_W_B      = Eigen::Vector3d::Zero(),
      .m_bias       = {},
      .m_segment_id = 0,
  };
  const auto result = finalizeInitializationResult(
      committed, estimate, "moving root committed" );
  EXPECT_EQ( result.status, UpdateStatus::kOk );
  ASSERT_TRUE( result.estimate.has_value() );
  EXPECT_EQ( result.estimate->timestamp, estimate.timestamp );
  EXPECT_TRUE( result.message.empty() );

  EXPECT_THROW( (void)finalizeInitializationResult(
                    committed, std::nullopt, "missing estimate" ),
                std::invalid_argument );
}

TEST( EstimatorTypes, InitializationFatalIsFailedWithoutEstimate )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationReason;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimate;
  using phad::estimator::internal::finalizeInitializationResult;

  InitializationDiagnostics fatal;
  fatal.m_path          = InitializationPath::kMoving;
  fatal.m_phase         = InitializationPhase::kJointConsistency;
  fatal.m_reason        = InitializationReason::kContractError;
  fatal.m_outcome       = InitializationOutcome::kFatalError;
  fatal.m_attempt_id    = 1U;
  fatal.m_current_graph = InitializationGraphResult::kNotEvaluated;
  fatal.m_transaction   = InitializationTransactionResult::kRolledBack;

  const auto result = finalizeInitializationResult(
      fatal, std::nullopt, "moving initialization contract error" );
  EXPECT_EQ( result.status, UpdateStatus::kFailed );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.message, "moving initialization contract error" );

  const VioEstimate estimate{
      .timestamp    = phad::common::Timestamp{ 250'000'000 },
      .T_W_B        = Eigen::Isometry3d::Identity(),
      .m_v_W_B      = Eigen::Vector3d::Zero(),
      .m_bias       = {},
      .m_segment_id = 0,
  };
  EXPECT_THROW(
      (void)finalizeInitializationResult( fatal, estimate, "invalid" ),
      std::invalid_argument );
}

TEST( EstimatorTypes, InitializationFinalizerNormalizesReusedEnvelope )
{
  using phad::estimator::InitializationDiagnostics;
  using phad::estimator::InitializationGraphResult;
  using phad::estimator::InitializationOutcome;
  using phad::estimator::InitializationPath;
  using phad::estimator::InitializationPhase;
  using phad::estimator::InitializationReason;
  using phad::estimator::InitializationTransactionResult;
  using phad::estimator::VioEstimate;
  using phad::estimator::VioUpdateResult;
  using phad::estimator::internal::finalizeInitializationResult;

  const VioEstimate estimate{
      .timestamp    = phad::common::Timestamp{ 250'000'000 },
      .T_W_B        = Eigen::Isometry3d::Identity(),
      .m_v_W_B      = Eigen::Vector3d::Zero(),
      .m_bias       = {},
      .m_segment_id = 0,
  };

  InitializationDiagnostics pending;
  pending.m_path          = InitializationPath::kMoving;
  pending.m_phase         = InitializationPhase::kCollectingEvidence;
  pending.m_reason        = InitializationReason::kPoseCountInsufficient;
  pending.m_outcome       = InitializationOutcome::kPending;
  pending.m_current_graph = InitializationGraphResult::kNotEvaluated;
  pending.m_transaction =
      InitializationTransactionResult::kEvidenceCommitted;
  VioUpdateResult reused_pending;
  reused_pending.estimate       = estimate;
  reused_pending.message        = "stale";
  const auto normalized_pending = finalizeInitializationResult(
      std::move( reused_pending ), pending, std::nullopt, "collecting" );
  EXPECT_FALSE( normalized_pending.estimate.has_value() );
  EXPECT_EQ( normalized_pending.message, "collecting" );

  InitializationDiagnostics committed;
  committed.m_path          = InitializationPath::kMoving;
  committed.m_phase         = InitializationPhase::kCommit;
  committed.m_outcome       = InitializationOutcome::kCommitted;
  committed.m_attempt_id    = 1U;
  committed.m_current_graph = InitializationGraphResult::kValidated;
  committed.m_transaction =
      InitializationTransactionResult::kRootCommitted;
  VioUpdateResult reused_committed;
  reused_committed.message        = "stale";
  const auto normalized_committed = finalizeInitializationResult(
      std::move( reused_committed ), committed, estimate, "ignored" );
  EXPECT_TRUE( normalized_committed.message.empty() );
}

TEST( EstimatorTypes, VioMeasurementConstructs )
{
  phad::estimator::VioMeasurement measurement{
      .m_timestamp    = phad::common::Timestamp{ 1'000'000'000 },
      .m_observations = {},
      .m_imu          = phad::test_support::stationaryImuPayload(
          phad::common::Timestamp{ 1'000'000'000 } ) };
  measurement.m_observations.push_back( phad::estimator::StereoObservation{
      .id           = 7,
      .left_pixel   = Eigen::Vector2d( 320.5, 240.0 ),
      .disparity_px = 12.0,
  } );

  ASSERT_EQ( measurement.m_observations.size(), 1U );
  EXPECT_EQ( measurement.m_observations.front().id, 7U );
  EXPECT_DOUBLE_EQ( measurement.m_observations.front().disparity_px, 12.0 );

  phad::estimator::VioUpdateResult result;
  result.status   = phad::estimator::UpdateStatus::kOk;
  result.estimate = phad::estimator::VioEstimate{
      .timestamp    = measurement.m_timestamp,
      .T_W_B        = Eigen::Isometry3d::Identity(),
      .m_v_W_B      = Eigen::Vector3d::Zero(),
      .m_bias       = {},
      .m_segment_id = 0,
  };
  EXPECT_TRUE( result.estimate.has_value() );
  EXPECT_TRUE( result.message.empty() );
}

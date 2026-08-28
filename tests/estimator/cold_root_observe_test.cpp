#include <gtest/gtest.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <limits>
#include <string_view>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/types.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "tests/estimator/vio_test_utils.hpp"

namespace
{

  using phad::common::Timestamp;
  using phad::estimator::ColdRootBootstrapPath;
  using phad::estimator::coldRootBootstrapPathToken;
  using phad::estimator::ColdRootGateState;
  using phad::estimator::coldRootGateStateToken;
  using phad::estimator::ColdRootGeometryResult;
  using phad::estimator::coldRootGeometryResultToken;
  using phad::estimator::ColdRootObserveDiagnostics;
  using phad::estimator::ColdRootPhase;
  using phad::estimator::coldRootPhaseToken;
  using phad::estimator::ColdRootReason;
  using phad::estimator::coldRootReasonToken;
  using phad::estimator::ColdRootSeedInputOrigin;
  using phad::estimator::coldRootSeedInputOriginToken;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::estimator::VioUpdateResult;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::RawImuInterval;

  [[nodiscard]] phad::camera::RectifiedStereoCalibration makeCalibration()
  {
    auto transform = phad::sensor::RigidTransform::create(
                         Eigen::Matrix4d::Identity() )
                         .value();
    return phad::camera::RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( transform ) )
        .value();
  }

  [[nodiscard]] std::vector<StereoObservation> makeObservations(
      std::size_t count = 10U, LandmarkId first_id = 1U )
  {
    std::vector<StereoObservation> observations;
    observations.reserve( count );
    for ( std::size_t index = 0U; index < count; ++index )
    {
      observations.push_back( StereoObservation{
          .id = first_id + static_cast<LandmarkId>( index ),
          .left_pixel =
              Eigen::Vector2d( 280.0 + 7.0 * static_cast<double>( index ),
                               220.0 ),
          .disparity_px = 12.0 } );
    }
    return observations;
  }

  [[nodiscard]] VioMeasurement makeMeasurement(
      std::int64_t                   timestamp_ns,
      std::vector<StereoObservation> observations = makeObservations() )
  {
    return VioMeasurement{
        .m_timestamp    = Timestamp{ timestamp_ns },
        .m_observations = std::move( observations ),
        .m_imu          = phad::test_support::stationaryImuPayload(
            Timestamp{ 0 }, Timestamp{ timestamp_ns } ) };
  }

  [[nodiscard]] ImuMeasurement imuWithAccelerationZ(
      std::int64_t timestamp_ns, double acceleration_z_mps2 )
  {
    return ImuMeasurement{
        .timestamp = Timestamp{ timestamp_ns },
        .accel_mps2 =
            std::array<double, 3>{ 0.0, 0.0, acceleration_z_mps2 },
        .gyro_radps = std::array<double, 3>{ 0.0, 0.0, 0.0 } };
  }

  [[nodiscard]] VioMeasurement makeMeasurementWithRaw(
      std::int64_t timestamp_ns, RawImuInterval raw,
      std::vector<StereoObservation> observations )
  {
    return VioMeasurement{ .m_timestamp    = Timestamp{ timestamp_ns },
                           .m_observations = std::move( observations ),
                           .m_imu          = std::move( raw ) };
  }

  [[nodiscard]] VioMeasurement makeStationaryMeasurement(
      std::int64_t begin_ns, std::int64_t end_ns,
      std::vector<StereoObservation> observations = makeObservations() )
  {
    return VioMeasurement{
        .m_timestamp    = Timestamp{ end_ns },
        .m_observations = std::move( observations ),
        .m_imu          = phad::test_support::stationaryImuPayload(
            Timestamp{ begin_ns }, Timestamp{ end_ns } ) };
  }

  void expectColdRootNotEvaluated(
      const ColdRootObserveDiagnostics& diagnostics )
  {
    EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_bootstrap_path,
               ColdRootBootstrapPath::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_seed_input_origin,
               ColdRootSeedInputOrigin::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_geometry_result,
               ColdRootGeometryResult::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_bootstrap_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_keyframe_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_stereo_population_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_root_geometry_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_imu_excitation_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_conditioning_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_initialization_solve_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_current_graph_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_EQ( diagnostics.m_commit_gate,
               ColdRootGateState::kNotEvaluated );
    EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_sample_count.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_min_samples.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_duration_ns.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_min_duration_ns.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_acc_std_max_mps2.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_acc_std_limit_mps2.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_gyr_std_max_radps.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_gyr_std_limit_radps.has_value() );
    EXPECT_FALSE(
        diagnostics.m_bootstrap_acc_norm_error_mps2.has_value() );
    EXPECT_FALSE(
        diagnostics.m_bootstrap_acc_norm_tolerance_mps2.has_value() );
    EXPECT_FALSE( diagnostics.m_bootstrap_timeout_ns.has_value() );
    EXPECT_FALSE( diagnostics.m_moving_bootstrap_enabled.has_value() );
    EXPECT_FALSE( diagnostics.m_moving_suffix_sample_count.has_value() );
    EXPECT_FALSE( diagnostics.m_moving_suffix_duration_ns.has_value() );
    EXPECT_FALSE( diagnostics.m_moving_acc_mean_norm_mps2.has_value() );
    EXPECT_FALSE(
        diagnostics.m_moving_acc_mean_norm_min_mps2.has_value() );
    EXPECT_FALSE(
        diagnostics.m_current_positive_disparity_count.has_value() );
    EXPECT_FALSE( diagnostics.m_accumulated_seed_enabled.has_value() );
    EXPECT_FALSE( diagnostics.m_pending_unique_seed_count.has_value() );
    EXPECT_FALSE( diagnostics.m_effective_seed_count.has_value() );
    EXPECT_FALSE( diagnostics.m_min_seed_observations.has_value() );
    EXPECT_FALSE(
        diagnostics.m_geometry_accepted_landmarks.has_value() );
    EXPECT_FALSE( diagnostics.m_geometry_min_landmarks.has_value() );
  }

  void expectSameLegacyResult( const phad::estimator::VioUpdateResult& lhs,
                               const phad::estimator::VioUpdateResult& rhs )
  {
    EXPECT_EQ( lhs.status, rhs.status );
    EXPECT_EQ( lhs.message, rhs.message );
    ASSERT_EQ( lhs.estimate.has_value(), rhs.estimate.has_value() );
    if ( lhs.estimate.has_value() )
    {
      EXPECT_EQ( lhs.estimate->timestamp, rhs.estimate->timestamp );
      EXPECT_TRUE( lhs.estimate->T_W_B.matrix().isApprox(
          rhs.estimate->T_W_B.matrix(), 1e-12 ) );
      EXPECT_TRUE( lhs.estimate->m_v_W_B.isApprox(
          rhs.estimate->m_v_W_B, 1e-12 ) );
      EXPECT_TRUE( lhs.estimate->m_bias.m_acc_mps2.isApprox(
          rhs.estimate->m_bias.m_acc_mps2, 1e-12 ) );
      EXPECT_TRUE( lhs.estimate->m_bias.m_gyr_radps.isApprox(
          rhs.estimate->m_bias.m_gyr_radps, 1e-12 ) );
      EXPECT_EQ( lhs.estimate->m_segment_id,
                 rhs.estimate->m_segment_id );
    }

    const auto& left  = lhs.diagnostics;
    const auto& right = rhs.diagnostics;
    EXPECT_EQ( left.num_observations, right.num_observations );
    EXPECT_EQ( left.num_retained_observations,
               right.num_retained_observations );
    EXPECT_EQ( left.num_seeded_landmarks, right.num_seeded_landmarks );
    EXPECT_EQ( left.num_current_visual_factors,
               right.num_current_visual_factors );
    EXPECT_EQ( left.num_current_mono_visual_factors,
               right.num_current_mono_visual_factors );
    EXPECT_EQ( left.unsupported_span_ns, right.unsupported_span_ns );
    EXPECT_EQ( left.num_landmarks, right.num_landmarks );
    EXPECT_EQ( left.num_shared, right.num_shared );
    EXPECT_EQ( left.num_mapped_observations,
               right.num_mapped_observations );
    EXPECT_EQ( left.num_disparity, right.num_disparity );
    EXPECT_EQ( left.num_cheirality, right.num_cheirality );
    EXPECT_EQ( left.lm_iterations, right.lm_iterations );
    EXPECT_EQ( left.window_size, right.window_size );
    EXPECT_EQ( left.segment_id, right.segment_id );
    EXPECT_EQ( left.prior_key, right.prior_key );
    EXPECT_DOUBLE_EQ( left.reproj_rms_before_px,
                      right.reproj_rms_before_px );
    EXPECT_DOUBLE_EQ( left.reproj_rms_after_px,
                      right.reproj_rms_after_px );
    EXPECT_DOUBLE_EQ( left.max_window_pose_shift_m,
                      right.max_window_pose_shift_m );
    EXPECT_EQ( left.low_connectivity, right.low_connectivity );
    EXPECT_EQ( left.pnp_success, right.pnp_success );
    EXPECT_EQ( left.pnp_inliers, right.pnp_inliers );
    EXPECT_EQ( left.outliers_culled, right.outliers_culled );
    EXPECT_EQ( left.outliers_culled_unique,
               right.outliers_culled_unique );
    EXPECT_DOUBLE_EQ( left.reproj_rms_after_cull_px,
                      right.reproj_rms_after_cull_px );
    EXPECT_EQ( left.outlier_reopt, right.outlier_reopt );
    EXPECT_EQ( left.outlier_reopt_failed, right.outlier_reopt_failed );
    EXPECT_EQ( left.outlier_reopt_rounds, right.outlier_reopt_rounds );
    EXPECT_EQ( left.culled_landmark_ids, right.culled_landmark_ids );
    EXPECT_EQ( left.probe_rejected_block_n,
               right.probe_rejected_block_n );
    EXPECT_EQ( left.probe_new_lm_n, right.probe_new_lm_n );
    EXPECT_EQ( left.probe_shift_top, right.probe_shift_top );
    EXPECT_DOUBLE_EQ( left.probe_res_mean_px, right.probe_res_mean_px );
    EXPECT_DOUBLE_EQ( left.probe_res_max_px, right.probe_res_max_px );
    EXPECT_EQ( left.probe_res_max_id, right.probe_res_max_id );
    EXPECT_EQ( left.probe_detail_valid, right.probe_detail_valid );

    const auto& left_vio  = left.m_vio;
    const auto& right_vio = right.m_vio;
    EXPECT_EQ( left_vio.m_nav_states, right_vio.m_nav_states );
    EXPECT_EQ( left_vio.m_imu_factors, right_vio.m_imu_factors );
    EXPECT_EQ( left_vio.m_bias_rw_factors, right_vio.m_bias_rw_factors );
    EXPECT_EQ( left_vio.m_visual_factors, right_vio.m_visual_factors );
    EXPECT_EQ( left_vio.m_root_prior_sets, right_vio.m_root_prior_sets );
    EXPECT_EQ( left_vio.m_integration_steps,
               right_vio.m_integration_steps );
    EXPECT_EQ( left_vio.m_integrated_duration_ns,
               right_vio.m_integrated_duration_ns );
    EXPECT_EQ( left_vio.m_visual_coast_duration_ns,
               right_vio.m_visual_coast_duration_ns );
    EXPECT_EQ( left_vio.m_non_keyframe_evictions,
               right_vio.m_non_keyframe_evictions );
    EXPECT_EQ( left_vio.m_imu_reintegrations,
               right_vio.m_imu_reintegrations );
    EXPECT_TRUE( left_vio.m_acc_cov_diag.isApprox(
        right_vio.m_acc_cov_diag, 1e-12 ) );
    EXPECT_TRUE( left_vio.m_gyr_cov_diag.isApprox(
        right_vio.m_gyr_cov_diag, 1e-12 ) );
    EXPECT_TRUE( left_vio.m_integration_cov_diag.isApprox(
        right_vio.m_integration_cov_diag, 1e-12 ) );
    EXPECT_TRUE( left_vio.m_bias_rw_sigmas.isApprox(
        right_vio.m_bias_rw_sigmas, 1e-12 ) );
    EXPECT_EQ( left_vio.m_completed_segment_id,
               right_vio.m_completed_segment_id );
  }

}  // namespace

TEST( ColdRootObserveContract, DefaultsAndTokensAreStable )
{
  const ColdRootObserveDiagnostics diagnostics;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_bootstrap_path,
             ColdRootBootstrapPath::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_geometry_result,
             ColdRootGeometryResult::kNotEvaluated );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
  EXPECT_EQ( diagnostics.m_bootstrap_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_keyframe_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_root_geometry_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_imu_excitation_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_conditioning_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_initialization_solve_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_current_graph_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_commit_gate,
             ColdRootGateState::kNotEvaluated );

  EXPECT_FALSE( diagnostics.m_bootstrap_sample_count.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_min_samples.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_duration_ns.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_min_duration_ns.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_acc_std_max_mps2.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_acc_std_limit_mps2.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_gyr_std_max_radps.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_gyr_std_limit_radps.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_acc_norm_error_mps2.has_value() );
  EXPECT_FALSE(
      diagnostics.m_bootstrap_acc_norm_tolerance_mps2.has_value() );
  EXPECT_FALSE( diagnostics.m_bootstrap_timeout_ns.has_value() );
  EXPECT_FALSE( diagnostics.m_moving_bootstrap_enabled.has_value() );
  EXPECT_FALSE( diagnostics.m_moving_suffix_sample_count.has_value() );
  EXPECT_FALSE( diagnostics.m_moving_suffix_duration_ns.has_value() );
  EXPECT_FALSE( diagnostics.m_moving_acc_mean_norm_mps2.has_value() );
  EXPECT_FALSE( diagnostics.m_moving_acc_mean_norm_min_mps2.has_value() );
  EXPECT_FALSE(
      diagnostics.m_current_positive_disparity_count.has_value() );
  EXPECT_FALSE( diagnostics.m_accumulated_seed_enabled.has_value() );
  EXPECT_FALSE( diagnostics.m_pending_unique_seed_count.has_value() );
  EXPECT_FALSE( diagnostics.m_effective_seed_count.has_value() );
  EXPECT_FALSE( diagnostics.m_min_seed_observations.has_value() );
  EXPECT_FALSE( diagnostics.m_geometry_accepted_landmarks.has_value() );
  EXPECT_FALSE( diagnostics.m_geometry_min_landmarks.has_value() );

  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kNotEvaluated ),
             std::string_view{ "not_evaluated" } );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kBootstrap ), "bootstrap" );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kKeyframe ), "keyframe" );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kStereoPopulation ),
             "stereo_population" );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kRootGeometry ),
             "root_geometry" );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kCurrentGraph ),
             "current_graph" );
  EXPECT_EQ( coldRootPhaseToken( ColdRootPhase::kCommit ), "commit" );

  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kNotEvaluated ),
             "not_evaluated" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kEvidenceInsufficient ),
             "evidence_insufficient" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kTimedOut ), "timed_out" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kKeyframeRequired ),
             "keyframe_required" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kPopulationInsufficient ),
             "population_insufficient" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kPopulationAccumulating ),
             "population_accumulating" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kGeometryRejected ),
             "geometry_rejected" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kGraphBuildFailed ),
             "graph_build_failed" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kGraphSolveFailed ),
             "graph_solve_failed" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kGraphValidationFailed ),
             "graph_validation_failed" );
  EXPECT_EQ( coldRootReasonToken( ColdRootReason::kCommitted ), "committed" );

  EXPECT_EQ( coldRootGateStateToken( ColdRootGateState::kNotEvaluated ),
             "not_evaluated" );
  EXPECT_EQ( coldRootGateStateToken( ColdRootGateState::kPassed ), "passed" );
  EXPECT_EQ( coldRootGateStateToken( ColdRootGateState::kFailed ), "failed" );

  EXPECT_EQ(
      coldRootBootstrapPathToken( ColdRootBootstrapPath::kNotEvaluated ),
      "not_evaluated" );
  EXPECT_EQ( coldRootBootstrapPathToken( ColdRootBootstrapPath::kCollecting ),
             "collecting" );
  EXPECT_EQ( coldRootBootstrapPathToken( ColdRootBootstrapPath::kStatic ),
             "static" );
  EXPECT_EQ( coldRootBootstrapPathToken( ColdRootBootstrapPath::kMoving ),
             "moving" );

  EXPECT_EQ(
      coldRootSeedInputOriginToken( ColdRootSeedInputOrigin::kNotEvaluated ),
      "not_evaluated" );
  EXPECT_EQ(
      coldRootSeedInputOriginToken( ColdRootSeedInputOrigin::kCurrentPacket ),
      "current_packet" );
  EXPECT_EQ(
      coldRootSeedInputOriginToken( ColdRootSeedInputOrigin::kAccumulated ),
      "accumulated" );

  EXPECT_EQ(
      coldRootGeometryResultToken( ColdRootGeometryResult::kNotEvaluated ),
      "not_evaluated" );
  EXPECT_EQ( coldRootGeometryResultToken( ColdRootGeometryResult::kAccepted ),
             "accepted" );
  EXPECT_EQ( coldRootGeometryResultToken(
                 ColdRootGeometryResult::kNonfiniteBackprojection ),
             "nonfinite_backprojection" );
  EXPECT_EQ(
      coldRootGeometryResultToken( ColdRootGeometryResult::kBehindCamera ),
      "behind_camera" );
  EXPECT_EQ( coldRootGeometryResultToken(
                 ColdRootGeometryResult::kEmptyAfterFilter ),
             "empty_after_filter" );
}

TEST( ColdRootObserveBootstrap,
      CollectingPublishesCurrentMeasurementAndStaticPredicate )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update( makeMeasurement( 10'000'000 ) );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_EQ( result.message, "collecting static bootstrap evidence" );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_observations, 10U );
  EXPECT_EQ( result.diagnostics.num_disparity, 0U );
  EXPECT_EQ( result.diagnostics.num_shared, 0U );
  EXPECT_EQ( result.diagnostics.window_size, 0U );
  EXPECT_EQ( result.diagnostics.segment_id, 0U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kBootstrap );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kEvidenceInsufficient );
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kCollecting );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_keyframe_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_root_geometry_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_current_graph_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_commit_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );

  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 10U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, false );
  EXPECT_EQ( diagnostics.m_bootstrap_sample_count, 3U );
  EXPECT_EQ( diagnostics.m_bootstrap_min_samples,
             options.m_bootstrap_min_samples );
  EXPECT_EQ( diagnostics.m_bootstrap_duration_ns, 10'000'000 );
  EXPECT_EQ( diagnostics.m_bootstrap_min_duration_ns,
             options.m_bootstrap_min_duration_ns );
  EXPECT_EQ( diagnostics.m_bootstrap_acc_std_max_mps2, 0.0 );
  EXPECT_EQ( diagnostics.m_bootstrap_acc_std_limit_mps2,
             options.m_bootstrap_max_acc_std_mps2 );
  EXPECT_EQ( diagnostics.m_bootstrap_gyr_std_max_radps, 0.0 );
  EXPECT_EQ( diagnostics.m_bootstrap_gyr_std_limit_radps,
             options.m_bootstrap_max_gyr_std_radps );
  EXPECT_EQ( diagnostics.m_bootstrap_acc_norm_error_mps2, 0.0 );
  EXPECT_EQ( diagnostics.m_bootstrap_acc_norm_tolerance_mps2,
             options.m_bootstrap_acc_norm_tol_mps2 );
  EXPECT_EQ( diagnostics.m_bootstrap_timeout_ns,
             options.m_bootstrap_timeout_ns );
  EXPECT_EQ( diagnostics.m_moving_bootstrap_enabled, false );
  EXPECT_FALSE( diagnostics.m_moving_suffix_sample_count.has_value() );
  EXPECT_FALSE( diagnostics.m_effective_seed_count.has_value() );
}

TEST( ColdRootObserveBootstrap, TimeoutIsAResultLevelBootstrapFailure )
{
  EstimatorOptions options;
  options.enable_pnp_init             = false;
  options.m_bootstrap_min_duration_ns = 20'000'000;
  options.m_bootstrap_timeout_ns      = 20'000'000;
  options.m_bootstrap_min_samples     = 4U;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update( makeMeasurement( 20'000'000 ) );

  EXPECT_EQ( result.status, UpdateStatus::kFailed );
  EXPECT_EQ( result.message, "static bootstrap timed out" );
  EXPECT_FALSE( result.estimate.has_value() );
  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kBootstrap );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kTimedOut );
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kCollecting );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 10U );
  EXPECT_EQ( diagnostics.m_bootstrap_sample_count, 3U );
  EXPECT_EQ( diagnostics.m_bootstrap_min_samples, 4U );
  EXPECT_EQ( diagnostics.m_bootstrap_duration_ns, 20'000'000 );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
}

TEST( ColdRootObservePopulation,
      EmptyVisualInputRunsPopulationGateBeforeKeyframeGate )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update(
      makeMeasurement( 50'000'000, std::vector<StereoObservation>{} ) );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_EQ( result.message,
             "static bootstrap ready; waiting for visual seed" );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_observations, 0U );
  EXPECT_EQ( result.diagnostics.num_disparity, 0U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kStereoPopulation );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kPopulationInsufficient );
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kStatic );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kCurrentPacket );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_keyframe_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 0U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, false );
  EXPECT_EQ( diagnostics.m_effective_seed_count, 0U );
  EXPECT_EQ( diagnostics.m_min_seed_observations, 10U );
  EXPECT_FALSE( diagnostics.m_pending_unique_seed_count.has_value() );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
}

TEST( ColdRootObservePopulation,
      NonKeyframeStopsBeforeStereoPopulationGate )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update( makeMeasurement( 50'000'000 ), false );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_EQ( result.message,
             "static bootstrap ready; waiting for a keyframe visual seed" );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 10U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kKeyframe );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kKeyframeRequired );
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kStatic );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_keyframe_gate, ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 10U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, false );
  EXPECT_FALSE( diagnostics.m_effective_seed_count.has_value() );
  EXPECT_FALSE( diagnostics.m_min_seed_observations.has_value() );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
}

TEST( ColdRootObservePopulation,
      CurrentPacketNineOfTenIsPopulationInsufficient )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update(
      makeMeasurement( 50'000'000, makeObservations( 9U ) ) );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_EQ( result.message,
             "insufficient observations to seed first segment" );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 9U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kStereoPopulation );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kPopulationInsufficient );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_keyframe_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kCurrentPacket );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 9U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, false );
  EXPECT_FALSE( diagnostics.m_pending_unique_seed_count.has_value() );
  EXPECT_EQ( diagnostics.m_effective_seed_count, 9U );
  EXPECT_EQ( diagnostics.m_min_seed_observations, 10U );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
}

TEST( ColdRootObservePopulation,
      AccumulatedSeedKeepsCurrentPendingAndEffectiveCountsDistinct )
{
  EstimatorOptions options;
  options.enable_pnp_init         = false;
  options.enable_accumulated_seed = true;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update(
      makeMeasurement( 50'000'000, makeObservations( 4U ) ) );

  EXPECT_EQ( result.status, UpdateStatus::kInitializing );
  EXPECT_EQ( result.message,
             "accumulating seed observations (first segment)" );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_disparity, 4U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kStereoPopulation );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kPopulationAccumulating );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_keyframe_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kFailed );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kAccumulated );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 4U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, true );
  EXPECT_EQ( diagnostics.m_pending_unique_seed_count, 4U );
  EXPECT_EQ( diagnostics.m_effective_seed_count, 4U );
  EXPECT_EQ( diagnostics.m_min_seed_observations, 10U );
  EXPECT_FALSE( diagnostics.m_attempt_id.has_value() );
}

TEST( ColdRootObserveBootstrap,
      MovingPathKeepsFullStaticAndRecentSuffixPredicatesDistinct )
{
  EstimatorOptions options;
  options.enable_pnp_init             = false;
  options.m_enable_moving_bootstrap   = true;
  options.m_bootstrap_min_duration_ns = 20'000'000;
  options.m_bootstrap_min_samples     = 3U;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto first = estimator.update( makeMeasurementWithRaw(
      10'000'000,
      RawImuInterval{
          .m_t_begin = Timestamp{ 0 },
          .m_t_end   = Timestamp{ 10'000'000 },
          .m_samples = { imuWithAccelerationZ( 0, 1.0 ),
                         imuWithAccelerationZ( 5'000'000, 20.0 ),
                         imuWithAccelerationZ( 10'000'000, 9.81 ) } },
      makeObservations( 9U ) ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing );
  ASSERT_EQ( first.message, "collecting static bootstrap evidence" );

  const auto second = estimator.update( makeMeasurementWithRaw(
      30'000'000,
      RawImuInterval{
          .m_t_begin = Timestamp{ 10'000'000 },
          .m_t_end   = Timestamp{ 30'000'000 },
          .m_samples = { imuWithAccelerationZ( 10'000'000, 9.81 ),
                         imuWithAccelerationZ( 20'000'000, 9.81 ),
                         imuWithAccelerationZ( 30'000'000, 9.81 ) } },
      makeObservations( 9U ) ) );

  EXPECT_EQ( second.status, UpdateStatus::kInitializing );
  EXPECT_EQ( second.message,
             "insufficient observations to seed first segment" );
  const ColdRootObserveDiagnostics& diagnostics =
      second.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kMoving );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_moving_bootstrap_enabled, true );
  EXPECT_EQ( diagnostics.m_bootstrap_sample_count, 5U );
  ASSERT_TRUE( diagnostics.m_bootstrap_acc_std_max_mps2.has_value() );
  EXPECT_GT( *diagnostics.m_bootstrap_acc_std_max_mps2,
             options.m_bootstrap_max_acc_std_mps2 );
  EXPECT_EQ( diagnostics.m_moving_suffix_sample_count, 3U );
  EXPECT_EQ( diagnostics.m_moving_suffix_duration_ns, 20'000'000 );
  EXPECT_EQ( diagnostics.m_moving_acc_mean_norm_mps2, 9.81 );
  EXPECT_EQ( diagnostics.m_moving_acc_mean_norm_min_mps2,
             std::numeric_limits<double>::epsilon() );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 9U );
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kStereoPopulation );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kPopulationInsufficient );
}

TEST( ColdRootObserveEntry,
      ActiveDiscontinuityAndInvalidInputAreNotColdRootEvaluations )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator active_estimator(
      makeCalibration(), phad::test_support::testImuParameters(), options );
  const auto root = active_estimator.update(
      makeStationaryMeasurement( 0, 50'000'000 ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  const auto active = active_estimator.update(
      makeStationaryMeasurement( 50'000'000, 100'000'000 ) );
  ASSERT_EQ( active.status, UpdateStatus::kOk ) << active.message;
  expectColdRootNotEvaluated( active.diagnostics.m_cold_root );

  VioEstimator discontinuity_estimator(
      makeCalibration(), phad::test_support::testImuParameters(), options );
  const auto discontinuity = discontinuity_estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ 50'000'000 },
      .m_observations = makeObservations(),
      .m_imu          = phad::sensor::MeasurementDiscontinuity{
                   .m_t_begin = Timestamp{ 0 }, .m_t_end = Timestamp{ 50'000'000 } } } );
  ASSERT_EQ( discontinuity.status, UpdateStatus::kInitializing );
  expectColdRootNotEvaluated( discontinuity.diagnostics.m_cold_root );

  VioEstimator invalid_estimator(
      makeCalibration(), phad::test_support::testImuParameters(), options );
  auto invalid_observations                 = makeObservations();
  invalid_observations.front().disparity_px = -1.0;
  const auto invalid                        = invalid_estimator.update( makeStationaryMeasurement(
      0, 50'000'000, std::move( invalid_observations ) ) );
  ASSERT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  expectColdRootNotEvaluated( invalid.diagnostics.m_cold_root );
}

TEST( ColdRootObserveEntry,
      BootstrapProvenanceFailurePublishesNoColdRootMeasurementSummary )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );
  const auto   first = estimator.update( makeMeasurement( 10'000'000 ) );
  ASSERT_EQ( first.status, UpdateStatus::kInitializing );

  const auto invalid = estimator.update( makeMeasurementWithRaw(
      20'000'000,
      RawImuInterval{
          .m_t_begin = Timestamp{ 10'000'000 },
          .m_t_end   = Timestamp{ 20'000'000 },
          .m_samples = { imuWithAccelerationZ( 10'000'000, 10.0 ),
                         imuWithAccelerationZ( 15'000'000, 9.81 ),
                         imuWithAccelerationZ( 20'000'000, 9.81 ) } },
      makeObservations() ) );

  ASSERT_EQ( invalid.status, UpdateStatus::kInvalidInput );
  EXPECT_EQ( invalid.message,
             "shared bootstrap endpoint has inconsistent IMU values" );
  expectColdRootNotEvaluated( invalid.diagnostics.m_cold_root );
}

TEST( ColdRootObserveAttempt,
      GeometryRollbackConsumesIdAndNextCommitUsesTheNextId )
{
  EstimatorOptions options;
  options.enable_pnp_init         = false;
  options.enable_accumulated_seed = true;
  options.min_seed_observations   = 10;
  VioEstimator subject( makeCalibration(),
                        phad::test_support::testImuParameters(), options );
  VioEstimator control( makeCalibration(),
                        phad::test_support::testImuParameters(), options );

  const auto first_observations = makeObservations( 5U, 1U );
  const auto subject_first      = subject.update( makeStationaryMeasurement(
      0, 50'000'000, first_observations ) );
  const auto control_first      = control.update( makeStationaryMeasurement(
      0, 50'000'000, first_observations ) );
  ASSERT_EQ( subject_first.status, UpdateStatus::kInitializing );
  ASSERT_EQ( control_first.status, UpdateStatus::kInitializing );
  ASSERT_FALSE(
      subject_first.diagnostics.m_cold_root.m_attempt_id.has_value() );

  auto rejected_observations = makeObservations( 5U, 101U );
  rejected_observations.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  const auto rejected = subject.update( makeStationaryMeasurement(
      50'000'000, 100'000'000, rejected_observations ) );
  ASSERT_EQ( rejected.status, UpdateStatus::kRejected ) << rejected.message;
  EXPECT_EQ( rejected.message,
             "failed to backproject landmark on first frame" );
  EXPECT_FALSE( rejected.estimate.has_value() );

  const ColdRootObserveDiagnostics& rejected_diagnostics =
      rejected.diagnostics.m_cold_root;
  EXPECT_EQ( rejected_diagnostics.m_phase, ColdRootPhase::kRootGeometry );
  EXPECT_EQ( rejected_diagnostics.m_reason,
             ColdRootReason::kGeometryRejected );
  EXPECT_EQ( rejected_diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kAccumulated );
  EXPECT_EQ( rejected_diagnostics.m_bootstrap_gate,
             ColdRootGateState::kPassed );
  EXPECT_EQ( rejected_diagnostics.m_keyframe_gate,
             ColdRootGateState::kPassed );
  EXPECT_EQ( rejected_diagnostics.m_stereo_population_gate,
             ColdRootGateState::kPassed );
  EXPECT_EQ( rejected_diagnostics.m_root_geometry_gate,
             ColdRootGateState::kFailed );
  EXPECT_EQ( rejected_diagnostics.m_current_graph_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( rejected_diagnostics.m_commit_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( rejected_diagnostics.m_geometry_result,
             ColdRootGeometryResult::kNonfiniteBackprojection );
  EXPECT_EQ( rejected_diagnostics.m_attempt_id, 1U );
  EXPECT_EQ( rejected_diagnostics.m_current_positive_disparity_count, 5U );
  EXPECT_EQ( rejected_diagnostics.m_pending_unique_seed_count, 10U );
  EXPECT_EQ( rejected_diagnostics.m_effective_seed_count, 10U );
  EXPECT_EQ( rejected_diagnostics.m_min_seed_observations, 10U );
  ASSERT_TRUE(
      rejected_diagnostics.m_geometry_accepted_landmarks.has_value() );
  EXPECT_LT( *rejected_diagnostics.m_geometry_accepted_landmarks, 10U );
  EXPECT_EQ( rejected_diagnostics.m_geometry_accepted_landmarks,
             rejected.diagnostics.probe_new_lm_n );
  EXPECT_EQ( rejected_diagnostics.m_geometry_min_landmarks, 1U );

  const auto legal_observations = makeObservations( 5U, 101U );
  const auto after_rejected     = subject.update( makeStationaryMeasurement(
      50'000'000, 100'000'000, legal_observations ) );
  const auto direct             = control.update( makeStationaryMeasurement(
      50'000'000, 100'000'000, legal_observations ) );
  ASSERT_EQ( after_rejected.status, UpdateStatus::kOk )
      << after_rejected.message;
  ASSERT_EQ( direct.status, UpdateStatus::kOk ) << direct.message;
  expectSameLegacyResult( after_rejected, direct );

  const ColdRootObserveDiagnostics& committed =
      after_rejected.diagnostics.m_cold_root;
  EXPECT_EQ( committed.m_phase, ColdRootPhase::kCommit );
  EXPECT_EQ( committed.m_reason, ColdRootReason::kCommitted );
  EXPECT_EQ( committed.m_attempt_id, 2U );
  EXPECT_EQ( direct.diagnostics.m_cold_root.m_attempt_id, 1U );
  EXPECT_EQ( committed.m_geometry_result, ColdRootGeometryResult::kAccepted );
  EXPECT_EQ( committed.m_geometry_accepted_landmarks, 10U );
  EXPECT_EQ( committed.m_geometry_min_landmarks, 1U );
  EXPECT_EQ( committed.m_root_geometry_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( committed.m_current_graph_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( committed.m_commit_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( committed.m_seed_input_origin,
             ColdRootSeedInputOrigin::kAccumulated );
  EXPECT_EQ( committed.m_current_positive_disparity_count, 5U );
  EXPECT_EQ( committed.m_pending_unique_seed_count, 10U );
  EXPECT_EQ( committed.m_effective_seed_count, 10U );
}

TEST( ColdRootObserveAttempt,
      CurrentPacketStaticRootPublishesCompleteCommitTrace )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto result = estimator.update(
      makeStationaryMeasurement( 0, 50'000'000 ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.estimate.has_value() );
  EXPECT_TRUE( result.message.empty() );
  EXPECT_EQ( result.diagnostics.num_disparity, 10U );

  const ColdRootObserveDiagnostics& diagnostics =
      result.diagnostics.m_cold_root;
  EXPECT_EQ( diagnostics.m_phase, ColdRootPhase::kCommit );
  EXPECT_EQ( diagnostics.m_reason, ColdRootReason::kCommitted );
  EXPECT_EQ( diagnostics.m_bootstrap_path, ColdRootBootstrapPath::kStatic );
  EXPECT_EQ( diagnostics.m_seed_input_origin,
             ColdRootSeedInputOrigin::kCurrentPacket );
  EXPECT_EQ( diagnostics.m_geometry_result,
             ColdRootGeometryResult::kAccepted );
  EXPECT_EQ( diagnostics.m_attempt_id, 1U );
  EXPECT_EQ( diagnostics.m_bootstrap_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_keyframe_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_stereo_population_gate,
             ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_root_geometry_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_imu_excitation_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_conditioning_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_initialization_solve_gate,
             ColdRootGateState::kNotEvaluated );
  EXPECT_EQ( diagnostics.m_current_graph_gate,
             ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_commit_gate, ColdRootGateState::kPassed );
  EXPECT_EQ( diagnostics.m_current_positive_disparity_count, 10U );
  EXPECT_EQ( diagnostics.m_accumulated_seed_enabled, false );
  EXPECT_FALSE( diagnostics.m_moving_bootstrap_enabled.has_value() );
  EXPECT_FALSE( diagnostics.m_pending_unique_seed_count.has_value() );
  EXPECT_EQ( diagnostics.m_effective_seed_count, 10U );
  EXPECT_EQ( diagnostics.m_min_seed_observations, 10U );
  EXPECT_EQ( diagnostics.m_geometry_accepted_landmarks, 10U );
  EXPECT_EQ( diagnostics.m_geometry_min_landmarks, 1U );
}

TEST( ColdRootObserveAttempt,
      DuplicateIdsCountInstalledUniqueLandmarksWithoutChangingLegacyEvents )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator                   estimator( makeCalibration(),
                                            phad::test_support::testImuParameters(), options );
  std::vector<StereoObservation> observations = makeObservations();
  for ( StereoObservation& observation : observations )
  {
    observation.id = 42U;
  }

  const auto result = estimator.update(
      makeStationaryMeasurement( 0, 50'000'000, observations ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  EXPECT_EQ( result.diagnostics.num_disparity, 10U );
  EXPECT_EQ( result.diagnostics.probe_new_lm_n, 10U );
  EXPECT_EQ( result.diagnostics.num_landmarks, 1U );
  EXPECT_EQ( result.diagnostics.m_cold_root.m_effective_seed_count, 10U );
  EXPECT_EQ(
      result.diagnostics.m_cold_root.m_geometry_accepted_landmarks, 1U );
}

TEST( ColdRootObserveAttempt, SegmentCompletionDoesNotResetAttemptHistory )
{
  EstimatorOptions options;
  options.enable_pnp_init = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto first_root = estimator.update(
      makeStationaryMeasurement( 0, 50'000'000 ) );
  ASSERT_EQ( first_root.status, UpdateStatus::kOk ) << first_root.message;
  ASSERT_EQ( first_root.diagnostics.m_cold_root.m_attempt_id, 1U );

  const auto discontinuity = estimator.update( VioMeasurement{
      .m_timestamp    = Timestamp{ 100'000'000 },
      .m_observations = makeObservations(),
      .m_imu          = phad::sensor::MeasurementDiscontinuity{
                   .m_t_begin = Timestamp{ 50'000'000 },
                   .m_t_end   = Timestamp{ 100'000'000 } } } );
  ASSERT_EQ( discontinuity.status, UpdateStatus::kDiscontinuity );
  expectColdRootNotEvaluated( discontinuity.diagnostics.m_cold_root );

  const auto second_root = estimator.update(
      makeStationaryMeasurement( 100'000'000, 150'000'000 ) );
  ASSERT_EQ( second_root.status, UpdateStatus::kOk )
      << second_root.message;
  EXPECT_EQ( second_root.diagnostics.segment_id, 1U );
  EXPECT_EQ( second_root.diagnostics.m_cold_root.m_phase,
             ColdRootPhase::kCommit );
  EXPECT_EQ( second_root.diagnostics.m_cold_root.m_attempt_id, 2U );
}

TEST( ColdRootObserveAttempt,
      OneInstanceAllocatesIdsOnlyForRealRootCalls )
{
  EstimatorOptions options;
  options.enable_pnp_init         = false;
  options.enable_accumulated_seed = true;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto collecting = estimator.update(
      makeStationaryMeasurement( 0, 10'000'000 ) );
  ASSERT_EQ( collecting.status, UpdateStatus::kInitializing );
  EXPECT_EQ( collecting.diagnostics.m_cold_root.m_phase,
             ColdRootPhase::kBootstrap );
  EXPECT_FALSE( collecting.diagnostics.m_cold_root.m_attempt_id.has_value() );

  const auto non_keyframe = estimator.update(
      makeStationaryMeasurement( 10'000'000, 50'000'000 ), false );
  ASSERT_EQ( non_keyframe.status, UpdateStatus::kInitializing );
  EXPECT_EQ( non_keyframe.diagnostics.m_cold_root.m_phase,
             ColdRootPhase::kKeyframe );
  EXPECT_FALSE( non_keyframe.diagnostics.m_cold_root.m_attempt_id.has_value() );

  const auto empty = estimator.update( makeStationaryMeasurement(
      50'000'000, 100'000'000, std::vector<StereoObservation>{} ) );
  ASSERT_EQ( empty.status, UpdateStatus::kInitializing );
  EXPECT_EQ( empty.diagnostics.m_cold_root.m_reason,
             ColdRootReason::kPopulationInsufficient );
  EXPECT_EQ( empty.diagnostics.m_cold_root.m_seed_input_origin,
             ColdRootSeedInputOrigin::kCurrentPacket );
  EXPECT_EQ( empty.diagnostics.m_cold_root.m_effective_seed_count, 0U );
  EXPECT_EQ( empty.diagnostics.m_cold_root.m_min_seed_observations, 10U );
  EXPECT_FALSE( empty.diagnostics.m_cold_root.m_attempt_id.has_value() );

  const auto accumulating = estimator.update( makeStationaryMeasurement(
      100'000'000, 150'000'000, makeObservations( 5U, 1U ) ) );
  ASSERT_EQ( accumulating.status, UpdateStatus::kInitializing );
  EXPECT_EQ( accumulating.diagnostics.m_cold_root.m_reason,
             ColdRootReason::kPopulationAccumulating );
  EXPECT_FALSE( accumulating.diagnostics.m_cold_root.m_attempt_id.has_value() );

  auto rejected_observations = makeObservations( 5U, 101U );
  rejected_observations.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  const auto rejected = estimator.update( makeStationaryMeasurement(
      150'000'000, 200'000'000, rejected_observations ) );
  ASSERT_EQ( rejected.status, UpdateStatus::kRejected );
  EXPECT_EQ( rejected.diagnostics.m_cold_root.m_phase,
             ColdRootPhase::kRootGeometry );
  EXPECT_EQ( rejected.diagnostics.m_cold_root.m_attempt_id, 1U );

  const auto committed = estimator.update( makeStationaryMeasurement(
      150'000'000, 200'000'000, makeObservations( 5U, 101U ) ) );
  ASSERT_EQ( committed.status, UpdateStatus::kOk ) << committed.message;
  EXPECT_EQ( committed.diagnostics.m_cold_root.m_phase,
             ColdRootPhase::kCommit );
  EXPECT_EQ( committed.diagnostics.m_cold_root.m_attempt_id, 2U );
}

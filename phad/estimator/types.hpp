#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

namespace phad::estimator
{

  using common::LandmarkId;

  struct StereoObservation
  {
    LandmarkId      id;
    Eigen::Vector2d left_pixel;  // rectified left
    // > 0: stereo disparity (depth via backproject); == 0: stereo failed —
    // no depth, kept in the window until stereo returns (Slice ⑦). < 0 is
    // invalid.
    double disparity_px;
  };

  struct VioMeasurement
  {
    common::Timestamp              m_timestamp;
    std::vector<StereoObservation> m_observations;
    sensor::ImuPayload             m_imu;
  };

  struct EstimatorOptions
  {
    int window_size               = 10;
    int min_landmark_observations = 2;
    // Minimum positive-disparity observations needed to seed a root.
    int min_seed_observations = 10;
    // Slice ⑥b: a new landmark must be observed this many frames before
    // seeding (single-frame disparity can be a SAD mismatch). 1 restores
    // the pre-⑥b behavior (tests use 1).
    int    min_track_observations_for_seed = 1;
    int    min_shared_landmarks            = 10;
    double stereo_sigma_px                 = 1.0;
    double huber_k_px                      = 3.0;  // <= 0 disables Robust wrapper
    double prior_rotation_sigma_rad        = 1e-4;
    double prior_translation_sigma_m       = 1e-4;
    bool   enable_pnp_init                 = true;
    double pnp_reproj_px                   = 2.0;
    double pnp_confidence                  = 0.99;
    int    min_pnp_inliers                 = 10;
    // enable_outlier_cull only gates mean-reproj cull; cheirality always
    // clears window observations for dropped landmarks.
    bool   enable_outlier_cull   = true;
    bool   enable_outlier_reopt  = true;  // false → 复现 b6fbcb6 只 cull
    int    max_outlier_reopts    = 3;     // ≥0；0 → 不重优；进 flattenConfig
    double outlier_avg_reproj_px = 4.0;
    // After mean-cull / cheirality erase: refuse same LandmarkId backproject.
    // false → allow rebirth (Slice ④ pseudo-permanent; A/B only).
    bool block_culled_rebirth = true;
    // Slice ⑦: a hanging landmark (kept alive only by zero-disparity
    // observations) is dropped once the body has moved this far since its
    // last stereo observation (E13: measured against the persistent
    // per-landmark last-stereo pose, not the window scan). 1.0 m is the
    // E13-composed gate (final decision: V2_02 -39% / V2_01 -15% vs
    // checkpoint, MH_03/V2_03 the smallest regressions of any variant).
    // <= 0 disables (keep all hanging landmarks — e9a21b3 behavior).
    // Bench CLI: --hanging-gate-m.
    double hanging_landmark_gate_m = 1.0;
    // Slice ⑦ E12g: refresh a far-return landmark's 3D to the current
    // backproject when its stale 3D projects this far off the observed left
    // pixel (or behind the camera); keep the stale 3D otherwise. <= 0
    // disables the refresh entirely (E13 pure-gate runs). Bench CLI:
    // --far-refresh-px.
    double far_return_refresh_px = 6.0;
    // Optionally accumulate sparse visual evidence while initializing a root.
    // CLI-only and excluded from config_hash.
    bool enable_accumulated_seed = false;
    // Session sets true when probe_b_path non-empty; NOT in flattenConfig.
    bool          enable_probe_b                = false;
    double        m_gravity_mps2                = 9.81;
    double        m_q_int                       = 0.0;
    std::int64_t  m_bootstrap_min_duration_ns   = 20'000'000;
    std::uint32_t m_bootstrap_min_samples       = 3;
    double        m_bootstrap_max_acc_std_mps2  = 0.05;
    double        m_bootstrap_max_gyr_std_radps = 0.005;
    double        m_bootstrap_acc_norm_tol_mps2 = 0.25;
    std::int64_t  m_bootstrap_timeout_ns        = 1'000'000'000;
    // Explicit moving-start fallback: use the shortest recent suffix that
    // satisfies the bootstrap duration/sample floor for gravity direction,
    // while leaving initial velocity and both biases at zero. Static
    // bootstrap remains the default and retains gyro-mean bias estimation.
    bool         m_enable_moving_bootstrap    = false;
    double       m_velocity_prior_sigma_mps   = 0.1;
    double       m_acc_bias_prior_sigma_mps2  = 0.1;
    double       m_gyr_bias_prior_sigma_radps = 0.01;
    std::int64_t m_visual_coast_horizon_ns    = 500'000'000;
  };

  enum class UpdateStatus : std::uint8_t
  {
    kOk            = 0,
    kInitializing  = 1,
    kDiscontinuity = 2,
    kVisualOutage  = 3,
    kInvalidInput  = 4,
    kRejected      = 5,
    kFailed        = 6
  };

  enum class ColdRootPhase : std::uint8_t
  {
    kNotEvaluated,
    kBootstrap,
    kKeyframe,
    kStereoPopulation,
    kRootGeometry,
    kCurrentGraph,
    kCommit
  };

  enum class ColdRootReason : std::uint8_t
  {
    kNotEvaluated,
    kEvidenceInsufficient,
    kTimedOut,
    kKeyframeRequired,
    kPopulationInsufficient,
    kPopulationAccumulating,
    kGeometryRejected,
    kGraphBuildFailed,
    kGraphSolveFailed,
    kGraphValidationFailed,
    kCommitted
  };

  enum class ColdRootGateState : std::uint8_t
  {
    kNotEvaluated,
    kPassed,
    kFailed
  };

  enum class ColdRootBootstrapPath : std::uint8_t
  {
    kNotEvaluated,
    kCollecting,
    kStatic,
    kMoving
  };

  enum class ColdRootSeedInputOrigin : std::uint8_t
  {
    kNotEvaluated,
    kCurrentPacket,
    kAccumulated
  };

  enum class ColdRootGeometryResult : std::uint8_t
  {
    kNotEvaluated,
    kAccepted,
    kNonfiniteBackprojection,
    kBehindCamera,
    kEmptyAfterFilter
  };

  [[nodiscard]] constexpr std::string_view coldRootPhaseToken(
      ColdRootPhase phase ) noexcept
  {
    switch ( phase )
    {
      case ColdRootPhase::kNotEvaluated:
        return "not_evaluated";
      case ColdRootPhase::kBootstrap:
        return "bootstrap";
      case ColdRootPhase::kKeyframe:
        return "keyframe";
      case ColdRootPhase::kStereoPopulation:
        return "stereo_population";
      case ColdRootPhase::kRootGeometry:
        return "root_geometry";
      case ColdRootPhase::kCurrentGraph:
        return "current_graph";
      case ColdRootPhase::kCommit:
        return "commit";
    }
    return "unknown";
  }

  [[nodiscard]] constexpr std::string_view coldRootReasonToken(
      ColdRootReason reason ) noexcept
  {
    switch ( reason )
    {
      case ColdRootReason::kNotEvaluated:
        return "not_evaluated";
      case ColdRootReason::kEvidenceInsufficient:
        return "evidence_insufficient";
      case ColdRootReason::kTimedOut:
        return "timed_out";
      case ColdRootReason::kKeyframeRequired:
        return "keyframe_required";
      case ColdRootReason::kPopulationInsufficient:
        return "population_insufficient";
      case ColdRootReason::kPopulationAccumulating:
        return "population_accumulating";
      case ColdRootReason::kGeometryRejected:
        return "geometry_rejected";
      case ColdRootReason::kGraphBuildFailed:
        return "graph_build_failed";
      case ColdRootReason::kGraphSolveFailed:
        return "graph_solve_failed";
      case ColdRootReason::kGraphValidationFailed:
        return "graph_validation_failed";
      case ColdRootReason::kCommitted:
        return "committed";
    }
    return "unknown";
  }

  [[nodiscard]] constexpr std::string_view coldRootGateStateToken(
      ColdRootGateState state ) noexcept
  {
    switch ( state )
    {
      case ColdRootGateState::kNotEvaluated:
        return "not_evaluated";
      case ColdRootGateState::kPassed:
        return "passed";
      case ColdRootGateState::kFailed:
        return "failed";
    }
    return "unknown";
  }

  [[nodiscard]] constexpr std::string_view coldRootBootstrapPathToken(
      ColdRootBootstrapPath path ) noexcept
  {
    switch ( path )
    {
      case ColdRootBootstrapPath::kNotEvaluated:
        return "not_evaluated";
      case ColdRootBootstrapPath::kCollecting:
        return "collecting";
      case ColdRootBootstrapPath::kStatic:
        return "static";
      case ColdRootBootstrapPath::kMoving:
        return "moving";
    }
    return "unknown";
  }

  [[nodiscard]] constexpr std::string_view coldRootSeedInputOriginToken(
      ColdRootSeedInputOrigin origin ) noexcept
  {
    switch ( origin )
    {
      case ColdRootSeedInputOrigin::kNotEvaluated:
        return "not_evaluated";
      case ColdRootSeedInputOrigin::kCurrentPacket:
        return "current_packet";
      case ColdRootSeedInputOrigin::kAccumulated:
        return "accumulated";
    }
    return "unknown";
  }

  [[nodiscard]] constexpr std::string_view coldRootGeometryResultToken(
      ColdRootGeometryResult result ) noexcept
  {
    switch ( result )
    {
      case ColdRootGeometryResult::kNotEvaluated:
        return "not_evaluated";
      case ColdRootGeometryResult::kAccepted:
        return "accepted";
      case ColdRootGeometryResult::kNonfiniteBackprojection:
        return "nonfinite_backprojection";
      case ColdRootGeometryResult::kBehindCamera:
        return "behind_camera";
      case ColdRootGeometryResult::kEmptyAfterFilter:
        return "empty_after_filter";
    }
    return "unknown";
  }

  struct ColdRootObserveDiagnostics
  {
    ColdRootPhase         m_phase  = ColdRootPhase::kNotEvaluated;
    ColdRootReason        m_reason = ColdRootReason::kNotEvaluated;
    ColdRootBootstrapPath m_bootstrap_path =
        ColdRootBootstrapPath::kNotEvaluated;
    ColdRootSeedInputOrigin m_seed_input_origin =
        ColdRootSeedInputOrigin::kNotEvaluated;
    ColdRootGeometryResult m_geometry_result =
        ColdRootGeometryResult::kNotEvaluated;
    std::optional<std::uint64_t> m_attempt_id;

    ColdRootGateState m_bootstrap_gate = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_keyframe_gate  = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_stereo_population_gate =
        ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_root_geometry_gate  = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_imu_excitation_gate = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_conditioning_gate   = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_initialization_solve_gate =
        ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_current_graph_gate = ColdRootGateState::kNotEvaluated;
    ColdRootGateState m_commit_gate        = ColdRootGateState::kNotEvaluated;

    std::optional<std::uint64_t> m_bootstrap_sample_count;
    std::optional<std::uint32_t> m_bootstrap_min_samples;
    std::optional<std::int64_t>  m_bootstrap_duration_ns;
    std::optional<std::int64_t>  m_bootstrap_min_duration_ns;
    std::optional<double>        m_bootstrap_acc_std_max_mps2;
    std::optional<double>        m_bootstrap_acc_std_limit_mps2;
    std::optional<double>        m_bootstrap_gyr_std_max_radps;
    std::optional<double>        m_bootstrap_gyr_std_limit_radps;
    std::optional<double>        m_bootstrap_acc_norm_error_mps2;
    std::optional<double>        m_bootstrap_acc_norm_tolerance_mps2;
    std::optional<std::int64_t>  m_bootstrap_timeout_ns;
    std::optional<bool>          m_moving_bootstrap_enabled;
    std::optional<std::uint64_t> m_moving_suffix_sample_count;
    std::optional<std::int64_t>  m_moving_suffix_duration_ns;
    std::optional<double>        m_moving_acc_mean_norm_mps2;
    std::optional<double>        m_moving_acc_mean_norm_min_mps2;

    std::optional<std::uint64_t> m_current_positive_disparity_count;
    std::optional<bool>          m_accumulated_seed_enabled;
    std::optional<std::uint64_t> m_pending_unique_seed_count;
    std::optional<std::uint64_t> m_effective_seed_count;
    std::optional<std::uint32_t> m_min_seed_observations;
    std::optional<std::uint64_t> m_geometry_accepted_landmarks;
    std::optional<std::uint32_t> m_geometry_min_landmarks;
  };

  struct ImuBias
  {
    Eigen::Vector3d m_acc_mps2  = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_gyr_radps = Eigen::Vector3d::Zero();
  };

  struct VioEstimate
  {
    common::Timestamp timestamp;
    Eigen::Isometry3d T_W_B;
    Eigen::Vector3d   m_v_W_B = Eigen::Vector3d::Zero();
    ImuBias           m_bias;
    std::uint32_t     m_segment_id = 0;
  };

  struct VioDiagnostics
  {
    std::uint32_t               m_nav_states               = 0;
    std::uint32_t               m_imu_factors              = 0;
    std::uint32_t               m_bias_rw_factors          = 0;
    std::uint32_t               m_visual_factors           = 0;
    std::uint32_t               m_root_prior_sets          = 0;
    std::uint32_t               m_integration_steps        = 0;
    std::int64_t                m_integrated_duration_ns   = 0;
    std::int64_t                m_visual_coast_duration_ns = 0;
    std::uint64_t               m_non_keyframe_evictions   = 0;
    std::uint64_t               m_imu_reintegrations       = 0;
    Eigen::Vector3d             m_acc_cov_diag             = Eigen::Vector3d::Zero();
    Eigen::Vector3d             m_gyr_cov_diag             = Eigen::Vector3d::Zero();
    Eigen::Vector3d             m_integration_cov_diag     = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 1> m_bias_rw_sigmas =
        Eigen::Matrix<double, 6, 1>::Zero();
    std::optional<std::uint32_t> m_completed_segment_id;
  };

  struct UpdateDiagnostics
  {
    std::uint32_t num_observations                = 0;
    std::uint32_t num_retained_observations       = 0;
    std::uint32_t num_seeded_landmarks            = 0;
    std::uint32_t num_current_visual_factors      = 0;
    std::uint32_t num_current_mono_visual_factors = 0;
    std::int64_t  unsupported_span_ns             = 0;
    std::uint32_t num_landmarks                   = 0;  // in the graph
    std::uint32_t num_shared                      = 0;  // new frame ∩ window landmark table
    std::uint32_t num_mapped_observations         = 0;  // map membership, independent of disparity
    std::uint32_t num_disparity                   = 0;  // obs with disparity_px > 0 (regardless of landmark table)
    std::uint32_t num_cheirality                  = 0;
    std::uint32_t lm_iterations                   = 0;
    std::uint32_t window_size                     = 0;
    std::uint32_t segment_id                      = 0;  // active estimator segment; 0 is first
    std::uint64_t prior_key                       = 0;  // Symbol('x', k) index k
    double        reproj_rms_before_px            = 0.0;
    double        reproj_rms_after_px             = 0.0;
    double        max_window_pose_shift_m         = 0.0;
    bool          low_connectivity                = false;
    bool          pnp_success                     = false;
    std::uint32_t pnp_inliers                     = 0;
    std::uint32_t outliers_culled                 = 0;
    std::uint32_t outliers_culled_unique          = 0;
    double        reproj_rms_after_cull_px        = 0.0;
    bool          outlier_reopt                   = false;  // rounds > 0
    bool          outlier_reopt_failed            = false;  // LM₂ 失败已回退；不进 diag.csv
    std::uint32_t outlier_reopt_rounds            = 0;      // 不进 diag.csv
    // 本帧永久移出地图的 id（mean-cull ∪ cheirality）；不进 diag.csv
    std::vector<common::LandmarkId> culled_landmark_ids;
    // Probe B 旁路字段；不进 diag.csv
    std::uint32_t probe_rejected_block_n = 0;
    std::uint32_t probe_new_lm_n         = 0;
    // 仅当 enable_probe_b 时填充；默认保持 0/空
    std::vector<std::pair<std::uint64_t, double>> probe_shift_top;  // key, |Δt|
    double                                        probe_res_mean_px = 0.0;
    double                                        probe_res_max_px  = 0.0;
    LandmarkId                                    probe_res_max_id{};
    bool                                          probe_detail_valid = false;
    ColdRootObserveDiagnostics                    m_cold_root;
    VioDiagnostics                                m_vio;
  };

  struct VioUpdateResult
  {
    UpdateStatus               status = UpdateStatus::kFailed;
    std::optional<VioEstimate> estimate;  // only when kOk
    UpdateDiagnostics          diagnostics;
    std::string                message;  // non-empty when not kOk
  };

}  // namespace phad::estimator

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

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

  struct KeyframeMeasurement
  {
    common::Timestamp              timestamp;
    std::vector<StereoObservation> observations;
    // 本帧与上一帧之间的 IMU 段 [t_prev, timestamp]（sync 切段语义，见
    // StereoImuPacket）。enable_imu=false 时 estimator 直接忽略；段含两端
    // 插值样本，相邻段共享右端样本，estimator 按需重建 AHRS 预积分。
    std::vector<sensor::ImuMeasurement> imu_samples;
    common::Timestamp                   t_prev{ 0 };
    bool                                imu_gap = false;
  };

  struct EstimatorOptions
  {
    int window_size               = 10;
    int min_landmark_observations = 2;
    // init and re-anchor. 10 为 slice-7 原值。pre-M4 round 2 实测否决
    // (2026-08-07): 阈值 5 与跨帧累积使 V2_03 re-anchor 9 → 32-68, 每段
    // 只带自身观测、锚误差无法修正 → 段错位贡献 +2.739 → +3.9~+5.6m,
    // ATE 3.628 → 5.2-6.7。门槛是质量门: 只放行足以滋养健康段的富帧。
    // CLI: --min-seed-observations。
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
    bool   use_constant_velocity_init      = true;
    bool   enable_reanchor                 = true;  // false reproduces M3.2 permanent reject
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
    // pre-M4 round 2 残存: 首段跨帧累积播种 (SVO DepthFilter 式证据累积)。
    // 全量累积(含 re-anchor)已被实测否决 —— re-anchor 放宽是纯毒(见
    // min_seed_observations 注释); 此处仅保留「首段专用」作用域: Gate F
    // (未初始化) 累积, Gate E (re-anchor) 保持原拒绝。默认关;
    // CLI: --estimator-enable-accumulated-seed (A/B, 不进 config_hash)。
    bool enable_accumulated_seed = false;
    // Session sets true when probe_b_path non-empty; NOT in flattenConfig.
    bool enable_probe_b = false;
    // Session sets true when vio_state_probe_path is non-empty. This only
    // enables read-only graph objective snapshots and is not in flattenConfig.
    bool enable_vio_state_probe = false;
    // Session sets true when fixed_lag_shadow_probe_path is non-empty. The
    // shadow never supplies production state and is not in flattenConfig.
    bool enable_fixed_lag_shadow = false;
    // ---- M4.4 gyro-visual fusion（默认开，进 config_hash）----
    // enable_imu=false 完全走纯视觉链；不初始化 IMU、不做 gyro alignment，
    // 也不建 AHRS factor。CLI: --no-imu。
    bool enable_imu = true;
    // 静止初始化用于 gravity-direction / accelerometer-bias 审计的模型值。
    double imu_gravity = 9.81007;
    // Gyroscope white-noise density；AHRS preintegration 使用 density²。
    double imu_gyr_noise_nd = 1.6968e-4;
    // Gyro-active graph 的窗口锚 pose prior。
    double imu_prior_pose_sigma = 1e-4;
    // M4.4 gyro-first staged activation: pure visual rotations first estimate
    // one shared gyro bias, then this tight prior freezes that calibrated value
    // while the pose-only AHRS factors are evaluated.  This is intentionally a
    // diagnostic vertical slice; the MH_01 VIO<VO gate decides whether it may
    // graduate to a production prior.
    double imu_prior_bias_gyro_sigma = 1e-6;
    // Pure-visual accepted-pose support required before gyro factors activate.
    // The activation update itself remains vision-only; fusion starts on the
    // next update so one graph never mixes calibration and consumption.
    double imu_gyro_align_window_s = 100.0;
    // ---- M4.4 non-blocking static gyro audit（进 config_hash）----
    // 检测窗口（累积缓冲尾部的连续 IMU 样本跨度）与方差阈值：窗口内
    // gyro/accel 逐轴 std 全部低于阈值 → 静止（设计稿 §5.1）。
    // M4.3d 门①调参 (2026-08-09): 2.0s 重验 → 0.1263 劣于 0.5s →
    // 0.1223 (init 估计逐位不变: a_mean 污染是稳态的 scale/tilt 混淆,
    // 非瞬态修正; 长窗另损 0.7% coverage)。保持 0.5s。
    double imu_init_window_s  = 0.5;
    double imu_init_gyro_std  = 1e-2;  // rad/s
    double imu_init_accel_std = 2e-1;  // m/s²
    // 超时自首帧起算（init 缓冲从首帧样本累积），需覆盖首个静止期出现的
    // 最晚时刻；EuRoC 全序列扫描（11 seq, 0.5s 滑窗）：MH_01/02 首个
    // 悬停分别在 t≈21.7s/26.4s，其余序列 ≤11s 或起飞前地面静止。30s
    // 给 MH_02 的 26.9s 检测终点留 ~3s 裕量；超时 → init 失败返回原因
    // （C9，不静默用伪初始化冒充成功）。
    double imu_init_timeout_s = 30.0;
    // accel 方差阈值 2e-1 依据 EuRoC 数据（C15 式扫参替代, plan 风险行
    // "滑动重试 + 超时（宽容）"）：旋翼振动底噪使真实静止窗口
    // (gt 速度 <0.01 m/s) 的 accel std 达 0.08-0.19 —— 原 2e-2 在
    // MH_01 首个悬停 (0.080) 前就超时失败; V2_01 地面静止 0.191、
    // V1_01 起飞前静止 0.148 也超 1e-1。2e-1 覆盖全部 11 序列的首个
    // 静止窗口 (最坏裕量 ~5%, V2_01)。误接受分析: 飞行中 0.5s 窗口
    // 要么 gyr_std ≥ 1e-2 (旋转/机动, 被 gyro 门拦下), 要么匀速平移
    // (比力恒定 = −g, 均值估计仍正确); 实际机动段 accel std 0.2-1.2
    // 远离阈值。g 估计误差 = 窗口内比力变化, ≤0.2/9.8 ≈ 2%。
  };

  enum class UpdateStatus : std::uint8_t
  {
    kOk       = 0,
    kRejected = 1,
    kFailed   = 2
  };

  enum class FusionMode : std::uint8_t
  {
    kVisionOnly = 0,
    kGyroVisual = 1
  };

  struct VioEstimate
  {
    common::Timestamp timestamp;
    Eigen::Isometry3d T_W_B;
  };

  // One-time snapshot of the exact static IMU window and state committed by
  // the production initializer.  It deliberately exposes Eigen/POD values
  // only; GTSAM types remain behind StereoVoEstimator's PIMPL boundary.
  struct ImuInitDiagnostics
  {
    std::uint32_t     imu_sample_count        = 0;
    std::int64_t      imu_t_i_ns              = 0;
    std::int64_t      imu_t_j_ns              = 0;
    double            imu_dt_s                = 0.0;
    Eigen::Vector3d   gyro_mean               = Eigen::Vector3d::Zero();
    Eigen::Vector3d   gyro_std                = Eigen::Vector3d::Zero();
    Eigen::Vector3d   acc_mean                = Eigen::Vector3d::Zero();
    Eigen::Vector3d   acc_std                 = Eigen::Vector3d::Zero();
    Eigen::Isometry3d T_W_B0                  = Eigen::Isometry3d::Identity();
    Eigen::Vector3d   velocity_W              = Eigen::Vector3d::Zero();
    Eigen::Vector3d   bias_gyro               = Eigen::Vector3d::Zero();
    Eigen::Vector3d   bias_acc                = Eigen::Vector3d::Zero();
    double            acc_mean_norm           = 0.0;
    double            gravity_model_magnitude = 0.0;
    double            gyro_std_limit          = 0.0;
    double            acc_std_limit           = 0.0;
  };

  // Read-only snapshot of the gyro prediction used to initialise an accepted
  // pose. This interface intentionally exposes only Eigen/POD values; GTSAM
  // AHRS preintegration remains private to StereoVoEstimator.
  struct GyroStateDiagnostics
  {
    std::uint64_t     state_key            = 0;
    std::uint32_t     imu_sample_count     = 0;
    std::int64_t      imu_t_i_ns           = 0;
    std::int64_t      imu_t_j_ns           = 0;
    double            imu_dt_s             = 0.0;
    bool              prediction_valid     = false;
    Eigen::Isometry3d predicted_T_W_B      = Eigen::Isometry3d::Identity();
    Eigen::Vector3d   prediction_bias_gyro = Eigen::Vector3d::Zero();
    // Pose actually inserted into the graph after optional PnP, before LM.
    // This separates gyro prediction/PnP initialization from factor effects.
    Eigen::Isometry3d graph_initial_T_W_B = Eigen::Isometry3d::Identity();
  };

  // Read-only objective decomposition for the gyro-visual graph actually
  // committed by an update. Boundary is the AHRS edge leaving the anchored
  // oldest pose; newest is the edge entering the current pose. Whitened norms
  // use the factor's Gaussian model and are dimensionless.
  struct GyroGraphCostDiagnostics
  {
    std::uint32_t gyro_factor_count                     = 0;
    std::uint32_t interior_factor_count                 = 0;
    std::uint32_t stereo_factor_count                   = 0;
    double        total_initial_cost                    = 0.0;
    double        total_posterior_cost                  = 0.0;
    double        gyro_initial_cost                     = 0.0;
    double        gyro_posterior_cost                   = 0.0;
    double        boundary_initial_cost                 = 0.0;
    double        boundary_posterior_cost               = 0.0;
    double        boundary_initial_residual_norm_rad    = 0.0;
    double        boundary_posterior_residual_norm_rad  = 0.0;
    double        boundary_initial_whitened_norm        = 0.0;
    double        boundary_posterior_whitened_norm      = 0.0;
    double        interior_initial_cost                 = 0.0;
    double        interior_posterior_cost               = 0.0;
    double        newest_initial_cost                   = 0.0;
    double        newest_posterior_cost                 = 0.0;
    double        newest_initial_residual_norm_rad      = 0.0;
    double        newest_posterior_residual_norm_rad    = 0.0;
    double        newest_initial_whitened_norm          = 0.0;
    double        newest_posterior_whitened_norm        = 0.0;
    double        stereo_initial_cost                   = 0.0;
    double        stereo_posterior_cost                 = 0.0;
    double        pose_prior_posterior_rotation_norm    = 0.0;
    double        pose_prior_posterior_translation_norm = 0.0;
    double        pose_prior_posterior_cost             = 0.0;
    double        bias_prior_posterior_norm             = 0.0;
    double        bias_prior_posterior_cost             = 0.0;
  };

  enum class FixedLagShadowReset : std::uint8_t
  {
    kNone      = 0,
    kBootstrap = 1,
    kSegment   = 2
  };

  // CLI-only read-only fixed-lag shadow snapshot. GTSAM state, factor slots
  // and timestamps remain behind StereoVoEstimator's PIMPL boundary.
  struct FixedLagShadowDiagnostics
  {
    bool                active                        = false;
    bool                update_ok                     = true;
    bool                reset                         = false;
    FixedLagShadowReset reset_reason                  = FixedLagShadowReset::kNone;
    std::uint64_t       current_epoch                 = 0;
    std::uint64_t       cutoff_epoch                  = 0;
    std::uint32_t       batch_window_size             = 0;
    std::uint32_t       smoother_pose_count           = 0;
    std::uint32_t       smoother_landmark_count       = 0;
    bool                bias_present                  = false;
    bool                bias_fixed                    = false;
    double              bias_delta_norm               = 0.0;
    std::uint32_t       user_factor_count             = 0;
    std::uint32_t       missing_owned_slot_count      = 0;
    std::uint32_t       timestamp_without_value_count = 0;
    std::uint32_t       marginalized_pose_count       = 0;
    std::uint32_t       retired_landmark_count        = 0;
    std::uint32_t       new_landmark_generation_count = 0;
    Eigen::Isometry3d   newest_T_W_B                  = Eigen::Isometry3d::Identity();
    double              newest_rotation_delta_rad     = 0.0;
    double              newest_translation_delta_m    = 0.0;
  };

  struct UpdateDiagnostics
  {
    std::uint32_t num_observations         = 0;
    std::uint32_t num_landmarks            = 0;  // in the graph
    std::uint32_t num_shared               = 0;  // new frame ∩ window landmark table
    std::uint32_t num_disparity            = 0;  // obs with disparity_px > 0 (regardless of landmark table)
    std::uint32_t num_cheirality           = 0;
    std::uint32_t lm_iterations            = 0;
    std::uint32_t window_size              = 0;
    std::uint32_t segment_id               = 0;  // increments on re-anchor; 0 is first segment
    std::uint64_t prior_key                = 0;  // Symbol('x', k) index k
    double        reproj_rms_before_px     = 0.0;
    double        reproj_rms_after_px      = 0.0;
    double        max_window_pose_shift_m  = 0.0;
    bool          low_connectivity         = false;
    bool          pnp_success              = false;
    std::uint32_t pnp_inliers              = 0;
    std::uint32_t outliers_culled          = 0;
    std::uint32_t outliers_culled_unique   = 0;
    double        reproj_rms_after_cull_px = 0.0;
    bool          outlier_reopt            = false;  // rounds > 0
    bool          outlier_reopt_failed     = false;  // LM₂ 失败已回退；不进 diag.csv
    std::uint32_t outlier_reopt_rounds     = 0;      // 不进 diag.csv
    // 本帧永久移出地图的 id（mean-cull ∪ cheirality）；不进 diag.csv
    std::vector<common::LandmarkId> culled_landmark_ids;
    // Probe B 旁路字段；不进 diag.csv
    std::uint32_t probe_rejected_block_n = 0;
    std::uint32_t probe_new_lm_n         = 0;
    // ---- M4.4 non-blocking static gyro audit ----
    // audit 未完成时为 true，但不决定 update status；session 单独计数
    // init_pending_frames。超时失败 sticky（session → SessionError）。
    // IMU-off 恒 false。
    bool        init_pending = false;
    bool        init_failed  = false;
    std::string init_failure_reason;
    // The graph mode actually used for this update. Gyro alignment commits at
    // the end of an update; gyro factors start on the next update.
    FusionMode    fusion_mode       = FusionMode::kVisionOnly;
    std::uint32_t gyro_factor_count = 0;
    // RMS norm of the post-alignment visual-vs-gyro rotation residual used to
    // calibrate gyro factor covariance. Zero until fusion is active.
    double gyro_alignment_residual_rms_rad = 0.0;
    // Shared gyro bias: accepted frames report the optimizer posterior;
    // rejected frames keep the last accepted value. bias_acc remains zero in
    // the established diag.csv schema because gyro-only has no acc-bias state.
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_acc  = Eigen::Vector3d::Zero();
    // Present only for accepted gyro-visual rows when the CLI-only state
    // probe is enabled. Production state never consumes it.
    std::optional<GyroGraphCostDiagnostics> gyro_graph_cost;
    // Present on accepted IMU-on rows only when the CLI-only fixed-lag shadow
    // is enabled. Production prediction, graph and returned estimate never
    // consume this snapshot.
    std::optional<FixedLagShadowDiagnostics> fixed_lag_shadow;
    // Present exactly once, on the update that commits static IMU init.
    // The update may still be rejected later by visual seeding.
    std::optional<ImuInitDiagnostics> imu_init;
    // Present only for accepted IMU-on states. prediction_valid=false keeps
    // accepted gap/warm-up states observable without pretending zeroes are a
    // gyro prediction.
    std::optional<GyroStateDiagnostics> gyro_state;
    // 仅当 enable_probe_b 时填充；默认保持 0/空
    std::vector<std::pair<std::uint64_t, double>> probe_shift_top;  // key, |Δt|
    double                                        probe_res_mean_px = 0.0;
    double                                        probe_res_max_px  = 0.0;
    LandmarkId                                    probe_res_max_id{};
    bool                                          probe_detail_valid = false;
  };

  struct VioUpdateResult
  {
    UpdateStatus               status = UpdateStatus::kFailed;
    std::optional<VioEstimate> estimate;  // only when kOk
    UpdateDiagnostics          diagnostics;
    std::string                message;  // non-empty when not kOk
  };

}  // namespace phad::estimator

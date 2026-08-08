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
    Eigen::Vector2d left_pixel;    // rectified left
    // > 0: stereo disparity (depth via backproject); == 0: stereo failed —
    // no depth, kept in the window until stereo returns (Slice ⑦). < 0 is
    // invalid.
    double          disparity_px;
  };

  struct KeyframeMeasurement
  {
    common::Timestamp              timestamp;
    std::vector<StereoObservation> observations;
    // M4.2: 本帧与上一帧之间的 IMU 段 [t_prev, timestamp]（sync 切段语义，
    // 见 StereoImuPacket）。IMU-off 时恒空且 imu_gap=true；IMU-on 且
    // enable_imu=false 时 estimator 直接忽略。段含两端插值样本，相邻段共享
    // 右端样本——estimator 按需在 buildGraph 时即时重建预积分（C3）。
    std::vector<sensor::ImuMeasurement> imu_samples;
    common::Timestamp                  t_prev{ 0 };
    bool                               imu_gap = false;
  };

  struct EstimatorOptions
  {
    int    window_size                = 10;
    int    min_landmark_observations  = 2;
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
    int    min_shared_landmarks       = 10;
    double stereo_sigma_px            = 1.0;
    double huber_k_px                 = 3.0;  // <= 0 disables Robust wrapper
    double prior_rotation_sigma_rad   = 1e-4;
    double prior_translation_sigma_m  = 1e-4;
    bool   use_constant_velocity_init = true;
    bool   enable_reanchor            = true;  // false reproduces M3.2 permanent reject
    bool   enable_pnp_init            = true;
    double pnp_reproj_px              = 2.0;
    double pnp_confidence             = 0.99;
    int    min_pnp_inliers            = 10;
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
    // ---- M4.2 IMU 机制（默认开，进 config_hash）----
    // enable_imu=false 完全走原 M3.3 链（不建 V/B 变量、无 IMU 因子）→
    // IMU-off 字节回归保证。CLI: --no-imu。
    bool enable_imu = true;
    // 伪初始化重力（C4/C5）：EuRoC g = 9.81007，Z-up（MakeSharedU）。
    double imu_gravity = 9.81007;
    // 噪声密度（implicit smart 无重积分情况下的 white noise 模型；按设计稿
    // §2.3：协方差 = 密度平方）。EuRoC 默认：acc_nd 2.0e-3 m/s²/√Hz、
    // gyr_nd 1.6968e-4 rad/s/√Hz、acc_rw 3.0e-3 m/s²/√Hz、
    // gyr_rw 1.9393e-5 rad/s²/√Hz。
    double imu_acc_noise_nd = 2.0e-3;
    double imu_gyr_noise_nd = 1.6968e-4;
    double imu_acc_rw       = 3.0e-3;
    double imu_gyr_rw       = 1.9393e-5;
    // 最老帧 / gap 恢复帧 priors（C11/C15，中等 sigma 不扫参）：
    // X prior 放松（IMU 因子约束重力/速度）；V prior 0 ± 1.0 m/s；
    // B prior 0 ± gyro 1e-3 rad/s / acc 1e-2 m/s²。
    double imu_prior_pose_sigma        = 1e-2;
    double imu_prior_vel_sigma         = 1.0;
    double imu_prior_bias_gyro_sigma   = 1e-3;
    double imu_prior_bias_acc_sigma    = 1e-2;
  };

  enum class UpdateStatus : std::uint8_t
  {
    kOk       = 0,
    kRejected = 1,
    kFailed   = 2
  };

  struct VioEstimate
  {
    common::Timestamp timestamp;
    Eigen::Isometry3d T_W_B;
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

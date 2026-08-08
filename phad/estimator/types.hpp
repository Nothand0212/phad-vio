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
    // B prior 0 ± gyro 1e-1 rad/s / acc 1e-1 m/s²。
    // C15: acc 原 1e-2 过紧 (EuRoC ~2e-2-5e-2, KnownBias 注入 0.3 无法
    // 被图吸收 → 泄漏进速度 → 位姿漂移) → 1e-1。
    // M4.3d: gyro 原 1e-3 有同样问题且更严重 —— prior 信息量 (1/σ²=1e6)
    // 远大于短链上 IMU 因子的局部 bias 信息 (0.5s 悬停段实测 ≈324),
    // 首帧 LM 一步把正确的 init bias (EuRoC 实测 z 轴 0.080 rad/s) 压回 0,
    // 之后预积分以错误 bias 重建 → 航向按 (b_true−b_est)·t 累积漂移
    // (MH_01 转误差 ±30°, ATE 0.458 vs 视觉门 0.100)。1e-1 后 bias prior
    // 信息量 100 已可被因子链吸收; 首图 yaw gauge (视觉因子只约束相对
    // 几何, 共旋零成本) 由 buildGraph 的 seed pose prior 单独关闭
    // (M4.3d), 之后 bias 由 init 初值 + 因子链观测性决定。
    double imu_prior_pose_sigma        = 1e-2;
    double imu_prior_vel_sigma         = 1.0;
    double imu_prior_bias_gyro_sigma   = 1e-1;
    double imu_prior_bias_acc_sigma    = 1e-1;
    // ---- M4.3 静止初始化（进 config_hash）----
    // 检测窗口（累积缓冲尾部的连续 IMU 样本跨度）与方差阈值：窗口内
    // gyro/accel 逐轴 std 全部低于阈值 → 静止（设计稿 §5.1）。
    double imu_init_window_s     = 0.5;
    double imu_init_gyro_std     = 1e-2;  // rad/s
    double imu_init_accel_std    = 2e-1;  // m/s²
    // 超时自首帧起算（init 缓冲从首帧样本累积），需覆盖首个静止期出现的
    // 最晚时刻；EuRoC 全序列扫描（11 seq, 0.5s 滑窗）：MH_01/02 首个
    // 悬停分别在 t≈21.7s/26.4s，其余序列 ≤11s 或起飞前地面静止。30s
    // 给 MH_02 的 26.9s 检测终点留 ~3s 裕量；超时 → init 失败返回原因
    // （C9，不静默用伪初始化冒充成功）。
    double imu_init_timeout_s    = 30.0;
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
    // ---- M4.3 静止初始化 ----
    // init 未完成时每帧为 true（session 计数 init_dropped_frames，C8）；
    // init 失败 sticky 为 true（C9，session → SessionError → kFailed）。
    // IMU-off 恒 false。
    bool          init_pending = false;
    bool          init_failed  = false;
    std::string   init_failure_reason;
    // 本帧优化后的 bias（接受帧 = LM 回写值；被拒帧 = 上一接受值）。
    // IMU-off 恒 0。进 diag.csv（Q3/C12：IMU-off 填 0 保证原列回归）。
    Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_acc  = Eigen::Vector3d::Zero();
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

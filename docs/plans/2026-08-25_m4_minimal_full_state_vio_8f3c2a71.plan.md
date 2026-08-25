---
name: M4 minimal full-state VIO
overview: 单一 VioEstimator 的 packet-to-state 与 visual-outage vertical slices 已完成；eviction reintegration 由下一分片完成。
todos:
  - id: full-state-packet-to-state
    content: 原子迁移 VioEstimator、tagged IMU payload、静止 bootstrap、X/V/B + ImuFactor + bias-RW，并以 estimator/sync/apps 定向测试验收
    status: completed
  - id: visual-outage-coast
    content: 实现 500 ms IMU-only coast、同段恢复与 horizon-boundary kVisualOutage transaction，并补 seam tests
    status: completed
  - id: eviction-reintegration
    content: 实现 non-keyframe eviction 的 raw interval 拼接、fresh reintegration 与 factor/window 原子替换，并补 bounded-window tests
    status: pending
isProject: false
---

# M4 minimal full-state VIO

## 切片划分

- **Slice ① — packet-to-state**：唯一外部行为是一个同步 packet 经
  `VioEstimator::update()` 原子提交 `X/V/B`；静止 packet 提交 root，下一条连续 raw
  interval 提交 successor、`ImuFactor` 与 bias-RW factor。raw/discontinuity 为 closed tagged
  payload；合法 discontinuity 完成当前 segment 且不建立跨 gap factor。malformed input 返回
  `kInvalidInput`，observable state 不变。
- **Slice ② — visual outage**：唯一外部行为是视觉支持短时不足时在 500 ms horizon 内保持
  state cadence，并在支持恢复时留在同一 segment；边界 packet 原子完成 visual-outage transition。
- **Slice ③ — eviction reintegration**：唯一外部行为是删除 non-keyframe 后，以相邻 raw provenance
  fresh reintegrate predecessor→successor，并原子替换 state/factor/window；窗口保持有界。

## Concrete contracts

### Public POD 与 ownership

- `sensor::RawImuInterval`：`m_t_begin`、`m_t_end`、value-owned
  `std::vector<ImuMeasurement> m_samples`。samples 是 source-normalized raw support，可在区间外各保留
  一个 bracket；copy/move 均为普通 value semantics。
- `sensor::MeasurementDiscontinuity`：只含 `m_t_begin`、`m_t_end`。
- `sensor::ImuPayload = std::variant<RawImuInterval, MeasurementDiscontinuity>`；
  `StereoImuPacket` 只含 `m_frame` 与 `m_imu`，不携带插值样本、`dt` 或 gap bool。
- `estimator::VioMeasurement`：`m_timestamp`、value-owned `m_observations`、`m_imu`；
  keyframe decision 继续作为同一 `update(measurement, is_keyframe)` 调用的 project-owned bool，
  不形成第二个 estimator seam。
- `estimator::ImuBias` 按 accelerometer、gyroscope 顺序各持有一个 `Eigen::Vector3d`；
  `VioEstimate` 返回 `timestamp/T_W_B`、`m_v_W_B`、`m_bias` 与 `m_segment_id`。
- `VioEstimator` 保持 move-only PIMPL。PIM、GTSAM keys/types、bootstrap accumulator、raw cache、
  graph、window 与 transaction 不出 implementation。

### Result、failure 与 caller policy

- `UpdateStatus`：`kOk`、`kInitializing`、`kDiscontinuity`、`kVisualOutage`、
  `kInvalidInput`、`kRejected`、`kFailed`；只有 `kOk` 有 estimate。
- `kInvalidInput` 覆盖 payload/tag、endpoint/anchor、timestamp arithmetic、sample order、finite、
  bracket、normalization closure 与 positive-`dt` 失败；validation 在 staging 前完成。
- `kFailed` 只表示合法输入上的 graph build、optimizer、extraction 或 transaction failure；
  RAII snapshot 恢复全部 state，再从恢复后的 state 生成 diagnostics。message 保留阶段前缀与
  `std::exception::what()`；estimator 不打日志，apps 将 `kFailed` 作为 session terminal error。
- active segment 的视觉支持不足走 coast/outage，不返回 `kRejected`；`kRejected` 只保留给无法形成 root
  visual seed 的合法输入，不用于 IMU input validation。
- active discontinuity 返回一次 `kDiscontinuity`，同时记录 completed segment id；下一 segment id
  递增并进入 initializing。initializing discontinuity 清空未提交 bootstrap evidence、推进 continuity
  anchor，返回 `kInitializing`。

### Static bootstrap 与 construction values

以下数值是本 vertical slice 的 deterministic C++ construction baseline；composition root 明确使用
`sync.imu_continuity_limit_ns=20'000'000`，其余值由 `EstimatorOptions` 构造并经 seam diagnostics 验证。
parser、serialization、canonical config snapshot 与 profile approval 随 authority migration 交付；不据此作
真实数据或 default-on 质量结论：

| key | value | contract |
|---|---:|---|
| `sync.imu_continuity_limit_ns` | `20'000'000` | 必填正整数；sync lifetime immutable；`delta == limit` 连续 |
| `estimator.imu.gravity_mps2` | `9.81` | finite、`>0`，ENU `g_W=[0,0,-g]` |
| `estimator.imu.q_int` | `0.0` | finite、`>=0`；直接设置 `integrationCovariance=q_int I3` |
| `estimator.imu.bootstrap_min_duration_ns` | `20'000'000` | 至少覆盖该正时长 |
| `estimator.imu.bootstrap_min_samples` | `3` | canonical nodes 去重后计数 |
| `estimator.imu.bootstrap_max_acc_std_mps2` | `0.05` | 三轴 sample std 的最大值 |
| `estimator.imu.bootstrap_max_gyr_std_radps` | `0.005` | 三轴 sample std 的最大值 |
| `estimator.imu.bootstrap_acc_norm_tol_mps2` | `0.25` | `abs(norm(mean_acc)-g)` 上限 |
| `estimator.imu.bootstrap_timeout_ns` | `1'000'000'000` | 超时返回 `kFailed`，清空本次 accumulator 后从新 anchor 重试 |
| `estimator.imu.velocity_prior_sigma_mps` | `1e-3` | root `V0` prior |
| `estimator.imu.acc_bias_prior_sigma_mps2` | `0.1` | root `b_a0=0` prior |
| `estimator.imu.gyr_bias_prior_sigma_radps` | `0.01` | root `b_g0=mean(gyr)` prior |
| `estimator.visual_coast_horizon_ns` | `500'000'000` | Slice ②；finite nonzero integer duration |

`sensor::ImuParameters` 由 estimator constructor 单独接收。唯一 parameter builder 设置
`accelerometerCovariance=acc_nd² I3`、`gyroscopeCovariance=gyr_nd² I3` 与上述
`integrationCovariance`。相邻 graph state 跨度 `DeltaT` 的 bias factor 使用 diagonal Sigmas
`[acc_rw sqrt(DeltaT)]×3, [gyr_rw sqrt(DeltaT)]×3`。

bootstrap 对 canonical nodes 做算术 mean/std。门闭合时
`b_g0=mean(gyr)`、`b_a0=0`、`v0=0`；`R_W_B` 取把 normalized `mean(acc)` 映射到 world `+Z`
的最小旋转，yaw gauge 为零，translation 为零。root timestamp 是闭门 packet 的图像 timestamp；
bootstrap nodes 不进入首条 `(t0,t1]` PIM。

### Sync、normalization 与 continuity

- sync 先配对 stereo，再等待足以判定目标 endpoint 的 raw IMU support；source 结束时缺 bracket
  明确输出 discontinuity。
- 首 packet 的 `t_begin` 是当前缓冲区最早 raw IMU timestamp；后续 packet 的 `t_begin` 精确等于
  上一 emitted image timestamp，`t_end` 等于当前 image timestamp。
- 与 `(t_begin,t_end)` 有正时长交叠的每个相邻 raw pair 都检查 integer `delta_ns`；bracket 完整且
  所有 `delta_ns <= limit_ns` 才输出 raw arm，否则输出相同 endpoints 的 discontinuity arm。
- estimator 对两个 endpoint exact-or-linear-interpolate，构造 closed canonical support；每对相邻
  nodes 以 integer ns 求唯一 positive `dt`，acc/gyr 均取 endpoint arithmetic mean，恰好调用一次
  `integrateMeasurement()`。不补 zero-`dt` call，不修改末段 `dt`。

### State/window、raw provenance 与 segment

- `frame_index` 在 estimator lifetime 内单调递增；GTSAM keys 固定为 `X(k)`、`V(k)`、`B(k)`。
  每个 committed packet state 各有一组 `X/V/B`；keyframe 只控制 seeding/eviction priority。
- successor 私有保存 caller raw interval；graph build 在 predecessor 当前 committed `B(i)` 处 fresh
  preintegrate，GTSAM factor 使用其标准 first-order bias correction。任何 reopt rebuild 使用该轮
  predecessor bias 重新构造 PIM。
- segment id 从 0 单调递增；`VioEstimate` 携带所属 id。discontinuity event 同时暴露 completed id 与
  pending next id，旧、新 segment 的 window、landmark、IMU、bias-RW 与 posterior 均不相连。
- Slice ③ 拼接两个相邻 raw intervals 时只删除共同 timestamp 的重复 endpoint；六个 measurement
  分量必须 bit-equal，否则 transaction 以 `kFailed` 回滚。拼接后走同一 public-input normalization、
  noise builder 与 fresh preintegration 路径；先 stage replacement graph，验证成功后再 commit window。

### Visual support 与 outage（Slice ②）

- predicate owner 是 estimator：当前 packet 能建立至少 `min_pnp_inliers` 个 finite、positive-disparity
  stereo correspondences 时为 supported；apps 不预判。
- unsupported 但 IMU 连续时，从首个 unsupported interval 开始累计 integer ns；累计 `<=500 ms` 的
  packet 仍提交 `X/V/B`、IMU 与 bias-RW，不建立 visual factor。
- 支持在 horizon 内恢复则清零 coast duration、segment 不变。将使累计时长首次 `>500 ms` 的 boundary
  packet 不提交 state/factor，原子完成 segment，返回 `kVisualOutage`；后续只经静止 bootstrap 建新 root。

## Slice ① 文件范围与验收

- 产品：`phad/sensor/stereo_imu_packet.hpp`、`phad/sync/stereo_pair_synchronizer.*`、
  `phad/estimator/types.hpp`、现有 estimator implementation/PIMPL、`apps/stereo_pair_stream.*`、
  `apps/stereo_vo_glue.hpp` 与全部 estimator/packet callsites。
- 原子 rename：class/header/source/callsites 一次改为 `VioEstimator`；删除 `G(k)`、gyro-only factor、
  options/diagnostics/state/lifecycle 及其 CMake/test entries，不提供 alias、wrapper 或 alternate backend。
- tests：通过 public seam 验证 root/successor state、endpoint exact/interpolation、`N-1` mean integration、
  noise mapping、discontinuity 无 bridge、malformed rollback；保留视觉 estimator 回归测试并迁移到明确
  deterministic IMU fixture。
- commands：刷新 CMake/compile database；构建并运行 `phad_estimator_tests`、`phad_sync_tests`、
  `phad_apps_tests`；最后执行 `git diff --check`、完整 diff 与 status 检查。

## Slice ② 文件范围与验收

- 产品：`phad/estimator/types.hpp`、现有 estimator implementation/PIMPL、
  `apps/stereo_vo_glue.hpp`、`apps/offline_vo_session.cpp` 与现有无窗 probe；不增加 backend 或第二 seam。
- transaction：coast duration 随 estimator state 一起 snapshot/rollback；合法 boundary packet 清空 active
  segment、推进 continuity anchor 并递增 segment id，不分配 frame key，不建立 factor。
- tests：只经 `VioEstimator::update()` 验证 unsupported cadence、无 current visual factor、horizon 内恢复、
  `==500 ms` 闭边界、`>500 ms` one-shot transition、新 root 无 bridge，以及 malformed raw 不改变 coast state。
- smoke：使用现有 `OfflineVoSession` 的 `max_frames` 跑 EuRoC 短前缀，只检查运行状态、coast/recovery cadence
  与 segment continuity，不进入 ATE/RPE 或正式 gate。

# M4 minimal full-state VIO Draft spec

日期：2026-08-25

状态：**Draft。架构合同已收口；本文不是产品代码、测试、replay、default-on 或既有 authority 的实施授权。**

## 1. 目标与证据边界

M4 的下一条 vertical slice 是单一 estimator 内最小可运行的 full-state VIO：在同步图像时刻维护
pose `X(k)`、velocity `V(k)` 与 canonical IMU bias `B(k)`，由同一 implementation 完成 raw IMU
规范化、预积分、factor 构造、窗口 lifecycle 与原子 transaction。gyro bias 不再作为独立的长期状态岛。

[ADR-0001](../adr/0001-gtsam-vio-backend.md) 已接受 GTSAM `Pose3`、`NavState`、
`imuBias::ConstantBias`、`PreintegratedImuMeasurements`、`ImuFactor` 与
`BetweenFactor<imuBias::ConstantBias>` 的长期方向，并要求 GTSAM ownership 留在 estimator 内部。
[ADR-0002](../adr/0002-stage-gated-gyro-only-bias-state.md) 进一步规定：full `X/V/B` activation
时删除临时 `G(k)`、gyro-only factor 与 lifecycle，改用 `B(k): imuBias::ConstantBias`，不保留 alias、
wrapper、dual-write 或并行实现。

当前 online-bias replay2 的 verdict 仍为 `HARD_ERROR`，原因是 negative-arm setup 的
harness/operator wrong-argument invocation；`product_attribution=false`、
`scientific_attribution=false`、`permission_granted=false`。[result ledger](../research/m4-online-gyro-bias-synthetic-result.md)
保全 #40 candidate commit `9bdd32df4089774b667db721815bb89a3007a47b`、tree
`732c8ea3e8a280693b58dac4211646f5a6a0e44a`、positive actual `4/4`、`37/37`、`10/10`、
`114/114`，以及有效 negative arms `5/19`。replay3 保持 paused；Q3 的唯一历史结论仍是
`HYPOTHESIS_FAIL / HALF_STABILITY / STOP`。这些结果不构成 full-state VIO 的产品或科学证据。

## 2. 唯一外部 seam

最终 estimator 名称为 `VioEstimator`。所有同步 packet 通过同一个 `VioEstimator::update()` interface；
不增加第二套 bootstrap、reset、gap-handler 或 IMU-only public interface。

每次 update 的 project-owned measurement POD 在概念上只包含：

- 当前同步图像 timestamp；
- project-owned visual observations；
- 一个 closed tagged IMU payload，恰好激活下列一臂：
  - raw interval：`t_begin`、`t_end`、raw IMU sample slice；
  - discontinuity declaration：`t_begin`、`t_end`。

上述名称只表达语义，不冻结 concrete C++ type 或 field identifier。caller 不传 GTSAM key、factor、
`Values`、PIM、预计算 `dt` 或 `InitialState`。apps/session 只组合和传输 project-owned POD；预积分、
`X/V/B`、gravity、bias correction、GTSAM parameter/factor、bootstrap accumulator、continuity cursor、
window、segment 与 transaction 全部属于 estimator implementation。public header 继续只暴露项目 POD、
Eigen 与 PIMPL，不暴露 GTSAM 类型。

```text
source normalization ──► sync interval assembly ──► apps pass-through
                              │                           │
                       raw | discontinuity               ▼
                                               VioEstimator::update()
                                         validate → integrate → stage
                                                  → commit/rollback
```

## 3. 运行时合同

### 3.1 State cadence 与静止 bootstrap

bootstrap 成功后，每个具有连续有效 IMU interval 的同步图像 packet 建立一个 `X/V/B` state。
keyframe 只控制 landmark seeding 与 keyframe-aware eviction，不控制导航 state cadence。视觉支持暂时不足
不自动取消该 packet 的 state。

首次 segment 与 measurement discontinuity 后的新 segment 都采用静止 bootstrap：

- `b_g0 = mean(gyro)`；
- `mean(accel)` 确定 roll/pitch；
- `v0 = 0`；
- yaw 保持 gauge；
- accelerometer bias 只接受 zero-mean prior，不声称由单姿态静止窗口完整观测。

静止门首次闭合的同步图像 timestamp 定义 root `t0`。timestamp `<= t0` 的 IMU measurement time span
只属于 bootstrap；不补发更早的 `X/V/B`，也不回放历史 packet。首条 PIM 覆盖 `(t0,t1]`，任何正时长
measurement span 不得同时贡献给 bootstrap 与传播。门闭合前返回 typed `kInitializing` 且无 estimate；
闭门 packet 原子提交 `X0/V0/B0` 并返回 `kOk` 与 estimate。

### 3.2 Raw IMU sample 与 interval

estimator seam 上的 `raw` sample 是 core-normalized、bias-containing 的逐样本测量：

- timestamp 已按已知固定 camera–IMU time offset 对齐，并以 canonical integer nanoseconds 表达；
- accelerometer/gyroscope 已完成 source-specific unit、axis/sign 与已知 deterministic static calibration；
- measurement 使用 SI units，表达在 canonical B/IMU frame；M4 继续采用 B=IMU；
- measurement 仍包含 physical accelerometer/gyroscope bias；source、sync 与 apps 不做 state-dependent
  bias correction；
- sample 尚未进行 endpoint interpolation、quadrature 或 preintegration。

raw interval 必须满足 `t_begin < t_end`，sample timestamps 严格递增，并提供足以 bracket 两个 endpoint
的 support。每个 raw payload 的 `t_end` 必须精确等于当前图像 timestamp；存在 predecessor anchor 时，
`t_begin` 必须精确等于该 anchor。active segment 使用最后一个 committed state timestamp；initializing
phase 使用 estimator 私有的 continuity anchor。所有比较都在 integer-nanosecond domain 中进行，不使用
tolerance、snap 或 endpoint 改写。

对任一 endpoint `tau`：若存在 `t == tau` 的 raw sample，原样采用；否则必须存在唯一相邻 pair
`t_l < tau < t_r`，并对 accelerometer 与 gyroscope 每个分量做线性插值：

```text
alpha  = (tau_ns - t_l_ns) / (t_r_ns - t_l_ns)
q(tau) = (1 - alpha) q_l + alpha q_r
```

不得 extrapolate、取 nearest、sample-hold 或单侧 fallback。estimator implementation 随后私有构造
canonical closed support：`t_begin` endpoint、严格位于 interval 内部的 raw samples、`t_end` endpoint。
其 timestamps 必须严格递增，整数段宽之和必须精确等于 `t_end - t_begin`。

对 canonical support 的每个相邻 node pair，先由正整数纳秒差得到唯一 `dt_i`，再分别取 accelerometer
与 gyroscope endpoint arithmetic mean，并恰好调用一次 PIM `integrateMeasurement()`。`N` 个 nodes
必须产生且只产生 `N-1` 个正时长 integration calls；不得增加 zero-`dt` endpoint call，也不得修改最后
一个 `dt` 来补齐 interval。

### 3.3 Sync continuity classification

future `phad/sync` interval assembly implementation 是 production path 上 raw/discontinuity classification
的唯一 owner。source adapter 只负责 static normalization 与 fixed time alignment；apps 和 estimator 不重做
classification。

对目标 `(t_begin,t_end)`，所有相邻 raw sample pair 中，其 open span 与目标 interval 有正时长交叠者都
属于 continuity check；这同时覆盖 endpoint bracket pair 与 interval 内部 pair。两端 bracket 都存在且每个
相关 pair 的正整数纳秒跨度满足 `delta_ns <= limit_ns` 时，sync 输出 raw arm；缺失 bracket 或任一跨度
`> limit_ns` 时，输出相同 endpoints 的 discontinuity arm。完全位于 interval 外、只接触 endpoint 的 pair
不参与。重复/逆序 timestamp 或不可表示的差值属于 invalid source/input，不伪装成 gap。

`limit_ns` 是概念名，不冻结 concrete config key。它必须是显式提供、可表达的正整数纳秒 fixed duration：

- composition root 从 resolved session sync-policy configuration 取得并传给 sync construction；
- 同一 sync implementation lifetime 内 immutable；
- `delta_ns == limit_ns` 仍连续；
- 不从 nominal `rate_hz`、frame/IMU period 或已观测 cadence 派生，不在线自适应；
- 没有 default、missing fallback、disabled 或 unbounded sentinel；
- production canonical config snapshot 与 `config_hash` 记录最终生效的 exact scalar。

### 3.4 Measurement discontinuity lifecycle

discontinuity arm 只声明未覆盖的 measurement span，不携带 samples、`dt` 或 PIM。estimator 不从 malformed
raw arm 推导 discontinuity。

active committed segment 上，declaration 必须满足：`t_begin < t_end`、`t_begin` 精确等于最后一个
committed state timestamp、`t_end` 精确等于当前图像 timestamp。valid declaration 在一次原子 transaction
中结束 active segment；该 packet 不建立 `X/V/B`、factor 或 estimate。既有 committed output 保全为完成段，
新 segment 重新进入静止 bootstrap；旧、新 segment 之间没有 IMU、bias-RW、visual、landmark 或 posterior
bridge。transition 成功后该次 update one-shot 返回 `kDiscontinuity`，随后合法 raw packets 在新 root 提交前
返回 `kInitializing`。

initializing phase 私有维护 continuity anchor。valid discontinuity 清空全部未提交 bootstrap evidence、把
anchor 推进到该 declaration 的 `t_end`，保持 initializing，并返回 `kInitializing`；该 packet 不贡献静止
证据。cold start 的首个 otherwise-valid payload 建立初始 anchor。caller 不缓存、拼接或恢复 estimator 私有
bootstrap state。

### 3.5 Visual outage lifecycle

active segment 的 IMU interval 连续有效、但视觉支持连续不足时，estimator 允许有限且非零的 IMU-only
coast。horizon 内每个 accepted packet 仍建立 `X/V/B`、IMU factor 与 bias-RW factor；没有可用视觉约束
时不建立 visual factor，也不得伪造、复用或外推 visual observations。视觉支持在 horizon 内恢复时沿用
同一 segment。

horizon 用尽后进入独立的 visual-outage transition，不复用 IMU discontinuity payload 或其 lifecycle
语义。visual-support predicate/owner、horizon 的单位/数值/configuration、boundary-packet result cadence、
transition transaction、re-anchor 与 recovery 是 implementation-plan blocking items，见 §7.1。

### 3.6 Keyframe-aware eviction

window 保留 keyframe-aware eviction。删除 predecessor 与 successor 之间的 non-keyframe 时，estimator
implementation 必须拼接其私有保存的 raw intervals，经 §3.2 同一路径 fresh reintegrate 一条
predecessor→successor PIM，并在同一 transaction 中替换 IMU factor、bias-RW factor 与 window state。
replacement 使用实际 state span、同一 noise mapping 与同一 integration model。

replacement 不得跨 segment，也不表示保留被删 state 的 visual factors、posterior correlation 或完整
marginalization information。shared endpoint 去重、raw provenance/storage、bias linearization point 与具体
transaction ordering 在 implementation plan 中确定。

## 4. Noise 与 construction model

source physical calibration 继续由 [`ImuParameters`](../../phad/sensor/imu_parameters.hpp) 表达
`acc_nd/gyr_nd/acc_rw/gyr_rw` 与 sample-rate metadata。estimator modeling policy 使用独立的
project-owned construction options；两者只在 estimator implementation 的唯一 GTSAM parameter builder 中
组合。apps 不构造 `PreintegrationParams`。

continuous-time white-noise density 到 PIM measurement covariance 的映射为：

```text
accelerometerCovariance = acc_nd² I3
gyroscopeCovariance     = gyr_nd² I3
```

跨度 `DeltaT` 的 `BetweenFactor<imuBias::ConstantBias>` 按 `[b_a,b_g]` 顺序使用：

```text
Q_B(DeltaT) = diag(acc_rw² DeltaT I3, gyr_rw² DeltaT I3)
```

`DeltaT` 由相邻 graph state 的 integer-nanosecond timestamps 求正差后转换为 seconds。
若 constructor 接收 diagonal Sigmas，则分别为 `acc_rw sqrt(DeltaT)` 与 `gyr_rw sqrt(DeltaT)`。caller
不得按 `rate_hz` 或 substep `dt` 预先离散化、重复平方或混淆 covariance/Sigma。

conceptual `q_int` 是 estimator construction 必填的 isotropic continuous-time position-covariance density，
单位 `m²/s`。唯一 builder 设置 `integrationCovariance = q_int I3`；不得平方、按 rate/`dt` 缩放或从四类
sensor density 派生。`q_int` 必须 finite 且 `>= 0`；显式 `0` 合法，并映射为 `Z3`，表示不增加独立的
position-integration/discretization uncertainty。missing、负数、NaN 或 infinity 不能形成可接收 update 的
estimator。

## 5. Result 与 transaction

所有 retained mutation 都先在 estimator implementation 中 stage，再整体 commit。§3 已冻结的 bootstrap、
measurement-discontinuity 与 eviction success paths 不得暴露部分提交；validation failure 不得改变
`X/V/B`、factor、window、trajectory、bootstrap evidence、continuity anchor、raw/PIM cache 或影响后续
行为的 counter。合法输入上的 optimizer/transaction failure rollback 仍由 implementation plan 收口。

本 Draft 冻结以下 conceptual result 语义；concrete enum layout 与 message schema 仍在 implementation plan
中确定：

| Result | Estimate | 语义 |
|---|---:|---|
| `kOk` | 有 | root 或 active packet 已原子提交。 |
| `kInitializing` | 无 | 当前 segment 尚未提交 root；合法 evidence 已累计或初始化期 discontinuity 已重置 accumulator。 |
| `kDiscontinuity` | 无 | active committed segment 已成功完成 measurement-discontinuity transition；one-shot event。 |
| `kInvalidInput` | 无 | payload 违反 interface；observable state 与调用前严格相同。 |

`kInvalidInput` 至少覆盖：非法 endpoint order/arithmetic、raw/declaration anchors 与 packet/predecessor 不匹配、
sample timestamp 重复或倒退、non-finite measurement、缺少两侧 bracket、normalization 无法精确闭合，以及
非 finite/非正 integration `dt`。invalid raw 不自动 skip、跨越传播、切段或 rebootstrap。合法输入上的 optimizer/
transaction failure、visual-outage result 与 caller retry/terminate policy 尚未冻结。

## 6. 最小验收合同

后续获得实施与测试授权时，验收应通过 `VioEstimator` external seam 观察行为，不把 estimator 私有 helper
或 GTSAM graph internals 变成第二套 test surface。最小 deterministic coverage 包括：

1. 静止 bootstrap 在闭门 packet 只提交一个 `X0/V0/B0`，不回放历史 state。
2. 第一条 PIM 只覆盖 `(t0,t1]`，bootstrap 与传播没有重复 measurement time span。
3. exact endpoint 原样采用；缺 exact sample 时用两侧线性 interpolation 构造 canonical support；缺 bracket
   原子拒绝。
4. `N` 个 canonical nodes 恰好形成 `N-1` 个 positive-`dt` mean-measurement integration calls。
5. IMU measurement covariance、bias-RW covariance 与 `q_int` mapping 满足 §4 的单位和公式。
6. 每个连续 raw packet 建立 `X/V/B`、IMU factor 与 bias-RW factor；可用 visual observations 才建立
   visual factor。
7. malformed raw/declaration 返回 `kInvalidInput`，随后相同合法输入的 observable result 不受前次调用影响。
8. valid measurement discontinuity 不建立跨 gap factor，完成段保全，新 root 无 posterior bridge。
9. transient visual outage 在 horizon 内继续 state cadence，恢复后留在同一 segment；horizon exhaustion
   进入独立 transition。
10. non-keyframe eviction 走同一 normalization/noise path fresh reintegrate，并与 factor/window replacement
    原子提交。

本 Draft 不授权执行这些测试，也不授权 replay3、真实轨迹评估或将 #40 evidence 重新归类为 full-state
VIO evidence。

## 7. OPEN 与 deferred

### 7.1 Implementation-plan blocking

以下事项必须在获得实施授权后的 implementation plan 中具体化，但不得扩大 §2 external seam：

- project-owned measurement/result/options 的 concrete C++ types、field names、copy/move ownership 与
  composition-root plumbing；
- visual observation contract、visual-support predicate/owner、有限 coast horizon、horizon-boundary result、
  visual-outage transaction 与最小 recovery/re-anchor 行为；
- static bootstrap window、stationarity gate、gravity、priors、timeout 与数值参数；
- `limit_ns`、`q_int` 与其余 noise/prior 的 concrete values、config keys、parser/serialization 及 construction
  validation delivery；
- state key allocation、landmark 与 `X/V/B` window 关系、raw interval storage、shared endpoint consistency、
  bias linearization point 与 eviction transaction order；
- legal-input optimizer/transaction failure 的 rollback、typed result、reason/message、logging 与 caller policy；
- segment identity 与最小 output representation，以及 current `StereoImuPacket`/sync/estimator callsites 的
  atomic migration sequence。

这些事项由 implementation plan 一次性具体化，并继续受 §2 external seam 约束。

### 7.2 Production hardening（不阻塞首个 vertical slice）

- `limit_ns` 的 default-on production profile approval 最终应引用 pinned timestamp characterization，至少同时
  覆盖 known-contiguous 与 intended-discontinuity pairs，并证明选值位于两者可分离区间。
- intended-discontinuity evidence 可以包含 deterministic sample-deletion bridges 与独立标注的 natural gaps；
  project-owned versioned label record 是 natural-gap authority，不进入 runtime interface。
- profile registry、manifest schema、全量 injection coverage、review/completeness、artifact tooling、approval
  status 与 amendment workflow 延后到 production hardening；首个 slice 只需要显式、可审计的 construction
  value 与 deterministic seam tests。
- dynamic visual-inertial initialization、anisotropic noise、正式 marginalization、跨 segment global trajectory
  stitching、online IMU intrinsic/extrinsic 与 camera–IMU time-offset state 留给后续 milestone。
- 真实数据收益、GT、ATE/RPE、default-on 与全序列 gates 需要独立 evidence authority 和执行许可。

## 8. 参考依据与 authority 迁移

参考 checkout identity：Tassel `812594d669df3a174404872c50765af2bcdad0e4`；slambook2
`e62d9cd1f95bab337fb4251608543b71d004c420`。Tassel 的 full state 与 IMU factor 支持把 velocity 与两类
bias 纳入同一导航状态；不要求 phad 复制其 Ceres、FEJ、marginalization 或动态初始化。slambook2 正文没有
实现 IMU 路线，不能为 full VIO factor/noise/lifecycle 提供证据。旧 graphify 图来自 dirty `7f102cb`，只用于
源码定位，不是 authority 或 evidence。

endpoint interpolation 与 adjacent-pair mean 的工程依据包括 pinned
[Kimera-VIO `ThreadsafeImuBuffer`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/utils/ThreadsafeImuBuffer.cpp)、
[OpenVINS `Propagator`](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp)、
[VINS-Mono propagation](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp#L104-L114)
与 [ORB-SLAM3 preintegration](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L1683-L1725)。
noise mapping 依据 pinned GTSAM
[`PreintegrationParams`](https://github.com/borglab/gtsam/blob/e0a2c93730d31b4b464bb1de25c334a0910998c0/gtsam/navigation/PreintegrationParams.h)、
[`IMUIntegrationCovariance`](https://github.com/borglab/gtsam/blob/e0a2c93730d31b4b464bb1de25c334a0910998c0/gtsam/navigation/doc/IMUIntegrationCovariance.md)
与 [`testImuFactor`](https://github.com/borglab/gtsam/blob/e0a2c93730d31b4b464bb1de25c334a0910998c0/gtsam/navigation/tests/testImuFactor.cpp#L112-L116)。

当前 authority/implementation 与目标合同存在两处已知迁移点：

- [architecture §2.4](../design/architecture.md) 仍让 frontend 仅在 keyframe 提交，并让 public
  `KeyframeMeasurement` 携带 `PreintegratedImu`；目标 seam 改为每个连续同步 packet 提交 raw IMU payload。
- [conventions §5.2](../design/conventions.md) 与当前
  [`StereoImuPacket`](../../phad/sensor/stereo_imu_packet.hpp) 把 endpoint interpolation 放在 sync，并用
  `imu_gap` bool 表达 current M4.1 gap；目标 seam 让 sync 只选择 tagged arm，由 estimator 私有完成 endpoint
  normalization 与 full-state lifecycle。
- 当前 [`UpdateStatus`](../../phad/estimator/types.hpp) 只有 `kOk/kRejected/kFailed`；§5 的 conceptual
  lifecycle results 需要随 estimator 原子迁移，不能被描述为现有行为。

本文不修改或取代 [architecture](../design/architecture.md)、[conventions](../design/conventions.md)、
[estimator README](../../phad/estimator/README.md)、ADR、[roadmap](../design/roadmap.md)、产品代码或测试。实施获得独立授权后，
必须把 estimator rename、旧 `G(k)` 删除、PIM ownership、packet contract 与相关 authority 作为一次受控迁移
共同评审；不得通过 compatibility alias 或并行 backend 分阶段长期共存。

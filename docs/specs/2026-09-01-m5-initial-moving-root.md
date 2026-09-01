# M5 Slice B：estimator-owned initial moving root

- 日期：2026-09-01（Asia/Shanghai）
- 状态：**已定稿**（用户于 2026-09-01 逐项确认 product RED、ownership、evidence、solve、diagnostics 与 formal gates）
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- Wayfinding map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)
- 产品代码基线：`ba607c5738b3c5d9bc9cb0423199eee67648b0b3`
- ADR：[ADR-0003](../adr/0003-estimator-owned-initial-moving-root.md)
- 活架构：[M5 initial moving root](../design/m5-initial-moving-root.md)
- Implementation plan：[2026-09-01_m5_initial_moving_root_51a1b7e2.plan.md](../plans/2026-09-01_m5_initial_moving_root_51a1b7e2.plan.md)
- 产品输入前置：[M5 Slice A checkpoint](../benchmark/m5/tum-vi-product-input-seam_ba607c5_noconfig.md)
- 决策依据：[M5 产品切片能力与回归测试迁移图](../research/2026-08-31-note-m5-product-slice-capability-test-map.md)

本文冻结 #51 的产品行为、错误语义与验收合同。实现必须从本文与对应 implementation plan 派生；本文不声明功能已经实现或 gates 已通过。

## 1. 产品目标与 RED

当 existing static bootstrap 不成立且 moving initialization 已显式启用时，`VioEstimator::update()` 必须在多个真实图像时刻上联合使用 connected metric-stereo evidence 与相邻 raw IMU intervals：

- 证据不足或 candidate 可恢复拒绝：返回 `kInitializing`，不产生 estimate；
- 全部 gates 通过：原子提交 existing graph 优化后的 latest `X/V/B`；
- measurement 或内部合同破坏：返回 typed hard failure，并保持 estimator committed state不变。

产品 RED 同时包含：

1. deterministic exact-state synthetic oracle；
2. EuRoC `V1_02_medium` 首118帧真实 pipeline health/cadence gate。

任一单独通过都不足以完成本片。

## 2. Public interface

唯一 measurement interface 保持：

```cpp
[[nodiscard]] VioUpdateResult update(
    const VioMeasurement& measurement,
    bool keyframe = true);
```

不得新增：

- `initialize()`、`updateImu()`、public reset 或 callback；
- 第二 estimator、backend 或 app-side initialization pipeline；
- public GTSAM key、factor、PIM、matrix 或 candidate state；
- regular CSV/JSON initialization columns。

`VioUpdateResult::estimate` 只有 `status == kOk` 时可以存在。Initial moving root 的 pending 与 recoverable rejection 均返回 `kInitializing`，不使用 `kRejected`。

### 2.1 Accepted camera rotation ordering

`OfflineVoSession` 对同帧 accepted result 必须按以下顺序执行：

1. 由 `update.estimate->T_W_B.linear()` 与
   `calibration.T_B_left_rectified().rotation()` 计算
   `R_W_C = R_W_B * R_B_C_rectified`；
2. 若该帧是 keyframe，keyframe pixel/timestamp/rotation snapshot 使用这个
   current-frame `R_W_C`；
3. `last_accepted_rotation` 更新为同一个 `R_W_C`；
4. 后续 parallax compensation 才可消费该状态。

Non-identity `T_B_left_rectified()` 是该合同的必测输入；body rotation、identity
extrinsic 或上一 accepted frame rotation均不得代替 current accepted camera
rotation。

## 3. Activation 与 static arbitration

### 3.1 Activation seam

Existing seam 原位启用 formal moving initialization：

```text
EstimatorOptions::m_enable_moving_bootstrap
estimator.enable_moving_bootstrap
default = false
```

- `false`：只允许 static initialization；static timeout 返回
  `kFailed/kStaticTimeout`，并 exact rollback 本 packet 到 pre-update estimator
  state。
- `true`：static/moving evidence 并行积累，static-ready 绝对优先；static 未 ready 时 moving path 可以在其自身 gates 满足后尝试，不等待 static timeout。

不得新增 formal-specific activation key或保留旧 moving-root选择。

### 3.2 Legacy path retirement

实现必须删除：

- 旧 `20 ms mean(acc) + v=0 + biases=0` moving-root commit；
- `EstimatorOptions::enable_accumulated_seed`；
- `--estimator-enable-accumulated-seed` parser/wiring；
- 只验证 accumulated synthetic root 的现行 tests。

历史 research/benchmark 继续解释当时行为，不改写历史。

## 4. State ownership 与 frame contracts

### 4.1 Single state/window

`VioUpdateState` 必须继续是唯一 estimation-state owner，并复用一个 bounded `m_window`。不得建立拥有第二份 frames、landmarks、PIM 或 rollback lifecycle 的 stateful initializer。

每个 live provisional frame 必须保存：

- unique frame index；
- integer-ns timestamp；
- validated observations；
- keyframe status；
- optional predecessor frame identity；
- optional normalized raw IMU interval；
- typed visual pose seed/evidence；
- optional committed navigation state。

Live `m_window.front()` 必须同时没有 predecessor 与 IMU interval。每个后续
retained frame 必须同时具有直接 predecessor 与精确覆盖两端 timestamp 的
normalized interval。Selected component materialize 到 isolated candidate 时，最早
selected verified frame成为 gauge root并同样不携带 predecessor/attached interval；
其后 selected states 的 interval由 intervening raw intervals exact splice得到。

Candidate navigation state只能存在 isolated candidate copy，不能写回 live frame。
Live frame 中的 navigation state如果存在，必须已经由成功 current-graph commit
授权。

### 4.2 Pose authority

Implementation 必须以类型区分：

- `VisualPoseSeed`：只供 visual optimizer 初始化；
- `VerifiedVisualPose`：通过 final connected visual graph，可计入 evidence；
- navigation candidate/committed pose。

未对齐 PIM/NavState propagation 不得构造 `VisualPoseSeed`，不得参与 visual pose truth比较，也不得计入 pose floor。

PnP 继续只是 visual proposal。只有 connected metric-stereo graph 完成 final factor mask、per-pose support、fit 与 cheirality 后，pose 才能成为 `VerifiedVisualPose`。

### 4.3 Keyframe semantics

所有具有 `VerifiedVisualPose` 的 time-distinct frames 都可计入 moving evidence，不论 keyframe status。Keyframe status 可以继续控制 existing landmark growth与 window eviction priority，但不得直接授予或剥夺 initialization evidence资格。

## 5. Bounded evidence 与 visual component

### 5.1 Retention

Moving path enabled 时，所有通过基础 measurement contract 的 packets 必须进入 existing bounded window，包括当前未形成 verified pose 的 packet。Temporary visual disconnection不得立即清除 frames或landmarks。

Window capacity 必须沿用 existing deterministic policy；中间 frame eviction 必须 exact splice normalized intervals并重新验证 predecessor/endpoint/duration provenance。State 不得随初始化时长无界增长。

### 5.2 Latest-containing component

Candidate 只能来自 final retained stereo factors 构成的、包含 current/latest processed `VerifiedVisualPose` 的 maximal connected frame-landmark component。

- Current/latest frame 未 verified：保持 `kInitializing`，不 attempt。
- Visual gauge root：该 component 中最早的 verified frame。
- Product latest root：该 component 中 latest frame 的最终 graph state。
- Component frames 可以时间不连续；selected frames 间的所有 intervening raw intervals 必须 exact splice。
- 不包含 latest frame 的 component 永不授权 commit。

### 5.3 Population 与 support

必须独立检查：

1. `connected_landmarks.actual`：final-mask latest component 中唯一 positive-disparity `LandmarkId` 数；required 取 `min_seed_observations` 的现有数值 authority。
2. `visual_support.actual`：所有 selected non-gauge poses 的 unique retained stereo-factor count最小值；required 取 `min_pnp_inliers` 的现有数值 authority。

实施必须证明 existing option 与新计数单位等价；无法证明时停止，不得静默重解释或引入补偿转换。Left-only/mono factors不计入这两个 moving-init结构量。

## 6. Evidence revision 与 attempt cadence

Current frame 完成 verified admission，且本 update 的 connectivity、component membership、factor mask、eviction与 interval splice全部结算后，`evidence_revision` 必须单调增加一次。

- Unverified packet 不增加 candidate-eligible revision。
- 每个 revision 最多一次 deterministic candidate attempt。
- `last_attempted_evidence_revision` 执行 at-most-once 合同。
- 内部 GLS/refinement iterations不增加 revision或 `attempt_id`。
- `attempt_id` 只在真实 attempt 时存在，是 rollback snapshot外的单调 audit identity。

同一 evidence不得搜索多个 gauge roots、frame subsets、factor masks或 seeds。

## 7. Eligibility 与 visual gates

最早 attempt 必须同时满足：

```text
connected verified poses >= 6
first/latest verified timestamp span >= 250'000'000 ns
connected metric landmarks >= derived min_component_landmarks
weakest non-gauge pose stereo support >= derived min_pose_stereo_support
final visual fit passed
cheirality passed
latest processed frame is VerifiedVisualPose
```

Count/duration exact boundary通过。5 poses/250 ms、6 poses/<250 ms、latest未verified或 component不覆盖latest均不得分配 `attempt_id`。

### 7.1 Final visual mask、fit 与 cheirality

Visual evidence admission必须执行：

1. 对 provisional latest-containing component 的 unique positive-disparity stereo
   factors运行 gauge-fixed visual LM；
2. 对每个 factor的 underlying Gaussian noise执行whitening，不施加robust
   weight，计算 `q_f = ||r_f^w||^2`；
3. 第一轮仅保留
   `q_f <= chi_square_3(0.99) = 11.344866730144373` 的 factors；
4. 以该mask重建graph并恰好re-solve一次；
5. 第二轮在同一mask上重算`q_f`，不得继续remask或搜索；任一factor超限返回
   recoverable `kVisualFitRejected`；
6. 以second-solve final mask重新检查latest connectivity、unique landmark
   population与所有non-gauge poses的weakest support，并映射到各自typed reason；
7. 每个retained factor的landmark在对应rectified-left camera中必须finite且
   depth `> 0`；finite non-positive depth返回recoverable
   `kCheiralityRejected`，non-finite返回fatal `kContractError`。

两轮中每个factor evaluation、underlying noise model、whitened residual与
`q_f`都必须finite且dimension合法。任何evaluation/whitening exception、invalid
noise或non-finite必须立即映射
`kContractError/kFatalError/kFailed/kRolledBack`；不得把该factor从mask移除后
报告visual fit、connectivity、population或support不足。

`m_visual_fit` 的规范值为 second solve 后 worst retained factor：

```text
statistic = max(q_f)
limit = 11.344866730144373
effective_dof = 3
```

Final mask为空或无法保持structure/support时，先由对应visual structure reason
拒绝，`m_visual_fit=nullopt`。不得把PnP proposal RMS、current-graph
`outlier_avg_reproj_px`或aggregate visual Q替代该factor-level admission。

## 8. Observability gates

对 `N` 个 selected poses，分别构建：

| System | Required rank |
|---|---:|
| gyro bias | 3 |
| gravity tangent + per-frame velocities | `3N+2` |

两个 systems 必须分别：

1. 使用 adopted covariance whitening；
2. 对每个 finite nonzero column 做 L2 normalization；
3. 对 normalized Jacobian直接做 thin SVD；
4. 以 `tau = 1e-10` 计算 numerical rank，仅当
   `sigma_i > tau * sigma_max` 时计入 rank；
5. 达到 required rank后，以降序第 `required_rank` 个 singular value
   `sigma_required` 计算
   `condition = sigma_max / sigma_required <= 67'108'864`。

判定顺序固定为：先检查 whitened residual、Jacobian与column norms的
finiteness，再检查finite zero-norm column，再 normalization/SVD、rank与
condition。任何non-finite返回
`kContractError/kFatalError/kFailed/kRolledBack`；finite zero-norm column返回
recoverable `kRankDeficient/kRecoverableRejected/kInitializing/
kEvidenceCommitted`。Singular value等于relative threshold不计入rank；condition
等于limit通过。其余rank deficient与ill-conditioned也属于attempt-local
recoverable rejection；不得用raw integrated rotation、acc variance或column
norm magnitude替代这些gates。

## 9. Candidate solve

### 9.1 Sequential seed

每个 moving attempt 从 `b_g=0`、`b_a=0` 开始：

1. 用相邻 verified visual rotations与 PIM bias Jacobians 解三维 gyro bias；
2. 以更新 bias 从 raw normalized intervals重建 fresh PIM；
3. stereo scale固定为1，解 fixed-magnitude gravity tangent与每帧 velocity。

不得把 static gyro mean、moving gyro mean或 rejected candidate bias作为 moving seed/hint。

### 9.2 Joint refinement

Sequential seed 后最多执行8个 correction rounds：

- 每轮以 current `b_g` 从 raw intervals重建 fresh PIM；
- 构建 canonical rotation/position/velocity residual、candidate Jacobian与 innovation covariance；
- 执行 whitened GLS；
- `physical_delta.infinity_norm <= 1e-8` 时收敛；
- 否则更新 gyro bias、gravity tangent与所有 velocities，并把 gravity投影回 fixed magnitude。

Non-finite correction是 fatal contract error。达到 iteration cap仍不收敛是 recoverable `kNoConvergence`。Final gate前必须在 final `b_g` 上重新积分；只用 first-order bias correction不合格。

### 9.3 Final joint consistency

Final residual：

\[
r = [r_R, r_p, r_v]_{0:N-2}
\]

Innovation covariance：

\[
S = H_v C_v H_v^T + C_{pim}
\]

- `C_v`：同一 final masked visual graph、root gauge fixed、QR non-root pose joint marginal covariance，必须保留 off-diagonal pose blocks。
- `C_pim`：final-bias fresh PIM `preintMeasCov()`，跨 interval block diagonal。
- Visual/IMU independence与 interval block diagonal是本片 adopted approximations。

QR marginal、每个PIM covariance block、assembled `S`与supported-subspace
factorization必须通过dimension/order、finiteness、adopted symmetry/PSD checks。
任何non-finite、invalid covariance或factorization exception均映射
`kMoving / kJointConsistency / kFatalError / kContractError /
kRolledBack / kFailed`，不得映射为evidence insufficiency、rank deficiency或
joint inconsistency。

使用 supported-subspace factorization/SVD计算：

\[
Q = r^T S^+ r,\qquad
\nu = rank(S)-rank(S^{-1/2}J)
\]

Full-rank `ν = 6N-14`。必须满足：

\[
\nu > 0,\qquad Q \le \chi^2_\nu(0.99)
\]

ν 不大于0是 recoverable `kJointDofInsufficient`；Q超限是 recoverable `kJointInconsistent`。分组 residual统计只作diagnostics，不执行额外hard veto。

## 10. Current graph authority

Joint-consistent candidate 必须 materialize 到 isolated `VioUpdateState` 并执行 existing current graph完整路径，包括：

- bounded navigation provenance；
- key/value/factor/root-prior ownership validation；
- full `X/V/B`、IMU、bias RW、visual graph；
- primary LM；
- final values/finiteness/cheirality/visual quality；
- existing cull/reopt lifecycle。

只有 validated graph-optimized latest `X/V/B` 可以提交。Initialization candidate只是 graph seed。Graph output不重复执行 shared-bias initialization chi-square。

Graph build/invariants全部通过后：结构合法但约束不足返回 recoverable
`kCurrentGraphUnderconstrained`；finite solve完成但 existing
visual-quality/cull-reopt admission拒绝 candidate返回 recoverable
`kCurrentGraphRejected`。两者都映射为
`kRecoverableRejected/kInitializing/kEvidenceCommitted`。Missing prior、
ownership/topology/provenance错误、non-finite、generic/unknown exception均为
fatal `kContractError`。

## 11. Transaction contract

基础 measurement/interval validation 通过后建立一个默认 rollback 的 packet transaction：

| Path | Live state result | Status |
|---|---|---|
| no attempt / pending | evidence committed | `kInitializing` |
| recoverable attempt rejection | evidence committed，candidate discarded | `kInitializing` |
| success | validated graph state atomically installed | `kOk` |
| fatal | exact pre-update estimator state restored | `kFailed` |
| moving-disabled static timeout | exact pre-update estimator state restored | `kFailed` |

Pre-staging measurement contract violation继续返回 `kInvalidInput`，transaction不进入 evaluated状态。

Candidate必须是 isolated value state，不得让未授权 `X/V/B` 临时成为 live owner。Success install必须是 guaranteed/noexcept swap或等价原子替换。Fatal rollback后，attempt audit按值保留在 result envelope，existing topology diagnostics从restored state重建。

## 12. Typed initialization diagnostics

`UpdateDiagnostics` 必须增加 always-present `InitializationDiagnostics m_initialization`。Top-level lifecycle：

```text
InitializationPath:
  kNone | kStatic | kMoving

InitializationPhase:
  kInactive | kCollectingEvidence | kVisualSolve | kGyroAlignment |
  kGravityVelocitySolve | kJointConsistency | kCurrentGraph | kCommit

InitializationOutcome:
  kNotActive | kPending | kRecoverableRejected | kCommitted | kFatalError
```

Stable reasons：

```text
kNone
kStaticEvidenceInsufficient | kStaticTimeout
kPoseCountInsufficient | kDurationInsufficient
kVisualConnectivityInsufficient | kVisualPopulationInsufficient
kVisualSupportInsufficient | kVisualFitRejected | kCheiralityRejected
kRankDeficient | kIllConditioned | kNoConvergence
kJointDofInsufficient | kJointInconsistent
kCurrentGraphUnderconstrained | kCurrentGraphRejected
kContractError
```

Summaries：

- connected poses actual/required；
- duration actual/required ns；
- connected landmarks actual/required；
- optional visual support actual/required；
- optional visual fit statistic/limit/DoF；
- optional gyro rank、condition/limit；
- optional gravity/velocity rank、condition/limit；
- optional joint fit statistic/limit/DoF；
- current graph result：`kNotEvaluated | kValidated | kRejected`；
- transaction result：`kNotEvaluated | kEvidenceCommitted | kRootCommitted | kRolledBack`。

Presence rules：

- `path=kNone` 或 `path=kStatic` 时，三个 always-present moving summaries均为
  `actual=0, required=0`；该值的 non-applicable 语义由 path给出；
- moving collection开始后，connected summaries填真实值；
- static path的 moving-only optional summaries保持 `nullopt`；
- `attempt_id` 只在真实attempt后存在；
- optional group只有对应gate实际评估后存在；
- rank deficient导致condition未定义时，rank存在、condition为`nullopt`；
- 禁止NaN、sentinel数值或message表达absence。

`kPending` 只允许尚未attempt。存在`attempt_id`的可恢复失败必须是`kRecoverableRejected`。除 `kNotActive` 与 `kCommitted` 外，所有 outcomes必须有非`kNone`稳定reason；`kNotActive`与`kCommitted`必须使用`kNone`。合法组合由exhaustive validation tests冻结。

Canonical legal combinations：

| 场景 | path / phase / outcome | reason | attempt | graph / transaction | status |
|---|---|---|---|---|---|
| initialization未参与该update | `kNone / kInactive / kNotActive` | `kNone` | absent | `kNotEvaluated / kNotEvaluated` | existing non-init mapping |
| static collecting | `kStatic / kCollectingEvidence / kPending` | `kStaticEvidenceInsufficient` | absent | `kNotEvaluated / kEvidenceCommitted` | `kInitializing` |
| static timeout | `kStatic / kCollectingEvidence / kFatalError` | `kStaticTimeout` | absent | `kNotEvaluated / kRolledBack` | `kFailed` |
| static root | `kStatic / kCommit / kCommitted` | `kNone` | absent | `kValidated / kRootCommitted` | `kOk` |
| moving pre-attempt gate | `kMoving / gate phase / kPending` | 对应 evidence reason | absent | `kNotEvaluated / kEvidenceCommitted` | `kInitializing` |
| moving attempt rejection | `kMoving / rejecting phase / kRecoverableRejected` | 对应 attempt reason | present | evaluated result / `kEvidenceCommitted` | `kInitializing` |
| moving root | `kMoving / kCommit / kCommitted` | `kNone` | present | `kValidated / kRootCommitted` | `kOk` |
| moving fatal | `kMoving / failing phase / kFatalError` | `kContractError` | iff attempt已分配 | verdict若已形成，否则`kNotEvaluated` / `kRolledBack` | `kFailed` |

`kCurrentGraphUnderconstrained` 与 `kCurrentGraphRejected` 的 graph result均为
`kRejected`；前者由合法 structure上的约束不足触发，后者由 finite
quality/admission verdict触发。

Fatal rollback时，`m_initialization`描述current update audit，`m_transaction=kRolledBack`；existing window/graph diagnostics描述restored committed state。Graph gate尚未形成verdict时，`m_current_graph=kNotEvaluated`。

Raw matrices、singular values、column norms、solver trace、fingerprints、precision radii、component attribution与revision counters不得进入public interface或persistent schema。`message`只作人类文本。

## 13. Status mapping

| `InitializationOutcome` / failure stage | `UpdateStatus` | estimate |
|---|---|---|
| `kPending` | `kInitializing` | absent |
| `kRecoverableRejected` | `kInitializing` | absent |
| `kCommitted` | `kOk` | graph latest state |
| pre-staging measurement contract | `kInvalidInput` | absent |
| post-staging/internal fatal | `kFailed` | absent |
| moving-disabled static timeout | `kFailed` | absent |

Initial moving root不得增加existing rejected/seed-rejected counters。

## 14. Deterministic acceptance

### 14.1 Private mathematics

必须使用independent expected values覆盖：

- canonical residual/Jacobian与GTSAM factor对拍；
- gravity tangent independent central difference；
- covariance whitening/supported-subspace factorization；
- rank/condition exact boundary；
- GLS dense oracle；
- fresh-PIM bias refinement；
- 8-round convergence/no-convergence；
- Q、DoF与chi-square equality/nextafter边界。

### 14.2 Public static arbitration

必须覆盖：

1. Default `m_enable_moving_bootstrap=false` 的 stationary fixture 走 existing static path，提交 `b_g≈mean(gyr)`、fixed-gravity pose、`v=0`，下一 active update成功。
2. Default false 的 non-stationary input到达static timeout时返回`kFailed/kStaticTimeout`，不产生moving attempt或estimate，transaction为`kRolledBack`，post-call committed state与该packet前一致。
3. `m_enable_moving_bootstrap=true` 的同一stationary fixture仍由static path在moving floor前提交，结果与static-only control在预注册容差内一致。
4. Moving enabled时static timeout不使整体失败；后续verified evidence仍可形成moving attempt。

### 14.3 Public moving success

无随机解析fixture必须具有：

- exactly 6 frames / 250 ms；
- 分段解析的非交换三轴旋转；
- 时变三维加速度；
- `b_a=0`、known nonzero `b_g=[0.01,-0.02,0.03] rad/s`；
- 所有 selected frames具有充足、分散、正深度的metric stereo observations；
- 两个 normalized systems与Q/visibility margins由independent oracle证明远离阈值。

成功误差门：

```text
latest translation <= 1e-3 m
latest rotation    <= 1e-4 rad
latest velocity    <= 1e-3 m/s
gyro bias          <= 1e-4 rad/s
gravity direction  <= 1e-4 rad
```

资格前必须`kInitializing/nullopt`；success时`kOk`并提交latest graph state；下一active update继续成功。

### 14.4 Orthogonal negative mutants

由success fixture一次改变一个因果边，覆盖：

- 5 poses/250 ms；6 poses/<250 ms；
- latest disconnect、population不足、weakest-pose support不足；
- visual fit、cheirality；
- gyro与gravity/velocity rank deficiency；
- full-rank ill-condition；
- no-convergence；
- joint supported-subspace effective DoF不大于0；
- full-rank/healthy-condition joint inconsistency；
- current graph underconstraint；
- current graph finite quality/admission rejection；
- non-finite whitened residual/Jacobian/column norm、QR/PIM/S covariance或
  supported-subspace factorization的 fatal rollback；
- visual factor evaluation/whitening/non-finite `q_f` fatal rollback；
- fatal graph invariant/non-finite rollback。

每个mutant必须证明所有更早gates有裕量地通过，并断言最早typed reason、status、attempt presence、transaction result与下一clean revision恢复。Public tests不得把任意失败当作pass。

`OfflineVoSession` 另以 non-identity `T_B_left_rectified()` 覆盖 §2.1：同帧
accepted camera rotation必须先进入 keyframe snapshot，再成为下一帧 parallax
compensation 的 last-accepted rotation。

## 15. Real product gates

### 15.1 `V1_02-118`

固定 `V1_02_medium` ASL-native sequence root及首118个 image frames。Run必须使用：

```text
sequence/GT root = /home/lin/Projects/data/thidparty/euroc/native/V1_02_medium
max_frames = 118
estimator.enable_moving_bootstrap = true
eval.max_dt_ms = 2.5
eval.min_match_rate = 0.5
eval.rpe_delta_s = 1.0
ATE alignment = existing fixed-scale SE3 alignSe3
```

Canonical manifest由sequence root内
`LC_ALL=C find . -type f -print0 | sort -z | xargs -0 sha256sum`生成；其完整
text的预注册SHA-256为
`a80c02d6c9bd9e5cd76412f30b6859fd8acdabf0885de1cba494725a183ced8f`。
Run前必须重算并逐字匹配该digest；artifact verifier还必须按manifest逐文件复核
`meta.json.sequence_root`。`meta.json`记录exact code/config identity，checkpoint
同时保存manifest text/hash、canonical config text与evaluator settings。Config
hash以该次canonical text为authority，不从历史artifact推断。

必须满足：

```text
session error = none
image_frames = diag rows = emitted stereo = 118
failed = rejected = 0
trajectory finite and present
segments = 1
completion >= 0.90
coverage >= 0.90
ATE translation RMSE <= 1.0 m
RPE(1 s) translation RMSE <= 1.0 m
```

Session contract定义每个`diag.status == ok` row恰好追加一个同
`common::Timestamp` trajectory pose，其他status不追加；first ok row即 initial
root。Artifact verifier把TUM seconds换算为ns后以`<=500 ns` serialization容差与
diag timestamp一一匹配。基于该映射，first ok前全部initializing；first ok不早于
第6个emitted row，且与首input timestamp跨度不少于250 ms；first ok后全部ok；
first ok与last frame timestamps进入trajectory。不得把emitted row解释为
`VerifiedVisualPose`。

### 15.2 Fresh MH_01 static control

在任何candidate结果可见前，以exact`ba607c5`、clean isolated build、同一dataset/config/evaluator环境生成并重复fresh control。冻结repeatability envelope、manifest/calibration、`default_0337287b`与ATE/RPE association。

Candidate必须：

- first ok早于第6个emitted row且距first row小于250 ms；之后全部ok；
- error none、rejected/failed 0、segments 1、reanchors 0；
- finite trajectory；completion/coverage不低于fresh control；
- ATE/RPE不高于control加预注册numerical comparison tolerance。

默认comparison tolerance为`1e-6 m`；只有在查看candidate前发现control repeatability超过该值时，才允许按实测抖动上界重新冻结。`c999f58`只作historical sanity reference。旧`phad_apps_mh01_test`不进入formal denominator。

### 15.3 TUM VI corridor1-100

完整保留#53 loader/input structural predicates，并新增：

```text
failed = rejected = 0
trajectory finite and present
segments = 1
reanchors = 0
```

External cadence沿用§15.1的 ok-to-trajectory映射与`<=500 ns` artifact容差：first ok前全部initializing；first ok不早于第6个diag row且距first input timestamp不少于250 ms；之后到第100帧全部ok；trajectory pose count等于ok row数；first ok与last frame timestamps存在。

该gate不声明moving branch identity，不新增TUM VI GT、ATE/RPE、room sequence或full-sequence benchmark。

## 16. Regression matrix 与停止条件

Formal acceptance按以下顺序执行：

1. compile与完整`-L unit`；
2. private mathematics与public synthetic tests；
3. `V1_02-118`；
4. TUM VI corridor1-100；
5. MH_01 camera/frontend与`OfflineVoSessionTest.*` controls；
6. fresh MH_01 full static regression。

任一gate失败即停止，保存actual status、typed reason、attempt/transaction、最早分叉、input/config/evaluator identity与artifact。一次只修正一个被证实因果边并原位重跑；不得为求绿调整冻结阈值、扩大EuRoC序列或实现后续能力。

## 17. Scope boundary

本片不交付：

- post-outage cold-root recovery；
- cross-root world-frame continuity；
- posterior physical precision admission或covariance model selection；
- moving initialization default-on；
- EuRoC-11 formal benchmark；
- TUM VI ground truth、ATE/RPE或full sequence；
- persistent initialization diagnostics schema；
- second estimator/backend或compatibility path。

## 18. 完成定义

仅当以下条件全部满足，#51 才可形成Slice B checkpoint：

1. §§2–13所有interface、state、solve、error与diagnostics contracts已实现。
2. §14 deterministic positive/negative oracles全部通过。
3. §15三个真实产品gates通过，并保存可复现identity与actual结果。
4. §16完整unit/controls通过；陈旧non-gating control的既有事实准确记录。
5. 旧moving-root与accumulated-seed initial-root paths已删除，无第二activation seam。
6. Diff完整审查，无用户既有或本片无关修改混入实现交付。
7. Checkpoint记录code/tree/config/dataset/evaluator、命令、实际结果、未执行验证与后续边界。

全部满足后立即停止。后续能力按独立Slice重新走design/spec/plan与授权。

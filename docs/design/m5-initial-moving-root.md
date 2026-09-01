# M5 initial moving root

本文档描述当前约定，不是绝对约束，会随项目开发修订。

- 状态：**已定稿（目标设计；尚未实现）**
- 日期：2026-09-01（Asia/Shanghai）
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- ADR：[ADR-0003](../adr/0003-estimator-owned-initial-moving-root.md)
- 产品输入前置：[M5 Slice A checkpoint](../benchmark/m5/tum-vi-product-input-seam_ba607c5_noconfig.md)
- 决策依据：[M5 产品切片能力与回归测试迁移图](../research/2026-08-31-note-m5-product-slice-capability-test-map.md)

本文定义 initial moving root 的活架构：在 static bootstrap 不成立时，同一 `VioEstimator::update()` 使用 connected metric stereo 与相邻 raw IMU intervals，建立一致的 latest `X/V/B`，或返回仍缺少的 evidence。它不是 implementation plan，也不声明当前代码已经具备该能力。

## 1. 目标与范围

产品目标是建立一条可拒绝、可诊断、可回滚的 moving-start initialization：

```text
validated packets
  → verified connected visual evidence
  → gyro-bias alignment
  → final-bias fresh PIM
  → fixed-scale gravity / per-frame velocity closure
  → joint consistency
  → existing current graph validation
  → atomic graph-optimized latest X/V/B commit
```

本设计只处理进程启动后的 initial root。Visual outage 后的 cold-root recovery、跨 root world-frame continuity、posterior physical precision admission 与 moving initialization default-on 不在本设计内。

## 2. Module 与 interface

`VioEstimator` 继续是 deep module。External interface 保持：

```cpp
[[nodiscard]] VioUpdateResult update(
    const VioMeasurement& measurement,
    bool keyframe = true);
```

Caller 不拥有或控制：

- initialization candidate；
- visual component 与 factor mask；
- GTSAM graph、keys、PIM 或 solve order；
- evidence revision、attempt cadence 或 rollback；
- static/moving arbitration。

`apps::OfflineVoSession` 继续逐图像调用 `update()`。`keyframe` 是 visual map growth 与 existing window policy 的输入，不决定 moving evidence frame 是否具有资格。

当同帧 `update()` 返回 `kOk` 时，session 必须先以
`R_W_C = R_W_B * R_B_C_rectified` 计算 accepted rectified-left-camera
rotation，再用该 rotation 写入 keyframe snapshot 和
`last_accepted_rotation`。`R_B_C_rectified` 必须来自
`T_B_left_rectified()`；snapshot 不得读取上一 accepted frame 的 rotation。

## 3. Canonical terms

| 术语 | 定义 |
|---|---|
| `VisualPoseSeed` | 只供 visual optimizer 初始化的 pose；可来自 visual gauge、先前 verified visual pose 或通过现有仲裁的 PnP proposal，不具备 evidence authority。 |
| `VerifiedVisualPose` | 已通过 connected metric-stereo graph 的 final mask、per-pose support、fit 与 cheirality 检查的 pose。只有该类型可计入 moving evidence。 |
| visual gauge root | Selected visual component 中最早的 verified frame；只固定 visual graph gauge。 |
| product latest root | Successful initialization 后发布的 latest processed frame 的 graph-optimized `X/V/B`。 |
| evidence revision | Current processed frame 完成 `VerifiedVisualPose` admission，并结算本 update 的 connectivity、factor mask、eviction 与 interval splice 后形成的一次 candidate-eligible evidence snapshot。 |
| candidate attempt | 从一个 evidence revision 唯一、确定性地构造并评估一次 isolated initialization candidate。 |

Visual gauge root 与 product latest root 是不同角色。任何可提交 candidate 必须覆盖 current/latest processed frame，且该 frame 本身必须是 `VerifiedVisualPose`。

## 4. State ownership 与 bounded lifecycle

### 4.1 单一 owner

`VioUpdateState` 继续是唯一 estimation-state owner，并持有单一 bounded window。`WindowFrame` 保留：

- `frame_index`、timestamp；
- observations 与 keyframe status；
- optional predecessor identity；
- optional `NormalizedImuInterval`；
- typed visual pose evidence/provenance；
- optional committed navigation state。

Live bounded window 的 front/root 必须同时没有 predecessor 与 IMU
interval；每个 non-root retained frame 必须同时拥有直接 predecessor 与
精确覆盖两端 timestamp 的 normalized raw interval。构造 selected candidate
时，最早 selected verified frame 是 visual gauge root，在 isolated candidate
view 中同样没有 predecessor/attached interval；其后的 selected states 才携带
由 intervening raw intervals exact splice 得到的 interval。该 rebase 不改写 live
evidence provenance。

Typed fields 必须防止 `VisualPoseSeed`、`VerifiedVisualPose` 与 navigation pose 互换使用。Live frame 只保存 evidence 与已提交 navigation state；未授权 candidate navigation state 只存在 isolated local copy。PIM/NavState rollout 可用于该 candidate 的数值初始化，但不得进入 visual pose seed、visual truth 比较或 live owner。

### 4.2 Bounded retention

Moving path enabled 时，所有通过基础 measurement 合同的 packets 进入同一个 bounded window，包括当前尚未形成 `VerifiedVisualPose` 的 packet。Visual connectivity 只决定谁可以 attempt；window capacity 决定 evidence 保留多久。

容量超限沿用 existing policy：

1. 优先逐出最老 non-keyframe；
2. 必要时逐出 front；
3. 中间 frame eviction 通过 `spliceNormalizedImuIntervals()` exact 合并相邻 raw provenance；
4. rebase root，并重新验证 predecessor/interval closure；
5. 仅清除不再被任何 window observation 引用的 landmarks。

暂时不属于 latest visual component 的 frames/landmarks 不立即删除，以允许后续重新连通。初始化状态始终受 existing `window_size` 约束。

### 4.3 Evidence revision 与 attempt cadence

Current frame 的 visual solve、final mask/support/fit/cheirality、component membership、eviction/splice 全部完成后，若 current frame 成为 `VerifiedVisualPose`，则 `evidence_revision` 单调增加一次。

Unverified packet 可以改变 bounded retention，但不形成可尝试 revision。每个 revision 最多一次 deterministic attempt；`last_attempted_evidence_revision` 防止同一 evidence 重算。GLS/refinement 的内部 iterations 属同一个 attempt。

`attempt_id` 只为真实发生的 attempt 分配，是 rollback snapshot 外的单调 audit identity；它不参与 candidate 输入，也不进入 persistent schema。

## 5. Static/moving arbitration

Moving path enabled 时，uninitialized updates 同时积累 static IMU statistics 与 moving visual/raw-interval evidence，并固定按以下顺序仲裁：

1. 先评估 existing static gate；
2. static ready 时只执行 existing static root path；
3. static 未 ready 且 moving path enabled 时继续 moving evidence admission/attempt；
4. moving path 不等待 static timeout；
5. moving path enabled 时，static timeout 只结束 static 候选，不使 estimator 整体失败；
6. moving path disabled 时，existing static-only timeout 继续返回 fatal `kStaticTimeout`。

Moving path disabled 时不得形成可提交 moving candidate 或推进 moving evidence revision。Static commit 后清除 provisional moving lifecycle。Static path 的 stationary `mean(gyr)`、gravity tilt 与零初速合同保持不变。

## 6. Visual evidence admission

### 6.1 Seed 与 verified result

Visual initialization 分为两个 internal contracts：

```text
visual gauge / previous verified visual pose / accepted PnP proposal
  → VisualPoseSeed
  → connected metric-stereo graph
  → final factor mask
  → per-pose support + fit + cheirality
  → VerifiedVisualPose
```

PnP 继续只是 proposal。只有 final connected visual graph output 可进入 pose count、duration、gyro alignment 或 gravity/velocity solve。

Visual fit admission 采用固定两次 solve：

1. 对 provisional latest-containing component 的所有 unique positive-disparity
   stereo factors执行一次 gauge-fixed visual LM；
2. 对每个 factor使用其 underlying Gaussian noise whitening（不使用 robust
   weight），计算 `q_f = ||r_f^w||^2`；三维 stereo factor仅在
   `q_f <= chi_square_3(0.99) = 11.344866730144373` 时进入 retained mask；
3. 只用 retained mask重建 graph并恰好 re-solve一次；
4. 对同一 retained mask重新计算 `q_f`，不得继续 remask/search；任何 factor
   超限返回 recoverable `kVisualFitRejected`；
5. Final-mask connectivity、population与weakest-pose support重新检查；
6. 每个 retained factor 的 landmark在其 rectified-left camera中必须具有 finite
   positive depth；finite non-positive depth返回 recoverable
   `kCheiralityRejected`，non-finite返回 fatal `kContractError`。

两轮中每个factor的evaluation、underlying noise、whitened residual与`q_f`都
必须finite且dimension合法；任一exception或non-finite立即返回
`kContractError/kFatalError/kFailed/kRolledBack`，不得通过mask移除而降级为
visual fit/structure rejection。

`m_visual_fit` 报告第二次 solve 后 worst retained-factor
`{statistic=max(q_f), limit=chi_square_3(0.99), effective_dof=3}`。Mask为空时先由
visual structure gate拒绝，fit summary保持 absent。Visual group aggregate可留在
private trace，不成为第二个 hard veto。

### 6.2 Latest component

在 final retained stereo factors 构成的 frame-landmark bipartite graph 中，选择包含 current/latest verified frame 的 maximal connected component：

- visual gauge 固定在该 component 中最早的 verified frame；
- selected frames 按 timestamp 排序；
- component 可以在时间上不连续；
- selected poses 间覆盖所有 intervening packets 的 normalized intervals 按 endpoint 顺序 exact splice；
- 不属于该 component 的 frames 不计 eligibility，也不阻塞该 component；
- 不覆盖 latest verified frame 的历史 component 永不授权 product latest root。

### 6.3 Population 与 support

两个视觉结构量独立门控：

| Summary | actual | required |
|---|---|---|
| connected landmarks | Final-mask latest component 中唯一 positive-disparity `LandmarkId` 数 | `EstimatorOptions::min_seed_observations` 的数值 authority |
| visual support | Selected non-gauge poses 中 unique retained stereo factors 的最小值 | `EstimatorOptions::min_pnp_inliers` 的数值 authority |

Moving implementation 立即把两个 existing option values 派生为语义明确的只读 internal requirements。实施必须用 tests 证明计数单位等价；不成立时停止并回到设计对齐。

Left-only/mono factors 不进入 moving-init metric-stereo population、support 或 visual covariance。

## 7. Eligibility 与 observability

### 7.1 Candidate attempt floor

最早 attempt 同时要求：

- latest selected component 至少 6 个 time-distinct `VerifiedVisualPose` frames（包含 visual gauge root 与 latest frame）；
- first/latest selected timestamps 跨度不少于 `250'000'000 ns`；
- visual connectivity、population、support、fit 与 cheirality 全部通过。

Count 与 duration 是 conjunction。Exact boundary 通过；任一不足时保持 `kInitializing`，不分配 `attempt_id`。

### 7.2 Observability systems

对 `N` 个 selected poses，两个 system 独立评价：

| System | Unknowns | Required rank |
|---|---|---:|
| gyro bias | 三维 `b_g` | 3 |
| gravity/velocity | gravity tangent 2-DOF + `N` 个三维 velocity | `3N+2` |

每个 system 使用相同 numerical procedure：

1. 使用 adopted covariance whitening residual/Jacobian；
2. 所有 finite nonzero columns 做 L2 normalization；
3. 对 normalized Jacobian 直接做 thin SVD；
4. 以 `tau = 1e-10` 计算 numerical rank：仅当
   `sigma_i > tau * sigma_max` 时该 singular value 才计入 rank；
5. 达到 required rank 后计算
   `kappa = sigma_max / sigma_required`，其中 `sigma_required` 是第
   `required_rank` 个降序 singular value；
6. `normalized_condition_limit = 1 / sqrt(double epsilon) = 67'108'864`。

Whitened residual、Jacobian或column norm出现non-finite时返回 fatal
`kContractError/kFailed/kRolledBack`。Finite zero-norm column返回 recoverable
`kRankDeficient/kInitializing/kEvidenceCommitted`。其余columns完成
normalization/SVD后，relative singular value等于rank threshold不计入rank。
Full rank但condition超限为ill-conditioned；condition等于limit通过。两个
systems必须分别通过，不以`max(condition)`代替逐system verdict。

Column norms 与 singular spectrum 只用于 private tests/diagnostics，不成为 physical excitation 或 posterior precision gate。

## 8. Candidate solve

### 8.1 Canonical seed

每个 moving evidence revision 从 canonical (b_g=0) 开始。Static gyro mean、上次 rejected candidate bias 或 moving-window gyro mean均不进入 moving candidate authority。

Initial sequential closure：

1. 由 adjacent verified visual rotations 与 PIM gyro-bias Jacobians 解 gyro bias；
2. 以该 bias 从 selected raw intervals 构建 fresh PIM；
3. 在 stereo scale 固定为 1、`b_a=0` 的前提下解 fixed-magnitude gravity tangent 与每帧 velocity。

### 8.2 Bounded joint refinement

Sequential seed 后最多执行 8 个 joint correction rounds。每轮：

1. 以当前 `b_g` 从 raw intervals 重建 fresh PIM；
2. 构建 canonical rotation/position/velocity residual 与 candidate Jacobian；
3. 构建 adopted innovation covariance并执行 whitened GLS；
4. physical correction vector 满足 `infinity_norm <= 1e-8` 时收敛；
5. 否则更新 `b_g`、gravity tangent 与所有 velocities；
6. gravity 重新投影到 fixed magnitude；
7. 下一轮在更新后的 state 上重新闭合。

Non-finite correction 是 fatal numerical contract error。8 rounds 后仍不收敛是 recoverable `kNoConvergence`。最终 consistency 与 graph validation使用 final-`b_g` raw reintegrated PIM；不以 `biasCorrectedDelta()` 的一阶值冒充 final PIM。

### 8.3 Final joint consistency

Final residual 对所有 selected intervals 拼接：

\[
r_{joint} = [r_R, r_p, r_v]_{0:N-2}
\]

Innovation covariance：

\[
S = H_v C_v H_v^T + C_{pim}
\]

- `C_v`：同一 final masked connected-stereo graph，root gauge 固定后以 QR 得到的 non-root pose joint marginal covariance；保留跨 pose off-diagonal blocks。
- `C_pim`：final-bias fresh PIM `preintMeasCov()`；跨 intervals 采用 block-diagonal white-noise/PIM approximation。
- Visual 与 IMU independence 是本片 adopted model assumption。

`C_v` QR marginal、每个 `C_pim` block、assembled `S` 与 supported-subspace
factorization必须具有合法dimension/order、finite entries与adopted
symmetry/PSD tolerance。任一合同失败或factorization异常均返回 fatal
`kContractError/kFailed/kRolledBack`，不得降级为rank/evidence不足。

使用 symmetric rank-revealing factorization / SVD 在 supported subspace 中求解，不显式构造 `S^-1`：

\[
Q = r_{joint}^T S^+ r_{joint}
\]

\[
\nu = rank(S) - rank(S^{-1/2} J_{candidate})
\]

Full-rank 情形下 residual dimension 为 `9(N-1)`，fitted variables 为 `3N+5`，因此 `ν = 6N-14`；最小 `N=6` 时 `ν=22`。

Gate 为：

\[
Q \le \chi^2_{\nu}(0.99)
\]

ν 不大于 0 返回 recoverable `kJointDofInsufficient`；Q 超限返回 recoverable `kJointInconsistent`；等于 limit通过。Gyro 与 position/velocity 分组统计只作 diagnostics，不增加额外 veto。该 per-attempt 0.99 gate 不声明全生命周期 false-reject rate或 posterior physical precision已校准。

## 9. Current graph 与 transaction

### 9.1 Current graph authority

通过 joint consistency 的 candidate materialize 到 isolated `VioUpdateState`，随后运行 existing current graph 完整路径：

- bounded window rebase/IMU splice；
- graph build 与 key/value/factor/root-prior ownership validation；
- `X/V/B`、`ImuFactor`、bias random walk 与 visual factors；
- primary LM；
- final value/finiteness/cheirality/visual quality；
- existing cull/reopt lifecycle。

Validated graph-optimized latest `X/V/B` 是唯一最终提交对象。Initialization candidate 只是 graph seed。Graph output 不再套用 shared-bias initialization chi-square，因为 current graph 已允许 per-frame bias random walk与不同 final visual mask；其 topology、solver 与 quality checks 构成独立 current-graph gate。

### 9.2 Packet transaction

基础 measurement/interval contract通过后创建一个默认 rollback 的 packet RAII transaction：

1. Live transactional state 接收 packet、visual evidence、eviction/splice 与 evidence revision；
2. Candidate 从 live evidence复制为 isolated local state；
3. Pending/no-attempt：commit evidence，返回 `kInitializing`；
4. Recoverable rejection：丢弃 candidate，记录 last-attempted revision，commit evidence，返回 `kInitializing`；
5. Success：以 validated graph state 原子替换 live evidence state，commit，返回 `kOk`；
6. Fatal：rollback 到 pre-update state，返回 `kFailed`。

Moving-disabled static timeout 也走 fatal rollback：本 packet、bootstrap
mutation、window/evidence/revision均不提交，返回
`kFailed/kStaticTimeout`。`attempt_id` 与 current-update failure audit 位于
rollback snapshot外。Candidate 不得保存指向 live containers 的
reference/pointer/iterator。

### 9.3 Current graph failure taxonomy

Graph build/invariants 通过后分两类 recoverable verdict：结构合法但约束不足返回 `kCurrentGraphUnderconstrained`；finite solve完成但 existing visual-quality/cull-reopt admission拒绝 candidate返回 `kCurrentGraphRejected`。两者都丢弃 candidate、commit本 update evidence并返回 `kInitializing`。Missing prior、key/value/factor ownership、predecessor/interval、topology、non-finite、generic optimizer exception 与未分类 library exception均为 fatal。

## 10. Typed initialization diagnostics

`UpdateDiagnostics` 增加 always-present value aggregate：

```cpp
struct InitializationDiagnostics
{
  InitializationPath    m_path    = InitializationPath::kNone;
  InitializationPhase   m_phase   = InitializationPhase::kInactive;
  InitializationReason  m_reason  = InitializationReason::kNone;
  InitializationOutcome m_outcome = InitializationOutcome::kNotActive;
  std::optional<std::uint64_t> m_attempt_id;

  InitializationCountSummary    m_connected_poses;
  InitializationDurationSummary m_duration;
  InitializationCountSummary    m_connected_landmarks;
  std::optional<InitializationCountSummary> m_visual_support;
  std::optional<InitializationFitSummary> m_visual_fit;
  std::optional<InitializationRankSummary> m_gyro_rank;
  std::optional<InitializationThresholdSummary>
      m_gyro_normalized_condition;
  std::optional<InitializationRankSummary> m_gravity_velocity_rank;
  std::optional<InitializationThresholdSummary>
      m_gravity_velocity_normalized_condition;
  std::optional<InitializationFitSummary> m_joint_fit;

  InitializationGraphResult       m_current_graph;
  InitializationTransactionResult m_transaction;
};
```

Lifecycle enums：

```text
Path:    None | Static | Moving
Phase:   Inactive | CollectingEvidence | VisualSolve | GyroAlignment |
         GravityVelocitySolve | JointConsistency | CurrentGraph | Commit
Outcome: NotActive | Pending | RecoverableRejected | Committed | FatalError
```

Stable reasons：

```text
None
StaticEvidenceInsufficient | StaticTimeout
PoseCountInsufficient | DurationInsufficient
VisualConnectivityInsufficient | VisualPopulationInsufficient
VisualSupportInsufficient | VisualFitRejected | CheiralityRejected
RankDeficient | IllConditioned | NoConvergence
JointDofInsufficient | JointInconsistent
CurrentGraphUnderconstrained | CurrentGraphRejected
ContractError
```

Finalization results：

```text
CurrentGraph: NotEvaluated | Validated | Rejected
Transaction:  NotEvaluated | EvidenceCommitted | RootCommitted | RolledBack
```

Summary helpers只包含 project-owned标量：actual/required、value/limit、statistic/limit/effective DoF。Raw matrix、residual/Jacobian/covariance、singular spectrum、column norms、solver trace、fingerprint、precision radius、component attribution与 evidence revisions不进入 public interface。

Top-level path/phase/outcome/reason 始终显式存在。Optional summary只表示该 gate是否实际评估；禁止 NaN 或 message充当 absence sentinel。`path=None` 或 `path=Static` 时，三个 always-present moving summaries规范化为 `actual=0, required=0`，其 non-applicable 语义由 path显式给出；moving collection开始后填真实 actual/required。Static path的 moving-only optional summaries保持 absent。

Existing topology diagnostics 在 fatal rollback 后从 restored committed state重建；`m_initialization` 是 current-update audit，并以 transaction result说明是否生效。Fatal发生在 graph gate形成前时 `m_current_graph=NotEvaluated`。

## 11. `UpdateStatus` mapping

| Initialization outcome | `UpdateStatus` | estimate |
|---|---|---|
| Pending | `kInitializing` | absent |
| RecoverableRejected | `kInitializing` | absent |
| Committed | `kOk` | graph-optimized latest state |
| pre-staging measurement contract error | `kInvalidInput` | absent |
| post-staging/internal fatal | `kFailed` | absent |
| moving-disabled static timeout | `kFailed` | absent |

Initial moving root不使用 `kRejected`，也不增加 `UpdateStatus` values。`VioUpdateResult::message` 继续是人类文本，不作为 parser或测试合同。

## 12. Activation 与退役

`EstimatorOptions::m_enable_moving_bootstrap` 与 `estimator.enable_moving_bootstrap` 保持唯一 activation seam，default `false`：

- `false`：static-only，timeout fatal；
- `true`：static-first + formal moving initialization，static timeout不终止 moving collection。

旧短时 `mean(acc)` moving root path删除。`enable_accumulated_seed` 及其 CLI wiring/tests只服务旧 initial-root Gate F，在 formal evidence window启用后完整删除。相同 config value必须与 code commit共同解释，不声明跨版本算法 identity。

## 13. Verification architecture

验证分两层：

1. Estimator-private pure mathematics tests验证 residual/Jacobian、central difference、covariance whitening、supported-subspace factorization、SVD rank/condition、dense GLS oracle、fresh-PIM refinement与 Q/DoF。
2. Public `VioEstimator::update()` sequence tests验证 eligibility、static precedence、typed outcomes、evidence retention、fatal rollback、graph commit与下一 active update。

Expected values来自无随机解析真值，不调用 production helper反算。Success fixture使用 exactly 6 frames/250 ms、非交换三轴旋转、时变三维加速度、`b_a=0` 与已知非零 `b_g`。Negative mutants一次只改变一个因果边并断言最早 typed reason。

真实产品门为：

- `V1_02_medium` 首118帧 moving RED；
- `ba607c5` fresh MH_01 static control；
- TUM VI `corridor1_512_16` 首100帧 product root/continuity gate。

Formal matrix通过即形成 Slice B checkpoint并停止，不扩为 EuRoC-11 gate。

## 14. 后续边界

Slice B checkpoint 之后，按独立问题继续：

1. Post-outage cold-root recovery；
2. Cross-root world-frame continuity；
3. Posterior physical precision admission；
4. Moving initialization default-on。

这些能力不得成为 initial moving root implementation 或验收的隐含 predecessor。

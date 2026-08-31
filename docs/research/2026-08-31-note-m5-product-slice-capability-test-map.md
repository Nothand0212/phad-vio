# M5 产品切片能力与回归测试迁移图

本文档描述当前分析结果，不是 ADR、spec、implementation plan 或实施授权。

- 日期：2026-08-31（Asia/Shanghai）
- 状态：**已确认（capability/test map；非 spec、plan 或实施授权）**
- 产品基线：`main@46a84b589a2ca49d56f8502641abd5897f3bb39e`
- 产品基线 tree：`6d362adc929b1dc03b242baee930e9332075d9c3`
- M4 产品代码锚点：`34c309196440710be86d0a05e32901d58bfdd9aa`
- 只读 donor：`181360667b1bf73d82bd7c7dc45ee05be4745b99`
- donor tree：`ea4a095754096af38999a1ebe504a809f3f04c14`
- Wayfinding map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)
- Slice A child：[#53](https://github.com/Nothand0212/phad-vio/issues/53)（TUM VI product input seam）
- Slice B child：[#51](https://github.com/Nothand0212/phad-vio/issues/51)（estimator-owned initial moving root）
- Slice A spec：[2026-08-31-m5-tum-vi-input-seam.md](../specs/2026-08-31-m5-tum-vi-input-seam.md)
- Slice A plan：[2026-08-31_m5_tum_vi_input_seam_7b0a8974.plan.md](../plans/2026-08-31_m5_tum_vi_input_seam_7b0a8974.plan.md)

`main@46a84b5` 已包含最新 agent 工作约定；其 `CMakeLists.txt`、`apps/`、`phad/` 与 estimator/camera/frontend 产品测试相对 `34c3091` 没有产品行为增量。本文以该 main 为唯一新分支基线。donor 仅用于读取已经验证过的行为、源码与测试，不作为 cherry-pick 范围或产品 ancestry。

## 1. 产品目标与边界

M5 首个产品闭环只交付 **initial moving root**：当 static bootstrap 不成立时，同一 `VioEstimator::update()` 在多个真实图像时刻上联合使用 metric stereo 与相邻 raw IMU interval，诚实地产生一致的最新 `X/V/B`，或返回仍需什么 evidence。

长期架构保持：初始化由 estimator 拥有；调用方不拥有 candidate、GTSAM graph、rollback 或求解顺序。算法顺序为：

```text
connected metric stereo
  → gyro-bias alignment
  → fresh PIM
  → fixed-scale gravity / per-frame velocity closure
  → existing GTSAM graph validation
  → atomic latest-root commit
```

首个产品闭环不同时承担 post-outage cold-root recovery 或跨-root world-frame continuity。两者在 initial moving root checkpoint 后分别立项。

## 2. 核心矛盾卡

### 2.1 Slice A：TUM VI 图像进入现有双目 pipeline

| 问题 | 当前事实 |
|---|---|
| roadmap 出口 | TUM VI handheld moving start 能完成初始化 |
| 用户可观察 RED | TUM VI 数据集可以打开，但原始图像不能进入现有 rectified stereo/frontend 路径 |
| 最早失败边界 | 实际 product caller 中，`OfflineVoSession` 与 frontend probe 固定调用 `euroc::open()`；直接使用 TUM VI adapter 后，`StereoRectifier::create()` 继续拒绝 equidistant，且 `rectify()` / `StereoTracker` 只接受 `uint8` |
| 本片唯一干预 | 闭合现有 product input path：composition root 显式选择已存在的 dataset adapter，再由 camera-owned 整图路径兑现 `uint16 raw → rectified uint8` 与 equidistant stereo geometry 合同 |
| 真实 product endpoint | 至少一个 TUM VI handheld sequence 的双目帧通过既有 sync/session 进入 frontend |
| hard gates | 原始 `phad::io` 图像深度保持不变；像素转换显式且确定；`T_B_left_rectified` 方向正确；EuRoC radtan/uint8 路径不回归 |
| 先观察的风险 | TUM VI 像素有效位宽与强度分布作为 observe-only 输入诊断；不改变固定映射，也不做逐帧自适应调参 |

OpenCV `cv::fisheye::stereoRectify()` 与 `cv::fisheye::initUndistortRectifyMap()` 是本片优先复用的成熟实现。`StereoRectifier` implementation 私有拥有 raw pixel canonicalization；其 interface 接受支持的 raw 灰度双目输入并产出 rectified `uint8` stereo。`uint8` 输入保持现有 native-depth remap；`uint16` 输入先以 native depth 完成 geometric remap，再在最终 rectified 输出处按 `dst = static_cast<uint8_t>(src >> 8)` 量化。该映射等价于 `floor(src / 256)`：低 8 位直接截断，合法 `uint16` 输入无需额外饱和；`convertTo(CV_8U, 1.0 / 256.0)` 使用最近整数舍入，不满足本合同。

错误 interface 保持 `CameraModelResult<T>` / `CameraModelError` 与现有三类 code。调用方可修复的 model、size、channel 与 pixel-type 合同拒绝使用 `kOutsideModelDomain`；OpenCV exception 或生成结果失效使用 `kNumericalFailure`。`detail` 提供 `create` / `rectify` stage、`left` / `right` side、expected / actual 与原始库原因；它是人类诊断文本，不是可解析 schema。

Slice A 不新增 regular diagnostics 字段；现有 `timing.rectify` 继续记录长期成本。raw `uint16`、rectified `uint8` 与 frontend 实际消费的 conversion-path 证据，以及强度分布等 observe-only 信息，进入定向测试或 opt-in acceptance artifact。

Dataset adapter selection 位于 app composition root：调用方显式执行 `euroc::open()` 或 `tum_vi::open()`，再把格式中立的 `StereoImuDataset` 传给 `runOfflineVoSession()`。Session 只运行共同的 replay → sync → rectify → frontend → estimator pipeline。

Initialization-only prefix 是成功运行但尚无 estimator 轨迹的 session 结果：`error = nullopt`、`trajectory = nullopt`。需要轨迹的 probe / bench composition root 继续把缺失 trajectory 判为产品失败；Slice A 的 input-seam gate 不把 `kInitializing` 或无 trajectory 误报成 session hard error。

### 2.2 Slice B：initial moving root

| 问题 | 当前事实 |
|---|---|
| roadmap 出口 | 无静止段时建立可信 `X/V/B`；失败不返回伪成功；MH static 路径不退化 |
| 用户可观察 RED | main 的 moving bootstrap 不能提供正式多帧 gyro bias、gravity 与 initial velocity 闭合 |
| 最早失败边界 | estimator initialization state 只有 static fast path 与粗 moving fallback，没有 time-aware multi-frame candidate |
| 本片唯一干预 | 在既有 `VioEstimator` 内加入 initial-only provisional evidence 与最小一致性求解/提交路径 |
| 真实 product endpoint | exact synthetic state、`V1_02-118` tight loop、MH_01 static regression 与一个 TUM VI handheld moving start |
| hard gates | 时间/坐标/单位、verified visual pose、fresh PIM、final gravity/velocity closure、joint residual consistency、current graph validation、atomic rollback |
| 先观察的风险 | posterior physical precision、500 ms propagation budget 与 component attribution 先留在 diagnostics/research，不阻塞本片 |

Slice B 不设置独立 raw integrated-rotation 或 accelerometer-variance hard threshold。Pose count、duration 与 visual connectivity 达到 eligibility 后，以 gyro-bias 和 gravity/velocity candidate systems 的 full required rank 与 normalized condition limit 判断 observability / conditioning；不足保持 `kInitializing`。Raw motion summaries 只作 failure diagnostics。Column-normalized condition 不证明绝对 posterior precision，final joint consistency、current-graph validation 与 atomic commit 仍须独立通过。

Candidate solve 的最早 attempt floor 为：同一 visual connected component 内至少 6 个 time-distinct pose-bearing evidence frames（包含 root），且首末 timestamp 跨度至少 250 ms。Count 与 duration 取 conjunction；不足保持 `kInitializing`。该 floor 是预注册 engineering 下界，不声明成功、observability 或 precision；实施测试须覆盖 5/250、6/<250 与 6/250 的精确边界。

Final joint consistency 使用最小 same-metric covariance authority：visual covariance 来自同一 masked connected-stereo graph 在 root gauge 固定后的 non-root pose QR joint marginals；inertial covariance 来自 final gyro bias 下 fresh PIM 的 `preintMeasCov()`。Innovation covariance 为 `S = H_v C_v H_v^T + C_pim`，joint whitened statistic 按 effective DoF 使用 `chi-square(0.99)` hard veto。`C_pim` 的跨 interval block-diagonal 结构是首片 adopted white-noise / PIM 建模近似；相邻 normalized intervals 复用 shared endpoint，因此 raw numeric support 不是字面独立。统计解释只在该 adopted model 内成立，不声明真实 false-reject rate 或 posterior physical precision 已校准。

## 3. Main 已有能力：直接复用

| 能力 | main authority | 使用方式 |
|---|---|---|
| 单一 estimator measurement interface | `phad/estimator/vio_estimator.hpp`：`VioEstimator::update()` | 保持不变；不新增 `initialize()`、callback 或第二入口 |
| full-state `X/V/B`、landmark 与 active graph lifecycle | `phad/estimator/vio_estimator.cpp`、`internal/vio_update_transaction.hpp` | 作为 successful candidate 的唯一安装目标 |
| packet 级原子 transaction | `internal::VioUpdateTransaction` | 直接复用 clone/commit/rollback；不复制第二套 transaction |
| raw IMU interval normalization / splice | `internal/imu_interval.{hpp,cpp}` | 作为 evidence interval 与 fresh reintegration 的输入合同 |
| static bootstrap fast path | `VioEstimator` 当前 bootstrap lifecycle | 必须保持；moving path 只在 static 不成立时累积 evidence |
| GTSAM graph build / solve / validation | estimator private implementation | candidate 最终必须经过既有 graph seam，不新增 backend |
| TUM VI loader 与 equidistant calibration parsing | `phad/io/dataset/tum_vi/` | 保留 raw `uint16`；不在 dataset adapter 静默截断 |
| radtan rectification 与 uint8 frontend | `phad/camera`、`phad/frontend` | Slice A 扩展同一深 module；EuRoC 行为作为回归 control |
| offline session / benchmark product caller | `apps/offline_vo_session.*`、`phad_vo_bench` | composition root 打开 concrete adapter 后，作为 Slice A/B 的共同真实 caller；不建立第二条实验 pipeline |

## 4. Donor 产品能力：按行为选择性迁移

下表的 “donor symbol” 只用于定位已经实现过的行为。新实现从 main 演进，不得整体 cherry-pick donor commit 或照搬其 public diagnostics surface。

| 必须保留的行为 | donor symbol / path | 迁移方式 | 目标 slice |
|---|---|---|---|
| 每个 evidence frame 保留 timestamp、observations、keyframe、predecessor 与对应 IMU interval | `WindowFrame`、`VioEstimator::Impl::admitProvisionalFrame()` | 在 main 的 private `VioUpdateState` 上最小扩展；只保留 initial moving 所需状态 | B |
| pre-root PIM rollout 不拥有 visual pose truth | `WindowFrame::m_visual_T_W_B`、verified predecessor / PnP proposal | 迁移 invariant 与 tests；允许 root gauge、已验证前驱或视觉 consensus，禁止未对齐 PIM pose 直接进入 visual solve | B |
| connected metric stereo visual solve | `prepareStereoViInitialization()` | 迁移 estimator-private solve；不公开 GTSAM keys、marginals 或 graph 顺序 | B |
| canonical final-state residual 与 same-metric iterative GLS | `buildCanonicalQualificationSystem()`、`solveQualificationGeneralizedLeastSquares()`、`qualifyCandidate()` | 迁移 residual/Jacobian/fresh-PIM/gravity-velocity closure；posterior budget decision 不进入首片 | B |
| final candidate 变换到 gravity-aligned world | `materializeStereoViCandidate()` | 保留 final gravity、velocity、landmark 与 pose 的同一 transform；`b_a=0` 仍是首片明确假设 | B |
| isolated candidate/current graph validation | donor `VioEstimator::updateUnchecked()` M5 path | 接入 main 既有 graph/transaction；recoverable evidence 不足返回 `kInitializing`，contract/graph invariant 破坏 hard fail | B |
| 同帧 accepted camera rotation 先于 keyframe snapshot | donor `apps/offline_vo_session.cpp` accepted rotation block | 迁移 ordering invariant；使用 `T_B_left_rectified()`，覆盖 non-identity extrinsic | B |
| compact typed initialization result | donor phase/reason/outcome taxonomy | 重新设计小 interface，只表达稳定产品语义；内部 solver/provenance 细节不外泄 | B |

### 4.1 不从 donor 固定下来的内容

- `kMinimumInitializationFrames = 3` 与 `0.1 s` 不是产品 evidence floor；新 spec 须结合 sensor rate、synthetic observability 和一手实现预先定义 conservative pose/duration/excitation 门。
- full-rank / healthy condition 不是 candidate 成功的充分条件；final-state joint residual consistency 与 current-graph validation 必须保留。
- posterior physical budgets、500 ms propagated precision admission、component attribution 和 covariance-model selection 不作为首片 predecessor。
- attempt fingerprint、failure capture tap、formal receipt 与完整 replay schema 不进入 regular estimator/session interface。

## 5. 回归测试迁移图

### 5.1 Main 现有回归：继续作为 control

- static root 建立 `X/V/B` 与下一帧成功更新；
- raw interval invalid / discontinuity / propagation failure 的 transaction rollback；
- 500 ms visual coast、active-map support、eviction 与 reintegration；
- `VioEstimator::update()` public failure/status 合同；
- EuRoC radtan rectification、frontend 与 offline session 既有 suites。

### 5.2 Donor 测试：迁移语义或改写

| donor test | 产品不变量 | 处理 |
|---|---|---|
| `VioFullState.MovingRootCommitsLatestExactStereoViStateAfterTemporalEligibility` | exact moving pose/velocity/gravity/bias 与 latest-root cadence | 迁移；evidence floor 由新 spec 提供 |
| `ColdRootRepair.JointQualificationAlignsTiltedGravityWithFreshPim` | tilted gravity、fresh PIM 与 final velocity closure | 迁移并改名为 initial-moving 语义 |
| `VioFullState.TimeDistinctDisconnectedStereoPacketsCannotBecomeCotemporalRoot` | time-distinct connected visual evidence | 迁移 |
| `VioInitialization.ValidatedKeyframeAndNonKeyframesRemainDistinctProvisionalEvidence` | frame identity、keyframe 与 interval 归属 | 迁移 |
| `VioInitialization.RootRequiresThreeTimeDistinctConnectedFrames` | root 必须等到正式 evidence floor | **改写**为 spec-owned frame/duration/excitation 门，不保留 exact 3 |
| `VioInitialization.DisconnectedSparsePacketsDoNotPoolIntoRootPopulation` | disconnected evidence 不得伪造 root | 迁移 |
| `CandidateQualificationTest.CanonicalSystemMatchesGtsamResidualAndJacobians` | canonical residual/Jacobian 与 GTSAM 对拍 | 迁移 |
| `CandidateQualificationTest.GravityTangentJacobianMatchesIndependentCentralDifference` | gravity tangent 符号与尺度 | 迁移 |
| `CandidateQualificationTest.GeneralizedLeastSquaresMatchesDenseOracleAndImprovesWhitenedCost` | solve metric 与 gate metric 一致 | 迁移 |
| `CandidateQualificationTest.IterativeRefinementReintegratesBiasAndClosesPerturbedCandidate` | final bias 后 fresh PIM、final gravity 后 velocity closure | 迁移 |
| donor recoverable/fatal transaction cases | evidence retained、candidate discarded、active state 不泄漏 | 选择最小 public-seam cases 迁移；不照搬 fault/fingerprint harness |
| `OfflineVoSessionTest.DynamicRootSnapshotsAcceptedCameraRotationWithNonIdentityExtrinsic` | accepted camera rotation 与 keyframe snapshot 同帧 | 迁移 |
| `m5_v1_02_prefix_contract.py` | fixed endpoint tight product RED | **重写**为 schema-independent product predicate，不继承 83/112-column header |

### 5.3 Slice A 新增测试

采用三层 gate：

1. deterministic camera unit oracle：equidistant stereo projection / rectification、`uint16 → uint8` 高位提取边界、typed failure、rectified row alignment、positive baseline 与 `T_B_left_rectified`；
2. EuRoC radtan / `uint8` control：现有 rectification 与 frontend reference 行为不变；
3. bounded real TUM VI product prefix：固定 `corridor1_512_16` 从首帧开始的 100 个 stereo frames；composition root 打开 dataset，经共同 `runOfflineVoSession()` 到达 frontend。必须满足 `error = nullopt`、`image_frames = 100`、`diag.size() = 100`、`sync.emitted_stereo = 100`、左右 drop / overflow 全零、`failed = 0`、累计 `num_observations > 0` 且累计 `num_disparity > 0`。不门控 trajectory、`kOk` 数量、initialization verdict 或 ATE。

若固定 prefix 首次失败，保存对应结构字段与 opt-in artifact 并诊断最早失败边界；不得静默延长 prefix、换序列或追加经验 coverage threshold。

## 6. 首片 candidate hard gates 与 observe-only 证据

| 分类 | 首片处理 |
|---|---|
| timestamp、interval identity、frame/单位、calibration、finite/PSD contract | hard gate |
| pose count、duration、visual connectivity | spec-owned eligibility gate；不足为 `kInitializing` |
| visual inlier support / fit、cheirality | hard gate |
| gyro bias → fresh PIM → gravity/velocity iterative closure | candidate solve invariant |
| gyro 与 gravity/velocity full required rank、normalized condition/limit | observability / conditioning hard gate；不足为 `kInitializing`，但通过不单独构成成功 |
| final joint residual statistic/limit/effective DoF | 使用 visual joint marginal + fresh-PIM block-diagonal innovation covariance 与 `chi-square(0.99)`；只在 adopted model 内作 hard veto |
| isolated current graph build/solve/validation | hard gate |
| latest-root transaction commit / rollback | hard gate |
| posterior radii、physical budgets、500 ms propagated precision | observe-only；不改变本片 verdict |
| component covariance/attribution、model selection | research |

## 7. Compact typed initialization diagnostics

`UpdateDiagnostics` 内新增 project-owned nested `InitializationDiagnostics` POD/enums，由 `VioUpdateResult` 按值返回。首片不追加 `diag.csv` columns、不扩 `summary.json`，也不提升现有 persistent schema version；复杂 residual、covariance、fingerprint、solver trace 与 replay receipt 只进入 estimator-private tests 或未来独立 versioned opt-in failure artifact。

最小 helper summaries 为 project-owned POD；可放在 `InitializationDiagnostics` 内部，placement 不改变 interface 语义：

```cpp
struct InitializationCountSummary
{
  std::uint32_t m_actual   = 0;
  std::uint32_t m_required = 0;
};

struct InitializationDurationSummary
{
  std::int64_t m_actual_ns   = 0;
  std::int64_t m_required_ns = 0;
};

struct InitializationFitSummary
{
  double       m_statistic     = 0.0;
  double       m_limit         = 0.0;
  std::int64_t m_effective_dof = 0;
};

struct InitializationRankSummary
{
  std::uint32_t m_actual   = 0;
  std::uint32_t m_required = 0;
};

struct InitializationThresholdSummary
{
  double m_value = 0.0;
  double m_limit = 0.0;
};
```

`InitializationDiagnostics` 的稳定字段为：

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

  std::optional<InitializationFitSummary>       m_visual_fit;
  std::optional<InitializationRankSummary>      m_gyro_rank;
  std::optional<InitializationRankSummary>      m_gravity_velocity_rank;
  std::optional<InitializationThresholdSummary> m_normalized_condition;
  std::optional<InitializationFitSummary>       m_joint_fit;

  InitializationGraphResult m_current_graph =
      InitializationGraphResult::kNotEvaluated;
  InitializationTransactionResult m_transaction =
      InitializationTransactionResult::kNotEvaluated;
};
```

Enums 只表达稳定产品语义：

| enum | values |
|---|---|
| `InitializationPath` | `kNone`、`kStatic`、`kMoving` |
| `InitializationPhase` | `kInactive`、`kCollectingEvidence`、`kVisualSolve`、`kGyroAlignment`、`kGravityVelocitySolve`、`kJointConsistency`、`kCurrentGraph`、`kCommit` |
| `InitializationReason` | `kNone`、`kPoseCountInsufficient`、`kDurationInsufficient`、`kVisualConnectivityInsufficient`、`kVisualSupportInsufficient`、`kVisualFitRejected`、`kCheiralityRejected`、`kRankDeficient`、`kIllConditioned`、`kNoConvergence`、`kJointInconsistent`、`kCurrentGraphRejected`、`kContractError` |
| `InitializationOutcome` | `kNotActive`、`kPending`、`kRecoverableRejected`、`kCommitted`、`kFatalError` |
| `InitializationGraphResult` | `kNotEvaluated`、`kValidated`、`kRejected` |
| `InitializationTransactionResult` | `kNotEvaluated`、`kCommitted`、`kRolledBack` |

Default / optional 合同：

- Always-present enums、count / duration summaries 与 result enums 使用上面的显式 default；`kNone` / `kInactive` path 下 actual / required 为 `0 / 0`。
- Moving path 收集 evidence 时，`m_connected_poses`、`m_duration` 与 `m_connected_landmarks` 填 actual / required；static path 不制造 moving-evidence 数字。
- `m_attempt_id` 只在首次 candidate attempt 后存在。
- `m_visual_support` 只在 visual-support gate 实际评估后存在。
- `m_visual_fit`、两个 rank summary、condition summary 与 `m_joint_fit` 均为 whole-group optional；禁止部分填充。
- `0 / 0` 只表示 `kNone` / `kInactive` 的明确定义 default；已进入 moving path 后，不用它模拟“未评估”。Optional whole groups 只用 `std::nullopt` 表示 absence，禁止 NaN 或特殊 enum 组合。`VioUpdateResult::message` 保持人类诊断文本，不作为解析或测试合同。

covariance matrices、full residual/Jacobian、component attribution、posterior radii、coverage multipliers、physical limits、fingerprint、solver-nearby identity 与 replay receipt 留在 estimator-private tests、opt-in failure artifact 或独立 research output。 tests 应从 compact product interface 断言行为，不把内部求解阶段重新投影成长期 CSV columns。

## 8. Research snapshot 边界

以下资产保留其科学与审计价值，但不迁入首个 product slice：

| snapshot | 保留价值 | 产品边界 |
|---|---|---|
| precision study （ 1.48M rows，`coverage_failed` ） | 证明短 prefix nonlinear velocity / propagation coverage gap | 不产生 physical budget，不授权 precision admission |
| D11 formal discovery （ manifest `2fdf847a...` ） | component/end-to-end covariance attribution 与 short-prefix availability | `candidate_token=null`；不选择 product model 或 evidence minimum |
| aborted calibration audit | 固定已消费 seed 与无科学效力的 partial rows | 不恢复、不复用 authorization；未来研究使用新 namespace |
| candidate failure bundle / replay machinery | 复杂失败的可审计重放 | 首片没有稳定 product consumer；需要真实重复失败后再评估迁移 |
| `phad.vo_diag.v112` | 历史 research artifact 的解释 authority | 不成为新产品 schema 的兼容前缀 |

## 9. Slice 顺序与停止条件

```text
Slice A  TUM VI raw image → rectified uint8 stereo
  ↓ product input seam 可达
Slice B  initial moving root
  ↓ initial product checkpoint
Slice C  post-outage cold-root recovery
  ↓ root availability checkpoint
Slice D  cross-root world-frame continuity
```

Slice A 是 Slice B 的**产品验证 predecessor**，不是 estimator 源码依赖。若开发资源允许并行，B 可用 synthetic/EuRoC 开发，但不得在 A 合入并完成 TUM VI product run 前声明 M5 产品出口完成。

每片共同停止条件：

1. tight RED 未按预测改善时停止，不扩数据集、不调 threshold；
2. 新机制没有真实 product caller 时停止，不继续增加 runner/schema/protocol；
3. 需要 posterior budget 或 covariance model 才能解释失败时，先证明失败已经稳定落在 uncertainty surface；
4. 当前片验收通过即停止，后续能力进入下一 issue/spec。

## 10. 正式文档与 issue 的后续顺序

Issue topology 已落地：#42 保持 M4→M5 trajectory-continuity wayfinding map；#53 为 Slice A TUM VI product input child；#51 为 Slice B estimator-owned initial moving root child。两者均已建立 native sub-issue relation。Post-outage cold-root recovery 后续另建 child。Slice B 的 TUM VI product verification 以前置 Slice A checkpoint 为条件，但 synthetic / EuRoC 开发不设置整票 blocking dependency。

后续执行顺序：

1. [x] 为 #53 冻结 Slice A spec 与 implementation plan；
2. [x] 实现并验收 TUM VI product input seam，形成 #53 checkpoint；
3. [ ] 从 latest main 与 confirmed map 派生 #51 的 estimator-owned staged initialization ADR/design、spec 与 plan；
4. [ ] 实施 Slice B，并在 #53 checkpoint 后完成 TUM VI product verification。

`docs/design/roadmap.md` 只记录本 map 的高层产品方向；本 map 不创建正式 spec/plan，也不授权实验或实现。

## 11. 确认结果

1. Slice A 的 raw pixel canonicalization 由 `StereoRectifier` implementation 私有拥有；其 interface 为“支持的 raw 灰度双目输入 → rectified `uint8` stereo”，当前不建立独立 photometric seam。
2. Slice A 在 native pixel depth 完成 geometric remap；`uint16` 只在最终 rectified 输出处按固定合同量化为 `uint8`，`uint8` control 保持现有 remap 路径。
3. Slice A 的 `uint16 → uint8` 固定映射为 `dst = static_cast<uint8_t>(src >> 8)`；低 8 位截断，合法输入无需额外饱和，运行时映射不依赖样本统计。
4. Slice A 保持现有 `CameraModelResult<T>` / `CameraModelError` interface 与三类 code；`detail` 携带 stage、side、expected / actual 和原始库原因，但不作为机器解析 schema。
5. Slice A 不新增 regular diagnostics 字段；保留 `timing.rectify`，conversion-path 与强度证据进入定向测试或 opt-in acceptance artifact。
6. App composition root 显式选择 `euroc::open()` 或 `tum_vi::open()`，并把格式中立的 `StereoImuDataset` 传给 `runOfflineVoSession()`；session 不拥有 concrete adapter switch。
7. Slice A 采用 deterministic camera oracle、EuRoC control 与 bounded real TUM VI session prefix 三层 gate；initialization-only session 返回 `error = nullopt`、`trajectory = nullopt`，轨迹要求由 composition root 判断。
8. Slice A 的真实 TUM VI gate 固定为 `corridor1_512_16` 从首帧开始的 100 个 stereo frames，使用既有 `PHAD_TUMVI_CORRIDOR1_PATH` opt-in 数据入口。
9. Slice A 的 100-frame product predicate 使用现有结构字段：完整处理 100 帧、双目 emit 100、左右 drop / overflow 全零、estimator hard failure 为零，累计 observations 与 positive disparity 均非零；trajectory、`kOk`、initialization verdict 与 ATE 不参与判定。
10. #42 保持 trajectory-continuity wayfinding map；#53 为 Slice A product-input child，#51 为 Slice B initial-moving-root child，均已建立 native sub-issue relation；cold-root recovery 后续另建 child。B 的 TUM VI product verification 依赖 A checkpoint，不把 B 的全部工作标成 blocked。
11. Slice B 不新增独立 raw-motion hard threshold；以 gyro 和 gravity/velocity candidate systems 的 full required rank 与 normalized condition limit 判断 observability / conditioning，不足保持 `kInitializing`。Raw motion summaries 只作 diagnostics，posterior precision 保持 observe-only。
12. Slice B 最早在同一 visual connected component 内具备至少 6 个 time-distinct pose-bearing evidence frames、且首末 timestamp 跨度不少于 250 ms 时尝试 candidate solve；count 与 duration 任一不足均保持 `kInitializing`。
13. Final joint consistency 的最小 covariance authority 为同一 visual graph 的 root-eliminated QR joint pose marginal 与 final-bias fresh PIM covariance；按 explicit visual/IMU independence 和 block-diagonal interval-noise approximation 构造 innovation，并以 effective DoF 的 `chi-square(0.99)` 作 adopted-model consistency veto。该 authority 不授予 posterior physical precision admission。
14. `UpdateDiagnostics` 新增 nested `InitializationDiagnostics` project-owned POD/enums，作为 `VioUpdateResult` 的 typed in-memory interface；首片不扩现有 CSV/JSON 持久 schema，也不提升其 version。
15. `InitializationDiagnostics` 使用 lifecycle、evidence、visual support / fit、observability、joint consistency、current graph 与 transaction 的最小 gate-summary 字段；新成员遵守 `m_` + snake_case，enum values 遵守 `k` + PascalCase。Visual support 为独立 optional actual/required summary，cheirality 有独立 typed reason；未评估 whole groups 使用 `std::optional`，不使用 NaN 或 sentinel。

## 12. #53 实施与验证回填

#53 已在 clean code commit `ba607c5738b3c5d9bc9cb0423199eee67648b0b3`
（tree `7950f03a1125280169170513aad0a399198d0e76`）完成 Slice A。camera
实现来自 `bc07ff8`，session/composition 实现来自 `f41fe64`。实际交付保持本图
定义的边界：composition root 显式调用 `euroc::open()` 或 `tum_vi::open()`，把
已打开的 `StereoImuDataset` 传入共同 `OfflineVoSession`；camera path 支持 equidistant
raw `uint16` 经 native-depth remap 后以 high-byte mapping 产出 rectified `uint8`。

实测配置与结果已记录于
[M5 TUM VI product-input checkpoint](../benchmark/m5/tum-vi-product-input-seam_ba607c5_noconfig.md)：

| 层级 | 实际命令 / 证据 | 结果 |
|---|---|---|
| unit regression | clean clone，`RelWithDebInfo`，指定八个 test targets 后 `ctest --test-dir build-unit --output-on-failure -L unit` | 357/357 passed；3 个 opt-in MH 环境测试 skipped |
| deterministic camera + EuRoC frontend control | `ctest --output-on-failure -L mh01` 中 `phad_camera_mh01_test` 与 `phad_frontend_mh01_test` | 2/2 passed，累计 `225.18 s` |
| EuRoC session control | `PHAD_EUROC_MH01_PATH=... ./build-mh01/phad_apps_tests --gtest_filter='OfflineVoSessionTest.*'` | 10/10 passed |
| bounded TUM VI product gate | `PHAD_TUMVI_CORRIDOR1_PATH` opt-in，固定 `corridor1_512_16` 首 100 个 stereo frames | loader + product 2/2 passed；structural predicate 无 error，100 frames/diag/emitted，drop/overflow/failed 均为 0 |

`phad_apps_mh01_test` 未作为 #53 formal PASS：本次发现它在当前树与
`main@46a84b5` 都使用同一静态 timeout / 旧 control，属于实施前已存在的陈旧
gate。它不改变 camera/frontend control 和 TUM VI product gate 的实测结论，但不能
作为 apps MH_01 回归已经验证的证据。

#53 未执行 TUM VI trajectory、ATE/RPE 或 full-sequence benchmark；`counts.ok=100`
仅作为 observe-only 结果，非本片 gate。#51 的 initial moving root、其实现或验证
仍未在本回填中声明完成。

## 13. Evidence index

### Main authority

- [`docs/design/roadmap.md`](../design/roadmap.md)
- [`docs/design/architecture.md`](../design/architecture.md)
- [`docs/agents/thin-honest-slice.md`](../agents/thin-honest-slice.md)
- [`docs/research/2026-08-26-note-m5-stereo-vi-init-recovery.md`](2026-08-26-note-m5-stereo-vi-init-recovery.md)
- [`docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)

### Read-only donor identity

以下文件以 `git show 1813606:<path>` 读取；它们不是 main 中的 authority：

- `phad/estimator/vio_estimator.cpp`
- `phad/estimator/internal/stereo_vi_initialization.{hpp,cpp}`
- `phad/estimator/internal/candidate_qualification.{hpp,cpp}`
- `phad/estimator/internal/vio_update_transaction.hpp`
- `apps/offline_vo_session.{hpp,cpp}`
- `tests/estimator/vio_full_state_test.cpp`
- `tests/estimator/vio_lifecycle_test.cpp`
- `tests/estimator/cold_root_observe_test.cpp`
- `tests/estimator/candidate_qualification_test.cpp`
- `tests/apps/offline_vo_session_test.cpp`

### Primary implementation references

- [OpenCV fisheye camera model and stereo rectification](https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html)
- [VINS-Mono initialization alignment](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/initial/initial_aligment.cpp)
- [ORB-SLAM3 local inertial initialization/refinement](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/LocalMapping.cc)
- [OpenVINS dynamic initializer](https://docs.openvins.com/classov__init_1_1DynamicInitializer.html)
- [GTSAM preintegrated IMU measurements](https://borglab.github.io/gtsam/preintegratedimumeasurements/)

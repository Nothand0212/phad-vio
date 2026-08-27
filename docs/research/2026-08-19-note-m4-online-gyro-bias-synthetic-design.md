# M4 在线 gyro bias state synthetic 资格设计

日期：2026-08-18
状态：Design complete；implementation、RED/GREEN 与 qualification 均未开始
Protocol ID：`PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1`

## 结论与权限先行

本协议重新定义 M4 的下一条可证伪因果边：在**纯 synthetic、固定窗口、无逐出**的受控图中，
允许 raw gyro interval 经一个显式在线三维 bias state 与 random-walk model 进入
`StereoVoEstimator` posterior，并验证 residual、Jacobian、noise、observability、state
lifecycle 与纯视觉关闭路径。

这是一份新协议，不是以下任一事项：

- 不是 `PHAD-M4-Q3-GYRO-ALIGN-V1` 的第二次运行、阈值修订或补考；
- 不是旧 Q4「把 Q3 offline constant bias 冻结后加入 rotation constraint」；
- 不是对 Q3 `HYPOTHESIS_FAIL` 的改判；
- 不是 apps/session 接线、真实序列运行、自然 feedback 或默认开启；
- 不是含 velocity、gravity、accelerometer measurement/bias 的完整 `X/V/B` VIO。

[Q3 result](2026-08-18-note-m4-minimal-gyro-q3-offline-bias-alignment-result.md) 的唯一失败项仍是
`HALF_STABILITY`：`0.001227119335357879 rad/s > 0.001 rad/s`，权限仍为 `STOP`。
它否决了已冻结的全局常值 visual-proxy nuisance 路线，不否决本协议提出的在线慢变 state；
同样，它也不为本协议提供 `G(0)` mean、prior covariance 或真实 sensor noise。

本协议的 synthetic `PASS` **只授权**：

1. 保留 default-off、无 real caller 的 estimator-private mechanism；
2. 新写一份独立的 `eviction information handoff` 设计/实施计划。

它不授权修改 apps、运行 real/GT/ATE/RPE、进入 natural feedback、修改长期 artifact schema、
改产品 config hash、默认开启、声称 bounded online capability，或声称完整 VIO 已完成。

依据包括：[online bias state 调研](2026-08-18-note-m4-vio-online-gyro-bias-state-research.md)、
[Q1 Observe result](2026-08-12-note-m4-minimal-gyro-q1-observe-result.md)、
[Q2 known-bias predict result](2026-08-13-note-m4-minimal-gyro-q2-known-bias-predict-result.md)、
[证据门控规则](../agents/evidence-gated-integration.md)和
[增量开发规则](../agents/incremental-development.md)。

## 1. 可证伪问题与唯一新增因果边

### 1.1 Hypothesis

若：

- 每个实际进入 batch graph 的 `Pose3 X(k)` 都有一个三维 gyro bias state `G(k)`；
- 每条合格的相邻 state interval 同时产生一个严格 rotation-only factor 与一个 bias RW factor；
- gyro measurement noise、bias RW、root prior 和 whitening 的量纲正确；
- 优化后的 `G(k)` 与 `X(k)` 一起 transactional writeback、rebuild 与 rollback；

则在下文冻结的 14-pose no-eviction synthetic scene 中，posterior 应恢复已知的慢变 bias，
并满足预注册 oracle、observability、Jacobian、mutant、zero-drift、determinism 与 exact-count 门。

任何一项不成立，都必须在 synthetic 层停止；不得用 ATE、真实数据、调 covariance、扩大窗口或
增加 full-IMU state 来覆盖失败。

### 1.2 唯一新增因果边

```text
public GyroInterval
        │
        ▼
private endpoint reducer → PreintegratedAhrsMeasurements
        │
        ▼
GyroRotationFactor(X_i, X_j, G_i) + RW(G_i, G_j)
        │
        ▼
same StereoVoEstimator batch posterior
```

本片允许 gyro 第一次在受控 synthetic graph 中改变 posterior。关键帧 policy、frontend tracks、
sync packet 构造、apps orchestration、真实 calibration、eval 与自然 feedback 全部冻结在边外。

## 2. 模块、interface 与依赖方向

### 2.1 外部 seam：项目自有 POD/value types

`StereoVoEstimator::update(KeyframeMeasurement, bool)` 保持唯一更新入口；不新增第二个 estimator，
不暴露 GTSAM type。冻结的 public 形态为：

```cpp
struct GyroInterval
{
  common::Timestamp                    m_t_prev;
  std::vector<sensor::ImuMeasurement> m_samples;
  bool                                 m_imu_gap = false;
};

struct GyroBiasOptions
{
  double          m_gyr_nd            = 0.0;  // rad/s/sqrt(Hz)
  double          m_gyr_rw            = 0.0;  // rad/s^2/sqrt(Hz)
  Eigen::Vector3d m_prior_mean_radps  = Eigen::Vector3d::Zero();
  double          m_prior_sigma_radps = 0.0;
};

struct KeyframeMeasurement
{
  // existing fields unchanged
  std::optional<GyroInterval> m_gyro_interval;
};

struct EstimatorOptions
{
  // existing fields unchanged
  std::optional<GyroBiasOptions> m_gyro_bias;
};
```

`EstimatorOptions::m_gyro_bias == std::nullopt` 是唯一关闭语义。关闭时实现不得读取或验证
`m_gyro_interval`，即使其中有少样本、重复 timestamp、NaN 或 overflow candidate，也必须保持既有
视觉 status、message、estimate 与旧 diagnostics 各字段 double bit pattern 不变。

`EstimatorOptions::m_gyro_bias` 是本协议唯一允许的 activation seam：只允许 synthetic product test
直接构造该 in-memory POD，保持 default-off。不得增加 CLI/config parser、`flattenConfig()`、
`config_hash`、persistent artifact、apps/session 或 real-data caller；该 seam 只选择 estimator private
implementation，绝不公开 GTSAM key/type、PIM、factor 或 state ownership。

本协议不把 `StereoImuPacket` 传入 estimator：它含 image 与 sync ownership，interface 过宽。也不把
`sensor::ImuParameters` 整体塞入 posterior；本 synthetic slice 只消费明确给出的 `m_gyr_nd`、
`m_gyr_rw` 与 synthetic root prior。未来 real calibration mapping 属于 apps qualification，不在本片。

### 2.2 Public diagnostics

冻结的 in-memory diagnostics 为：

```cpp
enum class GyroBreakReason : std::uint8_t
{
  kNone = 0,
  kMissingInterval,
  kDeclaredGap,
  kRejectedEndpoint,
  kEvictedEndpoint,
  kSegmentChange
};

struct GyroWindowBias
{
  std::uint64_t                  m_frame_index = 0;
  common::Timestamp              m_timestamp;
  std::optional<Eigen::Vector3d> m_bias_radps;
};

struct GyroDiagnostics
{
  std::optional<Eigen::Vector3d> m_bias_radps;
  std::vector<GyroWindowBias>    m_window_biases;
  std::uint32_t m_rotation_factors       = 0;
  std::uint32_t m_rw_factors             = 0;
  std::uint32_t m_root_priors            = 0;
  std::uint32_t m_relinearization_rounds = 0;
  GyroBreakReason m_break_reason = GyroBreakReason::kNone;
};

struct UpdateDiagnostics
{
  // existing fields unchanged
  std::optional<GyroDiagnostics> m_gyro;
};
```

off 时 `UpdateDiagnostics::m_gyro == nullopt`。on 时 `m_gyro` present；任何 `m_bias_radps` 只要 present
就必须 finite。`m_window_biases` 按 `m_frame_index` 严格递增，当前 graph 每个 `G(k)` 恰有一项，且
`m_frame_index`/`m_timestamp` 与对应 `X(k)` 完全相同。entry 的 `m_bias_radps` present 当且仅当它所在
connected component 在当前 graph 至少含一条 rotation factor；prior-only singleton 为 null，已
观测 component 的 root 与只有 RW prediction 的 terminal（主门的 `G13`）均 present。

factor/root counts 表示**本次 update 完成后当前 rebuilt graph** 的数量，不是进程累计值。
`m_relinearization_rounds` 只在最终 status 为 `kOk` 时表示 initial solve 之后实际执行的额外 PIM
rebuild+solve 次数，不含 initial solve；这包括内部 reopt 成功或 fallback 后最终仍为 `kOk` 的更新。
单次 graph solve 的额外次数范围为 `0..2`，字段对该成功 update 的所有 graph solves 求和。
它是成功结果字段，不是 solver trace：任何最终 `kRejected` / `kFailed` 都必须报 `0`，即使 cap
之前确实执行过 extra round。成功 `kOk` 时 scalar `m_bias_radps` 必须与
`m_window_biases.back().m_bias_radps` 相等。hard rejected 或 failed update 的 scalar 必须为 null，
`m_relinearization_rounds=0`、`m_break_reason=kNone`；`m_window_biases` 与 factor/root counts 则是完整
rollback 后的 committed topology 确定性 materialize
的 snapshot，不能泄漏本次 provisional state。diagnostics 本身不进入 transaction state，避免
committed state 与 result 出现双重真相；root-only、prior-only 状态不得把 prior mean 冒充 recovered
bias。

本片不修改 `diag.csv`、`summary.json`、`meta.json`、`est.tum` 或 `kf.tum` schema。break reason
只有 topology 已知有效、但不能连边时才使用。malformed interval 由 private typed
`GyroIntervalErrorCode` 处理，再映射到现有 `UpdateStatus` 与稳定 message；不为一次 synthetic
实验扩张公共错误 interface。

### 2.3 内部 seam 与依赖

`StereoVoEstimator::Impl` 继续拥有 GTSAM key、PIM、factor、graph rebuild、LM、writeback 与
rollback。内部新增：

- `G(k) = gtsam::Symbol('g', frame_index)`，value type 为 `gtsam::Vector3`；
- `ValidatedGyroInterval`：保留 immutable raw samples、exact duration 与 endpoint provenance；
- 唯一的 timestamp validation、endpoint trapezoid 与 `dt` reducer；
- `GyroRotationFactor`：private `NoiseModelFactorN<Pose3, Pose3, Vector3>`，内部组合官方
  `AHRSFactor`；
- 每个 `WindowFrame` 的 bias initial value、segment、显式 predecessor `frame_index`、validated
  raw interval 与 component/root metadata；
- 单一 header-only `StereoVoUpdateState` / `StereoVoUpdateTransaction` private seam，聚合全部会在
  visual staging 后变化的 mutable ownership，并以 RAII 管理 committed/provisional state。

Q2 public wrapper 与 private factor 必须调用同一个 reducer；禁止复制第二套 endpoint average、
timestamp subtraction 或 seconds conversion。Q2 的 known-bias public contract 与结果保持不变。

`G(k)` 是 [ADR-0002](../adr/0002-stage-gated-gyro-only-bias-state.md) 与
[`docs/design/conventions.md` §3.1](../design/conventions.md#31-阶段性-gyro-only-资格-keyadr-0002) 允许的窄例外：
只存在于本协议 synthetic、default-off、无 real caller 的 estimator private implementation。
完整 VIO 的 canonical bias key 仍是 `B(k): imuBias::ConstantBias`；同一 graph 禁止 `G/B` 双写、
alias 或 adapter。未来 full `X/V/B` slice 获授权时，必须删除 `G(k)`、gyro-only factor 与其
lifecycle，直接改用 `B(k)`，不保留兼容层。

依赖方向保持：

```text
common / camera / sensor → phad::estimator → PRIVATE GTSAM
```

禁止 `sync → estimator`、`estimator → apps/frontend/io/eval`，也禁止另建并行 `Rot3` state 只为直接
复用 GTSAM `AHRSFactor`。当前 graph 的权威 orientation 已在 `Pose3 X(k)`；复制 `R(k)` 会新增一套
同步约束与 gauge。

## 3. 冻结 state 与 factor 数学

### 3.1 A1：每个实际 graph pose state

本协议只允许 A1：**每个实际存在于当前 graph 的 `Pose3 X(k)` 都有且只有一个 `G(k)`**。
`keyframe` 只是 apps 给 estimator 的 policy 标记，不等于 graph-state ownership；当前源码会让
accepted non-keyframe 进入 window/graph。因此不得写成“每 keyframe 一个 bias”，也不得在本片切换
到只连接 keyframe 的 A2。

每条合格 link `(i,j)` 添加：

```text
GyroRotationFactor(X_i, X_j, G_i)
BetweenFactor<Vector3>(G_i, G_j, zero, sigma_rw)
```

每个断开的 bias connected component 有一个独立 root prior。本片 root prior 总是使用显式
`GyroBiasOptions.m_prior_mean_radps/m_prior_sigma_radps`，绝不使用 Q3 nuisance estimate、被逐出 bias
point estimate 或旧 posterior covariance。

新 `G_j` 的 initial value 也按 topology 唯一冻结：exact link `(i,j)` 继承 predecessor `G_i` 最近的
bias，当前 update 已有 provisional value 时用最近 update-local provisional，否则用 committed
value；新 connected component/root（首帧、missing、gap、rejected、evicted 或 segment change）则
精确使用 protocol `prior_mean_radps`。禁止从 gyro sample truth、Q3 nuisance、被逐出 point estimate
或旧 posterior 构造另一种初值。

#### 3.1.1 Private topology initializer test seam

上述 exact-link inherit / new-component prior 选择集中在新建的 header-only private seam
`phad/estimator/internal/gyro_bias_initial_value.hpp`；它不是 public API、option 或 feature switch：

```cpp
enum class GyroBiasInitialKind
{
  kExactLink,
  kComponentRoot
};

enum class GyroBiasInitialErrorCode
{
  kMissingPredecessor,
  kNonFiniteSelectedValue
};

using GyroBiasInitialResult =
    std::variant<Eigen::Vector3d, GyroBiasInitialErrorCode>;

[[nodiscard]] GyroBiasInitialResult selectGyroBiasInitialValue(
    GyroBiasInitialKind                      kind,
    const std::optional<Eigen::Vector3d>& provisional,
    const std::optional<Eigen::Vector3d>& committed,
    const Eigen::Vector3d&                prior_mean_radps );
```

`kExactLink` 精确按 provisional → committed 优先级选择；两者都缺失返回
`kMissingPredecessor`，已选值 non-finite 返回 `kNonFiniteSelectedValue`，不得 fallback 到下一候选。
`kComponentRoot` 只返回 finite `prior_mean_radps` 并忽略两个 predecessor 参数；prior non-finite 返回
`kNonFiniteSelectedValue`。signature 故意没有 gyro truth、Q3 nuisance、evicted mean 或 covariance
参数。

product test 冻结以下可二进制精确表示、彼此不同且逐轴 nonzero 的 literals（rad/s）：

```text
provisional = [ 0.03125, -0.0625,  0.125]
committed   = [-0.25,     0.5,    -1.0  ]
prior       = [ 1.5,     -2.0,     0.75 ]
```

用它们 bit-exact 覆盖全部选择与错误分支；root case 同时传入上述两个 predecessor，证明它们被
忽略。graph build 必须只调用该
selector，并另断言 `Values[G_i]` 与 PIM `biasHat_i` 来自同一 round-start snapshot。production-source
initializer mutant 只交换 exact-link/root 的 selected source，必须被 targeted test 杀死；禁止在
PIMPL 复制第二套选择逻辑。

#### 3.1.2 Private visual-staging transaction seam

完整 rollback 不依赖不可稳定触达的 public optimizer exception fixture。新建唯一 header-only private
seam `phad/estimator/internal/stereo_vo_update_transaction.hpp`，不依赖 GTSAM，并冻结以下 ownership：

```cpp
struct GyroFrameState
{
  Eigen::Vector3d                      m_bias_radps;
  std::uint32_t                        m_segment_id = 0;
  std::uint64_t                        m_component_id = 0;
  std::optional<std::uint64_t>         m_predecessor_frame_index;
  std::optional<ValidatedGyroInterval> m_interval;
};

struct WindowFrame
{
  std::uint64_t                  m_frame_index = 0;
  common::Timestamp              m_timestamp{0};
  Eigen::Isometry3d              m_T_W_B = Eigen::Isometry3d::Identity();
  std::vector<StereoObservation> m_observations;
  bool                           m_is_keyframe = true;
  std::optional<GyroFrameState>  m_gyro;
};

struct StereoVoUpdateState
{
  std::deque<WindowFrame> m_window;
  std::unordered_map<LandmarkId, Eigen::Vector3d> m_landmarks_w;
  std::unordered_map<LandmarkId, std::vector<common::Timestamp>> m_track_times;
  std::unordered_map<LandmarkId, Eigen::Isometry3d> m_T_W_B_last_stereo;
  std::optional<Eigen::Isometry3d> m_T_W_B_last_accepted;
  std::optional<Eigen::Isometry3d> m_T_W_B_prev_accepted;
  std::uint64_t m_next_frame_index = 0;
  bool m_initialized = false;
  std::uint32_t m_segment_id = 0;
  std::unordered_set<LandmarkId> m_culled_ids;
  std::unordered_map<LandmarkId, StereoObservation> m_pending_seed_obs;
  std::optional<common::Timestamp> m_eligible_visual_rejected_timestamp;
  std::uint64_t m_next_gyro_component_id = 0;
};

class StereoVoUpdateTransaction
{
 public:
  explicit StereoVoUpdateTransaction(
      std::unique_ptr<StereoVoUpdateState>& owner );
  ~StereoVoUpdateTransaction() noexcept;

  StereoVoUpdateTransaction( const StereoVoUpdateTransaction& ) = delete;
  StereoVoUpdateTransaction& operator=( const StereoVoUpdateTransaction& ) = delete;
  StereoVoUpdateTransaction( StereoVoUpdateTransaction&& ) = delete;
  StereoVoUpdateTransaction& operator=( StereoVoUpdateTransaction&& ) = delete;

  void rollback() noexcept;
  void commit() noexcept;

 private:
  static std::unique_ptr<StereoVoUpdateState> clone(
      const std::unique_ptr<StereoVoUpdateState>& owner );
  std::unique_ptr<StereoVoUpdateState>& m_owner;
  std::unique_ptr<StereoVoUpdateState>  m_before;
};
```

`GyroFrameState::m_predecessor_frame_index == nullopt` 唯一表示 component root，不再保存重复 root
bool；reportability 与 factor/root counts 从恢复后的 window topology 确定性 materialize，public
`GyroDiagnostics` 不进入 `StereoVoUpdateState`。provisional GTSAM `Values` 仍是单次 solve 的局部量，
只有最终合法 writeback 才进入 transaction 当前拥有的 `WindowFrame::m_gyro`。

本协议新增或迁出的所有 C++ `struct` / `class` data member 统一遵守
[`cpp-naming.md`](../agents/cpp-naming.md)：`m_` + snake_case，transform 使用
`m_T_target_source`。该规则同样覆盖 reducer result carrier、initializer/factor private state、
`ValidatedGyroInterval`、`GyroFrameState`、迁出的 `WindowFrame`、`StereoVoUpdateState` 与
transaction；方法和参数仍按现有规则。已存在的 public `KeyframeMeasurement` / `EstimatorOptions` /
`UpdateDiagnostics` 旧成员不在本片重命名，只有新增的 `m_gyro_interval`、`m_gyro_bias` 与 `m_gyro`
遵守新成员命名。

`Impl` 只保留 calibration、options、`K`、`body_P_sensor` 与 noise model 等不变 ownership，以及唯一
`std::unique_ptr<StereoVoUpdateState> m_state`；不得再散落保存上述 mutable fields。transaction constructor
先校验 non-null owner，再把当前 visual-staging committed state deep-copy 到 `m_before`；copy 失败时
owner 不变。constructor **不得**把 copy swap 成 live state：production 必须继续原地修改原 owner，
避免仅因启用 transaction 就改变 `unordered_map` 的 bucket/iteration 表示并破坏成功/off identity。
`rollback()` 只做 `m_owner.swap(m_before)` 后 reset 被替换的 provisional state；`commit()` 只 reset
snapshot；destructor 在仍持有 snapshot 时调用同一 rollback。`commit()` / `rollback()` 幂等，显式
rollback 后 destructor 为 no-op；三者均 `noexcept`，不得在 destructor 中对含 Eigen/STL container 的
state 做可能抛异常的 assignment。

transaction 构造点精确位于 basic/gyro validation、visual support 与既有 intentional pre-staging
`m_pending_seed_obs` / rejected-endpoint provenance 动作之后，且在 seed、reanchor、normal 三个 visual
staging branch 之前；它位于 gyro 与 `keyframe` guard 之外。post-staging hard return 必须先显式
`rollback()`，再从恢复后 `*m_state` 重新查询并 materialize committed diagnostics；任何 iterator、
reference 或 pointer 都不得跨 rollback。为使该顺序可静态证明，transaction region 内所有最终
`kRejected` / `kFailed` 都必须经过唯一 private/local
`finalizePostStagingHardResult(...)`；该 finalizer 严格按“`transaction.rollback()` → 从新
`*m_state` 构造 result 与 `m_gyro` topology diagnostics → return”执行，禁止 transaction region 直接 hard
return。未提交的 exception unwind 由 destructor rollback；唯一成功 `kOk` return 在 result 完整形成后
调用一次 `commit()`。正常 gyro on/off 成功路径均必须保持 `m_state.get()` 的进入地址 identity；
constructor 和 commit 都不得替换 live owner。

production source 在唯一 constructor、唯一 failure-finalizer 的 explicit rollback 与唯一 success commit 前
各放置一个 exact marker：

```text
// PHAD_M4_ONLINE_BIAS_TRANSACTION_CTOR
// PHAD_M4_ONLINE_BIAS_ROLLBACK_BEFORE_DIAGNOSTICS
// PHAD_M4_ONLINE_BIAS_TRANSACTION_COMMIT
```

marker 不是 feature switch；§9.3 static verifier 用它们定位并验证 call count、支配关系、guard、
failure-finalizer 与 rollback-before-materialize 顺序。

现有 product test source 直接测试上述 production state/transaction 本体，而非 generic surrogate：

```text
StereoVoUpdateTransaction.ScopeExitRestoresEveryField
StereoVoUpdateTransaction.ExceptionUnwindRestoresEveryField
StereoVoUpdateTransaction.ExplicitRollbackRestoresEveryField
StereoVoUpdateTransaction.CommitPublishesEveryField
```

每门都在 constructor 前保存 `owner_before = owner.get()`，constructor 后精确断言
`owner.get() == owner_before`；这是 no-live-copy 的动态合同。每门另构造至少两个 window frame；
before/after 都是完整、nonempty、非平凡 state，且每个字段都被
修改并比较。frame 逐项覆盖 `m_frame_index`、`m_timestamp`、`m_T_W_B`、
`m_observations`、`m_is_keyframe` 与 `m_gyro` optional；nested gyro 同时修改/比较
`m_bias_radps`、`m_segment_id`、`m_component_id`、`m_predecessor_frame_index`、一个 root、一个
exact link 与 `m_interval` 中的 raw samples/duration/provenance。`m_window`、`m_landmarks_w`、
`m_track_times`、`m_T_W_B_last_stereo`、`m_T_W_B_last_accepted`、`m_T_W_B_prev_accepted`、
`m_next_frame_index`、`m_initialized`、`m_segment_id`、`m_culled_ids`、`m_pending_seed_obs`、
`m_eligible_visual_rejected_timestamp` 与 `m_next_gyro_component_id`
同样填充后逐字段 exact 比较；bool 必须翻转，optional 必须覆盖 semantic presence/null，而不是用
“两边都 nondefault”掩盖只有两个取值或 root 必须 null 的事实。前 3 门分别通过 normal scope exit、
真实 C++ exception unwind 与显式 rollback 恢复；它们在 rollback/destructor 后断言 owner 指向
snapshot-owned 对象，即 `owner.get() != owner_before`，同时所有字段与 before exact 相等，不错误要求
恢复到已被销毁的原地址。commit 门逐字段保留 mutation，并在 `commit()` 后断言
`owner.get() == owner_before`。public 层资格化正常 KF/non-KF staging、14-prefix 状态推进，以及
§4.4 stable unknown-predecessor post-staging hard return；后者不是 optimizer exception，V1 不声称动态
触达 public outer optimizer failure。

### 3.2 Residual 与 cost

令：

```text
R_ij_visual = R_WB_i^T R_WB_j
DeltaR_imu  = gyro interval 在候选 G_i 下的旋转预积分
```

rotation residual、RW residual 与 root residual 冻结为：

```text
r_R = Log(DeltaR_imu^T R_ij_visual)
r_b = G_j - G_i
r_0 = G_root - m_prior_mean_radps
```

对 interval 总时长 `Delta t`：

```text
Sigma_R  = m_gyr_nd^2 * Delta t * I3
Q_b      = m_gyr_rw^2 * Delta t * I3
sigma_rw = m_gyr_rw * sqrt(Delta t)
Sigma_0  = m_prior_sigma_radps^2 * I3
```

图 cost 必须等于：

```text
0.5 * sum_links(
    r_R^T Sigma_R^-1 r_R +
    r_b^T Q_b^-1 r_b) +
0.5 * sum_roots(r_0^T Sigma_0^-1 r_0)
```

gyro、RW 与 root factor 都不使用 Huber、noise inflation、动态权重或 axis-specific tuning。
`Diagonal::Sigmas` 接受标准差，禁止把 `Q_b` 的 variance 直接传给它。
构造任何 noise model 前，`m_gyr_nd^2 * Delta t`、`m_gyr_rw^2 * Delta t`、
`m_gyr_rw * sqrt(Delta t)` 与 `m_prior_sigma_radps^2` 的派生 scalar 都必须分别为 finite 且严格大于
零；只检查输入 literal 为正不够，不能让 underflow/overflow 产生 zero、Inf 或 NaN noise。

### 3.3 Endpoint trapezoid 与 PIM lifecycle

对严格递增 samples `(t_s, omega_s)`，唯一 reducer 逐相邻 endpoint 生成：

```text
dt_s       = exact_int64_nanoseconds(t_s+1 - t_s) * 1e-9
omega_mean = 0.5 * (omega_s + omega_s+1)
```

并验证：

```text
m_samples.front().timestamp == GyroInterval.m_t_prev
m_samples.back().timestamp == KeyframeMeasurement.timestamp
sum(exact dt_ns)            == timestamp - m_t_prev
```

本次 update 的首次 graph solve，其 initial round 必须以 visual-staging-entry `WindowFrame` 中 committed
`G/X/L` 为 initial values，并以 committed `G_i` 作为 `biasHat`。同一 graph solve 的额外 round
改用上一 round 留在 update-local Values 的 provisional `G/X/L`，并以 provisional `G_i` 作为
`biasHat`。若之后进入 outlier reopt，其 initial round 同样从上一成功 graph solve 的 update-local
provisional `G/X/L` 开始，而不是退回 staging-entry committed window。每个 round 都必须**新构造**
`PreintegratedAhrsMeasurements`，再从 `ValidatedGyroInterval` 的 raw samples 经唯一 reducer 重积分。
Params 冻结为：

```text
gyroscopeCovariance = m_gyr_nd^2 I3
body_P_sensor        = unset
omegaCoriolis        = unset
```

每个 fixed-PIM round 构建 graph 前，`Values[G_i]` 与该 link PIM 的 `biasHat_i` 必须来自同一份
round-start snapshot 并逐轴相等；不得用 provisional Values 搭配旧 committed `biasHat`，也不得用
新 component 的 prior initial 搭配 predecessor 的其他快照。新 `G_j` 的 Values initial 严格采用
§3.1 的 exact-link inherit / new-component prior 规则。

必须调用 derived `integrateMeasurement(omega_mean, dt)`，使 covariance 按 `Q_g dt` 传播；直接调用
base `integrateGyroMeasurement` 不会形成所需 covariance，禁止使用。

在一次固定 graph/LM round 内，PIM 与其 `biasHat` 不变。private `GyroRotationFactor` 内部组合该 PIM
构造的官方 `AHRSFactor`，`evaluateError(candidate G_i)` 只委托其一阶 bias correction、residual 与
解析 Jacobian；**禁止在 `evaluateError()` 内按 candidate bias 重积分**，否则 factor 不再是固定的
非线性函数且会破坏 bias derivative 语义。

`resetIntegration()` 只清同一个 PIM 的 rotation、duration、Jacobian 与 covariance，不改变
`biasHat`；不得用 reset 冒充换 linearization point。一次 fixed-PIM LM round 完整结束后，下一次
显式 outer round 才能以该 round 的 provisional candidate 从 raw samples 新建 PIM。首次 graph
solve initial 的来源是 visual-staging-entry committed snapshot；同 graph extra 与后续 outlier reopt initial
的来源是最近成功的 update-local provisional snapshot。全程只有最终返回 `kOk` 时才能把最后一份
provisional state 一次性 commit 到 `WindowFrame`。

factor 的 noise model 使用 `pim.preintMeasCov()`，并必须满足
`Sigma_R = m_gyr_nd^2 Delta t I3` 的 whitening 门。first-order correction 相对 full fresh
reintegration 的有效域由 §6.4 单独冻结；超域必须停止，不能静默沿用。

private constant `kMaxFixedPimRounds=3` 冻结一次 estimator graph solve 的总 round 数（首轮 + 最多
2 个额外 bias relinearization rounds），不是 public tuning option。每个 round 内部 PIM 固定：
round 结束若任一 factor 的
`max_axis(abs(G_opt_i-biasHat_i)) >1e-3 rad/s`，该 candidate 只作为下一 outer round 的 provisional
linearization point；不得写回 `WindowFrame`。下一 round 从同一 validated raw samples 新建 PIM，
并以 provisional `G/X/L` 作为 initial values。只有所有 factor 都进入有效域的最终 round 才能一次性
commit。第 3 个总 round 结束后仍超域则 typed reject 并完整 rollback。

gyro samples 已在 body/IMU `B` frame；不得把 `T_B_left_rectified` 再作用到 gyro。factor 只触碰
两个 `Pose3` 的 rotation tangent block；两个 translation Jacobian block 必须逐元素为 exact zero。

## 4. Lifecycle state machine、错误与 exact counts

### 4.1 状态机

概念状态机如下；不要求把它另做 public enum：

```text
Disabled (m_gyro_bias == nullopt)
   └─ ignore all gyro content; execute unchanged VO path

Enabled / RootOnly
   ├─ accepted exact link ──► Linked / reportable bias
   ├─ missing, gap, rejected endpoint, segment change ──► Broken + new component root
   ├─ predecessor later evicted ──► Broken + independent reset root
   └─ malformed provided interval ──► reject transaction; state unchanged

Linked
   ├─ next accepted exact link ──► Linked
   ├─ valid topology break ──► Broken + new component root
   └─ graph/LM/writeback failure ──► rollback; previous committed state
```

frame 0 没有 predecessor，建立首个 root，`m_break_reason=kNone`，scalar 与唯一 window entry 的
`m_bias_radps` 都为 null。新 component 只有 root/prior、还没有 rotation observation 时也必须保持
对应 bias null。

### 4.2 Validation、provenance 与错误优先级

gyro factor 只给冻结的 stereo graph 新增约束；不得改变 visual factor、landmark ownership、
backprojection、outlier cull/reopt 或 keyframe/non-keyframe gate 的既有语义。gyro bias 与 pose、
landmark 使用同一 update transaction，一起 rebuild、writeback 或 rollback。

处理顺序冻结为：

1. `m_gyro_bias==nullopt` 时完全忽略 gyro optional 的 presence 与 bytes，直接走 bit-identical visual
   path；
2. gyro 启用时，先执行既有 visual measurement basic syntax/timestamp validation；
3. 对 `m_imu_gap=false` 的 provided interval 执行 gyro structural validation；若同时有多个错误，
   private code precedence 固定为
   `kTooFewSamples > kEndpointMismatch > kNonIncreasingTimestamp > kNonFiniteGyro >`
   `kTimestampOverflow > kInvalidNoiseScale`；
4. 再执行既有 visual support/backprojection/gating；因此通过 basic validation 的 malformed gyro
   必须先于 low-shared 等 visual support rejection 返回；
5. visual accepted 后先 stage pose/landmark/segment，再按 §4.3 分类 topology，最后构建 joint
   graph 并进入 LM；最终 `kOk` 才一次性 commit。

`m_imu_gap=true` 时不得读取或验证 `m_samples`，其 payload 即使 malformed 也不改变结果。

estimator 只保留一个 optional `m_eligible_visual_rejected_timestamp` provenance。它仅在 basic visual
validation 与 gyro structural validation 都已通过、随后由明确的 visual support/backprojection
stage 返回 `kRejected` 时写入。malformed gyro、optimizer failure、bias round cap 或其他 hard
failure 不得写入；一个 accepted update 完成 topology 分类后清除。这样只有被明确观察到的 visual
endpoint rejection 才能产生 `kRejectedEndpoint`，不能把未知 timestamp 猜成 topology break。

### 4.3 Accept-time registration 与 topology precedence

predecessor `frame_index` 只能在当前 visual update 最终 accepted 时登记，并且必须满足 interval
structurally valid、same segment、predecessor 是实际 graph state。`buildGraph` 只按显式 predecessor
`frame_index` 查 endpoint；禁止按 deque 相邻位置猜 link，禁止把多个 packet 合并，禁止从
surviving timestamps 臆造新 interval。

accepted staged state 的 topology classification precedence 冻结为：

```text
segment change
  > missing interval
  > declared gap
  > exact predecessor (final graph survives: attach; otherwise evicted break)
  > latest eligible visual rejected endpoint
  > unknown endpoint mismatch (hard reject)
```

首个 accepted state 是特例：interval absent 或 declared gap 都只是预期的无前驱 root，reason 为
`kNone`；首帧提供 non-gap interval 则没有合法 predecessor，必须 hard reject 为 endpoint mismatch。
segment change 的优先级只作用于 structural validation 之后的 topology classification，不会掩盖
malformed interval。

| 事件 | 当前 visual 结果 | rotation/RW | component/root | public reason |
|---|---|---:|---|---|
| 首个 accepted state，absent/gap | 保持既有结果 | `+0/+0` | 建首 root | `kNone` |
| 非首帧 optional interval absent | 可 `kOk` | `+0/+0` | 当前 state 开新 component | `kMissingInterval` |
| 非首帧 `m_imu_gap=true` | 可 `kOk` | `+0/+0` | 当前 state 开新 component | `kDeclaredGap` |
| exact predecessor 在 final graph 中存活 | 保持既有结果 | `+1/+1` | 沿当前 component | `kNone` |
| exact predecessor 在 final graph 已逐出 | 当前 graph 继续 | 删除该 link，绝不跨接 | surviving component 独立 reset | `kEvictedEndpoint` |
| `m_t_prev` 是 latest eligible visual rejected endpoint | 当前帧可 `kOk` | `+0/+0` | 当前 state 开新 component | `kRejectedEndpoint` |
| reanchor / segment change | 保持既有 seed 语义 | 不跨 segment | 新 segment 独立 root | `kSegmentChange` |

这里的 independent root 是显式 synthetic prior reset，不是 posterior handoff。特别是 eviction 后，
不得把 evicted bias mean 配一个固定 sigma 后称为“保留了历史信息”。

### 4.4 Malformed interval 与 hard failure

当 gyro 启用、interval provided 且未声明 gap 时，以下 private typed errors 均令本次 update
`kRejected`，bias/window/landmarks/frame index/segment/last accepted pose 全部 transactional
unchanged：

| `GyroIntervalErrorCode` | stable message |
|---|---|
| `kTooFewSamples` | `gyro interval requires at least two samples` |
| `kEndpointMismatch` | `gyro interval endpoints do not match pose timestamps` |
| `kNonIncreasingTimestamp` | `gyro sample timestamps must be strictly increasing` |
| `kNonFiniteGyro` | `gyro sample is non-finite` |
| `kTimestampOverflow` | `gyro interval duration overflow` |
| `kInvalidNoiseScale` | `gyro interval produces invalid noise scale` |

unknown `m_t_prev` mismatch 是 malformed endpoint；只有命中 §4.2 的 latest eligible provenance 时，
才分类为 valid `kRejectedEndpoint`。所有 hard reject/fail 的 public `m_break_reason` 都是 `kNone`；
typed break 只描述 accepted state 的有效 topology break。

稳定 public hard-path 门精确命名
`StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics`：先接受 root `t0`，再输入
视觉 support 合格的 `t1`，其 non-gap interval 的 `m_samples.front().timestamp == m_t_prev`、
`m_samples.back().timestamp == t1` 且 duration/noise 全部合法，但 `m_t_prev` 既不是已接受
predecessor `t0`，也不是 eligible visual-rejected provenance。该输入必须在 structural validation 与
visual staging 之后的 topology classification 命中 unknown predecessor，并经唯一
`finalizePostStagingHardResult(...)` 返回 `kRejected` / `gyro interval endpoints do not match pose timestamps`。
finalizer 必须先 explicit rollback，再生成 result：`m_bias_radps=nullopt`、
`m_relinearization_rounds=0`、`m_break_reason=kNone`，`m_window_biases` 与 rotation/RW/root counts
逐项 exact 等于只含 `t0` root 的 committed topology：`m_window_biases.size()==1`，entry 是
frame 0 / `t0` / null，`m_rotation_factors=0`、`m_rw_factors=0`、`m_root_priors=1`。随后接受一个从
`t0` 到 fresh `t2` 的
exact update，并与另一 fresh control 只执行 `t0 → t2` 的 status/message/estimate/旧 diagnostics/
`m_gyro` 全字段 bit-exact 对拍，证明 `t1` 的 window、track、pose、counter、provenance 与 gyro
topology 均未污染后续更新。

constructor 对 present `GyroBiasOptions` 要求 `m_gyr_nd > 0`、`m_gyr_rw > 0`、
`m_prior_sigma_radps > 0` 且所有值 finite；违反时沿现有 options-validation 风格抛出明确异常。
`m_prior_sigma_radps^2` 也必须 finite 且严格大于零。对每个 validated interval，factor 构造前必须
再次验证 §3.2 的全部派生 variance/sigma finite 且严格大于零；失败映射
`kInvalidNoiseScale`/`kRejected`，并保持 update transactional unchanged。
PIM、factor、LM 或 Values 中 bias missing/wrong type/non-finite 是 `kFailed`，必须保留原始原因并
rollback，不能静默关闭 gyro 后报告 `kOk`。

bias outer loop 另有 private `GyroSolveErrorCode::kRelinearizationLimit`；3 个总 round（首轮 + 最多
2 个额外 round）耗尽时映射为
`kRejected`，stable message 为 `gyro bias relinearization did not converge`，并恢复完整的
visual-staging-entry committed state。它不是 topology break，因此 `GyroBreakReason` 保持
`kNone`。

### 4.5 Forest invariants 与 exact counts

当前 bias graph 必须是一片 forest。对每次成功 graph rebuild：

```text
N_G                 == N_X
N_rotation          == N_rw
N_G                 == N_rotation + N_root_components
diagnostics counts  == current graph counts
```

连续 3 个 accepted exact states 的 exact census 为：

```text
N_G=3, N_rotation=2, N_rw=2, N_root=1
```

`t0 accepted → t1 rejected → t2 accepted(packet=t1→t2)` 时，t2 可视觉 `kOk`，但当前 graph
必须为 `N_rotation=0, N_rw=0, N_root=2`，reason 为 `kRejectedEndpoint`；严禁 `t0→t2`。
随后 `t3 exact(t2→t3)` 后为 `N_rotation=1, N_rw=1, N_root=2`。

missing、declared gap、segment change 与 eviction 的负向测试也必须闭合相同 forest identity。

### 4.6 Rebuild、writeback 与 rollback

- §3.1.2 的 visual-staging transaction 对 KF/non-KF、gyro on/off 相同。snapshot 位于 intentional
  pre-staging pending/provenance 动作之后，覆盖此时的完整 `StereoVoUpdateState`；任何后续 hard
  failure 都恢复该 staging-entry committed state。pre-staging validation/support/rejected-endpoint
  语义不伪称被这一 transaction 回滚。
- 本次 update 首次 graph solve 的 initial round 从 staging-entry committed `G/X/L` 开始；同 graph
  extra round 从上一 round provisional 开始；后续 outlier reopt 的 initial round从上一成功 graph
  solve 的 provisional `G/X/L` 开始。root prior mean 始终是 protocol options 的固定值，不得改成
  上轮 optimized mean。
- 每次 graph build 的 `Values[G_i]` 与对应 PIM `biasHat_i` 必须取自同一 round-start snapshot；新
  exact-linked `G_j` 按 predecessor 最近 update-local provisional/committed bias 初始化，新 component
  则按 protocol prior mean 初始化，绝不读取 Q3 nuisance。
- 首次 LM 与每个成功 outlier reopt 都必须检查每个预期 `G(k)` exists/type/finite，并更新
  update-local provisional snapshot；在最终 `kOk` 前不得 writeback committed window。
- 每个 graph solve/outlier-reopt solve 总共最多 3 个 fixed-PIM LM rounds（首轮 + 最多 2 个额外
  relinearization rounds）；provisional round 不得污染
  committed window。仅当最终为 `kOk` 时，`m_relinearization_rounds` 才报告本次 update 内在
  initial solve 之后**实际调用**的额外 PIM rebuild+solve 次数，包括内部成功/fallback
  后仍返回 `kOk` 的 solve；任何最终 `kRejected` / `kFailed` 均归零，即使 cap path 已运行
  extra round，也不把该 result field 冒充 actual solver trace。
- successful reopt 后的下一轮 rebuild 必须重建相同 gyro/RW/root topology，不重复计 factor。
- failed reopt 回到上一成功 graph solve 的 update-local provisional snapshot；只有 outer hard
  failure 才由统一 RAII seam 回到 visual-staging-entry committed state。hard result 的 gyro
  diagnostics 必须由唯一 `finalizePostStagingHardResult(...)` 在显式 rollback 后从恢复 topology
  重新 materialize，不能保存或恢复另一份 diagnostics 真相。
- `seedSegment` 清旧 window 时同步清旧 bias/link/component；新 segment 使用独立 synthetic root，
  不继承旧 posterior。

### 4.7 Transaction qualification 与 public coverage 边界

GTSAM 默认 Levenberg–Marquardt 会给 linearized system 加 damping，并在 `tryLambda()` 内处理线性
求解异常；“无 stereo factor、translation 欠约束”因此不是稳定的 outer
`IndeterminantLinearSystemException` 触发器。本协议删除该不可达 public fixture，不用 mock、public
failure hook、test-only solver switch 或错误的自然异常假设制造动态覆盖。

完整 transaction 改由 §3.1.2 四个 direct tests 动态资格化：它们使用 production 实际拥有的
`StereoVoUpdateState` 与 RAII 类型，逐字段证明 normal scope、exception unwind、explicit rollback 与
commit 及 owner-address identity。production `update()` 实际使用该 seam，则由 §9.3 的
constructor/failure-finalizer/commit/order/guard static marker 与 3 个 transaction source mutations 证明；
正常 public KF/non-KF staging、14 个 prefix、off identity 与 lifecycle gates 继续证明成功路径。
另外，§4.4 的 stable unknown-predecessor public gate 动态触达 post-staging hard finalizer 并证明
rollback-before-materialize，但不冒充 optimizer exception。这些证据必须同时存在，且不得据此宣称
public outer optimizer failure 已被动态触达。

## 5. Synthetic input freeze

### 5.1 公共 visual schedule

主 recovery fixture 冻结为：

| 项 | 值 |
|---|---|
| pose states | `14`，frame index `0..13` |
| gyro intervals | `13`，每段 `0.1 s` |
| keyframe schedule | frame 0=`K`，frame 1=`K`，frame 2..13=`12 × non-KF` |
| `window_size` | `14` |
| visual motion | stationary，所有 `R_WB_i=I`、translation 不变 |
| visual support | `20` 个 exact shared stereo landmarks，满足 non-KF PnP 门 |
| reanchor | main fixture 禁用 |
| outlier cull/reopt/probe | main fixture 禁用 |
| gyro/body frame | `B`；main fixture `T_B_left=I` |

visual geometry 复制自
[`stereo_vo_reanchor_test.cpp`](../../tests/estimator/stereo_vo_reanchor_test.cpp) 的冻结 source
blob `f6ef740f62e7dec2bc6da62d82b25bd3bce1ec3f`、SHA-256
`1a04bf8cddd2ec57a2c26f801e34047914331d280cf8d153fffbd5fdb07d6889`（calibration/projection
lines 27–53，landmarks lines 99–123）：

```text
fx=400, fy=400, cx=320, cy=240
baseline=0.12 m, width=640, height=480
T_B_left=I

p_left = (T_W_B * T_B_left)^-1 * p_W
u       = 400 * p_left.x / p_left.z + 320
v       = 400 * p_left.y / p_left.z + 240
disp    = 48 / p_left.z
```

每帧都以 `T_W_B=I` 投影以下 20 个 exact shared points；A 的 ids 为 `1..10`，B 的 ids 为
`1000..1009`，顺序与坐标逐项对应：

```text
A = [( 0.40,  0.10, 5.0), (-0.30,  0.20, 4.5),
     ( 0.10, -0.25, 6.0), ( 0.60, -0.10, 5.5),
     (-0.50, -0.20, 4.8), ( 0.00,  0.30, 5.2),
     ( 0.25,  0.15, 4.2), (-0.20, -0.15, 5.8),
     ( 0.35, -0.05, 5.3), (-0.15,  0.25, 4.6)] m

B = [(1.40, -0.30, 5.4), (0.90,  0.35, 4.9),
     (1.10, -0.15, 6.2), (1.60,  0.05, 5.1),
     (0.65, -0.40, 4.7), (1.00,  0.20, 5.6),
     (1.25, -0.20, 4.4), (0.80,  0.30, 5.9),
     (1.35, -0.05, 5.0), (0.85,  0.15, 4.8)] m
```

frame timestamp 冻结为 `t_i=(i+1)*100000000 ns`。frame 0 无 interval；frame `i+1` 的
interval 使用 `t_i`、`t_i+50000000 ns`、`t_i+100000000 ns` 三个 endpoints/midpoint samples。

完整 options snapshot 为：

```text
window_size=14
min_landmark_observations=2
min_seed_observations=10
min_track_observations_for_seed=1
min_shared_landmarks=10
stereo_sigma_px=0.003
huber_k_px=0
prior_rotation_sigma_rad=1e-4
prior_translation_sigma_m=1e-4
use_constant_velocity_init=true
enable_reanchor=false
enable_pnp_init=false
pnp_reproj_px=2
pnp_confidence=0.99
min_pnp_inliers=10
enable_outlier_cull=false
enable_outlier_reopt=false
max_outlier_reopts=0
outlier_avg_reproj_px=4
block_culled_rebirth=true
hanging_landmark_gate_m=1
far_return_refresh_px=6
enable_accumulated_seed=false
enable_probe_b=false
m_gyro_bias.m_gyr_nd=1e-4
m_gyro_bias.m_gyr_rw=1e-3
m_gyro_bias.m_prior_mean_radps=[0,0,0]
m_gyro_bias.m_prior_sigma_radps=0.1
```

`stereo_sigma_px=0.003` 是 exact-pixel synthetic pose anchor，不是产品默认或 real noise。独立
linearized joint stereo `X/L` + gyro/RW preflight 得到 frame 13 worst truth error
`~3.4055019255e-4 rad/s`，axis drift ratios `~[0.937371,0.942770,0.941578]`、L2 ratio
`~0.941369`、whitened design matrix `condition(A)~2153.56`。默认 `stereo_sigma_px=1.0` 不能闭合
public `5e-4` 门；因此不能省略该 snapshot，也不能把本 synthetic 值宣传为产品调参。

该 schedule 是 protocol 的组成部分。现实现有 7-keyframe hard cap；若 14 帧都标 keyframe，即使
`window_size=14` 也会逐出。因此必须使用 `K,K,N×12`，以同时保持 2 keyframes ≤ 7 和 14 个
temporal graph states 全部在窗。

把 `window_size` 扩成 14 只是一项 synthetic fixed-graph privilege，不能外推为 bounded online
能力，也不能成为未来逃避 eviction handoff 的方案。

### 5.2 主 bias/noise literals

对 `i=0..13`：

```text
b_i = [0.012, -0.018, 0.025]
    + i * [0.0001, -0.00015, 0.0002] rad/s
```

对 interval `i=0..12`，timestamps 为 `t_i, t_i+0.05 s, t_i+0.1 s`，三条 gyro samples：

```text
omega(t_i)        = b_i
omega(t_i+0.05 s) = b_i - 0.5 * Delta b
omega(t_i+0.1 s)  = b_{i+1}
```

两个等长 subinterval 的 endpoint trapezoid 总 mean 精确为左端 `b_i`；真实 visual angular rate 为
0，因此 interval `i` 直接观测 `G_i`。冻结 noise/prior：

```text
m_gyr_nd            = 1e-4 rad/s/sqrt(Hz)
m_gyr_rw            = 1e-3 rad/s^2/sqrt(Hz)
m_prior_mean_radps  = [0, 0, 0]
m_prior_sigma_radps = 0.1
```

这些只是 synthetic model literals，不是 EuRoC calibration、产品默认或 Q3 输出。

## 6. Qualification gates

所有门必须同时通过。没有“多数通过”、加权总分或用终局指标覆盖 mechanism failure 的语义。

### 6.1 Independent oracle verifier

oracle 必须由独立 `phad_online_gyro_bias_oracle_tests` target 复算。该 target 只链接
`Eigen3::Eigen` 与 `GTest::gtest_main`，不得链接或 include `phad::estimator`、GTSAM、private factor
或 reducer。它独立构造 dense linear system，复算：

- 14-pose `G0..G13` literals；
- whitening matrices、vectors 与 cost；
- observability `H` 的 rank、singular values 与 condition。

产品测试只消费本文冻结 literals，不共享 oracle helper。这样 oracle 和被测实现不会因复用同一
公式/代码而形成同构 tautology。

独立 oracle 对本文列出的 `G0..G13` 每个 coefficient 使用 absolute error
`<=1e-12 rad/s`；对 §6.6 列出的 singular values、`condition(H)` 与 `condition(H^T H)` 同时要求
absolute error `<=1e-9` 且 relative error `<=1e-10`。rank 仍只使用 §6.6 的冻结 tolerance。这些是
跨 Eigen/libm 的 double 复算工程预算，不是 recovery science 门，也不得反向修改 public estimator
的 `5e-4` 门。

### 6.2 No-eviction 14-pose recovery 主门

独立 dense linear oracle 的冻结完整 literals：

```text
G0  = [0.012061728029528, -0.018092592044293, 0.025123449878755]
G1  = [0.012123576676337, -0.018185365014506, 0.025247150992009]
G2  = [0.012209001999483, -0.018313502999225, 0.025418003097272]
G3  = [0.012303429322112, -0.018455143983168, 0.025606858299808]
G4  = [0.012401285966853, -0.018601928950280, 0.025802571802151]
G5  = [0.012500428578447, -0.018750642867671, 0.026000857106645]
G6  = [0.012599999768489, -0.018899999652733, 0.026199999517784]
G7  = [0.012699570727019, -0.019049356090529, 0.026399141446707]
G8  = [0.012798712412569, -0.019198068618854, 0.026597424822338]
G9  = [0.012896566510688, -0.019344849766032, 0.026793133020306]
G10 = [0.012990987119495, -0.019486480679243, 0.026981974238579]
G11 = [0.013076394847798, -0.019614592271697, 0.027152789695432]
G12 = [0.013138197423899, -0.019707296135849, 0.027276394847716]
G13 = [0.013138197423899, -0.019707296135849, 0.027276394847716]
```

完整 oracle 必须在 qualification receipt 中发布 `G0..G13`，不能只发布三个端点。这里明确拆成
两个不同的门，禁止把 exact-pose oracle 精度错误施加给 joint visual estimator。独立 oracle 自身
复算这些 listed coefficients 时使用 §6.1 的 absolute `<=1e-12 rad/s` engineering tolerance；这与
下方 private DUT graph 的 `<=1e-8 rad/s` 门不是同一个 comparison。

#### 6.2.1 Private fixed-pose graph 门

private test graph 将全部 `X0..X13` 的 rotation exact 固定为 §5 的 identity truth，只优化
`G0..G13`，并加入同一组 rotation/RW/root factors。门：

1. 所有 14 个 bias coefficients 与上述独立 bias-only oracle 的最大绝对差
   `<=1e-8 rad/s`；
2. `N_G=14`、`N_rotation=13`、`N_rw=13`、`N_root=1`；
3. `G13=G12`，末端 state 只由 RW prediction 连接，不能伪称观测了 `b13`。

fixed-pose oracle audit references：`G0→G12` L2 drift ratio
`0.897059249383202`；observed states 的 worst coefficient error
`1.23605152284177e-4 rad/s`；frame 13 相对 `b13` 的 one-step worst error
`3.23605152284179e-4 rad/s`。两帧 prefix fixed-pose oracle 的 terminal bias 为：

```text
fixed_pose_prefix(frame1) =
  [0.011999880001200, -0.017999820001800, 0.024999750002500]
```

这些 literals 只核对 private exact-pose math，不能当作 public estimator 的 exact expected。

#### 6.2.2 Public estimator recovery 门

public test 必须经过 `StereoVoEstimator::update()`、§5.1 的 20-point visual factors 与
`stereo_sigma_px=0.003`，不得用 `NonlinearEquality` 或直接注入 pose。门：

1. final `m_window_biases` 恰有 14 项，`m_frame_index=0..13` 严格升序，`m_timestamp` 等于对应 `t_i`，
   所有 optional 均 present/finite；scalar 等于最后一项；
2. final current graph 为 `N_G=14`、`N_rotation=13`、`N_rw=13`、`N_root=1`，全程没有
   eviction reason；
3. 每项 `i=0..13` 都满足
   `max_axis(abs(m_window_biases[i].m_bias_radps-b_i))<=5e-4 rad/s`；这同时覆盖 RW-only terminal
   `G13` 相对 `b13` 的 one-step error；
4. frame 0 update 的 scalar 与当时唯一 window entry bias 均 null；frame 1 是首个 reportable
   current bias，且每次 update
   单独报告的 round-count vector 必须精确为
   `[0,1,0,0,0,0,0,0,0,0,0,0,0,0]`，因此 frame 1 为 `1`、final frame 13 为 `0`，不得解释为
   进程累计值；
5. frame 13 scalar 相对 frame 1 scalar 的逐轴 drift sign 与 `b12-b0` 一致；
6. 每轴 signed drift ratio 与 L2 drift ratio 都在 `[0.85,1.05]`：

```text
rho_axis = (bias_out(frame13)_axis - bias_out(frame1)_axis)
         / (b12_axis - b0_axis)

rho_L2   = norm(bias_out(frame13) - bias_out(frame1))
         / norm(b12 - b0)
```

public joint preflight 的 axis ratios `~[0.937371,0.942770,0.941578]`、L2 ratio
`~0.941369` 与 frame 13 worst truth error `~3.4055019255e-4 rad/s` 只用于证明门在实现前可闭合；
qualification 必须发布真实 public actual，不能把 preflight 或 private literals 回填成 product
output。特别禁止要求 public window vector 与 bias-only `G0..G13` 在 `1e-8` 内相同。

fixed-PIM preflight 另观察到 frame 1 initial max bias delta
`2.49997500024499754e-2 rad/s`，第一次 fresh-PIM extra round 后打印为 exact zero；这只解释为何
frame 1 应触发一次 extra round，不是 bit-exact qualification gate，也不授权新增 public per-round
trace。生产 hard acceptance 始终只有 final factor
`max_axis(abs(G_opt_i-biasHat_i))<=1e-3 rad/s` 与 public `m_relinearization_rounds==1`。

主门不能只验 final census；对每个成功 prefix `i=0..13`，必须在该次 update 返回时逐项冻结：

```text
UpdateStatus                         = kOk
UpdateDiagnostics.window_size       = i + 1
GyroDiagnostics.m_window_biases.size = i + 1
N_G                                 = i + 1
N_rotation                          = i
N_rw                                = i
N_root                              = 1
m_break_reason                      = kNone
m_window_biases[j].m_frame_index    = j,       j=0..i
m_window_biases[j].m_timestamp      = t_j,     j=0..i
```

prefix `i=0` 时唯一 entry 与 scalar 都为 null；每个 `i>=1` 的 component 已含 rotation
observation，因此 entries `0..i` 全部 present/finite，scalar present 且等于 back。每个 prefix 都不得
出现 eviction 或 index/timestamp 跳号。按 update 顺序收集的 extra-round vector 必须为
`[0,1,0,0,0,0,0,0,0,0,0,0,0,0]`；这些是 14 个独立 per-update actual，不是只在 final
diagnostics 推导出的累计计数。

### 6.3 SO(3)、Pose3 Jacobian 与 mutation 门

单 interval fixture：

```text
R_i      = Exp([0.2, -0.1, 0.05])
omega    = [0.3, -0.22, 0.17] rad/s
bias     = [0.012, -0.018, 0.025] rad/s
measured = [0.312, -0.238, 0.195] rad/s
m_t_prev = 0 s
t_curr   = 1 s
m_samples = [(0 s, measured), (1 s, measured)]
biasHat  = bias
G_i      = bias
R_j      = R_i * Exp(omega)
```

因此唯一 endpoint reducer 产生 `dt=1 s`、`omega_mean=measured`；PIM 在 `biasHat=bias` 处构造，
正确 candidate 的 `G_i-biasHat=0`。测试不得绕过 public timestamps/samples 直接注入预积分结果。

门：

- correct residual norm `<=1e-12 rad`；
- analytic Jacobian 对 central difference，`h=1e-7`，所有 coefficient 最大绝对差 `<=1e-6`；
- 对两个 Pose3 translation block 的任意 perturbation，residual change `<=1e-12`；analytic
  translation Jacobian 的相关元素必须逐元素 exact zero；
- factor residual 与官方 `PreintegratedAhrsMeasurements`/AHRS residual oracle 方向一致。

冻结 mutant residual norms：

| mutant | norm rad |
|---|---:|
| wrong bias sign | `0.06601234538` |
| omit bias | `0.03300615423` |
| rad-as-deg | `0.40188442470` |
| deg-as-rad | `2.10645984131` |
| visual inverse | `0.81804645345` |
| left-compose | `0.02412947757` |
| `Rz(90°)` wrong frame | `0.55273358029` |

每个 mutant 必须被 test 杀死；统一 hard separation 为 `>=0.02 rad`。最近的错误实现是
left-compose 的 `0.02412947757 rad`，因此不得放宽到覆盖它。

另设 public estimator calibration fixture：

```text
R_B_left = Exp([0.2, -0.1, 0.05])
t_B_left = [0.1, -0.02, 0.03] m
```

保持同一 body gyro truth，但**不得**把它的 joint visual/gyro posterior 与 identity-extrinsic
fixture 作逐值相等或 `1e-8` 近似比较：改变 calibration 会合法地改变 stereo Jacobian 与 Fisher
information。non-identity fixture 必须独立满足 §6.2 的 public body-truth error `<=5e-4 rad/s`、
frame 1→13 signed-axis/L2 drift ratio `[0.85,1.05]`、14-prefix presence/order、counts、round vector 与
lifecycle 门。

冻结 independent dense joint linearization preflight（不链接 product/GTSAM）只作门值可达性与
frame-mutant separation 证据：

```text
identity final       = [0.013124651725517, -0.019696753295537, 0.027259449807447]
non-identity final   = [0.013126669336378, -0.019698172265706, 0.027261275987490]
correct cross-fixture max difference = 2.017610861e-6 rad/s  # 合法，不是 gate
wrongly apply R_B_left          worst body-truth error ~= 4.48e-3 rad/s
wrongly apply inverse(R_B_left) worst body-truth error ~= 5.22e-3 rad/s
```

正确路径对 body gyro **不施加任何 camera extrinsic**。private factor 的 `Rz(90°)` wrong-frame
mutant 继续由上表 `>=0.02 rad` separation 杀死；repo-external production-source frame arm 则在
non-identity public path 的 gyro staging 中单独错误施加一次 `R_B_left`，必须让同一 body-truth
`<=5e-4` targeted test 非零退出。inverse 方向由 test-local independent mutant oracle 证明同样跨门，
不另占 production-source arm。

### 6.4 First-order correction 与 full reintegration 门

冻结一个与主 recovery 分离的 noncommuting fixture：

```text
t_k          = 0.00, 0.01, ..., 0.40 s       # 41 samples
omega_true(t)= [1.1*cos(0.7*t), 0.7, 1.1*sin(0.7*t)] rad/s
biasHat      = [0.12, -0.08, 0.05] rad/s
measured(t)  = omega_true(t) + biasHat
```

以该 raw interval 在 `biasHat` 处构造一次 fixed PIM。对
`delta_bias ∈ {-1e-3,-0.9e-3,...,1e-3}^3` 的 `21^3=9261` lattice，分别计算：

```text
candidate       = biasHat + delta_bias
DeltaR_first    = cached PIM 对 candidate 的一阶 bias correction
DeltaR_full     = 以 candidate 为新 biasHat、从 raw samples fresh reintegration
geodesic_error  = norm(Log(DeltaR_first^T DeltaR_full))
```

独立 frozen actual 的最大值为：

```text
max geodesic_error = 2.0431967621531336e-08 rad
at delta_bias      = [0.001, -0.001, -0.001] rad/s
gate               = <=3e-8 rad
```

另在 `{-1e-3,0,1e-3}^3` 的 27 个 corner/axis/zero points，用 full reintegration central
difference `h=1e-7 rad/s` 对拍 cached PIM 的 bias analytic Jacobian：

```text
oracle max coefficient error = 2.4420145350312339e-09
gate                         = <=1e-7
```

Pose3 factor 的完整 `X_i/X_j/G_i` Jacobian 仍使用 §6.3 的 `<=1e-6` 门。

`||candidate-biasHat||_inf <=1e-3 rad/s` 是 V1 唯一获资格的一阶 correction domain；它是主
synthetic 每 edge 最大 bias drift `2e-4 rad/s` 的 5 倍。该 domain 不授权跨 update 缓存 PIM；每次
graph rebuild/outlier-reopt outer round 仍须以 round biasHat 从 raw samples fresh build。

LM round 结束若 final candidate 超域，必须按 §3.3 进入显式 outer relinearization；不得 clamp、
inflate noise、在 `evaluateError()` 内重积分或把超域 candidate 直接 commit。总共最多 3 个
fixed-PIM rounds（首轮 + 最多 2 个额外 rounds）；
最终仍有 factor 超域、或 41-sample gate/Jacobian gate 失败，结论为 `HYPOTHESIS_FAIL` 并 `STOP`。

### 6.5 Noise、whitening 与 cost 门

冻结 fixture：

```text
dt      = 0.25 s
m_gyr_nd = 4e-4 rad/s/sqrt(Hz)
m_gyr_rw = 1e-3 rad/s^2/sqrt(Hz)
Sigma_R = 4e-8 I3
Q_b     = 2.5e-7 I3
```

rotation residual：

```text
r_R          = [1, -2, 3]e-4 rad
whitened r_R = [0.5, -1, 1.5]
squared norm = 3.5
factor error = 1.75
```

RW residual：

```text
r_b          = [2, -1, 3]e-4 rad/s
whitened r_b = [0.4, -0.2, 0.6]
squared norm = 0.56
factor error = 0.28
```

总 error 必须为 `2.03`。matrix comparison 使用 relative `1e-12`、absolute `1e-18`；cost
absolute tolerance `1e-12`。以下 mutants 必须全部被杀死：

- density 直接当 covariance；
- missing square；
- `Delta t` 使用除法而非乘法；
- 把 variance 传给 `Diagonal::Sigmas`。

通过只资格化这里声明的 isotropic synthetic model，不资格化任何真实 calibration。

### 6.6 Observability 门

使用 `G0..G3`、3 条 RW 与 3 条 rotation observations 的独立 whitened Jacobian fixture：

```text
dt          = 0.25 s
m_gyr_nd    = 4e-4
m_gyr_rw    = 1e-3
prior_sigma = 0.1
```

必须分别验证：

1. Between-only `H` 为 `9x12`，rank `9`、nullity `3`；
2. Between + root prior 虽为 rank `12`，但没有 rotation observation 时只能分类为
   `PRIOR_ONLY`，不得声称 bias recovery；
3. 加入 3 条 rotation observations 后，whitened `H` rank 为 `12`，四个 singular values
   各按三轴重复：

```text
3888.522624890523
3037.265646588222
1823.594963735669
1008.221501460588
```

4. `condition(H)=3.8568138244 <=4`；
5. `condition(H^T H)=14.8750128761 <=16`。

独立 oracle 对上述 listed singular values 与两个 listed condition actual 的复算，必须同时满足
absolute error `<=1e-9` 且 relative error `<=1e-10`；`<=4/<=16` 则仍是 observability science gate，
两者不得混用。

rank tolerance 冻结为：

```text
sigma_max * max(m,n) * epsilon_double
```

不能因为 root prior 令代数矩阵满秩，就把无 rotation information 的结果叫 observable。

### 6.7 Zero-drift 与 exact-zero 门

constant-bias fixture 完整复用 §5.1 的 14-state visual/keyframe/window schedule、13 个 interval 与
三 sample timestamps，只把 §5.2 的 bias truth 改为：

```text
b_i = [0.012, -0.018, 0.025] rad/s
```

第三条完整 interval 后：

- current bias 每轴 absolute error `<=1e-6 rad/s`；
- 后续 reported bias 每轴 range `<=1e-6 rad/s`。
- frame 0 最少输出 `m_bias_radps=nullopt`；frame 1..13 最少输出 present、finite 的 current bias，
  并记录每帧 status、break reason 与 factor/root counts。

独立 oracle worst 约为 `3.13e-8 rad/s`。

exact zero-bias/zero-rotation fixture 同样复用该 14-state schedule、timestamps 与最少输出集合，只把
全部 gyro samples 和 bias truth 设为 exact zero：

- reported bias norm `<=1e-12 rad/s`；
- gyro-on 与 gyro-off pose rotation difference `<=1e-12 rad`；
- total gyro/RW/root cost `<=1e-18`。

### 6.8 Determinism 与 off identity 门

在加入任何 online-bias product declaration 前，必须先以 authority commit 的旧 public estimator
interface 建立 `AuthorityBaselineVisualControl`。它复用 §5.1 的 calibration、20-point visual
geometry、14-state `K,K,N×12` schedule 与全部旧 visual options，但不引用尚不存在的
`GyroInterval`、`GyroBiasOptions`、`m_gyro_interval`、`m_gyro_bias` 或新 diagnostics。test source 中 helper
与 control test 的 byte-exact 区域（含 marker 行）固定为：

```text
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_BEGIN
...
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_END
```

该冻结 helper 以固定 field order 编码 frame 0..13 的 status、message、estimate timestamp/pose 与
**全部旧** `UpdateDiagnostics`；整数、optional/vector/string presence/length/content 和每个 double 的
IEEE-754 bit pattern 都进入 canonical byte stream。首次运行发布完整 canonical bytes 及其 SHA-256，
并冻结 control region SHA-256、baseline source commit/tree/blob。该运行必须发生在任何 product
declaration 之前；同一新 build 内仅比较不同 gyro payload 不能替代这份 authority baseline。

随后 capability RED 的主 test 另用独立 marker region：

```text
// PHAD_M4_ONLINE_BIAS_NO_EVICTION_BEGIN
...
// PHAD_M4_ONLINE_BIAS_NO_EVICTION_END
```

后续允许在两个 marker region 之外扩展同一 test file；不得改动两个冻结 region。最终 verifier 必须
用 RED 前冻结的 repo-external script byte-exact 抽取两段，逐一对拍 region SHA 与各自 historical
commit/blob；不要求后续增长后的整文件 SHA 永远等于 RED 时的 SHA。

§5 的 14-state main recovery fixture 用 16 个 fresh estimator instances 独立运行；每次都收集
frame 0..13 的最少输出集合。以下必须逐项 bit-identical：

- 每帧 `UpdateStatus` 与 message；
- `GyroBreakReason`、rotation/RW/root counts 与实际 extra-round count；
- scalar 与 `m_window_biases` 每项的 optional presence、frame index、timestamp、顺序；
- 所有 pose、bias 与旧 diagnostics double 的 bit pattern。

关闭门另比较 `m_gyro_bias=nullopt` 下：

1. measurement 无 gyro interval；
2. interval 合法；
3. interval 少样本、timestamp 逆序/重复、endpoint mismatch；
4. gyro 含 NaN/Inf；
5. duration overflow candidate。

五组都必须把同一冻结 encoder 产生的 canonical bytes/digest 与上述 authority baseline 逐帧对拍，
得到相同的 status、message、estimate 和旧 diagnostics 各字段 bit pattern；只做同一新 build 内的
payload 互比不够。`UpdateDiagnostics::m_gyro` 必须为 `nullopt`。本片不改既有 config hash 输入，也不改
`est.tum`、`kf.tum`、`diag.csv` 三主 artifact schema。

### 6.9 Exact-count 与 lifecycle 门

除 14-pose 主门外，至少冻结以下独立测试：

1. **连续 3 state：** exact `2 rotation + 2 RW + 1 root`。
2. **rejected endpoint：** `t0 accepted, t1 visual rejected, t2 accepted(packet=t1→t2)`；t2
   `+0/+0`、reason `kRejectedEndpoint`、two components/two roots；t3 exact 后只新增 t2→t3。
3. **missing interval：** 非首帧当前视觉可 `kOk`、`+0/+0`、新 root、reason
   `kMissingInterval`；下一 exact 只从当前 state 起链。首帧 absent 另断言 `kNone`。
4. **declared gap：** 非首帧与 missing 相同，但 reason `kDeclaredGap`；不得读取 gap payload；
   首帧 gap 另断言 `kNone`。
5. **reanchor：** all-new rich keyframe 触发 `seedSegment` 后旧 links/roots 全消失；新 segment
   `+1 root`、reason `kSegmentChange`；下一 same-segment exact interval 才 attach。
6. **interior non-KF eviction safety：** `window_size=3`，schedule `K0,K1,N2,N3`；N3 入窗后
   现策略逐出 N2，必须保留 K0→K1，删除 N2→N3，绝不生成 K1→N3，reason
   `kEvictedEndpoint`。surviving graph 为 3 G、1 rotation、1 RW、2 roots。
7. **malformed/precedence matrix：** `<2` samples、front/back mismatch、duplicate/reverse、
   non-finite gyro、duration overflow、invalid derived noise 各自 `kRejected`；组合错误按 §4.2
   exact precedence；gap payload 全忽略；之后一个 exact update 证明 provenance、frame index、bias
   与 window 未变。
8. **unknown-predecessor post-staging hard path：**
   `StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics` 按 §4.4 用 structurally valid
   interval 在 topology classification 命中 unknown predecessor；必须 `kRejected`，scalar null、rounds
   `0`、reason none，`m_window_biases` 与 counts exact 等于 rollback 后 committed `t0` root，再以
   `t0→t2` exact update 与 fresh control 全状态对拍零污染。该门专门动态证明 explicit rollback
   早于 diagnostics materialization，不是 optimizer exception fixture。
9. **writeback/rebuild：** initial LM 和成功 outlier reopt 后 update-local current bias 可见、finite，
   counts 不重复；最终 `kOk` 才 commit，下一 update 使用该 committed bias warm start。
10. **rollback / commit：** failed reopt 回上一成功 graph solve 的 provisional `G/X/L`；四个
   `StereoVoUpdateTransaction.*` direct tests 对 production 完整 state 逐字段闭合 scope-exit、exception、
   explicit rollback 与 commit，并执行 §3.1.2 的 owner-address assertions。outer hard result 只能经唯一
   failure-finalizer，先 rollback 到 staging-entry committed state 再 materialize scalar-null/current topology
   diagnostics；不声明 public optimizer failure 的动态覆盖，也不得返回虚假 `kOk` 或部分 state。
11. **bias outer relinearization：** 主 fixture 首个 `0→约0.025 rad/s` correction 必须触发 fresh-PIM
    extra round；每个 round 内 factor 固定，最终所有 factor delta `<=1e-3`，总 fixed-PIM rounds
    `<=3`（首轮 + 最多 2 个额外 rounds）；每个 update 的 diagnostics 独立计数，完整 vector 必须为
    `[0,1,0,0,0,0,0,0,0,0,0,0,0,0]`，即 frame 1
    `m_relinearization_rounds==1`、final frame 13 `==0`，不是累计 `1`。该 actual count 仅对最终
    `kOk` 有效；最终 `kRejected` / `kFailed` 无条件为 `0`。V1 不构造不可执行的
    deterministic nonconvergent optimizer fixture；
    `kMaxFixedPimRounds=3`、loop condition、超限 stable message 与完整 rollback 只做 source/static
    review，并以临时 loop-bound source mutation 证明 static verifier 能检出无界/错界；不声称取得
    动态 cap-path coverage，也不得拿现有 LM2 failure 冒充 bias cap failure。

其中 visual lifecycle tests 必须复用而不是重写既有触发机制，并在 receipt 锁定下列 source；private
transaction tests 则直接包含 production header，不另造 surrogate：

- low-shared visual reject：[`keyframe_update_test.cpp`](../../tests/estimator/keyframe_update_test.cpp)
  lines 113–141，blob `73274733a90448956090350cf0a4a086b0c29c29`；
- reanchor 与 reject rollback：
  [`stereo_vo_reanchor_test.cpp`](../../tests/estimator/stereo_vo_reanchor_test.cpp) lines 148–229，
  blob `f6ef740f62e7dec2bc6da62d82b25bd3bce1ec3f`；
- successful outlier reopt 与 LM2 failure fallback：
  [`stereo_vo_outlier_reopt_test.cpp`](../../tests/estimator/stereo_vo_outlier_reopt_test.cpp)
  lines 206–253 / 510–579，blob `4bd1e1c7f36425817f73d3923c7829a1544f8c7e`。

## 7. Eviction 的严格分线

14-pose recovery 主门明确 no-eviction。eviction 在 V1 只有负向 safety test：证明 endpoint 消失后
断链、独立 reset、无跨接；**不要求也不允许声明 eviction 后 bias continuity/recovery**。

V1 未解决：原 root 被逐出时，surviving `X/G` 的 joint information、pose-bias correlation、
marginal cost 与 covariance 如何保留。以下任一做法都触发 `STOP`：

- 把 evicted bias point estimate 加固定 sigma 当作 posterior prior；
- 跨 evicted/missing/rejected/gap/segment endpoint 连接 rotation 或 RW factor；
- 声称 point writeback 保留了历史 joint information；
- 仅把 window 加大后声称已支持 bounded online；
- 在 V1 中顺手加入 marginalization、Schur prior 或 covariance handoff。

synthetic PASS 后允许规划的下一 slice 必须单独预注册：

1. surviving `X/G` joint information 的表示；
2. marginal factor 或明确等价物，禁止 point-only；
3. 相对本协议 no-eviction frozen oracle 的 mean 与 information/cost 等价门；
4. correlation、whitening与 repeated rebuild 不 double-count；
5. interior non-KF eviction、连续 front eviction、rollback 与 determinism；
6. 无法保真时的 explicit reset/`INCONCLUSIVE` 语义。

该下一 slice 通过前仍不得 real run。

## 8. 门值来源与禁止解释

| 门值 | 来源 | 允许解释 |
|---|---|---|
| main truth/noise/prior literals | 实现前构造的 synthetic model | 只定义本 fixture，不是产品参数 |
| oracle literal comparison `1e-12/1e-9/1e-10` | 跨 Eigen/libm double 复算预算 | 只做 independent-oracle engineering identity |
| private fixed-pose graph `<=1e-8` | 独立 dense oracle 与 deterministic solver 对拍预算 | 数学实现一致性，不约束 public joint graph |
| public all-state error `<=5e-4` | joint visual preflight worst `~3.4055019255e-4` 向上留界 | 覆盖 `.003 px` fixture，不是 real accuracy |
| drift ratio `[0.85,1.05]` | joint preflight axis `~.937–.943`、L2 `~.941` 的外包络 | 慢变方向/幅度未被 prior 压没 |
| condition `<=4/<=16` | oracle actual `3.8568138244/14.8750128761` 向上取整 | 只验证冻结 H |
| Jacobian `h=1e-7`, error `<=1e-6` | central-difference truncation/roundoff 预算 | analytic derivative 对拍 |
| first-order domain `1e-3` | synthetic per-edge drift `2e-4` 的 5 倍 | fixed-PIM LM linearization 有效域 |
| first-order error `<=3e-8` | 41-sample lattice actual `2.0431967621531336e-8` 向上留界 | cached correction 对 fresh PIM |
| bias J error `<=1e-7` | 27-point actual `2.4420145350312339e-9` 向上留界 | PIM bias derivative 对 full FD |
| total round cap `3` | 1 个首轮 + 最多 2 个额外 relinearization rounds | 不授权无界重积分循环 |
| residual `1e-12`、cost `1e-18` | double precision deterministic zero fixture | 数值零，不是传感器噪声 |
| mutant separation `>=0.02 rad` | 最近错误 left-compose `0.02412947757 rad` 下取安全界 | 杀死列出的七类错误 |
| zero-drift `<=1e-6` | oracle worst `~3.13e-8` 向上留界 | constant synthetic bias stability |
| 16 fresh runs | 固定的 deterministic replication budget | 不是统计置信区间 |

synthetic z truth excursion 为 `12*0.0002=0.0024 rad/s`。它只提供覆盖尺度：大于 Q3 已观察到的
early/late 最大差 `0.001227119335357879 rad/s`；不得把 `0.0024` 写成 Q3 measured drift，也不得
用 Q3 结果反向选择 prior/noise 或放宽本协议门。

所有阈值在 implementation/qualification 前冻结。若独立 oracle、公式或 literal 有错，必须在
第一次 qualification 前以新 protocol version 修订并说明原因；看过 qualification actual 后不得
原地改 V1。

## 9. RED → GREEN evidence protocol

本节定义未来实施证据；本文没有执行这些命令，也不声称任何测试已通过。

### 9.1 RED

authority lock 后、任何 product declaration 或 RED 前，先在 repo 与全部 worktree 之外写入本协议的
external verifier script，冻结 script version、source SHA-256、exact invocation 与 interpreter/tool
versions，再移除其全部 write mode bits并验证。脚本必须先 `realpath` 解析 evidence root、positive
archive、每个 mutation scratch 与 `git worktree list --porcelain` 返回的所有 worktree root（包含
symlink 解析）；任一 evidence/archive/scratch 等于或位于任一 worktree root 下都必须非零退出。
positive archive 与 mutation scratch 必须是不同且互不包含的目录；archive 完成后移除全部 write
mode bits并复验，mutation 只发生在另建的 writable scratch。writable-entry checker 必须用
`find -perm /222` 检出 owner/group/other 任一 write bit，并先对 mode `0644` 文件和 mode `0755`
目录的 owner-only-writable negative fixture 证明 checker 非零 fail closed；禁止使用会漏掉 owner
write bit 的 predicate。该冻结脚本同时负责 §6.8 两个 marker region 的 byte-exact extraction/hash、allowlist、
§3.1.2 production transaction constructor/failure-finalizer/commit/order/guard 与 `Impl` ownership static
checks，以及 §9.3 fresh-restore arms；脚本漂移使 receipt 失效。

脚本第一条 executable statement 必须是 `set -euo pipefail`。所有 `realpath`、
`git worktree list --porcelain`、`find`、`chmod`、`mktemp` 与 `cp` 调用都必须显式捕获并检查 exit
status，任何非零都 fail closed；禁止用 process substitution 隐藏 producer exit，也禁止用
command substitution 中 `find` 的空 stdout 把执行错误误判为“没有 writable entry”。
staged/committed allowlist 检查
统一使用 `--no-renames --name-status --diff-filter=ACMRTD`，解析并验证 `C/R` 的 old/new 两个 path，
同时显式处理 `A/M/T/D`；未知或 malformed status 一律非零退出。

第一步不是 capability RED，而是 §6.8 的 authority-baseline control：只新增 product test source 与
existing-target CMake entry，control/helper region 只使用旧 public interface。先在没有任何 online-bias
declaration 的 source tree 运行并冻结 14-frame canonical bytes/digest、control region SHA-256、baseline
commit/tree 与 test blob；control 本身必须 GREEN。

随后才在独立 marker region 加入 public estimator seam 的
`NoEvictionFourteenPoseRecovery`，在 gyro types/state/factor 不存在时得到预期 compile/test failure。
RED receipt 至少记录：

- design commit/tree/blob/SHA-256；
- authority-baseline canonical bytes/digest、control region SHA-256 与 baseline commit/tree/test blob；
- `NoEvictionFourteenPoseRecovery` region SHA-256 与 RED commit/tree/test blob；
- external verifier script version/source SHA-256、exact command 与通过的 repo/worktree exclusion；
- writable checker 在 `0644` 文件/`0755` 目录 negative fixture 上的预期非零 exit，以及实际
  verifier 的 `/222` 空列表结果；positive archive 此时只要求 canonical empty directory 已存在，
  其 `/222` 空列表门延后到 clean GREEN 内容复制完成并移除全部 write mode bits 之后；
- exact build/test command、cwd、compiler 与 exit code；
- 缺失 symbol/assertion，证明失败来自尚未实现的 online bias capability，而不是路径、依赖或
  harness 错误；
- 当时 staged/committed diff 与 allowlist check。

两个 marker region 在各自首次运行前必须冻结；之后修改任一 region 的 fixture、helper、options、
expected 或 threshold 都使原 baseline/RED receipt 失效，必须从相应 historical identity 重建证据。
同一 test file 后续可在 marker 外增加 lifecycle tests，因此只把 RED 时的整文件 SHA/blob作为历史
provenance，不把它误作最终文件必须永不增长的 identity。

不允许为得到 RED 添加空实现、identity fallback 或“先返回 prior mean”的假 success。

独立 oracle target 必须先于 product GREEN 成为可复现基准，但它不能链接 product/GTSAM，也不能
从 product test include literals helper。oracle target 失败是 protocol/harness failure，不得用 product
actual 回填 expected。

### 9.2 GREEN 顺序

1. 独立 oracle：复算 linear solution、whitening、rank/SV/condition 与 frozen literals；
2. 唯一 reducer：让 Q2 existing tests 与 interval validation/endpoint trapezoid tests 过门；
3. private factor：residual、whitening、cost、Jacobian 与 mutation gates；
4. fixed-PIM lifecycle：embedded AHRS first-order correction 对 fresh full reintegration；
5. private State/RAII：四个 direct transaction gates 先 RED 后 GREEN；
6. `G/RW/root` fixed graph：observability 与独立 oracle；
7. public estimator：14-pose recovery、zero drift、non-identity extrinsic、exact counts；
8. lifecycle：breaks、malformed、unknown-predecessor rollback-before-materialize、writeback/rebuild 与
   eviction safety；
9. off identity 与 16-run determinism；
10. 执行 §9.3 的 production-module activation/deletion、3 个 transaction source arms、static-cap 与
    12 个 numerical-semantic mutant arms；
11. 只运行受影响的 `phad_estimator_tests`、独立 oracle target 与既有 Q2 tests；V1 不要求或声称
    全量 unit regression。

GREEN receipt/result 必须记录每个 gate 的 threshold、actual、PASS/FAIL，以及：

```text
N_G, N_rotation, N_rw, N_root
private fixed-pose G0..G13 oracle and actual
public m_window_biases m_frame_index/m_timestamp/presence/value for every frame
public truth errors, frame1/frame13 drift ratios, round-count vector
rank, singular values, condition(H), condition(H^T H)
whitened vectors and graph error
max Jacobian coefficient error
first-order-vs-full fixture errors, final fixed-PIM domain acceptance and per-update extra-round counts
all deletion/graph/static-cap exits, all 3 transaction source-arm exits, and all 12 numerical-semantic mutant exits
zero-drift / exact-zero actuals
16-run bitwise digests
authority-baseline canonical bytes/digest, both frozen marker-region identities and off-identity comparisons
all typed break reasons, precedence cases, provenance, unknown-predecessor rollback-before-materialize result/control digest,
four direct transaction field/address checks, owner-only-writable 0644/0755 negative-check exits,
and production-use static identity
fixture source blobs, historical baseline/RED commit+test blobs, external verifier identity and full options snapshot
```

mutation evidence 应逐个施加单一错误，确认 targeted verifier 非零退出后恢复；不得给 production
target 留长期 mutant 开关。

### 9.3 Production-module activation/deletion、transaction source arms、static-cap 与 numerical-semantic verifier

本 verifier 在原始 RED 与 clean GREEN 之后执行，不能替代 §9.1 的 capability RED：

1. 用 §9.1 已冻结的 external verifier 重新证明 evidence/archive/scratch canonical realpaths 均在 repo
   与全部 worktree 之外且互不包含；把 clean GREEN 的 exact source tree、commands、logs、test
   binaries/digests 复制到 positive archive，记录 path、tree/blob/SHA-256，移除全部 write mode bits
   并验证无 writable entry；从 archive 复制到独立 scratch 后复跑定向 tests，形成 positive arm；
2. deletion arm A 只删除 `gyro_interval_reducer.cpp`，保留 CMake source entry；configure/build
   必须因 missing production source 非零退出；
3. deletion arm B 从同一 archive fresh restore 后只删除 `gyro_rotation_factor.cpp`，同样必须因
   missing production source 非零退出；
4. graph-activation arm 再次 fresh restore，只移除 `StereoVoEstimator::Impl` 向 graph 注册 gyro
   state/factor 的调用；build 必须成功，而 14-pose recovery test 必须非零退出；
5. transaction-activation arm 从 fresh restore 只在 transaction constructor 后插入一次提前
   `commit()`，保留末尾 commit；build 必须成功，而 source/static verifier 必须因 commit 数量/顺序
   破坏而非零退出；
6. transaction-constructor-wrong-swap arm 从 fresh restore 只在 production transaction constructor 的
   deep-copy 后加入 `m_owner.swap(m_before)`，把 snapshot 错当 live owner；build 必须成功，四个
   direct gate 的 constructor-address assertion 必须非零退出，其中 commit gate 还必须证明 commit 不能
   恢复进入地址。这一动态 address gate 而非单纯字段 comparator 是 wrong-swap 的指定 kill。
7. transaction-rollback-order arm 从 fresh restore 只交换唯一
   `finalizePostStagingHardResult(...)` 中 explicit `transaction.rollback()` 与 gyro diagnostics
   materialization 的先后；build 必须成功，
   `StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics` 与 source/static verifier 必须均非零
   退出。
8. static-cap arm 只破坏或移除 `kMaxFixedPimRounds=3` 的 bounded loop condition；source/static
   verifier 必须非零退出。它只证明 cap 存在且边界未漂移，不声称动态触达 optimizer cap failure。

production source 冻结 constructor/rollback-before-diagnostics/commit 三个 marker。static verifier 必须证明：唯一 constructor 位于
gyro/keyframe guard 外、支配 seed/reanchor/normal 三个 staging branch；唯一 commit 位于 result 完整
形成后、唯一成功 `kOk` return 前；两者顺序固定。它还必须证明 transaction region 内不存在
直接 post-staging `kRejected` / `kFailed` return，所有 hard branch 都调用唯一
`finalizePostStagingHardResult(...)`，而且该 finalizer 内唯一 explicit rollback 严格位于从恢复
`*m_state` materialize diagnostics 之前，marker 不在 guard 内且只出现一次。static verifier 同时证明
`Impl` 的 mutable ownership 只有
`unique_ptr<StereoVoUpdateState>`，不存在同名 legacy mutable field。删 state field 会让 direct test 的
逐字段 comparator 编译失败；alias/no-deep-copy、缺 destructor rollback 与 commit/rollback 错误由
字段/lifecycle assertions 杀死；constructor wrong-swap 则精确由四门的 constructor-address assertion 与上述
production mutant 杀死，不再冒充为普通逐字段回滚已证明。上述 static/source evidence 不冒充 public
outer exception 动态覆盖。

上述 transaction activation、constructor wrong-swap 与 rollback-order 是 **3 个独立 transaction source
arms**，不计入下面的 numerical semantics。另外必须对 production source 执行以下 **12 个**
独立 numerical-semantic mutant arms；test-local frozen separation
只提供数值预期，不能替代这些 production-source mutations：

| arm | production-source 单一错误 |
|---|---|
| factor 1 | 反转 `candidate-biasHat` increment 的符号 |
| factor 2 | 忽略 candidate bias，始终使用 `biasHat` |
| factor 3 | 把 rad 当 degree |
| factor 4 | 把 degree 当 rad |
| factor 5 | 使用 visual relative rotation inverse |
| factor 6 | 把正确 right-compose 改为 left-compose |
| frame | 在 non-identity estimator gyro staging 中错误施加一次 `R_B_left` frame rotation |
| noise 1 | 把 gyro noise density 直接当 covariance |
| noise 2 | 漏掉 density 的平方 |
| noise 3 | covariance 对 `Delta t` 使用除法而非乘法 |
| noise 4 | 把 RW variance 传给 `Diagonal::Sigmas` |
| initializer | 交换 exact-link 与 component-root 的 selected value source |

每个 numerical-semantic arm 都必须从只读 positive archive fresh restore 到新的 writable scratch，只改
production source 的一个 hunk，记录 exact patch、受影响文件 before/after SHA-256；configure/build
必须成功，对应 `GyroRotationFactor`/non-identity public/fixed-PIM/noise/
`GyroBiasInitialValue` targeted test 必须非零退出。不得改 test、expected 或 threshold。每个
deletion、graph activation、3 个 transaction source
arms、static-cap 与 numerical-semantic arm 后
都丢弃整个 scratch；下一 arm
再次 fresh restore。跨不同 scratch 的 hard identity 只比较 source tree/逐文件 SHA、两个 marker
region、exact command/toolchain identity 与 targeted semantic test outcome；positive arm 自身的 binary
digest 可记录为 provenance，但在没有另行冻结并资格化可复现编译映射（含路径、build-id 与环境）时，
不得要求不同 scratch 的 binary bytes/SHA 相同。外部脚本仍必须重新抽取 §6.8 两个 marker region
并与 historical SHA/blob 逐字一致。不得以长期 feature switch、空实现、删 assertion 或修改
expected 制造结果。

### 9.4 Verdict

| verdict | 条件 | 权限 |
|---|---|---|
| `PASS` | 所有 identity、allowlist、mechanism、negative 与 regression gates 全过 | 仅保留 default-off private mechanism，并写下一 eviction-handoff plan |
| `HYPOTHESIS_FAIL` | harness 有效，但任一 recovery/math/noise/Jacobian/lifecycle gate 不过 | 保存负结果，停止扩权 |
| `INCONCLUSIVE` | support/observability/fixture attribution 不足，无法评价机制 | 不把 prior-only 当 pass；修 protocol 后新版本重来 |
| `HARD_ERROR` | build/harness/identity/allowlist/receipt 损坏 | 不产生科学结论，修基础设施后重新 preflight |

## 10. 分阶段 authority 与 exact allowlist

### 10.1 Pre-implementation exact-six authority transaction

production RED 之前先完成一次独立的 exact-six authority transaction；这一阶段只允许：

```text
docs/research/2026-08-19-note-m4-online-gyro-bias-synthetic-design.md
docs/adr/0002-stage-gated-gyro-only-bias-state.md                 # 新建
docs/design/conventions.md
docs/design/roadmap.md
docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md # 新建
phad/estimator/README.md
```

该 transaction 只锁定 protocol、ADR、canonical key 窄例外、roadmap authority、可执行计划与
estimator 当前 module contract；不得
修改 product code、tests 或 CMake。`docs/architecture/**`、除本 design 外的 `docs/research/**`，
尤其 Q3 design/result，全部只读；Q3 的 `HYPOTHESIS_FAIL/STOP` 不得被重写或重解释。

六文件必须在同一 commit/tree 上接受 fresh Standards review 与 fresh Spec review；两轴都达到 zero
finding 后，记录六个文件及 commit/tree 的 blob/SHA-256 exact identity。任一 finding 修复都会使
旧 review identity 失效，必须双轴 fresh 重审。只有 authority commit 已存在且 identity lock 完成，
§10.2 production allowlist 才另行生效；authority review 不能与 production RED 混成一个证据回合。

### 10.2 Production implementation allowlist

authority lock 后，V1 production RED→GREEN 只允许以下路径；其中标注“新建”的文件当前不存在：

```text
phad/estimator/types.hpp
phad/estimator/stereo_vo_estimator.cpp
phad/estimator/gyro_rotation_predictor.cpp
phad/estimator/internal/gyro_bias_initial_value.hpp         # 新建，header-only PRIVATE
phad/estimator/internal/stereo_vo_update_transaction.hpp    # 新建，header-only PRIVATE
phad/estimator/internal/gyro_interval_reducer.hpp          # 新建
phad/estimator/internal/gyro_interval_reducer.cpp          # 新建
phad/estimator/internal/gyro_rotation_factor.hpp           # 新建，PRIVATE
phad/estimator/internal/gyro_rotation_factor.cpp           # 新建
tests/estimator/stereo_vo_online_gyro_bias_test.cpp         # 新建，product tests
tests/estimator/online_gyro_bias_oracle_test.cpp            # 新建，independent oracle
CMakeLists.txt
```

production implementation/qualification 期间，§10.1 的六个 authority files 与 Q3 全部只读；本
production allowlist 独立生效，不反向授权修改 roadmap、plan、ADR、conventions 或 design。

private files 使用 namespace `phad::estimator::internal`。`GyroRotationFactor` 必须继承 canonical
`gtsam::NoiseModelFactorN<Pose3, Pose3, Vector3>`、内部组合 `AHRSFactor`、实现 `clone()`，并通过
`Pose3::rotation()` Jacobian 把 Rot3 derivative 链式提升为 `[H_R, 0]`；不得复制官方 residual 实现。

`CMakeLists.txt` 只允许：

- 把两个 new private `.cpp` 加入既有 `phad_estimator`；
- 把 product test source 加入既有 `phad_estimator_tests`；
- 新增 `phad_online_gyro_bias_oracle_tests` test-only target，只链接 `Eigen3::Eigen` 与
  `GTest::gtest_main`，注册为 unit test；
- 保持 GTSAM 对 `phad_estimator` 为 `PRIVATE`，不新增 product library、runner、app 或长期实验
  target，不手写裸 `-lgtsam`。

两个 header-only private seams 不增加 CMake source/target；transaction direct tests 复用同一个 product
test source 与既有 `phad_estimator_tests`。

明确不在 allowlist：

```text
apps/**
phad/sync/**
phad/sensor/**
phad/frontend/**
phad/eval/**
phad/bench/**
docs/design/roadmap.md
docs/plans/**
docs/research/2026-08-12-note-m4-minimal-gyro-slice-design.md
docs/research/m4-minimal-gyro-q3-*.md
```

### 10.3 Post-verdict bookkeeping

只有 §9.4 verdict 已生成后，才开启一个不含 product code 的 bookkeeping transaction：

```text
docs/research/2026-08-24-note-m4-online-gyro-bias-synthetic-result.md       # 新建，完整 receipt
docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md # 只更新状态/证据链接
docs/design/roadmap.md                                             # 只记录 verdict/下一权限
```

design、ADR、conventions、`phad/estimator/README.md` 与 Q3 继续只读；bookkeeping 不得回改
truth、threshold、options 或 oracle。
`PASS` 后获准的新 eviction-information-handoff plan 是下一独立 authority slice，不得夹带进本
qualification implementation/result commit。`HYPOTHESIS_FAIL`、`INCONCLUSIVE`、`HARD_ERROR`
同样必须写 result/receipt，但不扩 production 权限。

## 11. STOP conditions 与非结论

出现以下任一情况立即停止，不继续加补丁或扩大实验：

1. 任一 frozen gate `FAIL`/`INCONCLUSIVE`；
2. 为过门需要修改 truth、noise、prior、window schedule、threshold 或 oracle；
3. factor 直接约束 translation，或需要复制 `Rot3` state；
4. Q2 wrapper 与 private PIM 出现两套 endpoint reducer；
5. 在一次 LM round 内更换 PIM `biasHat`、在 `evaluateError()` 内重积分，或误以为
   `resetIntegration()` 会换 bias；
6. 3 个总 fixed-PIM rounds（首轮 + 最多 2 个额外 rounds）后仍有 factor 超过 §6.4 domain，或
   frozen correction/Jacobian gate 失败；
7. `N_G != N_X`、`N_rotation != N_rw` 或 forest identity 不闭合；
8. malformed interval 部分污染 window/bias，或 off path 读取 gyro content；
9. missing/rejected/gap/segment/evicted endpoint 被跨接；
10. 需要用 point-only prior 冒充 eviction information handoff；
11. 需要修改 apps、真实 config/artifact schema，或运行 real/GT/ATE/RPE；
12. authority-baseline digest、任一 marker region 或 external verifier identity 漂移，或 evidence/
    archive/scratch 位于 repo/worktree 内；
13. 任一 direct transaction field/address gate、production constructor/failure-finalizer/commit/order/guard
    static gate、unknown-predecessor rollback-before-materialize public gate，或 3 个 transaction source arms 任一未闭合；
14. 任一 production-source numerical-semantic mutant 未被对应 targeted test 杀死；
15. 试图据 synthetic PASS 声称 Q3 已通过、旧 Q4 已授权、完整 VIO 完成或可以 default-on。

本协议最终只能回答：在一个冻结的 no-eviction synthetic graph 中，online gyro bias state、
rotation-only factor、RW、root prior、noise 与 lifecycle 是否按声明工作。它不能回答真实数据收益、
长期窗口信息保真、完整 IMU observability、产品鲁棒性或默认策略。

## 12. Source anchors

- 当前 public types：[`phad/estimator/types.hpp`](../../phad/estimator/types.hpp)
- 当前 PIMPL interface：[`stereo_vo_estimator.hpp`](../../phad/estimator/stereo_vo_estimator.hpp)
- 当前 graph rebuild/lifecycle：[`stereo_vo_estimator.cpp`](../../phad/estimator/stereo_vo_estimator.cpp)
- Q2 endpoint-trapezoid helper：[`gyro_rotation_predictor.cpp`](../../phad/estimator/gyro_rotation_predictor.cpp)
- gyro noise/rw units：[`imu_parameters.hpp`](../../phad/sensor/imu_parameters.hpp)
- estimator module contract：[`phad/estimator/README.md`](../../phad/estimator/README.md)
- GTSAM [`PreintegratedAhrsMeasurements`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L32-L106)
- GTSAM [`AHRSFactor` residual](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L92-L168)
- GTSAM gyro-only [factor/RW test](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L410-L468)

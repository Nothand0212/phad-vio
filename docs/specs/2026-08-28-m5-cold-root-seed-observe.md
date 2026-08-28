# M5 cold-root current-path Observe spec

本文档描述当前约定，不是绝对约束，会随项目开发修订。

- 日期：2026-08-28
- 状态：**已定稿**
- issue：[#48](https://github.com/Nothand0212/phad-vio/issues/48)
  （epic：[#42](https://github.com/Nothand0212/phad-vio/issues/42)）
- control：`main@34c309196440710be86d0a05e32901d58bfdd9aa`
- mechanism baseline：`db226563c9a386bc70e4f19665ec909bd80905ad`
  / `default_0337287b`
- 前序合同：
  [mapped-landmark bearing continuity](2026-08-27-m4-mapped-landmark-bearing-continuity.md)
- 现状证据：
  [clean EuRoC-11 checkpoint](../benchmark/m4/mapped-landmark-bearing-continuity_db22656_0337287b.md)
  与
  [evidence JSON](../benchmark/m4/mapped-landmark-bearing-continuity_db22656_0337287b.evidence.json)
- 资格：
  [证据门控的信息接入](../agents/evidence-gated-integration.md) Q1 Observe

## 1. 目标与可证伪问题

本片只回答：

> `V2_03_difficult` 在 `96.6–116.7 s` 的 289 个 terminal
> `initializing` frames 中，每一帧实际执行到了 cold-root current path 的哪一
> phase、被哪一条已有 gate 挡住；哪些 root attempts 真实发生；这些事实能否
> 从逐帧 artifact 独立复算，且不改变任何既有 estimator 行为？

本片只取得 Q1 Observe 权限：导出当前实现已经计算或已经作出的判定。它不新增
root initializer，不计算 shadow observability / conditioning，不改变 seed、graph、
posterior、segment 或输出轨迹。

Observe PASS 证明的是“当前拒绝机制可审计”，不是“root 已能恢复”，更不是
“恢复后的 root 属于旧 world frame”。

## 2. 冻结证据与判断边界

以下事实是本片输入，不由本片重新判定：

| 项 | 冻结事实 |
|---|---|
| `#47` formal gate | `FAIL`；唯一失败分量是 V2_03 endpoint coverage |
| V2_03 continuity | segments `18→9`、completion `0.535138→0.593961`、段内 RMS `0.061664→0.060236 m`、绝对段间分量 `2.019657→1.981580 m` 通过 |
| V2_03 coverage | `0.791345→0.761354`，未通过冻结 floor |
| candidate last ok | `96.5 s` |
| terminal epoch | `96.6–116.7 s`，289 个 `initializing` frames |
| terminal observation | `num_obs` 最大 200；已有 `num_disparity` 最大 9 |
| 当前 seed threshold | `min_seed_observations=10` |
| 当前 run | `--estimator-enable-moving-bootstrap`；accumulated seed 未开启 |

`num_disparity < 10` 是强约束线索，但现有 artifact 不能逐帧区分 bootstrap、
keyframe、population、geometry 与后续 graph stage。Observe 不得在新 artifact
产生前，把线索写成预定结论。

完整数值、逐序列 tail、机制计数和 V2_03 时间线继续以 checkpoint 与 evidence
JSON 为准；本片不追溯改变 `#47` 的正式判定。

`db22656` 机制合入 developmental `main` 只确立本片的 control 与 mechanism
baseline；它不改变 `#47` formal gate `FAIL`，也不构成对冻结 gate 的补判。

## 3. 领域术语

本 spec 使用以下互不替代的词：

| 术语 | 本片含义 |
|---|---|
| cold-root frame | `VioEstimator::update()` 进入调用时 estimator 尚未 initialized，且输入已通过进入当前 root path 所需的既有校验 |
| phase | 本次调用在 cold-root current path 上最终停止或提交的语义阶段 |
| primary reason | 本次调用在该 phase 上唯一的等待、失败或完成原因 |
| root attempt | 已通过前置 gate，并实际进入现有 `seedRoot()` 调用的一次执行；不是 `initializing` 的同义词 |
| `attempt_id` | estimator instance 内 root attempt 的诊断身份；存在不代表成功提交 |
| `not_evaluated` | 本次调用没有执行该 gate 或不属于 cold-root current path；不表示通过、失败或数值零 |
| root commit | root attempt 经过现有 graph 与 validation，并随 `VioUpdateTransaction` 原子提交为 `kOk` |

`segment`、active map、recovery、world frame 与 cross-root alignment 维持既有合同。
本片不把 active-map reset 后的独立 local root 称为 relocalization。

## 4. Module、interface 与 seam

`VioEstimator` 继续是拥有 root eligibility 与 transaction 的 deep Module：

1. `VioEstimator::update(const VioMeasurement&, bool)` 仍是唯一测量 interface；
2. 不新增 public initializer、observe method、callback 或第二条 update seam；
3. estimator implementation 直接从真实控制流填写 typed diagnostics value，不解析
   自己的 `message` 文本；该 value 允许 `std::optional`，但不含动态容器、原始
   measurement 或 GTSAM 业务对象；
4. `UpdateDiagnostics` 增加一个 `ColdRootObserveDiagnostics` 成员；
5. `OfflineVoSession` 只是 adapter：逐字段复制并写入 `diag.csv`，不推导 reason、
   不聚合成 policy、不改变 keyframe/drop/stop 行为；
6. bootstrap nodes、pending seed map、GTSAM camera、factor graph、Values、PIM、
   Jacobian/Hessian 和 transaction snapshot 均留在 implementation 内。

Observe 数据始终由 estimator 生成；artifact 是否存在不能反向改变 estimator 的
gate 执行。

## 5. Current-path phase 与 primary reason

### 5.1 Phase token

public enum 与 CSV token 一一对应：

| token | 语义 |
|---|---|
| `not_evaluated` | 本次调用不属于可归因的 cold-root current path，或被既有输入 / provenance hard error 抢先终止 |
| `bootstrap` | 现有 static/moving bootstrap readiness 未通过，或 bootstrap timeout |
| `keyframe` | bootstrap 已 ready、视觉非空，但当前 packet 不是 keyframe |
| `stereo_population` | 当前实际 seed population 未达到门限，或 accumulated population 仍在累积 |
| `root_geometry` | 已实际调用 `seedRoot()`，但现有 backprojection/front-facing/non-empty gate 拒绝 |
| `current_graph` | `seedRoot()` 已成功，现有 root graph build/solve/validation 未完成 |
| `commit` | 现有 root transaction 成功提交 |

enum 声明顺序不表示所有调用都按该顺序访问。当前实现会在 keyframe gate 前处理
空 observations；这种调用归入 `stereo_population`，population 为真实的 `0`，
keyframe gate 为 `not_evaluated`。

### 5.2 Primary reason token

| token | 合法 phase | 语义 |
|---|---|---|
| `not_evaluated` | `not_evaluated` | 没有 cold-root eligibility 结论 |
| `evidence_insufficient` | `bootstrap` | 当前 bootstrap evidence 尚不能让现有 static 或 moving path ready |
| `timed_out` | `bootstrap` | 命中既有 bootstrap timeout |
| `keyframe_required` | `keyframe` | 当前非 keyframe |
| `population_insufficient` | `stereo_population` | 实际比较 population 小于 `min_seed_observations` |
| `population_accumulating` | `stereo_population` | accumulated seed 已开启，但 pending unique population 仍不足 |
| `geometry_rejected` | `root_geometry` | 真实 root attempt 被现有 geometry gate 拒绝 |
| `graph_build_failed` | `current_graph` | 现有 root graph build / initial graph construction 失败 |
| `graph_solve_failed` | `current_graph` | 现有 root graph LM / linear system 失败 |
| `graph_validation_failed` | `current_graph` | optimized navigation state / landmark 或提交前现有验证失败 |
| `committed` | `commit` | root transaction 成功，返回 `kOk` root |

primary reason 从实际 branch 直接产生。`result.message` 继续用于人类错误详情，但
不是 reason 的 authority，也不是 CSV parser 的输入。

`MeasurementDiscontinuity`、raw interval invalid、observation invalid 和 active
segment 的 `kOk/kVisualOutage` 均使用 `not_evaluated/not_evaluated`；它们不能
冒充 cold-root rejection。

## 6. Typed diagnostics 合同

### 6.1 Gate state

每个 gate 使用同一稳定三态：

| token | 含义 |
|---|---|
| `not_evaluated` | 本调用没有执行该 gate |
| `passed` | 本调用真实执行且通过 |
| `failed` | 本调用真实执行且未通过 |

`ColdRootObserveDiagnostics` 至少包含以下 gate：

1. `bootstrap_gate`；
2. `keyframe_gate`；
3. `stereo_population_gate`；
4. `root_geometry_gate`；
5. `imu_excitation_gate`；
6. `conditioning_gate`；
7. `initialization_solve_gate`；
8. `current_graph_gate`；
9. `commit_gate`。

当前 main 没有正式 IMU excitation gate、conditioning calculation 或 staged
initialization solve，因此第 5–7 项在本片必须始终为 `not_evaluated`。不得用
bootstrap variance、LM 是否成功、数值零或代理 condition number 填成 passed。

`current_graph_gate` 只描述 root 成功 seed 后进入的既有 full-state graph；它
不是 M5 initialization solve。

### 6.2 Optional 数值

所有只在部分 phase 有意义的数值在 public diagnostics value 中使用
`std::optional`：

- `nullopt` 只表示 `not_evaluated`；
- 数值 `0` 是已观测的真实零值；
- 一个 predicate 的 value 与本次实际 threshold 必须成对存在；
- 不允许用 `-1`、NaN、无穷或默认零表达缺失。

这里的 public 类型是允许 `std::optional` 的 typed value，不要求
trivially-copyable。field presence 由下表冻结，不能按“整行是否初始化”统一清空
或补零：

| field | 存在条件 |
|---|---|
| phase、reason、所有 gate、bootstrap path、seed origin、geometry result | 每个返回的 `VioUpdateResult` 都存在；未执行时写稳定 `not_evaluated` token |
| `current_positive_disparity_count` | update entry 为 uninitialized，且 raw IMU、observation 与 bootstrap provenance 的 cold-root pre-gate validation 均已通过；bootstrap early return 也存在，active/invalid/discontinuity 不存在 |
| `enable_accumulated_seed` | 与上一行相同；这是实际 option snapshot，不表示 accumulated branch 已执行，`false` 是真实值 |
| bootstrap sample/duration/stat 与各自 threshold、timeout | current main 实际完成本次 `bootstrapStats` 与 duration 计算；任一 value/threshold predicate 成对存在 |
| `enable_moving_bootstrap` | static path 未 ready，控制流实际读取 moving option；`false` 是真实值 |
| moving suffix sample/duration/stat 与 epsilon threshold | moving option 已开启、`recentBootstrapBegin` 返回值存在且 recent stats 已实际计算 |
| keyframe gate | bootstrap ready、observations 非空且既有 keyframe branch 实际执行 |
| pending unique seed count | accumulated branch 实际访问并在本次纳入 current packet 后 |
| effective seed count、minimum、seed origin | stereo population gate 实际执行；empty-observation 既有 branch 以 `0`、current packet 与实际 minimum 表达其等价 population input，不新增 threshold 决策 |
| geometry accepted count 与 minimum `1` | 真实 root attempt 已执行，包括 geometry rejection 后 rollback |
| `attempt_id` | 真实 `seedRoot()` 调用已发生 |

`current_positive_disparity_count` 是 measurement summary，不是 stereo gate 结果；因此
它可以存在而 `stereo_population_gate=not_evaluated`。两个 boolean 在存在时只写
`0/1`，空 field 与配置值 `false` 不得互换。

### 6.3 Bootstrap path 与现有 predicate

`bootstrap_path` token：

| token | 含义 |
|---|---|
| `not_evaluated` | 未进入现有 bootstrap 统计 |
| `collecting` | static 未 ready，moving 也未使本调用 ready |
| `static` | 现有 `static_ready` 为真；它优先于 moving fallback |
| `moving` | static 未 ready，现有 shortest-recent-suffix moving fallback ready |

bootstrap diagnostics 只记录当前代码已有 predicate 的输入与 threshold：

| value | threshold / 配对项 |
|---|---|
| bootstrap sample count | `m_bootstrap_min_samples` |
| bootstrap duration ns | `m_bootstrap_min_duration_ns` |
| max accelerometer std | `m_bootstrap_max_acc_std_mps2` |
| max gyroscope std | `m_bootstrap_max_gyr_std_radps` |
| `abs(mean_acc_norm - gravity)` | `m_bootstrap_acc_norm_tol_mps2` |
| bootstrap duration ns | `m_bootstrap_timeout_ns` |

static 未 ready、控制流读取 moving option 时，记录
`m_enable_moving_bootstrap` 的实际值；若 option 已开启且
`recentBootstrapBegin` 返回值存在、recent stats 确实计算，再记录 suffix sample
count / duration、mean-acc norm 与当前实现使用的非零 epsilon threshold。

这些字段描述当前 bootstrap readiness，不得命名或解释为 IMU excitation、
gyro-bias observability、gravity/velocity conditioning 或 dynamic initialization
质量。

### 6.4 Stereo seed population

`seed_input_origin` token：

| token | 含义 |
|---|---|
| `not_evaluated` | stereo population gate 未执行 |
| `current_packet` | 实际比较或送入 root attempt 的 population 来自当前 packet |
| `accumulated` | 实际比较或送入 root attempt 的 population 来自现有 pending unique buffer |

必须分别记录：

1. `current_positive_disparity_count`：输入校验通过后的当前 packet
   `disparity_px > 0` 数量；这是 measurement summary，即使 bootstrap 抢先
   阻塞也可以存在，但 `stereo_population_gate` 仍保持 `not_evaluated`；
2. `pending_unique_seed_count`：只有现有 accumulated path 在本次实际访问并
   更新 pending buffer 时才存在，语义是纳入当前 packet 后的 unique count；
3. `effective_seed_count`：现有 stereo population gate 消费的 population；正常
   branch 是实际与 `min_seed_observations` 比较的值，empty-observation 既有 branch
   记录等价输入 `0`；
4. `min_seed_observations`：本次使用的实际 threshold；
5. `enable_accumulated_seed` 的实际值；
6. 既有 `is_keyframe` 继续作为 packet 输入事实，keyframe 是否真正作为 gate
   执行由 `keyframe_gate` 表达。

现有 `num_disparity` 的历史列和值不改。新字段不能通过提前改写旧列来制造旧
artifact 差异。

当 `attempt_id` 存在时，`effective_seed_count` 必须等于实际传给 `seedRoot()`
的 measurement 中 positive-disparity population；否则 attempt identity 与输入
证据不成立。

### 6.5 Root geometry

`root_geometry_result` token：

| token | 含义 |
|---|---|
| `not_evaluated` | 没有真实 root attempt |
| `accepted` | 现有 root geometry gate 通过 |
| `nonfinite_backprojection` | 现有 backprojection 产生非有限点 |
| `behind_camera` | 现有 front-facing 检查失败 |
| `empty_after_filter` | 跳过不合格 observation 后没有可安装 landmark |

真实 attempt 还必须记录 attempt-local accepted landmark count 与现有最小要求
`1`。计数描述真实执行，即使随后 transaction rollback 也不能被清零为
“未尝试”。

不公开 landmark IDs、三维点、depth/disparity 分布、parallax、spatial rank、
condition number 或相机/GTSAM 对象。本片先识别最早持续阻塞 gate；root-quality
Observe 需要新的证据问题。

geometry secondary result 只投影现有 `seedRoot()` 正常返回路径中的真实 branch。
现有未捕获 exception 继续原样传播；Observe 不得新增 catch、不得把 exception
转换为 `kRejected/kFailed`，也不承诺为没有产生 `VioUpdateResult` 的调用写 CSV。
implementation 可以用 private attempt-local outcome 保留正常返回分支，但不得改变
`seedRoot()` 的调用次数、返回/抛出行为或 transaction mutation。

### 6.6 Attempt identity

`attempt_id` 合同：

1. public 类型为 optional unsigned integer；CSV 缺失表示没有 attempt；
2. 每个 estimator instance 从 `1` 开始严格单调递增；
3. 仅在即将真实调用现有 `seedRoot()` 时分配；
4. 不为 collecting、非 keyframe、population insufficient 或 discontinuity
   frame 分配；
5. 它是诊断观察历史，不属于 `VioUpdateTransaction`；rejection、graph failure
   或 rollback 仍消耗并保留该 ID；
6. discontinuity 或 segment completion 不重置；只有构造新的 estimator instance
   才重置；
7. geometry、current graph 和 commit phase 必须有 ID；其它 phase 必须无 ID；
8. ID 存在不表示成功，只有 `commit/committed` 表示 root 已原子提交。

counter 的 increment 与 result assignment 必须位于现有 `seedRoot()` call-site 的
紧邻前方，两者与真实调用之间不得出现 branch、可提前 return 的语句或第二个
policy owner。

## 7. `diag.csv` schema

不新增 CLI、sidecar、JSON blob 或第二套 session state。现有 26 列保持原顺序，
在尾部连续追加以下 38 列，形成固定 64 列 schema：

### 7.1 Identity、phase 与 gate

```text
cold_root_phase
cold_root_reason
cold_root_bootstrap_path
cold_root_seed_input_origin
cold_root_geometry_result
cold_root_attempt_id
cold_root_bootstrap_gate
cold_root_keyframe_gate
cold_root_stereo_population_gate
cold_root_geometry_gate
cold_root_imu_excitation_gate
cold_root_conditioning_gate
cold_root_initialization_solve_gate
cold_root_current_graph_gate
cold_root_commit_gate
```

### 7.2 Bootstrap predicate

```text
cold_root_bootstrap_sample_count
cold_root_bootstrap_min_samples
cold_root_bootstrap_duration_ns
cold_root_bootstrap_min_duration_ns
cold_root_bootstrap_acc_std_max_mps2
cold_root_bootstrap_acc_std_limit_mps2
cold_root_bootstrap_gyr_std_max_radps
cold_root_bootstrap_gyr_std_limit_radps
cold_root_bootstrap_acc_norm_error_mps2
cold_root_bootstrap_acc_norm_tolerance_mps2
cold_root_bootstrap_timeout_ns
cold_root_moving_bootstrap_enabled
cold_root_moving_suffix_sample_count
cold_root_moving_suffix_duration_ns
cold_root_moving_acc_mean_norm_mps2
cold_root_moving_acc_mean_norm_min_mps2
```

moving suffix 使用与 bootstrap 相同的 sample/duration minimum，因此不复制第二套
threshold 列。

### 7.3 Stereo population 与 geometry

```text
cold_root_current_positive_disparity_count
cold_root_accumulated_seed_enabled
cold_root_pending_unique_seed_count
cold_root_effective_seed_count
cold_root_min_seed_observations
cold_root_geometry_accepted_landmarks
cold_root_geometry_min_landmarks
```

enum/gate 写稳定 ASCII token；optional 数值与 `attempt_id` 的 `nullopt` 写空 CSV
field。boolean 写 `0/1`，只有 optional 存在时才写值。不得写自由文本 exception
或 `result.message` 到上述稳定列。

session 只扩展当前已经生成的 `VoDiagRow`；不为既有 session hard-stop path 新造
row，也不改变 `FrameCounts`、warning、summary、config hash 或运行终态。

typed taxonomy 覆盖每个正常返回的 `VioUpdateResult`；`diag.csv` 只持久化 session
按既有 cadence 已经生成 row 的 result。当前 `kFailed/kInvalidInput` 会先终止
session，因此 bootstrap timeout、current-graph failure 和 input-invalid 只要求在
estimator result-level 定向测试中验证，不承诺新增 CSV row。不得为扩大 artifact
覆盖而改变 hard-stop、warning 或继续运行语义。V2_03 terminal epoch 均为
`kInitializing`，不受这项 artifact 边界影响。

## 8. Phase/gate 一致性

每个 `VioUpdateResult` 必须满足：

1. 恰有一对 primary phase/reason；
2. active frame、hard discontinuity 和 input-invalid 为
   `not_evaluated/not_evaluated`，所有 cold-root-only optional 均为空；
3. normal cold-root frame 的最终 phase 不得为 `not_evaluated`；
4. 已真实执行的前序 gate 为 `passed`，首次阻塞 gate 为 `failed`，未访问的后续
   gate 为 `not_evaluated`；
5. `commit/committed` 必须同时满足 `status=kOk`、estimate 存在、entry state
   为 uninitialized、attempt ID 存在、geometry/current graph/commit gate passed；
6. `root_geometry/geometry_rejected` 必须满足 `status=kRejected`、attempt ID
   存在、geometry gate failed；
7. `current_graph/*` 必须满足 `status=kFailed`、attempt ID 存在、geometry gate
   passed、current graph gate failed、commit gate not evaluated；
8. bootstrap timeout 使用 `bootstrap/timed_out`；其它抢先发生的既有 hard
   invariant failure 不伪装为 cold-root rejection；
9. primary reason、gate state、value/threshold、bootstrap path、seed origin 与
   geometry result 的组合必须由 typed invariant 检查覆盖；无效组合不能静默落盘。

entry-state 与 field presence 的优先级为：

| update entry / validation | Observe identity | measurement summary 与 option snapshot |
|---|---|---|
| initialized active segment | `not_evaluated/not_evaluated` | 全空，即使 packet 有正视差 |
| uninitialized，但 discontinuity 或任一 cold-root pre-gate validation 未通过 | `not_evaluated/not_evaluated` | 全空 |
| uninitialized，且全部 cold-root pre-gate validation 通过 | 按真实 cold-root phase/reason | current positive-disparity 与 accumulated option 必须存在；其它字段按 §6.2 presence 表 |

## 9. 行为与 transaction 不变项

Observe 不得参与任何既有决策。对相同输入：

- `UpdateStatus`、`message`、estimate presence 与所有 estimate 数值不变；
- bootstrap accumulation、pending seed、keyframe eligibility、`seedRoot()` 调用
  次序和次数不变；
- window、active map、track history、cull/rebirth、PnP、factor、support 与
  visual coast 不变；
- graph、initial values、LM、outlier cull/reopt 与 validation 不变；
- transaction commit/rollback、segment ID 与 completed segment 不变；
- `est.tum`、`kf.tum` 和现有 diagnostics/counters 不变；
- config、config hash 与默认运行参数不变。

允许新增的持久变化只有 `UpdateDiagnostics` typed value、`VoDiagRow` 镜像与
`diag.csv` 尾部新列。`attempt_id` counter 是非事务诊断历史，不能被 graph 或
lifecycle 读取。

## 10. 机制门

长期测试只经 public `VioEstimator::update()` 与既有 offline session interface
观察，不新增 production test seam。

### 10.1 Estimator taxonomy

定向测试至少覆盖：

1. active `kOk`、discontinuity 与 input-invalid 的 `not_evaluated`；
2. bootstrap evidence insufficient 与 timeout；
3. static path ready 与 moving path ready 的真实 predicate/threshold 配对；
4. bootstrap ready、empty observations：population `0`、keyframe gate
   not evaluated；
5. bootstrap ready、non-empty、non-keyframe：`keyframe_required`；
6. current population `9` / threshold `10`：`population_insufficient`；
7. accumulated seed 的 pending unique、effective count 与 input origin；
8. 真实 geometry rejection：attempt ID 存在、attempt-local result 保留、
   transaction state rollback；
9. root graph build/solve/validation 的 typed enum 映射与 result-level invariant；只对
   当前可由合法 public input 确定触达的 branch 写 public test，无法自然触达的
   branch 明确标记 `not_covered`，不得为覆盖 enum 新增 production injection seam、
   改变 solver/error path 或声称已经验证；
10. root commit：所有前序 gate passed、`commit/committed`、estimate 存在；
11. 同一 estimator instance 的多帧序列依次覆盖 bootstrap collecting、non-keyframe、
    current population 不足、accumulated population 不足、真实 geometry rejection
    与真实成功 root；前四类均无 ID，后两类依次取得连续且不重用的 ID；
12. 每条测试同时对拍既有 status/message/estimate/legacy diagnostics 与后续合法
    update 行为。

geometry secondary token 只要求覆盖可由合法 public input 构造的实际 branch；
不得为覆盖 enum 而放宽 input validation 或增加 production injection seam。

### 10.2 Session 与 artifact

测试必须证明：

- CSV 新列顺序与 token 稳定；
- optional 空 field 与真实数值零可区分；
- session 是逐字段 copy，不按 message 二次分类；
- 旧 26 列逐行 projection 不变；
- row count、timestamp、status 和 segment cadence 不变；
- 不新增 FrameCounts、warning、summary decision 或 config key。

## 11. V2_03 natural replay gate

### 11.1 实验问题、输入与成本

实验问题：

> terminal 289 个 `initializing` frames 的最早实际阻塞 phase/reason 分布是什么；
> 当前实现是否发生任何真实 root attempt？

在本机 Release 下，只运行一次 `V2_03_difficult`，使用 checkpoint 相同的
canonical config 与 `--estimator-enable-moving-bootstrap`。成本预算按 frozen raw
summary 的 candidate `wall_s≈37.993` 估算为约 `38 s`；本工作树未重新计时。
不运行 MH_01、TUM VI 或 EuRoC-11。

control 复用 checkpoint 记录的 frozen db22656 raw artifact，不重新跑 control。
比较忽略 git identity、wall/timing 和新增列，只比较行为 projection。

### 11.2 行为等价 PASS

candidate 必须相对 frozen control 满足：

1. `diag.csv` row count、timestamp、status、segment ID 与旧 26 列 projection
   逐行相同；
2. `est.tum` 与 `kf.tum` byte-identical；
3. status counts 保持 `1141 ok / 771 initializing / 9 visual_outage`；
4. segments 保持 `9`，last ok 保持 `96.5 s`；
5. completion、coverage、ATE/RPE 与段内/段间分量保持 control 值；
6. config canonical text / hash 不变；
7. 新 diagnostics 不改变 runtime status、warning 或 termination policy。

### 11.3 Observe mechanism PASS

对 `96.6–116.7 s` 的 289 个 terminal rows：

1. 每行均为 cold-root frame，phase/reason 不得为 `not_evaluated`；
2. phase/reason bucket 总和精确为 `289`；
3. 每行 gate trace 满足 §8，未执行 gate 明确为 `not_evaluated`；
4. current positive-disparity count 均为真实 measurement summary，不再因
   bootstrap early return 与旧 `num_disparity=0` 混淆；
5. effective population、threshold、seed origin 与 population reason 可独立复算；
6. attempt IDs 必须唯一且严格递增，并与 geometry/current graph/commit rows
   一一对应；没有真实 `seedRoot()` 调用则所有 attempt ID 为空；
7. 非空 attempt ID 的 phase 集合必须是
   `{root_geometry,current_graph,commit}` 的子集，并逐行通过可机读 invariant；
8. 输出 phase/reason 直方图、bootstrap path 分布、gate pass/fail/not-evaluated
   分布，以及 attempt count；
9. 明确指出最早持续阻塞 gate；若证据分散，则结论为 mixed，不以最大 bucket
   替代完整分布。

Observe PASS 不要求 coverage、completion、segments 或 accuracy 改善。

### 11.4 STOP

出现任一条件立即停止，不扩实验：

- 行为 projection 与 frozen control 不一致；
- 289 行不能完整对账；
- primary reason 与 gate trace/value/threshold 自相矛盾；
- attempt ID 在未调用 `seedRoot()` 时出现，或真实 attempt 没有 ID；
- current path 未计算的 excitation/conditioning/initialization solve 被报告为
  passed/failed；
- 需要修改 seed threshold、打开 accumulated seed、增加 shadow calculation
  才能解释结果。

失败结果同样是完整 Q1 证据；记录最早不一致处后回到 spec，不在同一轮修算法。

## 12. 本片不授权

本片不授权：

- 修改 `min_seed_observations`、bootstrap threshold 或 keyframe 判定；
- 默认开启 accumulated seed；
- 计算或接入 dynamic gyro-bias、gravity/velocity alignment、MAP refinement；
- 引入 condition/rank/parallax/root-quality acceptance；
- 改变 active-map observation/factor/PnP/support 语义；
- 继承 bounded NavState handoff；
- 建立跨-root world-frame alignment / relocalization 因果边；
- 改写 `#47` formal gate；
- 进入 full EuRoC-11、TUM VI 或下一资格阶段。

本片完成后，下一方向只由 Observe artifact 决定：如果绑定门是 population，后续
单独讨论 multi-frame root evidence；如果真实 attempts 已发生，则按最早 geometry
或 current-graph failure 另立合同。root availability 与 cross-root alignment 继续
分开验收。

# M5 post-outage world-frame recovery

- 日期：2026-09-07（Asia/Shanghai）
- 修订：2026-10-14（用户确认采用有界恢复）
- 状态：**已定稿**（段内局部求解，复用既有 coast horizon 的有界世界系恢复）
- Issue：[#58](https://github.com/Nothand0212/phad-vio/issues/58)，Part of [#42](https://github.com/Nothand0212/phad-vio/issues/42)
- Plan：[58c9f001](../plans/2026-10-14_m5_world_frame_recovery_58c9f001.plan.md)；历史实施记录：[58a7c201](../plans/2026-09-07_m5_post_outage_world_frame_recovery_58a7c201.plan.md)
- 复用合同：[initial moving root](2026-09-01-m5-initial-moving-root.md)、[M4 active continuity](2026-08-26-m4-world-frame-continuity.md)

## 1. 产品目标与独立边界

本片只验证 EuRoC `V2_03_difficult` 在 visual outage 后，以 estimator-private
IMU-propagated anchor 将独立求解的新 moving root 表达到旧世界系，能否改善跨段
连续性与整体 ATE。历史 raw control 为 ATE `1.960225035 m`、RPE
`0.416012655 m`、completion `0.490369599`、coverage `0.724935733`，有 7 次
outage、6 次恢复 root；这些数值是对照线索，正式裁决使用本片 fresh C1。

`VioEstimator::update()` 保持唯一 public ingestion seam。500 ms coast、PnP、
solver、所有已有门限、static precedence、attempt/status/segment/root cadence
保持不变。旧 map、window、factor、posterior 不跨段；anchor 只承担 world expression。
局部 solve、current graph validation 与 bias authority 复用 #51 合同。

#58 独立于 #51 的 formal exact RED；本片通过不产生 #51 checkpoint，也不授权
moving initialization default-on。原 ADR-0003 的 initial-root ownership 与
ADR-0004 的验证职责继续成立；本片具体化其交给后续独立工作的 recovery 边界。

## 2. O(1) anchor 与时间合同

Anchor 是 `VioUpdateState` 内的 optional private value，保存传播用 `X/V/B`、
当前 timestamp、源 committed frame/segment/timestamp 与 world lineage。它不拥有
第二 window、历史 raw interval 列表或 selected earliest-frame anchor cache。

1. 超过既有 coast horizon 的 packet 在现有 `kVisualOutage` transaction 内，先从
   旧段 latest committed navigation state，使用**当前 normalized IMU interval**
   和旧 bias 传播到 outage timestamp，再完成旧段并保留 anchor。当前 outage
   分支早于正常 propagation 返回，因此这一 interval 必须显式覆盖。
2. 后续 initializing packet 在同一 packet transaction 内将 anchor 从此前 endpoint
   推进至当前 latest timestamp。每个 evidence-committed packet 恰好推进一次；
   pending 与 recoverable rejection 均提交该推进，invalid input 与 fatal rollback
   不推进。原有 interval endpoint/value 合同继续适用。
3. Existing moving initializer 独立生成 candidate，完成
   `materializeMovingCandidate()` 与 `runCurrentGraphLifecycle()`。只有 validated
   latest frame timestamp 与 propagated anchor timestamp 精确相等，才计算 bridge。
4. 以 lineage 的 immutable `m_source_timestamp` 为起点，比较 validated latest
   timestamp 与既有 `m_visual_coast_horizon_ns`。恰好等于 horizon 时可以 bridge；
   超过时保留新段局部 root，报告 `NotApplicable / AnchorExpired` 并消费 anchor。
   Pending/retry 更新传播 endpoint，但不重置这个时间预算。
5. 窗口内在 isolated owner 检查 transform，安装 private optional `T_W_local`，
   随原 owner swap 原子提交并消费 anchor；本 packet 返回世界系 latest estimate。
   时间比较须避免 int64 overflow，不引入第二个配置项。

旧 bias 始终只供 anchor propagation。新局部 candidate 的 evidence、seed、solve、
prior、threshold 与 mask 不读取 anchor。Bridge 后不追加优化或改变 candidate attempt。

## 3. Gravity-preserving world expression

局部 current graph 已遵守 canonical Z-up gravity。令同一 latest timestamp 的
anchor pose 为 `(R_A,p_A)`，validated local latest pose 为 `(R_L,p_L)`，定义：

```text
D = R_A R_L^T
a = D00 + D11
b = D10 - D01
rho = hypot(a, b)
tau = 64 * numeric_limits<double>::epsilon()
```

`rho <= tau` 为 yaw 数值不可定义，返回 typed `YawUndefined` fatal 并 rollback；
`rho > tau` 时唯一采用 `psi = atan2(b,a)`、`Q = Rz(psi)`。等号归退化分支。
这是 dimensionless double 舍入下的**可定义性门**，不是精度、激励、置信度或
产品调优门限；不引入配置项，不按数据集 sweep。测试必须覆盖等号、两侧最近
可表示值、零及正常非退化旋转。Non-finite 输入先判 `InvalidAnchor` 或
`InvalidTransform`，不能经比较或 fallback 伪装成可定义 yaw。

```text
t       = p_A - Q p_L
T_public = [Q,t] T_local
v_public = Q v_local
B_public = B_local
```

Validated state 的 navigation、landmark、retained visual seed/verified pose 与
`m_T_W_B_last_stereo` 留在段内坐标，PnP 与 graph 继续消费这些局部值。
Root commit 和后续 active update 仅在公开 navigation 上应用 `T_W_local`；
下一次 outage 从实际公开的世界系 navigation 捕获 anchor。
Body-frame bias、raw IMU、PIM covariance、camera extrinsics、observations、IDs、
timestamps 不变。保留 **current graph 优化后的每帧 bias**，包括 graph 求出的
accelerometer bias；不恢复 initializer 的 `b_a=0` seed，也不给 velocity 加 offset。

`Q g_W = g_W`，latest position 与 anchor position 对齐；roll/pitch 保留局部解。
一般 full SE(3) 相对旋转不保持固定重力，因此不属于本片 gauge。整窗、缓存与
新 root priors 保持同一局部表达，完成段与 discontinuity 清除 `T_W_local`。
旧 anchor 的位置/yaw 漂移仍可能进入新段，产品改善必须由 §7 实测确认。

## 4. Lifecycle 与原子性

| 事件 | Anchor / evidence | Bridge observable |
|---|---|---|
| process start、普通 active update | 无 recovery anchor | NotApplicable |
| visual outage commit | 当前 interval 传播后保存；旧段完成 | Pending / AwaitingValidatedRoot |
| pending / recoverable rejection | anchor 与 evidence 同步提交 | Pending / AwaitingValidatedRoot |
| control moving root commit | 合同检查通过，保留 local world 表达，原子消费 anchor | Eligible / None |
| 窗口内 moving root commit | 安装世界输出 transform 与消费 anchor 原子完成 | Applied / None |
| 超时 moving root commit | 提交局部 root、新 world origin 并消费 anchor | NotApplicable / AnchorExpired |
| static root commit | static precedence 与局部 root 不变，原子消费 anchor | NotApplicable / StaticRootSelected |
| 合法 measurement discontinuity | 清 anchor、world lineage 与 provisional evidence | Cleared / MeasurementDiscontinuity（存在待清 bridge 时） |
| invalid input | 不 staging、不推进、不消费 | 不报告 Applied |
| post-staging fatal | anchor、evidence、revision、span 等恢复 pre-update state | FatalError / typed reason，RolledBack |

Pending/recoverable 使用 `EvidenceCommitted`；control Eligible、subject Applied 与
static bypass 与 expired root 使用 `RootCommitted`；合法 discontinuity 使用 `EvidenceCommitted`。
Discontinuity 行 origin 为 `MeasurementDiscontinuity`；若没有待清 bridge，则为
`NotApplicable / MeasurementDiscontinuity`。清除或消费时，该行保留被清 anchor 的
lineage 供审计，后续普通 active row 恢复 `None / NotApplicable / None`，optional
bridge 字段 absent。Initializing process-start origin 为 `ProcessStart`；合法
discontinuity 后重新收集期间 origin 保持 `MeasurementDiscontinuity`，不继承旧
lineage。Invalid input 的 transaction 为 `NotEvaluated`，不伪造 fatal rollback。

Discontinuity 必须覆盖 active 与 uninitialized 两条路径；pending 状态下须清除
window、landmarks、track/cache、bootstrap 与 moving evidence，不能保留跨 discontinuity
的 predecessor/interval。保持现有外部 status/segment 语义。两项必要 correctness
修复——pending discontinuity 遗留 window、fatal unsupported-span staged leakage——
先用 RED 复现，修复后进入 C/S 两臂共同基线，不计作 bridge 的单变量效果。

Control 的 Eligible moving root 与 static bypass 成功后均建立自身 world lineage：
持久 `world_origin_segment_id` 为当前 target segment。Subject 的 Applied root 才
继承 source world origin；该更新与 transform、owner swap和anchor消费属于同一次
原子安装。Control metadata不得声明未实际发生的跨段world continuity。

`RootCommitted` 审计行的 `m_lineage.m_world_origin_segment_id` 表达实际提交的
**目标 world origin**；`m_source_segment_id/m_source_frame_index/m_source_timestamp`
继续保留 immediate source身份。Pending、Cleared与FatalError行保留被处理anchor
的source world lineage用于审计；rollback恢复持久world origin。再次outage从刚
提交的最新导航状态及其真实world origin重新捕获anchor。EOF仍pending时不伪造root
或Applied。两臂发生首次bridge结果分叉后，后续lineage差异按此规则解释。

## 5. Typed observables 与实验 sidecar

使用独立、简单稳定的 `RecoveryBridgeDiagnostics` aggregate；以下为本片 C++
命名合同，避免与既有 `InitializationPath/Reason/TransactionResult` 混用：

| 成员 | 类型 / 语义 |
|---|---|
| `m_origin` | `RecoveryBridgeOrigin`: None, ProcessStart, VisualOutageRecovery, MeasurementDiscontinuity |
| `m_result` | `RecoveryBridgeResult`: NotApplicable, Pending, Eligible, Applied, Cleared, FatalError |
| `m_reason` | `RecoveryBridgeReason`: None, AwaitingValidatedRoot, StaticRootSelected, MeasurementDiscontinuity, TimestampMismatch, InvalidAnchor, YawUndefined, InvalidTransform, ContractError, AnchorExpired |
| `m_lineage` | optional `RecoveryBridgeLineage`，源 segment/frame/timestamp、目标 segment、world 起始 segment |
| `m_anchor_timestamp` / `m_root_timestamp` | optional integer-ns timestamp，分别表示实际传播 endpoint / validated product latest root |
| `m_attempt_id` | optional，复用同帧 initialization attempt identity，不分配第二 attempt counter |
| `m_transaction` | 复用 `InitializationTransactionResult` 的 NotEvaluated / EvidenceCommitted / RootCommitted / RolledBack |

Enum values 使用项目既有 `k` 前缀。Lineage 的成员固定为
`m_source_segment_id/m_source_frame_index/m_source_timestamp/m_target_segment_id/`
`m_world_origin_segment_id`；不以 message、NaN 或虚构 ID 表示 absence。
`UpdateDiagnostics::m_recovery_bridge` 与 `VoDiagRow::m_recovery_bridge` 按值保留该 snapshot。
合法组合由 exhaustive tests 冻结；`Applied` 只与同帧 moving `kOk`、estimate、
`RootCommitted` 共存，且 anchor/root timestamps 相等。

Failure audit 可位于 rollback snapshot 外；anchor 与 durable lineage 位于同一个
transaction owner 内。失败行需明确 RolledBack，不能将 prepared transform 当作
Applied。Control 保留同样 lifecycle/sidecar；eligible moving root 在 control 中为
Eligible / None / RootCommitted；subject 在窗口内对应 Applied / None / RootCommitted，
超过窗口对应 NotApplicable / AnchorExpired / RootCommitted，并建立 target world origin。
`Eligible` 是 cpp 直接发布的明确 typed result，apps/benchmark 不通过字段组合推断
或补造它。该 result 的合法性要求 origin 为 VisualOutageRecovery、initialization
moving committed、有效 lineage、同 timestamp anchor/root；这些字段用于核验
producer 合同。Static bypass 两臂均 NotApplicable。

实验 sidecar 每 processed packet 一行，含 `i/ts_ns/status/segment_id`、上述 typed
bridge snapshot 和同帧 initialization path/outcome/attempt/transaction。两臂写相同
schema；writer 只序列化 typed 值，不影响 algorithm。CLI-only
`--recovery-bridge-log <path>` 启用该 sidecar；该路径不进入 canonical config/hash。
固定 26 列 `diag.csv` 及常规
trajectory schema 不变，不新增 public measurement、配置 activation 或研究矩阵接口。

JSONL `schema_version=1` 的顶层固定为
`schema_version/i/ts_ns/status/segment_id/frontend/initialization/bridge`。
`initialization` 字段为 `path/phase/reason/outcome/attempt_id/current_graph/transaction`；
`bridge` 为 `origin/result/reason/lineage/anchor_ts_ns/root_ts_ns/attempt_id/transaction`。
Bridge origin/result/reason 与 initialization path/outcome/transaction 序列化为
**去除 k 前缀、保留大小写**的稳定 enum 名，如
`VisualOutageRecovery/Eligible/RootCommitted`；bridge transaction同样处理。
Initialization phase/reason/current_graph保留既有 enum integer。

`lineage` absent 为 null；存在时包含
`source_segment_id/source_frame_index/source_ts_ns/target_segment_id/world_origin_segment_id`。
`i` 是从 0 开始的 processed index，`ts_ns/source_ts_ns/anchor_ts_ns/root_ts_ns`
为 integer nanoseconds，optional absence写null；frame/segment identity与attempt同样
用integer。Writer位于 `apps/recovery_bridge_log.hpp`，只承担typed投影和JSONL写出。

`frontend` 包含 `is_keyframe` 与 `observations`；后者是传给当前 estimator packet
的完整有序 array，每项固定为 `[id,u,v,disparity]`。使用
`VoDiagRow::m_recovery_input` 的 optional `RecoveryBridgeInput` 按值保存调用前
processed index、timestamp、keyframe bit与observations，仅CLI collect启用时填充。
在 estimator update 前按原顺序取值，不排序、不删项、不记录
仅剩 PnP mask 的子集。`is_keyframe` 是本次 update 的原始输入 bit。Double 值采用
可 round-trip 的序列化精度，保留实际 typed 数值，不以显示精度舍入。Observation
中的 non-finite double使用 `{"f64_bits":"<固定16位小写hex>"}` tagged value，
将原始 `uint64_t` bit pattern前补零编码，保留 NaN payload/sign与infinity身份；
不能写成null、删除observation或由writer改变estimator的terminal结果。Finite值
仍写JSON number。逐行精确比较
C1/S1 的 `i/ts_ns/frontend` 投影（包含 `frontend.is_keyframe`）；bridge 等预期变化字段单独验收。
该观测只在 sidecar 启用时采集，不建立 replay 框架；sidecar on/off 的算法 payload
必须一致。Benchmark owner 负责此导出，cpp owner 负责上述 typed producer。

## 6. 两臂身份、执行顺序与 applicability

使用两个隔离 source snapshots/builds。共同 correctness 修复与 observability 完全
相同；唯一 source delta 是 eligible moving recovery 的 bridge 生效。Canonical config
及其 hash 相同，activation 以 source identity 区分；不通过 CLI/config/test fault-arm
切换产品臂。Control 同样维护并消费 anchor、执行定义性检查，成功时保留局部 world
表达；subject 才安装转换，避免将传播或 diagnostics 的差异混入效果。

冻结各自 source/build identity、共同 toolchain/libraries、input、frontend 有序
输出、完整 canonical config/meta 参数快照与 evaluator。source identity 必须覆盖
实际 dirty 内容，不能仅记录 HEAD。两臂仅同 config hash 不足以证明输入一致。

先完成本地机制测试与 launcher mock tests，按 **fresh C1 → 核验 → S1**
运行完整 V2_03。2026-10-14 的本地实现与验证已获用户授权；远程执行仍沿用
独立授权。C1
sidecar 必须复现预注册 outage/root 边界，以明确 `Eligible` result 确定每个恢复
root 的 applicability，并交叉核验同帧 initialization path；eligible moving
recovery 数为 0 时立即停止，不运行 S1，不扩大
序列或更改 static precedence。C1 的原有 `diag.csv/est.tum/kf.tum` 算法 payload
必须复现冻结的 bridge 前 control；共同修复或观测若改变原输出，即 qualification
STOP，保存首个差异，不将变化后的 C1 静默接纳为新 control。C1 不合格同样停止。
S1 按 C1 已冻结的 source/root timestamp 与 canonical horizon，在窗口内事件
Applied，超时事件 AnchorExpired；其余事件遵循 §4。应用与超时索引在 S1 前
由 C1 计算，全六组事件保留；不得事后按 subject 结果筛选。

## 7. 固定 paired 指标、产品门与停止条件

全体 6 组旧段末 ok → 恢复 root 的 zero-based processed index pairs 固定为：

```text
J = {(433,459), (701,809), (929,937),
     (1010,1097), (1157,1343), (1521,1563)}
```

所有 6 组均进入跨段统计，包括 static bypass；不按是否 Applied、跳变大小或
恢复耗时删事件。对同一 pair 端点 timestamp 使用同一 GT association，计算：

```text
Delta_est = inverse(T_est(i)) * T_est(j)
Delta_gt  = inverse(T_gt(i))  * T_gt(j)
E_pair    = inverse(Delta_gt) * Delta_est
e_t       = norm(translation(E_pair))             # m
e_R       = angle(rotation(E_pair))              # rad
```

逐事件报告 C1/S1/absolute delta、origin/path/applicability；全 J 分别报告 translation
与 rotation 的 RMS/median/max。跨段 translation RMS 是下降门；rotation 完整记录。
该误差扣除了相同端点 GT 真实运动，不能用裸 pose distance 代替。端点缺失、身份或
pair 不匹配为协议失败，不删除后继续求平均。

ATE 使用两臂共同 timestamp 支持集，各臂对该完整支持集仅做一次 fixed-scale
SE3 alignment，禁止分 segment 对齐；报告 C1/S1/absolute delta。同时保留各臂
原生全支持集 ATE/RPE、completion/coverage、支持集数量及差集。

V2_03 必须同时满足：

- pre-outage processed rows `0–433` 的 `diag.csv/est.tum/kf.tum` 对应 payload exact
  identity；首次实际 `Applied` **之前的全部 rows** 同样要求两臂全部算法 payload
  与有序 input exact identity，包含 outage → root 的所有 pending/recoverable rows，
  以及若先发生 static bypass 的 rows；不能只检查 `0–433`。全程有序 input
  `i/ts_ns/frontend`（包含 keyframe bit）exact identity，status/segment/root/attempt cadence
  不变。
- finite，`failed = rejected = 0`；全部窗口内 eligible moving recovery 对应 Applied，
  超时事件对应 AnchorExpired；static bypass、discontinuity、transaction 和 lineage 正确。
- 全 J 跨段 translation error RMS 与共同 timestamp ATE 均严格下降；
  completion/coverage 均不低于 C1。

任一身份、hard correctness、cadence 或产品门失败，立即停止当前矩阵，保存首个
typed expected/actual、source/config/input 与 artifact。Bridge 全成功但两个主指标
未同时改善亦停止；不调参、不 sweep，不以其他指标改善抵消该 verdict。

V2_03 全部门通过后，才扩 EuRoC core-4：`MH_01_easy/V1_03_difficult/`
`V2_02_medium/V2_03_difficult`，paired geometric-mean ratios `G_ATE <= 1.05`、
`G_RPE <= 1.05`，保留逐序列绝对米值、availability 与 continuity 向量及既有
[多指标 envelope](2026-08-27-m4-mapped-landmark-bearing-continuity.md)。本片 C1/S1
结果保留其身份，可复用已通过的 V2_03 pair。

TUM-VI room1–6 是后续 guardrail，**不在 V2_03 前提前运行全量**；须全程 bridge
NotApplicable，并保持 `diag.csv/est.tum/kf.tum` payload identity。任一触发或变化
即停止。MH_01 历史 formal exact RED 独立保留，不阻塞本产品实验。

## 8. 实现验收与证据

机制测试覆盖：outage 当前 interval；多次 pending/recoverable 后 latest timestamp；
非零 yaw/translation、不同 tilt、不同旧/新 bias 与非零 velocity；局部整窗/cache 不变；
恰好 horizon、超过 horizon、pending/retry 不重置预算与 expired root 新 world origin；
数值 yaw 边界；invalid/fatal rollback、重试 clean equivalence；static/discontinuity
清除；连续多次 outage lineage；EOF pending；下一 active update 与 offline-session
status/trajectory/sidecar cadence。Fault injection 仅沿用 internal testing seam。

历史证据入口：

- `/tmp/phad-m5-euroc11-full-current/20260905TFSCFI5Z/{RESULT.md,metrics.json}`
- `/tmp/phad-m5-tumvi6-full-current/20260907T041345Z/{RESULT.md,metrics.json}`
- [MH_01 formal 首对证据](../research/2026-09-03-note-m5-mh01-formal-rpe-first-pair.md)

交付报告区分合同冻结、机制测试、C1 eligibility、V2_03 产品门及后续 guardrail 的
实际完成状态。2026-10-14 的本地 core-4 与机制测试见
[恢复报告](../research/2026-10-14-note-m5-world-frame-recovery-local-solve.md)及
[evidence](../benchmark/m5/2026-10-14-bounded-world-frame-recovery.evidence.json)；
TUM-VI room1–6 guardrail 待验收。远程执行与 Git / Issue 写入沿用独立授权边界。

---
name: M5 post-outage world-frame recovery
overview: 以共同 correctness 修复与 typed control 为起点，验证 latest-to-latest yaw 加 translation bridge 对 V2_03 跨段误差和 ATE 的单变量效果；通过后才扩 core 与 TUM guardrail。
todos:
  - id: freeze-contract
    content: 冻结 issue 58 spec、typed producer与sidecar字段、source隔离activation和全6组pair验收
    status: completed
  - id: common-control-red-green
    content: 复现并修复pending discontinuity及fatal span rollback，完成O(1) anchor与Eligible typed control，停在control快照门
    status: completed
  - id: subject-bridge-red-green
    content: 在冻结共同control上只增加提交前整体world transform，验证yaw边界、lifecycle与offline-session合同
    status: completed
  - id: common-rollback-audit-v2
    content: 补canonicalStateJson的anchor XVB/lineage及initialization/world origin，完成public proposal-fatal rollback非零oracle，以独立common patch生成control v2
    status: completed
  - id: qualify-c1-run-s1
    content: C1 qualification已通过、S1执行完成；fixed-input/cadence协议STOP，首差独立复核完成，J/ATE产品裁决未执行
    status: pending
  - id: conditional-guardrails-report
    content: V2_03通过后才扩core与TUM guardrail，保存独立产品证据及各门实际状态
    status: pending
isProject: false
---

# M5 post-outage world-frame recovery implementation plan

历史实施记录。2026-10-14 的有界恢复修订与当前验收状态见
[58c9f001](2026-10-14_m5_world_frame_recovery_58c9f001.plan.md)。

- Plan ID：`58a7c201`
- 日期：2026-09-07（Asia/Shanghai）
- 状态：**fixed-input/cadence 协议 STOP：C1 qualification PASS，S1 执行完成但 paired gate 未通过；首差独立复核完成，J/ATE 产品裁决未执行**
- Issue：[#58](https://github.com/Nothand0212/phad-vio/issues/58)，Part of [#42](https://github.com/Nothand0212/phad-vio/issues/42)
- Spec：[post-outage world-frame recovery](../specs/2026-09-07-m5-post-outage-world-frame-recovery.md)
- Design：[M5 当前执行顺序](../design/m5-initial-moving-root.md#14-当前执行与后续边界)
- Roadmap：[M5](../design/roadmap.md#m5正式初始化)

本计划独立承载 V2_03 post-outage 产品实验。#51 spec/plan 的历史 formal 合同和
RED 保持原状，不能用本片结果生成其 checkpoint 或升级 default。用户已有 dirty
内容按实际 source snapshot 记录；不从 HEAD 或聊天摘要推导完整代码身份。

## 1. 已确认合同与职责

| 项目 | 冻结选择 |
|---|---|
| 对齐 endpoint | O(1) anchor 推进到 validated product latest timestamp |
| Gauge | `D=R_A R_L^T`，`a=D00+D11`，`b=D10-D01`，`Q=Rz(atan2(b,a))`，`t=p_A-Q p_L` |
| 数值可定义性 | `hypot(a,b) <= 64*double epsilon` fatal；等号退化，无配置、无调优 |
| State | 整体转换 poses/landmarks/world caches，velocity 只旋转，bias 保留 local graph optimized 值 |
| Control / subject | 相同 canonical config、共同修复/观测/anchor；control Eligible，subject 提交前 transform 后 Applied |
| Static / discontinuity | static success 原子消费且 NotApplicable；合法 discontinuity 清 anchor、lineage 与 provisional evidence |
| Producer / consumer | cpp 产出 `RecoveryBridgeDiagnostics`；apps 按值投影及写 sidecar，scripts核验，不推断 Eligible |

Cpp owner 负责 estimator、typed enums/state 与 estimator tests；apps/scripts owner
负责 `VoDiagRow::m_recovery_bridge`、CLI-only `--recovery-bridge-log`、JSONL、input
identity 与 paired evaluator。双方以 spec §5 字段名交接，避免各自创建第二套
RecoveryDiagnostics/结果枚举。公共摄入 seam 与固定 diag26 不变。

## 2. 第一步：共同 control 的 RED → GREEN

共同 estimator control 已冻结于
`/tmp/phad-m5-estimator-control-frozen-20260907T072728Z`；该目录 `tests.log` 记录
23 suites、218/218 PASS。此前 pending discontinuity 与 fatal unsupported-span
两项 RED→GREEN、3 条 graph/history 定向测试均已通过。独立 review 结论为无
blocking production，当时新增 P2 验证补项：初版 `canonicalStateJson` 缺少 anchor `X/V/B`、
lineage、`initialization_origin` 与 `world_origin`，该版序列化断言不能证明这些
字段的完整 rollback。该 common seam 补充及 public proposal-fatal rollback 非零
oracle 已完成，作为独立 common patch 生成 control v2，subject activation继续隔离。
旧 control 冻结目录保留；v2冻结路径为
`/tmp/phad-m5-estimator-control-v2-frozen-20260907T0741`，隔离测试 **219/219 PASS**。
这两个目录是 estimator 冻结证据；main 已完成完整两臂 source freeze，原目录
`/tmp/phad-m5-recovery-pair-final-20260907` 保留。Launcher pre-publication 修复后
重冻的实际执行目录为 `/tmp/phad-m5-recovery-pair-launcherfix-20260907`，包含完整
source manifests、V2_03 逐文件输入 manifest 与 plan。C1 qualification 已通过；
S1 的 paired protocol STOP 见 §4。

在 `tests/estimator/vio_full_state_test.cpp` 按一次一个可观察失败推进：

1. **Pending discontinuity RED**：已有 moving provisional window 时输入合法
   discontinuity，随后新 packet 不得引用旧 predecessor、landmark 或 evidence。
   复现后修复 `VioEstimator::update()` 的 uninitialized discontinuity 分支，
   保留既有外部 status/segment cadence。
2. **Fatal unsupported-span RED**：在 post-staging span 失败上证明 state 与返回
   diagnostics 均恢复 pre-update；复现后统一走显式 rollback/finalization。
3. 引入 spec §5 typed aggregate 与 `VioUpdateState` private anchor/lineage；在
   outage transaction 覆盖当前 normalized interval，pending/recoverable 随
   evidence 推进，fatal/invalid 不推进。验证 static/discontinuity 清除与多次 outage。
4. Control 在 validated moving latest root 完成资格/数值检查后报告 Eligible，
   原子消费 anchor并提交原局部 state，world origin设为当前segment；metadata与
   实际局部world重开一致。现有 solve/admission/cadence保持不变。
   增加 typed 合法组合、fatal retry clean equivalence 与下一 active update tests。

源码 seam：`completeActiveSegment()`、`stageMovingVisualEvidence()`、
`materializeMovingCandidate()`、`runCurrentGraphLifecycle()`、owner swap；
transaction 复用 `internal/vio_update_transaction.hpp`。清空共同状态不能混入
subject-only transform，更不能让 control 不维护 anchor。

**Control 快照门：第一阶段在此停止并向 main 回报。** 提供共同修复的已观察 RED、
GREEN test 结果、typed producer/sidecar 接线状态及 source delta。冻结包含共同
修复和相同观测的 control snapshot/build 后，才进入 subject。不要在冻结前将
transform 混入 control。此门是实施交接，不声明完整 V2_03 qualification 已通过。

## 3. 第二步：subject 的唯一 transform 与本地验证

本地实施已完成：subject 定向 **13/13 PASS**、全套 **221/221 PASS**；production
及测试 diff 的独立 review 均已完成、无新缺陷。Control 固定 Eligible、subject
固定 Applied及非零输出oracle的角色边界保持不变。

从冻结共同 control 生成隔离 subject snapshot/build；唯一 source delta 是
validated owner 安装前的 world expression 与 Eligible→Applied 结果。两臂使用
相同 config/meta 参数快照，bridge 不增加 config activation 或产品 test fault-arm。
Subject在同一次bridge安装中将world origin改为source lineage origin；control
仍使用当前segment。RootCommitted sidecar反映各自实际目标world，rollback恢复原值。

角色测试 delta 明确固定：frozen control 的成功恢复断言 `Eligible`、原 local
world 输出与当前 segment world origin；subject 的成功恢复必须断言 `Applied`，
并用非零 yaw/translation 的独立输出 oracle 验证实际变换及 source world origin。
不能只把 enum 期望改名，也不能用同时接受 Eligible/Applied 的断言掩盖角色差异。

先独立数学 RED，再实现转换：覆盖非零 yaw/translation、不同 tilt、非零 local
velocity、旧/新 bias 不同；断言 latest position 对齐、gravity 不变、bias 原值、
所有 retained world state/cache 一致，且相对 geometry 保持。数值退化测试包含
`rho=0`、`rho=tau`、`nextafter(tau,0)`、`nextafter(tau,+infinity)` 与正常旋转。
Scalar 门的边界 oracle 与合法 SO(3) 输入的集成 oracle 分开，不能依赖构造旋转
后舍入恰好命中 threshold。Non-finite、timestamp mismatch 分别验证最早 typed reason。

沿用 internal one-shot fault seam 验证 propagation、转换、owner install 前失败，
检查 anchor/evidence/span/lineage exact rollback，以及下一次 clean retry。
Moving pending/recoverable、static bypass、合法 discontinuity、EOF pending、
二次 outage 与下一 active graph 均由 sequence tests 覆盖。

Apps 同步完成 sidecar RED→GREEN：每 processed packet 一行，integer timestamp、
typed initialization/bridge、update 前完整有序 frontend observations 与原始 keyframe
bit。验证 sidecar on/off 的算法 payload identity、Eligible/Applied 合法映射、
static/discontinuity/fatal 行与 `ok`→trajectory timestamp cadence。复用 existing
session、writer 和 evaluator，不建立 replay 框架。

Apps 已回报实际 `writeTum` sidecar off/on 的 est+kf 原始 bytes identity、diag、
config及terminal行为测试 PASS。Observation non-finite值的sidecar编码为
`{"f64_bits":"<固定16位hex>"}` tagged value，按spec §5保留原double bits。
Apps CLI误flag修复 **1/1 PASS**。Runner完整mock suite **22/22 PASS**，main已独立
重跑通过；Static Pending准确tuple及非法变体经独立复核，最终review结束、无新
发现。随后 launcher pre-publication seam 完成真实 native start/upload 的 RED→GREEN，
pair **28/28**、native **35/35**、native TUM mocks **32/32 PASS**，独立 review
通过，immutable run.json P1 闭合。重冻后的 paired prep 与 C1 qualification 已完成。

定向 estimator/apps/scripts tests 通过后，运行受影响 unit suites及必要构建检查。
报告真实命令与结果；仅在修改、失败或未解决风险需要时扩大测试。此阶段不运行
完整 TUM-VI room1–6，也不以 benchmark 代替 rollback tests。

## 4. 第三步：fresh C1 qualification → S1

实验问题：在相同输入与局部 root cadence 下，仅改变 world expression，能否同时
降低全 6 组跨段 translation RMS 与共同 timestamp ATE？用户已授权远程 C1→gate→S1，
两次隔离 V2_03 processes 已执行；完整输入 manifest 与 source snapshots 绑定在
`/tmp/phad-m5-recovery-pair-launcherfix-20260907/plan.json`。

**实际结果：fixed-input/cadence 协议 STOP。** C1 经 main 与独立 `verify_control`
通过，`diag.csv/est.tum/kf.tum` 与冻结 baseline 全字节一致；六个恢复 root
`459/809/937/1097/1343/1563` 全部 Moving Eligible。S1 build、unit、V2_03 均完成
且 return code 为 0，但 `check-pair` 在 `outage.indices` 停止：
expected `[434,702,930,1011,1158,1522,1631]`，
actual `[434,702,930,1010,1159,1523,1631]`。

首差记录：真实输入最早分叉 `i=484`、`ts_ns=1413394906105760512`，observations
C1/S1 为 `34/32`；更早 `i=460` 输入仍同，`pnp_inliers=28/26`，仅记录该事实。
Initialization reason 首差 `i=716` 为 `6/5`，status 首差 `i=808` 为
`initializing/ok`，segment 首差 `i=1010` 为 `3/4`。Reviewer 已最终确认 `i=484`
为首次有序 ingestion 硬门失败；`0..458` 的 diag/est/kf 原 bytes 独立通过，
`i=459` 的 Eligible/world1→Applied/world0 为预期。独立 review 已结束。
原始证据、run IDs 与结论边界见
[first input fork 记录](../research/2026-09-07-note-m5-recovery-pair-first-input-fork.md)。

运行前记录：两臂 source archive/manifest 与 binary、共同 toolchain/libraries、
input/evaluator identity、完整 canonical config/meta、500 ms horizon、sidecar path。
沿用现有 provenance 手段，不为此引入新 identity 框架。两臂实际输入以 sidecar
`i/ts_ns/frontend` 有序投影逐行 exact 比较，包含 `frontend.is_keyframe`；具体
JSONL v1编码以spec §5和 `apps/recovery_bridge_log.hpp` 为准。

**C1 qualification gate：**

- `diag.csv/est.tum/kf.tum` 复现冻结 bridge 前 control；共同修复或观测导致原输出
  改变即 STOP，不静默重设 baseline。
- outage/root/status/segment/attempt cadence符合 spec；sidecar 明确报告 Eligible，
  并核验 origin/lineage/latest timestamps 与 initialization moving committed。
- 列出全 J 每个恢复 root 的 applicability；Eligible 数为 0 即停止，不运行 S1。

C1 通过后运行 S1 一次。首次实际 Applied 前的全部算法 payload/input 必须 exact，
包括 `0–433`、outage→root pending，以及之前的 static bypass；全程有序 input 与
status/segment/root/attempt cadence保持不变。任一资格/身份/事务失败立即停止。

## 5. 第四步：产品判定与条件性扩展

当前因协议 STOP，以下 J/共同支持 ATE 产品裁决未执行，core/TUM 未扩展，阈值与
PnP 未调整。Evaluator 准备产物保留于执行目录 `evaluator-prep/`。本次结果不构成
机制证伪或数值收益失败；下一候选仅为 `i=460` 同输入下 gauge 等变性的只读因果
检查，尚未执行；reviewer 首差复核已完成，当前停止调查。

固定 `J={(433,459),(701,809),(929,937),(1010,1097),(1157,1343),(1521,1563)}`；
六组均保留，static 事件不删除。使用 spec §7 的 GT-corrected relative-pose error，
逐事件报告 translation（m）、rotation（rad）及 RMS/median/max。端点缺失不缩减 J。
ATE 在共同 timestamp 上各臂全程只做一次 fixed-scale SE3 alignment；同时保留原生
全支持集 ATE/RPE、completion/coverage、支持集数量/差集与 segment 向量。

V2_03 同时要求 finite、zero failed/rejected、全部 eligible 对应 Applied、全 J
translation RMS 与共同 timestamp ATE 严格下降、completion/coverage不降。
失败保存首个 typed expected/actual 与完整身份后停止，不调参。机制成功但产品门
失败同样结束本实验，不自动扩大矩阵。

**只有 V2_03 通过后**，才扩 core-4 `MH_01/V1_03/V2_02/V2_03` 的既有多指标
envelope，`G_ATE/G_RPE <= 1.05`；随后 TUM-VI room1–6 作 guardrail，要求全程
bridge NotApplicable 且 `diag.csv/est.tum/kf.tum` payload identity。保留已合格
V2_03 pair 的独立 source/input identity供 core复用。各层失败停止当前矩阵。

保存独立 #58 本地 evidence/report：逐门 expected/actual、全 J、共同与原生支持集、
source/config/input/evaluator、命令、实际执行与未执行项。#51 formal RED 独立记账。
不因合同落盘将任何实现、C1 或产品测试标为 completed。

## 6. 授权与停止边界

本计划记录用户已确认的合同、实施与远程 C1→gate→S1 授权及实际结果。当前已因
fixed-input/cadence 协议失败停止；本次只更新 plan 与短证据记录，独立 review
结论已回填，完成后停写。重试、扩展矩阵、调参及新增因果实验均未执行。
Commit、push、PR与其他远程写入不属于自动步骤。
历史数值入口与 sidecar/schema authority均见 spec；尚未执行的 guardrail保持 pending。

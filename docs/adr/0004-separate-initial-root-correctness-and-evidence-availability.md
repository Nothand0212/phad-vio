# ADR-0004：分离 initial-root correctness 与 stereo-evidence availability

- 状态：已接受
- 日期：2026-09-01
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 关系：细化 [ADR-0003](0003-estimator-owned-initial-moving-root.md) 的验证职责；不改变 estimator ownership、metric-stereo evidence、candidate solve 或 atomic commit 合同

## 背景

M5 Slice B 的首个真实运行证明了两种产品责任具有不同 owner：

- `initial-root correctness`：当 qualified connected metric-stereo 与 raw IMU
  evidence 已经存在时，`VioEstimator` 是否只提交通过全部 gates 的 latest
  `X/V/B`；
- `stereo-evidence availability`：frontend 是否能在固定真实 prefix 内足够早地
  产出满足 population 与 per-pose support 的 stereo observations。

固定 `V1_02-118` 上，`completion >= 0.90` 要求 first `ok` 不晚于 0-based
index 11；index 0–11每帧只有 4 个 positive-disparity observations，而已冻结的
weakest non-gauge pose support 要求 10。该运行直到 index 103 的 support 达到 10
时才形成首次 attempt，并在同一 revision 通过 gyro、gravity/velocity、joint
consistency 与 current graph 后提交 root。因此，原单一 gate 同时约束了 estimator
correctness 与 frontend evidence latency，无法把失败归属到一个 module。

同步诊断也具有独立语义：`emitted_stereo` 统计已经成功配对的 stereo frames，包含
尚未被 session 消费或不能形成首个正时长 IMU packet 的 pair；它不是 estimator
processed-packet count。

## 决策

M5 后续使用两个独立、正交的产品责任：

1. **Initial-root correctness** 继续归 #51 / `VioEstimator`。Formal evidence由
   deterministic exact-state oracles、fixed `V1_02-118` estimator-lifecycle gate、
   TUM VI root/continuity gate与fresh MH_01 static regression组成。V1 lifecycle
   gate验证 typed pending/attempt/commit、external cadence、single-segment finite
   trajectory与transaction；它记录 completion、coverage、ATE、RPE 和
   `emitted_stereo`，但不以这些 frontend-availability/available-span 指标决定 #51。
2. **Stereo-evidence availability** 由独立 frontend slice拥有。它复用同一
   `V1_02-118` input identity与原数值产品目标：completion/coverage 均不低于
   `0.90`，ATE/RPE(1 s) translation RMSE 均不高于 `1.0 m`。该 slice 必须通过
   真实 stereo matching提升 qualified evidence availability；不得改变 #51 的
   support、population、mono exclusion、candidate或transaction合同。

`V1_02-118` 明确定义为 session 实际交给 estimator 的前 118 个 packets：
`image_frames == diag rows == 118`。`emitted_stereo` 保留 synchronizer 的 paired
counter语义，作为诊断记录，不再充当 consumed-packet authority。

Formal V1 lifecycle test通过 `runOfflineVoSession()` 的 in-memory result读取每帧
`InitializationDiagnostics`；regular `diag.csv` / `summary.json` schema保持不变。
Static 与 moving path按各自合同解释 cadence：#51 的 fixed V1 lifecycle gate必须
实际覆盖 moving commit；未来 availability gate允许 static precedence在 evidence
足够时自然提交。

## 后果

- #51 的 checkpoint 表示 estimator-owned initial moving root correctness，不代表
  fixed V1 prefix 已达到 frontend evidence-availability目标。
- 原 V1 数值健康目标保留并转交独立 frontend issue/slice；其 issue identity在创建后
  回填 roadmap 与 checkpoint。
- V1 lifecycle 与 frontend availability 可以各自 RED/GREEN，避免通过改 estimator
  support语义吸收 frontend不足，也避免用 coverage掩盖正确的 typed pending。
- Slice B 当前实现仍需闭合既有 spec 已要求的 parallel evidence retention、
  initializing diagnostics 与 exact MH control validation，再重跑修订后的 formal
  matrix。

# ADR-0003：由 `VioEstimator` 建立并原子提交 initial moving root

- 状态：已接受
- 日期：2026-09-01
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 关系：补充 [ADR-0001](0001-gtsam-vio-backend.md)，不改变 GTSAM 后端与 canonical `X/V/B` state 选择

## 背景

M4 已具有单一 `VioEstimator::update()`、完整 `X/V/B` graph、raw IMU interval、PIM、bounded window 与 packet transaction。现有 moving bootstrap 却只用短 IMU suffix 的 `mean(acc)` 估计 tilt，并以零 velocity、零 bias 直接建立 root；graph 可求解不等于该 moving root 已完成 gyro bias、gravity 与 velocity 闭合。

M5 Slice B 需要在保持现有产品 seam 的前提下，让无静止起步只在多帧视觉与惯性证据一致时产生首个导航状态。该决策同时受 deterministic exact-state oracle 与 `V1_02_medium` 118-frame 真实产品 RED 约束。

## 决策

Initial moving root 由 `VioEstimator` implementation 私有拥有。调用方继续只提交 `VioMeasurement` 并消费 `VioUpdateResult`；不新增 `initialize()`、第二 estimator 或第二 backend。

`VioUpdateState` 保持唯一 estimation-state owner，并复用单一 bounded window。Live frame 显式区分 visual optimizer seed、经过 connected metric-stereo graph 验证的 visual pose evidence与已提交 navigation state；navigation candidate只存在 isolated local copy。未对齐的 PIM propagation 永不成为 visual pose authority。

Static 与 moving evidence 并行积累，static-ready 始终优先。Moving path 只使用包含 latest verified visual frame 的 connected visual component；至少 6 个 time-distinct poses 且跨度不少于 250 ms 后，每个 evidence revision 最多执行一次 deterministic candidate attempt。

Candidate 顺序固定为 gyro-bias alignment、final-bias fresh PIM、fixed-scale gravity/per-frame velocity closure、bounded joint refinement、joint consistency、existing current graph validation。Existing graph 的 validated optimized latest `X/V/B` 是唯一可提交状态。

一个默认 rollback 的 packet transaction 保护 live evidence。Candidate 始终为 isolated local state；可恢复拒绝保留 verified evidence并返回 `kInitializing`，fatal error rollback current packet，成功时原子安装 graph state并返回 `kOk`。

现有 `m_enable_moving_bootstrap` / `estimator.enable_moving_bootstrap` 原位启用 formal moving initializer，保持 default `false`。旧短时 moving-root shortcut 与 accumulated-seed initial-root path 退役，不保留兼容路径。

## 后果

- Public measurement seam、canonical `X/V/B` keys、existing graph lifecycle 与 default 权限不扩大。
- `UpdateDiagnostics` 增加 always-present、in-memory-only 的 compact `InitializationDiagnostics`；regular CSV/JSON schema 不变。
- Rank/condition、joint consistency 与 current graph validation 是相互独立的 admission gates；normalized condition 不声明 posterior physical precision。
- Slice B 只交付 initial moving root。Post-outage cold-root recovery、cross-root world-frame continuity、posterior precision admission 与 default-on 由后续独立工作决定。

## 验证

该决策以两层 deterministic oracle、`V1_02-118` moving RED、fresh `ba607c5` MH_01 static control 与 TUM VI corridor1 首100帧 product gate验收。全部 bounded gates 通过后形成 Slice B checkpoint并停止。

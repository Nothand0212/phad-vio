# ADR-0002：synthetic 资格阶段采用 gyro-only `G(k)` state

- 状态：已接受（阶段性；有明确退役条件）
- 日期：2026-08-18
- 关系：补充 [ADR-0001](0001-gtsam-vio-backend.md)，不取代完整 VIO 的
  `X/V/B` 目标

## 背景

[ADR-0001](0001-gtsam-vio-backend.md) 将 `imuBias::ConstantBias` 的 `B(k)`
定义为完整 VIO 的 accelerometer/gyroscope bias state。当前
`PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1` 只资格化 gyro measurement 到在线 gyro
bias state、rotation-only factor 与 random walk 的最小因果边；它不接入
accelerometer、velocity、gravity、真实数据或产品 caller。

Q3 的冻结运行已经以 `HYPOTHESIS_FAIL`、权限 `STOP` 结束。Q3 的
`visual_posterior_aligned_nuisance` 不是已资格化的 physical gyro bias，不能作为本阶段的
初值或 prior。

## 决策

仅在 `PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1` 的 default-off、无 real caller
synthetic 资格阶段，允许 estimator 内部使用：

```text
G(k) = gtsam::Symbol('g', frame_index) -> gtsam::Vector3
```

`G(k)` 只表示在 body frame `B` 中表达、单位为 `rad/s` 的三维 gyro bias。每个实际进入
factor graph 的 `Pose3 X(k)` 都有且只有一个对应的 `G(k)`；这里的 `k` 是该 pose 的
`frame_index`，包含 accepted non-keyframe graph state。

`G(k)` 的 GTSAM key、type 与 ownership，以及相关 factor 与 lifecycle，只由 estimator
PIMPL/private implementation 拥有，不得进入 public API。允许按本协议通过不含 GTSAM
key/type 的 project-owned POD 公开 in-memory bias value 与 diagnostics；它们不得成为 config
key 或持久 artifact schema，也不得形成真实数据或 app caller。

本协议唯一允许的 activation seam 是 test code 直接构造 project-owned POD
`EstimatorOptions::m_gyro_bias`；它虽然位于 estimator public value type 中，但在本阶段严格为
test-only、in-memory、default-off。不得为它新增 CLI/config parser、`flattenConfig()` 条目、
`config_hash` 输入、persistent artifact 字段或任何 apps/session/real-data caller。该 seam 只选择
是否启用 estimator private implementation，绝不公开 `G(k)` 的 GTSAM key/type、PIM、factor 或
ownership。

完整 VIO 的 canonical key 合同保持不变：`B(k)` 继续且只用于
`imuBias::ConstantBias`。同一 factor graph 中不得以 `G(k)` 和 `B(k)` 重复表示同一个
physical gyro bias。

当包含 velocity、accelerometer、gravity 与完整 bias 的 `X/V/B` slice 另行获得实施授权时，
必须直接删除 `G(k)` state、gyro-only factor 及其 lifecycle，并改用 `B(k)`；不得引入 key
alias、compatibility adapter、dual-write、migration fallback 或其他兼容层。

Q3 nuisance estimate 不得用于初始化或约束 `G(k)`；本决策不改变 Q3 的
`HYPOTHESIS_FAIL/STOP` 结论，也不授权重跑、重解释 Q3 或进入真实数据资格化。

## 理由

现在直接引入六维 `B(k)` 会同时引入尚未资格化的 accelerometer bias、velocity 与 gravity
语义；冻结 Q3 nuisance bias 又会把已失败的半窗常量假设带入新图。阶段性的三维 `G(k)`
只暴露本协议需要验证的 gyro bias 因果边，同时通过明确删除门保持与 ADR-0001 长期目标的
单向演进关系。

## 后果

- synthetic graph 可以独立验证 gyro bias state、random walk、rotation-only factor、重建与
  transactional writeback；通过不代表完整 VIO 或真实序列收益。
- `G(k)` 是受协议约束、预期删除的资格机制，不是第二套长期 bias 表示。
- full `X/V/B` 实施必须以删除 `G(k)` 为 activation gate；不存在兼容期或双表示阶段。

## 被拒绝的方案

- **冻结 Q3 early/full constant nuisance bias**：Q3 已因 half-window stability gate 失败，且该
  nuisance 未被认证为 physical gyro bias。
- **采用 piecewise offline bias**：仍绕过在线状态、process model 与优化后 bias 回写，不能资格化
  当前需要的因果边。
- **现在直接使用 `B(k): imuBias::ConstantBias`**：会把未获资格的 accelerometer/velocity/gravity
  维度带入本阶段，扩大机制归因范围。
- **额外复制一套 `Rot3` state**：与现有 `Pose3 X(k)` 重复表达 rotation，增加同步与 gauge 风险。
- **让 `G(k)` 与 `B(k)` 共存或桥接**：alias、adapter、dual-write 或 migration fallback 会形成两套
  physical gyro bias 权威，违反单一 canonical key 与无兼容层约束。

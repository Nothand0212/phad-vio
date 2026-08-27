# 视觉中断期轨迹连续性 spec（片 1b）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-26；1b 合同修订：2026-08-27

issue：[#43](https://github.com/Nothand0212/phad-vio/issues/43)（epic：[#42](https://github.com/Nothand0212/phad-vio/issues/42)）

control：[M4 checkpoint `c999f58` / `default_0337287b`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)

证据：[方向分析](../research/m4-next-direction-analysis.md)、
[coast 传播漂移探针](../research/m4-coast-propagation-probe-design.md)、
[seed-span 补充探针](../research/m4-seed-span-propagation-probe-design.md)、
[活动局部估计开源对照](../research/m4-local-map-continuity-open-source-comparison.md)与
[1a 机制及产品门记录](../plans/2026-08-26_m4_world_frame_inheritance_slice1a_c5177d31.plan.md)。

## 1. 目标与证据边界

片 1b 的目标是维持 active local estimator continuity：在 raw IMU 连续、
当前视觉 overlap 不足以达到 `min_pnp_inliers` 时，仍让已验证的当前观测
进入 active segment 的 window、track 与 landmark 生命周期，并在满足既有
因子门时继续约束旧段世界系。后续 packet 一旦按既有 support predicate
恢复，就在同一 `segment_id` 中继续；只有当前 endpoint 仍不受支撑且会
超出既有 visual-coast budget 时，才完成该段并进入诚实重开。

当前决策由以下已观测事实支撑：

| 事实 | 结果 | 合同含义 |
|---|---:|---|
| 原 coast 路径的窗口地图 | `25→23→21→19→17→14→11→0` | 低 overlap 时停止摄入会把恢复所需的地图生命周期耗尽 |
| 同期 `num_shared` | 长期停在 `2–5` | 新 track 未成为 map landmark，后续 packet 无法达到既有 support 门 |
| 1a V1_03 产品门 | control `1.406996 m`；1a `2.249778 m` | 单帧 root seed 不能形成跨段视觉校正，传播的绝对锚误差被 root prior 固定 |
| 1a support→root 位置误差 | median `0.386915 m`、p90 `1.156857 m`、max `1.708144 m` | 仅传播 `NavState` 不构成可校正的世界系连续 |
| root→下一视觉支撑帧 | local median `0.016123 m`；旧 support→该帧 `0.395970 m` | 局部新段可以稳定，但旧、新段之间没有视觉因果边 |

因此，1b 的产品路径在 active segment 中优先完成观测摄入、landmark
播种与既有 visual factor 附着。`codex/m4-world-frame-inheritance-1a`
保留为机制和失败证据；片 1b 的产品实施以里程碑分支为起点。

## 2. Authority 关系与不变项

本文具体化 [M4 minimal full-state VIO spec](m4-minimal-full-state-vio-spec.md)
§3.5 的 visual-outage recovery 合同，不改变其余 full-state VIO 拓扑：

- 唯一测量摄入 seam 仍为 `VioEstimator::update()`；frontend 和 apps 不接收
  estimator 的 overlap、graph 资格或 recovery policy。
- 每个 accepted packet 仍使用现有 `X/V/B`、IMU factor、bias-RW factor、
  root priors、window size、eviction/reintegration 与 GTSAM solver。
- `min_pnp_inliers`、`min_landmark_observations`、
  `min_track_observations_for_seed`、`min_seed_observations`、keyframe 判定与
  所有数值不变。
- visual factor 仍只是现有 `GenericStereoFactor`；没有新 factor 类型、
  solver、loop closure、global map 或 stitching。
- `MeasurementDiscontinuity` 仍是独立的硬切段事件；旧、新 segment 之间
  没有 IMU、bias-RW、visual、landmark 或 posterior bridge。
- 传播异常或非有限 pose/velocity/bias 仍是 hard failure，不会返回
  origin 作为成功结果。

§3.5 中“没有可用视觉约束时不建立 visual factor”仍成立。片 1b 只把
“达到完整 PnP support 门”与“存在满足现有单个 landmark 门的可用
visual factor”分开；后者存在时可以在 coast 期间约束 active segment。

合同使用三个不可互换的视觉概念：

| 概念 | 成立时机 | 行为含义 |
|---|---|---|
| observation retained | 已验证 observation 在 transaction 中写入当前 `WindowFrame` | 维护 observation 与 track 生命周期 |
| graph visually constrained | 最终成功 solve 的 graph 含连接当前 frame 的 `GenericStereoFactor` | 运行既有视觉质量路径并用视觉残差约束 active segment |
| full visual support | 摄入前 committed map 上的 positive-disparity `num_shared >= min_pnp_inliers` | 当前 packet 可结束 low-support coast，并在成功 commit 后刷新 support anchor、重置 coast duration |

低于 full visual support 门的 packet 统一称为 **low-support coast**。它可以
graph visually constrained，因此实现中的 low-support predicate 不表达
“IMU-only”；当前 graph 是否含视觉约束由实际 factor 集合独立确定。

## 3. 模块边界与状态所有权

1b 继续以 estimator 内部的既有事务状态作为 observation、track 与
landmark 的唯一 staging/ownership 边界。这组 window、track、landmark 与
graph state 共同构成本片的 active local estimator：

| 数据 | 唯一 owner | 生命周期 |
|---|---|---|
| 当前帧已接受 observations | `WindowFrame::m_observations` | 跟随 window 插入、eviction 与 segment completion |
| track timestamp history | `VioUpdateState::m_track_times` | 用于既有 seed-age 门；随 transaction 提交/回滚 |
| 世界系 landmarks | `VioUpdateState::m_landmarks_w` | 按既有 seed、prune、cull/rebirth 与 window 规则维护 |
| support/coast/segment 状态 | `VioUpdateState` | 只由 estimator 在同一 update transaction 中更新 |

apps 只把 project-owned `VioMeasurement` 和既有 `keyframe` bit 交给
`update()`，然后消费 result/diagnostics。调用方不缓存、重放或重新注入
estimator-private observations。

## 4. Observation、track 与 landmark 生命周期

### 4.1 当前 packet 的摄入顺序

对 raw IMU 连续且通过输入校验的 active-segment packet：

1. 在任何当前 packet 摄入之前，仅使用 committed `m_landmarks_w` 统计
   positive-disparity `num_shared`。
2. `visual_supported := num_shared >= min_pnp_inliers`；该结果在本 packet
   中锁定，不使用后续新 seed 的 landmark 重算。
3. 进入 `VioUpdateTransaction`，传播 `X/V/B` 并构造当前 `WindowFrame`。
4. `WindowFrame::m_observations` 接收已验证 observation。supported 路径仍
   使用现有 PnP 仲裁后 mask；coast 路径没有 PnP mask，因此保留全部
   已验证输入 observation。
5. `m_track_times` 按现有 normal-path 口径记录完整已验证
   measurement，包括 zero-disparity 和可能被 PnP mask 从 frame 中剔除的
   shared outlier。
6. 只有当前帧是 keyframe，且 track age、positive disparity、cull/rebirth
   与 backprojection 都通过现有门时，才创建新 landmark。单个 ID
   无法 backproject 时只是该 ID 不 seed。
7. zero-disparity observation 可以保留 ID 与 timestamp 生命周期，但不
   backproject、不 seed、不建立 stereo factor。

ordinary coast 不使用首段 root bootstrap 的 `pending_seed_obs`。window capacity、
non-keyframe eviction、raw-IMU reintegration、landmark prune 和 hanging-landmark 规则仍对
已摄入 coast frame 生效。`completeActiveSegment()` 仍清除 window、landmark、
track history 与相关 cache，不把它们带入新段。

### 4.2 Graph admission 与质量处理

已摄入 observation 只在同时满足以下条件时进入 graph：

1. `disparity_px > 0`；
2. 该 ID 已在 committed/staged `m_landmarks_w` 中；
3. window 内该 landmark 的 positive-disparity observation count 达到现有
   `min_landmark_observations`。

资格满足后，`buildGraph()` 继续建立现有 `GenericStereoFactor`。本 packet
新 seed 的 landmark 可以使同窗口内较早的 coast observations 获得现有因子
资格；这不会反向修改本 packet 已锁定的 `visual_supported`。

只要当前 graph 存在 visual factors，就按现有 configuration 运行同一套
cheirality/mean cull 与 optional reopt，不以 `low_visual_support` 作为质量处理
禁用信号。没有 visual factor 时，该视觉质量步骤自然跳过。

`num_current_visual_factors > 0` 只证明当前 committed graph 受视觉约束；它
不改变已锁定的 full visual support、不刷新 last-support anchor，也不重置
coast duration 或 `unsupported_span_ns`。

## 5. 精确状态机与 status cadence

### 5.1 Active segment 决策顺序

`m_visual_coast_horizon_ns` 保持现有 `500'000'000 ns` (500 ms) 取值与
inclusive-boundary 语义；本片不为它新增 CLI 或 canonical-config key。

| 当前 packet | 行为 | status / estimate | segment |
|---|---|---|---|
| valid `MeasurementDiscontinuity` | 原子完成 active segment；当前 packet 不建 state/factor | `kDiscontinuity` / 无 | `segment_id` 递增一次 |
| raw 连续，摄入前 `num_shared >= min_pnp_inliers` | normal update，或从 coast 在原段恢复 | `kOk` / 有 | 不变 |
| raw 连续，support 不足，累加后 `coast_duration <= horizon` | 传播、摄入、建图并提交 coast state | `kOk` / 有 | 不变 |
| raw 连续，support 不足，当前 interval 将使 `coast_duration > horizon` | 当前 packet 不摄入；原子完成 active segment | `kVisualOutage` / 无 | `segment_id` 递增一次 |
| malformed raw/discontinuity/observation | 零修改返回 | `kInvalidInput` / 无 | 不变 |
| propagation/primary graph/optimizer/non-finite hard failure | 全量 rollback | `kFailed` / 无 | 不变 |

support 判定先于 visual-outage 判定。因此：

- 一个 unsupported packet 使 coast duration **恰好等于** horizon 时，仍以旧段
  `kOk` 提交；
- 下一 packet 若已满足 support predicate，则优先在原段恢复；
- 下一 packet 若仍 unsupported，且其 raw interval 会超出 budget，则单次
  返回 `kVisualOutage`，不产生同 timestamp 的第二个 result 或 output row。

每个成功提交的 low-support interval 都累加 coast duration，包括 graph
visually constrained 的 interval。只有达到 full visual support 且 primary
graph transaction 成功 commit 的 packet 才结束 coast 并重置该 budget；visual
factor count 不参与 segment-liveness 判定。

outage 后沿用现有 uninitialized/bootstrap 路径；在新 root 成功提交前返回
`kInitializing`。新 root 不继承旧段 map 或 posterior。
uninitialized 期的 root seed gate 与 `kRejected` 语义沿用 M4 full-state VIO 现有合同；
片 1b 不改变该路径的 status cadence。

### 5.2 Support 与 PnP 语义

support predicate 保持为摄入前的 positive-disparity shared-ID count。PnP 只是现有
pose initialization proposal：成功时按现有仲裁规则替换传播初值并 mask shared
outliers；未成功时继续使用传播初值构建 graph。PnP 未成功不单独把
已达 support 门的 packet 改判为 coast；primary graph transaction 必须成功才能
提交恢复。

## 6. Transaction、失败与累计状态

### 6.1 原子边界

除纯输入校验和只读 predicate 计算外，下列所有 retained mutation 必须在
同一 `VioUpdateTransaction` 内 stage：

- coast duration、last-support anchor 与 `unsupported_span_ns`；
- propagated `X/V/B`、frame index、window frame 与 raw/PIM provenance；
- observations、track history、landmark refresh/seed/prune/cull；
- graph build/solve、optional reopt 的局部成功轮次；
- segment identity、counters 与 committed diagnostics。

primary propagation、eviction reintegration、graph build/solve 或结果有限性检查失败时，
整个 packet 回滚，失败 diagnostics 从回滚后的 committed state 重建。不保留
本 packet 的 observation、track timestamp、landmark、frame index、span 或 counter。

两个现有局部语义保持不变：

- 单个 ordinary landmark 无法 backproject 时，只跳过该 seed；
- optional outlier-reopt round 失败时，回滚到该 round 之前的 window/landmark
  状态并报告 `outlier_reopt_failed`；已成功的 primary solve 仍可提交 `kOk`。

coast visual-factor 导致的 primary solve 失败是 `kFailed`，不会丢弃视觉因子后
改以 IMU-only 状态提交。

### 6.2 `unsupported_span_ns`

`unsupported_span_ns` 是纯诊断累计量：

- 定义为当前 image timestamp 减去最后一个成功提交且满足既有
  support predicate 的 packet timestamp；
- 只有后续真正 supported transaction 成功 commit 才归零；root seed 不自动
  构成 support；
- 跨 visual-outage `completeActiveSegment()` 保留，用于显示连锁 outage 期间
  距离上次真实支撑的时长；
- `MeasurementDiscontinuity` 使该 anchor 失效并把 span 清零；
- 无 anchor 时为 `0`；不参与 support、outage、seed 或其他 runtime 决策。

`m_visual_coast_duration_ns` 仍是每个 active segment 内的有界 budget 计量，
`completeActiveSegment()` 时归零。两者的控制语义不合并。

逐 packet 诊断口径冻结为：

| 结果 | `unsupported_span_ns` |
|---|---:|
| 成功 committed coast | 当前 timestamp 相对 last-support anchor 的实际值；无 anchor 时为 `0` |
| 成功同段恢复 | `0` |
| committed `kVisualOutage` | 当前 timestamp 相对 last-support anchor 的实际值；anchor 跨 transition 保留 |
| outage 后 `kInitializing` | 有 anchor 时继续按当前 timestamp 累计；无 anchor 时为 `0` |
| valid discontinuity | `0` |
| `kInvalidInput` / full-rollback `kFailed` | 调用前 committed 值，不向当前 timestamp 推进 |

## 7. Segment 与 diagnostics 合同

### 7.1 Segment identity

- coast 与原段恢复期间 `segment_id` 不变。
- 成功的 `kVisualOutage` 或 active-segment `kDiscontinuity` transition 使
  `segment_id` 递增一次，并用 `completed_segment_id` 指明完成的旧段。
- session 的 `segments` 继续按出现 estimate 的新 `segment_id` 计数。
- 片 1b 在同一世界锚的 active segment 内恢复，不增加
  `robustness.reanchors`；该基线计数保持 `0`。

### 7.2 最小机制诊断

保留 `num_observations` 作为 raw measurement count，并新增三个 packet-level
committed counters：

| 字段 | 语义 |
|---|---|
| `num_retained_observations` | 当前 frame 最终写入 window 的 observation 数；包含 zero-disparity，使用现有 PnP mask 后口径 |
| `num_seeded_landmarks` | 本次已提交 transaction 中成功 backproject 并插入 map 的新 landmark 数；同 transaction 后续 cull 由现有 cull diagnostics 另行表达 |
| `num_current_visual_factors` | 产生最终 committed optimized state 的最后一次成功 solve graph 中，连接当前 frame 的 `GenericStereoFactor` 数 |

三项进入 `UpdateDiagnostics` 和 `diag.csv`。没有当前 committed frame 时，
update result 的三项均为 `0`。按现有 session cadence，`kVisualOutage` 和
`kDiscontinuity` 会写出零计数 row；`kInvalidInput` 和 `kFailed` 在写 row 之前
终止 session，其零计数仅存在于 `VioUpdateResult::diagnostics`。`diag.csv` 在
基线列尾按以下顺序追加：

```text
unsupported_span_ns,num_retained_observations,num_seeded_landmarks,num_current_visual_factors
```

不新增 `UpdateStatus`。从上一行 `unsupported_span_ns > 0` 到当前行归零、
且 `segment_id` 不变，就表示一次原段内恢复。
`num_current_visual_factors > 0` 且 `unsupported_span_ns > 0` 表示受视觉约束的
low-support coast，不表示 full visual support 或恢复。

## 8. TDD 与定向机制门

### 8.1 测试 seams

1. estimator 机制行为通过 `VioEstimator::update()` 的 result/estimate/diagnostics
   与既有只读 `observationTimestamps()` 验证。
2. status cadence、CSV schema/value 与 stop policy 通过 `OfflineVoSession` 验证。
3. 产品门通过现有 `phad_vo_bench` 与 EuRoC artifacts 验证。

测试不直接访问 `Impl`、`WindowFrame` 或私有 graph helper。

### 8.2 Red→green vertical cycles

1. all-new non-keyframe coast：observations 与 track timestamps 提交，seed/factor 为 `0`。
2. keyframe seed 门：未达既有 track 次数时不 seed；达门后仅
   positive-disparity IDs seed。
3. factor admission：`num_shared < min_pnp_inliers` 时当前 frame 可有 visual
   factors，但 packet 仍是 low-support coast、duration/span 继续累加，新 seed
   不回流到 support。
4. 同段视觉链恢复：上一轮 retained observations 经既有门 seed 并形成 factor；
   下一 packet 使用同一批 IDs 在摄入前达到 support 门，在同一 `segment_id`
   提交并令 `unsupported_span_ns` 归零。
5. boundary/cadence：exact horizon 是 `kOk`；下一 supported endpoint 可恢复；
   已有 visual factors 但仍低于 support 门的 interval 不重置 budget；下一
   unsupported over-horizon endpoint 为 `kVisualOutage`。
6. discontinuity 与 hard-failure subject/control 对拍：验证 window、track、landmark、
   frame index、span、segment 和 diagnostics 的清除或回滚。
7. `OfflineVoSession` 验证三个新 counter、CSV 顺序、status stop policy 与
   `segments/reanchors`。

每轮只先写一个可观察的失败测试，然后做使其转绿的最小实现。

### 8.3 机制门

按以下顺序执行，任一失败都停止在机制层：

1. 新 1b 测试与既有 coast、PnP、keyframe、outlier cull/reopt、eviction、
   discontinuity、rollback 及 `OfflineVoSession` 相关 filters；
2. 完整 `phad_estimator_tests`；
3. 完整 `phad_apps_tests`；
4. diff 审计确认 `update()` 签名、所有既有门限、solver 和 factor 类型
   未变。

机制 PASS 必须同时证明：coast retained/seed/current-frame factor 均可观测；
当前 packet 仍保持原 support 判定；后续 packet 同段恢复；over-horizon 与
discontinuity cadence 精确；factor-bearing low-support coast 仍连续消耗同一
budget；hard failure 后 subject/control 的后续合法调用一致。

## 9. EuRoC stop-on-failure 产品门

正式 run 使用行为基线 `c999f58` 的同一 canonical snapshot
`default_0337287b`，不新增配置增量键。门控严格串行：

```text
MH_01_easy → V1_03_difficult → V2_02_medium → V2_03_difficult
```

任一 sequence 失败立即停止，不跳过、不扫描 horizon/prior sigma/门限，
也不进入其余七条 EuRoC。

### 9.1 MH_01 硬门

- ATE `<= 0.070539 m`；
- `segments == 1`、`failed == 0`、`rejected == 0`；
- completion 与 coverage 均不低于 `0.999728`。

### 9.2 连续性机制靶

| sequence | baseline ATE 上界 | baseline completion 下界 | baseline coverage 下界 | segments 硬门 |
|---|---:|---:|---:|---:|
| V1_03_difficult | `1.406996 m` | `0.964635` | `0.974860` | `< 16` |
| V2_02_medium | `1.981976 m` | `0.968484` | `0.972731` | `< 9` |
| V2_03_difficult | `1.707513 m` | `0.506507` | `0.791345` | `< 52` |

每条机制靶必须同时满足：

1. 至少一次 `segment_id` 不变的 coast→recovery episode 中，在恢复前或恢复
   packet 上可见 `num_retained_observations > 0`、`num_seeded_landmarks > 0`
   与 `num_current_visual_factors > 0`；
2. 同一 episode 的 `unsupported_span_ns` 从 `> 0` 归零；
3. `segments` 严格低于表中基线，ATE 不高于基线，completion/coverage 不低于
   基线；
4. 用 `scripts/segment_ate_decomp.py` 复算的绝对段间分量不高于对应
   baseline；
5. `failed == 0`、`rejected == 0`、`reanchors == 0`，无 hard failure；
6. artifact 保留 clean code identity、完整 canonical config、命令与逐事件 diagnostics。

## 10. 实施基线与范围边界

片 1b 的实施分支从
`Nothand0212/m4-online-gyro-bias-synthetic-replay2` (`a7f7a34`) 建立。该
里程碑分支与 `c999f58` 在 `phad/`、`apps/`、tests 与 CMake 上无差异。
实施只引入本 spec 的 observation-intake、累计诊断、测试与 artifact
wiring；1a 分支不作为产品代码基线。

本片不改 [architecture](../design/architecture.md)、[conventions](../design/conventions.md)或
[roadmap](../design/roadmap.md)；不扩大 visual-coast horizon，不扫描 prior sigma 或放宽
门限；不做 #44、片 3 单目因子、片 4/M5 动态初始化、loop closure、
global map、cross-segment stitching 或新 factor 类型。

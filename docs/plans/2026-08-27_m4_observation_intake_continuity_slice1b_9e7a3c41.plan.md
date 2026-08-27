---
name: M4 轨迹连续性 Slice 1b 观测摄入解耦
overview: 在既有 VioEstimator::update() seam 内维持 active local estimator continuity，让连续 raw IMU 的低视觉支撑 packet 持续维护 observation、track、landmark 与既有 visual factors，并以 TDD、定向机制门和串行 EuRoC 产品门验收原段内恢复。
todos:
  - id: prepare-slice-base
    content: 经用户授权后从 a7f7a34 建立 1b 短生命周期分支，记录并保留无关 dirty paths，确认产品代码与 c999f58 同基线
    status: pending
  - id: red-retain-coast-observations
    content: 先经 VioEstimator public seam 写 all-new non-keyframe coast 摄入红灯，再做 observations 与 track history 进入既有 transaction state 的最小实现
    status: pending
  - id: red-green-landmark-lifecycle
    content: 逐个验证 keyframe、track age、positive disparity、cull/rebirth 与 backprojection 既有门，使 coast keyframe 可按原规则 refresh/seed landmark
    status: pending
  - id: red-green-graph-admission
    content: 先写低于 min_pnp_inliers 仍有 current-frame GenericStereoFactor、继续累计 coast budget 且不回流 support 的红灯，再复用 buildGraph 和现有 cull/reopt 质量路径
    status: pending
  - id: red-green-recovery-state-machine
    content: 逐个完成下一 packet 同段恢复、exact horizon、over-horizon outage 和 discontinuity cadence 红绿循环
    status: pending
  - id: red-green-transaction-truth
    content: 以 subject/control 对拍覆盖 invalid input、propagation、eviction、primary graph/solver 与 non-finite hard failure，验证新摄入状态全量 rollback
    status: pending
  - id: plumb-committed-diagnostics
    content: 接入 unsupported_span_ns 和 retained/seeded/current-frame-factor 三项 committed counters，完成 UpdateDiagnostics、OfflineVoSession、diag.csv 与 schema 测试
    status: pending
  - id: run-mechanism-gates
    content: 先运行新行为与相关 filters，再完整运行 phad_estimator_tests 和 phad_apps_tests，完成 diff/check 与 factor/solver/门限审计
    status: pending
  - id: freeze-clean-candidate
    content: 在差异审查通过且获得用户明确授权后形成带 #43 的 clean 候选 commit，不 push/merge
    status: pending
  - id: run-euroc-product-gates
    content: 按 MH_01→V1_03→V2_02→V2_03 串行运行 default_0337287b，每门核对机制计数、segments、ATE、completion/coverage 与绝对段间分量，任一失败立即停止
    status: pending
  - id: align-current-module-docs
    content: 以实际实现和门控结果同步 estimator/apps/bench 当前 README 与 #43 证据，交付可复现的命令、config 和 artifact identity
    status: pending
isProject: false
---

# M4 轨迹连续性 Slice 1b 观测摄入解耦

## 状态

**合同已逐项对齐；本计划尚未开始编码、commit、EuRoC run、push 或
merge。**

本片在 active segment 内解耦“当前观测是否被 estimator 持有”与“当前
packet 是否达到完整 PnP support”。已摄入观测仍只能经过现有
keyframe/seed/landmark/factor 门进入 map 与 graph，后续 packet 只在摄入前
shared-ID count 达到既有 `min_pnp_inliers` 时恢复 full visual support。
受既有 visual factors 约束但未达该门的 packet 仍是 low-support coast，并
连续消耗同一个 500 ms coast budget。

## 1. Authority、control 与实施身份

按具体性从高到低使用：

1. [`m4-world-frame-continuity-spec.md`](../specs/2026-08-26-m4-world-frame-continuity.md)。
2. [`m4-minimal-full-state-vio-spec.md`](../specs/2026-08-25-m4-minimal-full-state-vio.md)。
3. [#43](https://github.com/Nothand0212/phad-vio/issues/43) 正文及截至
   2026-08-27 的最新 comments。
4. [1a 机制及产品门记录](2026-08-26_m4_world_frame_inheritance_slice1a_c5177d31.plan.md)、
   [`m4-coast-propagation-probe-design.md`](../research/m4-coast-propagation-probe-design.md)、
   [`m4-seed-span-propagation-probe-design.md`](../research/m4-seed-span-propagation-probe-design.md)
   与 [`m4-local-map-continuity-open-source-comparison.md`](../research/m4-local-map-continuity-open-source-comparison.md)。
5. [`evidence-gated-integration.md`](../agents/evidence-gated-integration.md)、
   [`incremental-development.md`](../agents/incremental-development.md) 与
   [`git-workflow.md`](../agents/git-workflow.md)。

| 项 | 值 |
|---|---|
| 行为基线 | `c999f5891d8efbc353e2e4001df7656a9e19907a` (`c999f58`) |
| canonical config | `default_0337287b` |
| checkpoint | [`minimal-full-state-vio_c999f58_0337287b.md`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md) |
| 实施基线分支 | `Nothand0212/m4-online-gyro-bias-synthetic-replay2` |
| 实施基线 commit | `a7f7a34265dd1d9cd8d1fed2887a28ac0bcf5aa9` (`a7f7a34`) |
| 计划分支 | `codex/m4-observation-intake-continuity-1b`（获得授权后创建） |
| issue | [#43](https://github.com/Nothand0212/phad-vio/issues/43)；epic [#42](https://github.com/Nothand0212/phad-vio/issues/42) |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| artifact root | `/home/lin/Projects/data/phad-bench` |

`a7f7a34` 与 `c999f58` 在 `phad/`、`apps/`、tests 和 CMake 上无差异，
因此正式产品对比直接使用 `c999f58/default_0337287b` 权威 artifact。
实施不整体 cherry-pick `805212d`；`codex/m4-world-frame-inheritance-1a`
保留为代码机制和失败证据。

开工前记录 `git status --short --branch`。不 stash、clean、reset 或整理现有无关
dirty paths；若分支切换与它们冲突，停止并请用户处理权限边界。

## 2. 已对齐的决策

| 决策点 | 结论 |
|---|---|
| 连续性路径 | 优先在 active segment 内摄入观测并恢复；剩余 over-horizon outage 沿用现有诚实重开 |
| owner | 复用 `WindowFrame::m_observations`、`m_track_times`、`m_landmarks_w` 和既有全量 transaction |
| observation intake | 每个 accepted coast packet 保留全部已验证 observations，含 zero-disparity |
| landmark lifecycle | 按 normal path 的 keyframe、track age、disparity、cull/rebirth 与 backprojection 门 refresh/seed |
| graph admission | positive disparity + map 中存在 ID + 既有 `min_landmark_observations`，建立现有 `GenericStereoFactor` |
| quality path | graph 有 visual factor 时运行现有 cheirality/mean cull 与 optional reopt，不以 coast bit 禁用 |
| 视觉语义 | observation retained、graph visually constrained、full visual support 分别表达摄入、实际因子约束与完整支撑 |
| support predicate | 摄入前 committed map 上 positive-disparity `num_shared >= min_pnp_inliers`；当前新 seed 不回流 |
| PnP | 保持可选 pose proposal 语义；未成功不单独否定 support，primary graph commit 才确立恢复 |
| horizon | 当前 endpoint 恢复优先；unsupported 且累加后 `== horizon` 可提交，`> horizon` 才 outage |
| segment liveness | factor-bearing low-support interval 仍累加 coast duration；只有 full visual support 的成功 commit 重置 budget |
| rollback | propagation、intake、seed、window、graph、quality、span 与 diagnostics 同一 transaction |
| unsupported span | 跨 visual-outage segment 的纯诊断累计量；真实 support commit 归零，discontinuity 清 anchor |
| segment / reanchors | 原段恢复不改 `segment_id`，不增 `reanchors`；只有 outage/discontinuity 完成段 |
| diagnostics | `unsupported_span_ns` + retained/seeded/current-frame-factor 三个 committed counters |
| status | 保持现有 enum 与一 packet 一 result cadence |
| 产品门 | `MH_01→V1_03→V2_02→V2_03`，机制生效与产品不回归同时成立，任一失败停止 |

## 3. 代码地图与单一因果边

### 3.1 现有耦合点

`VioEstimator::update()` 已把 support 判定放在当前 map 的
`num_shared` 上，但同一 `imu_only_coast` boolean 又同时阻止：

- `candidate.m_observations = measurement.m_observations`；
- 当前 measurement 进入 `m_track_times`；
- coast 上的 cheirality/mean cull 和 reopt。

结果是 coast packet 仍有 `X/V/B` 与 IMU/bias factors，但其观测、track age 和
新 landmark 资格在进窗前被丢失。

实施时把该 local predicate 收窄为 `low_visual_support` 语义；graph 是否受
视觉约束由实际 visual-factor presence 表达，不再由同一个 boolean 推断。

### 3.2 1b 的唯一新增因果边

```text
accepted low-support packet
  → observation/track lifetime enters the existing transaction state
  → existing landmark seed and graph gates may become satisfiable
  → existing GenericStereoFactor may constrain the active segment
  → a later packet may satisfy the unchanged support predicate
```

factor-bearing low-support packet 仍沿同一 coast clock 前进；上述因果链必须在
500 ms budget 内使后续 packet 达到 full visual support，才能保持原段 liveness。

不新增公开类型、input option、pending coast buffer、solver 或 factor。

### 3.3 预计修改文件

| 文件 | 计划修改 |
|---|---|
| `phad/estimator/vio_estimator.cpp` | 分开 full-support、low-support intake 与 actual visual-factor presence；复用 seed/buildGraph；以 visual-factor presence 驱动质量路径；实现 support-first horizon 与新 diagnostics |
| `phad/estimator/internal/vio_update_transaction.hpp` | 把 last-support anchor 与 unsupported span 纳入整体 snapshot state；window/track/landmark 继续由现有深拷贝覆盖 |
| `phad/estimator/types.hpp` | 新增 `unsupported_span_ns` 和三个 packet counters，不改 `UpdateStatus` |
| `apps/offline_vo_session.hpp/.cpp` | 映射三个 counters 与 `unsupported_span_ns`，在基线 CSV 尾追加四列 |
| `tests/estimator/*.cpp` | 按§5 的红绿循环新增/修订 public-seam 行为测试 |
| `tests/apps/offline_vo_session_test.cpp` | 锁定 CSV schema/value、status stop policy、segments/reanchors |
| `phad/estimator/README.md`、`phad/bench/README.md` | 实现通过后同步当前 lifecycle、diagnostics 与 artifact 语义 |

`VioEstimator::update()` 签名、`VioMeasurement`、apps/frontend 转换 seam、config parser 和
canonical config 均不改。

## 4. 数据流与状态机实施

### 4.1 Packet-entry 只读决策

保持现有 `m_visual_coast_horizon_ns == 500'000'000`（500 ms）以及
inclusive-boundary 语义；本片不新增 CLI 或 canonical-config key。

1. 先处理 `MeasurementDiscontinuity` 及 raw/observation validation。
2. 使用调用前 committed `m_landmarks_w` 统计当前 positive-disparity
   `num_shared`。
3. 锁定 `visual_supported`（full support）/ `low_visual_support`；本 packet
   的 seed、cull 或 solve 不重算该结果。
4. 若当前 endpoint supported，进入普通/恢复 transaction；若 unsupported，
   再判断新 duration 是否 `> m_visual_coast_horizon_ns`。
5. unsupported 且 over-horizon 的 packet 不建 candidate，仅提交既有
   `kVisualOutage` completion transaction。

### 4.2 Transaction 内的 observation lifecycle

1. 传播 `X/V/B`，检查 pose、velocity 与 bias 有限。
2. 新 candidate 先保留当前已验证 observations。
3. supported + PnP 仲裁成功时，沿用现有 shared-outlier mask；track history
   仍使用完整 measurement。
4. coast 上同样记录完整 track history；对 positive-disparity observations
   运行既有 far-return refresh 和 keyframe-only new-landmark seed。
5. 候选帧进窗，执行既有 capacity/eviction/reintegration 与 landmark prune。
6. `buildGraph()` 按现有 map-membership、positive-disparity 和
   `min_landmark_observations` 统一决定历史/当前 observations 的 factor 资格。
7. graph 有 visual factor 时运行现有 cull/reopt；保留产生最终 committed
   optimized state 的最后一次成功 solve 所使用 graph 的 current-frame
   visual-factor count。
8. primary solve 成功后写回 window/landmark，更新 support/coast/span/diagnostics，
   最后单次 commit。

第 7 步的 factor presence 不刷新 last-support anchor，也不重置 coast duration
或 `unsupported_span_ns`；只有摄入前已达门且第 8 步成功 commit 才完成恢复。

### 4.3 Diagnostics 计数时机

- `num_retained_observations`：candidate 完成现有 PnP mask 后的 observation count。
- `num_seeded_landmarks`：本次 committed transaction 中成功插入 map 的新 ID
  事件数；后续同 packet cull 不改写该事件计数。
- `num_current_visual_factors`：产生最终 committed optimized state 的最后一次
  成功 solve 所使用 graph 中，pose key 属于当前 frame 的 stereo factors 数。
- primary solve 后发生 cull 但没有成功 reopt 时，保留 primary solve graph 的
  计数；成功 reopt 时改用该 reopt graph 的计数；optional reopt 失败并局部
  回滚时，仍保留前一次成功 solve graph 的计数。
- 三个 packet counters 保持 update-local，只在整个 transaction commit 后发布到
  result，不在 `VioUpdateState` 中复制一份持久状态。
- full packet rollback 后三项均为 `0`；失败 result 的 stateful diagnostics 从
  调用前 snapshot 重建。

### 4.4 `unsupported_span_ns` 和 segment

- `m_last_visual_support_timestamp` 只在 initialized packet 的摄入前
  `num_shared >= min_pnp_inliers` 且整个 update commit 后建立/刷新。
- root seed 不刷新 support anchor。
- visual outage completion 保留 anchor/span；discontinuity 清除它们。
- committed coast/outage/initializing row 在有 anchor 时写当前 timestamp 的实际 span；
  恢复/discontinuity row 写 `0`；invalid/full-rollback failure 保留调用前值。
- `unsupported_span_ns` 只写 diagnostics，不作为 branch predicate。
- `segment_id` 只在 committed outage/discontinuity completion 时递增；恢复时不变。

## 5. TDD vertical cycles

所有机制测试都通过 `VioEstimator::update()` result/diagnostics 与既有
`observationTimestamps()` 只读 seam 验证，不直接查看 private state。每个循环必须
先观察 RED，再实现该单一行为的最小 GREEN。

### Cycle 1：coast observation intake

在 `tests/estimator/keyframe_update_test.cpp` 或新的同级 behavior fixture 中：

- root 后提交 all-new non-keyframe low-support packet；
- 断言 `kOk`、estimate 存在、`segment_id` 不变、PnP 未成功；
- 断言 `num_retained_observations == measurement.m_observations.size()`；
- positive-disparity 和 zero-disparity IDs 都在 `observationTimestamps()` 中出现；
- 断言 `num_seeded_landmarks == 0`、`num_current_visual_factors == 0`。

最小 GREEN 只解除 candidate observation 和 track-history 的 coast 摄入耦合，不改
seed、graph 或状态机。

### Cycle 2：keyframe 与 landmark 门

- 使用显式 `min_track_observations_for_seed` fixture，分别证明未达门和达门。
- 非 keyframe 在达 track age 后仍不 seed。
- keyframe 达门后，positive-disparity 新 ID 增加
  `num_seeded_landmarks`，zero-disparity ID 仍只有 lifetime。
- 被 cull/rebirth policy 禁止的 ID 不重建。

最小 GREEN 复用现有 refresh/seed loop，不新建 pending coast map。

### Cycle 3：coast graph admission 与 support 诚信

- 构造摄入前 `num_shared < min_pnp_inliers` 但 window 内已达
  `min_landmark_observations` 的 keyframe packet。
- 断言当前 `num_current_visual_factors > 0`，且仍按 coast 累加 duration/span。
- 断言当前新 seed 的 IDs 不使当前 `num_shared` 或 support 变化。
- 用既有 conflicting observation fixture 证明 coast factors 运行同一 cull/reopt 路径。
- 分别锁定 primary solve 后 cull 且无成功 reopt、成功 reopt、reopt 失败局部
  回滚三种结果的 `num_current_visual_factors` 口径。

最小 GREEN 让 `buildGraph()` 直接消费已摄入 window observations，并把视觉
质量路径的执行条件改为“有实际 visual factors”。

### Cycle 4：后续 packet 同段恢复

- 使用上一轮 committed coast seeds 作为当前 pre-entry map。
- 使用同一批 landmark IDs 构造当前 positive-disparity shared count 达
  `min_pnp_inliers`；断言 `kOk`、
  `segment_id` 不变、`unsupported_span_ns == 0`。
- PnP success/fallback 保持现有 fixture 的独立期望；恢复以 primary graph commit 为准。

### Cycle 5：horizon 与 discontinuity cadence

用固定 nanosecond literals 而不是复制实现算法：

- exact horizon + unsupported：`kOk`、有 estimate、摄入计数非零；
- exact horizon 前至少一个 low-support committed packet 满足
  `num_current_visual_factors > 0`，并证明它没有重置 coast duration/span；
- 下一 supported endpoint：原段恢复，无 outage row；
- 对照支路的下一 unsupported endpoint：`kVisualOutage`、无 estimate、三个
  packet counters 为 `0`、`completed_segment_id` 为旧段；
- valid discontinuity：`kDiscontinuity`，window/map/track 清除，span anchor 失效。

### Cycle 6：transaction truth

每个 subject 都建立一个不插入失败的 control，失败后对两者发送同一个后续
合法 packet，比较：

- status/estimate/pose/velocity/bias/segment；
- observation timestamps、window size、prior key、landmark/factor counts；
- coast duration、unsupported span、frame index 效果与 committed diagnostics；
- cull/reopt/eviction/reintegration counters。

覆盖 malformed observation/raw、non-finite propagation、eviction reintegration failure、primary
graph build/solve failure。失败 packet 不能为后续 seed 提供 track age。

### Cycle 7：OfflineVoSession 与 CSV

- 基线 CSV 尾追加
  `unsupported_span_ns,num_retained_observations,num_seeded_landmarks,num_current_visual_factors`。
- 用固定 `VoDiagRow` literals 锁定 header、列数、值顺序和 bool/number 格式。
- `kVisualOutage`/`kDiscontinuity` 写出三个 packet counters 均为 `0` 的 row。
- `kInvalidInput`/`kFailed` 返回结果中的三个 packet counters 均为 `0`，session
  在追加 `diag.csv` row 前终止；`kRejected` 继续。新合同不改该策略。
- 同段恢复不增 `segments` 或 `reanchors`；outage 后新 estimate 才增
  `segments`。

## 6. 定向机制门

### 6.1 Build 与第一层 filters

```bash
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --target phad_estimator_tests phad_apps_tests --parallel 2

./build/phad_estimator_tests \
  --gtest_filter='VioObservationIntake.*:VisualOutageLifecycle.*:KeyframeUpdateTest.*:StereoVoPnpTest.*:StereoVoOutlierCullTest.*:StereoVoOutlierReoptTest.*:VioFullState.*:VioEviction.*'

./build/phad_apps_tests \
  --gtest_filter='OfflineVoSessionTest.*:VoBenchCliTest.*'
```

实际新 fixture 名在第一个 RED 时锁定；若复用既有 suite，同步更新 filter，
不为满足命令而新增空 suite。

### 6.2 第二层 affected binaries

```bash
./build/phad_estimator_tests
./build/phad_apps_tests
git diff --check
```

两个 binary 全绿后才进入候选差异审查。审查项：

- `VioEstimator::update()` 签名与 `VioMeasurement` 不变；
- 不存在新 solver/factor 类型或 apps-side observation cache；
- `min_pnp_inliers`、`min_landmark_observations`、
  `min_track_observations_for_seed`、`min_seed_observations` 与 keyframe 条件未改；
- current packet support 只在 intake 前计算一次；
- actual visual-factor presence 不刷新 support anchor 或 coast budget；
- 新增 state/counters 在 transaction clone/swap 中全覆盖；
- invalid/hard-failure 路径没有 IMU-only 成功 fallback；
- config canonical text/hash 与 `default_0337287b` 一致。

任一机制门失败即停止，不运行 EuRoC。

## 7. Clean candidate 与操作边界

机制门全绿、diff 审查完成后，先向用户报告 task-owned 差异、实际测试与
工作区状态。只在获得明确授权后创建带 `(#43)` 的 clean candidate commit，
例如：

```text
feat(estimator): retain coast observations for visual recovery (#43)
```

本计划不授权 push、merge、rebase、清理无关文件或改写 1a 证据分支。

## 8. EuRoC 产品门

### 8.1 固定命令模板

使用 clean candidate commit 串行运行。每条完成后立即判门，只有 PASS 才将
`<sequence>` 替换为下一条：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence> \
  --bench-root /home/lin/Projects/data/phad-bench \
  --sequence-name <sequence> \
  --repo /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish \
  --estimator-enable-moving-bootstrap
```

机制靶每条随后运行：

```bash
.venv/bin/python scripts/segment_ate_decomp.py \
  /home/lin/Projects/data/phad-bench/<sequence>/<commit7>/default_0337287b \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence>
```

### 8.2 串行顺序与门限

```text
MH_01_easy → V1_03_difficult → V2_02_medium → V2_03_difficult
```

| sequence | ATE 上界 | completion 下界 | coverage 下界 | segments |
|---|---:|---:|---:|---:|
| MH_01_easy | `0.070539 m` | `0.999728` | `0.999728` | `== 1` |
| V1_03_difficult | `1.406996 m` | `0.964635` | `0.974860` | `< 16` |
| V2_02_medium | `1.981976 m` | `0.968484` | `0.972731` | `< 9` |
| V2_03_difficult | `1.707513 m` | `0.506507` | `0.791345` | `< 52` |

MH_01 另要求 `failed=0`、`rejected=0`、`reanchors=0`。三条机制靶各自
还必须满足：

1. 至少一次 `segment_id` 不变的 coast→recovery episode 中，在恢复前或恢复
   packet 上可见 retained/seeded/current-frame-factor 三类计数各自 `>0`；
2. 同一 episode 的 `unsupported_span_ns` 从 `>0` 归零；
3. `failed=0`、`rejected=0`、`reanchors=0`；
4. `segment_ate_decomp.py` 计算的绝对段间分量不高于
   `c999f58/default_0337287b` 对应 baseline；
5. artifact 的 `meta.json` 是 clean candidate identity，`config_hash == 0337287b`，
   `config_canonical_text` 与§8.3 完全一致。

任一 hard failure、机制计数不可观测、segments 未严格下降、ATE 回归、
completion/coverage 回归、绝对段间分量增加或 artifact identity 不匹配，
都立即停止。停止后保留命令、config、artifact 和逐事件诊断，不扫描
horizon、prior sigma 或门限，不运行后续 sequence。

### 8.3 完整 canonical config snapshot

本片没有配置增量键；正式 run 必须保持：

```text
estimator.block_culled_rebirth=true
estimator.enable_moving_bootstrap=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.max_outlier_reopts=3
estimator.min_landmark_observations=2
estimator.min_pnp_inliers=10
estimator.min_seed_observations=10
estimator.min_shared_landmarks=10
estimator.min_track_observations_for_seed=1
estimator.outlier_avg_reproj_px=4
estimator.pnp_confidence=0.98999999999999999
estimator.pnp_reproj_px=2
estimator.prior_rotation_sigma_rad=0.0001
estimator.prior_translation_sigma_m=0.0001
estimator.stereo_sigma_px=1
estimator.velocity_prior_sigma_mps=0.10000000000000001
estimator.window_size=10
eval.max_dt_ms=2.5
eval.min_match_rate=0.5
eval.rpe_delta_s=1
session.dataset_format=euroc
session.drop_culled_tracks=true
session.skip_drop_min_culled=4
session.zombie_drop_age=5
tracker.forward_backward_px=0.5
tracker.lk_pyramid_levels=4
tracker.lk_window_px=21
tracker.mask_radius_px=20
tracker.max_depth_m=25
tracker.max_epipolar_px=1.5
tracker.max_tracks=200
tracker.min_depth_m=0.29999999999999999
tracker.min_disparity_px=2
tracker.min_distance_px=20
tracker.quality_level=0.0030000000000000001
tracker.stereo_bidir_px=0.5
tracker.stereo_check_bidir=true
tracker.stereo_row_tol_px=0
tracker.stereo_sad_half_win_px=7
tracker.stereo_uniq_ratio=0.5
```

CLI-only 参数：无。`--estimator-enable-moving-bootstrap` 已进入 canonical snapshot 与
hash。

## 9. 交付与停线

完成所有获授权步骤后，交付内容应包含：

- task-owned 源码、测试、diagnostics 和 README 差异；
- 实际运行的 filters、完整 affected binaries、`git diff --check` 与结果；
- 按实际停线位置产生的 EuRoC commands、clean commit/config identity、artifacts
  与逐门判定；
- 机制计数、segments、ATE、completion/coverage、加权段内 RMS 和绝对段间
  分量；
- 未执行的后续门、当前限制和剩余风险。

三条机制靶全部 PASS 后，仍先向用户报告并等待是否扩大到剩余七条
EuRoC 与新的 11/11 checkpoint。

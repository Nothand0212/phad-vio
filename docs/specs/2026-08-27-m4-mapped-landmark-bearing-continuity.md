# 已建图 landmark 左目重投影连续性 spec（片 3）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

- 日期：2026-08-27
- 状态：**已定稿**（用户于 2026-08-27 确认方案 A）
- issue：[#47](https://github.com/Nothand0212/phad-vio/issues/47)
  （epic：[#42](https://github.com/Nothand0212/phad-vio/issues/42)）
- 前序合同：[视觉中断期轨迹连续性 spec（片 1b）](2026-08-26-m4-world-frame-continuity.md)
- 方向证据：[M4 下一方向分析](../research/2026-08-26-note-m4-next-direction-analysis.md)
  §D、§E
- 现状证据：Remote CI run
  `20260827T092127Z-ci-euroc11-b837bc9361fb-2fb02b`

## 1. 目标、事实与可证伪问题

本片只回答一个产品问题：

> 对已经拥有世界系 3D 坐标的 landmark，当前帧只有合法左目像素而没有
> 有效右目匹配时，这条 bearing measurement 能否在不改变双目尺度与播种
> 合同的前提下，形成可审计的 pose / landmark 约束，并减少 active local
> estimator 的视觉中断？

已确认的方向是：已建图 landmark 的 zero-disparity observation 使用
GTSAM `GenericProjectionFactor`，进入 PnP、visual support 与
`min_landmark_observations` 路径；双目继续独占新 landmark 的深度与播种。

### 1.1 已观测事实

| 证据 | 结果 | 本片含义 |
|---|---:|---|
| M4 checkpoint 视差产出率中位数 | V2_01 / V2_02 `23.7%`；V2_03 `5.9%` | 当前路径会忽略大量左目 bearing |
| 片 1b record-only run 的 segments | `93 → 30` | observation intake 已减少中断，但未闭合连续性 |
| 同 run 的 V2_03 | ATE `2.081321 m`；segments `18`；段内 RMS `0.062 m` | 剩余误差仍主要来自段间关系，而非段内优化 |
| V2_03 initializing 帧 | `875` 帧；约 `127` 个左目 observation、正视差中位数 `1` | 双目 root seed 仍会饿死；本片只能在 map 尚存时阻止 outage，不能凭左目冷启动 |
| GTSAM 4.3 | 已有 `GenericProjectionFactor<Pose3, Point3, Cal3_S2>` | 不需要新依赖、solver 或自定义 Jacobian |

上述 Remote CI run 的 source identity 为
`sha256:b837bc9361fba359c1efdbd574c317644b5cc33debb76367e7489a44d128c0a3`，
对应 `42f99e9` / tree `76b46a3e`，但 `git_dirty=true`，且 MH_01 超过片 1b
冻结硬门。因此它只提供选方向与预注册门限的证据，不构成产品 PASS。

### 1.2 当前证据缺口

现有 `diag.csv` 只能同时看到：

- `num_disparity`：所有 positive-disparity observations；
- `num_shared`：positive-disparity 且 ID 已在 committed map 中。

它不能直接量出“zero-disparity 且 ID 已在 map 中”的 eligible population。
本片必须先使该 population 可观察；若目标序列上不存在足以改变 support 的
population，则结论为 `INCONCLUSIVE/STOP`，不把 factor 代码进入自然反馈。

## 2. Authority 关系与范围

本文扩展片 1b 中“visual factor 仅为 `GenericStereoFactor`”的局部范围边界，
不推翻其 observation intake、visual coast、segment 或 transaction 合同。

本片不与以下活文档冲突：

- [architecture](../design/architecture.md) 的显式 landmark、GTSAM deep
  module 与 stereo scale 方向保持不变；新增的是已建图 landmark 的 bearing
  residual，不是单目系统或单目尺度初始化。
- [roadmap](../design/roadmap.md) 暂缓的“单目支持与单目尺度初始化”仍暂缓；
  产品输入、root seed 与尺度来源仍是 stereo。
- [ADR-0001](../adr/0001-gtsam-vio-backend.md) 的 GTSAM ownership、显式
  landmark 与 batch graph 决策不变，不需要新 ADR。

以下不变项冻结：

1. 唯一 public measurement Interface / test Seam 仍为
   `VioEstimator::update(const VioMeasurement&, bool)`；只读辅助 seam 仍为
   `observationTimestamps()`。
2. GTSAM calibration、noise、factor、graph 与 lifecycle 继续藏在
   `VioEstimator` PIMPL 内；不新增 public Optimizer、factor factory 或
   modality Adapter。
3. `StereoObservation` 的表示不变：`disparity_px > 0` 是 stereo，
   `== 0` 是合法 left-only，`< 0` 是 invalid input。
4. `min_pnp_inliers`、`min_landmark_observations`、
   `min_track_observations_for_seed`、`min_seed_observations`、
   `stereo_sigma_px`、`huber_k_px` 及其默认值不变。
5. 新 landmark 仍只可由 positive-disparity observation backproject；
   left-only observation 不 seed、不 refresh 3D、不参加 root seed count。
6. window size、keyframe policy、IMU factor、bias-RW factor、root priors、
   solver、eviction/reintegration、hanging gate、segment 与 discontinuity
   cadence 不变。
7. 不引入 cross-segment bridge、relocalization、loop closure、global map、
   M5 initialization 或 threshold/noise 扫描。

## 3. 术语与单一分类规则

分类只读取通过输入校验的当前 observation 与**摄入前 committed map**：

| 名称 | 精确定义 | 能力 |
|---|---|---|
| stereo observation | `disparity_px > 0` | 左/右重投影；可提供深度并 seed |
| left-only observation | `disparity_px == 0` | 只有左目 bearing；不能提供新深度 |
| mapped observation | observation ID 在摄入前 `m_landmarks_w` 中 | 可形成 3D–2D correspondence |
| `num_shared` | mapped **且 stereo** 的当前 observation 数 | 保留历史 stereo-overlap 诊断语义 |
| `num_mapped_observations` | mapped observation 总数，不区分是否有 disparity | 本片的 PnP / support population |
| eligible mono factor | left-only，且 staged map 中已有该 ID，且该 landmark 的 window observation count 达门 | 可附着单目 factor |

同一 observation 只能走一条 factor 路径：

```text
disparity_px > 0  -> GenericStereoFactor
disparity_px == 0 -> GenericProjectionFactor（仅 mapped / 已 seed ID）
```

positive-disparity observation 不额外建立左目 factor，避免同一个左像素被双重
计权。zero-disparity 的 unmapped ID 只保留 observation / track timestamp，
等待以后某个 keyframe 上 stereo 回归并成功 seed。

## 4. Deep Module 边界与数据流

`VioEstimator` 继续作为 deep Module：调用方只提交 project-owned
`VioMeasurement`，不知道 GTSAM factor variant。modality selection、PnP
correspondence、factor admission 与 mixed-factor quality scoring 都是 PIMPL
内部实现细节，共享同一个 map/window/transaction owner。

每个 raw-IMU continuous active-segment packet 按以下顺序处理：

1. 校验 timestamp、IMU interval、left pixel 与 disparity。
2. 在摄入前 committed map 上一次性统计 `num_disparity`、`num_shared` 与
   `num_mapped_observations`；本 packet 后续 seed 不回流这些计数。
3. `full_visual_support := num_mapped_observations >= min_pnp_inliers`。
4. 若启用 PnP 且 mapped count 达门，用全部 mapped observations 建立
   3D–2D correspondence，并执行 §5 的 modality-aware 仲裁。
5. accepted PnP mask 只从当前 `WindowFrame` 删除 mapped outliers；stereo 与
   left-only 一视同仁。原始 validated measurement 仍完整写入
   `m_track_times`，沿用既有 mask 语义。
6. keyframe seeding 继续只处理 positive-disparity new IDs；left-only new IDs
   不 backproject。
7. 当前 frame 进入 window 后，以 staged map 为准统计每个 landmark 的全部
   factor-capable window observations；达到既有
   `min_landmark_observations` 才把 landmark 与其 observations 入图。
8. `buildGraph()` 按 §6 为每条 eligible observation 建立恰好一个 visual
   factor，运行 primary LM、cheirality/mean-cull 与 optional reopt。
9. 只有整个 primary transaction 成功，mapped support 才刷新
   last-support anchor、归零 visual coast / `unsupported_span_ns` 并提交。

`low_connectivity` 的 overlap 输入同步改为 `num_mapped_observations`；它不再
因缺少右目匹配而把有充分 3D–2D bearing support 的 packet 标成低连接。

## 5. PnP correspondence、仲裁与 mask

### 5.1 Correspondence

`tryPnpInit()` 对每个 mapped observation 使用：

```text
object point = committed m_landmarks_w[id]
image point  = observation.left_pixel
```

是否有当前 stereo disparity 不影响 PnP 的 3D–2D 几何资格；landmark 深度来自
既有 map。`solvePnPRansac`、iteration count、`pnp_reproj_px`、confidence 与
`min_pnp_inliers` 不变。

### 5.2 Modality-aware acceptance score

PnP 仍只是 proposal，不直接授权 pose 或 mask。proposal 与 propagation guess
都在 PnP 返回的同一 inlier index 集上计算未白化 residual：

- stereo observation：现有 `(uL,uR,v)` residual 的 L2 norm；
- left-only observation：`(u,v)` residual 的 L2 norm；
- mixed score：
  `sqrt(sum(squaredNorm(residual_i)) / number_of_inlier_observations)`。

该定义在全部 observation 都是 stereo 时与现有 `stereoRmsAtPose()` 数值完全
一致；不按 residual scalar 维数重归一化。任一 index 非法、landmark / pose
非有限、投影在相机后方或 residual 非有限，令该 candidate score 无效。

acceptance band 继续使用 `stereo_sigma_px`：proposal score 不高于 guess
score 加一个 sigma 才接受；proposal score 无效时 fallback；guess score 无效
但 proposal 有效时可接受。fallback 保留 propagation guess 与全部当前
observations，不应用 PnP mask，也不拒帧。

### 5.3 Inlier mask

accepted proposal 的 inlier indices 映射到同一次 correspondence scan 的
mapped IDs。mapped stereo 与 mapped left-only outlier 都从当前
`WindowFrame::m_observations` 删除；unmapped IDs 保留。diagnostics 的
`pnp_inliers` 继续表示所有 modality 的 RANSAC inlier 总数。

## 6. Observation count 与 factor admission

`min_landmark_observations` 的 count 改为 window 内该 ID 的 factor-capable
observation 总数：

- staged map 已有该 ID 时，stereo 与 left-only 都计数；
- staged map 没有该 ID 时，不会仅凭 left-only count 插入 landmark value；
- 某个 new ID 后来由 stereo 成功 seed 后，窗口中更早保留的 left-only
  observations 可在同一次 graph rebuild 中取得 factor 资格；
- cull 后从 window 删除的 observations 不再计数，`block_culled_rebirth`
  语义不变。

达到 count 门后：

### 6.1 Stereo factor

positive-disparity observation 沿用：

```cpp
gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
```

measurement、`Cal3_S2Stereo`、`body_P_sensor` 与现有 3D robust noise 不变。

### 6.2 Monocular projection factor

left-only mapped observation 使用：

```cpp
gtsam::GenericProjectionFactor<
    gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
```

- measurement：`gtsam::Point2(left_pixel.x(), left_pixel.y())`；
- calibration：由既有 `Cal3_S2Stereo::calibration()` 构造并由 PIMPL 共享拥有；
- extrinsic：与 stereo factor 相同的 `body_P_sensor`；
- noise：2D `Isotropic::Sigma(2, stereo_sigma_px)`，按既有 `huber_k_px`
  使用相同 Huber wrapper；
- cheirality flags：沿用 GTSAM 默认 `false/false`，由 §7 的统一质量路径处理。

不新增 mono sigma、权重 ratio、开关或 canonical-config key。

## 7. Mixed-factor 质量、cull 与 reopt

新增 factor 取得改变 posterior 的权限时，也必须进入现有质量闭环，不能只
优化而不被诊断或剔除。

### 7.1 Reprojection diagnostics

现有 `reproj_rms_before_px`、`reproj_rms_after_px` 与
`reproj_rms_after_cull_px` 的语义扩展为所有 visual projection factors：

```text
sqrt(sum_f ||unwhitenedError_f||^2 / number_of_visual_factors)
```

stereo-only graph 的数值保持不变；mixed graph 不按 2D/3D residual 维数重新
归一化。skipping-missing-landmark 与 reopt graph 使用相同 factor visitor。

### 7.2 Cheirality

- `GenericStereoFactor` 的 3D `2*fx` sentinel 与
  `GenericProjectionFactor` 的 2D `2*fx` sentinel 都计入
  `num_cheirality`。
- `countBehindCameraLandmarks()` 与 `dropCheiralityLandmarks()` 对实际入图的
  stereo / left-only observations 使用同一 left-camera z test。
- 被判定 behind-camera 的 landmark 沿用现有全窗口 erase、culled-ID 与
  optional reopt 语义。

### 7.3 Mean-cull

per-landmark mean score 同时遍历两类 factor；每条 factor 贡献一次
`unwhitenedError().norm()`。既有 `n_factors >= 4`、
`outlier_avg_reproj_px`、cull/rebirth、multi-round reopt trigger 与计数不变。

mono factor 导致 primary graph build/solve 或有限性检查失败时，返回
`kFailed` 并回滚整个 packet；不得静默删除 mono factors 后以 stereo-only 或
IMU-only graph 报告 `kOk`。optional reopt round 的局部失败语义保持现有合同。

## 8. Support、coast 与 segment 状态机

片 1b 的状态机只替换 support population，不改变时序：

| 当前 packet | 行为 |
|---|---|
| active，`num_mapped_observations >= min_pnp_inliers` | normal / same-segment recovery；成功 commit 后刷新 support anchor 并归零 coast/span |
| active，mapped count 不足，累加后 coast `<= horizon` | low-support coast；mono/stereo factors 可继续约束 graph，但不刷新 support |
| active，mapped count 不足且下一 interval 使 coast `> horizon` | 当前 packet 不摄入，单次 `kVisualOutage` 完成 active segment |
| uninitialized | root bootstrap/seed 仍只看 positive disparity；mapped count 自然为 `0` |
| valid discontinuity | 既有硬切段；清 support anchor/span |

support 仍在当前 packet 摄入和 seed 前锁定；同 packet 新 seed 不得把 coast
改判为 recovery。PnP failure 不改写 support classification；只有后续 primary
graph transaction 成功才允许提交 recovery。

## 9. Diagnostics 与 artifact schema

### 9.1 保留语义

- `num_disparity`：当前 validated measurement 中 `disparity_px > 0` 的数量；
- `num_shared`：其中 ID 已在摄入前 committed map 中的数量；
- `num_retained_observations`：PnP mask 后当前 frame 最终保留数；
- `pnp_inliers`：accepted mixed-modality PnP inlier 总数。

保留 `num_shared` 的 stereo-only 语义，确保历史 `num_disparity` / `num_shared`
诊断仍可区分“右目匹配稀缺”与“stereo map overlap 稀缺”。

### 9.2 新字段与扩展语义

在 `UpdateDiagnostics` 与 `diag.csv` 尾部依次追加：

| 字段 | 精确语义 |
|---|---|
| `num_mapped_observations` | 当前 packet 摄入前，所有 ID 已在 committed map 的 validated observations 数；含 stereo 与 left-only |
| `num_current_mono_visual_factors` | 产生最终 committed optimized state 的最后一次成功 solve graph 中，连接当前 frame 的 `GenericProjectionFactor` 数 |

`num_mapped_observations - num_shared` 即当前 eligible left-only mapped
population。没有当前 committed frame，或 hard failure / outage / discontinuity
未产生 solve 时，`num_current_mono_visual_factors == 0`。

既有字段作兼容扩展：

- `num_current_visual_factors`：当前 frame 的 stereo + mono visual factor
  总数；没有 mono 时与旧值完全相同；
- `VioDiagnostics::m_visual_factors`：最后成功 graph 的 stereo + mono visual
  factor 总数。

因此当前 stereo factor 数可由
`num_current_visual_factors - num_current_mono_visual_factors` 精确得到，不再
追加冗余字段。`diag.csv` 从 24 列扩为 26 列；其余列顺序不变。

## 10. Transaction 与失败语义

除输入校验与摄入前只读计数外，以下状态继续由同一个
`VioUpdateTransaction` 原子拥有：

- PnP mask 后 observations、track timestamps 与 candidate pose；
- stereo-only seeding、landmark count/admission 与 mixed factor graph；
- visual coast、last-support anchor、`unsupported_span_ns`、window、map；
- primary solve、cull、reopt、factor diagnostics 与 final `VioEstimate`。

`kInvalidInput` 不创建 factor、不推进任何状态。primary propagation、graph
build/solve 或 optimized values 非有限时，全量 rollback；失败 diagnostics 从
rollback 后 committed state 重建，新 current-factor 字段为 `0`。随后相同合法
input 的 subject/control 行为必须一致。

## 11. Evidence-gated 实施权限

最终合同不等于一步取得全部权限。后续 plan 必须按以下顺序拆分，每一门失败
立即 STOP：

### Q1 Observe：eligible population

只追加 `num_mapped_observations` 与 CSV wiring，不改变 support、PnP、factor、
pose、status 或 lifecycle。

PASS 同时要求：

1. 合成 public-seam 测试精确区分 stereo mapped、left-only mapped、unmapped；
2. 除新增 CSV 列外，当前 control 的 trajectory、status/cadence、既有
   diagnostics 与 summary bit-equivalent；
3. 在 `V2_02_medium` 或 `V2_03_difficult` 至少出现一个 active-segment
   episode，存在
   `num_shared < min_pnp_inliers <= num_mapped_observations`；
4. artifact 报告 eligible 帧数、episode 数与距下一次 recovery/outage 的位置。

不存在第 3 项 population 时结论为 `INCONCLUSIVE/STOP`；不实现 Q4/Q5。

### Q4 Constrain：factor 与质量机制

在 synthetic/frozen input 上授权 left-only mapped observation 进入 posterior；
support 与 PnP population 仍保持 stereo-only，避免同时改变 liveness decision。

PASS 必须证明：factor eligible/attached count、public estimate 对解析 2D
projection / GTSAM reference graph 的 known-motion oracle、stereo
no-double-count、mixed observation-count admission、mixed RMS、cheirality、
mean-cull/reopt 与 rollback 全部成立。positive-disparity-only control 的
estimate / diagnostics 保持既有容差或 bit-equivalent 合同。

### Q5 Feedback：PnP 与 support

先让 mixed correspondence 进入 PnP 及其 modality-aware 仲裁，再让
`num_mapped_observations` 取得 support/liveness decision 权限。plan 必须把二者
做成独立 Red→Green cycle；只有前一机制门通过才进入后一 cycle。

最终 natural replay 必须证明：mapped left-only observation 实际产生 mono
factor；存在 `num_shared < min_pnp_inliers <= num_mapped_observations` 的同段
recovery；segments/ATE 改善不能在 mono factor count 为零时归因给本机制。

本片不新增运行时开关；资格阶段按短分支上的顺序 commit 与 stop gate 控制
权限，未通过的后续 commit 不进入最终 product branch。

## 12. TDD 与机制验收

所有长期测试只通过 public Seam 观察行为，不访问 `Impl`、`WindowFrame` 或
private graph helpers。

Red→Green cases 至少覆盖：

1. classification：stereo mapped / left-only mapped / unmapped 的三个计数；
2. stereo control：positive-disparity mapped observation 只建一个 stereo
   factor，mono count 为 `0`；
3. mono factor：已建图 left-only observation 达 count 门后只建一个
   `GenericProjectionFactor`，estimate 受其约束；
4. no seed：unmapped left-only observation retained，但 seed/factor 均为 `0`；
5. delayed eligibility：旧 left-only observations 在同 ID 后续 stereo seed
   后进入 mixed graph；
6. mixed admission：一条 stereo + 一条 left-only 达到既有 observation count
   门，两条 factor 均存在；
7. PnP：纯 left-only mapped correspondence 可成功；mixed score 接受/拒绝边界
   正确；accepted mask 删除两种 mapped outlier、保留 unmapped ID 与完整 track
   history；
8. support：`num_shared == 0` 但 mapped count 达门时可在原 segment 恢复；
   同 packet seed 不回流 support；
9. quality：mono cheirality、四 factor mean-cull、successful reopt 与 failed
   reopt rollback；
10. primary hard failure：mono-bearing graph 失败后 window/map/track/coast/span/
    segment/factor diagnostics 回滚，后续合法调用与 fresh control 一致；
11. `OfflineVoSession`：26 列 schema、字段顺序、outage/discontinuity/failed
    zero-count cadence。

机制执行顺序：新 filters → 相关 PnP/coast/cull/reopt/rollback filters → 完整
`phad_estimator_tests` → 完整 `phad_apps_tests` → `git diff --check` 与完整 diff
审计。

## 13. Q0 control 与 EuRoC stop-on-failure 产品门

### 13.1 Q0

正式实现前先在 clean `42f99e9`、canonical `default_0337287b` 上串行复跑
`MH_01_easy → V1_03_difficult → V2_02_medium → V2_03_difficult`，冻结每条的
ATE、RPE、completion、coverage、segments、段内 RMS、段间分量与完整
`meta.json`。若 clean Q0 与现有 record-only run 的差异无法由 source manifest
解释，先修 control，不进入 Q1。

现有 run 仅预注册以下上界/下界；clean Q0 只能收紧，不能在看到 candidate
后放宽：

| sequence | ATE 上界 | RPE 上界 | completion 下界 | coverage 下界 | segments |
|---|---:|---:|---:|---:|---:|
| MH_01_easy | `0.070539` | Q0 值 | `0.999728` | `0.999728` | `1` |
| V1_03_difficult | `0.861563` | `0.130757` | `0.974407` | `0.974860` | `<= 2` |
| V2_02_medium | `0.171546` | `0.057113` | `0.974020` | `0.974009` | `1` |
| V2_03_difficult | `1.707513` | `0.688425` | `0.535138` | `0.791345` | `< 18` |

MH_01 ATE 保留 M4 checkpoint 的冻结硬门；不能用片 1b dirty run 的
`0.072559` 放宽。V2_03 必须同时收回相对 checkpoint 的 ATE 回归并减少当前
18 段；只改善其一不构成产品 PASS。

### 13.2 串行门

正式 candidate 严格按表中顺序运行，任一失败立即停止，不运行后续序列、
不扫 sigma/Huber/门限、不扩大 visual-coast horizon。

每条除表格门外还必须满足：

1. `failed == 0`、`rejected == 0`、`reanchors == 0`；
2. 四条 artifact 都报告 eligible/attached 分布；V2_03 必须实际出现
   `num_mapped_observations - num_shared > 0` 与
   `num_current_mono_visual_factors > 0`；
3. V2_03 至少有一次 flip-capable episode：
   `num_shared < min_pnp_inliers <= num_mapped_observations`，并在同一
   `segment_id` 中成功提交；V1_03 / V2_02 记录该 episode 数但不设非零硬门；
4. `num_current_visual_factors >= num_current_mono_visual_factors` 每帧成立；
5. 用 `scripts/segment_ate_decomp.py` 比较段内 RMS 与绝对段间分量；V2_03
   的段内 RMS 不高于 clean Q0，段间分量严格下降；
6. artifact 保留 clean code/tree identity、canonical config、命令、逐帧
   diagnostics 与 Q1/Q4/Q5 机制计数。

通过四条串行门只授权本行为进入当前 stereo VIO 默认路径，不授权完整单目
输入、单目初始化、跨段 relocalization 或下一里程碑。

## 14. 已确认决策

以下组合构成 plan 与实现的冻结输入：

1. 保留 `num_shared` 的 stereo-only 历史语义，新增
   `num_mapped_observations` 作为 PnP/support population；
2. `num_current_visual_factors` 扩展为两类 visual factor 总数，并新增一个
   mono breakdown 字段；
3. mixed PnP / RMS / mean-cull 按“每 observation / factor 一个 L2 norm”统计，
   使 stereo-only 数值不变；
4. 按 Q1 Observe → Q4 Constrain → Q5 PnP/support 的证据门逐步扩权；
5. 四条 EuRoC 串行门采用 §13 的预注册数值。

以上五项已于 2026-08-27 确认。

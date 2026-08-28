---
name: M4 已建图 landmark 左目重投影连续性
overview: 在既有 VioEstimator::update() seam 内，先在同一台本机补齐 clean Q0 的 core-4/EuRoC-11 并冻结多序列产品 envelope，再按 Q1 Observe、Q4 Constrain、Q5 PnP/support 的证据门，让已建图 landmark 的 left-only bearing 进入诊断、posterior 与连续性反馈；双目继续独占深度、播种与尺度。完成机制回归后运行一次 matched clean candidate 本机实验，与 Q0 全量对拍并据结果确定下一工作方向。
todos:
  - id: isolate-control-and-workspace
    content: '记录 42f99e9、default_0337287b 与现有 dirty paths；在独立 clean worktree 运行 Q0，实施分支只显式暂存 #47 文件'
    status: completed
  - id: complete-q0-core4
    content: '补跑 clean Q0 的 V1_03→V2_02→V2_03；无 accuracy early-stop，冻结 core-4 identity、meta、逐序列精度、coverage 与段内/段间指标'
    status: completed
  - id: complete-q0-euroc11
    content: '在本机从 clean 42f99e9 串行补齐 EuRoC-11 control，并核对 11 条 source/toolchain/config/artifact'
    status: completed
  - id: freeze-product-envelope
    content: '在任何 Q1/Q4/Q5 candidate 结果前计算 core-4/EuRoC-11 的 gate 基线，确认并冻结 G_ATE/G_RPE、有效性、tail review 与 continuity 数值规则'
    status: completed
  - id: q1-red-classification
    content: '先经 public seam 写 stereo mapped、left-only mapped、unmapped 三分类红灯及 CSV 第 25 列红灯'
    status: in_progress
  - id: q1-green-observe
    content: '最小接入 num_mapped_observations 与 OfflineVoSession/diag.csv，不改变 PnP、support、factor、pose 或 lifecycle'
    status: pending
  - id: q1-natural-eligibility-gate
    content: '在 clean Q1 snapshot 上量出 V2_02/V2_03 flip-capable population；对照 Q0 证明既有输出等价，无 population 则 STOP'
    status: pending
  - id: q4-red-green-factor-admission
    content: '逐个完成 stereo 单因子、mapped mono、unmapped no-seed、delayed eligibility 与 mixed count admission 的 Red→Green cycle'
    status: pending
  - id: q4-red-green-quality
    content: '统一 mixed visual factor 的 RMS、cheirality、mean-cull、reopt、probe 与 current-factor diagnostics，并覆盖 primary rollback'
    status: pending
  - id: q5a-red-green-mixed-pnp
    content: '先锁定纯 left-only 与 mixed correspondence、modality-aware 仲裁及 mask，再让全部 mapped observations 进入 PnP'
    status: pending
  - id: q5b-red-green-mapped-support
    content: '先锁定 num_shared 为零的同段恢复与同 packet seed 隔离，再将 support、coast anchor 与 low_connectivity 切到 mapped population'
    status: pending
  - id: finalize-schema-and-docs
    content: '完成 26 列 CSV、total/mono factor 语义、estimator/apps/bench 文档和 #47 证据同步'
    status: pending
  - id: run-mechanism-regressions
    content: '按新 filters→相关 lifecycle/cull/reopt/rollback filters→完整 estimator/apps suites 的顺序验证并审计差异'
    status: pending
  - id: freeze-clean-candidate
    content: '在用户授权下形成带 #47 的分阶段 local commits 和 clean candidate identity；不 push/merge'
    status: pending
  - id: run-local-candidate-experiment
    content: '从 clean candidate commit 在同一台本机运行 EuRoC-11，核对身份并与 matched clean Q0 做 11 条多序列产品判定'
    status: pending
  - id: analyze-next-direction
    content: '按机制生效、suite-level outcome、逐序列 tail、段内/段间误差与 initializing/outage 分布形成结论和下一 vertical slice 建议'
    status: pending
isProject: false
---

# M4 已建图 landmark 左目重投影连续性

## 状态

**Spec、实施计划、clean Q0 core-4/EuRoC-11 与完整数值 envelope 已定稿；Stage 0
完成，现按 Q1 classification Red→Green 开始实施。尚未取得 Q4/Q5 或 candidate
experiment 的执行资格。**

已完成的 control 事实：clean `42f99e9/default_0337287b` unit 共 `458/458`
通过（另有 3 个既有 skip）；EuRoC-11 为 11/11、55/55 required artifacts，统一
数据质量审计 PASS，`failed/rejected/reanchors == 0`。相对 M4 checkpoint，
`G_ATE=0.696757`、`G_RPE=0.645531`，ATE/RPE 算术均值分别为
`0.380628 / 0.113120 m`，segments 为 `93 → 30`。完整 identity、精度、coverage、
段内/段间分解、hash 与已冻结 envelope 见
[`mapped-landmark-bearing Q0`](../benchmark/m4/mapped-landmark-bearing-q0_42f99e9_0337287b.md)。

本计划把一个 deep Module 内部能力分四个可证伪阶段交付：Q1 只观察 eligible
population，Q4 只让 mapped left-only bearing 约束 posterior，Q5a 再扩展 PnP，
Q5b 最后扩展 support/liveness。每一阶段的前一门必须实际 PASS，后一阶段才取得
实施权限。

## 1. Authority、实施身份与工作区边界

按具体性从高到低使用：

1. [`mapped-landmark-bearing-continuity spec`](../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md)。
2. [`world-frame-continuity spec`](../specs/2026-08-26-m4-world-frame-continuity.md)。
3. [`minimal-full-state VIO spec`](../specs/2026-08-25-m4-minimal-full-state-vio.md)。
4. [#47](https://github.com/Nothand0212/phad-vio/issues/47) 与 epic
   [#42](https://github.com/Nothand0212/phad-vio/issues/42)。
5. [`evidence-gated-integration.md`](../agents/evidence-gated-integration.md)、
   [`incremental-development.md`](../agents/incremental-development.md) 与仓库测试/命名约定。

| 项 | 值 |
|---|---|
| 实施/Q0 基线 | `42f99e9`，tree/source identity 由 Q0 artifact 再冻结 |
| 行为 checkpoint | `c999f58` / `default_0337287b` |
| 当前 runtime continuity | `bf8c321` 已包含在 `42f99e9` |
| canonical config | `default_0337287b`，本片不增加 key |
| 实施分支 | `codex/m4-mapped-landmark-bearing-continuity`（已创建） |
| issue | `#47`；所有授权 commit subject 带 `(#47)` |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| artifact root | `/home/lin/Projects/data/phad-bench` |

当前工作树已有 `scripts/remote_ci.py`、`tests/scripts/test_remote_ci.py` 的独立
修改。实施不覆盖、不整理、不顺带提交这两个 path。开工时先记录
`git status --short --branch`，再采用以下隔离方式：

1. Q0 从 `42f99e9` 建立独立 clean worktree，避免当前文档和既有 dirty paths
   进入 control source manifest。
2. #47 在当前工作树的短分支上实施，只按显式 pathspec 暂存本计划列出的文件。
3. 每个需要自然 replay 的阶段，从该阶段授权 commit 建立独立 clean worktree；
   Q0 与 candidate 使用同一台本机、相同 Release toolchain/config/data。
4. 分支切换或显式暂存若与既有 dirty paths 发生重叠，立即停止，不 stash、reset、
   clean 或改写其内容。

已授权的文档证据使用 local commits 固化；剩余 production/test commits 只在本
计划确认后按 §4.2 逐门形成。全程不 push、merge 或修改 shared/prod 配置。

## 2. Deep Module、Interface 与 Seam

| 术语 | 本片结论 |
|---|---|
| Module | `VioEstimator` 继续独占 map/window、GTSAM calibration/noise/factor、PnP、quality 与 transaction |
| Interface | 唯一 measurement Interface 保持 `VioEstimator::update(const VioMeasurement&, bool)` |
| test Seam | 机制测试只读 `VioUpdateResult`、`UpdateDiagnostics`、`VioEstimate` 与既有 `observationTimestamps()` |
| app Seam | `OfflineVoSession` 只消费公开 result，写 `VoDiagRow`/`diag.csv`；不知道 factor variant |
| 内部 seam | anonymous namespace 的 mixed visual-factor visitor 与 PIMPL helpers；不向 header 暴露 GTSAM |

`GenericProjectionFactor` 是 `VioEstimator` 深模块内部的新能力，不形成新的 public
factor factory、optimizer、modality Adapter 或测试后门。测试可独立构造 GTSAM
reference graph 作为 oracle，但不得读取 `Impl`、`WindowFrame` 或 private graph。

冻结的数据流为：

```text
validated current observations + pre-intake committed map
  ├─ positive disparity ──> stereo mapped count ──> GenericStereoFactor
  ├─ zero disparity + mapped ─> mapped count ─────> GenericProjectionFactor
  └─ zero disparity + unmapped ─> lifetime only; later stereo may seed

mixed attached factors
  ─> primary LM ─> RMS/cheirality/mean-cull ─> optional reopt ─> transaction commit

mixed mapped population
  ─> Q5a PnP proposal/mask ─> Q5b support/coast/segment decision
```

每条 observation 最多建立一个 visual factor；left-only observation 不提供深度、
不触发 backprojection，也不改变 root/new-landmark seed 门。

## 3. 代码地图与预计修改

### 3.1 生产代码

| 文件/位置 | 计划修改 |
|---|---|
| `phad/estimator/types.hpp` | 在 `UpdateDiagnostics` 追加 `num_mapped_observations`、`num_current_mono_visual_factors`；保持 `num_shared` 语义；现有 total 字段扩为 mixed total |
| `phad/estimator/vio_estimator.cpp` includes / `VioGraphInfo` | 引入 `ProjectionFactor.h`；为 mono calibration/noise 和 current mono count 建立 PIMPL-owned state |
| `vio_estimator.cpp` visual helpers | 把 `stereoReprojRms*`、`countCheiralityFactors`、`fillProbeLandmarkResiduals` 和 mean-cull 的 type branch 收敛到一个 private mixed-factor visitor |
| `Impl::tryPnpInit()` / pose score | 扫描全部 mapped observations；按 observation modality 计算 2D/3D residual L2 norm；保持 all-stereo 数值 |
| `Impl::countObservations()` / `buildGraph()` | count staged-map 中全部 factor-capable observations；按 disparity 分类且只附着一种 factor；记录 total/mono/current counts |
| `dropCheiralityLandmarks()` / `countBehindCameraLandmarks()` | 只让实际 eligible 的两类 visual observation 参与统一 left-camera z 检查 |
| `VioEstimator::update()` entry/support/mask/final diagnostics | 一次性统计三类 population；Q5 前后按门切换 PnP/support；mapped inlier mask 同时覆盖 stereo/left-only；commit 最后发布 counters |
| `apps/offline_vo_session.hpp/.cpp` | `VoDiagRow` 映射两个新字段；Q5a 后 fallback population 跟随 mapped；CSV 尾部按冻结顺序追加两列 |

`Cal3_S2Stereo::calibration()` 在当前 GTSAM 4.3 header 中返回左目
`const Cal3_S2&`；PIMPL 用它构造共享 `Cal3_S2`。mono noise 使用
`Isotropic::Sigma(2, stereo_sigma_px)`，并复用既有 Huber 开关与 `huber_k_px`。
不修改 dependency、solver、config parser、canonical config 或 CMake target 依赖。

### 3.2 测试与文档

| 文件 | 计划修改 |
|---|---|
| `tests/estimator/mapped_landmark_bearing_test.cpp`（新） | classification、factor/no-double-count、no-seed、delayed eligibility、mixed admission、known-motion oracle |
| `CMakeLists.txt` | 仅把新测试源接入既有 `phad_estimator_tests` target |
| `tests/estimator/stereo_vo_pnp_test.cpp` | 纯 left-only/mixed PnP、score acceptance/fallback、mapped mask 与 track-history 合同 |
| `tests/estimator/stereo_vo_outlier_cull_test.cpp` | mono cheirality、四 mixed factors 的 mean-cull |
| `tests/estimator/stereo_vo_outlier_reopt_test.cpp` | mixed successful reopt 与 failed-round local rollback |
| `tests/estimator/vio_full_state_test.cpp` | support flip、同 packet seed 隔离、primary hard rollback 与 fresh control 对拍 |
| `tests/estimator/stereo_vo_diagnostics_test.cpp` | total/mono factor breakdown、stereo-only diagnostics 与 lifecycle cadence |
| `tests/apps/offline_vo_session_test.cpp` | Q1 25 列中间合同、最终 26 列 schema/value、outage/discontinuity/failed zero cadence |
| `phad/estimator/README.md` | mapped bearing admission、mixed quality、support 与 diagnostics 当前合同 |
| `apps/AGENTS.md` 或 `apps` 当前 README | 只在现有权威位置同步 `VoDiagRow`/CSV 语义 |
| `phad/bench/README.md`、`docs/benchmark/m4/` | 实验 identity、机制计数、门控结果与复现命令 |

实现时先复核每个目录的 `AGENTS.md`；若没有实际行为或文档语义变化，不为补齐
清单而修改文件。

## 4. Stage 0：control、身份与分支

### 4.1 Q0 clean control

在 detached clean `42f99e9` worktree 中完成两层 control：

1. 复核 `git rev-parse HEAD^{tree}`、`git status --porcelain=v1`、config canonical
   text/hash、compiler/GTSAM/OpenCV identity 与实际命令。当前工作树的文档与
   `scripts/remote_ci.py`、`tests/scripts/test_remote_ci.py` 不进入 snapshot。
2. 已完成 Release + tests，并按
   `MH_01_easy → V1_03_difficult → V2_02_medium → V2_03_difficult`
   补齐 core-4。
3. accuracy、completion、coverage 或 segments 的单项数值不终止 core-4；单条
   运行失败也保留 artifact 并在可执行时继续。只有 source/config/input/evaluator
   身份或 artifact schema 系统性失效才停止 suite。
4. 每条冻结 ATE/RPE、completion/coverage、segments/reanchors、段内加权 RMS、
   绝对段间分量、status/cadence 与完整 `meta.json`。
5. 已从同一 clean `42f99e9` worktree 与 Release build 在本机串行补齐
   EuRoC-11 Q0 control；每条保留独立 artifact，并核对 source/tree、toolchain、
   data、config。11/11、55/55 required artifacts 的统一数据质量审计已通过。
6. 已用 11 条 `summary.json` 计算逐序列 ratio 表并补齐
   `mapped-landmark-bearing-q0_42f99e9_0337287b.md`；结合 M4 checkpoint 与绝对
   工程量级形成 `G_ATE/G_RPE`、completion/coverage、tail review 与 V2_03
   continuity 数值提案，已于 2026-08-28 经用户确认并写回 spec/benchmark 冻结。
7. clean Q0 与既有 record-only run 若存在 manifest 不能解释的差异，先解决
   reproducibility；在多序列 envelope 冻结前不进入 Q1。

Q0 的 EuRoC-11 是 control 测量；最终只运行一次 clean **candidate** EuRoC-11。
两次 run 使用同一台本机、相同 Release toolchain/config/data，使 source commit
成为主要实验变量。最终 `core-4` 产品 ratio 取两次 matched 本机实验中对应四条
的子集，不跨机器或工具链计算 ratio。

### 4.2 授权后的分支与 logical commits

#47 短分支与 spec/Q0/gate 文档 checkpoints 已存在。计划确认后的剩余 logical
commits 为：

1. `feat(estimator): observe mapped landmark population (#47)` — Q1
2. `feat(estimator): constrain mapped landmark bearings (#47)` — Q4
3. `feat(estimator): use mapped observations for pnp (#47)` — Q5a
4. `feat(estimator): use mapped observations for support (#47)` — Q5b
5. `docs(benchmark): record mapped bearing experiment (#47)` — 最终证据

每个机制 commit 必须处于对应 GREEN 且门控通过的状态。RED 证据记录在执行日志/
issue comment，不提交不可用代码。每次 commit 用显式 pathspec 审核 staged diff；不
包含当前 `scripts/remote_ci.py` 与 `tests/scripts/test_remote_ci.py` 修改。

## 5. Q1 Observe：先量 eligible population

### 5.1 RED

通过 `VioEstimator::update()` 构造 root 后的当前 packet，分别包含：

- mapped positive-disparity observation；
- mapped zero-disparity observation；
- unmapped positive-disparity observation；
- unmapped zero-disparity observation。

红灯精确锁定：

- `num_disparity` 只数两条 positive-disparity；
- `num_shared` 只数 mapped positive-disparity；
- `num_mapped_observations` 数两条 mapped，不受 disparity 影响；
- 本阶段 estimate、support、PnP、factor totals 与 lifecycle 保持 control 行为。

`OfflineVoSessionTest` 同步先锁定第 25 列
`num_mapped_observations` 位于原 24 列尾部，三类没有 committed current frame 的
row 写 `0`。

### 5.2 GREEN

在 `VioEstimator::update()` 已完成输入校验、尚未摄入/seed 前，用 committed
`m_landmarks_w` 单次扫描得到 `num_disparity`、`num_shared` 与
`num_mapped_observations`。此 commit 只发布新 diagnostic，并映射到
`VoDiagRow`/CSV；以下 predicate 仍读取 `num_shared`：

- `full_visual_support` / coast / support anchor；
- `low_connectivity`；
- PnP entry 与 mask；
- `countObservations()` 与 factor construction。

### 5.3 Q1 验证与自然资格门

1. 运行新 classification/CSV filters 和完整 estimator/apps tests。
2. 从 clean Q1 commit 生成 artifact；逐 row 比较 Q0：除新增 CSV 尾列和 source
   identity 外，trajectory、status/cadence、旧 24 列、summary 必须一致。
3. 在 V2_02 或 V2_03 统计 active-segment
   `num_shared < min_pnp_inliers <= num_mapped_observations` 的 frame/episode 数、
   连续区间、距下一 recovery/outage 的位置。
4. artifact 写出 per-sequence eligible frames、episodes、最大/中位 mapped-minus-
   shared gap。

至少一个目标序列出现 flip-capable episode 才进入 Q4。否则在 #47 记录
`INCONCLUSIVE/STOP`，直接转入“输入 population/track identity”方向分析。

## 6. Q4 Constrain：factor admission 与 mixed quality

Q4 期间 PnP/support 继续读取 `num_shared`，使 posterior 权限与 liveness 权限分开
验证。

### 6.1 Cycle Q4.1：factor 分类与 admission

按一个 test → 最小实现 → GREEN 的顺序完成：

1. **Stereo control**：mapped positive-disparity observation 只建立一个 stereo
   factor；`num_current_mono_visual_factors == 0`，旧 estimate/diagnostics 保持容差。
2. **Mapped mono**：已有 3D 的 left-only observation 达 count 门后建立恰好一个
   `GenericProjectionFactor`；total 加一、mono 加一。
3. **No seed**：unmapped left-only 被 retained 并进入 track history，但
   `num_seeded_landmarks == 0`、factor count 为零。
4. **Delayed eligibility**：窗口较早的 left-only observation 在同 ID 后续由
   stereo keyframe seed 后，可在同一次 rebuild 取得 factor 资格。
5. **Mixed admission**：一条 stereo + 一条 left-only 共同满足既有
   `min_landmark_observations`，分别建立一个 factor；cull 后已 erase 的 observation
   不参与 count。

最小实现：

- PIMPL 新增共享 `Cal3_S2` 与 2D robust noise；
- `countObservations()` 以 staged map 为资格 owner，计数所有
  `disparity_px >= 0` 的 factor-capable observations；
- `buildGraph()` 先执行共同的 map/count admission，再以
  `disparity_px > 0` / `== 0` 建立互斥 factor；
- `VioGraphInfo` 同时记录 total visual、current total、current mono；
- final diagnostics 只采用产生最终 committed optimized state 的最后一次成功
  solve graph counters。

known-motion 测试以独立 GTSAM reference graph 或解析 pinhole projection 作为
oracle，比较公开 `VioEstimate::T_W_B`；不以“比 control 更好”代替绝对 oracle。

### 6.2 Cycle Q4.2：一个 private mixed-factor visitor

先分别写 mixed RMS 与 stereo-only equality 红灯，再在 anonymous namespace 内
建立一个只识别以下两种 factor 的 visitor：

- `GenericStereoFactor<Pose3, Point3>`；
- `GenericProjectionFactor<Pose3, Point3, Cal3_S2>`。

visitor 统一提供 pose key、landmark key、modality 与未白化 residual；非 visual
factor 返回 no-match。用它改造：

- graph RMS 与 skipping-missing-landmark RMS；
- Probe B per-landmark residual；
- 2D/3D `2*fx` sentinel cheirality count；
- per-landmark mean-cull，每条 factor 贡献一次 L2 norm；
- actual attached factor 对应的 left-camera z 检查。

RMS 始终按 visual factor 数作分母，不按 residual scalar 数作分母；all-stereo
graph 数值必须保持原合同。

### 6.3 Cycle Q4.3：cull、reopt 与 rollback

依次锁定：

1. mapped mono behind-camera 进入 `num_cheirality` 并沿既有全窗口 erase/cull-ID
   路径处理；
2. 同一 landmark 的四条 mixed factors 触发既有 mean-cull，三条不触发；
3. mixed cull 后 successful reopt 更新 total/mono current counts 与最终 RMS；
4. failed optional reopt 恢复 round 前 window/map，保留最后一次成功 solve 的
   factor diagnostics；
5. mono-bearing primary build/solve/finite failure 返回 `kFailed`，整个 packet 的
   window/map/track/coast/span/segment 回滚；新 current mono count 为零；相同后续
   合法 input 与 fresh control 对拍一致。

Q4 PASS 后再运行 positive-disparity-only control，确认估计、旧 diagnostics 与
stereo factor 数保持既有容差或冻结的 bit-equivalent 合同。

## 7. Q5a Feedback：mixed PnP

### 7.1 RED cases

在 `stereo_vo_pnp_test.cpp` 通过 public seam 逐个锁定：

1. root 已建图后，纯 left-only mapped correspondences 达门可成功 PnP；
2. mixed correspondence 的 proposal/guess 都在同一 RANSAC inlier indices 上评分；
3. stereo residual 使用 `(uL,uR,v)` L2，left-only 使用 `(u,v)` L2，mixed RMS
   分母为 inlier observation 数；
4. all-stereo acceptance/fallback fixture 数值不变；
5. proposal score 无效时 fallback，guess 无效而 proposal 有效时可接受；
6. accepted mask 删除 mapped stereo 与 mapped left-only outlier，保留 unmapped
   observation；完整 validated measurement 仍写入 track history。

### 7.2 GREEN

- 把 private `stereoRmsAtPose()` 收敛为 modality-aware pose score；投影、pose、
  landmark、index 或 residual 非有限时返回 invalid score。
- `tryPnpInit()` 的 correspondence scan 接纳所有 pre-intake mapped observations，
  `solvePnPRansac` 参数不变。
- PnP entry 使用 `num_mapped_observations >= min_pnp_inliers`；本阶段
  `full_visual_support` 仍读取 `num_shared`，所以 PnP 只改变 proposal/mask，尚不
  刷新 support anchor。
- accepted inlier indices 映射回同一次 mapped-ID scan；不重扫不同 population。
- `OfflineVoSession` 的 PnP fallback 诊断 population 同步为 mapped overlap。

Q5a filters、既有 PnP suite、Q4 quality/rollback filters 全部通过后才进入 Q5b。

## 8. Q5b Feedback：support、coast 与 segment

### 8.1 RED cases

1. active segment 中 `num_shared == 0`、mapped count 达
   `min_pnp_inliers`，primary graph 成功后在原 `segment_id` 恢复；coast/span 归零。
2. mapped count 低于门时保持 low-support coast；mixed factors 可约束 graph，但
   不刷新 last-support anchor。
3. 当前 packet 新 stereo seed 不回流 pre-intake mapped count，不把同 packet coast
   改判 recovery。
4. exact horizon、over-horizon outage、valid discontinuity cadence 沿用现有合同。
5. PnP fallback 不改 support classification；primary transaction 失败不提交恢复。
6. `low_connectivity` 在 full support packet 上使用 mapped overlap 与既有
   `min_shared_landmarks`。

### 8.2 GREEN

只替换以下 population input：

- `full_visual_support`；
- PnP/support recovery 的 last-support anchor 与 coast/span reset；
- `low_connectivity` overlap。

root seed、`min_seed_observations`、`min_track_observations_for_seed`、
backprojection、hanging landmark、far-return refresh、window、horizon、segment 和
discontinuity 时序不变。

## 9. Diagnostics 与 CSV 迁移

最终 `UpdateDiagnostics`/`VoDiagRow`/`diag.csv` 尾部顺序：

```text
...,
num_current_visual_factors,
num_mapped_observations,
num_current_mono_visual_factors
```

最终 CSV 为 26 列；前 24 列位置不变。Q1 commit 暂时为 25 列，Q4 增加第 26
列。字段语义：

| 字段 | 最终语义 |
|---|---|
| `num_shared` | pre-intake committed map 中的 positive-disparity overlap |
| `num_mapped_observations` | pre-intake committed map 中全部 validated overlap |
| `num_current_visual_factors` | 最后一次成功 solve graph 中连接 current frame 的 stereo + mono total |
| `num_current_mono_visual_factors` | 上述 total 中的 mono breakdown |
| `VioDiagnostics::m_visual_factors` | 最后一次成功 graph 的 stereo + mono total |

每 row 必须满足
`num_current_visual_factors >= num_current_mono_visual_factors`。outage、
discontinuity、invalid/rejected/failed 且没有 committed current solve 的 mono count
为零；hard failure 的 stateful diagnostics 从 rollback 后 state 重建。

## 10. TDD 执行顺序与验证命令

每个 cycle 严格执行：写一个 public-seam assertion → 运行并记录期望 RED → 只做
使该 assertion GREEN 的最小生产修改 → 运行相邻回归。GREEN 后再进入下一个
case；不在 RED/GREEN 中穿插无关重构。

建议 filters：

```bash
cmake -S . -B build -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --target phad_estimator_tests phad_apps_tests --parallel

./build/phad_estimator_tests \
  --gtest_filter='MappedLandmarkBearingTest.*'
./build/phad_estimator_tests \
  --gtest_filter='StereoVoPnpTest.*Mapped*:StereoVoPnpTest.*Mixed*'
./build/phad_estimator_tests \
  --gtest_filter='StereoVoOutlierCullTest.*Mono*:StereoVoOutlierReoptTest.*Mixed*'
./build/phad_estimator_tests \
  --gtest_filter='VioFullState.*Mapped*:StereoVoDiagnostics.*Mapped*'
./build/phad_apps_tests \
  --gtest_filter='OfflineVoSessionTest.WriteDiagCsvMatchesProbeContract'
```

实际 test 名称在 Red 时固定，后续命令随之精确更新。机制稳定后的扩大顺序：

```bash
ctest --test-dir build --output-on-failure -R phad_estimator_tests
ctest --test-dir build --output-on-failure -R phad_apps_tests
git diff --check
git status --short
git diff --stat
git diff -- <本片显式路径>
```

最终审计必须确认：

- include/constructor 与本机实验固定的 GTSAM 4.3 一致；
- no-double-count、2D/3D noise dimension、extrinsic direction 与 residual 分母；
- support 在 pre-intake population 上锁定；
- current factor count 跟随最后一次成功 solve/reopt graph；
- 失败路径没有 mono drop 后的隐式成功；
- config hash、frontend output、solver/window/horizon 未改变。

## 11. Natural replay、本机实验与产品门

### 11.1 Q1 natural replay

Q1 eligibility 默认在 clean Q1 worktree 的 Release build 上只跑 V2_02/V2_03，
沿用现有 bench/artifact 合同并串行执行；单项 accuracy 不形成停止条件。
保存：

- commit/tree/config/build identity；
- V2_02/V2_03 的 eligible frames/episodes；
- status/cadence 和旧字段的 Q0 equivalence 报告。

### 11.2 最终一次 matched candidate 本机实验

机制与 local regression 全部通过后，从 clean candidate commit 建立独立
worktree，以与 Q0 相同的 Release toolchain/config/data 串行运行 11 条：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence> \
  --out /home/lin/Projects/data/phad-bench/<candidate-run>/<sequence> \
  --sequence-name <sequence> \
  --repo <clean-candidate-worktree> \
  --estimator-enable-moving-bootstrap
```

必须核对 `git_dirty=false`、candidate HEAD/tree、canonical config、compiler、
GTSAM/OpenCV identity、unit suite 和 11 个 sequence artifact；全部终态并通过数据
质量检查后再分析，不从运行中的部分 artifact 下结论。

bench 继续生成完整 raw record；#47 对 matched clean Q0/candidate 结果应用已冻结
产品 gate。结论只在 11 条全部终态后形成；accuracy tail 与单条失败均不会取消
其余序列。无需修改 `scripts/remote_ci.py` 或其现有测试改动。

### 11.3 冻结多序列产品门

Stage 0 生成并经用户确认的 gate record 至少包含：

| 层 | primary / guardrail | 冻结内容 |
|---|---|---|
| `core-4` | `G_ATE` / `G_RPE` | aggregate envelope、逐序列原始值与 tail review 规则 |
| `EuRoC-11` | `G_ATE` / `G_RPE` | 最终 aggregate envelope、11 条完整性与 tail review 规则 |
| validity | completion/coverage、finite、task status | 有效性界限；所有 4/11 sequence 必须进入结果表 |
| continuity | segments、段内 RMS、绝对段间分量 | V2_03 改善目标及容差 |

candidate 的 suite-level ratio 按 spec §13 计算；原始值、绝对 delta、ratio、算术
均值与几何均值全部落盘。单序列 accuracy tail 进入 `REVIEW`，不触发 early-stop；
identity/hard validity、机制门或冻结 aggregate envelope 失败才形成自动 `FAIL`。

所有正式 candidate 还要求 `failed/rejected/reanchors == 0`；V2_03 必须同时出现：

- `num_mapped_observations - num_shared > 0`；
- `num_current_mono_visual_factors > 0`；
- `num_shared < min_pnp_inliers <= num_mapped_observations` 的同段成功 commit；
- 达到 gate record 冻结的 continuity 改善量。

若某条 hard-fail，最终结论至少为 `FAIL`，但仍收集其余 sequence；如果只有
per-sequence tail，则完成 §12 的归因后给出 `PASS` 或 `REVIEW` 结论。

## 12. 实验分析与下一方向决策

最终报告先回答“机制是否实际生效”，再回答“产品指标是否改善”：

1. **Mechanism**：eligible/attached/flip episode 数；mono factor current/total
   分布；PnP success/fallback；cull/reopt/cheirality；失败与 rollback。
2. **Continuity**：segments、outage endpoint、same-segment recovery、initializing
   frames、completion/coverage。
3. **Accuracy**：ATE/RPE、段内加权 RMS、绝对段间分量；逐序列对 Q0 与 M4
   checkpoint。
4. **Identity**：source/config/toolchain/data/命令完整，Q0 与 candidate 可复现。

按证据选择下一 vertical slice：

| 结果 | 结论 | 下一工作方向 |
|---|---|---|
| eligible、mono factors、support flip 均非零，core-4/EuRoC-11 aggregate 与 continuity gate 通过 | 本片机制与产品假设成立 | 将行为纳入当前 stereo VIO 默认路径；优先 M5 正式动态初始化，处理剩余 cold-root initializing |
| aggregate 与机制门通过，存在少量 per-sequence tail | 当前为 `REVIEW`，不能只由 aggregate 或 tail 单独定性 | 对 tail 做绝对量级、RPE、coverage、segments 与段内/段间归因，再决定接受或收窄承诺场景 |
| mono factors 生效但 support flip 稀少，segments 基本不变 | bearing 约束存在，liveness population 不足 | 分析 track identity/map retention 与 outage 前可复用 landmark 寿命，形成新的 Observe gate |
| support flip/segments 改善但 suite-level accuracy 或 continuity guardrail 失败 | 连续性收益伴随错误约束 | 按 residual、cheirality、cull/reopt 与序列片段定位 correspondence/quality 根因 |
| eligible population 存在但 attached mono 为零 | factor admission/count 生命周期未闭合 | 回到 Q4 synthetic/natural 差异，核查 PnP mask、window count、seed/cull 顺序 |
| V2_03 主要剩余为 segment root initializing，active 段内已稳定 | 本片完成 active-map 能力，瓶颈转移 | M5 moving/dynamic initialization；是否需要 relocalization 由段间分解另立 spec |
| Q1 无 flip-capable population | 当前数据不能证实本片的 liveness 假设 | 停止 Q4/Q5，转向 frontend disparity/track population 与 ID 持续性的观测设计 |

报告只对实际 run 观察到的分支给结论；其余保持预注册决策表，不作为已发生事实。

## 13. 完成条件与交付

#47 的“实现完成”至少要求：

1. Q0 identity 与 control artifact 完整；
2. Q1/Q4/Q5 各自的 Red 证据、GREEN tests 与 stop gate 可追溯；
3. 所有长期测试经 public seam，不新增 GTSAM public Interface；
4. estimator/apps 完整 suites 和 diff audit 通过；
5. clean candidate commit 可复现，matched 本机 EuRoC-11 已完整运行；
6. 机制计数与 11 序列分析已写入 benchmark/research 记录和 #47；
7. clean Q0/candidate 的 core-4 与 EuRoC-11 aggregate、tail review、validity 和
   continuity 判定完整；未执行的门不标为 PASS；
8. 给出一个由实际实验分支支持的下一工作方向及其首个 Observe 问题。

若完整多序列判定尚未形成，交付状态保持在已取得的资格阶段，并明确剩余验证与
环境差异。所有 commit、push、merge、远端清理和 shared 配置修改均遵守单独授权
边界。

# M4 基线之后的下一步方向分析

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-26

锚定基线：[M4 checkpoint `c999f58` / `default_0337287b`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)

相关：[Slice ⑦ checkpoint](../benchmark/m3.3/slice-7_e77ee5d_402d1925.md)、
[M3.3 残余 failure 债](2026-08-05-note-m3-3-remaining-failure-debt.md)、
[estimator AGENTS](../../phad/estimator/AGENTS.md)、[roadmap](../design/roadmap.md)

本文只做方向分析：不改代码、不改 `docs/design/roadmap.md`、不跑 bench、不建 issue。

---

## A. 一页式结论

M4 的 IMU 已经成功。checkpoint 发表的 11 条均值 ATE `0.5793 m` 里，
**`0.4454 m`（76.9%）不是段内轨迹误差，而是段之间的相对错位**。

把 93 个 segment 逐段独立对齐 GT 后（92 段可对齐，见 §B.4），段内加权
RMS 的 11 条均值是
`0.1339 m`，对 ORB-SLAM3 stereo 论文均值 `0.084` 为 **1.6×**。同口径下
Slice ⑦ 是 **12.2×**。也就是说，纯 VO 时代「差一个数量级」的局部精度
差距，在 M4 之后已经基本不存在了。

因此下一步不是初始化质量、不是立体匹配、不是精度调参，而是
**把已经存在的段内精度接成一条全局轨迹**。四片排序：

| 片 | 内容 | 依据 |
|---|---|---|
| 1 | 视觉中断时保住世界系（不清空观测 + 锚在传播后的 `NavState`） | §B 分解 + §C 机制 |
| 2 | MH_02 定向诊断（唯一段内回归） | §B |
| 3 | 单目重投影因子通道 | §D 信息量 |
| 4 | M5 正式动态初始化 | §E 片 4 |

片 1 最高价值且不新增解算器；M5 的 staged initializer 排末位，理由见
§E——5 条单段序列在极粗糙的 root 下段内质量已达 `0.070–0.230 m`。

---

## B. 段内 / 段间 ATE 分解（本次实测）

### B.1 方法与口径校验

口径：最近邻时间关联（`max_dt 2.5 ms`）+ Umeyama（不估尺度）+
translation RMSE，与 `summary.json` 同源。实现为仓内
[`scripts/segment_ate_decomp.py`](../../scripts/segment_ate_decomp.py) 的口径：按
`diag.csv` 的 `segment_id` 把 `est.tum` 切段，每段独立喂
`build/phad_traj_eval`，与全轨迹一次对齐的结果对照。

口径校验：11 条全局 ATE 与 checkpoint §3 表**逐序列吻合**——8 条完全
相同（如 MH_01 `0.0705388` vs `0.070539`），V1_03 / V2_02 / V2_03 三条
在 `phad_traj_eval` 的 6 位有效数字打印精度内一致（`1.407000` vs
`1.406996` 等），最大偏差 `4e-6 m`。

段内加权 RMS 按各段的 **matched pair 数**加权（与 ATE RMSE 自身的样本集
一致）：\(\mathrm{RMS} = \sqrt{\sum_k (m_k/\sum m)\,\mathrm{ATE}_k^2}\)。

「段间错位占比」定义为 \(1 - (\mathrm{RMS}_\text{段内}/\mathrm{ATE}_\text{全局})^2\)，
即全局误差平方中不能由段内误差解释的份额。

### B.2 11 条分解表

| sequence | 全局 ATE (m) | 段内加权 RMS (m) | 段间错位占比 | segments | 段内 vs Slice ⑦ 全局 | 段内 / ORB-SLAM3 |
|---|---:|---:|---:|---:|---:|---:|
| MH_01 | 0.0705 | 0.0705 | 0.0% | 1 | −13% | 2.4× |
| MH_02 | 0.2905 | 0.2497 | 26.1% | 6 | **+179%** | 13.1× |
| MH_03 | 0.0950 | 0.0950 | 0.0% | 1 | −26% | 4.0× |
| MH_04 | 0.2132 | 0.2132 | 0.0% | 1 | −26% | 2.5× |
| MH_05 | 0.2303 | 0.2303 | 0.0% | 1 | −29% | 4.4× |
| V1_01 | 0.1121 | 0.0983 | 23.1% | 2 | −16% | 2.8× |
| V1_02 | 0.0791 | 0.0780 | 2.7% | 3 | −85% | 3.1× |
| V1_03 | 1.4070 | 0.0471 | **99.9%** | 16 | **−99%** | **0.8×** |
| V2_01 | 0.1848 | 0.1848 | 0.0% | 1 | −40% | 4.5× |
| V2_02 | 1.9820 | 0.1493 | **99.4%** | 9 | −82% | 5.3× |
| V2_03 | 1.7075 | 0.0563 | **99.9%** | 52 | **−98%** | **0.1×** |
| **均值** | **0.5793** | **0.1339** | — | — | 10/11 改善 | **1.6×** |

ORB-SLAM3 stereo 逐序列值取自 [Slice ⑦ §7](../benchmark/m3.3/slice-7_e77ee5d_402d1925.md)
引用的论文 Table II（`0.084` 为其 11 条均值）；「vs Slice ⑦ 全局」是本次
段内 RMS 相对 Slice ⑦ **全局** ATE 的变化，两者口径不对等，只用于回答
「M4 的局部精度是否已优于纯 VO 时代的整条轨迹」。

### B.3 三个可直接读出的事实

1. **三条最差序列的误差几乎全是拼接产物。** V1_03 `1.4070 → 段内
   0.0471`（99.9% 为错位），已优于 ORB-SLAM3 stereo 的 `0.061`；
   V2_03 `1.7075 → 0.0563`（99.9%），远优于其 `0.521`；V2_02
   `1.9820 → 0.1493`（99.4%）。

2. **段内质量 10/11 优于 Slice ⑦ 的全局质量。** 唯一回归是 MH_02
   `+179%`。这条对比可以用同一工具跨里程碑核对：
   [2026-08-05-note-m3-3-remaining-failure-debt.md §5-3](2026-08-05-note-m3-3-remaining-failure-debt.md)
   在 2026-08-07 用同一脚本测得 V1_03 段内 `0.768`、V2_03 段内 `0.889`；
   M4 之后分别是 `0.0471` 与 `0.0563`，即 **16× 与 16× 的段内改善**。
   段间错位占比同期从 84.5% / 75% 升到 99.9% / 99.9%——段内被修好之后，
   剩下的全部是拼接。

3. **单段序列已经没有段间问题。** MH_01/03/04/05 与 V2_01 是
   `segments=1`，段内 = 全局，ratio `2.4–4.5×`。它们是「不引入拼接时
   M4 能到哪」的参照，也是 §E 片 4 排序的依据。

### B.4 一处记账

V1_02 的 `seg2`（19 帧）落在 GT 时间跨度之外——该段区间为
`[1403715608.462, 1403715609.362]`，而 GT 止于 `1403715608.412`，
`phad_traj_eval` 返回 `eval error 3: no estimate pose falls within 2500000 ns`。
这 19 帧同样不进入全局 ATE 的匹配集，所以段内 RMS 排除它与全局口径一致，
不是本次分解引入的偏差。其余 92 段全部成功对齐。

---

## C. 段间错位的机制（代码 + 实测）

### C.1 每段回世界原点

`completeActiveSegment()` 清空全部状态并置 `m_initialized=false`
（[`vio_estimator.cpp:687-704`](../../phad/estimator/vio_estimator.cpp)）。
重新 seed 时 `bootstrap_T_W_B` 初始化为 `Identity()`（`:1775`），此后
**只有 `.linear()` 被赋值**（`:1866`）——平移分量留在 identity。
`seedRoot()` 的 velocity 传 `Zero()`（`:2067`），moving 路径的 bias 也从
零重来。

实测确认：11 条序列共 **93 个 segment，每一段首帧位置都精确为
`[0,0,0]`**（0 个例外）。每次中断都把新段的世界原点重置到中断处，段与
段之间没有任何相对位姿约束——这就是 §B 里 99.9% 的来源。

### C.2 中断由 estimator 自身的二值门自持

```
visual_supported = !initialized || num_shared >= min_pnp_inliers   // :1912
imu_only_coast   = initialized && !visual_supported                // :1915
```

`min_pnp_inliers` 默认 `10`。一旦 `imu_only_coast` 成立：

- `candidate.m_observations` 被**置空**（`:2085-2087`，观测仅在
  `!imu_only_coast` 时才拷入）——该帧的观测被主动丢弃；
- 新 landmark 只从 `candidate.m_observations` 播种（`:2256`）⇒ 空观测无法
  播种；
- 于是 `num_landmarks` 归零、`num_shared` 无法回升 ⇒ `visual_supported`
  结构上不可能再为真；
- 直到 `m_visual_coast_horizon_ns`（默认 `500'000'000`，即 500 ms）走完，
  `completeActiveSegment()` 必然被调用（`:1935-1950`）。

即：**这不是「视觉失效」，而是一个自持的门。** MH_02 的 6 段切换处逐帧
实测（`diag.csv`，取 seg0→seg1 的过渡）：

| idx | status | seg | obs | lm | shared | disp |
|---|---|---:|---:|---:|---:|---:|
| 294 | ok | 0 | 159 | 42 | 8 | 34 |
| 295 | ok | 0 | 158 | 42 | 6 | 36 |
| 296 | ok | 0 | 170 | 30 | 7 | 38 |
| 297 | ok | 0 | 157 | 30 | **2** | 35 |
| 298 | **visual_outage** | 1 | 162 | **0** | 3 | 27 |
| 299 | ok | 1 | 162 | 0 | 0 | 34 |
| 300 | ok | 1 | 152 | 16 | 16 | 40 |

前端始终健康（`obs` 150–200），视差始终可用（`disp` 27–40）。崩掉的只有
`shared`（跨帧共享的已建图 landmark）与随后被清零的 `lm`。MH_02 的另外
4 次切换、V1_03 的 16 次、V2_02 的 9 次、V2_03 的 52 次形态相同。

`visual_outage` 帧数与段数是同一件事：MH_02 `5 outage / 6 segs`、
V1_01 `1/2`、V1_02 `2/3`、V1_03 `16/16`、V2_02 `9/9`、V2_03 `52/52`
（后三条的末次 outage 之后没有再产出被接受的位姿，故段数不再 +1）。

### C.3 与 M3.3 的落差，以及一个失效的计数器

M3.3 Slice ① 的 re-anchor 把 prior 打在最后一个被接受的位姿上；
`enable_reanchor` 选项现已从**全部源码**移除（`.hpp`/`.cpp` 中 0 处命中，
仅存在于 `docs/` 与 `docs/plans/` 的历史记录）。

一个需要注意的副作用：`robustness.reanchors` 计数器仍在
[`phad/bench/run_summary.hpp:71`](../../phad/bench/run_summary.hpp)、
`apps/offline_vo_session.hpp:132` 与序列化路径中存活，但
**全仓源码里已无任何自增点**（`.hpp`/`.cpp` 的 12 处命中全是声明、读取、
序列化或断言）——因此 checkpoint §3 的 `reanchors` 列在 11 条上全为 `0`，
即使 V2_03 实际切了 52 段。`tests/apps/offline_vo_session_test.cpp` 的三处
`EXPECT_EQ( result.counts.reanchors, 0U )` 还把这个恒零行为钉住了。
`phad/bench/README.md:76` 记载的
「`segments = reanchors + 1`」不再成立，`phad/estimator/README.md:50,91`
也仍在描述已删除的 `enable_reanchor`。后续分片读这两个字段时不能把
`reanchors=0` 当作「没有发生段切换」。

本节结论与 [2026-08-05-note-m3-3-remaining-failure-debt.md §5-3](2026-08-05-note-m3-3-remaining-failure-debt.md)
的既有判断一致（「修复优先级 = re-anchor 锚质量」、「结构性修复只能来自
M4」）；M4 已经交付了段内精度这一半，锚质量这一半仍未交付。

---

## D. 上游的信息量问题

### D.1 零视差观测在三处被丢弃

`GenericStereoFactor` 是当前唯一的视觉因子，且只在
`disparity_px > 0` 时建立（[`:1446`](../../phad/estimator/vio_estimator.cpp)）。
零视差观测同时被排除在另外两处：

| 位置 | 行为 | 注释理由 |
|---|---|---|
| `:1446` | 不建因子 | 「would project a degenerate right pixel」 |
| `:1137` | 不计入 `min_landmark_observations` | 「could admit a 1-factor point that slides freely along its ray」 |
| `:899` | 不进 PnP 对应集 | 「cannot constrain PnP — they have no stereo depth」 |

三处的理由在「立体因子」框架内都是正确的。问题在于框架本身：一个
**已经三角化过**的 landmark，其后续零视差观测依然携带完整的方位信息，
却因为无法构成立体因子而被整体丢弃。

### D.2 视差产出率（本次实测，`num_disparity / num_obs` 中位数）

| MH_01 | MH_02 | MH_03 | MH_04 | MH_05 | V1_01 | V1_02 | V1_03 | V2_01 | V2_02 | V2_03 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 69.4% | 68.8% | 71.6% | 67.7% | 70.0% | 74.0% | 65.6% | 59.2% | **23.7%** | **23.7%** | **5.9%** |

V2 组是重灾区：V2_01/V2_02 丢掉约 76% 的观测，V2_03 丢掉 94%。这与
V2_03 的 `896` 帧 `initializing`、`52` 段直接同源。

### D.3 外部对照（本次核对源码）

**[slambook2 ch13](https://github.com/gaoxiang12/slambook2)**——立体**只用于造点**，
位姿与 BA 全部走单目重投影：

- `BuildInitMap()`（`frontend.cpp:353`）与 `TriangulateNewPoints()`
  （`:105`）是立体的唯一用途；
- `EstimateCurrentPose()`（`:142`）遍历 `features_left_`，**只要该特征
  持有 `map_point_`** 就加一条单目 `EdgeProjectionPoseOnly(mp->pos_, K)`，
  不问右目是否匹配；
- `Backend::Optimize()`（`backend.cpp:94-97`）按 `feat->is_on_left_image_`
  选左/右外参，各建一条**单目** `EdgeProjection`；
- 关键的方向差异：`InsertKeyframe()`（`frontend.cpp:71-95`）的触发条件正是
  `tracking_inliers_ < num_features_needed_for_keyframe_`，随后立刻
  `DetectFeatures()` + `FindFeaturesInRight()` + `TriangulateNewPoints()`。
  **overlap 变薄即补点**——与 phad 的「overlap 变薄即停止播种并清空观测」
  （§C.2）方向完全相反。

**ORB-SLAM2/3**：`Optimizer::PoseOptimization`
（[ORB-SLAM3 `Optimizer.cc:866`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Optimizer.cc)）
按 `pFrame->mvuRight[i] < 0` 分派——无右目匹配时建单目
`EdgeSE3ProjectXYZOnlyPose`，否则建立体边。左目没有右目匹配的关键点仍是
一等观测，只是换一种因子。

**可用件（本地已核对）**：GTSAM `4.3a0` 已装
`GenericProjectionFactor`（`/usr/local/include/gtsam/slam/ProjectionFactor.h:40`）；
`Cal3_S2Stereo::calibration()`（`Cal3_S2Stereo.h:105`）可直接取出单目内参。
片 3 不需要新依赖。

### D.4 一个回溯解释

这一个设计选择——「没有视差就没有观测」——解释了 M3.3 十余片实验
（SAD 变体、剔点阈值、悬挂门、zombie 龄、skip-drop）为何多被证伪：
它们都在管理由该选择造成的观测稀缺，而没有触及稀缺的来源。

---

## E. 四片排序与门控

### 片 1 — 视觉中断时保住世界系（最高价值，不新增解算器）

两半，都在既有代码路径上：

1. **世界系连续。** IMU 连续时（raw interval provenance 完好、非
   `MeasurementDiscontinuity`），新 root 锚在传播后的 `NavState`
   （位姿 + 速度），bias 继承上一段末值，而不是回原点、`v=0`、`b=0`。
   `segment_id` 可继续自增作簿记，但世界系不变。
2. **解耦 `min_pnp_inliers` 的双重语义。** 该阈值现在同时表示「PnP 是否
   可解」与「是否继续摄入观测」。低 overlap 帧应继续摄入观测并播种
   landmark（即 slambook2 的补点触发方向），不再清空
   `candidate.m_observations`。

第 2 半直接减少中断次数，第 1 半让残余中断无害。所需构件全部已在位：
`preintegrate()`（`:1276`）、`pim.predict()`（`:2049`，已在产出
`gtsam::NavState predicted`）、`seedRoot()`（`:731`）、
`VioUpdateTransaction` 回滚，以及
[`std::variant<RawImuInterval, MeasurementDiscontinuity>`](../../phad/sensor/stereo_imu_packet.hpp)
提供的 provenance 判据。无新 solver、无新线性代数、不涉及可观测性理论。

**量化预期**（把段间错位项压掉，上界是 §B 的段内水平）：
V1_03 `1.407 → ~0.05`、V2_03 `1.708 → ~0.06`、V2_02 `1.982 → ~0.15`、
11 条均值 `0.579 → ~0.134`。

**门控**：硬门 MH_01 不劣于 `0.070539`（单段序列，片 1 不应触及）；
V1_03 / V2_02 / V2_03 各自的段间错位占比须显著下降。

**传播精度（已实测）**：见
[coast 传播漂移探针](2026-08-26-note-m4-coast-propagation-probe-design.md)。58 个单次
outage 事件上，500 ms 桥接的位置漂移中位数 `0.0708 m`、p90 `0.1624 m`、
姿态中位数 `0.603°`——约为段内加权 RMS（`0.1339 m`）的一半，桥接不会主导
误差。探针门 PASS。

**由此产生的两条设计约束**：

- 世界系继承须带**有界横限**：仅在连续无支撑跨度 `≤ ~600 ms`（探针实测
  范围）时继承，超出则诚实重开新段。该界不得外推到秒级。
- V2_03 有一半 outage 属**连锁人群**（新段从未取得视觉支撑即再次中断，
  最长链 8 次、最长无支撑跨度 `13 s`），传播桥接对其无效。这抬高了第 2 半
  的优先级：真正的机制是别让 coast 走满。

门控仍须保留逐事件明细——最差单次事件（V1_03 `0.4754 m` / `2.93°`）出现在
`0.37 m/s` 的低速段，聚合中位数会掩盖它。

### 片 2 — MH_02 定向诊断

MH_02 是 11 条里唯一的段内回归（`+179%`），其 RPE 亦上升
（checkpoint §5：`+0.055146`，MH 组最大、全表第二，仅次于 V2_02 的
`+0.206378`）。误差高度集中：`seg5`（`n=2611`，段内 ATE
`0.2673`）承担几乎全部，而 `seg0–seg4` 分别是 `0.0070 / 0.0036 / 0.0179 /
0.0108 / 0.0665`（帧数 298 / 38 / 14 / 39 / 11）。

先短诊断定位 seg5 的失效模式，再决定动作。不以调 IMU noise 或 root prior
代替诊断。

需要单列一片的原因：seg5 占 2634/3034 帧，MH_02 的段内质量几乎就等于
seg5 的段内质量。片 1 只能拿掉 MH_02 那 26.1% 的段间份额
（`0.2905 → ~0.2497`），`0.2497` 这个段内值不会因片 1 而变好——它是全表
最差的 `段内 / ORB-SLAM3` ratio（`13.1×`），必须单独查。

### 片 3 — 单目重投影因子通道

为**已建图** landmark 的零视差观测建 `GenericProjectionFactor`，并允许其
进入 PnP 对应集与 `min_landmark_observations` 计数。立体保持「造点」职责
（slambook2 的分工）。预期收益集中在 V2_01 / V2_02 / V2_03（§D.2）。

排在片 1 之后：片 1 会改变 overlap 与 segment 的统计口径，先做片 3 会污染
归因。

### 片 4 — M5 正式动态初始化

采纳两份输入文档的共同核心：gyro bias LS → fresh reintegration → 固定尺度
的 g/v 线性解 → 重力 2-DOF 切空间精化 → 既有 GTSAM LM 精化 → 验收后原子
提交。

排末位的依据是 §B.3 第 3 条：5 条单段序列在极粗糙的 root 下——`v=0`、
`b=0`、20 ms mean-acc 定姿
（[`m_bootstrap_min_duration_ns = 20'000'000`](../../phad/estimator/types.hpp)，
line 87）——段内质量已是 `0.0705 / 0.0950 / 0.2132 / 0.2303 / 0.1848`。
粗糙 root 不是当前 EuRoC 上的绑定约束。

真正需要 staged initializer 的是 M5 出口条款本身：手持起步（无静止段）与
IMU 真断裂后的硬重启。那是产品合同问题，不是当前 11 条的精度瓶颈。

---

## F. 两份输入文档的记账

两份 agent 调研文档作为次要输入被引用，不删除、不改写：
[2026-08-26-note-m5-stereo-vi-init-recovery.md](2026-08-26-note-m5-stereo-vi-init-recovery.md)（下称文档 1）、
[2026-08-26-note-phad-vio-dynamic-initialization-and-recovery-research.md](2026-08-26-note-phad-vio-dynamic-initialization-and-recovery-research.md)（下称文档 2）。

### 可直接采纳

**文档 1**：把上游源码钉死到 commit SHA + 行区间的做法；「论文有、此 SHA
无」的显式区分（VINS-Fusion `failureDetection()` 第一行 `return false`；
ORB-SLAM3 的 `minTime` / `nMinKF` 是函数内局部变量而非 YAML 项；Kimera
OGA 源码在但未接入 pipeline；OpenVINS 重初始化仅有 TODO）；三个 private
helper 的最小形态与 7 个初始化诊断字段；`b_a` 钉零 + 弱 prior。

**文档 2**：recovery 的世界系桥接 `T_{WV'}` 与「继承传播后的 `NavState`
作先验」——与本文片 1 同向，且文档 2 的 `情况 2 / 情况 3` 分类比本文更细；
「500 ms 是输出 coast 合同，不等于 IMU 物理连续性失效」；接受新 bias 后
必须 fresh reintegration；用 prior Mahalanobis 与 whitened residual 取代
论文里的魔法数。

### 需推迟

文档 2 的 `InitializationMode` / `InitializationPhase` /
`InitializationReason`（14 个 reason 值）三枚举 + 约 6 个新结构 + 10 个
helper + 约 32 个公开诊断字段，在有可测需求之前属于投机抽象。对照文档 1
只提 3 个 helper + 7 个字段。

两份的实施序列都是里程碑体量而不是 vertical slice：文档 1 的「推荐路线」
7 步，文档 2 的「最终实施顺序」14 步。

### 需修正

1. **优先级与 §B 的分解不符。** 两份都把 staged initializer 作为主线
   （文档 2 的实施序列首步是「只加 diagnostics 不改算法」，但求解器主线
   同样是 staged 对齐）。§B 显示当前 11 条的误差主体是拼接，不是初始化
   质量；staged initializer 应为片 4。

2. **两份都未识别 §C.2 的自持门。** 文档 1 把 `kVisualOutage` 当作既有
   lifecycle 状态（`completeActiveSegment` + commit），文档 2 给出了
   coast 合同 / 视觉局部地图重置 / IMU 连续性的三情况分类——两者都没有
   指出：观测是被 `imu_only_coast` **主动置空**的，因而 overlap 在结构上
   无法恢复、500 ms coast 必然走完。这是片 1 第 2 半的直接依据。

3. **两份都未指出每段回世界原点。** 文档 1 在对齐语境下反而明确主张
   「平移原点保持第一帧位置，不重设 identity」（L204），文档 2 只在冷启动
   语境下令 `p^W_0 = 0`。而 §C.1 实测 93/93 段首帧均为 `[0,0,0]`——这正是
   §B 里 99.9% 错位的机制，也是两份文档都缺的那一环。

4. **两份都未指出 `docs/design/roadmap.md` 的 M4 段落与本 checkpoint 矛盾。**
   roadmap L609–618 仍停在「M4 仍在进行中且未完成」、Q3
   `HYPOTHESIS_FAIL / HALF_STABILITY / STOP`、replay2 `HARD_ERROR`、
   `permission_granted=false`；而 M4 checkpoint 已是 11/11 clean 全量。
   本次只记录，不改 roadmap。

5. **文档 1 的 VINS-Fusion 引用指向了错误的一篇。** 文档 1 引
   [arXiv:1901.03642](https://arxiv.org/abs/1901.03642)，该文标题为
   *A General Optimization-based Framework for **Global Pose Estimation**
   with Multiple Sensors*（融合 GPS 等全局传感器的 pose graph）。本主题
   应为 local odometry 的
   [arXiv:1901.03638](https://arxiv.org/abs/1901.03638)（*...for **Local
   Odometry Estimation** with Multiple Sensors*，即 stereo / mono+IMU /
   stereo+IMU 那篇）。文档 2 引的正是 `1901.03638`，此处无需修正。

---

## G. 证据缺口与旁记

1. **单次 500 ms 桥接的传播精度已实测**（median `0.0708 m`，见
   [探针](2026-08-26-note-m4-coast-propagation-probe-design.md)）。剩余未知量是**多秒级
   连锁跨度**：V2_03 的最长无支撑跨度为 `13 s`，该量级的 IMU-only 传播
   精度未测，探针的 PASS 不覆盖它。片 1 因此只在已测量的 `~600 ms` 范围
   内继承世界系。
2. **V1_02 `95` 帧、V2_01 `110` 帧、V2_03 `896` 帧 `initializing` 的成因
   分类需读 `result.message`**，该字段不在 `diag.csv` 中，当前无法离线归因。
3. **「零视差观测中属于已建图 landmark 的比例」当前无法从 `diag.csv` 直接
   测**，因此 §D 只能给出视差产出率（23.7% / 5.9%）而不能给出片 3 的收益
   幅度——片 3 的预期是推断。建议片 3 先加该计数再动因子。
4. **`robustness.reanchors` 已是恒 `0` 的死字段**（§C.3），而
   `phad/bench/README.md` 与 `phad/estimator/README.md` 仍按旧语义描述它。
5. **`docs/design/roadmap.md` M4 段落与 checkpoint 矛盾**（§F 需修正 4），本次
   只记录。
6. **checkpoint §5 的 RPE 计数与其自身表格不一致**：正文写「RPE 为 4 条
   下降、7 条上升」，而同节表格的 `RPE Δ` 列为 3 条下降（V1_02 / V1_03 /
   V2_03）、8 条上升。以表格数值为准。
7. **生产路径上有调试输出**：
   [`vio_estimator.cpp:2276`](../../phad/estimator/vio_estimator.cpp) 的
   `std::cerr << "[6b] skip seed"` 在 seed 门限不足时无条件写 stderr。

### 复现方式

```bash
cmake --build build --target phad_traj_eval -j 8
python3 scripts/segment_ate_decomp.py \
  /home/lin/Projects/data/phad-bench/<sequence>/c999f58/default_0337287b \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence>
```

本文 §B 表格为 11 条序列各跑一次上述命令的汇总（段内 RMS 改用 matched
pair 加权，见 §B.1）。§C.2 与 §D.2 的逐帧量直接取自各 run 的
`diag.csv`；§C.1 的段首位置取自 `est.tum` 与 `diag.csv` 的
`segment_id` 关联。

# `phad::estimator` VO / VIO 后端

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录持有固定窗口 batch BA：消费 `KeyframeMeasurement`，输出 `T_W_B` 与
诊断。M2.3 为纯双目 VO（`StereoVoEstimator`）；不读图像、不做关键帧决策、
不依赖 `phad::frontend`。

CMake target：`phad_estimator`（alias `phad::estimator`），公开依赖
`phad::common`、`phad::camera`；GTSAM 为 PRIVATE（PIMPL 藏图与 Values，
include 标 SYSTEM）。

## 职责边界

| 做 | 不做 |
|---|---|
| 固定窗口 pose / landmark / 窗口内观测 | 关键帧决策（由 apps/session 决定）、feature track 生命周期 |
| `GenericStereoFactor` + LM、最老帧 Prior gauge | 边缘化、smart factor |
| M4.2：IMU 预积分因子（`CombinedImuFactor` + bias 随机游走 `BetweenFactor`）、伪初始化、`preint.Predict` 初值链（`enable_imu` 开关） | IMU 原始数据消费（由 sync 切段；estimator 只吃帧间段） |
| 重叠断裂时 re-anchor（`enable_reanchor`）；M4.2 IMU-on 时退役（窗口重建只清 landmark 表） | 分段 TUM / Atlas 式多轨迹 |
| 共视 / cheirality / 重投影 / `segment_id` / PnP 诊断 | ATE（`phad::eval`） |
| 正常路径 `solvePnPRansac` proposal + stereo 一致性仲裁 + 本帧 inlier 掩码 | frontend track 生命周期 |
| BA 后 mean-reproj / cheirality 剔点 + 拒同 id 复生 | frontend track 生命周期（由 apps 回传 drop） |
| `body_P_sensor = T_B_left_rectified` | 未校正左目外参 |

## 文件布局

| 文件 | 作用 |
|---|---|
| `types.hpp` | `StereoObservation`、`KeyframeMeasurement`、`VioUpdateResult` 等合同 |
| `stereo_vo_estimator.hpp` / `.cpp` | `StereoVoEstimator`（PIMPL 藏 GTSAM） |

## 数据流

```text
FrameTracks (frontend)        IMU 原始样本 (sync 切段插值, M4.1)
        │                              │
        ▼                              ▼
apps/stereo_vo_glue.hpp  ──► KeyframeMeasurement + imu_samples / t_prev / imu_gap
                                                      │
                                                      ▼
                                            StereoVoEstimator::update
                                    (M4.2: pending 拼接段 → 窗口 → 即时重建预积分)
                                                      │
                                                      ▼
                                               VioUpdateResult
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                         OfflineVoSession                 phad_euroc_runner
                    dropTracks(culled ids)                 估计轨迹叠加
                    → probe / phad_vo_bench
```

## IMU 机制（M4.2）

`enable_imu`（默认 true，进 config_hash；CLI `--no-imu` 关闭）打开后，
estimator 在固定窗口 batch BA 上叠加 IMU 因子与 V/B 变量；关闭时完整走原
M3.3 链（V/B 完全不进 graph，IMU-off 字节回归保证）。

### 段语义（与 sync 对齐）

- `KeyframeMeasurement.imu_samples` 是本帧与上一帧之间的 IMU 段
  `[t_prev, timestamp]`（`StereoImuPacket` 切段语义）：样本 `i` 覆盖区间
  `[t_i, t_{i+1}]`，右端样本不积分，段内 ΣΔt ≡ 图像间隔；相邻段共享右端
  样本（右端 = 下段左端）。
- 无 IMU 数据源时段恒空且 `imu_gap = true` → estimator 走 IMU-off 路径
  （PnP/恒速初值、无 V/B 因子）。

### pending 拼接（D9）

- 每帧的段**先**追加到 `pending_imu`（共享边界样本去重：与 pending 尾部同
  stamp 时只留一份），`pending_gap |= 本帧 gap`；
- 被拒/失败帧的段保留在 pending，下一帧从最后接受位姿继续预积分（段链不
  因失败帧断裂，拼接后 ΣΔt 不变式仍成立）；
- 成功帧：`candidate.imu_samples = pending`（含本帧段），帧入窗口后 pending
  清空。

### 因子图（buildGraph）

- 最老帧 = 段头锚：X prior（sigma 放松至 `imu_prior_pose_sigma`，IMU 因子
  约束重力/速度）+ `PriorFactor<Velocity>(0)` + `PriorFactor<ConstantBias>(0)`
  （C11/C15，中等 sigma 不扫参）；
- 相邻非 gap 帧对：从 j 帧原始样本即时重建预积分
  （`PreintegratedCombinedMeasurements(params, biasHat)`，biasHat = i 帧 bias
  当前值，C3）→ `CombinedImuFactor(X_i, V_i, X_j, V_j, B_i, B_j, preint)` +
  `BetweenFactor<ConstantBias>(B_i, B_j, Δbias=0, σ = rw·√dt)`；
- gap 帧（`imu_gap` 或段样本 < 2）：不建 V/B 变量、跳过其 IMU 因子（视觉
  照常）；gap 后第一帧（无入链）加 V/B weak prior 防 indeterminant LM；
- 噪声换算（设计稿 §2.3）：协方差 = 密度平方（acc/gyr 白噪声 + bias 随机
  游走 4 参数进 config）。

### 初值链与伪初始化（D8 / C4 / C5）

- IMU-on 且段可积分 → 位姿初值 = `preint.Predict(last_X, last_V, last_B)`
  （V/B 初值 = Predict 输出 / 上一帧 bias）；
- PnP 与恒速保留为 `imu_gap` / IMU-off 的兜底链；
- 伪初始化：首帧种子段 V=0、B=0，重力 g=9.81007（Z-up，`MakeSharedU`）。

### re-anchor 退役与回滚（D12 / D9）

- IMU-on 时 `overlap_broken` 不再 seedSegment：清一次 `landmarks_W`
  （`imu_window_rebuilt` 标记），候选帧照常进窗口（Predict 初值），
  `segment_id` 不增（segments/reanchors 恒 1/0）；观测不足不拒帧（图 = 仅
  IMU 因子 + priors 亦有解）；
- 事务回滚：IMU-on 时 `restore()` 不回滚 `last_accepted/prev` 与 pending
  段（失败帧的段已在入口挂入 pending），只回滚 landmark/观测/窗口结构。

## 段生命周期（M3.3）

`update()` 在已初始化且 `num_shared == 0`（新帧 landmark id 与窗口内
`landmarks_W` 无交集）时视为**重叠断裂**，不再永久拒帧：

| 条件 | 结果 |
|---|---|
| `enable_reanchor == false` | `kRejected`（M3.2 旧行为，A/B 对照用） |
| `num_obs < min_seed_observations` | `kRejected`，**不污染**窗口 / landmark / `segment_id` |
| 否则 | `seedSegment(anchor)`，`segment_id` 递增，继续 `kOk` |

`seedSegment` 初始化与 re-anchor **共用**：清空 `window` 与 `landmarks_W`，
只保留本帧；`track_times` 与 `next_frame_index` 继续累积（保证
`prior_key` 在图里唯一）。首段 anchor 为 `Identity()`；re-anchor 的 anchor
为 `poseInitialValue()`（`use_constant_velocity_init` 开则恒速外推，关则
沿用上一位姿）。

**re-anchor 帧的位姿是预测值**（等于 anchor），不是测量值：新段窗口只有一帧，
prior 与由该帧 backproject 得到的 landmark 初值自洽，优化不会移动它。
该帧仍记 `kOk`；段边界靠 `segment_id` 跳变表达，`UpdateStatus` 不加新枚举。

`min_seed_observations`（默认 10）首段初始化与 re-anchor **共用**，避免
「1 个观测建窗口」或「输出等于 anchor 的假位姿」刷高 completion。

## 关键帧参数（M3.3 Slice ⑤）

`update()` 新增 `bool keyframe = true` 参数（默认 `true` 保持向后兼容）：

```cpp
VioUpdateResult update(const KeyframeMeasurement& measurement,
                       bool keyframe = true);
```

| 路径 | keyframe=true | keyframe=false |
|---|---|---|
| 触发 | apps/session `isKeyframe()` 返回 true | session `isKeyframe()` 返回 false |
| 校验 | 共享 | 共享 |
| reanchor/首帧 | 可 | **不可**（非关键帧 `shared=0` → `kRejected`） |
| PnP + 仲裁 | 正常 | 正常（复用同一 `tryPnpInit` + stereo RMS 仲裁） |
| landmark backproject | 是 | **不** |
| 窗口 push/pop | 是 | **不** |
| buildGraph + LM | 是 | **不** |
| cull + reopt | 是 | **不** |
| 位姿输出 | `estimate` 字段有值 → 写 `est.tum` | `estimate` 为 `nullopt` → **不**写轨迹 |
| last/prev_accepted | LM 后更新 | PnP 后更新（保持恒速预测链） |

非关键帧 `shared < min_pnp_inliers` 时返回 `kRejected`（不更新 last/prev）。
snapshot 回退备份仅在 `keyframe=true` 时执行。

关键帧选择逻辑（`isKeyframe()`）在 apps/session 层，不在 estimator。

## `segment_id` 语义

- `UpdateDiagnostics.segment_id`：当前帧所属段；首段为 `0`，每次成功
  re-anchor 后递增。
- 正常帧：`segment_id` 不变。
- re-anchor 成功帧：`segment_id` 比上一接受帧大 1。
- `kRejected` / `kFailed`：诊断里的 `segment_id` 反映**回滚后**的状态
  （seed 门限拒帧时不递增）。

## PnP 初值与 stereo 一致性仲裁（M3.3 Slice ③）

正常路径（`initialized && num_shared > 0`）在 `poseInitialValue()` guess 之上
可选跑 `cv::solvePnPRansac`（PIMPL 内、`PRIVATE opencv_calib3d`）：

| 条件 | 结果 |
|---|---|
| `enable_pnp_init == false` | `T_W_B = guess`，不 cull；复现 Slice ② |
| `num_shared < min_pnp_inliers` 或 RANSAC 失败 / inliers 不足 | fallback：`T_W_B = guess`，**不** cull、**不**拒帧 |
| proposal 的 stereo score 无效，或比 guess 差超过 `stereo_sigma_px` | fallback：`T_W_B = guess`，保留本帧全部观测，**不**应用 PnP mask |
| proposal 有效，且 guess 无效或 proposal RMS ≤ guess RMS + `stereo_sigma_px` | `T_W_B` 取 PnP；从本帧观测去掉 shared 外点（新 id 保留） |

首段 seed / re-anchor **不跑** PnP（`num_shared == 0`）。默认
`pnp_reproj_px=2.0`、`pnp_confidence=0.99`、`min_pnp_inliers=10`。

PnP 成功只生成 proposal，不直接授权 pose 或 mask。proposal 与 guess 都在 PnP
返回的同一 shared inlier 集上计算未白化 `(uL,uR,v)` RMS；非法 index、非有限投影
或 cheirality 令候选 score 无效。`stereo_sigma_px` 是既有观测噪声，也作为统计
等价带，不新增配置或 `config_hash` 输入。只有仲裁采用 proposal 后才应用其
inlier mask；回退不修改 measurement。

**掩码语义**：被掩码的 shared 外点仍写入 `track_times` /
`observationTimestamps()`；`num_observations` 保持测量原值（不是入图观测数）。
`landmarks_W` 与 frontend track 不动——伪永久生命周期见 Slice ④。

诊断：`UpdateDiagnostics.pnp_success` / `pnp_inliers`；session 汇总
`pnp_successes` / `pnp_fallbacks`（仅正常路径；seed / re-anchor 不计
fallback）。详见 `docs/research/m3.3-slice3-pnp-design.md`、
`docs/research/m3.3-pnp-stereo-consistency-design.md` 与
`docs/research/m3.3-pnp-stereo-arbitration-results.md`。

## 外点剔除与多轮重优（M3.3 Slice ④ / ④b / ④e）

LM₁ 收敛写回位姿后、返回 `kOk` 前，可选按 landmark **平均 stereo 重投影**
（`||unwhitenedError||` 均值，≥4 观测）从 `landmarks_W` 删除高误差点，并经
共用 helper 清窗口观测。`reproj_rms_after_px` **始终**是 LM₁ 后、mean-cull
**前** 的全图 RMS（不受后续 reopt 轮次影响）。

**多轮热路径（Slice ④e）**：每趟 mean-cull / cheirality 必须用**该趟** LM 的
graph + values 打分。若本趟 `culled_round >= 4`（仅 mean-cull 计数；cheirality
不计触发）且 `enable_outlier_reopt`，且已成功轮数 `< max_outlier_reopts`，则
用剩余窗口观测与 `landmarks_W` **重建 factor graph** 再跑一趟 LM，写回后再次
cull。如此循环直至本趟 cull `< 4`、达到 `max_outlier_reopts`、或开关关闭。
某趟 LM 失败则回退到该趟开始前的 window / landmarks（保留此前已成功轮次），
仍返回 `kOk`（`outlier_reopt_failed=true`；`outlier_reopt == (rounds > 0)`）。
`max_outlier_reopts = 0` 或 `enable_outlier_reopt=false` 复现只 cull 不重优。

| 选项 | 语义 |
|---|---|
| `enable_outlier_cull`（默认 `true`） | `false` **只关** mean-reproj 剔点；cheirality 清窗口观测 helper 仍生效；无剔点则自然不触发 reopt |
| `outlier_avg_reproj_px`（默认 `4.0`） | 均值阈值（像素）；构造时须 `> 0`；bench 可用 `--outlier-avg-reproj-px` 覆盖扫参（Slice ④d） |
| `enable_outlier_reopt`（默认 `true`） | `false` → 只 cull 不重优；触发条件另需本趟 `culled_round >= 4` |
| `max_outlier_reopts`（默认 `3`） | ≥0；本帧最多成功 reopt 轮数；`0` → 永不重优；进 flattenConfig |
| `block_culled_rebirth`（默认 `true`） | `false` → 允许同 id stereo-backproject 重生（复现 Slice ④ 伪永久，仅 A/B） |

**拒复生（Slice ④c）**：mean-cull 与 cheirality 真正 erase 的 id 写入
`culled_ids_`；`block_culled_rebirth` 时 seed / 正常路径 skip 同 id
backproject。被删 id 的 `track_times` / `observationTimestamps()` **仍保留**。
本帧列表 `UpdateDiagnostics.culled_landmark_ids`（mean-cull ∪ cheirality）
仅在提交成功路径填充；`restore()` 后为空；**不**进 `diag.csv`。
`outliers_culled` / `unique` **仍只计** mean-reproj cull（跨轮累计）。

诊断：

| 字段 | 语义 |
|---|---|
| `outliers_culled` / `outliers_culled_unique` | 本帧 mean-reproj 删点数 / 去重 id（含 reopt 后各趟 cull） |
| `culled_landmark_ids` | 本帧永久移出地图的 id（mean-cull ∪ cheirality）；仅内存 / API |
| `lm_iterations` | LM₁ + 各成功 reopt 轮 LM 迭代累加 |
| `reproj_rms_after_cull_px` | **有成功 reopt**：最近成功轮 LM 后 graph RMS；**无 reopt / 首趟即失败**：Slice ④ 语义（cull 关时 `== reproj_rms_after_px`；cull 开时跳过已删 id 的 graph RMS） |
| `outlier_reopt` / `outlier_reopt_rounds` / `outlier_reopt_failed` | `outlier_reopt == (rounds > 0)`；成功轮数；是否有轮次失败已回退；**均不**进 `diag.csv` |

session 累计成功 reopt **次数**为
`FrameCounts.outlier_reopts`（Σ `outlier_reopt_rounds`，非帧数）→
`summary.json` 的 `robustness.outlier_reopts`。详见
`docs/research/m3.3-slice4-outlier-cull-design.md`、
`docs/research/m3.3-slice4b-outlier-reopt-design.md`、
`docs/research/m3.3-slice4c-cull-track-drop-design.md` 与
`docs/research/m3.3-slice4e-multiround-reopt-design.md`。

## 诊断 CSV 合同（probe）

```bash
phad_stereo_vo_probe <sequence-root> --tum <path> [--diag-csv <path>]
```

`--diag-csv` 每帧一行：

```text
timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,
window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,
num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,
pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,
is_keyframe
```

共 **19 列**（Slice ① 在 M2.3 的 13 列尾追加 `segment_id` → 14；Slice ③
再追加 `pnp_success,pnp_inliers` → 16；Slice ④ 再追加
`outliers_culled,reproj_rms_after_cull_px` → 18；Slice ⑤ 再追加
`is_keyframe` → 19；有意的契约变更）。
`pnp_success` 为 `0/1` 整数，`is_keyframe` 为 `0/1` 整数。`status` 为
`ok` / `rejected` / `failed`。非关键帧的优化相关列
（`reproj_rms_before/after`、`num_cheirality`、`lm_iterations`、`outliers_culled`
等）填 0。stdout summary 增加 `total_keyframes` / `total_track_only_frames`。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/stereo_vo_glue.hpp` | `FrameTracks` → `KeyframeMeasurement` |
| `apps/phad_stereo_vo_probe.cpp` | 无窗口验收与基线 |
| `apps/phad_euroc_runner.cpp` | 可视化叠加估计轨迹 |
| `tests/estimator/` | 合成单测 |

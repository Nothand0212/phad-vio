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
| 固定窗口 pose / landmark / 窗口内观测 | 关键帧决策、feature track 生命周期 |
| `GenericStereoFactor` + LM、最老帧 Prior gauge | 边缘化、smart factor、IMU |
| 重叠断裂时 re-anchor（`enable_reanchor`） | 分段 TUM / Atlas 式多轨迹 |
| 共视 / cheirality / 重投影 / `segment_id` / PnP 诊断 | ATE（`phad::eval`） |
| 正常路径 `solvePnPRansac` 初值 + 本帧 inlier 掩码 | frontend track 生命周期 |
| BA 后 mean-reproj 伪永久剔点（不重优） | 拒绝名单 / 真永久删 frontend track |
| `body_P_sensor = T_B_left_rectified` | 未校正左目外参 |

## 文件布局

| 文件 | 作用 |
|---|---|
| `types.hpp` | `StereoObservation`、`KeyframeMeasurement`、`VioUpdateResult` 等合同 |
| `stereo_vo_estimator.hpp` / `.cpp` | `StereoVoEstimator`（PIMPL 藏 GTSAM） |

## 数据流

```text
FrameTracks (frontend)
        │
        ▼
apps/stereo_vo_glue.hpp  ── filter kValid ──► KeyframeMeasurement
                                                      │
                                                      ▼
                                            StereoVoEstimator::update
                                                      │
                                                      ▼
                                               VioUpdateResult
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                         phad_stereo_vo_probe            phad_euroc_runner
                          TUM + diag CSV                  估计轨迹叠加
```

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

## `segment_id` 语义

- `UpdateDiagnostics.segment_id`：当前帧所属段；首段为 `0`，每次成功
  re-anchor 后递增。
- 正常帧：`segment_id` 不变。
- re-anchor 成功帧：`segment_id` 比上一接受帧大 1。
- `kRejected` / `kFailed`：诊断里的 `segment_id` 反映**回滚后**的状态
  （seed 门限拒帧时不递增）。

## PnP 初值（M3.3 Slice ③）

正常路径（`initialized && num_shared > 0`）在 `poseInitialValue()` guess 之上
可选跑 `cv::solvePnPRansac`（PIMPL 内、`PRIVATE opencv_calib3d`）：

| 条件 | 结果 |
|---|---|
| `enable_pnp_init == false` | `T_W_B = guess`，不 cull；复现 Slice ② |
| `num_shared < min_pnp_inliers` 或 RANSAC 失败 / inliers 不足 | fallback：`T_W_B = guess`，**不** cull、**不**拒帧 |
| 成功且 inliers ≥ `min_pnp_inliers` | `T_W_B` 取 PnP；从本帧观测去掉 shared 外点（新 id 保留） |

首段 seed / re-anchor **不跑** PnP（`num_shared == 0`）。默认
`pnp_reproj_px=2.0`、`pnp_confidence=0.99`、`min_pnp_inliers=10`。

**掩码语义**：被掩码的 shared 外点仍写入 `track_times` /
`observationTimestamps()`；`num_observations` 保持测量原值（不是入图观测数）。
`landmarks_W` 与 frontend track 不动——伪永久生命周期见 Slice ④。

诊断：`UpdateDiagnostics.pnp_success` / `pnp_inliers`；session 汇总
`pnp_successes` / `pnp_fallbacks`（仅正常路径；seed / re-anchor 不计
fallback）。详见 `docs/research/m3.3-slice3-pnp-design.md`。

## 外点剔除（M3.3 Slice ④）

LM 收敛写回位姿后、返回 `kOk` 前，可选按 landmark **平均 stereo 重投影**
（`||unwhitenedError||` 均值，≥4 观测）从 `landmarks_W` 删除高误差点，并经
共用 helper 清窗口观测。**不重优**：本帧 `estimate.T_W_B` 仍为剔点前 BA
结果；`reproj_rms_after_px` 仍是剔前 graph RMS。

| 选项 | 语义 |
|---|---|
| `enable_outlier_cull`（默认 `true`） | `false` **只关** mean-reproj 剔点；cheirality 清窗口观测 helper 仍生效 |
| `outlier_avg_reproj_px`（默认 `3.0`） | 均值阈值（像素）；构造时须 `> 0` |

**伪永久**：无拒绝名单；被删 id 的 `track_times` /
`observationTimestamps()` **保留**；frontend 再喂同 id → 按新点
backproject。Impl 侧 `culled_ids_` 仅用于去重统计。

诊断：`outliers_culled` / `outliers_culled_unique` /
`reproj_rms_after_cull_px`（剔后跳过被删 id 的 graph RMS，不重建图；
`enable_outlier_cull=false` 时等于 `reproj_rms_after_px`）。详见
`docs/research/m3.3-slice4-outlier-cull-design.md`。

## 诊断 CSV 合同（probe）

```bash
phad_stereo_vo_probe <sequence-root> --tum <path> [--diag-csv <path>]
```

`--diag-csv` 每帧一行：

```text
timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,
window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,
num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,
pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px
```

共 **18 列**（Slice ① 在 M2.3 的 13 列尾追加 `segment_id` → 14；Slice ③
再追加 `pnp_success,pnp_inliers` → 16；Slice ④ 再追加
`outliers_culled,reproj_rms_after_cull_px` → 18；有意的契约变更）。
`pnp_success` 为 `0/1` 整数。`status` 为 `ok` / `rejected` / `failed`。
stdout summary 含帧数、各状态计数、拒帧比例、`low_connectivity` 帧数、
`pnp_successes` / `pnp_fallbacks`、重投影 RMS 中位数与 p95；session 还会在
`warnings` 里按需汇总 `segments` / `reanchors` / `seed_rejected`、PnP
summary 与 outlier cull summary（见 `apps/AGENTS.md`）。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/stereo_vo_glue.hpp` | `FrameTracks` → `KeyframeMeasurement` |
| `apps/phad_stereo_vo_probe.cpp` | 无窗口验收与基线 |
| `apps/phad_euroc_runner.cpp` | 可视化叠加估计轨迹 |
| `tests/estimator/` | 合成单测 |

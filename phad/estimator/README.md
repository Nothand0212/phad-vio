# `phad::estimator` VO / VIO 后端

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录持有固定窗口 batch BA：消费 `KeyframeMeasurement`，输出 `T_W_B` 与
诊断。M2.3 为纯双目 VO（`StereoVoEstimator`）；不读图像、不做关键帧决策、
不依赖 `phad::frontend`。

CMake target：`phad_estimator`（alias `phad::estimator`），公开依赖
`phad::common`、`phad::camera`、`phad::sensor`；GTSAM 为 PRIVATE（PIMPL 藏图与 Values，
include 标 SYSTEM）。

M4 Q2 另提供纯 `integrateGyroRotation` helper：消费 timestamped gyro point
samples 与同 frame、同为 rad/s 的已知常值 bias，返回 `delta_R_i_j` 和 exact
`int64` 纳秒 duration。它按相邻 endpoint average 积分，不读取 accel；所有输入、
时间算术、非有限计算和无效 rotation 均返回 typed error，不返回 partial/identity
fallback。该 helper 当前没有 production caller，不进入 visual posterior、factor、
optimizer、配置或 feedback。

M4 的 `PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1` 另行冻结 default-off、无
real caller 的 estimator-private online gyro-bias 资格机制。本 README 与
[synthetic design](../../docs/research/2026-08-19-note-m4-online-gyro-bias-synthetic-design.md)、
[ADR-0002](../../docs/adr/0002-stage-gated-gyro-only-bias-state.md)、
[conventions](../../docs/design/conventions.md)、[roadmap](../../docs/design/roadmap.md) 和
[implementation plan](../../docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md)
共同构成 pre-implementation exact-six authority。`EstimatorOptions::m_gyro_bias` 只允许
synthetic test 直接构造 in-memory POD；不得接 parser、`flattenConfig()`、
`config_hash`、persistent artifact、apps/session 或真实数据。`G(k)` key/type、PIM、
factor、graph lifecycle 与 ownership 继续只在 PIMPL/private implementation；完整
`X/V/B` 获授权时直接删除该阶段机制，不保留兼容层。visual staging 后会变化的
mutable ownership 聚合在 private `StereoVoUpdateState`，由 production 实际使用的
`StereoVoUpdateTransaction` RAII seam 统一 commit/rollback；public diagnostics 不存入
State，而从 committed topology 确定性生成。

本片新增或迁出的所有 C++ `struct` / `class` data member 遵守 `m_` +
snake_case，transform 用 `m_T_target_source`。已有 public aggregate 旧成员不改名；只新增
`KeyframeMeasurement::m_gyro_interval`、`EstimatorOptions::m_gyro_bias` 与
`UpdateDiagnostics::m_gyro`。新 diagnostics 使用 `m_bias_radps`、`m_window_biases`、
`m_rotation_factors`、`m_rw_factors`、`m_root_priors`、`m_relinearization_rounds` 与
`m_break_reason`。`m_relinearization_rounds` 只在最终 `kOk` 时报告实际 extra count，包括
内部成功/fallback 后最终 `kOk`；它不是 solver trace，任何最终 `kRejected` /
`kFailed` 均为 `0`，即使 cap 前执行过 extra round。

## 职责边界

| 做 | 不做 |
|---|---|
| 固定窗口 pose / landmark / 窗口内观测 | 关键帧决策（由 apps/session 决定）、feature track 生命周期 |
| `GenericStereoFactor` + LM、最老帧 Prior gauge | 边缘化、smart factor；full IMU state、covariance propagation、IMU factor、gravity 与 accel residual |
| 重叠断裂时 re-anchor（`enable_reanchor`） | 分段 TUM / Atlas 式多轨迹 |
| 共视 / cheirality / 重投影 / `segment_id` / PnP 诊断 | ATE（`phad::eval`） |
| 正常路径 `solvePnPRansac` proposal + modality-aware 一致性仲裁 + 本帧 inlier 掩码 | frontend track 生命周期 |
| BA 后 mean-reproj / cheirality 剔点 + 拒同 id 复生 | frontend track 生命周期（由 apps 回传 drop） |
| `body_P_sensor = T_B_left_rectified` | 未校正左目外参 |

## 文件布局

| 文件 | 作用 |
|---|---|
| `types.hpp` | `StereoObservation`、`KeyframeMeasurement`、`VioUpdateResult` 等合同 |
| `stereo_vo_estimator.hpp` / `.cpp` | `StereoVoEstimator`（PIMPL 藏 GTSAM） |
| `gyro_rotation_predictor.hpp` / `.cpp` | Q2 known-bias deterministic rotation prediction |

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
                         OfflineVoSession                 phad_euroc_runner
                    dropTracks(culled ids)                 估计轨迹叠加
                    → probe / phad_vo_bench
```

## M4 active local visual continuity

`VioEstimator::update()` 在同一 active local estimator 中区分三层视觉语义：

| 层级 | 判定与作用 |
|---|---|
| observation retained | packet 通过现有校验与 mapped-observation PnP mask 后，observations 写入当前 transaction/window；`m_track_times` 按 normal path 更新，包含 zero-disparity observation |
| graph visually constrained | 最终成功 solve 的 graph 含当前 frame 的 mapped observation factor；positive disparity 建立 `GenericStereoFactor`，zero disparity 建立 `GenericProjectionFactor`，共同要求 map membership 与 `min_landmark_observations` |
| full visual support | packet 摄入前的 committed map 上，positive-disparity `num_shared >= min_pnp_inliers` |

低于 full visual support 的 accepted packet 仍可保留 observations、seed landmarks、
建立 visual factors 并运行既有 cheirality / mean-cull / optional-reopt 质量路径。
landmark refresh/seed 继续遵守 keyframe、track age、positive disparity、
cull/rebirth 与 backprojection 门。当前 packet 的新 seed 不回流到本 packet 的
support 判定。

每条 mapped observation 最多建立一个 visual factor。left-only bearing 复用左目
`Cal3_S2`、body-to-left extrinsic、像素 sigma 与 Huber 配置；它不提供深度，也不
参与 landmark seed。Q5a 起，摄入前已 mapped 的 left-only observation 可参与 PnP；
full-support 仍暂用 positive-disparity `num_shared`。stereo 与 mono factor 统一进入
RMS、Probe B、cheirality、mean-cull 和 optional reopt。

full visual support 每个 packet 只在摄入前计算一次。只有摄入前已达门且整个
primary transaction 成功 commit，才刷新 support anchor，并将
`m_visual_coast_duration_ns` 与 `unsupported_span_ns` 归零。实际 visual-factor
presence 不改变 support anchor；factor-bearing low-support packet 继续消耗同一个
500 ms coast budget。unsupported endpoint 恰好落在 horizon 时可提交 `kOk`；超过
horizon 才返回一次 `kVisualOutage`。后续 supported endpoint 优先在原
`segment_id` 恢复。visual outage 保留 support anchor/span 供后续诊断累计，
discontinuity 完成段并清除它们。

propagation、observation intake、track lifetime、landmark、window eviction /
reintegration、graph/solver、quality、support/span 与 diagnostics 受同一 transaction
保护。hard failure 返回实际状态并恢复调用前 committed state。成功 commit 后发布：

| `UpdateDiagnostics` 字段 | 语义 |
|---|---|
| `num_mapped_observations` | 当前 validated packet 中，ID 在摄入前 committed map 的 observation 数；包含 positive-disparity 与 zero-disparity observation |
| `num_retained_observations` | 现有 PnP mask 后写入当前 frame 的 observation 数 |
| `num_seeded_landmarks` | 本 transaction 插入 map 的新 landmark 事件数；同 packet 后续 cull 不回写该事件数 |
| `num_current_visual_factors` | 产生最终 committed optimized state 的最后一次成功 solve 中，属于当前 frame 的 stereo + mono factor 总数 |
| `num_current_mono_visual_factors` | 上述 current visual factor 中的 mono breakdown；必定不大于 total |
| `unsupported_span_ns` | 当前 timestamp 相对最近 committed full-support anchor 的诊断 span；support commit 与 discontinuity row 为 `0` |

primary solve 后 cull 而未成功 reopt 时，factor count 保留 primary graph 口径；
成功 reopt 后改用该 graph；optional reopt 失败并局部回滚时保留前一次成功 solve
的 graph 口径。packet counters 为 update-local，只在整个 transaction commit 后发布。

## 段生命周期（M3.3）

`update()` 在已初始化且 `num_shared == 0`（新帧 landmark id 与窗口内
`m_landmarks_w` 无交集）时视为**重叠断裂**，不再永久拒帧：

| 条件 | 结果 |
|---|---|
| `enable_reanchor == false` | `kRejected`（M3.2 旧行为，A/B 对照用） |
| `num_obs < min_seed_observations` | `kRejected`，**不污染**窗口 / landmark / `segment_id` |
| 否则 | `seedSegment(anchor)`，`segment_id` 递增，继续 `kOk` |

`seedSegment` 初始化与 re-anchor **共用**：清空 `m_window` 与 `m_landmarks_w`，
只保留本帧；`m_track_times` 与 `m_next_frame_index` 继续累积（保证
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
| PnP + 仲裁 | 正常 | 正常（复用同一 `tryPnpInit` + modality-aware RMS 仲裁） |
| landmark backproject | new-id seeding；existing-id refresh 共用 | 不 seed new id；existing-id refresh/backproject 共用 |
| 窗口 push/pop | 是 | 是；作为 temporal graph state 入窗，满窗时优先逐出最老 non-KF |
| buildGraph + LM | 是 | 是；与窗口内 keyframe/non-KF 一起优化 |
| cull + reopt | 是 | 是；共享同一 graph lifecycle |
| 位姿输出 | 成功时 `estimate` 有值 | 成功时同样有 `estimate`，可写轨迹 |
| last/prev_accepted | LM 后更新 | LM 后更新 |
| visual-staging transaction / hard-failure rollback | staging-entry 全窗口、landmark 与 accepted-pose state 同一 transaction | 与 keyframe 完全相同；不得因 `keyframe=false` 跳过 snapshot/restore |

非关键帧 `shared < min_pnp_inliers` 时返回 `kRejected`（不更新 last/prev）。
通过该门的 accepted non-KF 会拥有自己的 `frame_index`、window entry、`Pose3 X(k)`、LM
与优化后 estimate；`keyframe` 只控制首段/re-anchor、new-id seeding、7-keyframe cap 与 eviction
priority，不控制 graph state ownership。任何 post-staging `kRejected` / `kFailed` 都必须恢复 update
的 visual-staging-entry committed snapshot，KF/non-KF 语义相同。M4 迁出的 private `WindowFrame`
使用 `m_frame_index`、`m_timestamp`、`m_T_W_B`、`m_observations`、`m_is_keyframe`、
`m_gyro`；`GyroFrameState` 使用 `m_bias_radps`、`m_segment_id`、`m_component_id`、
`m_predecessor_frame_index`、`m_interval`。snapshot 覆盖 `StereoVoUpdateState` 的 `m_window`、
`m_landmarks_w`、`m_track_times`、`m_T_W_B_last_stereo`、`m_T_W_B_last_accepted`、
`m_T_W_B_prev_accepted`、`m_next_frame_index`、`m_initialized`、`m_segment_id`、`m_culled_ids`、
`m_pending_seed_obs`、`m_eligible_visual_rejected_timestamp`、`m_next_gyro_component_id`，以及全部
`G`/link/root/component。`Impl` 的唯一 mutable owner 是
`std::unique_ptr<StereoVoUpdateState> m_state`。

transaction constructor 只 deep-copy backup，不把 copy swap 成 live owner；正常 gyro on/off 成功
update 的 `m_state.get()` 地址保持不变。rollback 用 owner/backup `unique_ptr::swap` 恢复
snapshot-owned 对象，因此 rollback 后只要求字段 exact 恢复，不要求返回已销毁的原
owner 地址。basic/gyro validation、visual support 以及既有 intentional pre-staging
`m_pending_seed_obs` / rejected-endpoint provenance 动作发生在 snapshot 前，保持原有持久语义；
不得把它们误称为 post-staging rollback 的一部分。

所有 post-staging hard result 只能经唯一 private/local
`finalizePostStagingHardResult(...)`，严格先 explicit rollback，再从恢复后 `*m_state`
重新生成 result 与 `m_gyro` topology diagnostics，最后 return；禁止 transaction region 直接 hard
return，也不能跨 rollback 复用 state 内部 reference/pointer/iterator。stable public
`StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics` 必须以 structurally valid
unknown-predecessor interval 触达该顺序：返回 `kRejected`，scalar null、rounds `0`、reason none，
`m_window_biases` 和 counts exact 等于 rollback committed topology，随后 exact update 与 fresh control
bit-exact，不声称触达 public optimizer exception。

关键帧选择逻辑（`isKeyframe()`）在 apps/session 层，不在 estimator。

## `segment_id` 语义

- `UpdateDiagnostics.segment_id`：当前帧所属段；首段为 `0`，每次成功
  re-anchor 后递增。
- 正常帧：`segment_id` 不变。
- re-anchor 成功帧：`segment_id` 比上一接受帧大 1。
- `kRejected` / `kFailed`：诊断里的 `segment_id` 反映**回滚后**的状态
  （seed 门限拒帧时不递增）。

## PnP 初值与 modality-aware 一致性仲裁

正常路径在 `poseInitialValue()` guess 之上，使用摄入前 committed map 与当前
left observation 的 overlap 可选跑 `cv::solvePnPRansac`（PIMPL 内、
`PRIVATE opencv_calib3d`）。positive-disparity 与 left-only observation 均可建立
3D–2D correspondence：

| 条件 | 结果 |
|---|---|
| `enable_pnp_init == false` | `T_W_B = guess`，不 cull；复现 Slice ② |
| `num_mapped_observations < min_pnp_inliers` 或 RANSAC 失败 / inliers 不足 | fallback：`T_W_B = guess`，**不** cull、**不**拒帧 |
| proposal 的 score 无效，或比有效 guess 差超过 `stereo_sigma_px` | fallback：`T_W_B = guess`，保留本帧全部观测，**不**应用 PnP mask |
| proposal 有效，且 guess 无效或 proposal RMS ≤ guess RMS + `stereo_sigma_px` | `T_W_B` 取 PnP；从本帧观测去掉 mapped stereo / mono 外点，unmapped observation 保留 |

首段 seed / re-anchor **不跑** PnP。默认 `pnp_reproj_px=2.0`、
`pnp_confidence=0.99`、`min_pnp_inliers=10`。

PnP 成功只生成 proposal，不直接授权 pose 或 mask。proposal 与 guess 都在 PnP
返回的同一 mapped RANSAC inlier 集上计分。stereo observation 使用未白化
`(uL,uR,v)` residual 的 L2，mono observation 使用 `(u,v)` residual 的 L2；RMS 为
`sqrt(sum(||r_i||^2) / N_inlier_observations)`，每条 inlier observation 只占一个
分母单位。非法 index、非有限投影或 cheirality 令候选 score 无效；proposal 无效
时回退，有效 proposal 可在 guess 无法计分时被采用。`stereo_sigma_px` 是既有观测
噪声，也作为统计等价带，不新增配置或 `config_hash` 输入。只有仲裁采用 proposal
后才应用其 inlier mask；回退不修改 measurement。

**掩码语义**：被掩码的 mapped stereo / mono 外点仍写入 `m_track_times` /
`observationTimestamps()`；`num_observations` 保持测量原值（不是入图观测数）。
unmapped observation 保留；`m_landmarks_w` 与 frontend track 不动——伪永久生命
周期见 Slice ④。

诊断：`UpdateDiagnostics.pnp_success` / `pnp_inliers`；session 汇总
`pnp_successes` / `pnp_fallbacks`，fallback population 使用
`num_mapped_observations`（仅正常路径；seed / re-anchor 不计 fallback）。详见
`docs/research/2026-08-01-note-m3-3-slice3-pnp-design.md`、
`docs/research/2026-08-04-note-m3-3-pnp-stereo-consistency-design.md` 与
`docs/research/2026-08-04-note-m3-3-pnp-stereo-arbitration-results.md`。

## 外点剔除与多轮重优（M3.3 Slice ④ / ④b / ④e）

LM₁ 收敛写回位姿后、返回 `kOk` 前，可选按 landmark **平均 stereo 重投影**
（`||unwhitenedError||` 均值，≥4 观测）从 `m_landmarks_w` 删除高误差点，并经
共用 helper 清窗口观测。`reproj_rms_after_px` **始终**是 LM₁ 后、mean-cull
**前** 的全图 RMS（不受后续 reopt 轮次影响）。

**多轮热路径（Slice ④e）**：每趟 mean-cull / cheirality 必须用**该趟** LM 的
graph + values 打分。若本趟 `culled_round >= 4`（仅 mean-cull 计数；cheirality
不计触发）且 `enable_outlier_reopt`，且已成功轮数 `< max_outlier_reopts`，则
用剩余窗口观测与 `m_landmarks_w` **重建 factor graph** 再跑一趟 LM，写回后再次
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
`m_culled_ids`；`block_culled_rebirth` 时 seed / 正常路径 skip 同 id
backproject。被删 id 的 `m_track_times` / `observationTimestamps()` **仍保留**。
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
`docs/research/2026-08-01-note-m3-3-slice4-outlier-cull-design.md`、
`docs/research/2026-08-01-note-m3-3-slice4b-outlier-reopt-design.md`、
`docs/research/2026-08-02-note-m3-3-slice4c-cull-track-drop-design.md` 与
`docs/research/2026-08-02-note-m3-3-slice4e-multiround-reopt-design.md`。

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
is_keyframe,num_disparity,unsupported_span_ns,num_retained_observations,
num_seeded_landmarks,num_current_visual_factors,num_mapped_observations,
num_current_mono_visual_factors
```

共 **26 列**（Slice ① 在 M2.3 的 13 列尾追加 `segment_id` → 14；Slice ③
再追加 `pnp_success,pnp_inliers` → 16；Slice ④ 再追加
`outliers_culled,reproj_rms_after_cull_px` → 18；Slice ⑤ 再追加
`is_keyframe` → 19；随后追加 `num_disparity` → 20；M4 轨迹连续性在尾部追加
`unsupported_span_ns,num_retained_observations,num_seeded_landmarks,`
`num_current_visual_factors` → 24；mapped-bearing Q1 追加
`num_mapped_observations` → 25；Q4 追加
`num_current_mono_visual_factors` → 26）。
`pnp_success` 为 `0/1` 整数，`is_keyframe` 为 `0/1` 整数。`status` 为
`ok` / `rejected` / `failed`。accepted 非关键帧实际进入 graph/LM，因此优化相关列
（`reproj_rms_before/after`、`num_cheirality`、`lm_iterations`、`outliers_culled`
等）记录真实本帧结果；只有在 staging 前被拒绝的路径保持未执行值。
stdout summary 增加 `total_keyframes` / `total_track_only_frames`。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/stereo_vo_glue.hpp` | `FrameTracks` → `KeyframeMeasurement` |
| `apps/phad_stereo_vo_probe.cpp` | 无窗口验收与基线 |
| `apps/phad_euroc_runner.cpp` | 可视化叠加估计轨迹 |
| `tests/estimator/` | 合成单测 |

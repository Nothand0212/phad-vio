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
| M4.4：静止检测、visual-gyro bias alignment、pose-only AHRS factors、shared gyro bias 与只读 gyro-state diagnostics（`enable_imu` 开关） | accelerometer / gravity / velocity fusion；IMU 原始数据消费（由 sync 切段） |
| 重叠断裂时 re-anchor（`enable_reanchor`），gyro-active 时只跨连续窗口边建 factor | 分段 TUM / Atlas 式多轨迹 |
| 共视 / cheirality / 重投影 / `segment_id` / PnP 诊断 | ATE（`phad::eval`） |
| 正常路径 `solvePnPRansac` proposal + stereo 一致性仲裁 + 本帧 inlier 掩码 | frontend track 生命周期 |
| BA 后 mean-reproj / cheirality 剔点 + 拒同 id 复生 | frontend track 生命周期（由 apps 回传 drop） |
| `body_P_sensor = T_B_left_rectified` | 未校正左目外参 |
| M4.4 probe helper：用 GTSAM AHRS 预积分 raw gyro 段，返回 body-frame rotation evidence | 关键帧 policy、pixel 聚合与 shadow CSV |

## 文件布局

| 文件 | 作用 |
|---|---|
| `types.hpp` | `StereoObservation`、`KeyframeMeasurement`、`VioUpdateResult` 等合同 |
| `imu_rotation.hpp` / `.cpp` | 窄 IMU rotation helper；public API 只暴露 Eigen/POD，GTSAM 留在实现 |
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
                   (static gate → visual alignment → pending → AHRS + stereo BA)
                                                      │
                                                      ▼
                                               VioUpdateResult
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                         OfflineVoSession                 phad_euroc_runner
                    dropTracks(culled ids)                 估计轨迹叠加
                    → probe / phad_vo_bench
```

## Gyro-visual 机制（M4.4）

`enable_imu`（默认 true，进 config_hash；CLI `--no-imu` 关闭）打开后，
estimator 先用纯视觉相对旋转估计一个 shared gyro bias，再在固定窗口 stereo BA
上叠加 pose-only AHRS factors。production graph 不创建 velocity 或 full IMU bias
变量，也不消费 accelerometer；关闭时完整走纯视觉链。

### Non-blocking static gyro audit（M4.4）

estimator 在纯视觉估计旁路累计连续 IMU 段并滑动检测：尾部窗口跨度至少 `0.5 s`，
gyro/accel 逐轴 std 分别小于 `1e-2 rad/s` / `2e-1 m/s²` 才视为静止；
`imu_gap` 清空缓冲后重试，累计超过 `30 s` 仍未通过则返回带原因的
`kFailed`，session 将其升级为 `SessionError`，不静默回退伪初始化。
audit pending 期间视觉帧仍按 VO 合同 seed/update，`init_pending=true` 只用于诊断；
因此 gyro factor 激活前的 accepted pose 与 keyframe feedback 时间轴不被初始化改变。

检测通过后：

- `t0` 是静止窗口起点；`R_W_B0` 把 `normalize(a_mean)` 对齐世界 `+Z`，但只写入
  audit，不回写生产 pose；首个视觉 pose 保持 VO identity gauge；
- `bg_seed = gyro_mean`；acc mean/std、gravity magnitude 与
  `a_mean - imu_gravity * normalize(a_mean)` 只进入一次性 `imu_init` 审计，
  不进入 production graph；
- 不创建无视觉观测的虚拟 `NavState`，也不从 accelerometer 预测 translation；
- raw interval 仍按每个接受 pose pair 进入 alignment evidence；ready 只允许
  alignment/activation 继续，不重播或重锚视觉窗口；
- pending 帧数计入 `summary.json.trajectory.init_pending_frames`，不是丢帧数。

当前 EuRoC checkpoint 的门控值与已知 acc bias 的 scale/tilt 混淆见
[`docs/benchmark/m4.3/README.md`](../../docs/benchmark/m4.3/README.md)。

### 段语义（与 sync 对齐）

- `KeyframeMeasurement.imu_samples` 是本帧与上一帧之间的 IMU 段
  `[t_prev, timestamp]`（`StereoImuPacket` 切段语义）：样本 `i` 覆盖区间
  `[t_i, t_{i+1}]`，右端样本不积分，段内 ΣΔt ≡ 图像间隔；相邻段共享右端
  样本（右端 = 下段左端）。
- 无 IMU 数据源时段恒空且 `imu_gap = true`；该边不建 AHRS factor，视觉
  PnP/恒速链仍可工作。

### pending 拼接（D9）

- 每帧的段**先**追加到 `pending_imu`（共享边界样本去重：与 pending 尾部同
  stamp 时只留一份），`pending_gap |= 本帧 gap`；
- 被拒/失败帧的段保留在 pending，下一帧从最后接受位姿继续预积分（段链不
  因失败帧断裂，拼接后 ΣΔt 不变式仍成立）；
- 成功帧：`candidate.imu_samples = pending`（含本帧段），帧入窗口后 pending
  清空。

### Staged activation 与 shared gyro bias

静止 `bg_seed` 不直接驱动 production factors。接受帧从首个 visual pose 起运行
纯视觉图，并累计 last accepted → current 的视觉相对旋转与原始 IMU 段；static
audit ready 是 activation 的独立前置条件。累计
`imu_gyro_align_window_s`（默认 `100 s`）连续 support 后，一次性最小二乘求 shared
gyro bias 与 visual-vs-gyro rotation residual RMS：

- alignment 所在 update 仍报告 `vision_only` 且 gyro factor count 为 0；
- 只保留当前窗口最长的连续 IMU suffix，并把其首帧改为新 anchor；
- 下一 update 起进入 `gyro_visual`，避免同一张图同时产生 calibration evidence 又
  消费 calibration；
- gap / re-anchor 不贡献 alignment evidence；alignment 失败事务回滚并返回
  `kFailed`。

### 因子图与初值链

- warm-up graph 与 IMU-off graph 都只有 `X/L`、stereo factors 和既有 pose prior；
- gyro-active graph 额外创建一个 `Vector3 W(0)` shared gyro bias，目标是 alignment
  结果，prior sigma 为 `imu_prior_bias_gyro_sigma`；
- 每个连续非 gap 窗口边从 `cur.imu_samples` 即时重建
  `PreintegratedAhrsMeasurements`，再建立
  `PoseAhrsFactor(X_i,X_j,W(0))`；factor 只读取 Pose3 rotation，translation
  Jacobian 为零；
- factor covariance = PIM rotation covariance +
  `I * alignment_residual_rms_rad² / 3`，把实测的相关 visual/gyro mismatch 纳入
  权重，不新增调参键；
- gyro-active 且段可积分时，rotation 初值为
  `R_W_Bi * deltaRij(bg)`；translation 仍取视觉恒速 guess，PnP 始终可参与仲裁；
- optimizer 回写 `X/L/W(0)`，所有窗口帧同步同一个 gyro bias。窗口只从 front
  驱逐，保持 frame pair 与 IMU segment 一一对应。

### Gyro prediction / posterior 只读诊断

成功的 IMU-on update 在 `UpdateDiagnostics::gyro_state` 暴露 Eigen/POD snapshot：
state key、实际积分区间、pre-LM predicted pose 与 gyro bias。posterior pose/bg 分别
取同一个 `VioEstimate` 与 `UpdateDiagnostics::bias_gyro`；GTSAM 类型不越过 PIMPL。

- `gyro_state` 只在 `status=kOk && enable_imu` 时存在；rejected/failed 与 IMU-off
  不伪造状态；
- warm-up / gap 可用 `prediction_valid=false` 表达，不进入 prediction 指标；
- `fusion_mode`、`gyro_factor_count` 与
  `gyro_alignment_residual_rms_rad` 描述实际优化图；
- CLI-only `--vio-state-probe` sidecar 为 69 列，只记录 pose/bg/interval/fusion
  证据，不回写 graph、keyframe selector 或 config hash。

### Fixed-lag read-only shadow（M4.4 P1）

`EstimatorOptions::enable_fixed_lag_shadow` 仅由
`--fixed-lag-shadow-probe <path>` 打开，默认不构造 smoother。它在每次实际
`gyro_visual` 成功 update 的 post-cull window 上维护持久
`IncrementalFixedLagSmoother`：pose set 必须与 production active window 精确一致，
shared `W(0)` 只刷新 timestamp，landmark 使用独立递增的 `q` generation key，所有
caller factor 通过 `newFactorsIndices` 维护 slot ledger。

- dynamic lag 使用 `current_frame_index - window.front().frame_index`；依赖 GTSAM
  cutoff 严格 `<` 的合同保留 front pose；
- 已提交的 landmark factor 不事后追删；cull/消失只退休 generation 并停止刷新，
  旧信息自然到期。因此 shadow 不等价于 production 的 post-cull batch rebuild；
- 每次更新先复制完整 smoother+ledger，在 candidate 上完成 update、slot/orphan、
  pose-set 与 finite 检查后才 swap；异常令 production update 显式失败；
- re-anchor 是唯一允许的显式 `segment` reset；首次 gyro graph 报 `bootstrap`；
- diagnostics/CSV 只读，不参与 prediction、PnP、cull、selector 或下一帧初值，
  不能用 shadow pose 计算“候选 ATE”并宣称 production 精度收益。

### re-anchor 与回滚

- zero-overlap keyframe 仍走既有 re-anchor 质量门；active 时 anchor rotation 可用
  pending gyro 外推，translation 使用视觉恒速 guess；新段首帧不跨边建 factor；
- zero-overlap non-keyframe、empty observations 与 translation support 不足的
  non-keyframe 显式 `kRejected`，不把 gyro-only pose 冒充成功；
- LM1 与每轮 cull reopt 都重建同一 AHRS graph；失败恢复 window/landmarks/shared
  bias，pending IMU 仍保留给下一候选。

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
| landmark backproject | 是 | **不**（只消费既有 landmark） |
| 窗口 push/pop | 是 | 是 |
| buildGraph + LM | 是 | 是 |
| cull + reopt | 是 | 是 |
| 位姿输出 | 接受时 `estimate` 有值 | 接受时 `estimate` 有值 |
| last/prev_accepted | LM 后更新 | LM 后更新 |

非关键帧 `shared < min_pnp_inliers` 时返回 `kRejected`（不更新 last/prev）。所有进入
graph 的候选都使用同一事务 snapshot；失败不残留窗口或 landmark 修改。

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
is_keyframe,num_disparity,bias_gyro_x,bias_gyro_y,bias_gyro_z,
bias_acc_x,bias_acc_y,bias_acc_z
```

共 **26 列**（Slice ① 在 M2.3 的 13 列尾追加 `segment_id` → 14；Slice ③
再追加 `pnp_success,pnp_inliers` → 16；Slice ④ 再追加
`outliers_culled,reproj_rms_after_cull_px` → 18；Slice ⑤ 再追加
`is_keyframe` → 19；pre-M4 诊断追加 `num_disparity` → 20；M4.3 追加 gyro/
acc bias 六列 → 26；均为有意的契约变更）。`num_disparity` 是本帧
`disparity_px > 0` 的观测数，不查 landmark 表。IMU-off 时六个 bias 列恒 0；
gyro-visual 时 gyro 三列为 shared posterior，acc 三列恒 0（static audit 不写主
diag）。
`pnp_success` 为 `0/1` 整数，`is_keyframe` 为 `0/1` 整数。`status` 为
`ok` / `rejected` / `failed`。接受的非关键帧同样进入窗口和 BA，因此其优化相关
列是真实 graph 结果。stdout summary 增加 `total_keyframes` /
`total_track_only_frames`。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/stereo_vo_glue.hpp` | `FrameTracks` → `KeyframeMeasurement` |
| `apps/phad_stereo_vo_probe.cpp` | 无窗口验收与基线 |
| `apps/phad_euroc_runner.cpp` | 可视化叠加估计轨迹 |
| `tests/estimator/` | 合成单测 |

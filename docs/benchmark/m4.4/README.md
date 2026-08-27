# M4.4 诊断与负结果 Benchmark

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-10

状态：**已删除失败的 full-B5 runtime 并恢复 gyro-only B3；fixed-lag P0/P1
结构门 PASS，但 direct production handoff 未获授权**。MH_01 主产物保持 B3
byte-identical，1680-row shadow 无 slot/orphan/lifecycle failure；newest
shadow-vs-batch translation delta p95/max 达 `0.606/0.643 m`，且当前 shadow 不是闭环
反事实。Rule 4、schedule B0/B1、Combined-only、PIM/posterior、translation alignment、
covariance 与 velocity history 均作为负结果保留；MH_05/11 未扩跑，M4.4 尚未完成。

相关：

- issue：[#34](https://github.com/Nothand0212/phad-vio/issues/34)
- predecessor：[M4.3 checkpoint](../m4.3/README.md)
- design：[归档设计](../../research/m4.4-rule4-imu-rotation-design.md)
- diagnosis：[失败候选复盘](../../research/m4.4-rule4-imu-rotation-postmortem.md)
- plan：[实施计划](../../plans/2026-08-09_m4.4_rule4_imu_rotation_d6d46568.plan.md)
- successor design：[Keyframe Epoch Gate](../../research/m4.4-keyframe-epoch-gate-design.md)
- B0 plan：[行为保持 seam](../../plans/2026-08-09_m4.4_keyframe_epoch_gate_b0_7f102cb.plan.md)
- B1 design：[fixed-epoch shadow](../../research/m4.4-keyframe-shadow-probe-design.md)
- B1 result：[shadow probe 结果](../../research/m4.4-keyframe-shadow-probe.md)
- B1 plan：[实施计划](../../plans/2026-08-09_m4.4_keyframe_shadow_probe.plan.md)
- factor contract A：[Combined-only 结果](../../research/m4.4-combined-imu-factor-probe.md)
- state B design：[PIM/posterior 设计](../../research/m4.4-vio-state-probe-design.md)
- state B result：[PIM/posterior 结果](../../research/m4.4-vio-state-probe.md)
- state B plan：[实施计划](../../plans/2026-08-10_m4.4_vio_state_probe_b1a4c6e2.plan.md)
- gyro-first design：[设计定案](../../research/m4.4-gyro-visual-fusion-design.md)
- gyro-first plan：[实施计划](../../plans/2026-08-10_m4.4_gyro_visual_fusion_b.plan.md)
- covariance/bias 归因：[一手资料与 MH_01 sweep](../../research/m4.4-gyro-factor-covariance-reference.md)
- gyro-only recovery：[设计与结果](../../research/m4.4-gyro-only-recovery-design.md)
- gyro boundary objective：[完整 83.95 s 审计](../../research/m4.4-gyro-boundary-factor-reference.md)
- fixed-lag 一手资料：[API 与 lifecycle 对照](../../research/m4.4-gyro-fixed-lag-reference.md)
- fixed-lag P0：[installed-library 合同](../../research/m4.4-gyro-fixed-lag-contract-probe-design.md)
- fixed-lag P1：[设计](../../research/m4.4-gyro-fixed-lag-shadow-design.md) / [结果](../../research/m4.4-gyro-fixed-lag-shadow-result.md)

## 当前晋级候选：gyro-first B

### 身份与结果

| 项 | 值 |
|---|---|
| base code | `7f102cb059ffd87fce5ecd441d3ed26f0fd14c6f`；dirty M4.4 工作区 |
| clean candidate | `m44_gyro_align100_clean_b` / `f6808356` |
| predecessor | `m44_gyro_align100_residual_cov_b` / `ecc76369` |
| VO reference | `b9da3bf/imuoff_c793f690` |
| dataset | EuRoC ASL native `MH_01_easy` |
| execution | 2026-08-10；Release；串行独立 run；wall `184.064 s` |

clean candidate 与 predecessor 的 `est.tum`、`kf.tum`、`diag.csv` 均逐字节相同；
hash 改变只因删除 5 个已无 consumer 的 Combined/V/B 配置键。exact-common gate：

| support | candidate | VO | candidate − VO | verdict |
|---|---:|---:|---:|---|
| poses | 3240 | 3240 | 0 | 同 timestamp |
| ATE translation RMSE (m) | 0.0801837 | 0.0831874 | −0.0030037 | PASS |
| RPE 1 s translation RMSE (m) | 0.0169006 | 0.0183341 | −0.0014335 | PASS |

关联 support 为 matched `3218/3218`、RPE pairs `3198/3198`；completion
`0.879956545`、coverage `0.879923934`、failed `0`、segments/reanchors `1/0`。
state sidecar 共 3240 行，其中 vision-only warm-up 2002 行、gyro-active 1238 行；
gyro factor 累计 11121，alignment residual RMS
`0.000642847647495129 rad`。active ground-truth support 上 prediction/posterior：

| metric | prediction | posterior | posterior − prediction |
|---|---:|---:|---:|
| ATE translation RMSE (m) | 0.0830177 | 0.0830815 | +0.0000638 |
| RPE 1 s translation RMSE (m) | 0.0205961 | 0.0203353 | −0.0002608 |

这说明 gyro factors 的主要已测收益是局部 shape/RPE；全轨迹相对 VO 的 ATE
改善不能从 prediction→posterior 单项因果化。probe correction RMS 为 pose
`0.0041304 m`、rotation `0.0165603°`、shared bg `1.30e-10 rad/s`。

### IMU-off sentinel

clean code 的 `m44_gyro_clean_imuoff_sentinel` / `eb94f804` 与冻结 reference
`b9da3bf/imuoff_c793f690` / `c793f690` 的 `est.tum`、`kf.tum`、`diag.csv`
逐字节相同。两者都是 3681 poses、completion `0.999728408`、coverage `1.0`、
failed `0`；full-support ATE/RPE 分别为 `0.080964058 / 0.017781219 m`。hash 差异
只来自本片删除的无 consumer 配置键；在 VIO 的 3240 exact-common support 上，
该 sentinel 仍给出 VO ATE/RPE `0.0831874 / 0.0183341 m`，首门结论不变。

### MH_05 首个扩序列门：FAIL

配对 run 路径：

- VIO：`MH_05_difficult/7f102cb_dirty/m44_gyro_clean_b`，hash `f6808356`；
- VO：`MH_05_difficult/7f102cb_dirty/m44_gyro_clean_imuoff`，hash `eb94f804`。

全程 exact-common 2261 poses、matched `2220/2220`、RPE pairs `2199/2199`：

| metric | VIO | VO | VIO − VO | verdict |
|---|---:|---:|---:|---|
| ATE translation RMSE (m) | `0.335684` | `0.324107` | `+0.011577` | FAIL |
| RPE 1 s translation RMSE (m) | `0.0413833` | `0.0440090` | `-0.0026257` | PASS |

VIO/VO completion 分别 `0.994721 / 0.999560`，coverage `0.995158 / 1.0`，均无
estimator failure。按首失败停止条件，未扩跑其余序列。

phase split 显示 gyro graph 激活前 2001 poses 已 ATE FAIL，而激活后的 260 poses
ATE/RPE 都 PASS。VIO 的首个 `kOk` 时间比 VO 晚 0.55 s，原因是 static init 拒绝
前 11 个视觉帧；gyro graph 直到约 100 s 后才开始建 factor。因此现有 MH_01 PASS
混有 initialization schedule 效应，下一候选必须先做 non-blocking init 并重新门控。

### B2 non-blocking init：MH_01 FAIL

550 帧 sentinel 跨过 static ready：VIO/VO 的 `est.tum`、`kf.tum` 和 schedule
投影逐字节相同。全程 B2 `m44_gyro_nonblocking_b2/f6808356` 在 3681
exact-common poses 上 ATE/RPE 为 `0.0835455/0.0170120 m`，VO 为
`0.0809641/0.0177812 m`：ATE `+0.0025814` FAIL，RPE `-0.0007692` PASS。

gyro 激活前 2001 poses 两者完全相等；active 1680 poses 的 graph-initial probe
测得 prediction→graph-initial→posterior ATE 为
`0.0926101→0.0926028→0.0926759 m`，RPE 为
`0.0194998→0.0195770→0.0192903 m`。这把主要 ATE 伤害定位到 gyro-seeded
normal-frame PnP/initialization 级联；B3 只恢复 visual initialization ownership，
factor 模型保持不变。

### B3 visual initial ownership：MH_01 仍 FAIL

B3 run `m44_gyro_visual_init_b3/f6808356` 与 paired VO
`m44_gyro_clean_imuoff_sentinel/eb94f804` 在 3681 exact-common poses 上：

| metric | VIO | VO | VIO − VO | verdict |
|---|---:|---:|---:|---|
| ATE translation RMSE (m) | `0.0833210` | `0.0809641` | `+0.0023569` | FAIL |
| RPE 1 s translation RMSE (m) | `0.0168992` | `0.0177812` | `-0.0008820` | PASS |

completion `0.999728408`、coverage `1.0`、failed `0`。active suffix 的
prediction/graph-initial/posterior ATE 为
`0.0926003/0.0926168/0.0926643 m`，RPE 为
`0.0192969/0.0193894/0.0190702 m`。B3 消除了 gyro-seeded PnP 所有权问题，
但没有消除 translation ATE 回退。

### bias 与 AHRS covariance 归因：均否决

rolling visual/gyro alignment 显示 0.5 s bias error 中位数 `3.081e-3 rad/s`、
estimate step 中位数 `4.322e-3 rad/s`，相对 reference step 约 `1e-6 rad/s`；当前
10-frame window 不支持 per-frame bias state。

AHRS residual covariance 的 9 点 probe-only sweep 中，最小 ATE 是 `scale=1.2` 的
`0.0825194 m`，仍高于 VO `0.0809641 m`；`scale=0..1.3` 虽均改善 RPE，也没有
双指标通过点，`1.5/2` 连 RPE 也失败。按首门停止，未跑 MH_05。临时 scale 配置、
CLI 与测试已经删除，不属于当前产品接口。

reference rotation audit 仍确认 gyro measurement 本身更准：fixed aligned bias 的
interval rotation RMS `3.436e-4 rad`，VO 为 `5.575e-4 rad`。因此下一步核对
translation inertial state，而不是继续调 pose-only factor 权重。

### B5 撤回、B3 恢复与完整 boundary objective

raw B5、calibrated B6 与 frozen-`ba` 的 MH_01 ATE/RPE 分别为
`0.249563/0.0186294`、`0.0963460/0.0181547`、`0.691356/0.0334161 m`，均双门
FAIL；causal velocity history 亦退化。production 因此删除 full alignment、
`ImuFactor`、`V/B`、gravity/velocity/acc-bias state 与 full-only配置，而不是保留
runtime disable 兼容层。

恢复 run `m44_gyro_only_recovery_b3a1c0de/f6808356` 的 `est.tum`、`kf.tum`、
`diag.csv` 与 B3 逐字节相同，ATE/RPE 仍为
`0.0833209600/0.0168992426 m`；state sidecar 为 2001 rows vision-only + 1680 rows
gyro-visual，objective 全部有效。

完整 gyro suffix 上，boundary whitened norm median/p90/max=
`0.334/0.594/2.678σ`，仅 `29.82%` rows 高于 interior 单因子均值；八个完整 10 s
桶的 boundary max 均 `<1.11σ`。最大 ATE 坏桶的 boundary max 反而只有
`0.887σ`，不满足与 boundary outlier 同相的必要条件。因此关闭 boundary
Huber/information-scale/threshold 路线，不扩 MH_05。

### 完整 runtime snapshot

以下逐字取自 clean candidate `meta.json.config_canonical_text`：

```text
estimator.block_culled_rebirth=true
estimator.enable_imu=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.imu_gravity=9.8100699999999996
estimator.imu_gyr_noise_nd=0.00016967999999999999
estimator.imu_gyro_align_window_s=100
estimator.imu_init_accel_std=0.20000000000000001
estimator.imu_init_gyro_std=0.01
estimator.imu_init_timeout_s=30
estimator.imu_init_window_s=0.5
estimator.imu_prior_bias_gyro_sigma=9.9999999999999995e-07
estimator.imu_prior_pose_sigma=0.0001
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
estimator.use_constant_velocity_init=true
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

## 1. 身份与执行

| 项 | 值 |
|---|---|
| base code | `7f102cb059ffd87fce5ecd441d3ed26f0fd14c6f`（short `7f102cb`） |
| control | `m44_control_ba30` / `d70e45aa` |
| candidates | `m44_source_imu30` / `5b289f7d`；`m44_source_imu15` / `4018fae4`；`m44_source_imu10` / `11639aad` |
| rollback verify | `m44_rollback_control_verify` / `d70e45aa` |
| successor B0 | `m44b_b0_epoch_gate_verify` / `d70e45aa` |
| successor B1 shadow | `m44b_shadow_probe` / `d70e45aa` |
| dataset | EuRoC ASL native `MH_01_easy` |
| dataset path | `/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy` |
| bench root | `/home/lin/Projects/data/phad-bench` |
| execution | 2026-08-09；Release；串行独立 run |
| gate | candidate ATE `<=` 同源 control；completion 不降；segments/reanchors/failed=`1/0/0` |

候选 run 位于 `7f102cb_dirty`：dirty 内容是尚未提交的 M4.4 production candidate
及文档，raw `meta.json` 如实记录这一身份。control 与 rollback verify 位于
`7f102cb`。没有把 dirty 产物描述成 clean baseline。

命令模板：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --bench-root /home/lin/Projects/data/phad-bench \
  --config-label <label> \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio
```

15px、10px 候选分别追加：

```text
--keyframe-parallax-px 15
--keyframe-parallax-px 10
```

## 2. 配置快照

### 2.1 相对 control 的增量键

| 键 | control | IMU30 | IMU15 | IMU10 |
|---|---:|---:|---:|---:|
| `session.keyframe_parallax_px` | 既有常量 30，未入 config | 30 | 15 | 10 |
| `session.keyframe_min_frames_after_kf` | 无 | 0 | 0 | 0 |
| Rule 4 rotation | accepted BA-pose delta | last-KF→current gyro | last-KF→current gyro | last-KF→current gyro |

除上表外，候选 runtime config 与 control 相同；cooldown 始终关闭。

### 2.2 control 完整 `config_canonical_text`

以下直接取自 `m44_control_ba30_d70e45aa/meta.json`：

```text
estimator.block_culled_rebirth=true
estimator.enable_imu=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.imu_acc_noise_nd=0.059999999999999998
estimator.imu_acc_rw=0.0030000000000000001
estimator.imu_gravity=9.8100699999999996
estimator.imu_gyr_noise_nd=0.00016967999999999999
estimator.imu_gyr_rw=1.9392999999999999e-05
estimator.imu_init_accel_std=0.20000000000000001
estimator.imu_init_gyro_std=0.01
estimator.imu_init_timeout_s=30
estimator.imu_init_window_s=0.5
estimator.imu_prior_bias_acc_sigma=0.029999999999999999
estimator.imu_prior_bias_gyro_sigma=0.10000000000000001
estimator.imu_prior_pose_sigma=0.0001
estimator.imu_prior_vel_sigma=1
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
estimator.use_constant_velocity_init=true
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

### 2.3 IMU30 完整 `config_canonical_text`

以下直接取自 `m44_source_imu30_5b289f7d/meta.json`。IMU15/IMU10 只把
`session.keyframe_parallax_px` 改成 `15`/`10`，对应 hash 如 §1。

```text
estimator.block_culled_rebirth=true
estimator.enable_imu=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.imu_acc_noise_nd=0.059999999999999998
estimator.imu_acc_rw=0.0030000000000000001
estimator.imu_gravity=9.8100699999999996
estimator.imu_gyr_noise_nd=0.00016967999999999999
estimator.imu_gyr_rw=1.9392999999999999e-05
estimator.imu_init_accel_std=0.20000000000000001
estimator.imu_init_gyro_std=0.01
estimator.imu_init_timeout_s=30
estimator.imu_init_window_s=0.5
estimator.imu_prior_bias_acc_sigma=0.029999999999999999
estimator.imu_prior_bias_gyro_sigma=0.10000000000000001
estimator.imu_prior_pose_sigma=0.0001
estimator.imu_prior_vel_sigma=1
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
estimator.use_constant_velocity_init=true
estimator.window_size=10
eval.max_dt_ms=2.5
eval.min_match_rate=0.5
eval.rpe_delta_s=1
session.dataset_format=euroc
session.drop_culled_tracks=true
session.keyframe_min_frames_after_kf=0
session.keyframe_parallax_px=30
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

successor B1 相对本快照没有配置增量；CLI-only 参数为
`--keyframe-shadow-probe /home/lin/Projects/data/phad-bench/probes/m44b_mh01_shadow_7f102cb.csv`，
不进入 `config_hash`。

## 3. MH_01 质量表

`total KF candidates` 包含初始化阶段 442 个被拒帧；`accepted KF` 为其扣除这
442 帧后的实际 accepted count。

| run | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | failed | total KF candidates | accepted KF |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control BA30 | completed_with_warnings | 0.122337397 | 0.022956790 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 960 | 518 |
| IMU30 | completed_with_warnings | 0.138622478 | 0.022833546 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 762 | 320 |
| IMU15 | completed_with_warnings | 0.155501216 | 0.022261365 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 848 | 406 |
| IMU10 | completed_with_warnings | 0.132913217 | 0.022121573 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 975 | 533 |
| rollback verify | completed_with_warnings | 0.122337397 | 0.022956790 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 960 | 518 |
| successor B0 | completed_with_warnings | 0.122337397 | 0.022956790 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 960 | 518 |
| successor B1 shadow | completed_with_warnings | 0.122337397 | 0.022956790 | 0.879956545 | 0.879923934 | 1 | 0 | 0 | 960 | 518 |

所有 run 均为 ok/image=`3240/3682`、rejected=`442`。既有 warning 是
PnP fallback summary 口径，不是新增运行失败。

## 4. Robustness

| run | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| control BA30 | 66 / 66 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |
| IMU30 | 59 / 59 | 1 | 1 | 1 / 4 | 0 / 3239 | 0 | 0 |
| IMU15 | 62 / 62 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |
| IMU10 | 54 / 54 | 1 | 1 | 1 / 4 | 0 / 3239 | 0 | 0 |
| rollback verify | 66 / 66 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |
| successor B0 | 66 / 66 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |
| successor B1 shadow | 66 / 66 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |

## 5. 相对 control

| candidate | ATE Δ (m) | ATE Δ (%) | RPE Δ (m) | completion Δ | accepted KF Δ | 判读 |
|---|---:|---:|---:|---:|---:|---|
| IMU30 | +0.016285081 | +13.31% | −0.000123244 | 0 | −198 | FAIL；明显变稀 |
| IMU15 | +0.033163819 | +27.11% | −0.000695425 | 0 | −112 | FAIL；ATE 非单调变差 |
| IMU10 | +0.010575820 | +8.64% | −0.000835217 | 0 | +15 | FAIL；总量恢复仍未恢复时序/ATE |
| successor B0 | 0 | 0 | 0 | 0 | 0 | PASS；行为保持 seam，不是精度候选 |
| successor B1 shadow | 0 | 0 | 0 | 0 | 0 | PASS；只读 probe，不是精度候选 |

IMU10 与 control 只有 133 个 accepted KF timestamp 精确重合；mean gap 分别为
`0.3042 s` 与 `0.3131 s`。因此不能把 ATE 差异简单归因为总 KF 数或平均间隔。

## 6. 判定与未执行范围

- source gate：**FAIL**；三条候选 ATE 都劣于 control；
- 原 M4 绝对门 `ATE <= 0.100 m`：control 与候选均 **FAIL**；
- completion/coverage 与结构门保持，但不能抵消 ATE 回归；
- 30/15/10px 的 RPE 都略优，只能视为局部指标变化，不能据此晋级；
- cooldown：只有 B1 frozen-epoch veto 计数，**未运行 live candidate**；
- `MH_01/MH_05/V1_01` sentinel：**未运行**；
- EuRoC 11/11：**未运行**；
- 没有试 5px、动态阈值或 threshold × cooldown 组合。

## 7. 完整性与回退核验

| 检查 | 结果 |
|---|---|
| command rc | 6/6 run 完成；均 `completed_with_warnings` |
| artifacts | 每个 run 均有 `meta.json`、`summary.json`、`est.tum`、`kf.tum`、`diag.csv` |
| code identity | control/rollback=`7f102cb`；候选=`7f102cb_dirty`，未伪装 clean |
| config identity | `d70e45aa` / `5b289f7d` / `4018fae4` / `11639aad` 与 raw meta 一致 |
| canonical config | control 与 IMU30 完整文本见 §2；15/10 单键增量可复现 |
| production rollback | tracked diff 为空；restored unit PASS |
| behavior rollback | control 与 rollback verify 的 ATE/RPE/trajectory/robustness 相同 |
| successor B0 | control 与 B0 的三文件 `cmp=0`；schedule exact=100%，rolling Δ=0 |
| successor B1 shadow | 3682 frame rows / 518 committed epochs；timestamp/selected axis exact；三主文件相对 B0 `cmp=0` |
| 未执行项 | live guard、sentinel、full-suite；shadow 没有支持 guard 晋级 |

关键产物 MD5：

| run | `est.tum` | `kf.tum` | `diag.csv` |
|---|---|---|---|
| control | `93318dc194cf1be06f580d2ef55188aa` | `261664a00835a97ff368ec96037caa6d` | `71bdff9377e4c0073bd7127fed032581` |
| IMU30 | `252b301618451e5135959f80f360facd` | `1404a5ff30046007f9ff87c2fcba0b86` | `10fb2bc387e2650bef55415bee9f745c` |
| IMU15 | `032b471fa669ec2a978597f24b33ccc3` | `36c5ea5479ca4659d0e400112cca62c3` | `d0d1bbce76e6ee8f1a9e96c8e2b325de` |
| IMU10 | `e09388afb296582813ab1af02a831248` | `f497ec8062cca79ce8d54223e2aaf3b3` | `49185f3003d718236222a1c98a4fc02e` |
| rollback verify | `93318dc194cf1be06f580d2ef55188aa` | `261664a00835a97ff368ec96037caa6d` | `71bdff9377e4c0073bd7127fed032581` |
| successor B0 | `93318dc194cf1be06f580d2ef55188aa` | `261664a00835a97ff368ec96037caa6d` | `71bdff9377e4c0073bd7127fed032581` |
| successor B1 shadow | `93318dc194cf1be06f580d2ef55188aa` | `261664a00835a97ff368ec96037caa6d` | `71bdff9377e4c0073bd7127fed032581` |

raw artifacts：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb/m44_control_ba30_d70e45aa
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/m44_source_imu30_5b289f7d
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/m44_source_imu15_4018fae4
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/m44_source_imu10_11639aad
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb/m44_rollback_control_verify_d70e45aa
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/m44b_b0_epoch_gate_verify_d70e45aa
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/m44b_shadow_probe_d70e45aa
/home/lin/Projects/data/phad-bench/probes/m44b_mh01_shadow_7f102cb.csv
```

## 8. Successor B0 行为保持门

B0 新增 `apps::KeyframeEpochGate`，把匿名 selector 提取为
`decide -> estimator.update -> resolve` transaction。threshold、rotation source、
config、CLI 与 CSV schema 均未改变。B0 run 的 `meta.json.config` 和
`config_canonical_text` 与 §2.2 的 control 快照逐项相同，hash 仍为 `d70e45aa`。

实际验证：

- Release build PASS；production-interface tests `11/11`；apps tests
  `28 PASS + 3 SKIP`；unit `353/353 PASS`；
- ATE/RPE 精确为 `0.12233739677110454 / 0.022956790060637233 m`；
- completion/coverage 精确为
  `0.8799565453557849 / 0.8799239336301563`；
- ok/rejected/image=`3240/442/3682`，accepted KF=`518`，
  segments/reanchors/failed=`1/0/0`；
- accepted schedule 相对 control exact/±1/±2 均 `100%`，无首次分叉，
  10-frame rolling max `|Δ|=0`；
- raw run 如实标为 `7f102cb_dirty`，未把工作区结果伪装为正式 clean baseline。

B0 只证明 seam 等价。它既没有改善当前 VIO ATE，也没有满足纯 VO 的
`0.080964 / 0.017781 m` 锚；B1 fixed-production-schedule shadow 结果见下一节。

## 9. Successor B1 fixed-epoch shadow 门

B1 新增 CLI-only `KeyframeShadowProbe`。production selector 仍是 B0 `pose_lag`；
sidecar 在同一 accepted-KF epoch 上额外计算 IMU rotation parallax，不改变 estimator
输入或 config identity。

实际验证：

- Release 受影响目标构建通过；helper `5/5`、gate/shadow/CLI `28/28`、Python
  `4/4`；unit 为 364 total、`361 PASS + 3 SKIP`、0 fail；
- sidecar 为 3683 行（header + 3682 frame），MD5
  `4d7a7a1822077ba5e88659611171e656`；
- timestamp 与 `diag.csv`、`prod_selected` 与 `is_keyframe` 均逐行 exact；
- post-init IMU evidence `3239/3239` valid，无 gap/helper invalid；
- 10/15/30px frozen candidate add/remove 分别为 `556/219`、`234/278`、
  `3/350`；10/15px isolated hit 仅 `38/690`、`28/309`；
- Basalt-family strict `N=5` 会 veto 10px `344/690`、15px `113/309` 个
  `frames_since_kf<=6` hit，是大范围 rescheduling；
- 因此 two-hit、hysteresis、hard cooldown 与 dual-evidence veto 均未获机制证据，
  没有实现 live guard。

本门只表示 B1 probe 完成，不表示 M4.4 完成。当前 VIO 仍劣于纯 VO 锚；下一刀需
重新对齐，不能自动进入 B2 source replacement。

## 10. Combined-only factor contract A

A 删除了 `CombinedImuFactor` 外重复的 bias `BetweenFactor`，没有改变配置、
schedule、窗口、prior 或初始化。MH_01 相对 B1 control：

| metric | B1 control | Combined-only A | Δ |
|---|---:|---:|---:|
| ATE translation RMSE / m | 0.122337396771 | 0.122336126333 | −0.000001270439 |
| RPE translation RMSE / m | 0.022956790061 | 0.022956666710 | −0.000000123350 |
| completion / coverage | 0.879956545356 / 0.879923933630 | 同左 | 0 |
| accepted KF | 518 | 518 | 0 |

该修复按 GTSAM `CombinedImuFactor` 类型合同保留，但效果量不足；共同 support VO
仍为 `0.0831874 / 0.0183341 m`，双指标 FAIL。配置继续使用 §2.2 完整快照与
`d70e45aa`；raw run：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/
  m44_bias_transition_a_d70e45aa
```

完整 bias 与主产物对拍见链接中的 A 结果文档。

## 11. PIM prediction / posterior state B

B 仅增加 CLI-only state sidecar；与 A 的 `config_canonical_text`、hash、
`est.tum`、`kf.tum`、`diag.csv` 均逐字相同。3240 accepted rows 全部有有效 PIM
prediction；过滤 GT span 后有 3218 evaluator rows：

| metric | PIM prediction | posterior | posterior − prediction |
|---|---:|---:|---:|
| ATE translation RMSE / m | 0.122469 | 0.122336 | −0.000133 |
| RPE translation RMSE / m | 0.0235555 | 0.0229567 | −0.0005988 |

32 个完整 100-pose 桶中 posterior ATE 改善 `32/32`，RPE 改善 `31/32`；说明
optimizer correction 大体在救 prediction，而不是主要退化源。posterior 仍比共同
support VO 差 `+0.0391486 / +0.0046226 m`，所以 B 只完成根因选刀，不完成
M4.4。raw run：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/
  m44_vio_state_probe_b_d70e45aa
```

下一片只建议先做 initialization truth audit；尚无证据授权 fixed-lag、视觉策略或
online joint alignment 的 production 修改。

## 12. Fixed-lag P0/P1 read-only shadow

P0 用 installed GTSAM 定向锁定 cutoff、global separator、factor slot、timestamp
orphan 与 full-snapshot rollback 五类合同，`5/5 PASS`。P1 在 estimator PIMPL 内增加
默认关闭的 persistent `IncrementalFixedLagSmoother` shadow；CLI-only 30 列 sidecar
不反馈 production pose/bias/PnP/cull/selector/config。

MH_01 run：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7f102cb_dirty/
  m44_fixed_lag_shadow_p1_6e8a21d4
config_hash=f6808356
```

`est.tum`、`kf.tum`、`diag.csv` 与
`m44_gyro_only_recovery_b3a1c0de` 逐字节相同；3681 sidecar rows 中
inactive/active=`2001/1680`，bootstrap=`1`、segment reset=`0`，max batch/shadow pose
均为 `10`，missing owned slot/timestamp orphan=`0/0`。完整 unit `410/410`、0 fail。

shadow lifecycle 保持有界，但 estimate 持续与 rolling batch 分离：

| delta | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| rotation / rad | `0.03100` | `0.05544` | `0.06016` | `0.06204` |
| translation / m | `0.20814` | `0.60649` | `0.63478` | `0.64347` |

active suffix 只有 5 次 production mean-cull；3172 次 generation retirement 中 1180
个 nonzero-retirement rows 没有 same-row cull，translation delta 与 cull 的相关系数仅
`-0.011`。因此不能用“mean-cull 差异”解释持续分离，也不能把 batch-driven shadow pose
拼成有效 ATE。P1 结构门 PASS、direct handoff 否决；下一片需重新对齐
candidate-owned closed-loop shadow、直接 handoff 或停止 fixed-lag。完整参数快照、hash、
bucket 与证据边界见 [P1 结果](../../research/m4.4-gyro-fixed-lag-shadow-result.md)。

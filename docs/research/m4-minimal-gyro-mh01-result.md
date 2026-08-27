# M4 最小 gyro-aided VO：MH_01 结果

日期：2026-08-12

状态：**机制成立；Fused 硬门失败；本计划以负结果停止**

计划：[`c4e62b35`](../plans/2026-08-12_m4_minimal_gyro_slice_c4e62b35.plan.md)

跟踪：[#36](https://github.com/Nothand0212/phad-vio/issues/36)

## 结论

命名计划规定的四个 slice 已执行到停止条件：

- `off` 保持原 production VO 路径；`est.tum`、`kf.tum`、`diag.csv` 与实施前
  clean control 逐字节一致；
- `shadow` 的 segment ledger、shared absolute gyro bias、fixed-bias AHRS factor、
  独立 posterior 与 sidecar 均产生有效证据，且三项视觉主产物仍与 `off`
  逐字节一致；
- `fused` 的自然闭环运行无新增失败，completion、GT coverage 与输出 pose 数未退化；
- 但 exact-common support 上 translation ATE RMSE 从 `0.0809641 m` 恶化到
  `0.0908066 m`（`+12.16%`），1 s translation RPE RMSE 从 `0.0177812 m`
  恶化到 `0.0180957 m`（`+1.77%`）。两项“严格改善”条件均失败。

因此，结论是：**最小 fixed-bias gyro rotation constraint 的机制可工作，但写回 M3
自然闭环不能改善 MH_01。** 按计划保持 `gyro_mode=off` 默认值，不扫 noise、bias、
factor 权重或视觉阈值，不运行 MH_05 或 EuRoC 11/11。是否进入 full inertial
`X/V/B + gravity` 必须另行评审，不属于本计划。

## 运行身份与命令

数据集与三个运行目录：

```text
dataset=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
off=/home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice3_off
shadow=/home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice3_shadow
fused=/home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice4_fused
control=/home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control
code=a8e892f (dirty implementation checkout)
build_type=Release
```

运行命令除输出目录和 mode 外相同：

```bash
build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --gt-euroc /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out <RUN_DIR> --gyro-mode <off|shadow|fused> --force

python3 scripts/vio_vo_common_support.py \
  /home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice3_off \
  /home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice4_fused \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --output /home/lin/Projects/data/phad-bench/MH_01_easy/a8e892f_dirty/m4_minimal_gyro_slice4_fused/exact_common_report.json
```

三次运行均为 `completed_with_warnings`。warning 只包含如实记录的 dirty checkout
和既有 PnP summary；`failed=0`。wall time 分别为：`off=145.019 s`、
`shadow=148.886 s`、`fused=141.698 s`。本片不设性能门。

## Runtime 参数快照

以下参数来自三个 run 各自 `meta.json.config`，并与相应 `config_hash` 同源。三者
相对 Slice 0 control 唯一新增的配置键是 `estimator.gyro_mode` 与
`estimator.gyro_align_window_s`；三次运行除 mode 外完全相同。

| run | `estimator.gyro_mode` | `estimator.gyro_align_window_s` | `config_hash` |
|---|---|---:|---|
| off | `off` | `100.0` | `402fc9dc` |
| shadow | `shadow` | `100.0` | `44261e63` |
| fused | `fused` | `100.0` | `d3ae38ff` |

共同配置的完整 canonical snapshot（`estimator.gyro_mode=<mode>` 按上表替换）：

```text
estimator.block_culled_rebirth=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.gyro_align_window_s=100
estimator.gyro_mode=<mode>
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

## Off 与 Shadow 门

### 字节冻结

Slice 0 control、当前 `off` 与当前 `shadow` 的三个主产物 SHA-256 相同，且已逐项
执行 `cmp`：

```text
18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321  est.tum
4a4a1ba8f7fb2729e482d5aca644c9195ad605ccc0e94ffb9e1e670d1f898bb9  kf.tum
1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb  diag.csv
```

`off` 不生成 `gyro_state.csv`。`shadow` 生成 3682 个数据 row，identity/status
与 `diag.csv` 对齐；视觉主产物不受 shadow solve 影响。

### Mechanism 证据

| 项 | 实测值 |
|---|---:|
| 首次 `align_ready`（ns） | `1403636679813555456` |
| 首个 factor（ns） | `1403636679863555584` |
| alignment support / edges | `100.049999872 s / 2000` |
| rank / condition | `3 / 1.0000021656` |
| frozen bias (rad/s) | `(-0.00298738, 0.02130640, 0.07866176)` |
| alignment RMS (rad) | `0.0008771300` |
| valid gyro edges | `3680` |
| posterior rows | `1680` |
| factor count 范围 | `1..7` |

与 MH_01 GT 初始 gyro bias 约 `(-0.00317, 0.02127, 0.07850) rad/s` 相比，
估计差值为约 `(1.83e-4, 3.64e-5, 1.62e-4) rad/s`，三维范数
`2.47e-4 rad/s`。这只作为外部诊断，不进入算法，也不设 hard tolerance。

当前 edge 使 alignment ready 时没有 retroactive factor；首个 factor 严格出现在下一条
accepted edge。所有已记录 bias、residual、cost、pose、quaternion 与 PIM covariance
均为 finite；integer-ns interval 与 `pim_dt_s` 的最大浮点表示误差为约
`4.16e-17 s`。

为验证 shadow correction 不是空机制，另在 1680 个 posterior support timestamp 上构造
固定观测支持评估：off ATE/RPE 为 `0.0793033 / 0.0207739 m`，shadow posterior 为
`0.0792453 / 0.0206521 m`。两项略有改善，仅用作进入 Fused 的机制证据；不替代自然
闭环 hard gate。对应派生产物位于 shadow run 目录：`shadow_post.tum`、
`off_on_shadow_support.tum` 与 `off_shadow_common_support.json`。

## Fused exact-common 硬门

off 与 fused 各写 3681 个 pose，exact timestamp 交集也是 3681。以同一交集向 GT
做最近邻关联（门限 `2.5 ms`），得到 3637 个共同 GT match，丢弃 44 个；最大关联
时间差为 `256 ns`。1 s RPE 使用 3616 个共同 pair，另有 21 个 timestamp 无合格终点。

| translation 指标 | off (m) | fused (m) | 结论 |
|---|---:|---:|---|
| ATE RMSE | `0.0809641` | `0.0908066` | **恶化 12.16%** |
| ATE p50 / p95 / max | `0.0613758 / 0.1451273 / 0.1961070` | `0.0683816 / 0.1727752 / 0.2228976` | 均恶化 |
| RPE RMSE | `0.0177812` | `0.0180957` | **恶化 1.77%** |
| RPE p50 / p95 / max | `0.0121735 / 0.0348506 / 0.0801791` | `0.0124121 / 0.0359193 / 0.0801791` | p50/p95 恶化，max 持平 |
| adjacent jump p50 / p95 / max | `0.0221656 / 0.0446548 / 0.1038657` | `0.0221602 / 0.0445837 / 0.1038657` | 无新增大跳 |

其余 hard-gate 条件全部通过：

- `failed=0`，未增加失败；
- poses `3681 == 3681`；completion `0.9997284085 == 0.9997284085`；
  coverage `1.0 == 1.0`；
- factor coverage 为 `1680 / 1681 = 0.9994051160`；
- KF schedule 首次分叉 index 为 `422`：off timestamp
  `1403636683113555456`，fused timestamp `1403636683163555584`；最终 KF 数
  `662 → 663`，符合自然闭环允许分叉的合同；
- adjacent jump max 与 off 相同，没有以单帧大跳换取均值。

但 ATE、RPE 两个核心精度条件都失败，所以 `gate.core_pass=false`。权威机器可读报告在
Fused run 目录的 `exact_common_report.json`。

## 验证与限制

实现阶段实际通过：

- `phad_estimator_tests`：92/92；
- `phad_apps_tests`：33 passed、3 skipped；三个 skip 是未设置
  `PHAD_EUROC_MH01_PATH` 的可选数据集门，真实 MH_01 已由上述三次系统运行覆盖；
- `ctest --test-dir build -L unit --output-on-failure -j2`：128 项无失败，
  其中同样 3 项按上述条件 skipped；
- `python3 -m unittest tests/scripts/vio_vo_common_support_test.py -v`：1/1；
- 临时 venv 中重跑同一 Python test，并重算真实 exact-common report；新报告与保留报告
  逐字节一致；
- `phad_estimator_tests`、`phad_apps_tests`、`phad_vo_bench` Release build；
- off/control 与 off/shadow 的三主产物 `cmp`；
- exact-common evaluator 在 off=shadow 的相同轨迹上与 C++ summary 指标交叉验证到
  浮点精度。

限制与未执行项：

- feature runs 来自 dirty implementation checkout，不能作为 clean-commit 正式 baseline；
  但 code identity、完整 config、产物路径与校验值均已保存；
- 未运行 Sanitizer；
- 因 Fused hard gate 失败，按计划**未运行** MH_05、EuRoC 11/11，也未做任何调参；
- 未 commit、merge 或 push。

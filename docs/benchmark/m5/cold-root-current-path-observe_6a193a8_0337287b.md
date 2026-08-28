# M5 cold-root current-path Observe

状态：**Observe PASS**。`V2_03_difficult` terminal `96.6–116.7 s` 的 289 行全部最早
阻塞于 `stereo_population/population_insufficient`：bootstrap moving path 与 keyframe
gate 已通过，current-packet effective seed population 始终小于现有 threshold `10`。
terminal 没有真实 `seedRoot()` 调用，attempt count 为 `0`。

该结论只建立 current-path 可观测性。#47 formal product gate 继续为 **FAIL**；本结果不表示
root 已恢复，也不解决跨-root world-frame alignment。

## 1. 身份与执行范围

| 项 | 值 |
|---|---|
| issue / frozen spec | [#48](https://github.com/Nothand0212/phad-vio/issues/48) / [`2026-08-28-m5-cold-root-seed-observe.md`](../../specs/2026-08-28-m5-cold-root-seed-observe.md) |
| candidate | `6a193a8c4467f441ebbde1eda3b4b1250d32bf02`（`6a193a8`） |
| tree / parent | `306a7aef081d3365894a08391224a53a1a549fa3` / `e6705d57d3959abcb6b273db6a32c0d63298e0ef` |
| source | `codex/m5-cold-root-observe`，`git_dirty=false` |
| config | `default_0337287b`；canonical text 与 control 完全一致 |
| dataset | EuRoC ASL native `V2_03_difficult` |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native/V2_03_difficult` |
| frozen control | `db22656` artifact `/home/lin/Projects/data/phad-bench/m4-mapped-bearing-final-db22656-20260828T022252Z/V2_03_difficult` |
| candidate artifact | `/home/lin/Projects/data/phad-bench/m5-cold-root-observe-20260828T070950Z/V2_03_difficult` |
| run time | `2026-08-28T07:10:01Z` |
| build | Release；GNU 13.3.0；CMake 3.31.11；Ninja 1.13.0；OpenCV 4.6.0；GTSAM 4.3a0 |
| evidence | [完整审计 JSON](cold-root-current-path-observe_6a193a8_0337287b.evidence.json) |

本次只运行一次 candidate；未重跑 control，未运行 MH_01、其它 EuRoC、EuRoC-11 或
TUM VI。

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/V2_03_difficult \
  --out /home/lin/Projects/data/phad-bench/m5-cold-root-observe-20260828T070950Z/V2_03_difficult \
  --sequence-name V2_03_difficult \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-observe \
  --estimator-enable-moving-bootstrap
```

命令 `rc=0`，终态 `completed_with_warnings`。summary wall time 为 `130.361 s`，外层
计时为 `130.66 s`；timing 按 frozen 对账规则不参与行为等价判定。

## 2. Legacy 行为对账

| invariant | control | candidate | 结果 |
|---|---:|---:|---|
| `diag.csv` rows / columns | `1921 / 26` | `1921 / 64` | PASS |
| 前 26 列 projection SHA-256 | `065f8d15…ea49` | `065f8d15…ea49` | PASS；1921/1921 rows 逐字段完全相同 |
| `est.tum` SHA-256 | `265f065e…4e13` | `265f065e…4e13` | byte-identical |
| `kf.tum` SHA-256 | `afd099cd…7813` | `afd099cd…7813` | byte-identical |
| status counts | `1141 ok / 771 initializing / 9 visual_outage` | 相同 | PASS |
| segments / last ok | `9 / 96.5 s` | 相同 | PASS |
| completion / coverage | `0.5939614784 / 0.7613539000` | 相同 | PASS |
| ATE / RPE | `2.0418160161 / 0.6342349346 m` | 相同 | PASS |
| weighted local RMS / intersegment | `0.0602358861 / 1.9815801300 m` | 相同轨迹与 segment projection | PASS |
| config hash / canonical text | `0337287b` | `0337287b` | PASS |
| summary | frozen behavior fields | source identity/timing 之外完全相同 | PASS |

`meta.json` 的 config object、canonical text、hash、dataset identity 与 warnings 完全
一致；差异仅为 candidate source identity、创建时间和 output path。`summary.json` 的
差异仅为 source identity 与 timing。

## 3. Q1 Observe 结果

### 3.1 Terminal 289 行

| 观察项 | 结果 |
|---|---|
| interval / rows / status | `96.6–116.7 s` / `289` / 全部 `initializing` |
| phase / reason | 全部 `stereo_population / population_insufficient` |
| bootstrap | 全部 `moving`，gate `passed` |
| keyframe / population gate | 全部 `passed / failed` |
| seed origin | 全部 `current_packet` |
| current / effective population | 逐行相等；范围 `0–9` |
| threshold | 全部 `10` |
| geometry / current graph / commit gate | 全部 `not_evaluated` |
| attempt | 全部缺失；真实 attempt count `0` |
| old maxima | `num_obs=200`，`num_disparity=9` |

current 与 effective seed population 的 289 行 histogram 相同：

| count | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| rows | 114 | 34 | 26 | 33 | 40 | 18 | 11 | 9 | 2 | 2 |

因此 terminal 的最早持续阻塞 gate 是现有 stereo population gate：289 行没有一行达到
`min_seed_observations=10`，控制流没有进入 root geometry，也没有分配 attempt ID。

### 3.2 全序列 taxonomy

| phase / reason | rows |
|---|---:|
| `not_evaluated / not_evaluated` | 1141 |
| `stereo_population / population_insufficient` | 771 |
| `commit / committed` | 9 |

780 个 cold-root rows 全部使用 `moving` bootstrap 与 `current_packet` origin。其中 9 个
root attempt 依次取得 ID `1..9`，全部为 geometry accepted、current graph passed、
atomic commit；771 个 population-insufficient rows 均无 ID。rollback/rejection ID
语义由 estimator 定向测试覆盖，本次自然 artifact 未出现 rejection 或 graph failure。

formal IMU excitation、conditioning 与 initialization solve 三个 gate 在 1921 行中均为
`not_evaluated`。逐行 typed checker 验证 phase/reason、所有 gate、bootstrap
value/threshold、moving suffix presence、population origin/count、geometry presence 与
attempt identity，1921/1921 rows 全部通过。

## 4. 验证与覆盖边界

Release candidate tree 的实施门：

- `ColdRootObserve*`：15/15 passed；
- `VioFullState.*:VioInitialization.*`：22/22 passed；
- `phad_estimator_tests`：137/137 passed；
- `phad_apps_tests`：42 passed，3 个依赖 `PHAD_EUROC_MH01_PATH` 的既有条件测试 skipped；
- `phad_vo_bench` 与 `phad_stereo_vo_probe` build passed；
- clang-format、`git diff --check`、Standards/Spec review 与独立 red-team passed。

合法稳定 public input 无法确定触达的 root graph build/solve/validation branch，以及
本次 artifact 未自然出现的 `behind_camera` / `empty_after_filter` geometry branch，按
frozen spec 记为 `not_covered`；未新增 production injection seam。`kFailed` /
`kInvalidInput` 仍由 session 在 row append 前 hard-stop，不改变 CSV cadence。

## 5. Config canonical text

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

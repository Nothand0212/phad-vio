# M4 mapped-landmark bearing Q0 control

本文记录 [mapped-landmark bearing continuity spec](../../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md)
§13.1 的 clean Q0 control。Q0 在首条 `MH_01_easy` 数值门停止；后续
`V1_03_difficult`、`V2_02_medium`、`V2_03_difficult` 未运行。

## 1. 身份与执行范围

| 项 | 值 |
|---|---|
| code | `42f99e9117d2c04fdb4ebdfc308d9ca786312e60` (`42f99e9`) |
| tree | `76b46a3ebe696cd8d40ba12dd9581f21fe8cd0d4` |
| worktree | detached HEAD，`git_dirty=false` |
| config | `default_0337287b` |
| predecessor | `c999f58/default_0337287b` |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| artifact root | `/home/lin/Projects/data/phad-bench/m4-mapped-bearing-q0-42f99e9-20260827T134813Z` |
| run date | `2026-08-27`（Asia/Shanghai） |
| build | Release，GNU 13.3.0，Ninja |

实际命令：

```bash
cmake -S . -B build -G Ninja \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure -L unit --parallel 8
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out /home/lin/Projects/data/phad-bench/m4-mapped-bearing-q0-42f99e9-20260827T134813Z/MH_01_easy \
  --sequence-name MH_01_easy \
  --repo /tmp/phad-q0-MyEw1z/worktree \
  --estimator-enable-moving-bootstrap
```

完整 unit suite 为 `458/458` 通过；另有 3 个既有 skip。bench 返回
`completed_with_warnings`，唯一 warning 为 PnP 汇总。

## 2. 首门结果

| 指标 | `c999f58` | Q0 `42f99e9` | 差值 | Q0 门 |
|---|---:|---:|---:|---|
| ATE trans RMSE (m) | `0.0705388305` | `0.0725592618` | `+0.0020204313` (`+2.864%`) | **STOP**；上界 `0.070539` |
| RPE trans RMSE (m) | `0.0331756107` | `0.0326412725` | `-0.0005343382` | 记录为 Q0 值 |
| completion | `0.9997284085` | `0.9997284085` | `0` | PASS |
| coverage | `0.9997283354` | `0.9997283354` | `0` | PASS |
| segments | `1` | `1` | `0` | PASS |
| failed / rejected / reanchors | `0 / 0 / 0` | `0 / 0 / 0` | `0 / 0 / 0` | PASS |
| PnP success / fallback | `3667 / 13` | `3670 / 10` | `+3 / -3` | record-only |
| keyframe / track-only | `660 / 3022` | `663 / 3019` | `+3 / -3` | record-only |

两版均只有一个 segment，因此独立 segment ATE 等于各自全局 ATE；该门的
差异是段内轨迹差异，不是段间拼接分量。

按 spec 的 stop-on-failure 顺序，首条 ATE 超过冻结上界后没有运行后续三条
序列，也没有扫描 sigma、Huber、support 门限或 coast horizon。

## 3. 完整 canonical config

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

相对 predecessor 没有 config 增量键；唯一 CLI override
`--estimator-enable-moving-bootstrap` 已进入 canonical snapshot。

## 4. 完整性

| artifact | SHA-256 |
|---|---|
| `meta.json` | `edd1c341084ba88e2b21e329d9af57774999112ad406488f7466626259ca6b70` |
| `summary.json` | `1f53fe158805dfc482a1d5d6f60dfb4b24167b5a09b4163c6efd355801c58a2d` |
| `diag.csv` | `3f954e83cd321a141923c4b75926a67178d98a324bf8a9c9bd8483955e131e2c` |
| `est.tum` | `bcb5a95993644e6fa5b75d030e731e0d63b9a23f540157ee67c6376212cd2f97` |
| `kf.tum` | `7ba388a1c2b87652e7bcdb89468116e8a05e6f702b702523f4e6c024f1f042e9` |

首分叉与因果归属见
[Q0 control diagnosis](../../research/2026-08-27-note-m4-mapped-bearing-q0-control.md)。

# M4 最小 gyro-aided VO：MH_01 控制组

日期：2026-08-12

状态：Slice 0 控制组通过

跟踪：[#36](https://github.com/Nothand0212/phad-vio/issues/36)

## 结论

`main@7026ebf` 的生产 VO 行为可复现。clean control 的 MH_01 指标为：

- poses：`3681 / 3682`，completion `0.9997284084736556`，coverage `1.0`；
- translation ATE RMSE：`0.08096405792439258 m`；
- translation RPE RMSE：`0.017781218595500668 m`；
- `failed=0`、`rejected=1`、`segments=1`、`reanchors=0`；
- `est.tum`、`kf.tum`、`diag.csv` 与历史 `402d1925` 参考产物逐字节一致。

因此 Slice 1 可以开始。后续 byte gate 以本文 clean control 目录为权威来源。

## 运行身份与输入

clean control 在 `.worktree/main` 运行。该 checkout 的 HEAD 是只新增
`docs/worktrees.md` 的 `ec74a8466d6b502e82275ea5c6e8ee657365768f`；运行前确认：

```text
git_dirty=false
git diff 7026ebf -- CMakeLists.txt apps phad tests scripts
# 无输出，退出码 0
```

因此 runtime 代码与 `main@7026ebf` 相同，文档 commit 不影响可执行路径。

```text
dataset=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
output=/home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control
historical_reference=/home/lin/Projects/data/phad-bench/MH_01_easy/cbb4505_dirty/sad_baseline_402d1925
build_type=Release
compiler=GNU 13.3.0
config_hash=402d1925
```

实际命令：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON
cmake --build build --target phad_vo_bench -j2
build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --gt-euroc /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control \
  --errors-csv
```

运行完成状态是 `completed_with_warnings`；唯一 warning 是现有健康运行允许的
`vo pnp summary: pnp_successes=3670 pnp_fallbacks=10`。

## Runtime 参数快照

以下内容逐项来自本次 `meta.json.config`；不是根据默认值反推：

| key | value |
|---|---:|
| `estimator.block_culled_rebirth` | `true` |
| `estimator.enable_outlier_cull` | `true` |
| `estimator.enable_outlier_reopt` | `true` |
| `estimator.enable_pnp_init` | `true` |
| `estimator.enable_reanchor` | `true` |
| `estimator.far_return_refresh_px` | `6.0` |
| `estimator.hanging_landmark_gate_m` | `1.0` |
| `estimator.huber_k_px` | `3.0` |
| `estimator.max_outlier_reopts` | `3` |
| `estimator.min_landmark_observations` | `2` |
| `estimator.min_pnp_inliers` | `10` |
| `estimator.min_seed_observations` | `10` |
| `estimator.min_shared_landmarks` | `10` |
| `estimator.min_track_observations_for_seed` | `1` |
| `estimator.outlier_avg_reproj_px` | `4.0` |
| `estimator.pnp_confidence` | `0.99` |
| `estimator.pnp_reproj_px` | `2.0` |
| `estimator.prior_rotation_sigma_rad` | `0.0001` |
| `estimator.prior_translation_sigma_m` | `0.0001` |
| `estimator.stereo_sigma_px` | `1.0` |
| `estimator.use_constant_velocity_init` | `true` |
| `estimator.window_size` | `10` |
| `eval.max_dt_ms` | `2.5` |
| `eval.min_match_rate` | `0.5` |
| `eval.rpe_delta_s` | `1.0` |
| `session.dataset_format` | `euroc` |
| `session.drop_culled_tracks` | `true` |
| `session.skip_drop_min_culled` | `4` |
| `session.zombie_drop_age` | `5` |
| `tracker.forward_backward_px` | `0.5` |
| `tracker.lk_pyramid_levels` | `4` |
| `tracker.lk_window_px` | `21` |
| `tracker.mask_radius_px` | `20` |
| `tracker.max_depth_m` | `25.0` |
| `tracker.max_epipolar_px` | `1.5` |
| `tracker.max_tracks` | `200` |
| `tracker.min_depth_m` | `0.3` |
| `tracker.min_disparity_px` | `2.0` |
| `tracker.min_distance_px` | `20` |
| `tracker.quality_level` | `0.003` |
| `tracker.stereo_bidir_px` | `0.5` |
| `tracker.stereo_check_bidir` | `true` |
| `tracker.stereo_row_tol_px` | `0` |
| `tracker.stereo_sad_half_win_px` | `7` |
| `tracker.stereo_uniq_ratio` | `0.5` |

与 `config_hash` 同源的 `config_canonical_text`：

```text
estimator.block_culled_rebirth=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
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

## 指标与行为快照

| 项 | 值 |
|---|---:|
| image frames | `3682` |
| accepted / poses | `3681` |
| rejected / failed | `1 / 0` |
| keyframes / non-keyframes | `662 / 3020` |
| completion / coverage | `0.9997284084736556 / 1.0` |
| ATE trans mean / median / RMSE / max (m) | `0.07166281818634253 / 0.06137583266915215 / 0.08096405792439258 / 0.19610699910101395` |
| ATE rot RMSE / max (deg) | `2.0140355174028675 / 3.5620405199209837` |
| RPE trans mean / median / RMSE / max (m) | `0.014026016607395806 / 0.012173496049016734 / 0.017781218595500668 / 0.08017910585667215` |
| RPE rot RMSE / max (deg) | `0.15151944370189444 / 1.3055250123857283` |
| low connectivity | `2` |
| PnP successes / fallbacks | `3670 / 10` |
| outliers culled / unique | `5 / 5` |
| reanchors | `0` |

## 字节证据

clean control、同日 feature checkout 重复跑以及历史 `sad_baseline_402d1925` 三者的
`est.tum`、`kf.tum`、`diag.csv` 均通过 `cmp`。

```text
18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321  est.tum
4a4a1ba8f7fb2729e482d5aca644c9195ad605ccc0e94ffb9e1e670d1f898bb9  kf.tum
1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb  diag.csv
```

同日首次重复跑位于
`/home/lin/Projects/data/phad-bench/MH_01_easy/09462f1/m4_minimal_gyro_control`。
其运行过程中 feature worktree 出现用户持有的 `AGENTS.md` 规则改动，故 meta 如实记录
`git_dirty=true`；该跑只用作重复性证据，不作为权威 control。其三主产物仍与 clean control
逐字节一致。

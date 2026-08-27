# M4 mapped-landmark bearing Q0 control

本文记录 [mapped-landmark bearing continuity spec](../../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md)
§13.1 的 clean Q0 control。core-4 已完整运行；EuRoC-11 正在同一台本机从保留的
clean worktree 与 Release build 补齐。

## 1. 身份与执行范围

| 项 | 值 |
|---|---|
| code | `42f99e9117d2c04fdb4ebdfc308d9ca786312e60` (`42f99e9`) |
| tree | `76b46a3ebe696cd8d40ba12dd9581f21fe8cd0d4` |
| worktree | 两个 detached clean worktree，均为同一 commit/tree，`git_dirty=false` |
| config | `default_0337287b` |
| predecessor | `c999f58/default_0337287b` |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| artifact root | `/home/lin/Projects/data/phad-bench/m4-mapped-bearing-q0-42f99e9-20260827T134813Z` |
| run date | `2026-08-27`（Asia/Shanghai）；core-4 分两次续跑 |
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
  /home/lin/Projects/data/thidparty/euroc/native/<sequence> \
  --out /home/lin/Projects/data/phad-bench/m4-mapped-bearing-q0-42f99e9-20260827T134813Z/<sequence> \
  --sequence-name <sequence> \
  --repo <clean-42f99e9-worktree> \
  --estimator-enable-moving-bootstrap
```

完整 unit suite 为 `458/458` 通过；另有 3 个既有 skip。bench 返回
`completed_with_warnings`；每条都有 PnP 汇总，`V2_03_difficult` 另有已知 stereo
sync drop warning。

## 2. Core-4 结果

| sequence | ATE (m) | RPE (m) | completion | coverage | segments | 段内 RMS (m) | 绝对段间分量 (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| `MH_01_easy` | `0.072559262` | `0.032641273` | `0.999728408` | `0.999728335` | `1` | `0.072559` | `0.000` |
| `V1_03_difficult` | `0.861562954` | `0.130756976` | `0.974406701` | `0.974860335` | `2` | `0.144` | `0.718` |
| `V2_02_medium` | `0.171546155` | `0.057113027` | `0.974020443` | `0.974009375` | `1` | `0.172` | `0.000` |
| `V2_03_difficult` | `2.081321466` | `0.688425331` | `0.535137949` | `0.791345331` | `18` | `0.062` | `2.020` |

四条均为 `failed=0`、`rejected=0`、`reanchors=0`。Q0 的 segments 合计 `22`，
相对 M4 checkpoint 的 `78` 减少 `56`；`V1_03` 与 `V2_03` 的全局 ATE 仍主要由
段间关系贡献。

### 2.1 相对 M4 checkpoint

| sequence | ATE ratio | ATE 变化 | RPE ratio | RPE 变化 | segments `c999f58 → Q0` |
|---|---:|---:|---:|---:|---:|
| `MH_01_easy` | `1.0286` | `+2.86%` | `0.9839` | `-1.61%` | `1 → 1` |
| `V1_03_difficult` | `0.6123` | `-38.77%` | `0.2450` | `-75.50%` | `16 → 2` |
| `V2_02_medium` | `0.0866` | `-91.34%` | `0.1204` | `-87.96%` | `9 → 1` |
| `V2_03_difficult` | `1.2189` | `+21.89%` | `0.8201` | `-17.99%` | `52 → 18` |

四序列 ATE 算术均值从 `1.291756 m` 降到 `0.796747 m`，RPE 算术均值从
`0.470125 m` 降到 `0.227234 m`。等权归一化几何平均 ratio 为：

```text
G_ATE = 0.5077
G_RPE = 0.3928
```

这两个 aggregate 只描述 `42f99e9` 相对 milestone checkpoint 的整体变化；后续
candidate gate 使用同机 matched clean Q0 作为 immediate control。

### 2.2 当前产品判定边界

core-4 证明 control 可复现并提供逐序列 tail，但尚不形成 Q0 完整产品 envelope。
需先取得 matched 本机 EuRoC-11 control，再冻结 `G_ATE/G_RPE`、有效性、tail
review 与 continuity 数值规则。

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

## 4. Core-4 完整性

| 检查 | 结果 |
|---|---|
| required artifacts | 4/4 均有非空 `meta.json`、`summary.json`、`diag.csv`、`est.tum`、`kf.tum` |
| code identity | 4/4 为 `42f99e9117d2c04fdb4ebdfc308d9ca786312e60`，`git_dirty=false` |
| config identity | 4/4 为 `default_0337287b`；canonical text 逐字一致 |
| diag grain | 每条 image frame 一行；分别为 `3682 / 2149 / 2348 / 1921` 行，均与 summary 一致 |
| timestamps | 4/4 严格递增 |
| numeric validity | 所有 diag numeric cells 有限；计数非负；每行 `num_shared <= num_disparity <= num_obs` |
| status domain | 仅出现 `initializing`、`ok`、`visual_outage`；与当前 lifecycle 合同一致 |

| sequence / artifact | SHA-256 |
|---|---|
| `MH_01_easy/meta.json` | `edd1c341084ba88e2b21e329d9af57774999112ad406488f7466626259ca6b70` |
| `MH_01_easy/summary.json` | `1f53fe158805dfc482a1d5d6f60dfb4b24167b5a09b4163c6efd355801c58a2d` |
| `MH_01_easy/diag.csv` | `3f954e83cd321a141923c4b75926a67178d98a324bf8a9c9bd8483955e131e2c` |
| `MH_01_easy/est.tum` | `bcb5a95993644e6fa5b75d030e731e0d63b9a23f540157ee67c6376212cd2f97` |
| `MH_01_easy/kf.tum` | `7ba388a1c2b87652e7bcdb89468116e8a05e6f702b702523f4e6c024f1f042e9` |
| `V1_03_difficult/meta.json` | `58eda0cf646285e5639a6bc4bf7178114b7517621e307a675b65f9f7f50636be` |
| `V1_03_difficult/summary.json` | `14c0184d5ec4e649144d460d389bd27dda9307fff7e87103484653981023da3f` |
| `V1_03_difficult/diag.csv` | `57357a0e3457dcf5d1f92b6335507fb3fb6ea7fd12a956effa0f6c431cf1bf8d` |
| `V1_03_difficult/est.tum` | `46d99feb8f15aa43b9ca64744214072113415ac6b10407208a9f650d37e52b01` |
| `V1_03_difficult/kf.tum` | `72e0fd8450b9084380a86735529f8735e93587d48c7aa05a1f2b7419fb423537` |
| `V2_02_medium/meta.json` | `8c2bd9a47246194f5106caee0db55a820275356be5d4cc5c83bcee89b82a83b1` |
| `V2_02_medium/summary.json` | `e24d952c40454f31af9242d48ac248f839796eaacf15be0606423115d81df675` |
| `V2_02_medium/diag.csv` | `60b28396b898e7c3887e0c2bcd710e962d1acc69fbefe08f5597f06fc23c1e9c` |
| `V2_02_medium/est.tum` | `02684b420e59f8933b4a9a836740bf617b1d46c3f268ccd7c8480d60146f83ec` |
| `V2_02_medium/kf.tum` | `cc7294106bac44e152b12ba89005c0dc6690467a30a6136b2986a0b7cc798c62` |
| `V2_03_difficult/meta.json` | `21794e24e08e701ef5b8cdc3b744587cd59f0e0bc55c988d56de453477971d87` |
| `V2_03_difficult/summary.json` | `30ce9c3fb2a08e029edadefdb4f1c0bce680959f3c28772a6f27d26a440f3f19` |
| `V2_03_difficult/diag.csv` | `2c54f1606e9ae0e44b8c0568c924ff1c2abf7aa7e865882ea99e84fc46a00d3f` |
| `V2_03_difficult/est.tum` | `6b1f9b3b69175967fd909ed6aa2eac644139d7e8295d83deea76d03a5fcd223b` |
| `V2_03_difficult/kf.tum` | `1337c800e66c6220b7a574921462f77c28d090f84d9b0d2d7ce78ba1eb6b3b32` |

MH_01 的五个 hash 与首次 Q0 记录逐字一致，证明续跑没有改写已有 artifact。

## 5. EuRoC-11 本机 control 状态

沿用 §1 的 clean `42f99e9` worktree、Release build、canonical config、dataset
root 与 artifact root，在同一台本机串行补跑 core-4 之外的 7 条序列。Q0 完成后
对 11 条统一进行 identity、schema、数值、时序与 hash 校验，再冻结产品 envelope。

执行环境切换前的连通性检查未建立 SSH 会话、上传 snapshot 或创建 run，不产生
另一份 control artifact。

首分叉与因果归属见
[Q0 control diagnosis](../../research/2026-08-27-note-m4-mapped-bearing-q0-control.md)。

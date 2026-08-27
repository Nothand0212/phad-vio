# M4 Minimal Full-State VIO Benchmark（`default_0337287b`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-26

状态：**complete**；M4 full-state VIO 首个 clean EuRoC 11/11 全量
checkpoint，作为后续精度、同步与 visual-outage lifecycle 优化的对比锚点。

相关：

- issue：[#41](https://github.com/Nothand0212/phad-vio/issues/41)
- predecessor：[M3.3 Slice ⑦](../m3.3/slice-7_e77ee5d_402d1925.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `c999f5891d8efbc353e2e4001df7656a9e19907a`（short `c999f58`；`git_dirty=false`） |
| config | `default` / `0337287b` |
| predecessor | `e77ee5d` / `default_402d1925` |
| dataset | EuRoC ASL native，11 条固定序列 |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| bench root | `/home/lin/Projects/data/phad-bench` |
| artifact path | `/home/lin/Projects/data/phad-bench/<sequence>/c999f58/default_0337287b/` |
| execution | 2026-08-26，`phad_vo_bench` 串行 11 条，累计 wall time `1876.123 s` |
| gate | EuRoC 11 条 record-only；保留各序列 completion/coverage 口径 |

实际命令：

```bash
sequences=(
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult
  V1_01_easy V1_02_medium V1_03_difficult
  V2_01_easy V2_02_medium V2_03_difficult
)
for seq in "${sequences[@]}"; do
  ./build/phad_vo_bench \
    "/home/lin/Projects/data/thidparty/euroc/native/${seq}" \
    --bench-root /home/lin/Projects/data/phad-bench \
    --sequence-name "${seq}" \
    --repo /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish \
    --estimator-enable-moving-bootstrap
done
```

## 2. 配置快照

相对 predecessor 的增量键：

| 键 | predecessor | 当前 | 说明 |
|---|---|---|---|
| `estimator.enable_moving_bootstrap` | 无该键 | `true` | 允许无静止起步的 EuRoC 序列建立 full-state root |
| `estimator.velocity_prior_sigma_mps` | 无该键 | `0.1` | 放松滑窗 root velocity gauge prior |

M4 的 X/V/B state、IMU/bias-RW factors、raw provenance reintegration 和 segment
lifecycle 是 code behavior 增量，不能仅由 config 键表达。

完整 `config_canonical_text`：

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

CLI-only 参数：无。`--estimator-enable-moving-bootstrap` 已进入 config snapshot 和
hash。

## 3. 全量质量表

| sequence | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | ok / image | rejected | failed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | completed_with_warnings | 0.070539 | 0.033176 | 0.999728 | 0.999728 | 1 | 0 | 3681 / 3682 | 0 | 0 |
| MH_02_easy | completed_with_warnings | 0.290545 | 0.070780 | 0.998026 | 1.000000 | 6 | 0 | 3034 / 3040 | 0 | 0 |
| MH_03_medium | completed_with_warnings | 0.094969 | 0.042921 | 1.000000 | 1.000000 | 1 | 0 | 2700 / 2700 | 0 | 0 |
| MH_04_difficult | completed_with_warnings | 0.213229 | 0.062605 | 1.000000 | 1.000000 | 1 | 0 | 2032 / 2032 | 0 | 0 |
| MH_05_difficult | completed_with_warnings | 0.230276 | 0.054400 | 1.000000 | 1.000000 | 1 | 0 | 2272 / 2272 | 0 | 0 |
| V1_01_easy | completed_with_warnings | 0.112070 | 0.062576 | 0.997595 | 1.000000 | 2 | 0 | 2904 / 2911 | 0 | 0 |
| V1_02_medium | completed_with_warnings | 0.079079 | 0.046839 | 0.943242 | 0.944379 | 3 | 0 | 1612 / 1709 | 0 | 0 |
| V1_03_difficult | completed_with_warnings | 1.406996 | 0.533641 | 0.964635 | 0.974860 | 16 | 0 | 2073 / 2149 | 0 | 0 |
| V2_01_easy | completed_with_warnings | 0.184779 | 0.043345 | 0.951316 | 0.951294 | 1 | 0 | 2169 / 2280 | 0 | 0 |
| V2_02_medium | completed_with_warnings | 1.981976 | 0.474193 | 0.968484 | 0.972731 | 9 | 0 | 2274 / 2348 | 0 | 0 |
| V2_03_difficult | completed_with_warnings | 1.707513 | 0.839489 | 0.506507 | 0.791345 | 52 | 0 | 973 / 1921 | 0 | 0 |

11 条算术均值：ATE `0.579270 m`，RPE(1s) `0.205815 m`。各条 warning
主要来自 PnP fallback 摘要；MH_04、V1_02 和 V2_03 另有 stereo sync
drop warning。

## 4. Robustness

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 17 / 17 | 1 | 1 | 1 / 4 | 3667 / 13 | 2 | 0 |
| MH_02_easy | 15 / 15 | 1 | 1 | 1 / 7 | 2967 / 58 | 0 | 0 |
| MH_03_medium | 5 / 5 | 0 | 0 | 0 / 0 | 2694 / 5 | 0 | 0 |
| MH_04_difficult | 6 / 6 | 0 | 0 | 0 / 0 | 2022 / 9 | 0 | 0 |
| MH_05_difficult | 6 / 6 | 0 | 0 | 0 / 0 | 2259 / 12 | 0 | 0 |
| V1_01_easy | 7 / 7 | 0 | 0 | 0 / 0 | 2887 / 15 | 0 | 0 |
| V1_02_medium | 5 / 5 | 0 | 0 | 0 / 0 | 1586 / 23 | 3 | 0 |
| V1_03_difficult | 6 / 6 | 0 | 0 | 0 / 0 | 1847 / 142 | 0 | 0 |
| V2_01_easy | 24 / 24 | 0 | 0 | 0 / 0 | 2131 / 37 | 4 | 0 |
| V2_02_medium | 12 / 12 | 0 | 0 | 0 / 0 | 2062 / 196 | 0 | 0 |
| V2_03_difficult | 1 / 1 | 0 | 0 | 0 / 0 | 342 / 519 | 0 | 0 |

## 5. 相对 predecessor

以 M3.3 Slice ⑦ `e77ee5d/default_402d1925` 为精度 predecessor。`ATE Δ` 和
`RPE Δ` 为当前减 predecessor，负值表示误差下降。

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| MH_01_easy | -0.010425 | +0.015395 | +0.000000 | 0 | 0 | n/a | ATE 下降，RPE 上升 |
| MH_02_easy | +0.201176 | +0.055146 | +0.001973 | +5 | 0 | n/a | ATE/RPE 上升，segment 增加 |
| MH_03_medium | -0.033622 | +0.007288 | +0.000000 | 0 | 0 | n/a | ATE 下降，RPE 小幅上升 |
| MH_04_difficult | -0.074974 | +0.008553 | +0.001476 | 0 | 0 | n/a | ATE 下降，RPE 小幅上升 |
| MH_05_difficult | -0.093831 | +0.010391 | +0.000440 | 0 | 0 | n/a | ATE 下降，RPE 小幅上升 |
| V1_01_easy | -0.004273 | +0.016766 | -0.002405 | +1 | 0 | n/a | ATE 基本持平，RPE 上升 |
| V1_02_medium | -0.442425 | -0.072735 | +0.005815 | +2 | 0 | n/a | ATE/RPE 下降 |
| V1_03_difficult | -3.552536 | -0.402771 | +0.004188 | +10 | -5 | n/a | 误差下降，segment 增加 |
| V2_01_easy | -0.124636 | +0.002297 | +0.002632 | 0 | 0 | n/a | ATE 下降，RPE 基本持平 |
| V2_02_medium | +1.129129 | +0.206378 | +0.002556 | +8 | 0 | n/a | ATE/RPE 上升，segment 增加 |
| V2_03_difficult | -1.920287 | -0.047443 | +0.002603 | +42 | -9 | n/a | 误差下降，但 lifecycle 频繁切段 |

predecessor 文档未保存 `failed` 计数，因此该 delta 不可确认。当前 11 条
`failed=0`、`rejected=0`。当前均值相对 predecessor：ATE `-0.447882 m`，
RPE `-0.018249 m`；逐序列 ATE 为 9 条下降、2 条上升，RPE 为 4 条下降、
7 条上升。

## 6. 判定与风险

- MH_01 ATE `0.070539 m`，低于 predecessor `0.080964 m`，completion
  `0.999728`；M4 在该序列上形成可用的 full-state VIO 锚点。
- MH 组 completion 为 `0.998026–1.0`；MH_02 的 ATE 与 segments 是后续
  对比时需保留的局部回归信号。
- V1_03、V2_02 和 V2_03 的 ATE 仍高，且分别有 `16`、`9`、`52`
  个 segments。V2_03 只有 `0.506507` completion，其 `diag.csv` 中有
  `52` visual-outage 帧和 `896` initializing 帧；`summary.sync` 另记录
  `415` 个 right frame drops。
- 上述低 coverage 与 segment 变化会改变 ATE 匹配集；后续优化不应只根据
  ATE 单列归因，应同时比较 completion、coverage 和 lifecycle 计数。
- 该 commit 的 `summary.json` / `diag.csv` 未持久化 final VIO topology 和累计
  eviction/reintegration 计数；精确回溯这些字段时需使用同一 commit/config
  重跑直接 diagnostics probe。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 11/11 rc=0，串行 |
| artifacts | 11 份 `meta.json` + 11 份 `summary.json`；每条均有 `diag.csv` / `est.tum` / `kf.tum` |
| code identity | 11/11 为 `c999f5891d8efbc353e2e4001df7656a9e19907a`，`git_dirty=false` |
| config identity | 11/11 为 `default_0337287b` |
| canonical config | 11/11 与本文快照一致 |
| tests | `phad_estimator_tests` 89/89 PASS；`phad_sync_tests` 26/26 PASS；`phad_apps_tests` 41 PASS / 3 SKIP |
| raw artifacts | `/home/lin/Projects/data/phad-bench/<sequence>/c999f58/default_0337287b/` |
| 未执行项 | EuRoC 外数据集、ATE/RPE 以外的正式轨迹质量评估 |

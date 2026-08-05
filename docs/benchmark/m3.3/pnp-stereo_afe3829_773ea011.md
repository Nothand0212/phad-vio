# M3.3 PnP stereo 一致性仲裁 Benchmark（`default_773ea011`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-04（clean commit 补跑）

状态：**clean EuRoC 11/11 checkpoint 已完成**；MH_02 灾难消除、MH_05 大幅改善，
MH_03 出现已知回归；当前 estimator 默认

相关：

- issue [#26](https://github.com/Nothand0212/phad-vio/issues/26)、
  [#25](https://github.com/Nothand0212/phad-vio/issues/25)
- predecessor：[zombie-age=5](../../benchmark/m3.3/zombie-age_4cf55ca_773ea011.md)
- 设计：[PnP stereo 一致性](../../research/m3.3-pnp-stereo-consistency-design.md)
- 根因诊断：[MH_02 divergence](../../research/m3.3-mh02-divergence-diagnosis.md)
- 提交前 dirty 验证：[arbitration results](../../research/m3.3-pnp-stereo-arbitration-results.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `afe3829e7a72763444fba0e66070e6aac797b550`（short `afe3829`；11/11 `git_dirty=false`） |
| config | `default` / `773ea011`（**与 zombie-age 相同 hash**；纯代码差异） |
| predecessor | zombie-age `4cf55ca/default_773ea011` |
| dataset | EuRoC ASL native 11 条 |
| bench root | `/home/lin/Projects/data/phad-benchmark-ledger` |
| artifact path | `<bench_root>/<sequence>/afe3829/default_773ea011/` |
| execution | 独立 clean clone；11 条串行；`phad_vo_bench`；无算法 CLI override |
| run 结果 | 11/11 命令 rc=0；wall time 合计 `3322.972 s` |
| gate | 提交前 dirty 验证已过 MH_02 因果门 / MH_01 硬门 / MH_05 软门；本补跑为 clean 全量确认 |

复现命令：

```bash
for seq in \
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult \
  V1_01_easy V1_02_medium V1_03_difficult \
  V2_01_easy V2_02_medium V2_03_difficult
do
  build/phad_vo_bench \
    "/home/lin/Projects/data/thidparty/euroc/native/${seq}" \
    --bench-root /home/lin/Projects/data/phad-benchmark-ledger \
    --sequence-name "$seq" \
    --force
done
```

## 2. 配置快照

相对 zombie-age **无任何 config 增量**：42 键 canonical 完全相同。差异全部在
代码：PnP proposal 不再以“左目 inlier 数”单独决定采用，而是在同一 inlier 集上
比较 proposal / guess 的完整 stereo RMS，通过后才授权 pose 与 mask（修 MH_02
“零度 pose”灾难链）。

完整 `config_canonical_text`（42 键，与 zombie-age 相同）：

```text
estimator.block_culled_rebirth=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.huber_k_px=3
estimator.max_outlier_reopts=3
estimator.min_landmark_observations=2
estimator.min_pnp_inliers=10
estimator.min_seed_observations=10
estimator.min_shared_landmarks=10
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
tracker.lk_pyramid_levels=3
tracker.lk_window_px=21
tracker.mask_radius_px=20
tracker.max_depth_m=40
tracker.max_epipolar_px=1.5
tracker.max_tracks=200
tracker.min_depth_m=0.29999999999999999
tracker.min_disparity_px=0.5
tracker.min_distance_px=20
tracker.quality_level=0.01
tracker.stereo_bidir_px=0.5
tracker.stereo_check_bidir=true
tracker.stereo_row_tol_px=0
tracker.stereo_sad_half_win_px=7
tracker.stereo_uniq_ratio=0.5
```

CLI-only 参数：无。

## 3. 全量质量表

| sequence | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | ok / image | rejected | failed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | completed | 0.098784 | 0.018529 | 1.000000 | 1.000000 | 1 | 0 | 3682 / 3682 | 0 | 0 |
| MH_02_easy | completed_with_warnings | 0.092417 | 0.014003 | 1.000000 | 1.000000 | 1 | 0 | 3040 / 3040 | 0 | 0 |
| MH_03_medium | completed_with_warnings | 1.336250 | 0.278380 | 1.000000 | 1.000000 | 1 | 0 | 2700 / 2700 | 0 | 0 |
| MH_04_difficult | completed_with_warnings | 0.260463 | 0.055328 | 1.000000 | 1.000000 | 1 | 0 | 2032 / 2032 | 0 | 0 |
| MH_05_difficult | completed_with_warnings | 0.472225 | 0.188420 | 0.997800 | 1.000000 | 1 | 0 | 2268 / 2273 | 5 | 0 |
| V1_01_easy | completed_with_warnings | 0.172690 | 0.075828 | 0.998626 | 1.000000 | 1 | 0 | 2908 / 2912 | 4 | 0 |
| V1_02_medium | completed_with_warnings | 0.768334 | 0.153412 | 0.943275 | 0.944997 | 1 | 0 | 1613 / 1710 | 97 | 0 |
| V1_03_difficult | completed_with_failures | 3.353343 | 0.830455 | 0.951140 | 1.000000 | 15 | 14 | 2044 / 2149 | 33 | 72 |
| V2_01_easy | completed_with_warnings | 1.259471 | 0.537540 | 0.999123 | 1.000000 | 2 | 1 | 2278 / 2280 | 2 | 0 |
| V2_02_medium | completed_with_failures | 2.191815 | 0.514224 | 0.979131 | 0.999574 | 4 | 3 | 2299 / 2348 | 13 | 36 |
| V2_03_difficult | completed_with_failures | 7.671162 | 1.730457 | 0.452889 | 0.719794 | 12 | 11 | 870 / 1921 | 1011 | 40 |

## 4. Robustness

| sequence | culled / unique | reopts | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|
| MH_01 | 3 / 3 | 0 | 0 / 0 | 3681 / 0 | 0 | 0 |
| MH_02 | 2 / 2 | 0 | 0 / 0 | 3030 / 9 | 0 | 4 |
| MH_03 | 1 / 1 | 0 | 0 / 0 | 2686 / 13 | 0 | 9 |
| MH_04 | 3 / 3 | 0 | 0 / 0 | 2011 / 20 | 0 | 19 |
| MH_05 | 5 / 5 | 0 | 0 / 0 | 2239 / 28 | 0 | 26 |
| V1_01 | 18 / 18 | 0 | 0 / 0 | 2879 / 28 | 0 | 26 |
| V1_02 | 19 / 19 | 1 | 1 / 4 | 1564 / 48 | 0 | 38 |
| V1_03 | 22 / 22 | 0 | 0 / 0 | 1823 / 206 | 2 | 201 |
| V2_01 | 18 / 18 | 0 | 0 / 0 | 2174 / 102 | 0 | 95 |
| V2_02 | 16 / 16 | 0 | 0 / 0 | 2056 / 239 | 2 | 208 |
| V2_03 | 24 / 24 | 0 | 0 / 0 | 461 / 397 | 27 | 298 |

`deferred_*` / `evictable_marked` / `tracks_evicted` 恒 0（回退合同）。MH_02 的
cheirality 由 zombie-age 的 849 降到 0、culled 由 369 降到 2、PnP fallback 由
52 降到 9——仲裁直接消灭了“左目 inlier 授权 → 不可逆 cheirality 删除”链。

## 5. 相对 zombie-age（predecessor）

3/11 条 `est.tum` 逐字节一致（MH_01、MH_04、V1_01）；8 条改变。

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| MH_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_02 | -530013.774549 | -86752.472100 | +0.477961 | -6 | -6 | -1453 | 灾难消除：0.52 → 1.0 completion，ATE 0.092 m |
| MH_03 | +0.215484 | +0.081881 | 0 | 0 | 0 | 0 | **已知回归**：无失败帧但 ATE 1.12 → 1.34 m |
| MH_04 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_05 | -1.983501 | -0.523442 | 0 | 0 | 0 | 0 | 软门大幅改善：2.456 → 0.472 m |
| V1_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| V1_02 | -7.639363 | -1.422703 | 0 | -1 | -1 | 0 | 改善：8.41 → 0.77 m |
| V1_03 | -6.438149 | -0.165995 | +0.052583 | -1 | -1 | -113 | 改善：9.79 → 3.35 m，failed 185 → 72 |
| V2_01 | -3.920499 | -0.082618 | +0.157456 | -1 | -1 | -357 | 改善：5.18 → 1.26 m，failed 清零 |
| V2_02 | -2.192646 | -0.479365 | 0 | 0 | 0 | 0 | 改善：4.38 → 2.19 m，结构相同 |
| V2_03 | -1.735958 | -0.363799 | 0 | -1 | -1 | 0 | 改善：9.41 → 7.67 m，仍有 failures |

跨 coverage、segments 或匹配集合变化时（MH_02、V1_03、V2_01、V2_03），只陈述
数字变化，不把 ATE 差异直接归因为单一机制。

## 6. 判定与风险

- clean 全量确认 dirty 验证结论：MH_02 灾难消除（ATE `5.30e5 → 0.092 m`、
  completion `0.522 → 1.0`、`segments/reanchors/failed = 7/6/1453 → 1/0/0`）；
- MH_05 软门从 zombie-age 的 2.456 进一步降到 **0.472 m**；V1_02 / V1_03 /
  V2_01 / V2_02 / V2_03 全部改善；
- **MH_03 回归**：ATE/RPE `1.120766/0.196499 → 1.336250/0.278380`，与提交前
  dirty 验证完全一致——这是后续选片前必须保留的回归证据；
- V2_03 仍 `completed_with_failures`（completion 0.4529），V1_03 / V2_02 残余
  failure 债未清；
- 3/11 条逐字节不变说明仲裁只影响触发错误 proposal 的帧；
- 本 checkpoint 是当前 `afe3829` estimator 默认的权威全量记录；下一片需先结合
  MH_03 回归与 V1_03 / V2_02 / V2_03 剩余债重新对齐。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 11/11 命令 rc=0 |
| artifacts | 11/11 `meta.json` / `summary.json` 齐全 |
| code identity | 11/11 full commit=`afe3829e7a72763444fba0e66070e6aac797b550`，dirty=false |
| config identity | 11/11 `default_773ea011`（与 zombie-age 相同 hash，代码不同） |
| canonical config | 11 份 42 键 config / canonical text 完全一致 |
| predecessor artifacts | zombie-age `4cf55ca/default_773ea011` 11/11 齐全 |
| raw artifacts | `/home/lin/Projects/data/phad-benchmark-ledger` |
| 未执行项 | 无 |

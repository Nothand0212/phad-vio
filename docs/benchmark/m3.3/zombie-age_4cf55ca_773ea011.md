# M3.3 zombie-age=5 Benchmark（`default_773ea011`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-04（历史正式 baseline 迁移，非补跑）

状态：**EuRoC 11/11 clean 正式 baseline（record-only）**；MH_05 改善、MH_02/V1_02 恶化，
作为“当前生命周期默认”只记录、不验收

相关：

- issue [#26](https://github.com/Nothand0212/phad-vio/issues/26)
- predecessor：[Slice ④f](../../benchmark/m3.3/slice-4f_c446ac5_a5e90dc7.md)
- 设计：[zombie-drop-age probe](../../research/m3.3-zombie-drop-age-probe-design.md)
- 原始 baseline 记录：[m3.3 full-suite baseline 773ea011](../../research/m3.3-full-suite-baseline-773ea011.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `4cf55ca86bc89dfcfe05f730efa37c545e6d08af`（short `4cf55ca`；11/11 `git_dirty=false`） |
| config | `default` / `773ea011` |
| predecessor | Slice ④f `c446ac5/default_a5e90dc7`（④g 已回退，不是本 checkpoint 前驱） |
| dataset | EuRoC ASL native 11 条 |
| bench root | `/home/lin/Projects/data/phad-bench`（历史 root；④e/④f 补跑在 `/home/lin/Projects/data/phad-benchmark-ledger`） |
| artifact path | `<bench_root>/<sequence>/4cf55ca/default_773ea011/` |
| execution | 历史正式全量（record-only）；11 条串行；`phad_vo_bench` |
| run 结果 | 11/11 完成；wall time 合计 `3290.940 s` |
| gate | record-only：不设硬/软门，只记录数值与风险 |

复现命令（bench root 需指向历史 root 才能复用同名 artifact）：

```bash
for seq in \
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult \
  V1_01_easy V1_02_medium V1_03_difficult \
  V2_01_easy V2_02_medium V2_03_difficult
do
  build/phad_vo_bench \
    "/home/lin/Projects/data/thidparty/euroc/native/${seq}" \
    --bench-root /home/lin/Projects/data/phad-bench \
    --sequence-name "$seq" \
    --force
done
```

## 2. 配置快照

相对 Slice ④f 的唯一 config 增量是把 zombie 龄 drop 产品化默认纳入配置：

| 键 | Slice ④f | zombie-age | 说明 |
|---|---:|---:|---|
| `session.zombie_drop_age` | 不存在 | `5` | frontend track 连续无观测帧龄 ≥5 时 drop（n=5 为 MH_05 矩阵甜区） |

代码侧还包含 ④g 回退残留合同：`deferred_drops` / `deferred_drop_ids` /
`evictable_marked` / `tracks_evicted` 字段恒为 0（编排已回退，合同保留）。

完整 `config_canonical_text`（42 键）：

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

CLI-only 参数：无（`--zombie-drop-age`、`--defer-drop-topk`、`--evict-skip-culled`
均为 probe 用 CLI override，本 checkpoint 未启用，且不进 `config_hash`）。

## 3. 全量质量表

| sequence | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | ok / image | rejected | failed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | completed | 0.098784 | 0.018529 | 1.000000 | 1.000000 | 1 | 0 | 3682 / 3682 | 0 | 0 |
| MH_02_easy | completed_with_failures | 530013.866966 | 86752.486103 | 0.522039 | 1.000000 | 7 | 6 | 1587 / 3040 | 0 | 1453 |
| MH_03_medium | completed_with_warnings | 1.120766 | 0.196499 | 1.000000 | 1.000000 | 1 | 0 | 2700 / 2700 | 0 | 0 |
| MH_04_difficult | completed_with_warnings | 0.260463 | 0.055328 | 1.000000 | 1.000000 | 1 | 0 | 2032 / 2032 | 0 | 0 |
| MH_05_difficult | completed_with_warnings | 2.455726 | 0.711862 | 0.997800 | 1.000000 | 1 | 0 | 2268 / 2273 | 5 | 0 |
| V1_01_easy | completed_with_warnings | 0.172690 | 0.075828 | 0.998626 | 1.000000 | 1 | 0 | 2908 / 2912 | 4 | 0 |
| V1_02_medium | completed_with_warnings | 8.407697 | 1.576115 | 0.943275 | 0.944997 | 2 | 1 | 1613 / 1710 | 97 | 0 |
| V1_03_difficult | completed_with_failures | 9.791492 | 0.996450 | 0.898557 | 1.000000 | 16 | 15 | 1931 / 2149 | 33 | 185 |
| V2_01_easy | completed_with_failures | 5.179970 | 0.620158 | 0.841667 | 1.000000 | 3 | 2 | 1919 / 2280 | 4 | 357 |
| V2_02_medium | completed_with_failures | 4.384461 | 0.993589 | 0.979131 | 0.999574 | 4 | 3 | 2299 / 2348 | 13 | 36 |
| V2_03_difficult | completed_with_failures | 9.407120 | 2.094256 | 0.452889 | 0.719794 | 13 | 12 | 870 / 1921 | 1011 | 40 |

## 4. Robustness

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01 | 3 / 3 | 0 | 0 | 0 / 0 | 3681 / 0 | 0 | 0 |
| MH_02 | 369 / 369 | 25 | 32 | 32 / 539 | 1528 / 52 | 849 | 355 |
| MH_03 | 57 / 57 | 2 | 2 | 2 / 155 | 2688 / 11 | 200 | 9 |
| MH_04 | 3 / 3 | 0 | 0 | 0 / 0 | 2011 / 20 | 0 | 19 |
| MH_05 | 61 / 61 | 2 | 2 | 2 / 91 | 2238 / 29 | 65 | 26 |
| V1_01 | 18 / 18 | 0 | 0 | 0 / 0 | 2879 / 28 | 0 | 26 |
| V1_02 | 68 / 68 | 2 | 3 | 3 / 71 | 1564 / 47 | 60 | 38 |
| V1_03 | 231 / 231 | 4 | 5 | 5 / 345 | 1706 / 209 | 528 | 279 |
| V2_01 | 65 / 65 | 2 | 2 | 2 / 59 | 1815 / 101 | 79 | 361 |
| V2_02 | 61 / 61 | 4 | 4 | 4 / 56 | 2053 / 242 | 45 | 209 |
| V2_03 | 19 / 19 | 0 | 0 | 0 / 0 | 479 / 378 | 73 | 306 |

`deferred_drops` / `deferred_drop_ids` / `evictable_marked` / `tracks_evicted`
全部为 0（回退合同恒 0，未在表中展开）。

## 5. 相对 Slice ④f

4/11 条 `est.tum` 逐字节一致（MH_01、MH_04、V1_01、V2_03）；7 条改变。

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| MH_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_02 | +367249.301924 | +63816.082813 | -0.290132 | +3 | +3 | +882 | 大幅恶化：completion 0.812 → 0.522，zombie 冲刷 32 帧 / 539 ids |
| MH_03 | +0.162082 | +0.008203 | 0 | 0 | 0 | 0 | 无失败帧，数值回退 |
| MH_04 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_05 | -0.601152 | -0.060561 | 0 | 0 | 0 | 0 | 软门进一步改善（3.057 → 2.456） |
| V1_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| V1_02 | +7.953463 | +1.408700 | 0 | +1 | +1 | 0 | 明显恶化：0.45 → 8.41 m，zombie 3 帧 / 71 ids |
| V1_03 | +1.339222 | +0.121188 | -0.020475 | -2 | -2 | +45 | 恶化：completion 0.919 → 0.899，zombie 5 帧 / 345 ids |
| V2_01 | -0.024941 | +0.000870 | 0 | 0 | 0 | 0 | 数值微变 |
| V2_02 | +0.032773 | +0.057436 | 0 | 0 | 0 | 0 | 数值微变 |
| V2_03 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |

跨 coverage、segments 或匹配集合变化时（MH_02、V1_02、V1_03），只陈述数字变化，
不把 ATE 差异直接归因为单一机制。

## 6. 判定与风险

- 本 checkpoint 为 record-only 正式 baseline，不代表验收通过；MH_05 软门
  PASS（2.456）但 MH_02 灾难级恶化（ATE ≈`5.30e5 m`，completion 0.522）是已知风险；
- zombie 龄 drop 在 MH_02 高频触发（32 帧 / 539 ids），配合 cull 的 369 ids 推断
  该序列生命周期灾难被加剧；V1_02/V1_03 同样恶化；
- 4 条逐字节不变，说明 zombie 龄 drop 只影响存在长龄 track 的序列；
- MH_05 的 2 帧 zombie 冲刷（91 ids）带来 −0.60 m ATE，是 n=5 甜区收益的直接证据；
- 该 checkpoint 之后由 PnP stereo 仲裁（`afe3829`）在相同 config 上继续演进。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 历史正式 run 11/11 完成 |
| artifacts | 11/11 `meta.json` / `summary.json` 齐全 |
| code identity | 11/11 full commit=`4cf55ca86bc89dfcfe05f730efa37c545e6d08af`，dirty=false |
| config identity | 11/11 `default_773ea011` |
| canonical config | 11 份 42 键 config / canonical text 完全一致 |
| predecessor artifacts | Slice ④f `c446ac5/default_a5e90dc7` 11/11 齐全 |
| raw artifacts | `/home/lin/Projects/data/phad-bench`（历史 root） |
| 未执行项 | 无（复用既有 raw，未重复跑） |

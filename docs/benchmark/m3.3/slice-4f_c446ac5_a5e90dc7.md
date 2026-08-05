# M3.3 Slice ④f Benchmark（`default_a5e90dc7`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-04（历史 commit 补跑）

状态：**clean EuRoC 11/11 checkpoint 已补齐**；算法结论为“④f 已完成，后被 zombie-age=5 扩展”

相关：

- issue [#26](https://github.com/Nothand0212/phad-vio/issues/26)
- predecessor：[Slice ④e](../../benchmark/m3.3/slice-4e_0ced28b_3a21162e.md)
- 设计：[Slice ④f skip-drop](../../research/m3.3-slice4f-skip-drop-design.md)
- 诊断：[MH_05 failure diagnosis](../../research/m3.3-mh05-failure-diagnosis.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `c446ac5747476254af684a77a05f726c69bfd69c`（short `c446ac5`；11/11 `git_dirty=false`） |
| config | `default` / `a5e90dc7` |
| predecessor | Slice ④e `0ced28b/default_3a21162e` |
| dataset | EuRoC ASL native 11 条 |
| bench root | `/home/lin/Projects/data/phad-benchmark-ledger` |
| artifact path | `<bench_root>/<sequence>/c446ac5/default_a5e90dc7/` |
| execution | 独立 clean clone；11 条串行；`phad_vo_bench`；无算法 CLI override |
| run 结果 | 11/11 完成，`summary.json` 齐全（4 条 `completed_with_warnings` 或 `completed`，7 条 `completed_with_failures`）；wall time 合计 `3326.229 s` |
| 原 slice 门控 | MH_01 硬门 PASS；MH_05 软门 PASS（ATE ≈3.057 vs ④e ≈4.565）→ 判定完成 |

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

相对 Slice ④e 的 config 增量（④f 把 ④c 以来的 drop 行为正式纳入配置，并新增
mean-cull 阈值跳过）：

| 键 | Slice ④e | Slice ④f | 说明 |
|---|---:|---:|---|
| `session.drop_culled_tracks` | 不存在（④c 起隐式开启） | `true` | estimator cull/cheirality 擦除后 drop 对应 frontend track 的开关 |
| `session.skip_drop_min_culled` | 不存在 | `4` | `outliers_culled >= 4` 时跳过当帧 `dropTracks`（防 MH_05 式大规模 cull 后清空 track）；`0` 关闭 |

完整 `config_canonical_text`（41 键）：

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
| MH_02_easy | completed_with_failures | 162764.565042 | 22936.403290 | 0.812171 | 1.000000 | 4 | 3 | 2469 / 3040 | 0 | 571 |
| MH_03_medium | completed_with_warnings | 0.958684 | 0.188296 | 1.000000 | 1.000000 | 1 | 0 | 2700 / 2700 | 0 | 0 |
| MH_04_difficult | completed_with_warnings | 0.260463 | 0.055328 | 1.000000 | 1.000000 | 1 | 0 | 2032 / 2032 | 0 | 0 |
| MH_05_difficult | completed_with_warnings | 3.056878 | 0.772423 | 0.997800 | 1.000000 | 1 | 0 | 2268 / 2273 | 5 | 0 |
| V1_01_easy | completed_with_warnings | 0.172690 | 0.075828 | 0.998626 | 1.000000 | 1 | 0 | 2908 / 2912 | 4 | 0 |
| V1_02_medium | completed_with_warnings | 0.454234 | 0.167415 | 0.943275 | 0.944997 | 1 | 0 | 1613 / 1710 | 97 | 0 |
| V1_03_difficult | completed_with_failures | 8.452270 | 0.875262 | 0.919032 | 1.000000 | 18 | 17 | 1975 / 2149 | 34 | 140 |
| V2_01_easy | completed_with_failures | 5.204911 | 0.619288 | 0.841667 | 1.000000 | 3 | 2 | 1919 / 2280 | 4 | 357 |
| V2_02_medium | completed_with_failures | 4.351688 | 0.936153 | 0.979131 | 0.999574 | 4 | 3 | 2299 / 2348 | 13 | 36 |
| V2_03_difficult | completed_with_failures | 9.407120 | 2.094256 | 0.452889 | 0.719794 | 13 | 12 | 870 / 1921 | 1011 | 40 |

状态分布：1 条 `completed`、4 条 `completed_with_warnings`、6 条
`completed_with_failures`。命令完成不等于上述 failure 序列通过。

## 4. Robustness

本 checkpoint 的 schema 已有 `drops_skipped`，尚无 zombie-age 字段（zombie 从
`4cf55ca/default_773ea011` 起才进入 schema），zombie 列以 `—` 表示合同不存在，
不写成 0。

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01 | 3 / 3 | 0 | 0 | — | 3681 / 0 | 0 | 0 |
| MH_02 | 194 / 194 | 12 | 13 | — | 2434 / 31 | 196 | 522 |
| MH_03 | 36 / 36 | 2 | 2 | — | 2687 / 12 | 185 | 10 |
| MH_04 | 3 / 3 | 0 | 0 | — | 2011 / 20 | 0 | 19 |
| MH_05 | 28 / 28 | 1 | 1 | — | 2238 / 29 | 29 | 26 |
| V1_01 | 18 / 18 | 0 | 0 | — | 2879 / 28 | 0 | 26 |
| V1_02 | 187 / 187 | 3 | 3 | — | 1566 / 46 | 55 | 38 |
| V1_03 | 80 / 80 | 3 | 3 | — | 1753 / 204 | 246 | 195 |
| V2_01 | 36 / 36 | 1 | 1 | — | 1817 / 99 | 65 | 361 |
| V2_02 | 34 / 34 | 2 | 2 | — | 2059 / 236 | 45 | 207 |
| V2_03 | 19 / 19 | 0 | 0 | — | 479 / 378 | 73 | 306 |

`drops_skipped` 与当帧 cull 规模一致：所有 `outliers_culled >= 4` 的帧都被跳过，
无“小 cull 误跳过”发生（MH_01/MH_04 等 cull 少于 4 的序列跳过数为 0）。

## 5. 相对 Slice ④e

7/11 条 `est.tum` 改变，4 条逐字节一致（MH_01、MH_04、V1_01、V2_03）。

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| MH_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_02 | -114115.074324 | -61653.358865 | +0.042105 | -1 | -1 | -128 | 仍灾难发散，但幅度与失败帧数均下降 |
| MH_03 | +0.246303 | +0.032662 | 0 | 0 | 0 | 0 | 轻微回退（无失败帧，纯数值） |
| MH_04 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| MH_05 | -1.508187 | -0.477848 | 0 | 0 | 0 | 0 | 软门 PASS 的关键改善 |
| V1_01 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |
| V1_02 | -6.588439 | -0.442753 | +0.029240 | -2 | -2 | -47 | 灾难发散消除，无 failed |
| V1_03 | -16659711.910291 | -1351819.266734 | +0.265240 | -25 | -25 | -556 | 灾难发散消除，仍 failures |
| V2_01 | -0.814198 | -0.378760 | 0 | 0 | 0 | 0 | 结构相同，轨迹数值改善 |
| V2_02 | -0.039026 | +0.000069 | 0 | 0 | 0 | 0 | 结构相同，数值微变 |
| V2_03 | 0 | 0 | 0 | 0 | 0 | 0 | 逐字节一致 |

跨 coverage、segments 或匹配集合变化时（MH_02、V1_02、V1_03），只陈述数字变化，
不把 ATE 差异直接归因为单一机制。

## 6. 判定与风险

- 原门控（MH_01 硬门、MH_05 软门）双 PASS，④f 当时判定“已完成”；
- skip-drop 消除了两条灾难链：V1_03 ATE `1.67e7 m → 8.45 m`、V1_02 `7.04 m →
  0.45 m` 且 failed 清零；MH_05 ATE `4.565 → 3.057`；
- MH_02 仍是灾难级发散（ATE ≈`1.63e5 m`），V1_03 / V2_03 仍有
  `completed_with_failures`，生命周期 / PnP 失败链未根治；
- MH_03 无失败帧但 ATE 轻微回退（`0.712 → 0.959`），是 skip-drop 对健康序列的
  可观测副作用，需后续 checkpoint 跟踪；
- 4 条逐字节不变说明 skip-drop 只影响触发大规模 cull 的帧，不影响小 cull 序列；
- 后续 zombie-age=5（`4cf55ca/default_773ea011`）在此之上做多帧龄精确 drop。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 11/11 命令完成（loop 无失败退出） |
| artifacts | 11/11 `meta.json` / `summary.json` 齐全 |
| code identity | 11/11 full commit=`c446ac5747476254af684a77a05f726c69bfd69c`，dirty=false |
| config identity | 11/11 `default_a5e90dc7` |
| canonical config | 11 份 41 键 config / canonical text 完全一致 |
| predecessor artifacts | Slice ④e `0ced28b/default_3a21162e` 11/11 齐全 |
| raw artifacts | `/home/lin/Projects/data/phad-benchmark-ledger` |
| 未执行项 | 无 |

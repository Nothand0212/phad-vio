# M3.3 Slice ④e Benchmark（`default_3a21162e`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-04（历史 commit 补跑）

状态：**clean EuRoC 11/11 checkpoint 已补齐**；算法结论仍为“④e 部分完成”

相关：

- issue [#26](https://github.com/Nothand0212/phad-vio/issues/26)
- predecessor：[Slice ④d 原始记录](../../research/m3.3-slice4-baseline.md#9-slice-④d-mean-cull-阈值outlier_avg_reproj_px)
- 设计：[Slice ④e 多轮 cull↔LM](../../research/m3.3-slice4e-multiround-reopt-design.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `0ced28b417a030cefbd63131d52a7243264a22c7`（short `0ced28b`；11/11 `git_dirty=false`） |
| config | `default` / `3a21162e` |
| predecessor | Slice ④d `79505f8/default_85c97158` |
| dataset | EuRoC ASL native 11 条 |
| bench root | `/home/lin/Projects/data/phad-benchmark-ledger` |
| artifact path | `<bench_root>/<sequence>/0ced28b/default_3a21162e/` |
| execution | 独立 clean clone；11 条串行；`phad_vo_bench`；无算法 CLI override |
| run 结果 | 11/11 命令 rc=0；wall time 合计 `3125.851 s` |
| 原 slice 门控 | MH_01 硬门通过；MH_05 相对 ④d 持平，判定不够 |

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

相对 Slice ④d 唯一 config 增量是把多轮上限正式纳入配置：

| 键 | Slice ④d | Slice ④e |
|---|---:|---:|
| `estimator.max_outlier_reopts` | 不存在（实现等价单轮） | `3` |

完整 `config_canonical_text`（39 键）：

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
| MH_02_easy | completed_with_failures | 276879.639366 | 84589.762155 | 0.770066 | 1.000000 | 5 | 4 | 2341 / 3040 | 0 | 699 |
| MH_03_medium | completed_with_warnings | 0.712381 | 0.155634 | 1.000000 | 1.000000 | 1 | 0 | 2700 / 2700 | 0 | 0 |
| MH_04_difficult | completed_with_warnings | 0.260463 | 0.055328 | 1.000000 | 1.000000 | 1 | 0 | 2032 / 2032 | 0 | 0 |
| MH_05_difficult | completed_with_warnings | 4.565065 | 1.250271 | 0.997800 | 1.000000 | 1 | 0 | 2268 / 2273 | 5 | 0 |
| V1_01_easy | completed_with_warnings | 0.172690 | 0.075828 | 0.998626 | 1.000000 | 1 | 0 | 2908 / 2912 | 4 | 0 |
| V1_02_medium | completed_with_failures | 7.042673 | 0.610168 | 0.914035 | 0.944997 | 3 | 2 | 1563 / 1710 | 100 | 47 |
| V1_03_difficult | completed_with_failures | 16659720.362561 | 1351820.141996 | 0.653792 | 0.966946 | 43 | 42 | 1405 / 2149 | 48 | 696 |
| V2_01_easy | completed_with_failures | 6.019109 | 0.998048 | 0.841667 | 1.000000 | 3 | 2 | 1919 / 2280 | 4 | 357 |
| V2_02_medium | completed_with_failures | 4.390714 | 0.936084 | 0.979131 | 0.999574 | 4 | 3 | 2299 / 2348 | 13 | 36 |
| V2_03_difficult | completed_with_failures | 9.407120 | 2.094256 | 0.452889 | 0.719794 | 13 | 12 | 870 / 1921 | 1011 | 40 |

状态分布：1 条 `completed`、4 条 `completed_with_warnings`、6 条
`completed_with_failures`。命令完成不等于上述 failure 序列通过。

## 4. Robustness

本 checkpoint 的 schema 尚无 `drops_skipped` / zombie-age 字段，以 `—` 表示合同
不存在，不写成 0。

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01 | 3 / 3 | 0 | — | — | 3681 / 0 | 0 | 0 |
| MH_02 | 489 / 489 | 44 | — | — | 2272 / 64 | 393 | 281 |
| MH_03 | 23 / 23 | 1 | — | — | 2689 / 10 | 103 | 9 |
| MH_04 | 3 / 3 | 0 | — | — | 2011 / 20 | 0 | 19 |
| MH_05 | 28 / 28 | 1 | — | — | 2238 / 29 | 25 | 26 |
| V1_01 | 18 / 18 | 0 | — | — | 2879 / 28 | 0 | 26 |
| V1_02 | 145 / 145 | 5 | — | — | 1517 / 43 | 334 | 34 |
| V1_03 | 148 / 148 | 6 | — | — | 1171 / 191 | 2381 | 602 |
| V2_01 | 48 / 48 | 2 | — | — | 1816 / 100 | 171 | 361 |
| V2_02 | 38 / 38 | 2 | — | — | 2057 / 238 | 36 | 207 |
| V2_03 | 19 / 19 | 0 | — | — | 479 / 378 | 73 | 306 |

## 5. 相对 Slice ④d

9/11 条 `est.tum` 与 Slice ④d 逐字节一致；只有实际触发新增多轮预算的 V1_02、
V2_02 改变。

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| V1_02 | -25455.173505 | -5058.820957 | +0.123392 | 0 | 0 | -214 | 灾难发散降到 7.04 m，但仍 failures |
| V2_02 | -2.159345 | -0.131699 | +0.056218 | -1 | -1 | -132 | 明显恢复，仍有 36 failed |
| 其余 9 条 | 0 | 0 | 0 | 0 | 0 | 0 | `est.tum` 逐字节一致 |

## 6. 判定与风险

- 新增多轮预算并非“全序列无效”：V1_02 与 V2_02 明显改善；
- MH_05 没有 `rounds>1`，因而与 ④d 持平，原 slice 的软门“不够”仍成立；
- MH_02 ATE≈`2.77e5 m`、V1_03 ATE≈`1.67e7 m`，说明 ④e 远未解决 PnP/生命周期
  灾难链；
- 单看原来的 MH_01/MH_05 门无法看到 V1_02/V2_02 收益，也无法新增发现 V1_03
  灾难规模；这支持关键 checkpoint 全量记录策略。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 11/11 为 0 |
| artifacts | 11/11 `meta.json` / `summary.json` 齐全 |
| code identity | 11/11 full commit=`0ced28b417a030cefbd63131d52a7243264a22c7`，dirty=false |
| config identity | 11/11 `default_3a21162e` |
| canonical config | 11 份 39 键 config / canonical text 完全一致 |
| predecessor artifacts | Slice ④d `79505f8/default_85c97158` 11/11 齐全 |
| raw artifacts | `/home/lin/Projects/data/phad-benchmark-ledger` |
| 未执行项 | 无 |


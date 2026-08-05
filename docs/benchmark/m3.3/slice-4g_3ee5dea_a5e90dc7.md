# M3.3 Slice ④g Benchmark（`default_a5e90dc7`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-04（历史 commit 补跑）

状态：**clean EuRoC 11/11 checkpoint 已补齐**；算法结论为“④g 已证伪并回退编排”，
且本次补跑在全序列尺度上验证 ④g ≡ ④e bit-identical

相关：

- issue [#26](https://github.com/Nothand0212/phad-vio/issues/26)
- predecessor：[Slice ④f](../../benchmark/m3.3/slice-4f_c446ac5_a5e90dc7.md)
- 设计：[Slice ④g zombie-drop](../../research/m3.3-slice4g-zombie-drop-design.md)
- 事后诊断：[postmortem（deferred drop ≡ ④e）](../../research/m3.3-slice4g-postmortem.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `3ee5dea1fa5bdaae67dd0e19db4423be5d749f79`（short `3ee5dea`；11/11 `git_dirty=false`） |
| config | `default` / `a5e90dc7`（**与 ④f 相同 hash**；纯代码差异） |
| predecessor | Slice ④f `c446ac5/default_a5e90dc7` |
| dataset | EuRoC ASL native 11 条 |
| bench root | `/home/lin/Projects/data/phad-benchmark-ledger` |
| artifact path | `<bench_root>/<sequence>/3ee5dea/default_a5e90dc7/` |
| execution | 独立 clean clone；11 条串行；`phad_vo_bench`；无算法 CLI override |
| run 结果 | 11/11 命令 rc=0；wall time 合计 `3818.153 s` |
| 原 slice 门控 | MH_01 硬门 PASS；MH_05 软门 FAIL（ATE 4.565065 = ④e）→ 证伪、编排回退 |

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

相对 Slice ④f **无任何 config 增量**：41 键 canonical 完全相同。④g 的差异全部在
代码：`skip_drop_min_culled` 触发时把 culled ids 放入 `pending_drop`，在下一帧
`process` 前统一 flush `dropTracks`（④f 是当帧直接跳过、永不 drop）。④g 由此
新增 `deferred_drops` / `deferred_drop_ids` 计数合同。

完整 `config_canonical_text`（41 键，与 ④f 相同）：

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

## 4. Robustness

| sequence | culled / unique | reopts | deferred drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|
| MH_01 | 3 / 3 | 0 | 0 / 0 | 3681 / 0 | 0 | 0 |
| MH_02 | 489 / 489 | 44 | 44 / 569 | 2272 / 64 | 393 | 281 |
| MH_03 | 23 / 23 | 1 | 1 / 68 | 2689 / 10 | 103 | 9 |
| MH_04 | 3 / 3 | 0 | 0 / 0 | 2011 / 20 | 0 | 19 |
| MH_05 | 28 / 28 | 1 | 1 / 32 | 2238 / 29 | 25 | 26 |
| V1_01 | 18 / 18 | 0 | 0 / 0 | 2879 / 28 | 0 | 26 |
| V1_02 | 145 / 145 | 5 | 5 / 249 | 1517 / 43 | 334 | 34 |
| V1_03 | 148 / 148 | 6 | 7 / 306 | 1171 / 191 | 2381 | 602 |
| V2_01 | 48 / 48 | 2 | 3 / 101 | 1816 / 100 | 171 | 361 |
| V2_02 | 38 / 38 | 2 | 2 / 31 | 2057 / 238 | 36 | 207 |
| V2_03 | 19 / 19 | 0 | 0 / 0 | 479 / 378 | 73 | 306 |

`deferred_drops` / `deferred_drop_ids` 是 ④g 新增合同，表示 skip 当帧后延后 flush
的帧数与累计 track id 数。`drops_skipped` 字段在本 checkpoint schema 中不存在
（④g 代码把 skip 计数改为 deferred 口径）。

## 5. 相对 predecessor（④f）

**10/11 条 `est.tum` 与 ④f 不同；无一条一致。**

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| MH_01 | 0 | 0 | 0 | 0 | 0 | 0 | 与 ④f 数值相同（无 skip 触发） |
| MH_02 | +114115.074324 | +61653.358865 | -0.042105 | +1 | +1 | +128 | 回到 ④e 灾难平台 |
| MH_03 | -0.246303 | -0.032662 | 0 | 0 | 0 | 0 | 回到 ④e 数值（④f 的轻微回退消失） |
| MH_04 | 0 | 0 | 0 | 0 | 0 | 0 | 与 ④f 数值相同 |
| MH_05 | +1.508187 | +0.477848 | 0 | 0 | 0 | 0 | 软门 FAIL：回到 ④e 灾难平台 |
| V1_01 | 0 | 0 | 0 | 0 | 0 | 0 | 与 ④f 数值相同 |
| V1_02 | +6.588439 | +0.442753 | -0.029240 | +2 | +2 | +47 | 回到 ④e 的 7.04 m |
| V1_03 | +16659711.910291 | +1351819.266734 | -0.265240 | +25 | +25 | +556 | 回到 ④e 灾难级发散 |
| V2_01 | +0.814198 | +0.378760 | 0 | 0 | 0 | 0 | 回到 ④e 数值 |
| V2_02 | +0.039026 | -0.000069 | 0 | 0 | 0 | 0 | 回到 ④e 数值 |
| V2_03 | 0 | 0 | 0 | 0 | 0 | 0 | 与 ④f 数值相同 |

判读：④g 的延后一帧 drop 把所有被 ④f skip-drop 拯救的序列打回 ④e 平台，证实
“延后 flush”相对“立即 drop”在观测上等价于“不 drop”的反面——即 ④f 的收益来自
**这批 culled track 不再被 drop**，而非 drop 的时机。

## 6. 与 ④e 的全序列等价性（本次补跑新证据）

④g 与 ④e（`0ced28b/default_3a21162e`）逐字节对比：

| 产物 | 对比结果 |
|---|---|
| `est.tum` | **11/11 逐字节一致** |
| `diag.csv` | 抽查 MH_01–MH_05 共 5/5 逐字节一致 |

postmortem 仅对 MH_05 验证的「④g ≡ ④e bit-identical」结论，本次在全序列尺度
成立：`skip 当帧 + 下帧 process 前 flush` 与 `当帧立即 drop` 相对下一帧
`process` 观测等价，与序列无关。culled / reopts / cheirality / low connectivity
等 robustness 计数亦与 ④e 完全一致（④g 仅多出 deferred 合同计数）。

## 7. 判定与风险

- ④g 门控 FAIL 得到全序列确认：MH_05 ATE 4.565065 = ④e，且 11/11 全部回到
  ④e 平台（含 V1_03 ATE `1.67e7 m` 灾难级发散）；
- ④g 与 ④f 共享同一个 config hash（`a5e90dc7`）却产生完全不同的轨迹——本
  checkpoint 是“config identity 不能替代 code identity”的直接证据；
- 编排已回退（删除 `pending_drop`，恢复 ④f skip-without-drop），`deferred_*`
  合同保留恒 0；后续 zombie-age=5（`4cf55ca`）在 ④f 语义上继续演进；
- 无不可比较项；全部 delta 均在同一 config/canonical 上产生。

## 8. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | 11/11 命令 rc=0 |
| artifacts | 11/11 `meta.json` / `summary.json` 齐全 |
| code identity | 11/11 full commit=`3ee5dea1fa5bdaae67dd0e19db4423be5d749f79`，dirty=false |
| config identity | 11/11 `default_a5e90dc7`（与 ④f 相同 hash，代码不同） |
| canonical config | 11 份 41 键 config / canonical text 完全一致 |
| predecessor artifacts | Slice ④f `c446ac5/default_a5e90dc7` 11/11 齐全；④e `0ced28b/default_3a21162e` 11/11 齐全 |
| raw artifacts | `/home/lin/Projects/data/phad-benchmark-ledger` |
| 未执行项 | 无 |

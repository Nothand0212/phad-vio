# M5 accumulated cold-root qualification

状态：evidence valid；capability verdict 为 **`root_committed`**。在一次完整
`V2_03_difficult` global treatment 中，现有 accumulated-origin 路径产生 18 个真实
`seedRoot()` attempt，18 个均通过 geometry/current-graph/commit gate，并在同一
timestamp 产生 `ok` estimate 与 session output。

本结果证明当前基线上的 existing default-off capability 可越过 cold-root population
gate 并提交 root。treatment 在 control terminal epoch 之前已经分叉，因此结论范围是
**system-level qualification**。它不构成默认启用决策，也不改变 #47 formal gate：
跨-root world-frame alignment 仍为 **FAIL**。

## 1. Identity 与执行范围

| 项 | 值 |
|---|---|
| issue / parent | [#50](https://github.com/Nothand0212/phad-vio/issues/50) / [#42](https://github.com/Nothand0212/phad-vio/issues/42) |
| frozen spec / plan | [spec](../../specs/2026-08-28-m5-accumulated-cold-root-qualification.md) / [plan](../../plans/2026-08-28_m5_accumulated_cold_root_qualification_54ee0e04.plan.md) |
| candidate | `0a32ab713235ce633112760f40aa267e51f80ed9`（`0a32ab7`） |
| tree / parent | `e5db60655d9a4a03c4a3a3c886570839b4561815` / `a007cc8748214daca6d2b84acef0e419e61bcbb2` |
| branch / run clean | `codex/m5-cold-root-accumulated-qualify` / `git_dirty=false` |
| candidate source delta | 相对 `a007cc8` 仅 frozen spec/plan；production/app/test/build source diff 为空 |
| config | `default_0337287b`；canonical object/text 与 frozen control 相同 |
| dataset | EuRoC ASL native `V2_03_difficult` |
| control | [#48 checkpoint](cold-root-current-path-observe_6a193a8_0337287b.md) / [evidence JSON](cold-root-current-path-observe_6a193a8_0337287b.evidence.json) |
| control artifact | `/home/lin/Projects/data/phad-bench/m5-cold-root-observe-20260828T070950Z/V2_03_difficult` |
| treatment artifact | `/home/lin/Projects/data/phad-bench/m5-cold-root-accumulated-0a32ab7-20260828T084905Z/V2_03_difficult` |
| treatment run count | `1`；control 未重跑；未运行其它 sequence 或 tail replay |
| machine evidence | [完整审计 JSON](cold-root-accumulated-qualification_0a32ab7_0337287b.evidence.json) |

canonical invocation：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/V2_03_difficult \
  --out /home/lin/Projects/data/phad-bench/m5-cold-root-accumulated-0a32ab7-20260828T084905Z/V2_03_difficult \
  --sequence-name V2_03_difficult \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-accumulated-qualify \
  --estimator-enable-moving-bootstrap \
  --estimator-enable-accumulated-seed
```

命令 `rc=0`，终态 `completed_with_warnings`。summary wall time 为
`80.533947244 s`，外层计时为 `80.66 s`。warnings 为既有 stereo sync drop 与 PnP
summary；artifact 可解析、row/output 对账均通过。

## 2. Evidence validity：E0–E6

| gate | 结果 | 对账证据 |
|---|---|---|
| E0 candidate identity | PASS | commit/tree/parent、merge-base、meta code identity 与 clean run 一致；non-doc source diff 为空 |
| E1 treatment identity | PASS | exact argv 含两个 existing flags；555 个 treatment cold-root row 的 accumulated option snapshot 全为 `1` |
| E2 control comparability | PASS | dataset、sequence、64-column schema、timestamp axis、canonical config、moving bootstrap 与实际 threshold 一致 |
| E3 artifact integrity | PASS | 五个必需 artifact 均存在并可解析；`diag rows = sync emitted = 1921`；timestamp 唯一且严格递增；hash 已冻结 |
| E4 typed invariant | PASS | 1921/1921 rows 的 phase/reason/gate/presence/origin/attempt 组合合法；违规数 `0` |
| E5 attempt reconciliation | PASS | IDs `1..22` 唯一、连续且严格单调；每个 attempt 的 origin/population/threshold 同 row 对账 |
| E6 output reconciliation | PASS | 22 个 commit 均为 `ok`；timestamp 全部出现在 `est.tum` 与 `kf.tum`；`est.tum` timestamp 序列与全部 `ok` rows 精确相等 |

`enable_accumulated_seed` 是 CLI-only runtime option，不进入 `meta.config` 或
`config_hash`。本次不以 hash 推断 treatment identity：exact command 与 typed
`cold_root_accumulated_seed_enabled=1` snapshot 共同关闭该 blind spot；frozen control
对应 snapshot 为 `0`。

## 3. Treatment capability ledger

### 3.1 全程 taxonomy

| phase / reason | rows |
|---|---:|
| `not_evaluated / not_evaluated` | 1366 |
| `keyframe / keyframe_required` | 15 |
| `stereo_population / population_accumulating` | 518 |
| `commit / committed` | 22 |

23 个连续 cold-root episodes 中：

- 18 个 accumulated-origin episode 达到现有 threshold 并 commit；
- 4 个 attempt 使用 current-packet origin，作为 treatment 全程 ledger 的既有路径记录；
- 1 个 sequence-tail episode 停在 `population_accumulating`，未产生 attempt；
- 所有 22 个 attempt 均为 `geometry accepted → current graph passed → commit passed`，
  没有 natural rejection、graph failure 或 rollback。

accumulated-origin commit ledger：

| attempt IDs | commit time(s) | current → effective seed |
|---|---|---|
| `1, 2, 3` | `6.65, 7.35, 11.20` | `3→10, 7→11, 9→12` |
| `6, 7` | `38.65, 43.40` | `1→10, 3→10` |
| `10, 11, 12` | `60.15, 69.95, 75.05` | `3→11, 2→11, 7→16` |
| `13, 14, 15` | `76.55, 77.25, 91.50` | `7→11, 7→11, 7→14` |
| `16, 17, 18` | `97.10, 98.35, 99.40` | `4→11, 4→10, 7→11` |
| `19, 20, 21, 22` | `108.65, 109.55, 111.00, 112.90` | `2→11, 4→10, 5→10, 1→10` |

每个 accumulated attempt 的 pending unique 与 effective count 相等，且不小于实际
threshold `10`。18 个 commit timestamp 全部同时存在于 `est.tum` 与 `kf.tum`；
accepted landmark count 与 effective seed count 逐项一致。逐 episode 的 timestamp、
segment/status cadence、population range、origin、attempt 与 post-commit output 区间见
[machine evidence](cold-root-accumulated-qualification_0a32ab7_0337287b.evidence.json)。

### 3.2 Transaction 与 pending 状态

518 个 `population_accumulating` row 表示 transaction 已提交的 pending population；
这些 row 全部保持 `effective < threshold`、population gate failed、无 attempt ID。
accumulated commit row 保存的是 pre-commit diagnostic snapshot；accepted root 路径随后
清空 pending，再完成 estimator transaction commit。

本次 natural artifact 未出现 geometry/current-graph failure，因此不把 rollback 记为运行时
覆盖。既有定向测试
`ColdRootObserveAttempt.GeometryRollbackConsumesIdAndNextCommitUsesTheNextId` 与
`VioFullState.AccumulatedRootFailureDoesNotLeakPendingObservations` 分别覆盖 attempt-ID
消费和 pending rollback 语义。

## 4. First divergence 与 terminal observation

typed diagnostics 从 row 1 即按 treatment identity 分叉：reason
`population_insufficient → population_accumulating`、origin
`current_packet → accumulated`，并出现 option/pending snapshot。

旧 26 列 projection 的前 133 行相同；首个行为差异在 row 134、`6.65 s`：
treatment 已 `ok`、window size `1`，并保留/播种 10 个 observation；control 仍为
`initializing`。`est.tum` 与 `kf.tum` 的第一个 timestamp 同样是 treatment
`6.65 s`、control `7.65 s`。因此 control terminal epoch 只用于 system-level
observation，不构成隔离 tail A/B。

在 frozen control terminal `96.6–116.7 s` 的同一 289 个 timestamp 上，treatment 为：

| 观察项 | treatment |
|---|---|
| status | `135 ok / 147 initializing / 7 visual_outage` |
| attempts / commits | IDs `16..22`；7/7 accumulated-origin commit |
| estimate / keyframe output | `135 / 72` |
| last estimate | `113.4 s` |

最后 `113.6–116.7 s` 为 32-row `population_accumulating` episode：current、pending 与
effective population 均为 `0`，没有 attempt。现有路径可以恢复多个 cold root，但本次
artifact 没有显示 sequence endpoint 的持续 root availability。

## 5. Record-only metrics

以下数据不参与 capability verdict：

| metric | treatment | 相对 frozen control |
|---|---:|---:|
| completion | `0.7110879750` | `+0.1171264966` |
| coverage | `0.9147386472` | `+0.1533847472` |
| poses / segments | `1366 / 22` | `+225 / +13` |
| ATE translation RMSE | `1.9820268464 m` | `-0.0597891698 m` |
| RPE translation RMSE | `0.6701916858 m` | `+0.0359567512 m` |
| weighted local RMS / intersegment component | `0.071 / 1.911 m` | existing script 的 0.001 m display precision |

`segment_ate_decomp.py` 在 frozen treatment artifact 上对 1366 个 output、22 个 segments
完成 record-only 分解；全局 ATE 的脚本输出与 summary 在显示精度内一致。完整 summary、
timing、robustness、sync 与逐 segment 记录位于 machine evidence。

## 6. Validation 与 claim boundary

canonical run 前的 existing-path preflight：

- Release configure 与 `phad_estimator_tests`、`phad_apps_tests`、`phad_vo_bench` build
  passed；
- accumulated lifecycle / typed diagnostics / rollback 定向 estimator tests 4/4 passed；
- session/CSV 定向 apps tests 4/4 passed；
- estimator suite 137/137 passed；
- apps suite 42 passed，3 个依赖 `PHAD_EUROC_MH01_PATH` 的条件测试 skipped；
- candidate source delta、64-column header、control evidence 与 artifact target identity
  检查通过。

该 qualification 复用现有 option、threshold、CLI、transaction 与 diagnostics；没有增加
production injection seam，也没有修改 production/test behavior。结论只覆盖
accumulated population、真实 `seedRoot()` 调用、root transaction commit 与对应 output。
default policy 与 #47 world-frame alignment gate 保持独立。

## 7. Artifact hashes

| artifact | bytes | SHA-256 |
|---|---:|---|
| `diag.csv` | 738598 | `9ee682b1b2a5cc134acb055817631cb08e1719a275c29d40aa4ba94c1fce7670` |
| `est.tum` | 222812 | `39a788200facebf3b6ce39c10e2c27eba17a6a9c797eedce1e30e983b4d3268b` |
| `kf.tum` | 167680 | `52219d4210a81b88b042ae5b069f7c118c424e3da485cbbf2deb4cf2c08ee88a` |
| `meta.json` | 3930 | `834c1ae08fb43e531d7fa367a70870a8d1ebf1a951f74153f140878429d2100c` |
| `summary.json` | 2701 | `cebe99f4bad050f21e7b4fb962cc3e8e1956b1ae1e97890bb330b1d498e691de` |

full control hashes、legacy projection hashes、candidate binary hash 与所有算术对账见
[machine evidence](cold-root-accumulated-qualification_0a32ab7_0337287b.evidence.json)。

## 8. Config canonical text

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

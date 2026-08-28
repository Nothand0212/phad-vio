# M4 mapped-landmark bearing continuity 本机终验

状态：**clean EuRoC 11/11 终验完成**。mapped observation、mono factor、mapped PnP 与 support recovery 均在自然回放中实际触发；core-4、EuRoC-11 accuracy 与 suite availability 通过冻结 envelope。`V2_03_difficult` 的 continuity 五项中，segments、completion、段内 RMS 与绝对段间分量通过，endpoint coverage 为 `0.761354 < 0.791345`，因此 #47 的正式产品判定为 **FAIL**，本行为尚未获得进入当前 stereo VIO 默认路径的授权。

## 1. 身份与执行范围

| 项 | 值 |
|---|---|
| candidate | `db226563c9a386bc70e4f19665ec909bd80905ad`（`db22656`） |
| tree | `a011a891794e8137347e794dd267ce25a824c306` |
| source worktree | detached clean worktree，`git_dirty=false` |
| config | `default_0337287b`，与 Q0 canonical text 逐字一致 |
| immediate control | `42f99e9/default_0337287b` clean Q0 |
| mechanism predecessor | Q4 `5425ab9/default_0337287b` |
| dataset | EuRoC ASL native，固定 11 条 |
| dataset root | `/home/lin/Projects/data/thidparty/euroc/native` |
| raw artifact root | `/home/lin/Projects/data/phad-bench/m4-mapped-bearing-final-db22656-20260828T022252Z` |
| run time | 2026-08-28 10:24–10:39（Asia/Shanghai）；11 条串行 |
| benchmark wall time | `926.432 s` |
| build | Release；GNU 13.3.0；CMake 3.31.11；Ninja 1.13.0；OpenCV 4.6.0；GTSAM 4.3a0 |
| evidence | [完整审计 JSON](mapped-landmark-bearing-continuity_db22656_0337287b.evidence.json) |

实际命令：

```bash
git worktree add --detach /tmp/phad-mapped-bearing-final-gJBW9y/worktree db22656
cmake -S . -B build -G Ninja \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build --parallel 8
ctest --test-dir build --output-on-failure -L unit --parallel 8

./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence> \
  --out /home/lin/Projects/data/phad-bench/m4-mapped-bearing-final-db22656-20260828T022252Z/<sequence> \
  --sequence-name <sequence> \
  --repo /tmp/phad-mapped-bearing-final-gJBW9y/worktree \
  --estimator-enable-moving-bootstrap
```

clean candidate 的 Release unit suite 为 `478/478` 通过、0 failed；3 个需要
显式 EuRoC CMake 路径的 session tests 按既有条件 skipped。EuRoC 11 条命令均
rc=0、终态为 `completed_with_warnings`；warning 只包含 PnP 汇总，以及
`MH_04_difficult`、`V1_02_medium`、`V2_03_difficult` 的既有 stereo sync
unpaired-frame 记录。

## 2. 产品判定

| gate | 判定 | 证据 |
|---|---|---|
| identity / hard validity | PASS | 11/11 clean identity；55/55 核心文件非空；ATE/RPE 有限；`failed=rejected=reanchors=0` |
| mechanism | PASS | 25,874 个 eligible rows、25,863 个 mono-attached rows、551 个 flip-capable rows |
| core-4 accuracy | PASS | `G_ATE=0.654556`，`G_RPE=0.751251` |
| EuRoC-11 accuracy | PASS | `G_ATE=0.825808`，`G_RPE=0.839121` |
| suite availability | PASS | mean completion `0.949413 >= 0.933409`；mean coverage `0.964624 >= 0.956954` |
| `V2_03` continuity | FAIL | coverage `0.761354 < 0.791345`；其余四项通过 |
| formal product gate | **FAIL** | continuity 合同为五项联合条件；当前结果不授权 default-path adoption |

EuRoC-11 的归一化几何平均显示：ATE 相对 Q0 改善
`17.42%`，RPE 改善 `16.09%`。
算术均值 ATE 从 `0.380628 m` 降到
`0.301484 m`，RPE 从
`0.113120 m` 降到 `0.097207 m`。
平均 completion 增加 `0.600 pp`；
平均 coverage 下降 `-0.233 pp`，
但仍通过 suite floor。segments 总数从 `30` 降到
`19`。

## 3. EuRoC-11 全量结果

### 3.1 Accuracy、availability 与 segments

| sequence | ATE Q0 → candidate (m) | ATE ratio | RPE Q0 → candidate (m) | RPE ratio | completion | coverage | segments | review |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `MH_01_easy` | `0.072559 → 0.121985` | `1.6812` | `0.032641 → 0.031269` | `0.9580` | `0.999728` | `0.999728` | `1 → 1` | ATE |
| `MH_02_easy` | `0.092645 → 0.115345` | `1.2450` | `0.025542 → 0.024909` | `0.9752` | `1.000000` | `1.000000` | `1 → 1` | ATE |
| `MH_03_medium` | `0.094969 → 0.104187` | `1.0971` | `0.042921 → 0.041655` | `0.9705` | `1.000000` | `1.000000` | `1 → 1` | — |
| `MH_04_difficult` | `0.178573 → 0.167183` | `0.9362` | `0.058907 → 0.056172` | `0.9536` | `1.000000` | `1.000000` | `1 → 1` | — |
| `MH_05_difficult` | `0.258856 → 0.156579` | `0.6049` | `0.058137 → 0.052785` | `0.9079` | `1.000000` | `1.000000` | `1 → 1` | — |
| `V1_01_easy` | `0.112073 → 0.112663` | `1.0053` | `0.062536 → 0.046351` | `0.7412` | `1.000000` | `1.000000` | `2 → 1` | — |
| `V1_02_medium` | `0.072210 → 0.071860` | `0.9952` | `0.043866 → 0.042055` | `0.9587` | `0.944412` | `0.944379` | `1 → 1` | — |
| `V1_03_difficult` | `0.861563 → 0.155423` | `0.1804` | `0.130757 → 0.063051` | `0.4822` | `0.974872` | `0.974860` | `2 → 1` | — |
| `V2_01_easy` | `0.190590 → 0.163439` | `0.8575` | `0.043477 → 0.034044` | `0.7830` | `0.955702` | `0.955682` | `1 → 1` | — |
| `V2_02_medium` | `0.171546 → 0.105839` | `0.6170` | `0.057113 → 0.042747` | `0.7485` | `0.974872` | `0.974862` | `1 → 1` | — |
| `V2_03_difficult` | `2.081321 → 2.041816` | `0.9810` | `0.688425 → 0.634235` | `0.9213` | `0.593961` | `0.761354` | `18 → 9` | coverage |

ATE 为 7 条改善、4 条上升，其中 `MH_01_easy`、`MH_02_easy` 触发冻结的
ATE tail review；RPE 为 **11/11 改善**。所有 sequence 的 segments 均未增加，
`V1_01_easy`、`V1_03_difficult` 与 `V2_03_difficult` 分别从
`2→1`、`2→1`、`18→9`。

### 3.2 段内与绝对段间分量

| sequence | 段内加权 RMS Q0 → candidate (m) | 绝对段间分量 Q0 → candidate (m) |
|---|---:|---:|
| `MH_01_easy` | `0.072559 → 0.121985` | `0.000000 → 0.000000` |
| `MH_02_easy` | `0.092645 → 0.115345` | `0.000000 → 0.000000` |
| `MH_03_medium` | `0.094969 → 0.104187` | `0.000000 → 0.000000` |
| `MH_04_difficult` | `0.178573 → 0.167183` | `0.000000 → 0.000000` |
| `MH_05_difficult` | `0.258856 → 0.156579` | `0.000000 → 0.000000` |
| `V1_01_easy` | `0.097927 → 0.112663` | `0.014146 → 0.000000` |
| `V1_02_medium` | `0.072210 → 0.071860` | `0.000000 → 0.000000` |
| `V1_03_difficult` | `0.143581 → 0.155423` | `0.717982 → 0.000000` |
| `V2_01_easy` | `0.190590 → 0.163439` | `0.000000 → 0.000000` |
| `V2_02_medium` | `0.171546 → 0.105839` | `0.000000 → 0.000000` |
| `V2_03_difficult` | `0.061664 → 0.060236` | `2.019657 → 1.981580` |

单段 sequence 的段内 RMS 等于全局 ATE，绝对段间分量为零。
`V1_03_difficult` 从两段变为单段，ATE 从 `0.861563 m` 降到
`0.155423 m`；`V2_03_difficult` 的段内 RMS 从 `0.061664 m`
降到 `0.060236 m`，绝对段间分量从 `2.019657 m` 降到
`1.981580 m`。这说明全局困难序列的主要剩余误差仍是跨 root 的世界系关系，
而不是 active segment 内部精度。

## 4. 机制在自然回放中实际生效

| sequence | eligible rows / episodes | mono-attached rows | flip rows / episodes | mono factors sum / max | PnP success / fallback |
|---|---:|---:|---:|---:|---:|
| `MH_01_easy` | `3680` / `1` | `3680` | `0` / `0` | `82907` / `67` | `3670` / `10` |
| `MH_02_easy` | `3039` / `1` | `3039` | `29` / `11` | `68887` / `63` | `3028` / `11` |
| `MH_03_medium` | `2695` / `5` | `2695` | `0` / `0` | `61068` / `147` | `2689` / `10` |
| `MH_04_difficult` | `2025` / `5` | `2025` | `4` / `3` | `42630` / `147` | `2028` / `3` |
| `MH_05_difficult` | `2270` / `2` | `2270` | `3` / `2` | `43986` / `164` | `2267` / `4` |
| `V1_01_easy` | `2910` / `1` | `2910` | `4` / `4` | `73856` / `128` | `2906` / `4` |
| `V1_02_medium` | `1609` / `2` | `1609` | `0` / `0` | `36794` / `113` | `1610` / `3` |
| `V1_03_difficult` | `2075` / `14` | `2073` | `50` / `22` | `38255` / `125` | `2041` / `50` |
| `V2_01_easy` | `2166` / `3` | `2166` | `11` / `4` | `58257` / `103` | `2161` / `16` |
| `V2_02_medium` | `2279` / `6` | `2278` | `68` / `45` | `53320` / `104` | `2274` / `11` |
| `V2_03_difficult` | `1126` / `20` | `1118` | `382` / `125` | `16298` / `91` | `985` / `136` |
| **总计** | **25874 / 60** | **25863** | **551 / 216** | **576258 / —** | **25659 / 258** |

eligible row 定义为
`num_mapped_observations > num_shared`；flip-capable successful commit 定义为
`status=ok && num_shared < 10 <= num_mapped_observations`。全量有
`25874` 个 eligible rows，其中
`25863` 个实际附着 mono factor。
逐帧 `num_current_visual_factors >= num_current_mono_visual_factors` 11/11
成立，非 `ok` row 的 current factor count 均为零。

相对 Q4，Q5 的直接作用在两条开发序列上均可见：

| sequence | ATE Q4 → Q5 (m) | RPE Q4 → Q5 (m) | completion | coverage | segments |
|---|---:|---:|---:|---:|---:|
| `V2_02_medium` | `0.129114 → 0.105839` | `0.048336 → 0.042747` | `0.974446 → 0.974872` | `0.974435 → 0.974862` | `1 → 1` |
| `V2_03_difficult` | `2.150046 → 2.041816` | `0.696704 → 0.634235` | `0.535138 → 0.593961` | `0.791345 → 0.761354` | `18 → 9` |

`V2_03` 的 PnP 从 Q4 的 `591 success / 402 fallback` 变化为
`985 / 136`；mapped-observation PnP 与 support population 因此不仅存在于
synthetic tests，也改变了 natural replay 的实际 commit 分布。

## 5. V2_03：active-map continuity 改善，末端 cold-root 未恢复

| continuity component | Q0 | candidate | 冻结条件 | 结果 |
|---|---:|---:|---:|---|
| segments | `18` | `9` | `<= 17` | PASS |
| completion | `0.535138` | `0.593961` | `>= 0.555138` | PASS |
| coverage | `0.791345` | `0.761354` | `>= 0.791345` | **FAIL** |
| 段内 RMS (m) | `0.061664` | `0.060236` | `<= 0.070000` | PASS |
| 绝对段间分量 (m) | `2.019657` | `1.981580` | `<= 2.020000` | PASS |

状态帧由 Q0 的 `1028 ok / 875 initializing / 18 visual_outage` 变为
`1141 / 771 / 9`：candidate 多出 113 个 ok frames，少 104 个 initializing
frames，outage 次数减半。first valid pose 都在相对 `7.65 s`；Q0 在
`99.5–100.0 s` 还完成过一次短暂 cold-root 恢复，因此 last valid pose 为
`100.0 s`。candidate 的最后一个 active segment 更长地延续到 `96.5 s`，
但从 `96.6 s` 到序列结束 `116.7 s` 始终处于 initializing，last valid pose
因而提前 3.5 s，endpoint coverage 下降 2.999 pp。

这不是整体帧可用性下降：completion 与 suite mean 都改善；它精确暴露的是
**map 丢失后的 root 初始化仍不能稳定完成**。末端 initializing epoch 中
`num_obs` 最高 200、`num_disparity` 最高 9，但没有形成新的 full-state root。

## 6. Accuracy tail review

`MH_01_easy` 与 `MH_02_easy` 都保持单段、完整 completion/coverage，且
RPE 分别 `0.032641→0.031269 m`、`0.025542→0.024909 m`，所以 ATE tail
不属于 restart/coverage 退化。按匹配样本时间四分位复算对齐后 translation
error RMS：

| 时间四分位 | MH_01 Q0 → candidate (m) | MH_02 Q0 → candidate (m) |
|---|---:|---:|
| Q1 | `0.057619 → 0.114944` | `0.072026 → 0.094828` |
| Q2 | `0.047929 → 0.086555` | `0.048143 → 0.075990` |
| Q3 | `0.055415 → 0.092258` | `0.059404 → 0.080662` |
| Q4 | `0.111207 → 0.174062` | `0.152622 → 0.178714` |

两条的误差在四个区间都上升，末四分位绝对增量最大；这更符合 mono bearing /
mapped PnP 改变单段轨迹形状的表现。其 candidate ATE 分别为
`0.121985 m` 与 `0.115345 m`，绝对量级仍低于 0.13 m，且完整 11 序列的
ATE/RPE aggregate 明确改善。因此这两条保留为 regression monitoring tail，
不改变本次由 `V2_03` coverage 形成的正式 gate 结论。

## 7. 数据质量与可复现性

| 检查 | 结果 |
|---|---|
| required artifacts | 11/11、55/55 个 `meta.json`、`summary.json`、`diag.csv`、`est.tum`、`kf.tum` 非空 |
| identity | 11/11 full commit=`db226563c9a386bc70e4f19665ec909bd80905ad`，`git_dirty=false` |
| config / input | 11/11 `default_0337287b`；canonical text 与 Q0 一致；sequence root 精确匹配 |
| diag schema / grain | 11/11 为 26 列；每个 image frame 一行；row count 与 `trajectory.image_frames` 一致 |
| timestamps / linkage | diag、est、kf 各自严格递增；est 精确等于 `status=ok` timestamps；kf 精确等于 `ok && is_keyframe=1` 子集 |
| numeric / lifecycle | numeric cells 有限且非负；`num_shared <= num_mapped_observations <= num_obs`；factor count 合同成立 |
| status / hard counts | status 只含 `initializing`、`ok`、`visual_outage`；`failed=rejected=reanchors=0` |
| independent spot-check | `jq/awk` 独立复得 core-4、EuRoC-11 aggregate 与 V2_03 机制计数 |
| manifest | 55 个核心文件按 evidence 定义连接后的 SHA-256 为 `51fbf01fa5baf05b82da7ff1cb6fc90af6025e28140e78afca1287dcf83dbe8e` |

完整逐文件 SHA-256、逐序列 QA boolean、分段明细和 chart map 见
[审计 JSON](mapped-landmark-bearing-continuity_db22656_0337287b.evidence.json)。

## 8. Canonical config

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

相对 Q0 没有 config 增量键；唯一 CLI override
`--estimator-enable-moving-bootstrap` 已进入 canonical snapshot。

## 9. 方法、限制与结论边界

- ATE/RPE 以 clean Q0 为 immediate control；每条 sequence 等权，suite
  使用 normalized geometric mean。
- 每段以 `scripts/segment_ate_decomp.py` 调用同一个
  `phad_traj_eval` 独立 SE(3) 对齐，再按 active pose frames 加权 RMS。
- 逐序列 tail 只触发归因；正式判定同时使用 aggregate、availability、
  mechanism 与 continuity 合同。
- 本结果来自同一 clean candidate 的一次 matched deterministic run，未估计重复运行方差。
- coverage 是 first/last valid pose 的端点跨度，completion 是逐帧有效比例；
  两者在末端短暂 rebootstrap 存在与否时可能方向不同，因此均按冻结合同保留。
- 机制计数证明代码路径真实触发；精度变化的解释依赖 matched Q0/Q4 与完整
  EuRoC-11 联合证据，不把时序相关性表述为单一因果。

## 10. 下一 vertical slice

下一方向应进入 **M5 dynamic initialization / moving cold-root initialization**，
先处理 map 丢失后的 full-state root，而不是继续扩大 active-map support 权限。

首个 Observe 问题：

> `V2_03` 在 `96.6–116.7 s` 的 terminal initializing epoch 中，
> `num_obs` 最高 200、`num_disparity` 最高 9 时，root seed 被哪一条
> population、motion excitation、conditioning 或 solve 条件持续拒绝？

建议的最小切片：

1. 只增加 root-seed eligibility / rejection diagnostics，记录 observation
   population、有效 stereo seed、IMU excitation、conditioning 与失败原因；
2. 以 V2_03 末端 epoch 和 TUM VI hand-held start 作为 Observe 输入；
3. 保持 `db22656` 的 active-map factor、mapped PnP 与 support semantics 不变；
4. 复用当前 EuRoC-11 accuracy、suite availability 与 V2_03 continuity 合同验收；
5. `MH_01/MH_02` ATE tail 继续作为单段 accuracy guardrail，重点监控 RPE 与
   时间分桶误差。

当前证据不要求优先扩大 relocalization：active segment 内 RMS 已稳定、support
明显减少 restart；最直接的阻塞是 terminal cold-root 没有建立。跨 segment 的
world-frame 关系仍是后续独立问题，保留在 dynamic initialization 之后评估。

## 11. 原始证据

- candidate raw：`/home/lin/Projects/data/phad-bench/m4-mapped-bearing-final-db22656-20260828T022252Z`
- Q0 raw：`/home/lin/Projects/data/phad-bench/m4-mapped-bearing-q0-42f99e9-20260827T134813Z`
- Q4 raw：`/home/lin/Projects/data/phad-bench/m4-mapped-bearing-q4-5425ab9-20260828T005013Z`
- machine-readable evidence：
  [`mapped-landmark-bearing-continuity_db22656_0337287b.evidence.json`](mapped-landmark-bearing-continuity_db22656_0337287b.evidence.json)
- gate contract：
  [`2026-08-27-m4-mapped-landmark-bearing-continuity.md`](../../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md)

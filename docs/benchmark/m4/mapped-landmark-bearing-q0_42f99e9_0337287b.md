# M4 mapped-landmark bearing Q0 control

本文记录 [mapped-landmark bearing continuity spec](../../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md)
§13.1 的 clean Q0 control。core-4 与 EuRoC-11 均已在同一台本机完整运行，11 条
artifact 已通过统一数据质量审计。

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
| run date | `2026-08-27`（Asia/Shanghai）；11 条分阶段串行续跑 |
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

完整 unit suite 为 `458/458` 通过；另有 3 个既有 skip。11 条 bench 均返回
`completed_with_warnings`；每条都有 PnP 汇总，`MH_04_difficult`、
`V1_02_medium`、`V2_03_difficult` 另有 stereo sync drop warning。

## 2. EuRoC-11 结果

| sequence | ATE (m) | RPE (m) | completion | coverage | segments | 段内 RMS (m) | 绝对段间分量 (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| `MH_01_easy` | `0.072559262` | `0.032641273` | `0.999728408` | `0.999728335` | `1` | `0.072559` | `0.000` |
| `MH_02_easy` | `0.092644891` | `0.025542018` | `1.000000000` | `1.000000000` | `1` | `0.092645` | `0.000` |
| `MH_03_medium` | `0.094969054` | `0.042920880` | `1.000000000` | `1.000000000` | `1` | `0.094969` | `0.000` |
| `MH_04_difficult` | `0.178573179` | `0.058907064` | `1.000000000` | `1.000000000` | `1` | `0.178573` | `0.000` |
| `MH_05_difficult` | `0.258855896` | `0.058137175` | `1.000000000` | `1.000000000` | `1` | `0.258856` | `0.000` |
| `V1_01_easy` | `0.112073014` | `0.062535618` | `0.997595328` | `1.000000000` | `2` | `0.097927` | `0.014146` |
| `V1_02_medium` | `0.072209956` | `0.043865555` | `0.944411937` | `0.944379390` | `1` | `0.072210` | `0.000` |
| `V1_03_difficult` | `0.861562954` | `0.130756976` | `0.974406701` | `0.974860335` | `2` | `0.144` | `0.718` |
| `V2_01_easy` | `0.190589738` | `0.043476701` | `0.952192982` | `0.952172004` | `1` | `0.190590` | `0.000` |
| `V2_02_medium` | `0.171546155` | `0.057113027` | `0.974020443` | `0.974009375` | `1` | `0.172` | `0.000` |
| `V2_03_difficult` | `2.081321466` | `0.688425331` | `0.535137949` | `0.791345331` | `18` | `0.062` | `2.020` |

11 条均为 `failed=0`、`rejected=0`、`reanchors=0`。Q0 的 segments 合计 `30`，
相对 M4 checkpoint 的 `93` 减少 `63`；`V1_03` 与 `V2_03` 的全局 ATE 仍主要由
段间关系贡献。`V2_03` 的段内加权 RMS 只有 `0.061664 m`，而绝对段间分量为
`2.019656 m`，说明下一片应继续以 continuity/liveness 为主驱动，而不是放宽
局部精度。

### 2.1 相对 M4 checkpoint

| sequence | ATE ratio | ATE 变化 | RPE ratio | RPE 变化 | segments `c999f58 → Q0` |
|---|---:|---:|---:|---:|---:|
| `MH_01_easy` | `1.0286` | `+2.86%` | `0.9839` | `-1.61%` | `1 → 1` |
| `MH_02_easy` | `0.3189` | `-68.11%` | `0.3609` | `-63.91%` | `6 → 1` |
| `MH_03_medium` | `1.0000` | `+0.00%` | `1.0000` | `-0.00%` | `1 → 1` |
| `MH_04_difficult` | `0.8375` | `-16.25%` | `0.9409` | `-5.91%` | `1 → 1` |
| `MH_05_difficult` | `1.1241` | `+12.41%` | `1.0687` | `+6.87%` | `1 → 1` |
| `V1_01_easy` | `1.0000` | `+0.00%` | `0.9994` | `-0.06%` | `2 → 2` |
| `V1_02_medium` | `0.9131` | `-8.69%` | `0.9365` | `-6.35%` | `3 → 1` |
| `V1_03_difficult` | `0.6123` | `-38.77%` | `0.2450` | `-75.50%` | `16 → 2` |
| `V2_01_easy` | `1.0314` | `+3.14%` | `1.0030` | `+0.30%` | `1 → 1` |
| `V2_02_medium` | `0.0866` | `-91.34%` | `0.1204` | `-87.96%` | `9 → 1` |
| `V2_03_difficult` | `1.2189` | `+21.89%` | `0.8201` | `-17.99%` | `52 → 18` |

聚合结果为：

| suite | `G_ATE` | `G_RPE` | ATE 算术均值 `checkpoint → Q0` | RPE 算术均值 `checkpoint → Q0` | segments |
|---|---:|---:|---:|---:|---:|
| core-4 | `0.507725` | `0.392821` | `1.291756 → 0.796747 m` | `0.470125 → 0.227234 m` | `78 → 22` |
| EuRoC-11 | `0.696757` | `0.645531` | `0.579270 → 0.380628 m` | `0.205815 → 0.113120 m` | `93 → 30` |

EuRoC-11 平均 completion 从 `0.939048` 升至 `0.943409`，平均 coverage 从
`0.966758` 升至 `0.966954`。ATE 为 5 条改善、1 条数值等价、5 条上升；RPE 为
9 条改善、2 条上升。因此 Q0 相对 milestone checkpoint 的整体变化明确为改善，
同时保留 `MH_05`、`V2_03` 等单序列 tail。后续 candidate gate 使用同机 matched
clean Q0 作为 immediate control，不使用 checkpoint 直接判定 candidate。

### 2.2 已冻结的产品 envelope

下列数值由完整 Q0、M4 checkpoint 的多序列变化和绝对工程量级共同给出，已于
2026-08-28 确认并写入 spec：

| 层 | hard envelope |
|---|---|
| core-4 accuracy | `G_ATE <= 1.05` 且 `G_RPE <= 1.05` |
| EuRoC-11 accuracy | `G_ATE <= 1.05` 且 `G_RPE <= 1.05` |
| suite availability | 11 条平均 completion `>= 0.933409`，平均 coverage `>= 0.956954`，即各允许相对 Q0 回落不超过 `0.010` absolute |
| hard validity | 11/11 均有有限 ATE/RPE 与完整 artifact；identity/config/schema 成立；`failed == rejected == reanchors == 0`；spec §13.4 机制门全部成立 |
| `V2_03` continuity | `segments <= 17`；completion `>= 0.555138`（比 Q0 至少增加 `0.020`）；coverage `>= 0.791345`；段内 RMS `<= 0.070 m`；绝对段间分量 `<= 2.020 m` |

逐序列只进入 `REVIEW`，不自动 `FAIL`，触发条件建议为：

- ATE 增量 `> max(0.010 m, 10% * ATE_Q0)`；
- RPE 增量 `> max(0.005 m, 10% * RPE_Q0)`；
- completion 或 coverage 相对 Q0 下降 `> 0.010` absolute；
- segments 高于 Q0。

这组规则允许高精度 easy sequence 的毫米级变化。例如 `MH_01` 相对 checkpoint
增加 `0.002020 m`，不会触发 ATE tail；只有多序列 `G_ATE/G_RPE` 越过 `1.05`
才形成 suite-level accuracy `FAIL`。`V2_03` 则必须同时取得至少一个 segment 和
2 个百分点 completion 的实质改善，并保持其已经很低的段内误差。

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

## 4. EuRoC-11 完整性

| 检查 | 结果 |
|---|---|
| required artifacts | 11/11、55/55 均有非空 `meta.json`、`summary.json`、`diag.csv`、`est.tum`、`kf.tum` |
| code identity | 11/11 为 `42f99e9117d2c04fdb4ebdfc308d9ca786312e60`，`git_dirty=false` |
| config / input identity | 11/11 为 `default_0337287b`；canonical text 逐字一致；sequence root 与预注册 dataset 一致 |
| diag schema / grain | 11/11 为相同 24 列；每个 image frame 一行，row count 与 `trajectory.image_frames` 一致 |
| timestamps / linkage | `diag.csv`、`est.tum`、`kf.tum` 各自严格递增；est timestamp 属于 diag，kf timestamp 属于 est |
| numeric validity | 所有 numeric cells 有限且非负；每行 `num_shared <= num_disparity <= num_obs`，retained/seeded/PnP population 不越界 |
| lifecycle linkage | `poses_written == status=ok`；`segments` 等于有 `ok` pose 的 active segment 数；`kf.tum` 行数等于 `status=ok && is_keyframe=1` |
| status / hard counts | status 仅为 `initializing`、`ok`、`visual_outage`；11/11 的 `failed/rejected/reanchors` 均为 0 |
| manifest | 以下 55 行按表序连接后 SHA-256 为 `485c942e13d3dad38148dd3ce8f328ccca4d9549898ad3228600820a87655c2e` |

`segment_id` 可为尚未产出 pose 的 terminal initializing epoch 保留诊断编号，因此
`trajectory.segments` 按 active (`status=ok`) segment 计数；该口径与
`segment_ate_decomp.py` 的 pose 分段一致。

| sequence / artifact | SHA-256 |
|---|---|
| `MH_01_easy/meta.json` | `edd1c341084ba88e2b21e329d9af57774999112ad406488f7466626259ca6b70` |
| `MH_01_easy/summary.json` | `1f53fe158805dfc482a1d5d6f60dfb4b24167b5a09b4163c6efd355801c58a2d` |
| `MH_01_easy/diag.csv` | `3f954e83cd321a141923c4b75926a67178d98a324bf8a9c9bd8483955e131e2c` |
| `MH_01_easy/est.tum` | `bcb5a95993644e6fa5b75d030e731e0d63b9a23f540157ee67c6376212cd2f97` |
| `MH_01_easy/kf.tum` | `7ba388a1c2b87652e7bcdb89468116e8a05e6f702b702523f4e6c024f1f042e9` |
| `MH_02_easy/meta.json` | `e768e14a2175332e6d8431ecce4a7f1edc50e994f034370869ae753f4d655a08` |
| `MH_02_easy/summary.json` | `866437c731fedb2467ed2b887704ebe5fbe776621f353346376f7bf031e5ce46` |
| `MH_02_easy/diag.csv` | `72f5e81ff68bbec995a8d53af29be4957ca7295bb947d956fa4b2cf1e0554e8f` |
| `MH_02_easy/est.tum` | `ed4bae1e72b103686363dddf56838ca89a557eb106d15195275826a50e89405c` |
| `MH_02_easy/kf.tum` | `b23f865e2e6356c776c2d3f66c849ccdce991c5b75868b8c0efe890aa1328c72` |
| `MH_03_medium/meta.json` | `72c3b4d9e1b3425186f633cfff4a02855235c9aeff71f5bbd108820224c000b4` |
| `MH_03_medium/summary.json` | `8215754acad693d2befe905cd184f7315db16ff52e930d3bd4292cfca3b36c81` |
| `MH_03_medium/diag.csv` | `204e6f7997e4d7038274caeffd50a3649f4bf27ef2c6ce088c1c48b0bc325055` |
| `MH_03_medium/est.tum` | `8d93c5d49eb8d353443d5e5785ab1be0c16b63b943be7088aa29ce6d8bf332fb` |
| `MH_03_medium/kf.tum` | `44e5195d9000f0d0ffaa1fd6de1f142ae7ff48cac9c6b471c312837d73725983` |
| `MH_04_difficult/meta.json` | `df8ab12ed43668d48f0d1930c10720d5a75190603823f028094cab752b1e446e` |
| `MH_04_difficult/summary.json` | `cff75424c3b01c33fff41b796e95dc11c72ae55f561073d6f399be7ef1746ac2` |
| `MH_04_difficult/diag.csv` | `3877c32b09ed6e312216dc71a0d3c29d3352ffe7b4ad23fefc83b364d32b783e` |
| `MH_04_difficult/est.tum` | `f1783558fc69c52d1324bc159c0297db7cdb50cd7c74d98879c4e4fc7644022a` |
| `MH_04_difficult/kf.tum` | `9283457d9061c30a77e63796efce7f7235197b642307e116695ebff98c7e1a23` |
| `MH_05_difficult/meta.json` | `4129e43a9f5a65ac941aeee14f4609ac8efab1c415f1d11a3970a65f659203aa` |
| `MH_05_difficult/summary.json` | `cf4b778d627aa68f318d83bc77f2e9dff80543d16f31b6320886de32a2bd103a` |
| `MH_05_difficult/diag.csv` | `6fb0feeed1f9a769954b401a335c99df88a60a29ac6c50ac909b5bbcaff3b7ee` |
| `MH_05_difficult/est.tum` | `115029dcd5b897a80968284714f1426f440b040a4aee11ae6aeac4ca49d6177e` |
| `MH_05_difficult/kf.tum` | `433b7a2ad64b4179f9eb7d10e7bff060869435764c036ceb022a2c12f4f04c51` |
| `V1_01_easy/meta.json` | `3af1ccb124f358c0a3f6bc4dac0ff511e142e4e2a703d9fe64ebd91bcadb7330` |
| `V1_01_easy/summary.json` | `a641402d6c9669c3f49f25136880a24cf85205824b1808d5fdb700380cc1aced` |
| `V1_01_easy/diag.csv` | `3641772c72660e640ab42d96760c9e9da3a8ad3c4a9e15e3a4baa425903ce314` |
| `V1_01_easy/est.tum` | `5ef7b97c49091f2dbbba69f1c9e3d6444432353e8ef40f9d973da1f1add0667f` |
| `V1_01_easy/kf.tum` | `59393d3da57803f39b04eaa4ed0efba6eea825764bb115ed2b57df87dc2e46e7` |
| `V1_02_medium/meta.json` | `fcea3d9a065fc757bf32c5c71659b4fc4946096883e02aef9d00f37bc1afa8d9` |
| `V1_02_medium/summary.json` | `44d8677a900e2931bd36719a9457775f4cd275f4106b16faedeb73d97336fcad` |
| `V1_02_medium/diag.csv` | `d3d5bdbc382a4e623938c1547f12d9976d79cfcb1c9b836d869d974f30bd4593` |
| `V1_02_medium/est.tum` | `adc5f3ec3b134d77aba01f939ca276f0826241efc6af51eb5e0315677d7467fe` |
| `V1_02_medium/kf.tum` | `6d7fa9f71035c10962f5d64fb8383fbe2b6cec3ea80dab4a7a39d05b2782f6e6` |
| `V1_03_difficult/meta.json` | `58eda0cf646285e5639a6bc4bf7178114b7517621e307a675b65f9f7f50636be` |
| `V1_03_difficult/summary.json` | `14c0184d5ec4e649144d460d389bd27dda9307fff7e87103484653981023da3f` |
| `V1_03_difficult/diag.csv` | `57357a0e3457dcf5d1f92b6335507fb3fb6ea7fd12a956effa0f6c431cf1bf8d` |
| `V1_03_difficult/est.tum` | `46d99feb8f15aa43b9ca64744214072113415ac6b10407208a9f650d37e52b01` |
| `V1_03_difficult/kf.tum` | `72e0fd8450b9084380a86735529f8735e93587d48c7aa05a1f2b7419fb423537` |
| `V2_01_easy/meta.json` | `95a1d6e1bf02b0a8cfa946862bd98cb0b860a26d438f2f329f7b528d2c0c792a` |
| `V2_01_easy/summary.json` | `e9840883830e48782d5a686e4691a17d91a4e56accc9827208e05a88007d2537` |
| `V2_01_easy/diag.csv` | `b785a105d2cc4542ce69135904209134c830697f89f71cf6f4759a7a6a4345dd` |
| `V2_01_easy/est.tum` | `4291b855e4a2cdf82dc8060c251d96f42fb1b5eff73e5e454eb13af846106123` |
| `V2_01_easy/kf.tum` | `05490cbdd8227b83ef041dfa58cd97bfe19b78a110cfa0e55f14079a310b594e` |
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

MH_01 的五个 hash 与首次 Q0 记录逐字一致，证明分阶段续跑保持 artifact
确定性。首分叉与因果归属见
[Q0 control diagnosis](../../research/2026-08-27-note-m4-mapped-bearing-q0-control.md)，
多序列判定依据见
[product gate design](../../research/2026-08-27-note-m4-multisequence-product-gate.md)。

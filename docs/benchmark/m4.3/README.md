# M4.3 静止初始化 + 端到端三重门 Benchmark（`d70e45aa`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-09

状态：**complete（跑数与判定已完成）**；门① FAIL、门②量化 FAIL / dropout
断言 PASS、门③ PASS。主套件在 `b9da3bf_dirty` 上运行，因此这是可复现的失败
checkpoint，**不是 clean 全量 baseline**。

相关：

- issue：[#33](https://github.com/Nothand0212/phad-vio/issues/33)
- predecessor：[M3.3 Slice ⑦](../m3.3/slice-7_e77ee5d_402d1925.md)
- design：[M4 IMU 接入设计](../../research/m4-imu-integration-design.md)
- plan：[M4.3 静止初始化 + 端到端三重门](../../plans/2026-08-08_m4.3_static_init_8f2c91ab.plan.md)

## 1. 身份与执行

| 项 | 值 |
|---|---|
| run code | `b9da3bfa960f2a427e98afdf53c6e5264148f7ee`（short `b9da3bf`；`git_dirty=true`） |
| final code | `603cd9e3ab60899a4598a152a64b025f172f63de`（short `603cd9e`） |
| config | IMU-on `d70e45aa`；IMU-off `a5a403b0` |
| predecessor | `e77ee5d` / `default_402d1925`（M3.3 Slice ⑦，IMU-off） |
| dataset | EuRoC ASL native；本片 8 条选定序列 |
| sequence scope | `MH_01_easy MH_02_easy MH_03_medium MH_05_difficult V1_01_easy V1_02_medium V2_01_easy V2_02_medium` |
| bench root | `/home/lin/Projects/data/phad-bench` |
| artifact path | `<bench-root>/<sequence>/b9da3bf_dirty/<label>_<hash>/` |
| execution | 2026-08-09；`phad_vo_bench` Release；各 run 独立完成 |
| gate | MH_01 绝对门；MH_05 量化 + dropout；全序列机制证据；其余 record-only |

run 时的 dirty diff 是待提交的最终 M4.3d 算法状态；随后只把实验注释正式化并
修正注释中的槽位/“收紧”措辞，算法值与控制流未再改变，最终原样纳入
`603cd9e`。这个对应关系不能把 raw 产物改称 clean：所有 `meta.json` 仍如实保留
`b9da3bf_dirty` 身份与 warning。

主命令模板：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/<sequence> \
  --bench-root /home/lin/Projects/data/phad-bench \
  --config-label <label> \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio \
  --force
```

标签：MH_01 最终复现为 `m43d_final_verify`，MH_05 为
`m43d_gate2_mh05_imuon`，其余为 `m43d_recordonly`。IMU-off 对照追加
`--no-imu`；dropout 命令追加：

```text
--config-label m43d_gate2_mh05_dropout_full
--dropout-start-frame 1400 --dropout-frames 60 --dropout-keep-ratio 0.0
```

dropout 参数是 CLI-only，不进入 `config_hash`。实际 `diag.csv` 边界再次核验为
半开区间 `[1400, 1460)`，共 60 帧；早期交接摘要中的“61 帧”是计数笔误。

## 2. 配置快照

### 2.1 相对 run base `b9da3bf` 的配置增量

| 键 | `b9da3bf` HEAD | M4.3d | 说明 |
|---|---:|---:|---|
| `estimator.imu_acc_noise_nd` | `2.0e-3` | `6.0e-2` | 放松受错误 acc-bias 污染的 Δp/Δv 预积分约束 |
| `estimator.imu_prior_pose_sigma` | `1.0e-2` | `1.0e-4` | 收紧重验无量化收益；保持门① run 身份 |
| `estimator.imu_prior_bias_acc_sigma` | `1.0e-1` | `3.0e-2` | 抑制链内 acc bias walk |

同时纳入三个实现修复：

1. 本地 GTSAM fork 的 `ConstantBias::vector()` 是 `[acc; gyro]`，graph bias
   prior 与 bias random-walk sigma 均改为这个槽位顺序；
2. 静止初始化的 acc bias 从恒等式
   `a_mean - |a_mean| * gravity_dir == 0` 改为
   `a_mean - imu_gravity * gravity_dir`；
3. 最老帧 B prior 目标无条件取 `window.front().bias`，不再从第二张图开始
   回到 Zero 目标。

### 2.2 完整 `config_canonical_text`

以下文本直接取自 `m43d_final_verify_d70e45aa/meta.json`，与 8 条主 run 及
dropout run 同源：

```text
estimator.block_culled_rebirth=true
estimator.enable_imu=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.imu_acc_noise_nd=0.059999999999999998
estimator.imu_acc_rw=0.0030000000000000001
estimator.imu_gravity=9.8100699999999996
estimator.imu_gyr_noise_nd=0.00016967999999999999
estimator.imu_gyr_rw=1.9392999999999999e-05
estimator.imu_init_accel_std=0.20000000000000001
estimator.imu_init_gyro_std=0.01
estimator.imu_init_timeout_s=30
estimator.imu_init_window_s=0.5
estimator.imu_prior_bias_acc_sigma=0.029999999999999999
estimator.imu_prior_bias_gyro_sigma=0.10000000000000001
estimator.imu_prior_pose_sigma=0.0001
estimator.imu_prior_vel_sigma=1
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

8 条主 run 的 canonical text SHA-256 均为
`39f2687b17446b4ea04710da32ab82cb574bb99cc36cea973bd5bd9e66f5a06c`。

## 3. 主套件质量表（8/8）

`completed*` 表示算法与评估完成，但 summary 带 dirty-tree warning。

| sequence | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | ok / image | rejected | failed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | completed* | 0.122337 | 0.022957 | 0.879957 | 0.879924 | 1 | 0 | 3240 / 3682 | 442 | 0 |
| MH_02_easy | completed* | 0.105080 | 0.023546 | 0.823026 | 0.822968 | 1 | 0 | 2502 / 3040 | 538 | 0 |
| MH_03_medium | completed* | 0.161084 | 0.033012 | 0.920370 | 0.920341 | 1 | 0 | 2485 / 2700 | 215 | 0 |
| MH_05_difficult | completed* | 0.345609 | 0.067311 | 0.995161 | 0.995158 | 1 | 0 | 2262 / 2273 | 11 | 0 |
| V1_01_easy | completed* | 0.085248 | 0.047730 | 0.985234 | 0.985228 | 1 | 0 | 2869 / 2912 | 43 | 0 |
| V1_02_medium | completed* | 0.114236 | 0.073115 | 0.943860 | 0.943827 | 1 | 0 | 1614 / 1710 | 96 | 0 |
| V2_01_easy | completed* | 0.497773 | 0.061694 | 0.985965 | 0.985959 | 1 | 0 | 2248 / 2280 | 32 | 0 |
| V2_02_medium | completed* | 0.313561 | 0.072171 | 0.992760 | 0.992757 | 1 | 0 | 2331 / 2348 | 17 | 0 |

MH_01 的 `m43d_full_recipe`、`m43d_baseline_verify` 与最终
`m43d_final_verify` 三次运行，其 `est.tum` MD5 均为
`93318dc194cf1be06f580d2ef55188aa`，`diag.csv` MD5 均为
`71bdff9377e4c0073bd7127fed032581`；ATE/RPE 逐位复现。

## 4. Robustness

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 66 / 66 | 0 | 0 | 0 / 0 | 0 / 3239 | 0 | 0 |
| MH_02_easy | 38 / 38 | 0 | 0 | 0 / 0 | 0 / 2501 | 0 | 0 |
| MH_03_medium | 32 / 32 | 0 | 0 | 0 / 0 | 0 / 2484 | 0 | 0 |
| MH_05_difficult | 58 / 58 | 0 | 0 | 0 / 0 | 0 / 2261 | 0 | 3 |
| V1_01_easy | 61 / 61 | 0 | 0 | 0 / 0 | 0 / 2868 | 0 | 61 |
| V1_02_medium | 77 / 77 | 0 | 0 | 0 / 0 | 0 / 1613 | 0 | 4 |
| V2_01_easy | 90 / 90 | 0 | 0 | 0 / 0 | 0 / 2247 | 0 | 124 |
| V2_02_medium | 126 / 126 | 0 | 0 | 0 / 0 | 0 / 2325 | 0 | 123 |

IMU-on 的 pose 初值优先走预积分，故 `pnp_successes=0` 属设计行为；summary
中的 `pnp_fallbacks` 是既有计数口径，不表示这些帧实际使用了视觉 PnP 锚。

## 5. 相对 M3.3 Slice ⑦

Slice ⑦ 是当前 M4 checkpoint 的直接算法前驱，也是 IMU-off 对照锚。

| sequence | ATE Δ (m) | ATE Δ (%) | RPE Δ (m) | completion Δ | coverage Δ | 判读 |
|---|---:|---:|---:|---:|---:|---|
| MH_01 | +0.041373 | +51.1% | +0.005176 | −0.119771 | −0.120076 | 门①回归；初始化等待占主要 completion 差 |
| MH_02 | +0.015711 | +17.6% | +0.007912 | −0.173027 | −0.177032 | 回归；首个静止窗口较晚 |
| MH_03 | +0.032493 | +25.3% | −0.002621 | −0.079630 | −0.079659 | ATE 回归，RPE 略改善 |
| MH_05 | +0.021502 | +6.6% | +0.023302 | −0.004399 | −0.004842 | 门②量化 FAIL |
| V1_01 | −0.031095 | −26.7% | +0.001920 | −0.014766 | −0.014772 | ATE 改善，RPE 近似持平 |
| V1_02 | −0.407268 | −78.1% | −0.046459 | +0.006433 | +0.000585 | 显著改善 |
| V2_01 | +0.188358 | +60.9% | +0.020646 | +0.037281 | +0.034665 | 明显回归 |
| V2_02 | −0.539286 | −63.2% | −0.195644 | +0.026832 | +0.022156 | 显著改善 |

coverage 与初始化等待均发生变化，表中只陈述最终数字，不把跨 coverage 的 ATE
差异归因给单一机制。

## 6. 三重门判定

| 门 | 判定 | 证据 |
|---|---|---|
| ① MH_01 IMU-on ATE ≤ 0.100 m | **FAIL** | 0.122337 m；三次逐位复现 |
| ② MH_05 显著改善 + dropout 断言 | **量化 FAIL / 断言 PASS** | 0.345609 vs M3.3 0.324107（+6.6%）；60 帧全清仍连续 |
| ③ segments/reanchors=1/0 + bias 收敛 | **PASS** | 8/8 的 `segment_id` 集合均 `{0}`；gyro bias 为 `1e-3 rad/s` 量级；合成对拍通过 |

### 6.1 门①归因与 knob 穷尽

MH_01 的主要失败机制不是随机波动：

- 20–40 s 区间存在约 `−0.0535 m` 的刚性 z 偏移（std `0.0021 m`），与
  `0.5 * δba_z * Δt²` 一致（`δba_z≈0.0665 m/s²`、`Δt≈1.27 s`）；
- 静止窗口处并非理想水平静止：约 `0.8°` 倾斜与 accel scale
  `s≈1.0068` 污染了 `a_mean`。因此 `ba_z` 初始化为 `0.0091`，而 GT
  约为 `0.0756`，误差 `0.0667≈(s−1)g`；`ba_y` 也没有恢复 GT 的
  `+0.137`；
- 首个接受帧的 acc bias 为 `(-0.033322,+0.001849,+0.009106)`，末帧为
  `(-0.127564,+0.473551,-0.189378)`，LM 沿运行继续发生 bias walk；
- 在当前权重下，移动 pose 的代价
  `(0.053/σ_Δp)²` 低于移动 bias 的代价 `(δ/σ_prior)²`，LM 因而把错误
  bias 解释为 pose shift。

修复树上的有效 MH_01 knob 矩阵：

| 变体 | hash | ATE (m) | 结论 |
|---|---|---:|---|
| final：acc nd `6e-2`、acc prior `3e-2`、window `0.5s` | `d70e45aa` | **0.122337** | 保留 |
| acc nd `6e-3` | `c5341d09` | 0.127210 | 更紧侧变差 |
| acc nd `1e-1`（标称 ×50） | `b5fa4f8d` | 0.145658 | 更松侧变差 |
| acc nd `2e-1`（标称 ×100） | `758e8ec4` | 0.156711 | 更松侧变差 |
| acc prior 全通道 `1e-1` | `62dd15f2` | 0.144269 | bias walk 加重 |
| acc prior z=`1e-1`、x/y=`3e-2` | source-only | 0.130451 | 各向异性无甜区 |
| `biasAccOmegaInit` 值槽互换 | source-only | 0.145421 | 否决 |
| init window `2.0s` | `2524cf4a` | 0.126298 | 均值污染不变，另损 coverage |
| velocity prior `0.3` | `14bfcad5` | 0.144267 | 否决 |
| gyro nd ×30 | `c1e034b7` | 0.122338 | 无影响轴 |
| gyro rw ×100 | `1dbce642` | 0.122337 | 无影响轴 |
| acc rw `3e-4` | `14f76d4b` | 0.122339 | 无影响轴 |

标称 acc nd `2e-3` 在修复树上**没有成功实测**：`dbg_noise*` 时期的 run
全部 failed、`ate=null`。不能把那批 run 当成紧侧证据；当前只由 `6e-3`
补齐紧侧。结论是 `0.122337` 为当前设计和已扫 knob 中的强局部最优；运动基础
bias 初始化或 accel scale 联合估计属于结构性修复，超出 M4.3d。

### 6.2 门②与 dropout

| run | ATE (m) | RPE (m) | completion | segments / reanchors | 判读 |
|---|---:|---:|---:|---:|---|
| MH_05 IMU-on | 0.345609 | 0.067311 | 0.995161 | 1 / 0 | 相对 M3.3 +6.6%，量化 FAIL |
| MH_05 dropout `[1400,1460)` | 0.348466 | 0.067543 | 0.995161 | 1 / 0 | ATE 只增 0.002857；同 completion |
| MH_05 IMU-off | 0.324107 | 0.044009 | 0.999560 | 1 / 0 | 与 M3.3 Slice ⑦逐位相同 |

dropout raw 证据：60/60 帧 `num_obs=0` 且全部 `status=ok`，`segment_id`
全为 0；`est.tum` 无缺口（20 Hz），注入窗口内相邻位姿最大平移步长
`0.049115 m`。恢复后 completion 与 baseline 相同，整段没有新增 rejected。
CI 级 `DropoutInjectionKeepsChainWithImu` / `DropoutInjectionFreezesWithoutImu`
分别断言 IMU-on 连续恢复与 IMU-off 冻结，均通过。

### 6.3 门③与 bias

- 8 条主 run 的 `diag.csv.segment_id` 唯一集合均为 `{0}`，summary 全部
  `segments=1` / `reanchors=0`；
- MH_01 首个接受帧 gyro bias 为
  `(-0.003560,+0.021393,+0.079898) rad/s`，末帧为
  `(-0.002532,+0.019583,+0.078767) rad/s`；GT 约
  `(-0.0032,+0.0213,+0.0785) rad/s`，保持在 `1e-3 rad/s` 量级；
- 合成已知 bias、静止初始化、pending/gap、dropout 两臂均包含在 estimator
  `79/79` 通过的测试中；
- acc bias 的明显 walk 是门①失败机理，不能用 gyro 的收敛结论掩盖，已保留为
  M5 结构性初始化输入。

## 7. IMU-off 回归

| sequence | hash | ATE (m) | RPE (m) | completion | coverage | bias cells |
|---|---|---:|---:|---:|---:|---|
| MH_01_easy | `a5a403b0` | 0.080964 | 0.017781 | 0.999728 | 1.000000 | 3682×6 全 0 |
| MH_05_difficult | `a5a403b0` | 0.324107 | 0.044009 | 0.999560 | 1.000000 | 2273×6 全 0 |

MH_01 在跑数会话中已与 `c1d3481` 时代 IMU-off 参考做过对拍：`est.tum`
与原 `diag.csv` 列逐字节相同，新 bias 六列恒 0。当前 MH_01 产物 MD5：
`est.tum=834d8715f13715967c7afd3b724cf4bd`、
`diag.csv=607f1bca55e892b690af652d5bb9a0db`。MH_05 的 ATE/RPE/completion 与
M3.3 Slice ⑦逐位一致，说明 IMU-off 链没有被 M4.3d 改动污染。

## 8. 完整性核验与限制

| 检查 | 结果 |
|---|---|
| final verify command rc | `1/1 rc=0`；wall `197.545 s` |
| main artifacts | 8/8 `meta.json` + `summary.json` + `est.tum` + `diag.csv` |
| comparison artifacts | dropout 1/1、IMU-off 2/2 齐全 |
| config identity | 主 run + dropout 均 `d70e45aa`；IMU-off 均 `a5a403b0` |
| canonical config | 主 run 一致；SHA-256 见 §2.2 |
| code identity | raw 如实为 `b9da3bf_dirty`；算法 diff 已提交为 `603cd9e` |
| build | Release / Debug 均构建成功 |
| tests | `phad_estimator_tests` 79/79 PASS；`phad_apps_tests` 27 PASS / 3 SKIP（缺 `PHAD_EUROC_MH01_PATH` 的既有 ProbeB 用例） |
| raw artifacts | `/home/lin/Projects/data/phad-bench` |

限制与异常：

- 没有在 `603cd9e` clean commit 上重跑 8 条套件，故本文不能升级为 clean
  baseline；
- M4.3 选定套件未覆盖 `MH_04_difficult`、`V1_03_difficult`、
  `V2_03_difficult`，TUM VI record-only 也未执行；
- MH_03 首次尝试出现 completion=0 且未形成正式 output dir 的 transient；同命令
  重跑成功。现有证据不足以归因为数据损坏或算法非确定性，因此只记录，不作
  因果结论；
- 门①和门②量化未达标是本 checkpoint 的真实结果；关闭 M4.3 issue 表示本片
  实验与归因收口，不表示这些数值门已经通过。

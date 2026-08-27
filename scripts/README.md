# 离线脚本

消费 C++ 侧导出的轨迹、误差与 bench `summary.json`。脚本不参与 CMake 构建，
也不进 CI；绘图依赖装在本地 venv 里，`bench_table.py` 只依赖标准库。

## 环境

```bash
python3 -m venv .venv
.venv/bin/pip install -r scripts/requirements.txt
```

## `plot_trajectory.py`

画一条或多条 TUM 轨迹的 3D 曲线，三个轴使用同一比例。多条轨迹按原样叠加，
不做对齐。

```bash
.venv/bin/python scripts/plot_trajectory.py /tmp/mh01_gt.tum \
  --title "MH_01_easy groundtruth" --out /tmp/mh01_gt.png
```

不给 `--out` 时弹窗显示。`--label` 可重复，数量须与轨迹一致，默认用文件名。

## `plot_errors.py`

画 `phad_traj_eval --errors-csv` 的逐样本误差：平移误差、旋转误差随时间的
曲线，以及对齐后的估计与真值俯视图。位置列已经过 SE3 对齐，因此俯视图可以
直接叠加。

```bash
phad_traj_eval --est /tmp/mh01_est.tum --gt /tmp/mh01_gt.tum \
  --errors-csv /tmp/mh01_errors.csv
.venv/bin/python scripts/plot_errors.py /tmp/mh01_errors.csv \
  --out /tmp/mh01_errors.png
```

CSV 的列合同由 `apps/phad_traj_eval.cpp` 定义：

```text
timestamp_ns,dt_ns,err_trans_m,err_rot_deg,est_x,est_y,est_z,gt_x,gt_y,gt_z
```

## `plot_tracks.py`

画 `phad_stereo_frontend_probe` 的帧级与 track 生命表：track 数随时间、
track 长度直方图、epipolar error 直方图。

```bash
phad_stereo_frontend_probe /path/to/MH_01_easy \
  --frames-csv /tmp/mh01_frames.csv --tracks-csv /tmp/mh01_tracks.csv
.venv/bin/python scripts/plot_tracks.py \
  --frames-csv /tmp/mh01_frames.csv --tracks-csv /tmp/mh01_tracks.csv \
  --out /tmp/mh01_tracks.png
```

列合同见 `phad/frontend/README.md`。

## `bench_table.py`

递归扫描 `phad_vo_bench` 的 `bench_root`，把各 run 的 `summary.json` 拼成
对比表。默认 Markdown；`--csv` 输出 CSV。无第三方依赖，可在系统 python3
或 venv 下直接跑。

```bash
export PHAD_BENCH_ROOT=/home/lin/Projects/data/phad-bench
python3 scripts/bench_table.py "$PHAD_BENCH_ROOT"
python3 scripts/bench_table.py "$PHAD_BENCH_ROOT" --csv > /tmp/bench.csv
```

主列：sequence、commit、dirty、config（`label_hash8`）、status、
ATE/RPE trans RMSE、completion_rate、coverage_rate、rtf、wall_s。

## `keyframe_schedule_probe.py`

比较同一序列两个或多个 bench run 的 **accepted KF schedule**。只计
`diag.csv` 中 `status=ok && is_keyframe=1` 的行；图像 timestamp axis、sequence
或输入 schema 不兼容时明确失败。输出 exact/±N 帧的一对一匹配率、首处分叉、
accepted-KF interval、滚动窗口数量差和 candidate rule 构成。

```bash
python3 scripts/keyframe_schedule_probe.py \
  /path/to/reference_run \
  /path/to/candidate_run_a /path/to/candidate_run_b

python3 scripts/keyframe_schedule_probe.py \
  /path/to/reference_run /path/to/candidate_run --json
```

脚本只依赖标准库。定向测试：

```bash
python3 scripts/keyframe_schedule_probe_test.py
```

## `keyframe_shadow_probe.py`

分析 `phad_vo_bench --keyframe-shadow-probe <csv>` 生成的 fixed-production-epoch
sidecar。脚本严格校验逐帧 timestamp、epoch transaction 与 IMU 区间，报告
pose/IMU compensated parallax 分布、10/15/30px first-order candidate 差异、连续
hit 和 minimum-gap veto 计数。

```bash
python3 scripts/keyframe_shadow_probe.py /path/to/keyframe_shadow.csv
python3 scripts/keyframe_shadow_probe_test.py
```

报告中的 candidate/veto 只是在 production accepted epoch 固定时的 first-order
反事实；它不会模拟首次选择变化后的 estimator window 或下一 evidence baseline。

## `vio_vo_common_support.py`

在 VIO/VO 两条 TUM 轨迹的 exact timestamp intersection 上分别运行同一个
`phad_traj_eval`，避免 coverage 不同造成的比较混杂。`--assert-vio-better` 只有在
VIO ATE 与 RPE translation RMSE 都严格较低时返回 0；质量未过门返回 1，输入或
evaluator 合同错误返回 2。

```bash
python3 scripts/vio_vo_common_support.py \
  /path/to/vio_run /path/to/vo_run /path/to/euroc_sequence \
  --assert-vio-better
python3 scripts/vio_vo_common_support_test.py
```

`--start-common-index` 与 `--max-common-poses` 用于固定共同 support 的局部坏窗；
1 秒 RPE 要求至少 22 个 20 Hz poses。

## `vio_bias_probe.py`

把 `diag.csv` 中 `status=ok` 的 posterior gyro/acc bias 与 EuRoC ground truth
在 2.5 ms tolerance 下对齐，按固定 pose 桶输出 bias error、drift、KF 数和
`prior_key` 迁移。ground-truth span 外 rows 显式计为 `dropped_outside_gt`；span 内
无法匹配、schema/时间/finite 合同错误返回 2。

```bash
python3 scripts/vio_bias_probe.py \
  /path/to/vio_run /path/to/euroc_sequence \
  --bucket-frames 100 --output /path/to/vio_bias.csv
python3 scripts/vio_bias_probe_test.py
```

该 probe 只报告相关性，不把时间相关的 bias error 直接解释为因果。

## `vio_state_probe.py`

分析 `phad_vo_bench --vio-state-probe <csv>` 的 accepted IMU-on state sidecar，在
完全相同 timestamp 上比较 gyro-active 的 pre-LM one-step rotation prediction 与
最终 posterior pose。脚本
复用 `phad_traj_eval` 的 EuRoC 关联、SE(3) 对齐和 1 秒 RPE 口径，并输出固定
pose 桶与 pose/gyro-bias correction；每个桶独立对齐，只用于局部 shape/drift
归因，不回答 accelerometer、velocity 或绝对 gravity tilt。

```bash
python3 scripts/vio_state_probe.py \
  /path/to/vio_state.csv /path/to/MH_01_easy \
  --traj-eval build/phad_traj_eval \
  --bucket-poses 100 --output /path/to/vio_state_buckets.csv
python3 scripts/vio_state_probe_test.py
```

CSV required-field subset、duplicate column、finite/unit quaternion、timestamp/state
key 单调性和 IMU interval 都是 strict contract；允许在 required subset 后追加只读
diagnostic columns。accepted rows 超出 ground-truth span 时原始 sidecar 保留，脚本以
`2.5 ms` tolerance 过滤 evaluator support，并分别报告 accepted/evaluable/outside；
不足 22 个 poses 的尾桶只报告 correction，不伪造轨迹 metric。

## `gyro_graph_objective_probe.py`

分析新版 `vio_state.csv` 中 `gyro_visual` 图的 objective 分解，比较 anchored-oldest
boundary AHRS、interior 单因子均值与 newest AHRS 的 posterior cost、raw rotation
residual 和 whitened norm。脚本不读取 GT，也不替 estimator 选择 robust threshold；
duplicate/missing/nonfinite 列、factor count 与 cost identity 不一致均返回 2。

```bash
.venv/bin/python scripts/gyro_graph_objective_probe.py \
  /path/to/vio_state.csv --output /path/to/gyro_graph_objective.json
.venv/bin/python scripts/gyro_graph_objective_probe_test.py
```

## `gyro_fixed_lag_shadow_probe.py`

严格校验 `phad_vo_bench --fixed-lag-shadow-probe <csv>` 生成的 30 列
read-only fixed-lag sidecar，并输出 canonical JSON：active/inactive support、
bootstrap/segment reset、bounded pose/landmark/factor 数、shared-bias fixed 状态、
factor-slot/timestamp orphan 门，以及 newest shadow-vs-batch pose/bias delta 的
p50/p95/p99/max 与固定时间桶。

```bash
.venv/bin/python scripts/gyro_fixed_lag_shadow_probe.py \
  /path/to/fixed_lag_shadow.csv --bucket-s 10 \
  --output /path/to/fixed_lag_shadow.json
.venv/bin/python scripts/gyro_fixed_lag_shadow_probe_test.py
```

该 shadow 的后续 graph events 仍来自 production rolling batch；报告只证明
lifecycle/数值结构，不计算 shadow ATE，也不构成 production handoff 反事实。

## `imu_translation_alignment_probe.py`

在不改 production graph 的情况下，用 IMU-off metric stereo poses、gyro-enabled
run 的 aligned `bg`/activation 与 EuRoC raw IMU 做 staged velocity/gravity/shared
accelerometer-bias 对齐。`zero/static/estimated` 不消费 GT runtime 输入；
`reference_constant/reference` 仅作离线 oracle。输出 condition、训练 residual、
gravity/velocity/bias truth audit，以及 1 s held-out inertial/CV/visual prediction。

```bash
.venv/bin/python scripts/imu_translation_alignment_probe.py \
  /path/to/imuoff-run /path/to/gyro-run /path/to/MH_01_easy \
  --train-duration-s 10 --eval-duration-s 1 --train-stride 10 \
  --acc-bias-mode estimated --output /tmp/imu_alignment.json
.venv/bin/python scripts/imu_translation_alignment_probe_test.py
```

脚本当前显式要求 EuRoC IMU `T_BS=Identity`；不满足时失败，不静默忽略外参。GT
只用于输出 audit 字段，不参与 `condition` 或 held-out visual disagreement gate。

## `vio_window_history_probe.py`

对已经存在的 full-VIO run 做 read-only rolling PIM 反事实。脚本在起点 body frame
分别替换 candidate velocity、gravity、bias，并与全 GT 的 measurement-model floor
比较；同时报告 frozen-prefix/suffix positional phase alignment。默认固定输出
0.05/0.5/1.0 s 三个 horizon 和 10 s buckets。

```bash
.venv/bin/python scripts/vio_window_history_probe.py \
  /path/to/full-vio-run /path/to/paired-imuoff-run /path/to/MH_01_easy \
  --output /path/to/window_history.json
.venv/bin/python scripts/vio_window_history_probe_test.py
```

GT 只进入离线 counterfactual，不生成 estimator input。phase 的 position-fit rotation
不能用作 gravity truth；gravity 始终比较 candidate/reference 的 body-frame direction。

## `vio_velocity_history_probe.py`

在 full-VIO accepted state 上固定 candidate pose、gravity 与 bias，用 raw IMU 构造
线性的 position/velocity 方程，并比较四个 no-write velocity shadow：单区间重置、
全历史因果 Schur prior、无 incoming prior 的固定窗口，以及使用未来数据的 full-batch
上界。块三对角 normal system 以 O(N) 求解，不依赖 SciPy，也不引入估计器参数。

```bash
.venv/bin/python scripts/vio_velocity_history_probe.py \
  /path/to/full-vio-run /path/to/MH_01_easy \
  --output /path/to/velocity_history.json
.venv/bin/python scripts/vio_velocity_history_probe_test.py
```

GT 只用于 body-frame velocity error 的离线评分。`causal_marginal` 与
`fixed_window` 不读取未来；`full_batch` 在 JSON 中明确标为 future-leaking，不能作为
可部署结果或生产验收。

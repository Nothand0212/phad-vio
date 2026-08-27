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

## `vio_vo_common_support.py`

在 off/fused 两条 TUM trajectory 的**精确 timestamp 交集**上做一次共同 GT
关联，再用完全相同的 pose / RPE pair 支持集计算 fixed-scale SE3 ATE、1 s RPE、
p50/p95/max、相邻 pose jump、KF schedule 首次分叉点与 gyro factor coverage。
同时从两个 `summary.json` 审计失败数、pose 数、completion 与 coverage；只有两项
translation RMSE 都严格改善且其余门不退化时，`gate.core_pass` 才为 true。

```bash
.venv/bin/python scripts/vio_vo_common_support.py \
  /path/to/off_run /path/to/fused_run /path/to/MH_01_easy \
  --output /path/to/fused_run/exact_common_report.json
```

默认 GT 关联门为 `2.5 ms`，RPE delta 为 `1.0 s`，RPE 终点容差为
`25 ms`；可分别用 `--max-dt-ms`、`--rpe-delta-s`、
`--rpe-tolerance-ms` 覆盖。脚本依赖 numpy，按本页环境说明从 venv 运行。

合成合同测试：

```bash
.venv/bin/python -m unittest tests/scripts/vio_vo_common_support_test.py -v
```

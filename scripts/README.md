# 离线绘图脚本

消费 C++ 侧导出的轨迹与误差文件，产出静态图。脚本不参与 CMake 构建，也不进
CI；依赖装在本地 venv 里。

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

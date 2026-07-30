#!/usr/bin/env python3
"""绘制 `phad_traj_eval --errors-csv` 输出的逐样本误差。

CSV 列合同：
`timestamp_ns,dt_ns,err_trans_m,err_rot_deg,est_x,est_y,est_z,gt_x,gt_y,gt_z`。
位置列是对齐后的估计与匹配上的真值，因此俯视图可以直接叠加两条轨迹。
只看一个 RMSE 数字会掩盖误差在时间上的分布，这里同时画出误差曲线与轨迹。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REQUIRED_COLUMNS = (
    "timestamp_ns",
    "err_trans_m",
    "err_rot_deg",
    "est_x",
    "est_y",
    "gt_x",
    "gt_y",
)


def read_errors(path: Path) -> np.ndarray:
    """读取误差 CSV，返回带列名的结构化数组。"""
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None)
    data = np.atleast_1d(data)
    if data.size == 0:
        raise ValueError(f"{path}: contains no samples")
    missing = [name for name in REQUIRED_COLUMNS if name not in data.dtype.names]
    if missing:
        raise ValueError(f"{path}: missing columns {', '.join(missing)}")
    return data


def summarize(values: np.ndarray, unit: str) -> str:
    rmse = float(np.sqrt(np.mean(np.square(values))))
    return f"rmse {rmse:.4g} {unit}  mean {values.mean():.4g}  max {values.max():.4g}"


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("errors_csv", type=Path, help="phad_traj_eval 的 --errors-csv 输出")
    parser.add_argument("--out", type=Path, help="保存为图片而不是弹窗显示")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    arguments = parse_arguments(argv)
    try:
        data = read_errors(arguments.errors_csv)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    # 纳秒时间戳先减去起点再转秒，避免 1.4e18 量级直接落到浮点秒上。
    elapsed_s = (data["timestamp_ns"] - data["timestamp_ns"][0]) * 1e-9
    trans_m = data["err_trans_m"]
    rot_deg = data["err_rot_deg"]

    figure, (translation_axes, rotation_axes, top_down_axes) = plt.subplots(
        3, 1, figsize=(10, 11)
    )

    translation_axes.plot(elapsed_s, trans_m, linewidth=0.9)
    translation_axes.set_ylabel("translation error [m]")
    translation_axes.set_title(summarize(trans_m, "m"))
    translation_axes.grid(alpha=0.3)

    rotation_axes.plot(elapsed_s, rot_deg, linewidth=0.9, color="tab:orange")
    rotation_axes.set_xlabel("time [s]")
    rotation_axes.set_ylabel("rotation error [deg]")
    rotation_axes.set_title(summarize(rot_deg, "deg"))
    rotation_axes.grid(alpha=0.3)

    top_down_axes.plot(data["gt_x"], data["gt_y"], linewidth=0.9, label="groundtruth")
    top_down_axes.plot(
        data["est_x"], data["est_y"], linewidth=0.9, label="estimate (aligned)"
    )
    top_down_axes.set_xlabel("x [m]")
    top_down_axes.set_ylabel("y [m]")
    top_down_axes.set_aspect("equal", adjustable="datalim")
    top_down_axes.legend()
    top_down_axes.grid(alpha=0.3)

    figure.suptitle(arguments.errors_csv.name)
    figure.tight_layout()

    if arguments.out is not None:
        figure.savefig(arguments.out, dpi=150, bbox_inches="tight")
        print(f"wrote {arguments.out}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

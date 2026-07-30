#!/usr/bin/env python3
"""绘制一条或多条 TUM 轨迹的 3D 曲线。

输入是 `phad_euroc_gt_export` 或估计侧导出的 TUM 文件：
`timestamp tx ty tz qx qy qz qw`，空格分隔、无 header。多条轨迹画在同一
坐标系中，不做对齐——对齐后的比较用 `plot_errors.py` 消费
`phad_traj_eval --errors-csv` 的输出。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

TUM_COLUMNS = 8


def read_tum(path: Path) -> np.ndarray:
    """读取 TUM 文件，返回 (N, 8) 数组。"""
    data = np.loadtxt(path, comments="#", ndmin=2)
    if data.shape[1] != TUM_COLUMNS:
        raise ValueError(
            f"{path}: expected {TUM_COLUMNS} columns per line, got {data.shape[1]}"
        )
    if data.shape[0] == 0:
        raise ValueError(f"{path}: contains no poses")
    return data


def set_equal_aspect(axes, positions: np.ndarray) -> None:
    """用等边包围盒统一三个轴的比例，避免轨迹形状被拉伸。"""
    center = 0.5 * (positions.max(axis=0) + positions.min(axis=0))
    radius = 0.5 * float(np.max(positions.max(axis=0) - positions.min(axis=0)))
    radius = max(radius, 1e-3)
    axes.set_xlim(center[0] - radius, center[0] + radius)
    axes.set_ylim(center[1] - radius, center[1] + radius)
    axes.set_zlim(center[2] - radius, center[2] + radius)


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tum", nargs="+", type=Path, help="TUM 轨迹文件")
    parser.add_argument(
        "--label",
        action="append",
        default=[],
        help="图例名称，可重复；默认使用文件名。给出时数量须与轨迹一致",
    )
    parser.add_argument("--out", type=Path, help="保存为图片而不是弹窗显示")
    parser.add_argument("--title", default="trajectory", help="图标题")
    arguments = parser.parse_args(argv)
    if arguments.label and len(arguments.label) != len(arguments.tum):
        parser.error(
            f"got {len(arguments.label)} labels for {len(arguments.tum)} trajectories"
        )
    return arguments


def main(argv: list[str]) -> int:
    arguments = parse_arguments(argv)
    labels = arguments.label or [path.stem for path in arguments.tum]

    figure = plt.figure(figsize=(8, 7))
    axes = figure.add_subplot(projection="3d")
    all_positions = []
    for path, label in zip(arguments.tum, labels):
        try:
            data = read_tum(path)
        except (OSError, ValueError) as error:
            print(f"error: {error}", file=sys.stderr)
            return 1
        positions = data[:, 1:4]
        all_positions.append(positions)
        axes.plot(positions[:, 0], positions[:, 1], positions[:, 2], label=label)
        axes.scatter(*positions[0], marker="o", s=25)

    set_equal_aspect(axes, np.vstack(all_positions))
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_zlabel("z [m]")
    axes.set_title(arguments.title)
    axes.legend()

    if arguments.out is not None:
        figure.savefig(arguments.out, dpi=150, bbox_inches="tight")
        print(f"wrote {arguments.out}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

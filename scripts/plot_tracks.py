#!/usr/bin/env python3
"""绘制 `phad_stereo_frontend_probe` 输出的 track 指标。

`--frames-csv` 列合同：
`timestamp_ns,tracked,detected,valid,no_right_match,invalid_disparity,
depth_out_of_range,fb_rejected,epipolar_median_px,epipolar_p95_px`

可选 `--tracks-csv` 列合同：
`id,first_timestamp_ns,last_timestamp_ns,length`
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

FRAMES_COLUMNS = (
    "timestamp_ns",
    "tracked",
    "detected",
    "valid",
    "no_right_match",
    "invalid_disparity",
    "depth_out_of_range",
    "fb_rejected",
    "epipolar_median_px",
    "epipolar_p95_px",
)

TRACKS_COLUMNS = ("id", "first_timestamp_ns", "last_timestamp_ns", "length")


def read_csv(path: Path, required: tuple[str, ...]) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None)
    data = np.atleast_1d(data)
    if data.size == 0:
        raise ValueError(f"{path}: contains no samples")
    missing = [name for name in required if name not in data.dtype.names]
    if missing:
        raise ValueError(f"{path}: missing columns {', '.join(missing)}")
    return data


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames-csv",
        type=Path,
        required=True,
        help="phad_stereo_frontend_probe --frames-csv 输出",
    )
    parser.add_argument(
        "--tracks-csv",
        type=Path,
        help="可选，phad_stereo_frontend_probe --tracks-csv 输出",
    )
    parser.add_argument("--out", type=Path, help="保存为图片而不是弹窗显示")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    arguments = parse_arguments(argv)
    try:
        frames = read_csv(arguments.frames_csv, FRAMES_COLUMNS)
        tracks = None
        if arguments.tracks_csv is not None:
            tracks = read_csv(arguments.tracks_csv, TRACKS_COLUMNS)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    elapsed_s = (frames["timestamp_ns"] - frames["timestamp_ns"][0]) * 1e-9
    track_count = (
        frames["valid"].astype(np.int64)
        + frames["no_right_match"].astype(np.int64)
        + frames["invalid_disparity"].astype(np.int64)
        + frames["depth_out_of_range"].astype(np.int64)
    )

    figure, axes = plt.subplots(3, 1, figsize=(10, 11))
    count_axes, length_axes, epi_axes = axes

    count_axes.plot(elapsed_s, track_count, linewidth=0.9, label="total")
    count_axes.plot(elapsed_s, frames["valid"], linewidth=0.8, label="valid")
    count_axes.set_ylabel("tracks")
    count_axes.set_title(
        f"tracks over time  min {track_count.min()}  mean {track_count.mean():.1f}"
    )
    count_axes.grid(alpha=0.3)
    count_axes.legend(loc="best")

    if tracks is not None:
        lengths = tracks["length"].astype(np.float64)
        length_axes.hist(lengths, bins=min(50, max(10, int(lengths.max()))), color="tab:blue")
        length_axes.set_xlabel("track length [frames]")
        length_axes.set_ylabel("count")
        length_axes.set_title(
            f"track length  median {np.median(lengths):.0f}  max {lengths.max():.0f}"
        )
    else:
        length_axes.text(
            0.5,
            0.5,
            "pass --tracks-csv for length histogram",
            ha="center",
            va="center",
            transform=length_axes.transAxes,
        )
        length_axes.set_axis_off()
    length_axes.grid(alpha=0.3)

    epi_axes.hist(
        frames["epipolar_median_px"],
        bins=40,
        color="tab:orange",
        alpha=0.85,
        label="frame median",
    )
    epi_axes.hist(
        frames["epipolar_p95_px"],
        bins=40,
        color="tab:red",
        alpha=0.45,
        label="frame p95",
    )
    epi_axes.set_xlabel("epipolar error [px]")
    epi_axes.set_ylabel("frames")
    epi_axes.set_title(
        "epipolar error distribution  "
        f"median-of-medians {np.median(frames['epipolar_median_px']):.3g} px"
    )
    epi_axes.grid(alpha=0.3)
    epi_axes.legend(loc="best")

    figure.tight_layout()
    if arguments.out is not None:
        figure.savefig(arguments.out, dpi=150)
        print(f"wrote {arguments.out}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

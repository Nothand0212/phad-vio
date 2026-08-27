#!/usr/bin/env python3
"""Analyze the M4.4 P2b fixed-lag candidate artifacts of a bench run.

Input is a phad_vo_bench run output directory written with ``--candidate``.
The analyzer verifies the P2b structural ownership contract:

1. ``candidate/meta.json`` exists and records ``ownership=full_pipeline``,
   ``terminal=0`` and ``failed=0``;
2. ``candidate/diag.csv`` matches the strict schema, has one row per frame
   fed to the candidate, and its timestamps are strictly increasing;
3. every candidate diag row maps 1:1 onto the production ``diag.csv`` row at
   the same index by timestamp (frame alignment only — status / keyframe /
   segment / bias are candidate-owned and allowed to differ from production);
4. graph diagnostics on accepted rows are bounded: active pose count never
   exceeds the estimator window size + 2 (strict-< cutoff), and
   marginalized / retired counts are incremental (non-negative deltas);
5. ``candidate/est.tum`` row count equals the number of candidate diag rows
   with status ``ok``, and every line is finite.

Any violation raises ``ProbeError`` and exits non-zero. Only the Python
standard library is required.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, TextIO


HEADER = (
    "frame_index",
    "ts_ns",
    "status",
    "message",
    "selected_keyframe",
    "keyframe_rule",
    "epoch_committed",
    "keyframe_epoch",
    "segment_id",
    "reset_reason",
    "track_count",
    "observation_count",
    "disparity_count",
    "shared_count",
    "landmark_count",
    "culled_count",
    "dropped_track_count",
    "fusion_mode",
    "state_valid",
    "state_tx",
    "state_ty",
    "state_tz",
    "state_qw",
    "state_qx",
    "state_qy",
    "state_qz",
    "bias_gyro_x",
    "bias_gyro_y",
    "bias_gyro_z",
    "graph_valid",
    "graph_active_pose_count",
    "graph_active_factor_count",
    "graph_marginalized_pose_count",
    "graph_retired_landmark_count",
)

# production diag.csv 的逐行映射列（writeDiagCsv 合同）。
PROD_DIAG_HEADER = (
    "timestamp_ns",
    "status",
    "num_obs",
    "num_landmarks",
    "num_shared",
    "low_connectivity",
    "window_size",
    "prior_key",
    "reproj_rms_before_px",
    "reproj_rms_after_px",
    "num_cheirality",
    "lm_iterations",
    "max_window_pose_shift_m",
    "segment_id",
    "pnp_success",
    "pnp_inliers",
    "outliers_culled",
    "reproj_rms_after_cull_px",
    "is_keyframe",
    "num_disparity",
    "bias_gyro_x",
    "bias_gyro_y",
    "bias_gyro_z",
    "bias_acc_x",
    "bias_acc_y",
    "bias_acc_z",
)

# 非整数值字符串（analyzer 自己的 CSV 读取不依赖 schema 外的列）。
_STATUSES = {"ok", "rejected", "failed"}


class ProbeError(Exception):
    """Raised on the first ownership/schema violation."""


@dataclass
class Analysis:
    run_dir: Path
    frames: int = 0
    ok: int = 0
    rejected: int = 0
    failed: int = 0
    keyframes: int = 0
    candidate_wall_s: float = 0.0
    est_pose_count: int = 0
    est_finite: bool = True
    graph_rows: int = 0
    max_active_poses: int = 0
    window_size: int = 0
    checks: list[str] = field(default_factory=list)


def load_rows(path: Path, header: tuple[str, ...]) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or tuple(reader.fieldnames) != header:
            raise ProbeError(
                f"{path.name} schema mismatch: expected {len(header)} columns "
                f"in fixed order, got {reader.fieldnames}"
            )
        return [dict(row) for row in reader]


def check_meta(candidate_dir: Path, analysis: Analysis) -> None:
    meta_path = candidate_dir / "meta.json"
    if not meta_path.exists():
        raise ProbeError(f"missing {meta_path.name} in {candidate_dir}")
    try:
        meta = json.loads(meta_path.read_text())
    except json.JSONDecodeError as error:
        raise ProbeError(f"{meta_path} is not valid JSON: {error}") from error

    if meta.get("ownership") != "full_pipeline":
        raise ProbeError(
            f"{meta_path}: ownership must be full_pipeline, got "
            f"{meta.get('ownership')!r}"
        )
    if meta.get("terminal") != 0:
        raise ProbeError(
            f"{meta_path}: terminal must be 0 for a healthy P2b run, got "
            f"{meta.get('terminal')!r} ({meta.get('terminal_detail', '')})"
        )
    if meta.get("failed", 0) != 0:
        raise ProbeError(
            f"{meta_path}: failed must be 0 for a healthy P2b run, got "
            f"{meta.get('failed')!r}"
        )
    analysis.frames           = int(meta["frames"])
    analysis.ok               = int(meta["ok"])
    analysis.rejected         = int(meta["rejected"])
    analysis.failed           = int(meta["failed"])
    analysis.keyframes        = int(meta["keyframes"])
    analysis.candidate_wall_s = float(meta.get("wall_s", 0.0))
    analysis.checks.append("meta: ownership=full_pipeline terminal=0 failed=0")


def load_config_window(run_dir: Path) -> int:
    """estimator.window_size 记录在主目录 meta.json（配置快照），
    candidate/meta.json 不含 config。"""
    meta_path = run_dir / "meta.json"
    if not meta_path.exists():
        raise ProbeError(f"missing production {meta_path.name}")
    try:
        meta = json.loads(meta_path.read_text())
    except json.JSONDecodeError as error:
        raise ProbeError(f"{meta_path} is not valid JSON: {error}") from error
    config = meta.get("config", {})
    try:
        window = int(config["estimator.window_size"])
    except (KeyError, TypeError, ValueError) as error:
        raise ProbeError(
            f"{meta_path.name}: config.estimator.window_size missing or "
            f"non-positive"
        ) from error
    if window <= 0:
        raise ProbeError(
            f"{meta_path.name}: config.estimator.window_size must be > 0"
        )
    return window


def check_diag(candidate_dir: Path, run_dir: Path,
               analysis: Analysis) -> None:
    diag_path = candidate_dir / "diag.csv"
    if not diag_path.exists():
        raise ProbeError(f"missing {diag_path}")
    rows = load_rows(diag_path, HEADER)

    if len(rows) != analysis.frames:
        raise ProbeError(
            f"{diag_path.name} row count {len(rows)} != meta.frames "
            f"{analysis.frames} (one row per candidate frame)"
        )
    previous_ns: Optional[int] = None
    for row in rows:
        ts_ns = int(row["ts_ns"])
        if previous_ns is not None and ts_ns <= previous_ns:
            raise ProbeError(
                f"{diag_path.name}: timestamps must be strictly increasing "
                f"({ts_ns} after {previous_ns})"
            )
        previous_ns = ts_ns
    analysis.checks.append(
        f"diag: {len(rows)} rows, one per candidate frame, ts increasing"
    )

    ok_rows = sum(1 for row in rows if row["status"] == "ok")
    if ok_rows != analysis.ok:
        raise ProbeError(
            f"{diag_path.name}: status=ok rows {ok_rows} != meta.ok "
            f"{analysis.ok}"
        )
    for row in rows:
        if row["status"] not in _STATUSES:
            raise ProbeError(
                f"{diag_path.name}: unknown status {row['status']!r}"
            )

    # 与 production diag.csv 按帧对齐（ts 必须一致；status/keyframe/
    # segment/bias 由 candidate 拥有，P2b 下允许与 production 不同）。
    prod_diag = run_dir / "diag.csv"
    if not prod_diag.exists():
        raise ProbeError(f"missing production {prod_diag}")
    prod_rows = load_rows(prod_diag, PROD_DIAG_HEADER)
    if len(prod_rows) != len(rows):
        raise ProbeError(
            f"production diag rows {len(prod_rows)} != candidate diag rows "
            f"{len(rows)}"
        )
    for index, (cand, prod) in enumerate(zip(rows, prod_rows)):
        if int(cand["ts_ns"]) != int(prod["timestamp_ns"]):
            raise ProbeError(
                f"row {index}: candidate ts {cand['ts_ns']} != production "
                f"ts {prod['timestamp_ns']}"
            )
    analysis.checks.append(
        f"diag: {len(rows)} rows frame-aligned to production diag rows"
    )

    # graph 结构：active pose 有界（strict-< cutoff ⇒ ≤ window+1，容差 2），
    # marginalized/retired 为增量计数（非负）。
    window = load_config_window(run_dir)
    graph_rows = 0
    max_active_poses = 0
    for index, row in enumerate(rows):
        if row["graph_valid"] != "1":
            continue
        graph_rows += 1
        active = int(row["graph_active_pose_count"])
        max_active_poses = max(max_active_poses, active)
        if active > window + 2:
            raise ProbeError(
                f"row {index}: graph_active_pose_count {active} exceeds "
                f"window {window} + 2"
            )
        if int(row["graph_marginalized_pose_count"]) < 0:
            raise ProbeError(
                f"row {index}: graph_marginalized_pose_count must be >= 0"
            )
        if int(row["graph_retired_landmark_count"]) < 0:
            raise ProbeError(
                f"row {index}: graph_retired_landmark_count must be >= 0"
            )
    analysis.graph_rows      = graph_rows
    analysis.max_active_poses = max_active_poses
    analysis.checks.append(
        f"graph: {graph_rows} valid rows, max active poses "
        f"{max_active_poses} <= window {window} + 2"
    )


def check_est(candidate_dir: Path, run_dir: Path,
              analysis: Analysis) -> None:
    candidate_tum = candidate_dir / "est.tum"
    if not candidate_tum.exists():
        raise ProbeError(f"missing {candidate_tum}")
    lines = candidate_tum.read_text().splitlines()
    if len(lines) != analysis.ok:
        raise ProbeError(
            f"est.tum rows {len(lines)} != candidate diag status=ok rows "
            f"{analysis.ok}"
        )
    finite = True
    for line in lines:
        if "nan" in line or "inf" in line:
            finite = False
            break
    if not finite:
        raise ProbeError("est.tum contains non-finite pose entries")
    analysis.est_pose_count = len(lines)
    analysis.est_finite     = finite
    analysis.checks.append(
        f"est.tum: {len(lines)} finite poses == status=ok rows"
    )

    # kf.tum 存在性由 candidate 拥有：有 KF 时必须存在且行数 = KF 数，
    # 无 KF 时允许缺失。
    candidate_kf = candidate_dir / "kf.tum"
    if candidate_kf.exists():
        kf_lines = candidate_kf.read_text().splitlines()
        if len(kf_lines) != analysis.keyframes:
            raise ProbeError(
                f"kf.tum rows {len(kf_lines)} != meta.keyframes "
                f"{analysis.keyframes}"
            )
    elif analysis.keyframes > 0:
        raise ProbeError(
            f"missing {candidate_kf} although meta.keyframes "
            f"{analysis.keyframes} > 0"
        )


def analyze(run_dir: Path) -> Analysis:
    candidate_dir = run_dir / "candidate"
    if not candidate_dir.is_dir():
        raise ProbeError(f"no candidate/ directory under {run_dir}")

    analysis = Analysis(run_dir=run_dir)
    check_meta(candidate_dir, analysis)
    check_diag(candidate_dir, run_dir, analysis)
    check_est(candidate_dir, run_dir, analysis)
    return analysis


def render_markdown(analysis: Analysis, out: TextIO) -> None:
    out.write("# candidate_pipeline_probe\n\n")
    out.write(f"- run_dir: `{analysis.run_dir}`\n")
    out.write(f"- frames: {analysis.frames}\n")
    out.write(f"- ok/rejected/failed: {analysis.ok}/{analysis.rejected}/"
              f"{analysis.failed}\n")
    out.write(f"- keyframes: {analysis.keyframes}\n")
    out.write(f"- candidate wall_s: {analysis.candidate_wall_s:.6f}\n")
    out.write(f"- est.tum poses: {analysis.est_pose_count} "
              f"(finite: {analysis.est_finite})\n")
    out.write(f"- graph rows: {analysis.graph_rows}, max active poses: "
              f"{analysis.max_active_poses} (window {analysis.window_size})\n")
    out.write("\n## checks\n\n")
    for check in analysis.checks:
        out.write(f"- {check}\n")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Verify M4.4 P2b candidate-owned fixed-lag artifacts."
    )
    parser.add_argument("run_dir", type=Path,
                        help="phad_vo_bench run output directory "
                             "(--candidate run)")
    parser.add_argument("--markdown", type=Path, default=None,
                        help="optional markdown report path")
    arguments = parser.parse_args(argv)

    try:
        analysis = analyze(arguments.run_dir)
    except ProbeError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    render_markdown(analysis, sys.stdout)
    if arguments.markdown is not None:
        with arguments.markdown.open("w") as stream:
            render_markdown(analysis, stream)
    print(f"PASS: ownership=full_pipeline, {analysis.frames} frames, "
          f"terminal=0 failed=0")
    return 0


if __name__ == "__main__":
    sys.exit(main())

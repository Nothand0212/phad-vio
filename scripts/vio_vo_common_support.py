#!/usr/bin/env python3
"""在 off/fused/GT 的同一时间戳支持集上审计 VO 指标与闭环风险。"""

import argparse
import bisect
import csv
import json
import math
from decimal import Decimal
from pathlib import Path

import numpy as np


def _timestamp_ns(text):
    value = Decimal(text) * Decimal(1_000_000_000)
    integral = value.to_integral_value()
    if value != integral:
        raise ValueError(f"timestamp has sub-nanosecond precision: {text}")
    return int(integral)


def _rotation(qx, qy, qz, qw):
    q = np.asarray([qw, qx, qy, qz], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion must be finite and non-zero")
    w, x, y, z = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _pose(tx, ty, tz, qw, qx, qy, qz):
    transform = np.eye(4)
    transform[:3, :3] = _rotation(qx, qy, qz, qw)
    transform[:3, 3] = [tx, ty, tz]
    if not np.all(np.isfinite(transform)):
        raise ValueError("pose must be finite")
    return transform


def _load_tum(path):
    poses = {}
    previous = None
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(f"{path}:{line_number}: expected 8 fields")
            ts_ns = _timestamp_ns(fields[0])
            if previous is not None and ts_ns <= previous:
                raise ValueError(f"{path}:{line_number}: timestamps not increasing")
            values = [float(value) for value in fields[1:]]
            poses[ts_ns] = _pose(
                values[0], values[1], values[2], values[6], values[3],
                values[4], values[5]
            )
            previous = ts_ns
    if not poses:
        raise ValueError(f"{path}: trajectory is empty")
    return poses


def _load_euroc_gt(root):
    path = root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    poses = {}
    previous = None
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        next(reader, None)
        for line_number, row in enumerate(reader, 2):
            if len(row) < 8:
                raise ValueError(f"{path}:{line_number}: expected at least 8 columns")
            ts_ns = int(row[0])
            if previous is not None and ts_ns <= previous:
                raise ValueError(f"{path}:{line_number}: timestamps not increasing")
            values = [float(value) for value in row[1:8]]
            poses[ts_ns] = _pose(
                values[0], values[1], values[2], values[3], values[4],
                values[5], values[6]
            )
            previous = ts_ns
    if not poses:
        raise ValueError(f"{path}: groundtruth is empty")
    return poses


def _nearest_index(timestamps, timestamp):
    upper = bisect.bisect_left(timestamps, timestamp)
    if upper == 0:
        return 0
    if upper == len(timestamps):
        return len(timestamps) - 1
    lower = upper - 1
    return upper if timestamps[upper] - timestamp < timestamp - timestamps[lower] else lower


def _common_support(off, fused, gt, max_dt_ns):
    common = sorted(set(off) & set(fused))
    gt_timestamps = sorted(gt)
    matched = []
    for timestamp in common:
        gt_index = _nearest_index(gt_timestamps, timestamp)
        gt_timestamp = gt_timestamps[gt_index]
        if abs(timestamp - gt_timestamp) <= max_dt_ns:
            matched.append((timestamp, off[timestamp], fused[timestamp],
                            gt[gt_timestamp], timestamp - gt_timestamp))
    if len(matched) < 3:
        raise ValueError("exact-common support has fewer than 3 GT matches")
    return common, matched


def _align_se3(source, target):
    source = np.asarray(source)
    target = np.asarray(target)
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = np.zeros((3, 3))
    for source_point, target_point in zip(source, target):
        covariance += np.outer(target_point - target_center,
                               source_point - source_center)
    u, singular, vh = np.linalg.svd(covariance)
    if singular[0] <= 0.0 or singular[1] / singular[0] < 1e-8:
        raise ValueError("exact-common positions are degenerate for SE3 alignment")
    correction = np.eye(3)
    if np.linalg.det(u @ vh) < 0.0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vh
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def _angle_deg(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _stats(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError("metric values must be non-empty and finite")
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def _ate(matched, pose_index):
    estimate = [entry[pose_index] for entry in matched]
    groundtruth = [entry[3] for entry in matched]
    alignment = _align_se3(
        [pose[:3, 3] for pose in estimate],
        [pose[:3, 3] for pose in groundtruth],
    )
    trans_errors = []
    rot_errors = []
    for pose, reference in zip(estimate, groundtruth):
        aligned = alignment @ pose
        trans_errors.append(np.linalg.norm(aligned[:3, 3] - reference[:3, 3]))
        rot_errors.append(_angle_deg(aligned[:3, :3].T @ reference[:3, :3]))
    return _stats(trans_errors), _stats(rot_errors)


def _inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def _rpe_pairs(timestamps, delta_ns, tolerance_ns):
    pairs = []
    dropped = 0
    for index, timestamp in enumerate(timestamps):
        begin = index + 1
        if begin >= len(timestamps):
            dropped += 1
            continue
        target = timestamp + delta_ns
        upper = bisect.bisect_left(timestamps, target, begin)
        if upper == len(timestamps):
            best = len(timestamps) - 1
        elif upper == begin:
            best = upper
        else:
            lower = upper - 1
            best = lower if target - timestamps[lower] < timestamps[upper] - target else upper
        if abs(timestamps[best] - target) > tolerance_ns:
            dropped += 1
            continue
        pairs.append((index, best))
    if not pairs:
        raise ValueError("exact-common support has no RPE pairs")
    return pairs, dropped


def _rpe(matched, pose_index, pairs):
    trans_errors = []
    rot_errors = []
    for first, second in pairs:
        estimate_motion = _inverse(matched[first][pose_index]) @ matched[second][pose_index]
        gt_motion = _inverse(matched[first][3]) @ matched[second][3]
        error = _inverse(gt_motion) @ estimate_motion
        trans_errors.append(np.linalg.norm(error[:3, 3]))
        rot_errors.append(_angle_deg(error[:3, :3]))
    return _stats(trans_errors), _stats(rot_errors)


def _adjacent_jumps(matched, pose_index):
    jumps = []
    for first, second in zip(matched, matched[1:]):
        motion = _inverse(first[pose_index]) @ second[pose_index]
        jumps.append(np.linalg.norm(motion[:3, 3]))
    return _stats(jumps)


def _trajectory_summary(run_dir):
    with (run_dir / "summary.json").open() as stream:
        trajectory = json.load(stream)["trajectory"]
    return {
        key: trajectory[key]
        for key in ("poses_written", "completion_rate", "coverage_rate", "failed")
    }


def _keyframe_comparison(off_dir, fused_dir):
    off = list(_load_tum(off_dir / "kf.tum"))
    fused = list(_load_tum(fused_dir / "kf.tum"))
    index = 0
    while index < min(len(off), len(fused)) and off[index] == fused[index]:
        index += 1
    diverged = index < len(off) or index < len(fused)
    return {
        "off_count": len(off),
        "fused_count": len(fused),
        "common_prefix": index,
        "first_divergence_index": index if diverged else None,
        "off_ts_ns": off[index] if index < len(off) else None,
        "fused_ts_ns": fused[index] if index < len(fused) else None,
    }


def _factor_coverage(fused_dir):
    path = fused_dir / "gyro_state.csv"
    if not path.exists():
        return {"rows": 0, "ready_rows": 0, "factor_rows": 0,
                "posterior_rows": 0, "coverage_after_ready": 0.0}
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    ready = [row for row in rows if row.get("align_ready") == "1"]
    factor = [row for row in rows if int(row.get("gyro_factor_count", "0")) > 0]
    posterior = [row for row in rows if row.get("gyro_post_valid") == "1"]
    return {
        "rows": len(rows),
        "ready_rows": len(ready),
        "factor_rows": len(factor),
        "posterior_rows": len(posterior),
        "coverage_after_ready": len(factor) / len(ready) if ready else 0.0,
        "first_ready_ts_ns": int(ready[0]["ts_ns"]) if ready else None,
        "first_factor_ts_ns": int(factor[0]["ts_ns"]) if factor else None,
    }


def evaluate_runs(off_dir, fused_dir, gt_root, max_dt_ns=2_500_000,
                  rpe_delta_ns=1_000_000_000,
                  rpe_tolerance_ns=25_000_000):
    off_dir = Path(off_dir)
    fused_dir = Path(fused_dir)
    off = _load_tum(off_dir / "est.tum")
    fused = _load_tum(fused_dir / "est.tum")
    gt = _load_euroc_gt(Path(gt_root))
    common, matched = _common_support(off, fused, gt, max_dt_ns)
    timestamps = [entry[0] for entry in matched]
    pairs, dropped = _rpe_pairs(timestamps, rpe_delta_ns, rpe_tolerance_ns)

    off_ate, off_ate_rot = _ate(matched, 1)
    fused_ate, fused_ate_rot = _ate(matched, 2)
    off_rpe, off_rpe_rot = _rpe(matched, 1, pairs)
    fused_rpe, fused_rpe_rot = _rpe(matched, 2, pairs)
    off_summary = _trajectory_summary(off_dir)
    fused_summary = _trajectory_summary(fused_dir)
    checks = {
        "no_new_failures": fused_summary["failed"] <= off_summary["failed"],
        "poses_not_lower": fused_summary["poses_written"] >= off_summary["poses_written"],
        "completion_not_lower": fused_summary["completion_rate"] >= off_summary["completion_rate"],
        "coverage_not_lower": fused_summary["coverage_rate"] >= off_summary["coverage_rate"],
        "ate_strictly_better": fused_ate["rmse"] < off_ate["rmse"],
        "rpe_strictly_better": fused_rpe["rmse"] < off_rpe["rmse"],
    }
    return {
        "support": {
            "off_poses": len(off),
            "fused_poses": len(fused),
            "exact_common": len(common),
            "gt_matched": len(matched),
            "gt_dropped": len(common) - len(matched),
            "max_dt_ns": max(abs(entry[4]) for entry in matched),
        },
        "trajectory": {"off": off_summary, "fused": fused_summary},
        "ate_m": {"off": off_ate, "fused": fused_ate},
        "ate_rot_deg": {"off": off_ate_rot, "fused": fused_ate_rot},
        "rpe": {
            "delta_ns": rpe_delta_ns,
            "tolerance_ns": rpe_tolerance_ns,
            "pair_count": len(pairs),
            "dropped_no_partner": dropped,
        },
        "rpe_m": {"off": off_rpe, "fused": fused_rpe},
        "rpe_rot_deg": {"off": off_rpe_rot, "fused": fused_rpe_rot},
        "adjacent_jump_m": {
            "off": _adjacent_jumps(matched, 1),
            "fused": _adjacent_jumps(matched, 2),
        },
        "keyframes": _keyframe_comparison(off_dir, fused_dir),
        "factors": _factor_coverage(fused_dir),
        "gate": {"checks": checks, "core_pass": all(checks.values())},
    }


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("off_run", type=Path)
    parser.add_argument("fused_run", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-dt-ms", type=float, default=2.5)
    parser.add_argument("--rpe-delta-s", type=float, default=1.0)
    parser.add_argument("--rpe-tolerance-ms", type=float, default=25.0)
    return parser.parse_args()


def main():
    args = _parse_args()
    report = evaluate_runs(
        args.off_run,
        args.fused_run,
        args.euroc_root,
        max_dt_ns=round(args.max_dt_ms * 1_000_000),
        rpe_delta_ns=round(args.rpe_delta_s * 1_000_000_000),
        rpe_tolerance_ns=round(args.rpe_tolerance_ms * 1_000_000),
    )
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

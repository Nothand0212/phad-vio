#!/usr/bin/env python3
"""Analyze fixed-production-epoch keyframe IMU shadow evidence.

The input is the CSV emitted by ``--keyframe-shadow-probe``.  Shadow
candidate counts are first-order counterfactuals on the production accepted-KF
epoch; they are not a replay of the estimator under a different schedule.
Only the Python standard library is required.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO


HEADER = (
    "frame",
    "ts_ns",
    "epoch_before",
    "frames_since_kf",
    "prod_selected",
    "prod_rule",
    "prod_empty",
    "prod_bootstrap",
    "prod_low_tracks",
    "prod_timeout",
    "prod_low_survival",
    "prod_parallax",
    "obs",
    "common",
    "survival",
    "raw_px",
    "pose_comp_px",
    "pose_parallax_n",
    "since_kf_ns",
    "imu_valid",
    "imu_reason",
    "imu_sample_n",
    "imu_gap",
    "imu_t_i_ns",
    "imu_t_j_ns",
    "imu_dt_s",
    "imu_angle_rad",
    "bias_gyr_x",
    "bias_gyr_y",
    "bias_gyr_z",
    "imu_comp_px",
    "imu_parallax_n",
    "imu_gt_10",
    "imu_gt_15",
    "imu_gt_30",
    "status",
    "epoch_committed",
    "epoch_after",
)

RULES = {
    "none",
    "bootstrap",
    "low_tracks",
    "timeout",
    "low_survival",
    "parallax",
}
STATUSES = {"ok", "rejected", "failed"}
IMU_REASONS = {
    "none",
    "imu_disabled",
    "no_accepted_kf",
    "imu_gap",
    "insufficient_samples",
    "non_monotonic_timestamp",
    "non_finite_gyro",
    "non_finite_bias",
    "non_finite_result",
}

PASSIVE_INTEGER_FIELDS = (
    "obs",
    "common",
    "pose_parallax_n",
    "since_kf_ns",
)
PASSIVE_FLOAT_FIELDS = (
    "survival",
    "raw_px",
    "bias_gyr_x",
    "bias_gyr_y",
    "bias_gyr_z",
)


class ProbeError(RuntimeError):
    """The shadow CSV violates its schema or transaction contract."""


@dataclass(frozen=True)
class ShadowRow:
    frame: int
    ts_ns: int
    epoch_before: int
    frames_since_kf: int
    prod_selected: bool
    prod_rule: str
    prod_empty: bool
    prod_bootstrap: bool
    prod_low_tracks: bool
    prod_timeout: bool
    prod_low_survival: bool
    prod_parallax: bool
    pose_comp_px: float
    imu_valid: bool
    imu_reason: str
    imu_sample_n: int
    imu_gap: bool
    imu_t_i_ns: int
    imu_t_j_ns: int
    imu_dt_s: float
    imu_angle_rad: float
    imu_comp_px: float
    imu_parallax_n: int
    imu_gt_10: bool
    imu_gt_15: bool
    imu_gt_30: bool
    status: str
    epoch_committed: bool
    epoch_after: int


def _binary(value: str, field: str, path: Path, line: int) -> bool:
    if value == "0":
        return False
    if value == "1":
        return True
    raise ProbeError(
        f"{path}:{line}: {field} must be 0 or 1, got {value!r}"
    )


def _integer(value: str, field: str, path: Path, line: int) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field}") from exc


def _finite(value: str, field: str, path: Path, line: int) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field}") from exc
    if not math.isfinite(parsed):
        raise ProbeError(f"{path}:{line}: {field} must be finite")
    return parsed


def _selected_rule(row: ShadowRow) -> str:
    if row.prod_empty:
        return "none"
    if row.prod_bootstrap:
        return "bootstrap"
    if row.prod_low_tracks:
        return "low_tracks"
    if row.prod_timeout:
        return "timeout"
    if row.prod_low_survival:
        return "low_survival"
    if row.prod_parallax:
        return "parallax"
    return "none"


def load_rows(path: Path) -> tuple[ShadowRow, ...]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc

    rows: list[ShadowRow] = []
    with input_file:
        reader = csv.DictReader(input_file)
        if tuple(reader.fieldnames or ()) != HEADER:
            raise ProbeError(f"{path}: header mismatch")
        for index, raw in enumerate(reader):
            line = index + 2
            if None in raw or any(value is None for value in raw.values()):
                raise ProbeError(f"{path}:{line}: column count mismatch")
            rule = raw["prod_rule"]
            status = raw["status"]
            reason = raw["imu_reason"]
            if rule not in RULES:
                raise ProbeError(f"{path}:{line}: invalid prod_rule {rule!r}")
            if status not in STATUSES:
                raise ProbeError(f"{path}:{line}: invalid status {status!r}")
            if reason not in IMU_REASONS:
                raise ProbeError(f"{path}:{line}: invalid imu_reason {reason!r}")
            for field in PASSIVE_INTEGER_FIELDS:
                _integer(raw[field], field, path, line)
            for field in PASSIVE_FLOAT_FIELDS:
                _finite(raw[field], field, path, line)
            row = ShadowRow(
                frame=_integer(raw["frame"], "frame", path, line),
                ts_ns=_integer(raw["ts_ns"], "ts_ns", path, line),
                epoch_before=_integer(
                    raw["epoch_before"], "epoch_before", path, line
                ),
                frames_since_kf=_integer(
                    raw["frames_since_kf"], "frames_since_kf", path, line
                ),
                prod_selected=_binary(
                    raw["prod_selected"], "prod_selected", path, line
                ),
                prod_rule=rule,
                prod_empty=_binary(
                    raw["prod_empty"], "prod_empty", path, line
                ),
                prod_bootstrap=_binary(
                    raw["prod_bootstrap"], "prod_bootstrap", path, line
                ),
                prod_low_tracks=_binary(
                    raw["prod_low_tracks"], "prod_low_tracks", path, line
                ),
                prod_timeout=_binary(
                    raw["prod_timeout"], "prod_timeout", path, line
                ),
                prod_low_survival=_binary(
                    raw["prod_low_survival"],
                    "prod_low_survival",
                    path,
                    line,
                ),
                prod_parallax=_binary(
                    raw["prod_parallax"], "prod_parallax", path, line
                ),
                pose_comp_px=_finite(
                    raw["pose_comp_px"], "pose_comp_px", path, line
                ),
                imu_valid=_binary(
                    raw["imu_valid"], "imu_valid", path, line
                ),
                imu_reason=reason,
                imu_sample_n=_integer(
                    raw["imu_sample_n"], "imu_sample_n", path, line
                ),
                imu_gap=_binary(raw["imu_gap"], "imu_gap", path, line),
                imu_t_i_ns=_integer(
                    raw["imu_t_i_ns"], "imu_t_i_ns", path, line
                ),
                imu_t_j_ns=_integer(
                    raw["imu_t_j_ns"], "imu_t_j_ns", path, line
                ),
                imu_dt_s=_finite(
                    raw["imu_dt_s"], "imu_dt_s", path, line
                ),
                imu_angle_rad=_finite(
                    raw["imu_angle_rad"], "imu_angle_rad", path, line
                ),
                imu_comp_px=_finite(
                    raw["imu_comp_px"], "imu_comp_px", path, line
                ),
                imu_parallax_n=_integer(
                    raw["imu_parallax_n"], "imu_parallax_n", path, line
                ),
                imu_gt_10=_binary(
                    raw["imu_gt_10"], "imu_gt_10", path, line
                ),
                imu_gt_15=_binary(
                    raw["imu_gt_15"], "imu_gt_15", path, line
                ),
                imu_gt_30=_binary(
                    raw["imu_gt_30"], "imu_gt_30", path, line
                ),
                status=status,
                epoch_committed=_binary(
                    raw["epoch_committed"],
                    "epoch_committed",
                    path,
                    line,
                ),
                epoch_after=_integer(
                    raw["epoch_after"], "epoch_after", path, line
                ),
            )
            rows.append(row)

    if not rows:
        raise ProbeError(f"{path}: no frame rows")
    _validate_rows(path, rows)
    return tuple(rows)


def _validate_rows(path: Path, rows: list[ShadowRow]) -> None:
    expected_epoch = 0
    previous_ts: Optional[int] = None
    last_kf_frame: Optional[int] = None
    for index, row in enumerate(rows):
        line = index + 2
        if row.frame != index:
            raise ProbeError(f"{path}:{line}: frame axis is not contiguous")
        if previous_ts is not None and row.ts_ns <= previous_ts:
            raise ProbeError(
                f"{path}:{line}: timestamps are not strictly increasing"
            )
        previous_ts = row.ts_ns
        if row.epoch_before != expected_epoch:
            raise ProbeError(f"{path}:{line}: epoch_before mismatch")
        expected_frames = (
            row.frame - last_kf_frame if last_kf_frame is not None else -1
        )
        if row.frames_since_kf != expected_frames:
            raise ProbeError(f"{path}:{line}: frames_since_kf mismatch")
        selected_rule = _selected_rule(row)
        if row.prod_rule != selected_rule:
            raise ProbeError(f"{path}:{line}: production rule mismatch")
        if row.prod_selected != (row.prod_rule != "none"):
            raise ProbeError(f"{path}:{line}: prod_selected mismatch")

        committed = row.prod_selected and row.status == "ok"
        if row.epoch_committed != committed:
            raise ProbeError(f"{path}:{line}: epoch_committed mismatch")
        expected_after = expected_epoch + (1 if committed else 0)
        if row.epoch_after != expected_after:
            raise ProbeError(f"{path}:{line}: epoch_after mismatch")
        if committed:
            last_kf_frame = row.frame
        expected_epoch = expected_after

        if row.imu_valid:
            if row.imu_reason != "none":
                raise ProbeError(f"{path}:{line}: valid IMU reason mismatch")
            if row.imu_gap or row.imu_sample_n < 2:
                raise ProbeError(f"{path}:{line}: invalid valid-IMU metadata")
            if row.imu_t_i_ns >= row.imu_t_j_ns:
                raise ProbeError(f"{path}:{line}: invalid IMU interval")
            if row.imu_t_j_ns != row.ts_ns:
                raise ProbeError(f"{path}:{line}: IMU endpoint mismatch")
            expected_dt = (row.imu_t_j_ns - row.imu_t_i_ns) * 1e-9
            if not math.isclose(
                row.imu_dt_s, expected_dt, rel_tol=1e-12, abs_tol=1e-12
            ):
                raise ProbeError(f"{path}:{line}: IMU duration mismatch")
        else:
            if row.imu_reason == "none":
                raise ProbeError(f"{path}:{line}: invalid IMU reason mismatch")
            if row.imu_comp_px != 0.0 or row.imu_parallax_n != 0:
                raise ProbeError(
                    f"{path}:{line}: invalid IMU row has parallax evidence"
                )
        expected_thresholds = (
            row.imu_valid and row.imu_comp_px > 10.0,
            row.imu_valid and row.imu_comp_px > 15.0,
            row.imu_valid and row.imu_comp_px > 30.0,
        )
        actual_thresholds = (
            row.imu_gt_10,
            row.imu_gt_15,
            row.imu_gt_30,
        )
        if actual_thresholds != expected_thresholds:
            raise ProbeError(f"{path}:{line}: IMU threshold flags mismatch")


def _shadow_rule4(row: ShadowRow, threshold: float) -> bool:
    force = (
        row.prod_empty
        or row.prod_bootstrap
        or row.prod_low_tracks
        or row.prod_timeout
        or row.prod_low_survival
    )
    return not force and row.imu_valid and row.imu_comp_px > threshold


def _shadow_candidate(row: ShadowRow, threshold: float) -> bool:
    if row.prod_empty:
        return False
    if (
        row.prod_bootstrap
        or row.prod_low_tracks
        or row.prod_timeout
        or row.prod_low_survival
    ):
        return True
    return row.imu_valid and row.imu_comp_px > threshold


def _run_lengths(hits: list[bool]) -> Counter[int]:
    lengths: Counter[int] = Counter()
    current = 0
    for hit in hits:
        if hit:
            current += 1
        elif current > 0:
            lengths[current] += 1
            current = 0
    if current > 0:
        lengths[current] += 1
    return lengths


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = q * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def analyze(
    rows: tuple[ShadowRow, ...],
    thresholds: tuple[float, ...] = (10.0, 15.0, 30.0),
    max_min_gap_frames: int = 6,
) -> dict[str, object]:
    if not thresholds or any(
        not math.isfinite(value) or value <= 0.0 for value in thresholds
    ):
        raise ValueError("thresholds must be finite and positive")
    if max_min_gap_frames < 0:
        raise ValueError("max_min_gap_frames must be non-negative")

    valid = [row for row in rows if row.imu_valid]
    comparable = [row for row in valid if row.imu_parallax_n > 0]
    differences = [
        row.imu_comp_px - row.pose_comp_px for row in comparable
    ]
    reports: list[dict[str, object]] = []
    for threshold in thresholds:
        shadow = [_shadow_candidate(row, threshold) for row in rows]
        production = [row.prod_selected for row in rows]
        additions = [
            row.frame
            for row, prod, candidate in zip(rows, production, shadow)
            if candidate and not prod
        ]
        removals = [
            row.frame
            for row, prod, candidate in zip(rows, production, shadow)
            if prod and not candidate
        ]
        differing = sorted(additions + removals)
        rule4_hits = [_shadow_rule4(row, threshold) for row in rows]
        runs = _run_lengths(rule4_hits)
        veto_counts = {
            gap: sum(
                1
                for row, hit in zip(rows, rule4_hits)
                if hit
                and row.frames_since_kf >= 0
                and row.frames_since_kf <= gap
            )
            for gap in range(1, max_min_gap_frames + 1)
        }
        frame_gap_hits = Counter(
            row.frames_since_kf
            for row, hit in zip(rows, rule4_hits)
            if hit and row.frames_since_kf >= 0
        )
        reports.append(
            {
                "threshold": threshold,
                "production_candidates": sum(production),
                "shadow_candidates": sum(shadow),
                "additions": additions,
                "removals": removals,
                "first_difference": differing[0] if differing else None,
                "addition_rules": Counter(
                    rows[frame].prod_rule for frame in additions
                ),
                "removal_rules": Counter(
                    rows[frame].prod_rule for frame in removals
                ),
                "rule4_hits": sum(rule4_hits),
                "run_lengths": runs,
                "isolated_hits": runs.get(1, 0),
                "gap_veto_counts": veto_counts,
                "frame_gap_hits": frame_gap_hits,
            }
        )

    return {
        "frames": len(rows),
        "accepted_epochs": rows[-1].epoch_after,
        "valid_frames": len(valid),
        "first_valid_frame": valid[0].frame if valid else None,
        "reasons": Counter(row.imu_reason for row in rows),
        "comparable_frames": len(comparable),
        "pose_comp_percentiles": {
            q: _percentile([row.pose_comp_px for row in comparable], q)
            for q in (0.5, 0.9, 0.95, 0.99)
        },
        "imu_comp_percentiles": {
            q: _percentile([row.imu_comp_px for row in comparable], q)
            for q in (0.5, 0.9, 0.95, 0.99)
        },
        "difference_percentiles": {
            q: _percentile(differences, q)
            for q in (0.5, 0.9, 0.95, 0.99)
        },
        "thresholds": reports,
    }


def _format_counter(counter: Counter[object]) -> str:
    if not counter:
        return "none"
    try:
        keys = sorted(counter)
    except TypeError:
        keys = sorted(counter, key=str)
    return ", ".join(
        f"{key}:{counter[key]}" for key in keys
    )


def render_markdown(
    path: Path, result: dict[str, object], output: TextIO
) -> None:
    output.write("# Fixed-production-epoch keyframe shadow probe\n\n")
    output.write(f"Input: `{path}`\n\n")
    output.write(
        f"Frames / accepted epochs: `{result['frames']} / "
        f"{result['accepted_epochs']}`. IMU valid: "
        f"`{result['valid_frames']}`, first valid frame: "
        f"`{result['first_valid_frame']}`.\n\n"
    )
    output.write(
        "IMU reasons: `"
        + _format_counter(result["reasons"])  # type: ignore[arg-type]
        + "`.\n\n"
    )
    output.write("## Parallax distribution\n\n")
    output.write("| source | p50 | p90 | p95 | p99 |\n")
    output.write("|---|---:|---:|---:|---:|\n")
    for label, key in (
        ("pose_lag", "pose_comp_percentiles"),
        ("imu", "imu_comp_percentiles"),
        ("imu-pose", "difference_percentiles"),
    ):
        values = result[key]
        assert isinstance(values, dict)
        output.write(
            f"| {label} | {values[0.5]:.6f} | {values[0.9]:.6f} | "
            f"{values[0.95]:.6f} | {values[0.99]:.6f} |\n"
        )

    output.write("\n## Frozen-epoch candidates\n\n")
    output.write(
        "| threshold | production | shadow | add | remove | first diff | "
        "Rule4 hits | isolated runs |\n"
    )
    output.write("|---:|---:|---:|---:|---:|---:|---:|---:|\n")
    reports = result["thresholds"]
    assert isinstance(reports, list)
    for report in reports:
        assert isinstance(report, dict)
        first = report["first_difference"]
        output.write(
            f"| {report['threshold']:.3f} | "
            f"{report['production_candidates']} | "
            f"{report['shadow_candidates']} | "
            f"{len(report['additions'])} | {len(report['removals'])} | "
            f"{first if first is not None else 'none'} | "
            f"{report['rule4_hits']} | {report['isolated_hits']} |\n"
        )

    output.write("\n## Difference and hit decomposition\n\n")
    for report in reports:
        output.write(f"### Threshold {report['threshold']:.3f}px\n\n")
        output.write(
            "- additions by production rule: `"
            + _format_counter(report["addition_rules"])
            + "`\n"
        )
        output.write(
            "- removals by production rule: `"
            + _format_counter(report["removal_rules"])
            + "`\n"
        )
        output.write(
            "- Rule4 run lengths (length:count): `"
            + _format_counter(report["run_lengths"])
            + "`\n"
        )
        output.write(
            "- Rule4 hits by frames since KF (gap:hits): `"
            + _format_counter(report["frame_gap_hits"])
            + "`\n\n"
        )

    output.write("\n## First-order minimum-gap veto counts\n\n")
    max_gap = max(
        (
            max(report["gap_veto_counts"], default=0)
            for report in reports
            if isinstance(report, dict)
        ),
        default=0,
    )
    if max_gap == 0:
        output.write("No minimum-gap candidates requested.\n")
    else:
        output.write(
            "| threshold | "
            + " | ".join(
                f"gap<={gap}" for gap in range(1, max_gap + 1)
            )
            + " |\n"
        )
        output.write("|---:|" + "---:|" * max_gap + "\n")
        for report in reports:
            counts = report["gap_veto_counts"]
            output.write(
                f"| {report['threshold']:.3f} | "
                + " | ".join(
                    str(counts[gap]) for gap in range(1, max_gap + 1)
                )
                + " |\n"
            )

    output.write(
        "\n> These are frozen-production-epoch first-order counterfactuals. "
        "A veto or added candidate would change the next baseline and "
        "estimator window; this report does not simulate that live schedule.\n"
    )


def _parse_thresholds(text: str) -> tuple[float, ...]:
    try:
        values = tuple(float(part) for part in text.split(",") if part)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("invalid threshold list") from exc
    if not values or any(not math.isfinite(v) or v <= 0.0 for v in values):
        raise argparse.ArgumentTypeError(
            "thresholds must be finite positive values"
        )
    return values


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--thresholds", type=_parse_thresholds, default=(10.0, 15.0, 30.0)
    )
    parser.add_argument("--max-min-gap-frames", type=int, default=6)
    args = parser.parse_args(argv)
    try:
        rows = load_rows(args.csv_path)
        result = analyze(rows, args.thresholds, args.max_min_gap_frames)
        render_markdown(args.csv_path, result, sys.stdout)
    except (ProbeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

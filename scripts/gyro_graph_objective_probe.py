#!/usr/bin/env python3
"""Summarize boundary/interior costs in a gyro-visual optimizer graph."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Optional, TextIO


OBJECTIVE_FIELDS = (
    "timestamp_ns",
    "state_key",
    "fusion_mode",
    "gyro_factor_n",
    "gyro_obj_valid",
    "gyro_obj_factor_n",
    "gyro_obj_interior_n",
    "gyro_obj_stereo_n",
    "gyro_obj_total_init",
    "gyro_obj_total_post",
    "gyro_obj_gyro_init",
    "gyro_obj_gyro_post",
    "gyro_obj_boundary_init",
    "gyro_obj_boundary_post",
    "gyro_obj_boundary_r_init_rad",
    "gyro_obj_boundary_r_post_rad",
    "gyro_obj_boundary_w_init",
    "gyro_obj_boundary_w_post",
    "gyro_obj_interior_init",
    "gyro_obj_interior_post",
    "gyro_obj_new_init",
    "gyro_obj_new_post",
    "gyro_obj_new_r_init_rad",
    "gyro_obj_new_r_post_rad",
    "gyro_obj_new_w_init",
    "gyro_obj_new_w_post",
    "gyro_obj_stereo_init",
    "gyro_obj_stereo_post",
    "gyro_obj_pose_prior_r_post_rad",
    "gyro_obj_pose_prior_t_post_m",
    "gyro_obj_pose_prior_post",
    "gyro_obj_bias_prior_r_post_rps",
    "gyro_obj_bias_prior_post",
)

COUNT_FIELDS = (
    "gyro_factor_n",
    "gyro_obj_valid",
    "gyro_obj_factor_n",
    "gyro_obj_interior_n",
    "gyro_obj_stereo_n",
)

COST_FIELDS = tuple(
    field
    for field in OBJECTIVE_FIELDS
    if field.startswith("gyro_obj_")
    and field not in COUNT_FIELDS
)


class ProbeError(RuntimeError):
    """The sidecar violates the gyro-objective probe contract."""


def _parse_int(row: dict[str, str], field: str, line: int) -> int:
    try:
        value = int(row[field])
    except (KeyError, ValueError) as exc:
        raise ProbeError(f"line {line}: invalid integer {field}") from exc
    if value < 0:
        raise ProbeError(f"line {line}: negative integer {field}")
    return value


def _parse_float(row: dict[str, str], field: str, line: int) -> float:
    try:
        value = float(row[field])
    except (KeyError, ValueError) as exc:
        raise ProbeError(f"line {line}: invalid number {field}") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ProbeError(f"line {line}: non-finite or negative {field}")
    return value


def _close(actual: float, expected: float) -> bool:
    return abs(actual - expected) <= 1e-9 * max(1.0, abs(expected))


def _quantile(values: list[float], probability: float) -> float:
    if not values:
        raise ProbeError("cannot summarize empty values")
    ordered = sorted(values)
    index = probability * float(len(ordered) - 1)
    left = int(math.floor(index))
    right = int(math.ceil(index))
    if left == right:
        return ordered[left]
    weight = index - float(left)
    return ordered[left] * (1.0 - weight) + ordered[right] * weight


def _summary(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "p10": _quantile(values, 0.10),
        "p25": _quantile(values, 0.25),
        "median": _quantile(values, 0.50),
        "p75": _quantile(values, 0.75),
        "p90": _quantile(values, 0.90),
        "p95": _quantile(values, 0.95),
        "p99": _quantile(values, 0.99),
        "max": max(values),
        "mean": math.fsum(values) / float(len(values)),
    }


def _boundary_comparison(
    rows: list[dict[str, float | int]],
) -> dict[str, object]:
    boundary_to_interior: list[float] = []
    boundary_gt_interior = 0
    boundary_gt_newest = 0
    boundary_share: list[float] = []
    for row in rows:
        interior_mean = float(row["gyro_obj_interior_post"]) / float(
            row["gyro_obj_interior_n"]
        )
        boundary = float(row["gyro_obj_boundary_post"])
        newest = float(row["gyro_obj_new_post"])
        boundary_to_interior.append(boundary / interior_mean)
        boundary_gt_interior += int(boundary > interior_mean)
        boundary_gt_newest += int(boundary > newest)
        gyro_cost = float(row["gyro_obj_gyro_post"])
        if gyro_cost <= 0.0:
            raise ProbeError("gyro posterior cost must be positive")
        boundary_share.append(boundary / gyro_cost)
    return {
        "posterior_cost_to_interior_mean_ratio": _summary(
            boundary_to_interior
        ),
        "posterior_gyro_cost_share": _summary(boundary_share),
        "greater_than_interior_mean_fraction": (
            float(boundary_gt_interior) / float(len(rows))
        ),
        "greater_than_newest_fraction": (
            float(boundary_gt_newest) / float(len(rows))
        ),
    }


def _time_buckets(
    rows: list[dict[str, float | int]], first_timestamp_ns: int
) -> list[dict[str, object]]:
    duration_ns = 10_000_000_000
    grouped: dict[int, list[dict[str, float | int]]] = {}
    for row in rows:
        index = (int(row["timestamp_ns"]) - first_timestamp_ns) // duration_ns
        grouped.setdefault(index, []).append(row)

    result: list[dict[str, object]] = []
    for index, bucket in sorted(grouped.items()):
        comparison = _boundary_comparison(bucket)
        result.append(
            {
                "index": index,
                "start_offset_s": float(index * 10),
                "end_offset_s": float((index + 1) * 10),
                "rows": len(bucket),
                "first_timestamp_ns": int(bucket[0]["timestamp_ns"]),
                "last_timestamp_ns": int(bucket[-1]["timestamp_ns"]),
                "boundary": {
                    "posterior_whitened_norm": _summary(
                        [
                            float(row["gyro_obj_boundary_w_post"])
                            for row in bucket
                        ]
                    ),
                    **comparison,
                },
            }
        )
    return result


def _load_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream)
            header = next(reader, None)
        if header is None:
            raise ProbeError(f"{path}: empty CSV")
        if len(header) != len(set(header)):
            raise ProbeError(f"{path}: duplicate CSV column")
        missing = sorted(set(OBJECTIVE_FIELDS) - set(header))
        if missing:
            raise ProbeError(f"{path}: missing columns: {', '.join(missing)}")
        with path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    return header, rows


def analyze(path: Path) -> dict[str, object]:
    """Validate a state sidecar and return deterministic objective statistics."""
    _, rows = _load_rows(path)
    selected: list[dict[str, float | int]] = []
    previous_timestamp: Optional[int] = None
    previous_state_key: Optional[int] = None
    max_identity_error = 0.0

    for line, row in enumerate(rows, start=2):
        mode = row.get("fusion_mode", "")
        valid = _parse_int(row, "gyro_obj_valid", line)
        if valid not in (0, 1):
            raise ProbeError(f"line {line}: gyro_obj_valid must be 0 or 1")
        if mode != "gyro_visual":
            if valid != 0:
                raise ProbeError(
                    f"line {line}: non-gyro row has valid gyro objective"
                )
            continue
        if valid != 1:
            raise ProbeError(f"line {line}: gyro row has no objective")

        timestamp = _parse_int(row, "timestamp_ns", line)
        state_key = _parse_int(row, "state_key", line)
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ProbeError(f"line {line}: timestamp is not increasing")
        if previous_state_key is not None and state_key <= previous_state_key:
            raise ProbeError(f"line {line}: state key is not increasing")
        previous_timestamp = timestamp
        previous_state_key = state_key

        counts = {field: _parse_int(row, field, line) for field in COUNT_FIELDS}
        if counts["gyro_obj_factor_n"] != counts["gyro_factor_n"]:
            raise ProbeError(f"line {line}: gyro factor counts disagree")
        if counts["gyro_obj_factor_n"] == 0:
            raise ProbeError(f"line {line}: gyro graph has no AHRS factor")
        if (
            counts["gyro_obj_interior_n"] + 1
            != counts["gyro_obj_factor_n"]
        ):
            raise ProbeError(f"line {line}: invalid interior factor count")
        if counts["gyro_obj_stereo_n"] == 0:
            raise ProbeError(f"line {line}: gyro graph has no stereo factor")

        costs = {field: _parse_float(row, field, line) for field in COST_FIELDS}
        identities = (
            (
                costs["gyro_obj_gyro_init"],
                costs["gyro_obj_boundary_init"]
                + costs["gyro_obj_interior_init"],
            ),
            (
                costs["gyro_obj_gyro_post"],
                costs["gyro_obj_boundary_post"]
                + costs["gyro_obj_interior_post"],
            ),
            (
                costs["gyro_obj_boundary_post"],
                0.5 * costs["gyro_obj_boundary_w_post"] ** 2,
            ),
            (
                costs["gyro_obj_new_post"],
                0.5 * costs["gyro_obj_new_w_post"] ** 2,
            ),
        )
        for actual, expected in identities:
            max_identity_error = max(
                max_identity_error, abs(actual - expected)
            )
            if not _close(actual, expected):
                raise ProbeError(f"line {line}: inconsistent objective costs")
        selected.append(
            {
                "timestamp_ns": timestamp,
                "state_key": state_key,
                **counts,
                **costs,
            }
        )

    if not selected:
        raise ProbeError(f"{path}: no gyro_visual objective rows")

    comparative = [
        row
        for row in selected
        if int(row["gyro_obj_interior_n"]) > 0
        and float(row["gyro_obj_interior_post"]) > 0.0
    ]
    if not comparative:
        raise ProbeError(f"{path}: no positive interior objective support")

    first = selected[0]
    last = selected[-1]
    comparison = _boundary_comparison(comparative)
    return {
        "schema_version": 1,
        "source": {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        },
        "support": {
            "gyro_rows": len(selected),
            "comparative_rows": len(comparative),
            "first_timestamp_ns": int(first["timestamp_ns"]),
            "last_timestamp_ns": int(last["timestamp_ns"]),
            "duration_s": (
                int(last["timestamp_ns"]) - int(first["timestamp_ns"])
            )
            * 1e-9,
            "first_state_key": int(first["state_key"]),
            "last_state_key": int(last["state_key"]),
            "factor_count_min": min(
                int(row["gyro_obj_factor_n"]) for row in selected
            ),
            "factor_count_max": max(
                int(row["gyro_obj_factor_n"]) for row in selected
            ),
        },
        "boundary": {
            "posterior_whitened_norm": _summary(
                [float(row["gyro_obj_boundary_w_post"]) for row in selected]
            ),
            "posterior_residual_rad": _summary(
                [
                    float(row["gyro_obj_boundary_r_post_rad"])
                    for row in selected
                ]
            ),
            "posterior_cost": _summary(
                [float(row["gyro_obj_boundary_post"]) for row in selected]
            ),
            **comparison,
        },
        "newest": {
            "posterior_whitened_norm": _summary(
                [float(row["gyro_obj_new_w_post"]) for row in selected]
            ),
            "posterior_residual_rad": _summary(
                [float(row["gyro_obj_new_r_post_rad"]) for row in selected]
            ),
            "posterior_cost": _summary(
                [float(row["gyro_obj_new_post"]) for row in selected]
            ),
        },
        "priors": {
            "pose_posterior_cost": _summary(
                [
                    float(row["gyro_obj_pose_prior_post"])
                    for row in selected
                ]
            ),
            "bias_posterior_cost": _summary(
                [
                    float(row["gyro_obj_bias_prior_post"])
                    for row in selected
                ]
            ),
        },
        "identities": {"max_abs_cost_error": max_identity_error},
        "time_buckets": _time_buckets(
            comparative, int(first["timestamp_ns"])
        ),
    }


def _write_json(value: dict[str, object], output: Optional[Path]) -> None:
    text = json.dumps(value, indent=2, sort_keys=True) + "\n"
    if output is None:
        sys.stdout.write(text)
        return
    try:
        output.write_text(text, encoding="utf-8")
    except OSError as exc:
        raise ProbeError(f"cannot write {output}: {exc}") from exc


def main(argv: Optional[list[str]] = None, stream: TextIO = sys.stderr) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state_csv", type=Path)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args(argv)
    try:
        _write_json(analyze(arguments.state_csv), arguments.output)
    except ProbeError as exc:
        print(f"gyro graph objective probe: {exc}", file=stream)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

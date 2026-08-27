#!/usr/bin/env python3
"""Validate and summarize an M4.4 fixed-lag shadow CSV sidecar.

The sidecar is structural evidence only.  It measures bounded state, factor
ownership, reset lifecycle, and newest-state deltas; it does not compute a
shadow ATE or claim that the batch-driven shadow is a production handoff.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO


COLUMNS = (
    "timestamp_ns",
    "state_key",
    "fusion_mode",
    "active",
    "update_ok",
    "reset",
    "reset_reason",
    "current_epoch",
    "cutoff_epoch",
    "batch_window_n",
    "smoother_pose_n",
    "smoother_landmark_n",
    "bias_present",
    "bias_fixed",
    "bias_delta_norm",
    "user_factor_n",
    "missing_owned_slot_n",
    "timestamp_without_value_n",
    "marginalized_pose_n",
    "retired_landmark_n",
    "new_landmark_generation_n",
    "shadow_p_x",
    "shadow_p_y",
    "shadow_p_z",
    "shadow_q_x",
    "shadow_q_y",
    "shadow_q_z",
    "shadow_q_w",
    "newest_rot_delta_rad",
    "newest_trans_delta_m",
)


class ProbeError(RuntimeError):
    """The sidecar violates its schema or fixed-lag lifecycle contract."""


@dataclass(frozen=True)
class Row:
    timestamp_ns: int
    state_key: int
    fusion_mode: str
    active: bool
    update_ok: bool
    reset: bool
    reset_reason: str
    current_epoch: int
    cutoff_epoch: int
    batch_window_n: int
    smoother_pose_n: int
    smoother_landmark_n: int
    bias_present: bool
    bias_fixed: bool
    bias_delta_norm: float
    user_factor_n: int
    missing_owned_slot_n: int
    timestamp_without_value_n: int
    marginalized_pose_n: int
    retired_landmark_n: int
    new_landmark_generation_n: int
    shadow_p: tuple[float, float, float]
    shadow_q: tuple[float, float, float, float]
    newest_rot_delta_rad: float
    newest_trans_delta_m: float


def _parse_int(raw: dict[str, str], name: str, line: int) -> int:
    try:
        value = int(raw[name])
    except (KeyError, ValueError) as exc:
        raise ProbeError(f"line {line}: {name} is not an integer") from exc
    if value < 0:
        raise ProbeError(f"line {line}: {name} is negative")
    return value


def _parse_bool(raw: dict[str, str], name: str, line: int) -> bool:
    value = _parse_int(raw, name, line)
    if value not in (0, 1):
        raise ProbeError(f"line {line}: {name} is not 0 or 1")
    return value == 1


def _parse_float(raw: dict[str, str], name: str, line: int) -> float:
    try:
        value = float(raw[name])
    except (KeyError, ValueError) as exc:
        raise ProbeError(f"line {line}: {name} is not numeric") from exc
    if not math.isfinite(value):
        raise ProbeError(f"line {line}: {name} is non-finite")
    return value


def _parse_row(raw: dict[str, str], line: int) -> Row:
    if None in raw:
        raise ProbeError(f"line {line}: row has extra columns")
    return Row(
        timestamp_ns=_parse_int(raw, "timestamp_ns", line),
        state_key=_parse_int(raw, "state_key", line),
        fusion_mode=raw["fusion_mode"],
        active=_parse_bool(raw, "active", line),
        update_ok=_parse_bool(raw, "update_ok", line),
        reset=_parse_bool(raw, "reset", line),
        reset_reason=raw["reset_reason"],
        current_epoch=_parse_int(raw, "current_epoch", line),
        cutoff_epoch=_parse_int(raw, "cutoff_epoch", line),
        batch_window_n=_parse_int(raw, "batch_window_n", line),
        smoother_pose_n=_parse_int(raw, "smoother_pose_n", line),
        smoother_landmark_n=_parse_int(raw, "smoother_landmark_n", line),
        bias_present=_parse_bool(raw, "bias_present", line),
        bias_fixed=_parse_bool(raw, "bias_fixed", line),
        bias_delta_norm=_parse_float(raw, "bias_delta_norm", line),
        user_factor_n=_parse_int(raw, "user_factor_n", line),
        missing_owned_slot_n=_parse_int(raw, "missing_owned_slot_n", line),
        timestamp_without_value_n=_parse_int(
            raw, "timestamp_without_value_n", line
        ),
        marginalized_pose_n=_parse_int(raw, "marginalized_pose_n", line),
        retired_landmark_n=_parse_int(raw, "retired_landmark_n", line),
        new_landmark_generation_n=_parse_int(
            raw, "new_landmark_generation_n", line
        ),
        shadow_p=tuple(
            _parse_float(raw, name, line)
            for name in ("shadow_p_x", "shadow_p_y", "shadow_p_z")
        ),
        shadow_q=tuple(
            _parse_float(raw, name, line)
            for name in ("shadow_q_x", "shadow_q_y", "shadow_q_z", "shadow_q_w")
        ),
        newest_rot_delta_rad=_parse_float(raw, "newest_rot_delta_rad", line),
        newest_trans_delta_m=_parse_float(raw, "newest_trans_delta_m", line),
    )


def _is_zero(value: float) -> bool:
    return value == 0.0


def _validate_inactive(row: Row, line: int) -> None:
    if row.fusion_mode != "vision_only":
        raise ProbeError(f"line {line}: inactive row is not vision_only")
    integer_state = (
        row.current_epoch,
        row.cutoff_epoch,
        row.batch_window_n,
        row.smoother_pose_n,
        row.smoother_landmark_n,
        row.user_factor_n,
        row.missing_owned_slot_n,
        row.timestamp_without_value_n,
        row.marginalized_pose_n,
        row.retired_landmark_n,
        row.new_landmark_generation_n,
    )
    if (
        row.reset
        or row.reset_reason != "none"
        or any(integer_state)
        or row.bias_present
        or row.bias_fixed
        or not _is_zero(row.bias_delta_norm)
        or any(not _is_zero(value) for value in row.shadow_p)
        or row.shadow_q != (0.0, 0.0, 0.0, 1.0)
        or not _is_zero(row.newest_rot_delta_rad)
        or not _is_zero(row.newest_trans_delta_m)
    ):
        raise ProbeError(f"line {line}: inactive row contains shadow state")


def _validate_active(row: Row, line: int, first_active: bool) -> None:
    if row.fusion_mode != "gyro_visual":
        raise ProbeError(f"line {line}: active row is not gyro_visual")
    if row.current_epoch != row.state_key:
        raise ProbeError(f"line {line}: current_epoch differs from state_key")
    if row.cutoff_epoch > row.current_epoch:
        raise ProbeError(f"line {line}: cutoff_epoch exceeds current_epoch")
    if not 1 <= row.batch_window_n <= 10:
        raise ProbeError(f"line {line}: batch window is outside [1, 10]")
    if row.smoother_pose_n != row.batch_window_n:
        raise ProbeError(f"line {line}: shadow pose set differs from batch window")
    if row.user_factor_n == 0:
        raise ProbeError(f"line {line}: active graph has no user factors")
    if not row.bias_present:
        raise ProbeError(f"line {line}: shared bias is absent")
    if row.missing_owned_slot_n != 0 or row.timestamp_without_value_n != 0:
        raise ProbeError(f"line {line}: factor/timestamp lifecycle invariant failed")
    if row.reset != (row.reset_reason != "none"):
        raise ProbeError(f"line {line}: reset flag and reason disagree")
    if row.reset_reason not in ("none", "bootstrap", "segment"):
        raise ProbeError(f"line {line}: invalid reset_reason")
    if first_active and row.reset_reason != "bootstrap":
        raise ProbeError(f"line {line}: first active row is not bootstrap")
    if not first_active and row.reset_reason == "bootstrap":
        raise ProbeError(f"line {line}: bootstrap repeats")
    if (
        row.bias_delta_norm < 0.0
        or row.newest_rot_delta_rad < 0.0
        or row.newest_trans_delta_m < 0.0
    ):
        raise ProbeError(f"line {line}: delta is negative")
    quaternion_norm = math.sqrt(sum(value * value for value in row.shadow_q))
    if abs(quaternion_norm - 1.0) > 1e-9:
        raise ProbeError(f"line {line}: shadow quaternion is not unit length")


def load(path: Path) -> tuple[Row, ...]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"failed to open {path}: {exc}") from exc
    with input_file:
        reader = csv.DictReader(input_file)
        if tuple(reader.fieldnames or ()) != COLUMNS:
            raise ProbeError(f"{path}: CSV schema does not match fixed-lag sidecar")
        rows: list[Row] = []
        previous_timestamp: int | None = None
        previous_state_key: int | None = None
        active_started = False
        for line, raw in enumerate(reader, start=2):
            row = _parse_row(raw, line)
            if not row.update_ok:
                raise ProbeError(f"line {line}: update_ok is false")
            if previous_timestamp is not None and row.timestamp_ns <= previous_timestamp:
                raise ProbeError(f"line {line}: timestamp is not strictly increasing")
            if previous_state_key is not None and row.state_key <= previous_state_key:
                raise ProbeError(f"line {line}: state_key is not strictly increasing")
            if row.active:
                _validate_active(row, line, first_active=not active_started)
                active_started = True
            else:
                if active_started:
                    raise ProbeError(f"line {line}: inactive row follows active shadow")
                _validate_inactive(row, line)
            rows.append(row)
            previous_timestamp = row.timestamp_ns
            previous_state_key = row.state_key
    if not rows:
        raise ProbeError(f"{path}: sidecar has no rows")
    if not active_started:
        raise ProbeError(f"{path}: sidecar has no active fixed-lag support")
    return tuple(rows)


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = quantile * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _stats(values: list[float]) -> dict[str, float]:
    return {
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "max": max(values, default=0.0),
    }


def _buckets(rows: tuple[Row, ...], bucket_s: float) -> list[dict[str, object]]:
    start_ns = rows[0].timestamp_ns
    bucket_ns = bucket_s * 1e9
    last_index = int((rows[-1].timestamp_ns - start_ns) // bucket_ns)
    grouped: list[list[Row]] = [[] for _ in range(last_index + 1)]
    for row in rows:
        if not row.active:
            continue
        index = int((row.timestamp_ns - start_ns) // bucket_ns)
        grouped[index].append(row)
    output: list[dict[str, object]] = []
    for index, group in enumerate(grouped):
        output.append(
            {
                "start_s": index * bucket_s,
                "end_s": (index + 1) * bucket_s,
                "active_row_count": len(group),
                "rotation_delta_rad": _stats(
                    [row.newest_rot_delta_rad for row in group]
                ),
                "translation_delta_m": _stats(
                    [row.newest_trans_delta_m for row in group]
                ),
            }
        )
    return output


def analyze(path: Path, bucket_s: float = 10.0) -> dict[str, object]:
    if not math.isfinite(bucket_s) or bucket_s <= 0.0:
        raise ProbeError("bucket_s must be finite and positive")
    rows = load(path)
    active = [row for row in rows if row.active]
    fixed_rows = [row for row in active if row.bias_fixed]
    reset_counts = {
        "bootstrap": sum(row.reset_reason == "bootstrap" for row in active),
        "segment": sum(row.reset_reason == "segment" for row in active),
    }
    if reset_counts["bootstrap"] != 1:
        raise ProbeError("sidecar must contain exactly one bootstrap reset")

    return {
        "schema_version": 1,
        "source": str(path),
        "bucket_s": bucket_s,
        "row_count": len(rows),
        "active_row_count": len(active),
        "inactive_row_count": len(rows) - len(active),
        "first_timestamp_ns": rows[0].timestamp_ns,
        "last_timestamp_ns": rows[-1].timestamp_ns,
        "first_active_state_key": active[0].state_key,
        "last_active_state_key": active[-1].state_key,
        "reset_counts": reset_counts,
        "max_batch_window_n": max(row.batch_window_n for row in active),
        "max_smoother_pose_n": max(row.smoother_pose_n for row in active),
        "max_smoother_landmark_n": max(
            row.smoother_landmark_n for row in active
        ),
        "max_user_factor_n": max(row.user_factor_n for row in active),
        "marginalized_pose_total": sum(row.marginalized_pose_n for row in active),
        "retired_landmark_total": sum(row.retired_landmark_n for row in active),
        "new_landmark_generation_total": sum(
            row.new_landmark_generation_n for row in active
        ),
        "bias_fixed_row_count": len(fixed_rows),
        "first_bias_fixed_state_key": fixed_rows[0].state_key if fixed_rows else None,
        "missing_owned_slot_total": sum(
            row.missing_owned_slot_n for row in active
        ),
        "timestamp_without_value_total": sum(
            row.timestamp_without_value_n for row in active
        ),
        "bias_delta_norm": _stats([row.bias_delta_norm for row in active]),
        "rotation_delta_rad": _stats(
            [row.newest_rot_delta_rad for row in active]
        ),
        "translation_delta_m": _stats(
            [row.newest_trans_delta_m for row in active]
        ),
        "buckets": _buckets(rows, bucket_s),
        "lifecycle_gate_pass": True,
    }


def _write_json(result: dict[str, object], output: TextIO) -> None:
    json.dump(result, output, indent=2, sort_keys=True)
    output.write("\n")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecar_csv", type=Path)
    parser.add_argument("--bucket-s", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        result = analyze(arguments.sidecar_csv, arguments.bucket_s)
        if arguments.output is None:
            _write_json(result, sys.stdout)
        else:
            with arguments.output.open("w", encoding="utf-8") as output:
                _write_json(result, output)
    except (OSError, ProbeError) as exc:
        print(f"fixed-lag shadow probe: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

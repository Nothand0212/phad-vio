#!/usr/bin/env python3
"""Align estimator bias diagnostics with EuRoC ground truth.

The probe is read-only with respect to the estimator.  It accepts only
successful estimator rows, joins the nearest ground-truth state under an
explicit timestamp tolerance, and reports fixed-size buckets using only the
Python standard library.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO


Vector3 = tuple[float, float, float]

DIAG_FIELDS = (
    "timestamp_ns",
    "status",
    "window_size",
    "prior_key",
    "is_keyframe",
    "bias_gyro_x",
    "bias_gyro_y",
    "bias_gyro_z",
    "bias_acc_x",
    "bias_acc_y",
    "bias_acc_z",
)

GT_FIELDS = (
    "#timestamp",
    "b_w_RS_S_x [rad s^-1]",
    "b_w_RS_S_y [rad s^-1]",
    "b_w_RS_S_z [rad s^-1]",
    "b_a_RS_S_x [m s^-2]",
    "b_a_RS_S_y [m s^-2]",
    "b_a_RS_S_z [m s^-2]",
)

OUTPUT_FIELDS = (
    "bucket",
    "start_index",
    "end_index",
    "start_ns",
    "end_ns",
    "frames",
    "keyframes",
    "prior_advances",
    "window_min",
    "window_max",
    "acc_err_rms_mps2",
    "acc_err_mean_x_mps2",
    "acc_err_mean_y_mps2",
    "acc_err_mean_z_mps2",
    "acc_err_max_mps2",
    "acc_est_drift_mps2",
    "gyro_err_rms_rps",
    "gyro_err_mean_x_rps",
    "gyro_err_mean_y_rps",
    "gyro_err_mean_z_rps",
    "gyro_err_max_rps",
    "gyro_est_drift_rps",
)


class ProbeError(RuntimeError):
    """A probe input violates its schema or timestamp contract."""


@dataclass(frozen=True)
class DiagBias:
    timestamp_ns: int
    window_size: int
    prior_key: int
    is_keyframe: bool
    gyro: Vector3
    acc: Vector3


@dataclass(frozen=True)
class GroundtruthBias:
    timestamp_ns: int
    gyro: Vector3
    acc: Vector3


@dataclass(frozen=True)
class BiasSample:
    timestamp_ns: int
    window_size: int
    prior_key: int
    is_keyframe: bool
    est_gyro: Vector3
    est_acc: Vector3
    gt_gyro: Vector3
    gt_acc: Vector3


@dataclass(frozen=True)
class BiasBucket:
    bucket: int
    start_index: int
    end_index: int
    start_ns: int
    end_ns: int
    frames: int
    keyframes: int
    prior_advances: int
    window_min: int
    window_max: int
    acc_err_rms_mps2: float
    acc_err_mean_x_mps2: float
    acc_err_mean_y_mps2: float
    acc_err_mean_z_mps2: float
    acc_err_max_mps2: float
    acc_est_drift_mps2: float
    gyro_err_rms_rps: float
    gyro_err_mean_x_rps: float
    gyro_err_mean_y_rps: float
    gyro_err_mean_z_rps: float
    gyro_err_max_rps: float
    gyro_est_drift_rps: float


def _integer(value: str, field: str, path: Path, line: int) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field}") from exc


def _finite(value: str, field: str, path: Path, line: int) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field}") from exc
    if not math.isfinite(result):
        raise ProbeError(f"{path}:{line}: {field} must be finite")
    return result


def _vector(
    row: dict[str, str], fields: tuple[str, str, str], path: Path, line: int
) -> Vector3:
    return tuple(_finite(row[field], field, path, line) for field in fields)  # type: ignore[return-value]


def _open_csv(path: Path) -> tuple[TextIO, csv.DictReader]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    reader = csv.DictReader(input_file, skipinitialspace=True)
    return input_file, reader


def _require_fields(
    reader: csv.DictReader, required: tuple[str, ...], path: Path
) -> None:
    fields = set(reader.fieldnames or ())
    missing = [field for field in required if field not in fields]
    if missing:
        raise ProbeError(f"{path}: missing columns: {', '.join(missing)}")


def load_diag(path: Path) -> tuple[DiagBias, ...]:
    input_file, reader = _open_csv(path)
    rows: list[DiagBias] = []
    previous: Optional[int] = None
    with input_file:
        _require_fields(reader, DIAG_FIELDS, path)
        for index, raw in enumerate(reader, start=2):
            if None in raw or any(value is None for value in raw.values()):
                raise ProbeError(f"{path}:{index}: column count mismatch")
            timestamp_ns = _integer(
                raw["timestamp_ns"], "timestamp_ns", path, index
            )
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{index}: timestamps are not strictly increasing"
                )
            previous = timestamp_ns
            if raw["status"] != "ok":
                continue
            is_keyframe = _integer(
                raw["is_keyframe"], "is_keyframe", path, index
            )
            if is_keyframe not in (0, 1):
                raise ProbeError(
                    f"{path}:{index}: is_keyframe must be 0 or 1"
                )
            rows.append(
                DiagBias(
                    timestamp_ns=timestamp_ns,
                    window_size=_integer(
                        raw["window_size"], "window_size", path, index
                    ),
                    prior_key=_integer(
                        raw["prior_key"], "prior_key", path, index
                    ),
                    is_keyframe=bool(is_keyframe),
                    gyro=_vector(
                        raw,
                        ("bias_gyro_x", "bias_gyro_y", "bias_gyro_z"),
                        path,
                        index,
                    ),
                    acc=_vector(
                        raw,
                        ("bias_acc_x", "bias_acc_y", "bias_acc_z"),
                        path,
                        index,
                    ),
                )
            )
    if not rows:
        raise ProbeError(f"{path}: no status=ok rows")
    return tuple(rows)


def load_groundtruth(path: Path) -> tuple[GroundtruthBias, ...]:
    input_file, reader = _open_csv(path)
    rows: list[GroundtruthBias] = []
    previous: Optional[int] = None
    with input_file:
        _require_fields(reader, GT_FIELDS, path)
        for index, raw in enumerate(reader, start=2):
            if None in raw or any(value is None for value in raw.values()):
                raise ProbeError(f"{path}:{index}: column count mismatch")
            timestamp_ns = _integer(
                raw["#timestamp"], "#timestamp", path, index
            )
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{index}: timestamps are not strictly increasing"
                )
            previous = timestamp_ns
            rows.append(
                GroundtruthBias(
                    timestamp_ns=timestamp_ns,
                    gyro=_vector(raw, GT_FIELDS[1:4], path, index),
                    acc=_vector(raw, GT_FIELDS[4:7], path, index),
                )
            )
    if not rows:
        raise ProbeError(f"{path}: no groundtruth rows")
    return tuple(rows)


def join_bias(
    diag: tuple[DiagBias, ...],
    groundtruth: tuple[GroundtruthBias, ...],
    max_dt_ns: int,
) -> tuple[BiasSample, ...]:
    if max_dt_ns < 0:
        raise ProbeError("max_dt_ns must be non-negative")
    gt_times = [row.timestamp_ns for row in groundtruth]
    samples: list[BiasSample] = []
    for row in diag:
        # Match phad_traj_eval's support contract: poses strictly outside the
        # ground-truth span are reported as dropped, not treated as a nearest-
        # timestamp violation inside the span.
        if row.timestamp_ns < gt_times[0] or row.timestamp_ns > gt_times[-1]:
            continue
        insertion = bisect.bisect_left(gt_times, row.timestamp_ns)
        candidates = []
        if insertion < len(groundtruth):
            candidates.append(groundtruth[insertion])
        if insertion > 0:
            candidates.append(groundtruth[insertion - 1])
        nearest = min(
            candidates,
            key=lambda candidate: (
                abs(candidate.timestamp_ns - row.timestamp_ns),
                candidate.timestamp_ns,
            ),
        )
        dt_ns = abs(nearest.timestamp_ns - row.timestamp_ns)
        if dt_ns > max_dt_ns:
            raise ProbeError(
                f"{row.timestamp_ns}: no groundtruth within {max_dt_ns} ns"
            )
        samples.append(
            BiasSample(
                timestamp_ns=row.timestamp_ns,
                window_size=row.window_size,
                prior_key=row.prior_key,
                is_keyframe=row.is_keyframe,
                est_gyro=row.gyro,
                est_acc=row.acc,
                gt_gyro=nearest.gyro,
                gt_acc=nearest.acc,
            )
        )
    return tuple(samples)


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _mean_component(vectors: list[Vector3], component: int) -> float:
    return sum(vector[component] for vector in vectors) / len(vectors)


def _rms_norm(vectors: list[Vector3]) -> float:
    return math.sqrt(
        sum(sum(value * value for value in vector) for vector in vectors)
        / len(vectors)
    )


def summarize(
    samples: tuple[BiasSample, ...], bucket_frames: int
) -> tuple[BiasBucket, ...]:
    if not samples:
        raise ProbeError("cannot summarize zero samples")
    if bucket_frames < 1:
        raise ProbeError("bucket_frames must be positive")
    result: list[BiasBucket] = []
    for start in range(0, len(samples), bucket_frames):
        chunk = samples[start : start + bucket_frames]
        acc_errors = [
            _subtract(sample.est_acc, sample.gt_acc) for sample in chunk
        ]
        gyro_errors = [
            _subtract(sample.est_gyro, sample.gt_gyro) for sample in chunk
        ]
        prior_advances = sum(
            1
            for index in range(start, start + len(chunk))
            if index > 0
            and samples[index].prior_key != samples[index - 1].prior_key
        )
        result.append(
            BiasBucket(
                bucket=len(result),
                start_index=start,
                end_index=start + len(chunk),
                start_ns=chunk[0].timestamp_ns,
                end_ns=chunk[-1].timestamp_ns,
                frames=len(chunk),
                keyframes=sum(sample.is_keyframe for sample in chunk),
                prior_advances=prior_advances,
                window_min=min(sample.window_size for sample in chunk),
                window_max=max(sample.window_size for sample in chunk),
                acc_err_rms_mps2=_rms_norm(acc_errors),
                acc_err_mean_x_mps2=_mean_component(acc_errors, 0),
                acc_err_mean_y_mps2=_mean_component(acc_errors, 1),
                acc_err_mean_z_mps2=_mean_component(acc_errors, 2),
                acc_err_max_mps2=max(_norm(error) for error in acc_errors),
                acc_est_drift_mps2=_norm(
                    _subtract(chunk[-1].est_acc, chunk[0].est_acc)
                ),
                gyro_err_rms_rps=_rms_norm(gyro_errors),
                gyro_err_mean_x_rps=_mean_component(gyro_errors, 0),
                gyro_err_mean_y_rps=_mean_component(gyro_errors, 1),
                gyro_err_mean_z_rps=_mean_component(gyro_errors, 2),
                gyro_err_max_rps=max(_norm(error) for error in gyro_errors),
                gyro_est_drift_rps=_norm(
                    _subtract(chunk[-1].est_gyro, chunk[0].est_gyro)
                ),
            )
        )
    return tuple(result)


def render_csv(buckets: tuple[BiasBucket, ...], output: TextIO) -> None:
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(OUTPUT_FIELDS)
    for bucket in buckets:
        values = []
        for field in OUTPUT_FIELDS:
            value = getattr(bucket, field)
            values.append(f"{value:.17g}" if isinstance(value, float) else value)
        writer.writerow(values)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument("--start-ns", type=int)
    parser.add_argument("--end-ns", type=int)
    parser.add_argument("--bucket-frames", type=int, default=100)
    parser.add_argument("--max-dt-ms", type=float, default=2.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if args.bucket_frames < 1:
        parser.error("--bucket-frames must be positive")
    if not math.isfinite(args.max_dt_ms) or args.max_dt_ms < 0.0:
        parser.error("--max-dt-ms must be finite and non-negative")
    if (
        args.start_ns is not None
        and args.end_ns is not None
        and args.start_ns > args.end_ns
    ):
        parser.error("--start-ns must not exceed --end-ns")

    try:
        diag = load_diag(args.run_dir / "diag.csv")
        diag = tuple(
            row
            for row in diag
            if (args.start_ns is None or row.timestamp_ns >= args.start_ns)
            and (args.end_ns is None or row.timestamp_ns <= args.end_ns)
        )
        if not diag:
            raise ProbeError("selected interval has no status=ok rows")
        groundtruth = load_groundtruth(
            args.euroc_root
            / "mav0"
            / "state_groundtruth_estimate0"
            / "data.csv"
        )
        samples = join_bias(
            diag,
            groundtruth,
            max_dt_ns=int(args.max_dt_ms * 1_000_000.0),
        )
        dropped_outside_gt = len(diag) - len(samples)
        buckets = summarize(samples, args.bucket_frames)
        if args.output is None:
            print(
                f"samples={len(samples)} dropped_outside_gt="
                f"{dropped_outside_gt}",
                file=sys.stderr,
            )
            render_csv(buckets, sys.stdout)
        else:
            try:
                with args.output.open(
                    "w", encoding="utf-8", newline=""
                ) as output:
                    render_csv(buckets, output)
            except OSError as exc:
                raise ProbeError(f"cannot write {args.output}: {exc}") from exc
            print(
                f"samples={len(samples)} buckets={len(buckets)} "
                f"dropped_outside_gt={dropped_outside_gt} "
                f"output={args.output}"
            )
    except ProbeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

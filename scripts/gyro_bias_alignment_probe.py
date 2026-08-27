#!/usr/bin/env python3
"""Estimate gyro bias from pure-visual rotations and raw EuRoC IMU.

The probe is read-only.  It uses accepted timestamps and the static gyro-bias
seed from a gyro-enabled run, but takes relative rotations from a separate VO
trajectory so the calibration evidence is not influenced by gyro factors.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable, Optional, TextIO


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # w, x, y, z

OUTPUT_FIELDS = (
    "pair_count",
    "duration_s",
    "start_ns",
    "end_ns",
    "bias_x_rps",
    "bias_y_rps",
    "bias_z_rps",
    "delta_seed_x_rps",
    "delta_seed_y_rps",
    "delta_seed_z_rps",
    "delta_seed_norm_rps",
    "ref_bias_x_rps",
    "ref_bias_y_rps",
    "ref_bias_z_rps",
    "ref_error_x_rps",
    "ref_error_y_rps",
    "ref_error_z_rps",
    "ref_error_norm_rps",
    "rotation_residual_rms_rad",
    "rotation_residual_max_rad",
    "residual_mean_x_rad",
    "residual_mean_y_rad",
    "residual_mean_z_rad",
    "residual_std_x_rad",
    "residual_std_y_rad",
    "residual_std_z_rad",
    "residual_lag1_x",
    "residual_lag1_y",
    "residual_lag1_z",
    "bias_sigma_iid_x_rps",
    "bias_sigma_iid_y_rps",
    "bias_sigma_iid_z_rps",
    "normal_min_eigenvalue_s2",
    "normal_max_eigenvalue_s2",
    "normal_condition",
    "iterations",
)


class ProbeError(RuntimeError):
    """A probe input violates its schema or timestamp contract."""


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    gyro: Vector3


@dataclass(frozen=True)
class RotationPair:
    t_i_ns: int
    t_j_ns: int
    imu: tuple[ImuSample, ...]
    visual_delta: Quaternion


@dataclass(frozen=True)
class BiasEstimate:
    bias: Vector3
    rotation_residual_rms_rad: float
    rotation_residual_max_rad: float
    residual_mean: Vector3
    residual_std: Vector3
    residual_lag1: Vector3
    bias_sigma_iid: Vector3
    normal_min_eigenvalue: float
    normal_max_eigenvalue: float
    normal_condition: float
    iterations: int


@dataclass(frozen=True)
class ReferenceBias:
    timestamp_ns: int
    bias: Vector3


@dataclass(frozen=True)
class ProbeRow:
    pair_count: int
    duration_s: float
    start_ns: int
    end_ns: int
    estimate: BiasEstimate
    seed: Vector3
    reference: Vector3


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def _residual_stats(
    residuals: tuple[Vector3, ...],
) -> tuple[Vector3, Vector3, Vector3]:
    count = len(residuals)
    mean = tuple(
        sum(residual[axis] for residual in residuals) / float(count)
        for axis in range(3)
    )
    denominator = float(max(1, count - 1))
    std = tuple(
        math.sqrt(
            sum(
                (residual[axis] - mean[axis]) ** 2
                for residual in residuals
            )
            / denominator
        )
        for axis in range(3)
    )
    lag1: list[float] = []
    for axis in range(3):
        if count < 2:
            lag1.append(0.0)
            continue
        left = tuple(
            residuals[index - 1][axis] - mean[axis]
            for index in range(1, count)
        )
        right = tuple(
            residuals[index][axis] - mean[axis]
            for index in range(1, count)
        )
        norm_product = math.sqrt(
            sum(value * value for value in left)
            * sum(value * value for value in right)
        )
        lag1.append(
            0.0
            if norm_product <= 1e-30
            else sum(a * b for a, b in zip(left, right)) / norm_product
        )
    return mean, std, tuple(lag1)  # type: ignore[return-value]


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    w_l, x_l, y_l, z_l = left
    w_r, x_r, y_r, z_r = right
    return (
        w_l * w_r - x_l * x_r - y_l * y_r - z_l * z_r,
        w_l * x_r + x_l * w_r + y_l * z_r - z_l * y_r,
        w_l * y_r - x_l * z_r + y_l * w_r + z_l * x_r,
        w_l * z_r + x_l * y_r - y_l * x_r + z_l * w_r,
    )


def _quaternion_conjugate(value: Quaternion) -> Quaternion:
    return (value[0], -value[1], -value[2], -value[3])


def _quaternion_normalize(value: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in value))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid quaternion")
    return tuple(component / norm for component in value)  # type: ignore[return-value]


def _quaternion_exp(rotation_vector: Vector3) -> Quaternion:
    angle = _norm(rotation_vector)
    if angle < 1e-14:
        return _quaternion_normalize(
            (1.0, 0.5 * rotation_vector[0], 0.5 * rotation_vector[1],
             0.5 * rotation_vector[2])
        )
    scale = math.sin(0.5 * angle) / angle
    return (
        math.cos(0.5 * angle),
        scale * rotation_vector[0],
        scale * rotation_vector[1],
        scale * rotation_vector[2],
    )


def _quaternion_log(value: Quaternion) -> Vector3:
    normalized = _quaternion_normalize(value)
    if normalized[0] < 0.0:
        normalized = tuple(-component for component in normalized)  # type: ignore[assignment]
    vector = (normalized[1], normalized[2], normalized[3])
    sine = _norm(vector)
    if sine < 1e-14:
        return _scale(vector, 2.0)
    angle = 2.0 * math.atan2(sine, max(0.0, normalized[0]))
    return _scale(vector, angle / sine)


def integrate_rotation(samples: tuple[ImuSample, ...], bias: Vector3) -> Quaternion:
    if len(samples) < 2:
        raise ValueError("rotation pair needs at least two IMU samples")
    rotation: Quaternion = (1.0, 0.0, 0.0, 0.0)
    for left, right in zip(samples, samples[1:]):
        delta_ns = right.timestamp_ns - left.timestamp_ns
        if delta_ns <= 0:
            raise ValueError("IMU timestamps must be strictly increasing")
        delta_s = float(delta_ns) * 1e-9
        omega = _subtract(left.gyro, bias)
        rotation = _quaternion_normalize(
            _quaternion_multiply(
                rotation, _quaternion_exp(_scale(omega, delta_s))
            )
        )
    return rotation


def _rotation_residual(pair: RotationPair, bias: Vector3) -> Vector3:
    predicted = integrate_rotation(pair.imu, bias)
    error = _quaternion_multiply(
        _quaternion_conjugate(predicted), pair.visual_delta
    )
    return _quaternion_log(error)


def _solve_3x3(matrix: list[list[float]], vector: Vector3) -> Vector3:
    augmented = [matrix[row][:] + [vector[row]] for row in range(3)]
    for pivot in range(3):
        selected = max(
            range(pivot, 3), key=lambda row: abs(augmented[row][pivot])
        )
        augmented[pivot], augmented[selected] = (
            augmented[selected],
            augmented[pivot],
        )
        diagonal = augmented[pivot][pivot]
        if abs(diagonal) <= 1e-18:
            raise ValueError("gyro bias normal matrix is singular")
        for column in range(pivot, 4):
            augmented[pivot][column] /= diagonal
        for row in range(3):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            for column in range(pivot, 4):
                augmented[row][column] -= factor * augmented[pivot][column]
    return tuple(augmented[row][3] for row in range(3))  # type: ignore[return-value]


def _symmetric_eigenvalues(matrix: list[list[float]]) -> tuple[float, float, float]:
    value = [row[:] for row in matrix]
    for _ in range(32):
        row, column = max(
            ((0, 1), (0, 2), (1, 2)),
            key=lambda index: abs(value[index[0]][index[1]]),
        )
        off_diagonal = value[row][column]
        if abs(off_diagonal) <= 1e-18:
            break
        angle = 0.5 * math.atan2(
            2.0 * off_diagonal, value[column][column] - value[row][row]
        )
        cosine = math.cos(angle)
        sine = math.sin(angle)
        for index in range(3):
            if index in (row, column):
                continue
            left = value[index][row]
            right = value[index][column]
            value[index][row] = value[row][index] = cosine * left - sine * right
            value[index][column] = value[column][index] = (
                sine * left + cosine * right
            )
        diagonal_row = value[row][row]
        diagonal_column = value[column][column]
        value[row][row] = (
            cosine * cosine * diagonal_row
            - 2.0 * sine * cosine * off_diagonal
            + sine * sine * diagonal_column
        )
        value[column][column] = (
            sine * sine * diagonal_row
            + 2.0 * sine * cosine * off_diagonal
            + cosine * cosine * diagonal_column
        )
        value[row][column] = value[column][row] = 0.0
    return tuple(sorted(value[index][index] for index in range(3)))  # type: ignore[return-value]


def _linearize(
    pairs: tuple[RotationPair, ...], bias: Vector3
) -> tuple[list[list[float]], Vector3, tuple[Vector3, ...]]:
    epsilon = 1e-5
    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    gradient = [0.0, 0.0, 0.0]
    residuals: list[Vector3] = []
    for pair in pairs:
        residual = _rotation_residual(pair, bias)
        residuals.append(residual)
        columns: list[Vector3] = []
        for axis in range(3):
            plus = list(bias)
            minus = list(bias)
            plus[axis] += epsilon
            minus[axis] -= epsilon
            residual_plus = _rotation_residual(pair, tuple(plus))
            residual_minus = _rotation_residual(pair, tuple(minus))
            columns.append(
                _scale(_subtract(residual_plus, residual_minus), 0.5 / epsilon)
            )
        for row in range(3):
            gradient[row] += sum(
                columns[row][axis] * residual[axis] for axis in range(3)
            )
            for column in range(3):
                normal[row][column] += sum(
                    columns[row][axis] * columns[column][axis]
                    for axis in range(3)
                )
    return normal, tuple(gradient), tuple(residuals)  # type: ignore[arg-type,return-value]


def estimate_bias(
    pairs: tuple[RotationPair, ...], seed: Vector3
) -> BiasEstimate:
    if not pairs:
        raise ValueError("gyro bias estimation needs at least one rotation pair")
    if not all(math.isfinite(component) for component in seed):
        raise ValueError("gyro bias seed must be finite")

    bias = seed
    iterations = 0
    normal: list[list[float]] = []
    residuals: tuple[Vector3, ...] = ()
    for iterations in range(1, 9):
        normal, gradient, residuals = _linearize(pairs, bias)
        delta = _solve_3x3(normal, _scale(gradient, -1.0))
        bias = _add(bias, delta)
        if _norm(delta) < 1e-10:
            break
    normal, _, residuals = _linearize(pairs, bias)
    eigenvalues = _symmetric_eigenvalues(normal)
    condition = (
        math.inf
        if eigenvalues[0] <= 1e-18
        else eigenvalues[-1] / eigenvalues[0]
    )
    residual_norms = tuple(_norm(residual) for residual in residuals)
    residual_mean, residual_std, residual_lag1 = _residual_stats(residuals)
    residual_sum_sq = sum(
        component * component
        for residual in residuals
        for component in residual
    )
    residual_variance = residual_sum_sq / float(max(1, 3 * len(residuals) - 3))
    inverse_columns = tuple(
        _solve_3x3(
            normal,
            tuple(1.0 if row == column else 0.0 for row in range(3)),
        )
        for column in range(3)
    )
    bias_sigma_iid = tuple(
        math.sqrt(max(0.0, residual_variance * inverse_columns[axis][axis]))
        for axis in range(3)
    )
    return BiasEstimate(
        bias=bias,
        rotation_residual_rms_rad=math.sqrt(
            sum(value * value for value in residual_norms)
            / float(len(residual_norms))
        ),
        rotation_residual_max_rad=max(residual_norms),
        residual_mean=residual_mean,
        residual_std=residual_std,
        residual_lag1=residual_lag1,
        bias_sigma_iid=bias_sigma_iid,  # type: ignore[arg-type]
        normal_min_eigenvalue=eigenvalues[0],
        normal_max_eigenvalue=eigenvalues[-1],
        normal_condition=condition,
        iterations=iterations,
    )


def _finite(value: str, field: str, path: Path, line: int) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field}") from exc
    if not math.isfinite(parsed):
        raise ProbeError(f"{path}:{line}: {field} must be finite")
    return parsed


def _timestamp_ns(value: str, path: Path, line: int) -> int:
    try:
        timestamp = Decimal(value) * Decimal(1_000_000_000)
    except InvalidOperation as exc:
        raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
    if timestamp != timestamp.to_integral_value():
        raise ProbeError(f"{path}:{line}: timestamp is not integral nanoseconds")
    return int(timestamp)


def load_diag(path: Path) -> tuple[int, ...]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    timestamps: list[int] = []
    previous: Optional[int] = None
    with input_file:
        reader = csv.DictReader(input_file, skipinitialspace=True)
        required = {
            "timestamp_ns",
            "status",
        }
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ProbeError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line, row in enumerate(reader, start=2):
            try:
                timestamp = int(row["timestamp_ns"])
            except (TypeError, ValueError) as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp_ns") from exc
            if previous is not None and timestamp <= previous:
                raise ProbeError(f"{path}:{line}: timestamps are not strictly increasing")
            previous = timestamp
            if row["status"] != "ok":
                continue
            timestamps.append(timestamp)
    if len(timestamps) < 2:
        raise ProbeError(f"{path}: needs at least two accepted rows")
    return tuple(timestamps)


def load_init_seed(path: Path) -> Vector3:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.DictReader(input_file, skipinitialspace=True)
        required = {f"init_bg_{axis}" for axis in "xyz"}
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ProbeError(
                f"{path}: missing columns: {', '.join(sorted(missing))}"
            )
        rows = list(reader)
    if len(rows) != 1:
        raise ProbeError(f"{path}: expected exactly one init snapshot")
    return tuple(
        _finite(rows[0][f"init_bg_{axis}"], f"init_bg_{axis}", path, 2)
        for axis in "xyz"
    )  # type: ignore[return-value]


def load_visual_rotations(path: Path) -> dict[int, Quaternion]:
    rotations: dict[int, Quaternion] = {}
    previous: Optional[int] = None
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    for line_number, line in enumerate(lines, start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ProbeError(f"{path}:{line_number}: expected 8 TUM columns")
        timestamp = _timestamp_ns(fields[0], path, line_number)
        if previous is not None and timestamp <= previous:
            raise ProbeError(f"{path}:{line_number}: timestamps are not strictly increasing")
        previous = timestamp
        quaternion = (
            _finite(fields[7], "qw", path, line_number),
            _finite(fields[4], "qx", path, line_number),
            _finite(fields[5], "qy", path, line_number),
            _finite(fields[6], "qz", path, line_number),
        )
        try:
            rotations[timestamp] = _quaternion_normalize(quaternion)
        except ValueError as exc:
            raise ProbeError(f"{path}:{line_number}: invalid quaternion") from exc
    if len(rotations) < 2:
        raise ProbeError(f"{path}: needs at least two poses")
    return rotations


def load_imu(path: Path) -> tuple[ImuSample, ...]:
    samples: list[ImuSample] = []
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.reader(input_file)
        for line, row in enumerate(reader, start=1):
            if not row or row[0].startswith("#"):
                continue
            if len(row) < 4:
                raise ProbeError(f"{path}:{line}: expected at least 4 columns")
            try:
                timestamp = int(row[0])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
            gyro = tuple(
                _finite(row[index], f"gyro_{'xyz'[index - 1]}", path, line)
                for index in range(1, 4)
            )
            if samples and timestamp <= samples[-1].timestamp_ns:
                raise ProbeError(f"{path}:{line}: timestamps are not strictly increasing")
            samples.append(ImuSample(timestamp, gyro))  # type: ignore[arg-type]
    if len(samples) < 2:
        raise ProbeError(f"{path}: needs at least two IMU samples")
    return tuple(samples)


def load_reference_bias(path: Path) -> tuple[ReferenceBias, ...]:
    rows: list[ReferenceBias] = []
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.reader(input_file)
        for line, row in enumerate(reader, start=1):
            if not row or row[0].startswith("#"):
                continue
            if len(row) < 14:
                raise ProbeError(f"{path}:{line}: expected at least 14 columns")
            try:
                timestamp = int(row[0])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
            bias = tuple(
                _finite(row[index], f"bias_gyro_{'xyz'[index - 11]}", path, line)
                for index in range(11, 14)
            )
            if rows and timestamp <= rows[-1].timestamp_ns:
                raise ProbeError(f"{path}:{line}: timestamps are not strictly increasing")
            rows.append(ReferenceBias(timestamp, bias))  # type: ignore[arg-type]
    if len(rows) < 2:
        raise ProbeError(f"{path}: needs at least two reference rows")
    return tuple(rows)


def _interpolate_gyro(
    samples: tuple[ImuSample, ...], timestamps: tuple[int, ...], timestamp: int
) -> ImuSample:
    right = bisect.bisect_left(timestamps, timestamp)
    if right < len(samples) and samples[right].timestamp_ns == timestamp:
        return samples[right]
    if right == 0 or right == len(samples):
        raise ProbeError(f"IMU does not bracket timestamp {timestamp}")
    left_sample = samples[right - 1]
    right_sample = samples[right]
    alpha = float(timestamp - left_sample.timestamp_ns) / float(
        right_sample.timestamp_ns - left_sample.timestamp_ns
    )
    gyro = _add(
        left_sample.gyro,
        _scale(_subtract(right_sample.gyro, left_sample.gyro), alpha),
    )
    return ImuSample(timestamp, gyro)


def build_pairs(
    accepted_timestamps: tuple[int, ...],
    visual_rotations: dict[int, Quaternion],
    imu_samples: tuple[ImuSample, ...],
) -> tuple[RotationPair, ...]:
    imu_timestamps = tuple(sample.timestamp_ns for sample in imu_samples)
    pairs: list[RotationPair] = []
    for t_i, t_j in zip(accepted_timestamps, accepted_timestamps[1:]):
        if t_i not in visual_rotations or t_j not in visual_rotations:
            continue
        begin = bisect.bisect_right(imu_timestamps, t_i)
        end = bisect.bisect_left(imu_timestamps, t_j)
        segment = (
            _interpolate_gyro(imu_samples, imu_timestamps, t_i),
            *imu_samples[begin:end],
            _interpolate_gyro(imu_samples, imu_timestamps, t_j),
        )
        visual_delta = _quaternion_normalize(
            _quaternion_multiply(
                _quaternion_conjugate(visual_rotations[t_i]),
                visual_rotations[t_j],
            )
        )
        pairs.append(RotationPair(t_i, t_j, segment, visual_delta))
    if not pairs:
        raise ProbeError("accepted and visual trajectories have no usable pair")
    return tuple(pairs)


def interpolate_reference(
    rows: tuple[ReferenceBias, ...], timestamp_ns: int
) -> Vector3:
    timestamps = tuple(row.timestamp_ns for row in rows)
    right = bisect.bisect_right(timestamps, timestamp_ns)
    if right == 0 or right == len(rows):
        raise ProbeError(f"reference bias does not bracket timestamp {timestamp_ns}")
    left = rows[right - 1]
    next_row = rows[right]
    alpha = float(timestamp_ns - left.timestamp_ns) / float(
        next_row.timestamp_ns - left.timestamp_ns
    )
    return _add(left.bias, _scale(_subtract(next_row.bias, left.bias), alpha))


def analyze(
    pairs: tuple[RotationPair, ...],
    seed: Vector3,
    reference: tuple[ReferenceBias, ...],
    horizons: Iterable[int],
) -> tuple[ProbeRow, ...]:
    counts = sorted({min(count, len(pairs)) for count in horizons if count > 0})
    if len(pairs) not in counts:
        counts.append(len(pairs))
    rows: list[ProbeRow] = []
    for count in counts:
        selected = pairs[:count]
        estimate = estimate_bias(selected, seed)
        rows.append(
            ProbeRow(
                pair_count=count,
                duration_s=float(selected[-1].t_j_ns - selected[0].t_i_ns)
                * 1e-9,
                start_ns=selected[0].t_i_ns,
                end_ns=selected[-1].t_j_ns,
                estimate=estimate,
                seed=seed,
                reference=interpolate_reference(reference, selected[-1].t_j_ns),
            )
        )
    return tuple(rows)


def analyze_rolling(
    pairs: tuple[RotationPair, ...],
    seed: Vector3,
    reference: tuple[ReferenceBias, ...],
    window_pairs: int,
    stride_pairs: int,
) -> tuple[ProbeRow, ...]:
    """Estimate bias on complete, fixed-size windows at a fixed stride."""
    if window_pairs <= 0:
        raise ValueError("window_pairs must be positive")
    if stride_pairs <= 0:
        raise ValueError("stride_pairs must be positive")
    if window_pairs > len(pairs):
        raise ValueError("window_pairs must not exceed available pairs")

    rows: list[ProbeRow] = []
    last_start = len(pairs) - window_pairs
    for start in range(0, last_start + 1, stride_pairs):
        selected = pairs[start : start + window_pairs]
        estimate = estimate_bias(selected, seed)
        rows.append(
            ProbeRow(
                pair_count=window_pairs,
                duration_s=float(selected[-1].t_j_ns - selected[0].t_i_ns)
                * 1e-9,
                start_ns=selected[0].t_i_ns,
                end_ns=selected[-1].t_j_ns,
                estimate=estimate,
                seed=seed,
                reference=interpolate_reference(reference, selected[-1].t_j_ns),
            )
        )
    return tuple(rows)


def render_csv(rows: tuple[ProbeRow, ...], output: TextIO) -> None:
    writer = csv.DictWriter(output, fieldnames=OUTPUT_FIELDS)
    writer.writeheader()
    for row in rows:
        delta = _subtract(row.estimate.bias, row.seed)
        reference_error = _subtract(row.estimate.bias, row.reference)
        writer.writerow(
            {
                "pair_count": row.pair_count,
                "duration_s": f"{row.duration_s:.17g}",
                "start_ns": row.start_ns,
                "end_ns": row.end_ns,
                "bias_x_rps": f"{row.estimate.bias[0]:.17g}",
                "bias_y_rps": f"{row.estimate.bias[1]:.17g}",
                "bias_z_rps": f"{row.estimate.bias[2]:.17g}",
                "delta_seed_x_rps": f"{delta[0]:.17g}",
                "delta_seed_y_rps": f"{delta[1]:.17g}",
                "delta_seed_z_rps": f"{delta[2]:.17g}",
                "delta_seed_norm_rps": f"{_norm(delta):.17g}",
                "ref_bias_x_rps": f"{row.reference[0]:.17g}",
                "ref_bias_y_rps": f"{row.reference[1]:.17g}",
                "ref_bias_z_rps": f"{row.reference[2]:.17g}",
                "ref_error_x_rps": f"{reference_error[0]:.17g}",
                "ref_error_y_rps": f"{reference_error[1]:.17g}",
                "ref_error_z_rps": f"{reference_error[2]:.17g}",
                "ref_error_norm_rps": f"{_norm(reference_error):.17g}",
                "rotation_residual_rms_rad": (
                    f"{row.estimate.rotation_residual_rms_rad:.17g}"
                ),
                "rotation_residual_max_rad": (
                    f"{row.estimate.rotation_residual_max_rad:.17g}"
                ),
                "residual_mean_x_rad": f"{row.estimate.residual_mean[0]:.17g}",
                "residual_mean_y_rad": f"{row.estimate.residual_mean[1]:.17g}",
                "residual_mean_z_rad": f"{row.estimate.residual_mean[2]:.17g}",
                "residual_std_x_rad": f"{row.estimate.residual_std[0]:.17g}",
                "residual_std_y_rad": f"{row.estimate.residual_std[1]:.17g}",
                "residual_std_z_rad": f"{row.estimate.residual_std[2]:.17g}",
                "residual_lag1_x": f"{row.estimate.residual_lag1[0]:.17g}",
                "residual_lag1_y": f"{row.estimate.residual_lag1[1]:.17g}",
                "residual_lag1_z": f"{row.estimate.residual_lag1[2]:.17g}",
                "bias_sigma_iid_x_rps": (
                    f"{row.estimate.bias_sigma_iid[0]:.17g}"
                ),
                "bias_sigma_iid_y_rps": (
                    f"{row.estimate.bias_sigma_iid[1]:.17g}"
                ),
                "bias_sigma_iid_z_rps": (
                    f"{row.estimate.bias_sigma_iid[2]:.17g}"
                ),
                "normal_min_eigenvalue_s2": (
                    f"{row.estimate.normal_min_eigenvalue:.17g}"
                ),
                "normal_max_eigenvalue_s2": (
                    f"{row.estimate.normal_max_eigenvalue:.17g}"
                ),
                "normal_condition": f"{row.estimate.normal_condition:.17g}",
                "iterations": row.estimate.iterations,
            }
        )


def _parse_horizons(value: str) -> tuple[int, ...]:
    try:
        horizons = tuple(int(item) for item in value.split(",") if item)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("horizons must be comma-separated integers") from exc
    if not horizons or any(item <= 0 for item in horizons):
        raise argparse.ArgumentTypeError("horizons must be positive")
    return horizons


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gyro_run", type=Path)
    parser.add_argument("visual_run", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument(
        "--init-sidecar",
        type=Path,
        help="one-row vio_init.csv (default: <gyro_run>/vio_init.csv)",
    )
    parser.add_argument(
        "--horizon-pairs",
        type=_parse_horizons,
        default=(20, 50, 100, 200, 500, 1000, 2000),
    )
    parser.add_argument(
        "--rolling-window-pairs",
        type=int,
        help="estimate complete fixed-size rolling windows instead of prefixes",
    )
    parser.add_argument(
        "--rolling-stride-pairs",
        type=int,
        default=1,
        help="start-index stride for --rolling-window-pairs (default: 1)",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    try:
        timestamps = load_diag(args.gyro_run / "diag.csv")
        init_sidecar = (
            args.init_sidecar
            if args.init_sidecar is not None
            else args.gyro_run / "vio_init.csv"
        )
        seed = load_init_seed(init_sidecar)
        visual = load_visual_rotations(args.visual_run / "est.tum")
        imu = load_imu(args.euroc_root / "mav0" / "imu0" / "data.csv")
        reference = load_reference_bias(
            args.euroc_root
            / "mav0"
            / "state_groundtruth_estimate0"
            / "data.csv"
        )
        pairs = build_pairs(timestamps, visual, imu)
        pairs = tuple(
            pair
            for pair in pairs
            if reference[0].timestamp_ns < pair.t_j_ns
            < reference[-1].timestamp_ns
        )
        if not pairs:
            raise ProbeError("no rotation pair lies inside reference support")
        if args.rolling_window_pairs is None:
            rows = analyze(pairs, seed, reference, args.horizon_pairs)
        else:
            rows = analyze_rolling(
                pairs,
                seed,
                reference,
                args.rolling_window_pairs,
                args.rolling_stride_pairs,
            )
        if args.output is None:
            render_csv(rows, sys.stdout)
        else:
            with args.output.open("w", encoding="utf-8", newline="") as output:
                render_csv(rows, output)
    except (OSError, ProbeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.output is not None:
        print(
            f"pairs={len(pairs)} rows={len(rows)} output={args.output}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

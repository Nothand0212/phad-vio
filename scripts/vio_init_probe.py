#!/usr/bin/env python3
"""Audit one production VIO static-initialization snapshot against EuRoC."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, TextIO


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

INIT_FIELDS = (
    "imu_t_i_ns",
    "imu_t_j_ns",
    "imu_sample_n",
    "imu_dt_s",
    "gyro_mean_x",
    "gyro_mean_y",
    "gyro_mean_z",
    "gyro_std_x",
    "gyro_std_y",
    "gyro_std_z",
    "acc_mean_x",
    "acc_mean_y",
    "acc_mean_z",
    "acc_std_x",
    "acc_std_y",
    "acc_std_z",
    "gyro_std_limit",
    "acc_std_limit",
    "init_q_x",
    "init_q_y",
    "init_q_z",
    "init_q_w",
    "init_v_x",
    "init_v_y",
    "init_v_z",
    "init_bg_x",
    "init_bg_y",
    "init_bg_z",
    "init_ba_x",
    "init_ba_y",
    "init_ba_z",
    "acc_mean_norm",
    "gravity_model_magnitude",
)

IMU_FIELDS = (
    "#timestamp [ns]",
    "w_RS_S_x [rad s^-1]",
    "w_RS_S_y [rad s^-1]",
    "w_RS_S_z [rad s^-1]",
    "a_RS_S_x [m s^-2]",
    "a_RS_S_y [m s^-2]",
    "a_RS_S_z [m s^-2]",
)

GT_FIELDS = (
    "#timestamp",
    "p_RS_R_x [m]",
    "p_RS_R_y [m]",
    "p_RS_R_z [m]",
    "q_RS_w []",
    "q_RS_x []",
    "q_RS_y []",
    "q_RS_z []",
    "v_RS_R_x [m s^-1]",
    "v_RS_R_y [m s^-1]",
    "v_RS_R_z [m s^-1]",
    "b_w_RS_S_x [rad s^-1]",
    "b_w_RS_S_y [rad s^-1]",
    "b_w_RS_S_z [rad s^-1]",
    "b_a_RS_S_x [m s^-2]",
    "b_a_RS_S_y [m s^-2]",
    "b_a_RS_S_z [m s^-2]",
)


class ProbeError(RuntimeError):
    """An input violates the truth-audit contract."""


@dataclass(frozen=True)
class InitSnapshot:
    imu_t_i_ns: int
    imu_t_j_ns: int
    imu_sample_count: int
    imu_dt_s: float
    gyro_mean: Vector3
    gyro_std: Vector3
    acc_mean: Vector3
    acc_std: Vector3
    gyro_std_limit: float
    acc_std_limit: float
    q_W_B0: Quaternion
    velocity_W: Vector3
    bias_gyro: Vector3
    bias_acc: Vector3
    acc_mean_norm: float
    gravity_model_magnitude: float


@dataclass(frozen=True)
class GroundtruthState:
    timestamp_ns: int
    position_W: Vector3
    q_W_B: Quaternion
    velocity_W: Vector3
    bias_gyro: Vector3
    bias_acc: Vector3


@dataclass(frozen=True)
class ReferenceAt:
    target_ns: int
    left_ns: int
    right_ns: int
    alpha: float
    state: GroundtruthState


@dataclass(frozen=True)
class RawImuAudit:
    sample_count: int
    dt_min_s: float
    dt_median_s: float
    dt_max_s: float
    gyro_mean: Vector3
    gyro_std: Vector3
    acc_mean: Vector3
    acc_std: Vector3


@dataclass(frozen=True)
class RawImuMeasurement:
    timestamp_ns: int
    gyro: Vector3
    acc: Vector3


@dataclass(frozen=True)
class AuditResult:
    imu_t_i_ns: int
    imu_t_j_ns: int
    imu_sample_count: int
    imu_dt_s: float
    raw_imu_stats_verified: bool
    raw_imu_dt_min_s: float
    raw_imu_dt_median_s: float
    raw_imu_dt_max_s: float
    gt_t0_left_ns: int
    gt_t0_right_ns: int
    gt_t0_alpha: float
    gt_t0_bracket_span_ns: int
    gt_t1_left_ns: int
    gt_t1_right_ns: int
    gt_t1_alpha: float
    gt_t1_bracket_span_ns: int
    gyro_mean_rps: Vector3
    gyro_std_rps: Vector3
    acc_mean_mps2: Vector3
    acc_std_mps2: Vector3
    gyro_std_limit_rps: float
    acc_std_limit_mps2: float
    init_q_W_B0: Quaternion
    gt_t0_q_W_B: Quaternion
    gt_t1_q_W_B: Quaternion
    init_body_up_B: Vector3
    gt_body_up_B: Vector3
    gravity_angle_deg: float
    init_velocity_W_mps: Vector3
    gt_t0_velocity_W_mps: Vector3
    gt_t1_velocity_W_mps: Vector3
    initial_velocity_error_W_mps: Vector3
    initial_velocity_error_norm_mps: float
    gt_t0_speed_mps: float
    gt_t1_speed_mps: float
    gt_window_rotation_deg: float
    gt_window_translation_range_m: float
    gt_window_rotation_range_deg: float
    gt_window_speed_max_mps: float
    gt_window_speed_rms_mps: float
    gt_delta_v_over_dt_mps2: float
    init_gyro_bias_rps: Vector3
    gt_gyro_bias_rps: Vector3
    gyro_bias_error: Vector3
    gyro_bias_error_norm_rps: float
    support_gyro_bias_error: Vector3
    support_gyro_bias_error_norm_rps: float
    init_acc_bias_mps2: Vector3
    gt_acc_bias_mps2: Vector3
    acc_bias_error: Vector3
    acc_bias_error_norm_mps2: float
    acc_bias_error_parallel_mps2: float
    acc_bias_error_tangent_mps2: float
    support_acc_bias_error: Vector3
    support_acc_bias_error_norm_mps2: float
    acc_mean_norm_mps2: float
    gravity_model_magnitude_mps2: float
    acc_mean_norm_minus_gravity_model_mps2: float
    static_gyro_residual_rps: Vector3
    static_gyro_residual_norm_rps: float
    static_acc_residual_mps2: Vector3
    static_acc_residual_norm_mps2: float


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


def _quaternion(
    row: dict[str, str],
    fields: tuple[str, str, str, str],
    path: Path,
    line: int,
    tolerance: float = 1e-9,
) -> Quaternion:
    values = tuple(_finite(row[field], field, path, line) for field in fields)
    norm = math.sqrt(sum(value * value for value in values))
    if abs(norm - 1.0) > tolerance:
        raise ProbeError(f"{path}:{line}: unit quaternion required")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _open_csv(path: Path) -> tuple[TextIO, csv.DictReader]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    return input_file, csv.DictReader(input_file, skipinitialspace=True)


def load_init(path: Path) -> InitSnapshot:
    input_file, reader = _open_csv(path)
    with input_file:
        if tuple(reader.fieldnames or ()) != INIT_FIELDS:
            raise ProbeError(f"{path}: init probe schema mismatch")
        rows = list(reader)
    if len(rows) != 1:
        raise ProbeError(f"{path}: expected exactly one init row")

    row = rows[0]
    line = 2
    snapshot = InitSnapshot(
        imu_t_i_ns=_integer(row["imu_t_i_ns"], "imu_t_i_ns", path, line),
        imu_t_j_ns=_integer(row["imu_t_j_ns"], "imu_t_j_ns", path, line),
        imu_sample_count=_integer(
            row["imu_sample_n"], "imu_sample_n", path, line
        ),
        imu_dt_s=_finite(row["imu_dt_s"], "imu_dt_s", path, line),
        gyro_mean=_vector(
            row,
            ("gyro_mean_x", "gyro_mean_y", "gyro_mean_z"),
            path,
            line,
        ),
        gyro_std=_vector(
            row,
            ("gyro_std_x", "gyro_std_y", "gyro_std_z"),
            path,
            line,
        ),
        acc_mean=_vector(
            row, ("acc_mean_x", "acc_mean_y", "acc_mean_z"), path, line
        ),
        acc_std=_vector(
            row, ("acc_std_x", "acc_std_y", "acc_std_z"), path, line
        ),
        gyro_std_limit=_finite(
            row["gyro_std_limit"], "gyro_std_limit", path, line
        ),
        acc_std_limit=_finite(
            row["acc_std_limit"], "acc_std_limit", path, line
        ),
        q_W_B0=_quaternion(
            row,
            ("init_q_x", "init_q_y", "init_q_z", "init_q_w"),
            path,
            line,
        ),
        velocity_W=_vector(
            row, ("init_v_x", "init_v_y", "init_v_z"), path, line
        ),
        bias_gyro=_vector(
            row, ("init_bg_x", "init_bg_y", "init_bg_z"), path, line
        ),
        bias_acc=_vector(
            row, ("init_ba_x", "init_ba_y", "init_ba_z"), path, line
        ),
        acc_mean_norm=_finite(
            row["acc_mean_norm"], "acc_mean_norm", path, line
        ),
        gravity_model_magnitude=_finite(
            row["gravity_model_magnitude"],
            "gravity_model_magnitude",
            path,
            line,
        ),
    )
    if snapshot.imu_sample_count < 2:
        raise ProbeError(f"{path}:{line}: at least two IMU samples required")
    if snapshot.imu_t_i_ns >= snapshot.imu_t_j_ns:
        raise ProbeError(f"{path}:{line}: IMU interval is not increasing")
    if snapshot.imu_dt_s <= 0.0:
        raise ProbeError(f"{path}:{line}: IMU duration must be positive")
    expected_dt = (snapshot.imu_t_j_ns - snapshot.imu_t_i_ns) * 1e-9
    tolerance = max(1e-12, abs(expected_dt) * 1e-12)
    if abs(snapshot.imu_dt_s - expected_dt) > tolerance:
        raise ProbeError(f"{path}:{line}: IMU duration does not match endpoints")
    if any(value < 0.0 for value in snapshot.gyro_std + snapshot.acc_std):
        raise ProbeError(f"{path}:{line}: standard deviations must be non-negative")
    if (
        snapshot.gyro_std_limit <= 0.0
        or snapshot.acc_std_limit <= 0.0
        or snapshot.acc_mean_norm <= 0.0
        or snapshot.gravity_model_magnitude <= 0.0
    ):
        raise ProbeError(f"{path}:{line}: limits and gravity must be positive")
    if abs(snapshot.acc_mean_norm - _norm(snapshot.acc_mean)) > 1e-12:
        raise ProbeError(
            f"{path}:{line}: acc_mean_norm does not match acc_mean"
        )
    return snapshot


def load_groundtruth(path: Path) -> tuple[GroundtruthState, ...]:
    input_file, reader = _open_csv(path)
    states: list[GroundtruthState] = []
    previous: Optional[int] = None
    with input_file:
        if tuple(reader.fieldnames or ()) != GT_FIELDS:
            raise ProbeError(f"{path}: EuRoC groundtruth schema mismatch")
        for line, row in enumerate(reader, start=2):
            timestamp_ns = _integer(row["#timestamp"], "#timestamp", path, line)
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{line}: timestamps must be strictly increasing"
                )
            previous = timestamp_ns
            states.append(
                GroundtruthState(
                    timestamp_ns=timestamp_ns,
                    position_W=_vector(
                        row,
                        ("p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]"),
                        path,
                        line,
                    ),
                    q_W_B=_quaternion(
                        row,
                        ("q_RS_x []", "q_RS_y []", "q_RS_z []", "q_RS_w []"),
                        path,
                        line,
                        # Match the production EuRoC loader: the published
                        # batch reference contains norm error up to ~6.8e-5.
                        tolerance=1e-3,
                    ),
                    velocity_W=_vector(
                        row,
                        (
                            "v_RS_R_x [m s^-1]",
                            "v_RS_R_y [m s^-1]",
                            "v_RS_R_z [m s^-1]",
                        ),
                        path,
                        line,
                    ),
                    bias_gyro=_vector(
                        row,
                        (
                            "b_w_RS_S_x [rad s^-1]",
                            "b_w_RS_S_y [rad s^-1]",
                            "b_w_RS_S_z [rad s^-1]",
                        ),
                        path,
                        line,
                    ),
                    bias_acc=_vector(
                        row,
                        (
                            "b_a_RS_S_x [m s^-2]",
                            "b_a_RS_S_y [m s^-2]",
                            "b_a_RS_S_z [m s^-2]",
                        ),
                        path,
                        line,
                    ),
                )
            )
    if not states:
        raise ProbeError(f"{path}: no groundtruth rows")
    return tuple(states)


def _require_identity_t_bs(path: Path) -> None:
    try:
        contents = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    start = contents.find("T_BS:")
    if start < 0:
        raise ProbeError(f"{path}: missing T_BS")
    block = contents[start:]
    if re.search(r"\brows\s*:\s*4\b", block) is None or re.search(
        r"\bcols\s*:\s*4\b", block
    ) is None:
        raise ProbeError(f"{path}: T_BS must be 4x4")
    match = re.search(r"\bdata\s*:\s*\[([^]]*)\]", block, re.DOTALL)
    if match is None:
        raise ProbeError(f"{path}: missing T_BS data")
    try:
        values = tuple(float(value.strip()) for value in match.group(1).split(","))
    except ValueError as exc:
        raise ProbeError(f"{path}: invalid T_BS data") from exc
    if len(values) != 16 or not all(math.isfinite(value) for value in values):
        raise ProbeError(f"{path}: T_BS must contain 16 finite values")
    identity = tuple(
        1.0 if row == column else 0.0
        for row in range(4)
        for column in range(4)
    )
    if any(abs(actual - expected) > 1e-12 for actual, expected in zip(values, identity)):
        raise ProbeError(f"{path}: identity T_BS required for axis comparison")


def load_raw_imu(path: Path) -> tuple[RawImuMeasurement, ...]:
    input_file, reader = _open_csv(path)
    measurements: list[RawImuMeasurement] = []
    previous: Optional[int] = None
    with input_file:
        if tuple(reader.fieldnames or ()) != IMU_FIELDS:
            raise ProbeError(f"{path}: EuRoC IMU schema mismatch")
        for line, row in enumerate(reader, start=2):
            timestamp_ns = _integer(
                row["#timestamp [ns]"], "#timestamp [ns]", path, line
            )
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{line}: timestamps must be strictly increasing"
                )
            previous = timestamp_ns
            measurements.append(
                RawImuMeasurement(
                    timestamp_ns=timestamp_ns,
                    gyro=_vector(
                        row,
                        (
                            "w_RS_S_x [rad s^-1]",
                            "w_RS_S_y [rad s^-1]",
                            "w_RS_S_z [rad s^-1]",
                        ),
                        path,
                        line,
                    ),
                    acc=_vector(
                        row,
                        (
                            "a_RS_S_x [m s^-2]",
                            "a_RS_S_y [m s^-2]",
                            "a_RS_S_z [m s^-2]",
                        ),
                        path,
                        line,
                    ),
                )
            )
    if not measurements:
        raise ProbeError(f"{path}: no IMU rows")
    return tuple(measurements)


def _population_stats(values: tuple[Vector3, ...]) -> tuple[Vector3, Vector3]:
    count = len(values)
    mean = tuple(
        math.fsum(value[axis] for value in values) / count
        for axis in range(3)
    )
    variance = tuple(
        math.fsum((value[axis] - mean[axis]) ** 2 for value in values) / count
        for axis in range(3)
    )
    return (
        mean,  # type: ignore[return-value]
        tuple(math.sqrt(value) for value in variance),  # type: ignore[return-value]
    )


def _validate_raw_imu(
    snapshot: InitSnapshot, measurements: tuple[RawImuMeasurement, ...]
) -> RawImuAudit:
    adopted = tuple(
        measurement
        for measurement in measurements
        if snapshot.imu_t_i_ns
        <= measurement.timestamp_ns
        <= snapshot.imu_t_j_ns
    )
    if len(adopted) != snapshot.imu_sample_count:
        raise ProbeError(
            "raw IMU adopted-window sample count does not match sidecar"
        )
    if (
        not adopted
        or adopted[0].timestamp_ns != snapshot.imu_t_i_ns
        or adopted[-1].timestamp_ns != snapshot.imu_t_j_ns
    ):
        raise ProbeError("raw IMU adopted-window endpoints do not match sidecar")
    gyro_mean, gyro_std = _population_stats(
        tuple(measurement.gyro for measurement in adopted)
    )
    acc_mean, acc_std = _population_stats(
        tuple(measurement.acc for measurement in adopted)
    )

    for name, actual, recorded in (
        ("gyro mean", gyro_mean, snapshot.gyro_mean),
        ("gyro std", gyro_std, snapshot.gyro_std),
        ("accelerometer mean", acc_mean, snapshot.acc_mean),
        ("accelerometer std", acc_std, snapshot.acc_std),
    ):
        if any(abs(left - right) > 5e-12 for left, right in zip(actual, recorded)):
            raise ProbeError(f"raw IMU {name} does not match sidecar")

    dts = tuple(
        (right.timestamp_ns - left.timestamp_ns) * 1e-9
        for left, right in zip(adopted, adopted[1:])
    )
    return RawImuAudit(
        sample_count=len(adopted),
        dt_min_s=min(dts),
        dt_median_s=statistics.median(dts),
        dt_max_s=max(dts),
        gyro_mean=gyro_mean,
        gyro_std=gyro_std,
        acc_mean=acc_mean,
        acc_std=acc_std,
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _scale(value: Vector3, scalar: float) -> Vector3:
    return tuple(component * scalar for component in value)  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(value: Vector3) -> float:
    return math.sqrt(_dot(value, value))


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _conjugate(value: Quaternion) -> Quaternion:
    return (-value[0], -value[1], -value[2], value[3])


def _lerp_vector(left: Vector3, right: Vector3, alpha: float) -> Vector3:
    return tuple(
        a + alpha * (b - a) for a, b in zip(left, right)
    )  # type: ignore[return-value]


def _slerp(left: Quaternion, right: Quaternion, alpha: float) -> Quaternion:
    dot = sum(a * b for a, b in zip(left, right))
    if dot < 0.0:
        right = tuple(-value for value in right)  # type: ignore[assignment]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        interpolated = tuple(
            a + alpha * (b - a) for a, b in zip(left, right)
        )
        norm = math.sqrt(sum(value * value for value in interpolated))
        return tuple(value / norm for value in interpolated)  # type: ignore[return-value]
    theta = math.acos(dot)
    denominator = math.sin(theta)
    left_weight = math.sin((1.0 - alpha) * theta) / denominator
    right_weight = math.sin(alpha * theta) / denominator
    return tuple(
        left_weight * a + right_weight * b for a, b in zip(left, right)
    )  # type: ignore[return-value]


def _interpolate_state(
    left: GroundtruthState,
    right: GroundtruthState,
    target_ns: int,
    alpha: float,
) -> GroundtruthState:
    return GroundtruthState(
        timestamp_ns=target_ns,
        position_W=_lerp_vector(left.position_W, right.position_W, alpha),
        q_W_B=_slerp(left.q_W_B, right.q_W_B, alpha),
        velocity_W=_lerp_vector(left.velocity_W, right.velocity_W, alpha),
        bias_gyro=_lerp_vector(left.bias_gyro, right.bias_gyro, alpha),
        bias_acc=_lerp_vector(left.bias_acc, right.bias_acc, alpha),
    )


def _bracket(
    states: tuple[GroundtruthState, ...],
    target_ns: int,
    max_bracket_ns: int,
) -> ReferenceAt:
    timestamps = [state.timestamp_ns for state in states]
    index = bisect.bisect_left(timestamps, target_ns)
    if index < len(states) and states[index].timestamp_ns == target_ns:
        state = states[index]
        return ReferenceAt(target_ns, target_ns, target_ns, 0.0, state)
    if index == 0 or index == len(states):
        raise ProbeError(f"{target_ns} is outside groundtruth support")
    left = states[index - 1]
    right = states[index]
    span_ns = right.timestamp_ns - left.timestamp_ns
    if span_ns > max_bracket_ns:
        raise ProbeError(
            f"groundtruth bracket span {span_ns} ns exceeds "
            f"{max_bracket_ns} ns at {target_ns}"
        )
    alpha = (target_ns - left.timestamp_ns) / span_ns
    return ReferenceAt(
        target_ns=target_ns,
        left_ns=left.timestamp_ns,
        right_ns=right.timestamp_ns,
        alpha=alpha,
        state=_interpolate_state(left, right, target_ns, alpha),
    )


def _rotate(q: Quaternion, vector: Vector3) -> Vector3:
    rotated = _quaternion_multiply(
        _quaternion_multiply(q, (vector[0], vector[1], vector[2], 0.0)),
        _conjugate(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def _body_up(q_W_B: Quaternion) -> Vector3:
    return _rotate(_conjugate(q_W_B), (0.0, 0.0, 1.0))


def _vector_angle_deg(left: Vector3, right: Vector3) -> float:
    denominator = _norm(left) * _norm(right)
    if denominator <= 0.0:
        raise ProbeError("cannot compare zero-length directions")
    cosine = max(-1.0, min(1.0, _dot(left, right) / denominator))
    return math.degrees(math.acos(cosine))


def _rotation_angle_deg(left: Quaternion, right: Quaternion) -> float:
    relative = _quaternion_multiply(_conjugate(left), right)
    cosine = max(0.0, min(1.0, abs(relative[3])))
    return math.degrees(2.0 * math.acos(cosine))


def analyze(
    snapshot: InitSnapshot,
    euroc_root: Path,
    *,
    max_bracket_ns: int = 7_500_000,
) -> AuditResult:
    if max_bracket_ns < 0:
        raise ProbeError("maximum bracket span must be non-negative")
    _require_identity_t_bs(euroc_root / "mav0" / "imu0" / "sensor.yaml")
    _require_identity_t_bs(
        euroc_root
        / "mav0"
        / "state_groundtruth_estimate0"
        / "sensor.yaml"
    )
    raw_audit = _validate_raw_imu(
        snapshot,
        load_raw_imu(euroc_root / "mav0" / "imu0" / "data.csv"),
    )
    states = load_groundtruth(
        euroc_root
        / "mav0"
        / "state_groundtruth_estimate0"
        / "data.csv"
    )
    gt_t0_ref = _bracket(states, snapshot.imu_t_i_ns, max_bracket_ns)
    gt_t1_ref = _bracket(states, snapshot.imu_t_j_ns, max_bracket_ns)
    gt_t0 = gt_t0_ref.state
    gt_t1 = gt_t1_ref.state
    support = (
        (gt_t0,)
        + tuple(
            state
            for state in states
            if snapshot.imu_t_i_ns
            < state.timestamp_ns
            < snapshot.imu_t_j_ns
        )
        + (gt_t1,)
    )

    init_body_up = _body_up(snapshot.q_W_B0)
    gt_body_up = _body_up(gt_t0.q_W_B)
    velocity_error = _subtract(snapshot.velocity_W, gt_t0.velocity_W)
    delta_velocity = _subtract(gt_t1.velocity_W, gt_t0.velocity_W)
    gyro_error = _subtract(snapshot.bias_gyro, gt_t0.bias_gyro)
    acc_error = _subtract(snapshot.bias_acc, gt_t0.bias_acc)
    support_gyro_mean, _ = _population_stats(
        tuple(state.bias_gyro for state in support)
    )
    support_acc_mean, _ = _population_stats(
        tuple(state.bias_acc for state in support)
    )
    support_gyro_error = _subtract(snapshot.bias_gyro, support_gyro_mean)
    support_acc_error = _subtract(snapshot.bias_acc, support_acc_mean)
    acc_parallel = _dot(acc_error, gt_body_up)
    acc_tangent = _subtract(acc_error, _scale(gt_body_up, acc_parallel))
    static_gyro_residual = _subtract(snapshot.gyro_mean, gt_t0.bias_gyro)
    expected_static_acc = _add(
        _scale(gt_body_up, snapshot.gravity_model_magnitude), gt_t0.bias_acc
    )
    static_acc_residual = _subtract(snapshot.acc_mean, expected_static_acc)
    support_speeds = tuple(_norm(state.velocity_W) for state in support)
    translation_range = max(
        _norm(_subtract(state.position_W, gt_t0.position_W))
        for state in support
    )
    rotation_range = max(
        _rotation_angle_deg(gt_t0.q_W_B, state.q_W_B) for state in support
    )

    return AuditResult(
        imu_t_i_ns=snapshot.imu_t_i_ns,
        imu_t_j_ns=snapshot.imu_t_j_ns,
        imu_sample_count=snapshot.imu_sample_count,
        imu_dt_s=snapshot.imu_dt_s,
        raw_imu_stats_verified=True,
        raw_imu_dt_min_s=raw_audit.dt_min_s,
        raw_imu_dt_median_s=raw_audit.dt_median_s,
        raw_imu_dt_max_s=raw_audit.dt_max_s,
        gt_t0_left_ns=gt_t0_ref.left_ns,
        gt_t0_right_ns=gt_t0_ref.right_ns,
        gt_t0_alpha=gt_t0_ref.alpha,
        gt_t0_bracket_span_ns=gt_t0_ref.right_ns - gt_t0_ref.left_ns,
        gt_t1_left_ns=gt_t1_ref.left_ns,
        gt_t1_right_ns=gt_t1_ref.right_ns,
        gt_t1_alpha=gt_t1_ref.alpha,
        gt_t1_bracket_span_ns=gt_t1_ref.right_ns - gt_t1_ref.left_ns,
        gyro_mean_rps=snapshot.gyro_mean,
        gyro_std_rps=snapshot.gyro_std,
        acc_mean_mps2=snapshot.acc_mean,
        acc_std_mps2=snapshot.acc_std,
        gyro_std_limit_rps=snapshot.gyro_std_limit,
        acc_std_limit_mps2=snapshot.acc_std_limit,
        init_q_W_B0=snapshot.q_W_B0,
        gt_t0_q_W_B=gt_t0.q_W_B,
        gt_t1_q_W_B=gt_t1.q_W_B,
        init_body_up_B=init_body_up,
        gt_body_up_B=gt_body_up,
        gravity_angle_deg=_vector_angle_deg(init_body_up, gt_body_up),
        init_velocity_W_mps=snapshot.velocity_W,
        gt_t0_velocity_W_mps=gt_t0.velocity_W,
        gt_t1_velocity_W_mps=gt_t1.velocity_W,
        initial_velocity_error_W_mps=velocity_error,
        initial_velocity_error_norm_mps=_norm(velocity_error),
        gt_t0_speed_mps=_norm(gt_t0.velocity_W),
        gt_t1_speed_mps=_norm(gt_t1.velocity_W),
        gt_window_rotation_deg=_rotation_angle_deg(gt_t0.q_W_B, gt_t1.q_W_B),
        gt_window_translation_range_m=translation_range,
        gt_window_rotation_range_deg=rotation_range,
        gt_window_speed_max_mps=max(support_speeds),
        gt_window_speed_rms_mps=math.sqrt(
            math.fsum(speed * speed for speed in support_speeds)
            / len(support_speeds)
        ),
        gt_delta_v_over_dt_mps2=_norm(delta_velocity) / snapshot.imu_dt_s,
        init_gyro_bias_rps=snapshot.bias_gyro,
        gt_gyro_bias_rps=gt_t0.bias_gyro,
        gyro_bias_error=gyro_error,
        gyro_bias_error_norm_rps=_norm(gyro_error),
        support_gyro_bias_error=support_gyro_error,
        support_gyro_bias_error_norm_rps=_norm(support_gyro_error),
        init_acc_bias_mps2=snapshot.bias_acc,
        gt_acc_bias_mps2=gt_t0.bias_acc,
        acc_bias_error=acc_error,
        acc_bias_error_norm_mps2=_norm(acc_error),
        acc_bias_error_parallel_mps2=acc_parallel,
        acc_bias_error_tangent_mps2=_norm(acc_tangent),
        support_acc_bias_error=support_acc_error,
        support_acc_bias_error_norm_mps2=_norm(support_acc_error),
        acc_mean_norm_mps2=snapshot.acc_mean_norm,
        gravity_model_magnitude_mps2=snapshot.gravity_model_magnitude,
        acc_mean_norm_minus_gravity_model_mps2=(
            snapshot.acc_mean_norm - snapshot.gravity_model_magnitude
        ),
        static_gyro_residual_rps=static_gyro_residual,
        static_gyro_residual_norm_rps=_norm(static_gyro_residual),
        static_acc_residual_mps2=static_acc_residual,
        static_acc_residual_norm_mps2=_norm(static_acc_residual),
    )


def render_json(result: AuditResult, output: TextIO) -> None:
    json.dump(asdict(result), output, indent=2, sort_keys=True)
    output.write("\n")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("init_csv", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument("--max-bracket-ms", type=float, default=7.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_bracket_ms) or args.max_bracket_ms < 0.0:
        parser.error("--max-bracket-ms must be finite and non-negative")

    try:
        result = analyze(
            load_init(args.init_csv),
            args.euroc_root,
            max_bracket_ns=int(args.max_bracket_ms * 1_000_000.0),
        )
        if args.output is None:
            render_json(result, sys.stdout)
        else:
            try:
                with args.output.open("w", encoding="utf-8") as output:
                    render_json(result, output)
            except OSError as exc:
                raise ProbeError(f"cannot write {args.output}: {exc}") from exc
            print(f"output={args.output}")
    except ProbeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

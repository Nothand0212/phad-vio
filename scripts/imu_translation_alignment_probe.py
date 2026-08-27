#!/usr/bin/env python3
"""Probe stereo-metric velocity/gravity alignment before full VIO.

The probe is read-only.  It takes poses from an IMU-off run, the aligned gyro
bias and activation epoch from a gyro-enabled run, and raw EuRoC IMU data.  It
solves only per-pose velocity plus one fixed-norm gravity vector.  Accelerometer
bias is first held fixed for velocity/gravity alignment.  ``estimated`` then
adds one shared bias only after the gravity norm is fixed; ``reference`` modes
remain explicitly offline oracles.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
import sys
from dataclasses import dataclass, field
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Callable, Optional, TextIO

import numpy as np


Array = np.ndarray
BiasAt = Callable[[int], Array]


class ProbeError(RuntimeError):
    """A probe input violates its schema, frame, or timestamp contract."""


@dataclass(frozen=True)
class Pose:
    timestamp_ns: int
    rotation: Array
    position: Array


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    gyro: Array
    accel: Array


@dataclass(frozen=True)
class PreintegratedInterval:
    t_i_ns: int
    t_j_ns: int
    delta_p: Array
    delta_v: Array
    delta_rotation: Array = field(default_factory=lambda: np.eye(3))
    jacobian_p_bias_acc: Array = field(default_factory=lambda: np.zeros((3, 3)))
    jacobian_v_bias_acc: Array = field(default_factory=lambda: np.zeros((3, 3)))


@dataclass(frozen=True)
class ReferenceState:
    timestamp_ns: int
    position: Array
    quaternion: Array  # w, x, y, z; active body-to-world
    velocity: Array
    gyro_bias: Array
    accel_bias: Array

    @property
    def rotation(self) -> Array:
        return quaternion_to_matrix(self.quaternion)


@dataclass(frozen=True)
class AlignmentResult:
    velocities: Array
    gravity: Array
    unconstrained_gravity: Array
    rank: int
    raw_condition: float
    column_scaled_condition: float
    position_residual_rms_m: float
    velocity_residual_rms_mps: float


@dataclass(frozen=True)
class BiasAlignmentResult:
    velocities: Array
    gravity: Array
    unconstrained_gravity: Array
    accel_bias: Array
    rank: int
    raw_condition: float
    column_scaled_condition: float
    position_residual_rms_m: float
    velocity_residual_rms_mps: float


@dataclass(frozen=True)
class EvaluationResult:
    pair_count: int
    duration_s: float
    inertial_prediction_rms_m: float
    constant_velocity_prediction_rms_m: float
    visual_observation_rms_m: float
    inertial_visual_disagreement_rms_m: float
    constant_velocity_visual_disagreement_rms_m: float
    velocity_error_rms_mps: float
    velocity_error_end_mps: float


def _finite(value: str, field_name: str, path: Path, line: int) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ProbeError(f"{path}:{line}: invalid {field_name}") from exc
    if not math.isfinite(parsed):
        raise ProbeError(f"{path}:{line}: {field_name} must be finite")
    return parsed


def _timestamp_ns(value: str, path: Path, line: int) -> int:
    try:
        timestamp = Decimal(value) * Decimal(1_000_000_000)
    except InvalidOperation as exc:
        raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
    if timestamp != timestamp.to_integral_value():
        raise ProbeError(f"{path}:{line}: timestamp is not integral nanoseconds")
    return int(timestamp)


def _normalized(vector: Array, label: str) -> Array:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{label} must have finite non-zero norm")
    return vector / norm


def quaternion_to_matrix(quaternion: Array) -> Array:
    q = _normalized(np.asarray(quaternion, dtype=float), "quaternion")
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _quaternion_slerp(left: Array, right: Array, alpha: float) -> Array:
    q_left = _normalized(np.asarray(left, dtype=float), "left quaternion")
    q_right = _normalized(np.asarray(right, dtype=float), "right quaternion")
    dot = float(np.dot(q_left, q_right))
    if dot < 0.0:
        q_right = -q_right
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _normalized(
            q_left + alpha * (q_right - q_left), "interpolated quaternion"
        )
    theta = math.acos(dot)
    sine = math.sin(theta)
    return (
        math.sin((1.0 - alpha) * theta) / sine * q_left
        + math.sin(alpha * theta) / sine * q_right
    )


def _so3_exp(rotation_vector: Array) -> Array:
    value = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(value))
    if angle < 1e-12:
        skew = np.array(
            [
                [0.0, -value[2], value[1]],
                [value[2], 0.0, -value[0]],
                [-value[1], value[0], 0.0],
            ]
        )
        return np.eye(3) + skew + 0.5 * skew @ skew
    axis = value / angle
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _rotation_angle(rotation: Array) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def integrate_interval(
    samples: tuple[ImuSample, ...],
    gyro_bias: Array,
    accel_bias_at: BiasAt,
) -> PreintegratedInterval:
    if len(samples) < 2:
        raise ValueError("preintegration needs at least two IMU samples")
    bias_gyro = np.asarray(gyro_bias, dtype=float)
    if bias_gyro.shape != (3,) or not np.all(np.isfinite(bias_gyro)):
        raise ValueError("gyro bias must be a finite 3-vector")
    delta_rotation = np.eye(3)
    delta_velocity = np.zeros(3)
    delta_position = np.zeros(3)
    jacobian_velocity_bias_acc = np.zeros((3, 3))
    jacobian_position_bias_acc = np.zeros((3, 3))
    for left, right in zip(samples, samples[1:]):
        delta_ns = right.timestamp_ns - left.timestamp_ns
        if delta_ns <= 0:
            raise ValueError("IMU timestamps must be strictly increasing")
        delta_s = float(delta_ns) * 1e-9
        bias_acc = np.asarray(accel_bias_at(left.timestamp_ns), dtype=float)
        if bias_acc.shape != (3,) or not np.all(np.isfinite(bias_acc)):
            raise ValueError("accelerometer bias must be a finite 3-vector")
        corrected_accel = left.accel - bias_acc
        accel_i = delta_rotation @ corrected_accel
        jacobian_position_bias_acc += (
            jacobian_velocity_bias_acc * delta_s
            - 0.5 * delta_rotation * delta_s * delta_s
        )
        jacobian_velocity_bias_acc -= delta_rotation * delta_s
        delta_position += (
            delta_velocity * delta_s + 0.5 * accel_i * delta_s * delta_s
        )
        delta_velocity += accel_i * delta_s
        delta_rotation = delta_rotation @ _so3_exp(
            (left.gyro - bias_gyro) * delta_s
        )
    return PreintegratedInterval(
        samples[0].timestamp_ns,
        samples[-1].timestamp_ns,
        delta_position,
        delta_velocity,
        delta_rotation,
        jacobian_position_bias_acc,
        jacobian_velocity_bias_acc,
    )


def _alignment_system(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
) -> tuple[Array, Array]:
    pose_count = len(poses)
    unknown_count = 3 * pose_count + 3
    matrix = np.zeros((6 * len(intervals), unknown_count))
    vector = np.zeros(6 * len(intervals))
    identity = np.eye(3)
    gravity_column = 3 * pose_count
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], intervals)
    ):
        if interval.t_i_ns != pose_i.timestamp_ns or interval.t_j_ns != pose_j.timestamp_ns:
            raise ValueError("preintegrated interval does not match pose timestamps")
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        if delta_s <= 0.0:
            raise ValueError("pose timestamps must be strictly increasing")
        position_rows = slice(6 * index, 6 * index + 3)
        velocity_rows = slice(6 * index + 3, 6 * index + 6)
        velocity_i = slice(3 * index, 3 * index + 3)
        velocity_j = slice(3 * (index + 1), 3 * (index + 1) + 3)
        matrix[position_rows, velocity_i] = delta_s * identity
        matrix[position_rows, gravity_column:] = 0.5 * delta_s * delta_s * identity
        vector[position_rows] = (
            pose_j.position
            - pose_i.position
            - pose_i.rotation @ interval.delta_p
        )
        matrix[velocity_rows, velocity_i] = -identity
        matrix[velocity_rows, velocity_j] = identity
        matrix[velocity_rows, gravity_column:] = -delta_s * identity
        vector[velocity_rows] = pose_i.rotation @ interval.delta_v
    return matrix, vector


def _tangent_basis(gravity: Array) -> Array:
    direction = _normalized(gravity, "gravity")
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(seed, direction))) > 0.8:
        seed = np.array([0.0, 1.0, 0.0])
    first = _normalized(seed - direction * np.dot(seed, direction), "gravity tangent")
    second = np.cross(direction, first)
    return np.column_stack((first, second))


def _fixed_norm_system(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
    gravity: Array,
) -> tuple[Array, Array, Array]:
    pose_count = len(poses)
    tangent = _tangent_basis(gravity)
    matrix = np.zeros((6 * len(intervals), 3 * pose_count + 2))
    vector = np.zeros(6 * len(intervals))
    identity = np.eye(3)
    tangent_column = 3 * pose_count
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], intervals)
    ):
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_rows = slice(6 * index, 6 * index + 3)
        velocity_rows = slice(6 * index + 3, 6 * index + 6)
        velocity_i = slice(3 * index, 3 * index + 3)
        velocity_j = slice(3 * (index + 1), 3 * (index + 1) + 3)
        matrix[position_rows, velocity_i] = delta_s * identity
        matrix[position_rows, tangent_column:] = 0.5 * delta_s * delta_s * tangent
        vector[position_rows] = (
            pose_j.position
            - pose_i.position
            - pose_i.rotation @ interval.delta_p
            - 0.5 * gravity * delta_s * delta_s
        )
        matrix[velocity_rows, velocity_i] = -identity
        matrix[velocity_rows, velocity_j] = identity
        matrix[velocity_rows, tangent_column:] = -delta_s * tangent
        vector[velocity_rows] = (
            pose_i.rotation @ interval.delta_v + gravity * delta_s
        )
    return matrix, vector, tangent


def _solve_velocities(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
    gravity: Array,
) -> Array:
    pose_count = len(poses)
    matrix = np.zeros((6 * len(intervals), 3 * pose_count))
    vector = np.zeros(6 * len(intervals))
    identity = np.eye(3)
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], intervals)
    ):
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_rows = slice(6 * index, 6 * index + 3)
        velocity_rows = slice(6 * index + 3, 6 * index + 6)
        velocity_i = slice(3 * index, 3 * index + 3)
        velocity_j = slice(3 * (index + 1), 3 * (index + 1) + 3)
        matrix[position_rows, velocity_i] = delta_s * identity
        vector[position_rows] = (
            pose_j.position
            - pose_i.position
            - pose_i.rotation @ interval.delta_p
            - 0.5 * gravity * delta_s * delta_s
        )
        matrix[velocity_rows, velocity_i] = -identity
        matrix[velocity_rows, velocity_j] = identity
        vector[velocity_rows] = (
            pose_i.rotation @ interval.delta_v + gravity * delta_s
        )
    solution, _, rank, _ = np.linalg.lstsq(matrix, vector, rcond=None)
    if rank != matrix.shape[1]:
        raise ValueError("fixed-gravity velocity system is rank deficient")
    return solution.reshape((pose_count, 3))


def solve_alignment(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
    gravity_magnitude: float,
    refine_iterations: int = 4,
) -> AlignmentResult:
    if len(intervals) < 2:
        raise ValueError("alignment needs at least two intervals")
    if len(poses) != len(intervals) + 1:
        raise ValueError("alignment needs one more pose than interval")
    if not math.isfinite(gravity_magnitude) or gravity_magnitude <= 0.0:
        raise ValueError("gravity magnitude must be finite and positive")
    if refine_iterations < 1:
        raise ValueError("refine_iterations must be positive")
    matrix, vector = _alignment_system(poses, intervals)
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, vector, rcond=None)
    if rank != matrix.shape[1]:
        raise ValueError("velocity/gravity alignment system is rank deficient")
    unconstrained_gravity = solution[-3:]
    gravity = _normalized(unconstrained_gravity, "unconstrained gravity") * gravity_magnitude
    for _ in range(refine_iterations):
        refine_matrix, refine_vector, tangent = _fixed_norm_system(
            poses, intervals, gravity
        )
        refine_solution, _, refine_rank, _ = np.linalg.lstsq(
            refine_matrix, refine_vector, rcond=None
        )
        if refine_rank != refine_matrix.shape[1]:
            raise ValueError("fixed-norm gravity refinement is rank deficient")
        gravity = _normalized(
            gravity + tangent @ refine_solution[-2:], "refined gravity"
        ) * gravity_magnitude
    velocities = _solve_velocities(poses, intervals, gravity)

    position_residuals: list[float] = []
    velocity_residuals: list[float] = []
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], intervals)
    ):
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_residual = (
            velocities[index] * delta_s
            + 0.5 * gravity * delta_s * delta_s
            + pose_i.rotation @ interval.delta_p
            - (pose_j.position - pose_i.position)
        )
        velocity_residual = (
            velocities[index + 1]
            - velocities[index]
            - gravity * delta_s
            - pose_i.rotation @ interval.delta_v
        )
        position_residuals.append(float(np.dot(position_residual, position_residual)))
        velocity_residuals.append(float(np.dot(velocity_residual, velocity_residual)))

    column_norms = np.linalg.norm(matrix, axis=0)
    if np.any(column_norms <= 0.0):
        raise ValueError("alignment system has a zero column")
    scaled_singular_values = np.linalg.svd(
        matrix / column_norms[np.newaxis, :], compute_uv=False
    )
    return AlignmentResult(
        velocities=velocities,
        gravity=gravity,
        unconstrained_gravity=unconstrained_gravity,
        rank=int(rank),
        raw_condition=float(singular_values[0] / singular_values[-1]),
        column_scaled_condition=float(
            scaled_singular_values[0] / scaled_singular_values[-1]
        ),
        position_residual_rms_m=math.sqrt(
            sum(position_residuals) / float(len(position_residuals))
        ),
        velocity_residual_rms_mps=math.sqrt(
            sum(velocity_residuals) / float(len(velocity_residuals))
        ),
    )


def _fixed_norm_bias_system(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
    gravity: Array,
) -> tuple[Array, Array, Array]:
    pose_count = len(poses)
    tangent = _tangent_basis(gravity)
    tangent_column = 3 * pose_count
    bias_column = tangent_column + 2
    matrix = np.zeros((6 * len(intervals), bias_column + 3))
    vector = np.zeros(6 * len(intervals))
    identity = np.eye(3)
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], intervals)
    ):
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_rows = slice(6 * index, 6 * index + 3)
        velocity_rows = slice(6 * index + 3, 6 * index + 6)
        velocity_i = slice(3 * index, 3 * index + 3)
        velocity_j = slice(3 * (index + 1), 3 * (index + 1) + 3)
        matrix[position_rows, velocity_i] = delta_s * identity
        matrix[position_rows, tangent_column:bias_column] = (
            0.5 * delta_s * delta_s * tangent
        )
        matrix[position_rows, bias_column:] = (
            pose_i.rotation @ interval.jacobian_p_bias_acc
        )
        vector[position_rows] = (
            pose_j.position
            - pose_i.position
            - pose_i.rotation @ interval.delta_p
            - 0.5 * gravity * delta_s * delta_s
        )
        matrix[velocity_rows, velocity_i] = -identity
        matrix[velocity_rows, velocity_j] = identity
        matrix[velocity_rows, tangent_column:bias_column] = -delta_s * tangent
        matrix[velocity_rows, bias_column:] = (
            -pose_i.rotation @ interval.jacobian_v_bias_acc
        )
        vector[velocity_rows] = (
            pose_i.rotation @ interval.delta_v + gravity * delta_s
        )
    return matrix, vector, tangent


def _correct_intervals_for_bias(
    intervals: tuple[PreintegratedInterval, ...], delta_bias: Array
) -> tuple[PreintegratedInterval, ...]:
    return tuple(
        PreintegratedInterval(
            interval.t_i_ns,
            interval.t_j_ns,
            interval.delta_p + interval.jacobian_p_bias_acc @ delta_bias,
            interval.delta_v + interval.jacobian_v_bias_acc @ delta_bias,
            interval.delta_rotation,
            interval.jacobian_p_bias_acc,
            interval.jacobian_v_bias_acc,
        )
        for interval in intervals
    )


def solve_bias_alignment(
    poses: tuple[Pose, ...],
    intervals: tuple[PreintegratedInterval, ...],
    gravity_magnitude: float,
    accel_bias_seed: Array,
    refine_iterations: int = 4,
) -> BiasAlignmentResult:
    if len(intervals) < 3:
        raise ValueError("bias alignment needs at least three intervals")
    if len(poses) != len(intervals) + 1:
        raise ValueError("bias alignment needs one more pose than interval")
    bias = np.asarray(accel_bias_seed, dtype=float).copy()
    if bias.shape != (3,) or not np.all(np.isfinite(bias)):
        raise ValueError("accelerometer bias seed must be a finite 3-vector")
    initial = solve_alignment(
        poses, intervals, gravity_magnitude, refine_iterations
    )
    gravity = initial.gravity.copy()
    current_intervals = intervals
    rank = 0
    singular_values = np.array([])
    matrix = np.empty((0, 0))
    for _ in range(refine_iterations):
        matrix, vector, tangent = _fixed_norm_bias_system(
            poses, current_intervals, gravity
        )
        solution, _, rank, singular_values = np.linalg.lstsq(
            matrix, vector, rcond=None
        )
        if rank != matrix.shape[1]:
            raise ValueError("fixed-norm gravity/bias system is rank deficient")
        tangent_column = 3 * len(poses)
        delta_gravity = tangent @ solution[tangent_column : tangent_column + 2]
        delta_bias = solution[tangent_column + 2 : tangent_column + 5]
        gravity = _normalized(
            gravity + delta_gravity, "bias-refined gravity"
        ) * gravity_magnitude
        bias += delta_bias
        current_intervals = _correct_intervals_for_bias(
            current_intervals, delta_bias
        )
        if float(np.linalg.norm(delta_gravity)) < 1e-10 and float(
            np.linalg.norm(delta_bias)
        ) < 1e-10:
            break
    velocities = _solve_velocities(poses, current_intervals, gravity)
    position_residuals: list[float] = []
    velocity_residuals: list[float] = []
    for index, (pose_i, pose_j, interval) in enumerate(
        zip(poses, poses[1:], current_intervals)
    ):
        delta_s = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_residual = (
            velocities[index] * delta_s
            + 0.5 * gravity * delta_s * delta_s
            + pose_i.rotation @ interval.delta_p
            - (pose_j.position - pose_i.position)
        )
        velocity_residual = (
            velocities[index + 1]
            - velocities[index]
            - gravity * delta_s
            - pose_i.rotation @ interval.delta_v
        )
        position_residuals.append(float(np.dot(position_residual, position_residual)))
        velocity_residuals.append(float(np.dot(velocity_residual, velocity_residual)))
    column_norms = np.linalg.norm(matrix, axis=0)
    if np.any(column_norms <= 0.0):
        raise ValueError("bias alignment system has a zero column")
    scaled_singular_values = np.linalg.svd(
        matrix / column_norms[np.newaxis, :], compute_uv=False
    )
    return BiasAlignmentResult(
        velocities=velocities,
        gravity=gravity,
        unconstrained_gravity=initial.unconstrained_gravity,
        accel_bias=bias,
        rank=int(rank),
        raw_condition=float(singular_values[0] / singular_values[-1]),
        column_scaled_condition=float(
            scaled_singular_values[0] / scaled_singular_values[-1]
        ),
        position_residual_rms_m=math.sqrt(
            sum(position_residuals) / float(len(position_residuals))
        ),
        velocity_residual_rms_mps=math.sqrt(
            sum(velocity_residuals) / float(len(velocity_residuals))
        ),
    )


def load_tum_poses(path: Path) -> tuple[Pose, ...]:
    poses: list[Pose] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    for line_number, line in enumerate(lines, start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        row = stripped.split()
        if len(row) != 8:
            raise ProbeError(f"{path}:{line_number}: expected 8 TUM columns")
        timestamp = _timestamp_ns(row[0], path, line_number)
        position = np.array(
            [_finite(row[index], f"p_{'xyz'[index - 1]}", path, line_number) for index in range(1, 4)]
        )
        quaternion = np.array(
            [
                _finite(row[7], "q_w", path, line_number),
                _finite(row[4], "q_x", path, line_number),
                _finite(row[5], "q_y", path, line_number),
                _finite(row[6], "q_z", path, line_number),
            ]
        )
        if poses and timestamp <= poses[-1].timestamp_ns:
            raise ProbeError(f"{path}:{line_number}: timestamps are not strictly increasing")
        poses.append(Pose(timestamp, quaternion_to_matrix(quaternion), position))
    if len(poses) < 3:
        raise ProbeError(f"{path}: needs at least three poses")
    return tuple(poses)


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
            if len(row) < 7:
                raise ProbeError(f"{path}:{line}: expected at least 7 columns")
            try:
                timestamp = int(row[0])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
            gyro = np.array(
                [_finite(row[index], f"gyro_{'xyz'[index - 1]}", path, line) for index in range(1, 4)]
            )
            accel = np.array(
                [_finite(row[index], f"accel_{'xyz'[index - 4]}", path, line) for index in range(4, 7)]
            )
            if samples and timestamp <= samples[-1].timestamp_ns:
                raise ProbeError(f"{path}:{line}: timestamps are not strictly increasing")
            samples.append(ImuSample(timestamp, gyro, accel))
    if len(samples) < 2:
        raise ProbeError(f"{path}: needs at least two IMU samples")
    return tuple(samples)


def load_reference(path: Path) -> tuple[ReferenceState, ...]:
    states: list[ReferenceState] = []
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.reader(input_file)
        for line, row in enumerate(reader, start=1):
            if not row or row[0].startswith("#"):
                continue
            if len(row) < 17:
                raise ProbeError(f"{path}:{line}: expected at least 17 columns")
            try:
                timestamp = int(row[0])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp") from exc
            values = [_finite(value, f"column_{index}", path, line) for index, value in enumerate(row[1:17], start=1)]
            state = ReferenceState(
                timestamp,
                np.array(values[0:3]),
                _normalized(np.array(values[3:7]), "reference quaternion"),
                np.array(values[7:10]),
                np.array(values[10:13]),
                np.array(values[13:16]),
            )
            if states and timestamp <= states[-1].timestamp_ns:
                raise ProbeError(f"{path}:{line}: timestamps are not strictly increasing")
            states.append(state)
    if len(states) < 2:
        raise ProbeError(f"{path}: needs at least two reference states")
    return tuple(states)


def interpolate_reference(
    states: tuple[ReferenceState, ...], timestamps: tuple[int, ...], target_ns: int
) -> ReferenceState:
    right = bisect.bisect_left(timestamps, target_ns)
    if right < len(states) and states[right].timestamp_ns == target_ns:
        return states[right]
    if right == 0 or right == len(states):
        raise ProbeError(f"reference does not bracket timestamp {target_ns}")
    left = states[right - 1]
    right_state = states[right]
    alpha = float(target_ns - left.timestamp_ns) / float(
        right_state.timestamp_ns - left.timestamp_ns
    )

    def linear(left_value: Array, right_value: Array) -> Array:
        return left_value + alpha * (right_value - left_value)

    return ReferenceState(
        target_ns,
        linear(left.position, right_state.position),
        _quaternion_slerp(left.quaternion, right_state.quaternion, alpha),
        linear(left.velocity, right_state.velocity),
        linear(left.gyro_bias, right_state.gyro_bias),
        linear(left.accel_bias, right_state.accel_bias),
    )


def _interpolate_imu(
    samples: tuple[ImuSample, ...], timestamps: tuple[int, ...], target_ns: int
) -> ImuSample:
    right = bisect.bisect_left(timestamps, target_ns)
    if right < len(samples) and samples[right].timestamp_ns == target_ns:
        return samples[right]
    if right == 0 or right == len(samples):
        raise ProbeError(f"IMU does not bracket timestamp {target_ns}")
    left = samples[right - 1]
    right_sample = samples[right]
    alpha = float(target_ns - left.timestamp_ns) / float(
        right_sample.timestamp_ns - left.timestamp_ns
    )
    return ImuSample(
        target_ns,
        left.gyro + alpha * (right_sample.gyro - left.gyro),
        left.accel + alpha * (right_sample.accel - left.accel),
    )


def imu_segment(
    samples: tuple[ImuSample, ...], timestamps: tuple[int, ...], t_i_ns: int, t_j_ns: int
) -> tuple[ImuSample, ...]:
    if t_j_ns <= t_i_ns:
        raise ProbeError("IMU segment endpoints must be increasing")
    begin = bisect.bisect_right(timestamps, t_i_ns)
    end = bisect.bisect_left(timestamps, t_j_ns)
    return (
        _interpolate_imu(samples, timestamps, t_i_ns),
        *samples[begin:end],
        _interpolate_imu(samples, timestamps, t_j_ns),
    )


def build_intervals(
    poses: tuple[Pose, ...],
    imu_samples: tuple[ImuSample, ...],
    gyro_bias: Array,
    accel_bias_at: BiasAt,
) -> tuple[PreintegratedInterval, ...]:
    timestamps = tuple(sample.timestamp_ns for sample in imu_samples)
    return tuple(
        integrate_interval(
            imu_segment(
                imu_samples, timestamps, pose_i.timestamp_ns, pose_j.timestamp_ns
            ),
            gyro_bias,
            accel_bias_at,
        )
        for pose_i, pose_j in zip(poses, poses[1:])
    )


def _load_init_biases(path: Path) -> tuple[Array, Array]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.DictReader(input_file, skipinitialspace=True)
        required = {
            *(f"init_bg_{axis}" for axis in "xyz"),
            *(f"init_ba_{axis}" for axis in "xyz"),
        }
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ProbeError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if len(rows) != 1:
        raise ProbeError(f"{path}: expected exactly one init snapshot")
    gyro = np.array(
        [_finite(rows[0][f"init_bg_{axis}"], f"init_bg_{axis}", path, 2) for axis in "xyz"]
    )
    accel = np.array(
        [_finite(rows[0][f"init_ba_{axis}"], f"init_ba_{axis}", path, 2) for axis in "xyz"]
    )
    return gyro, accel


def _load_aligned_gyro_bias(path: Path) -> tuple[int, Array]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    with input_file:
        reader = csv.DictReader(input_file, skipinitialspace=True)
        required = {
            "timestamp_ns",
            "fusion_mode",
            *(f"pred_bg_{axis}" for axis in "xyz"),
        }
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ProbeError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line, row in enumerate(reader, start=2):
            if row["fusion_mode"] != "gyro_visual":
                continue
            try:
                timestamp = int(row["timestamp_ns"])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp_ns") from exc
            bias = np.array(
                [_finite(row[f"pred_bg_{axis}"], f"pred_bg_{axis}", path, line) for axis in "xyz"]
            )
            return timestamp, bias
    raise ProbeError(f"{path}: no gyro_visual row")


def _load_meta(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    if not isinstance(value, dict) or not isinstance(value.get("config"), dict):
        raise ProbeError(f"{path}: missing object config")
    return value


def _verify_identity_imu_extrinsic(path: Path) -> None:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    match = re.search(r"T_BS\s*:.*?data\s*:\s*\[([^\]]+)\]", text, re.DOTALL)
    if match is None:
        raise ProbeError(f"{path}: cannot parse T_BS.data")
    try:
        values = np.array([float(value) for value in match.group(1).split(",")])
    except ValueError as exc:
        raise ProbeError(f"{path}: invalid T_BS.data") from exc
    if values.size != 16 or not np.allclose(values.reshape((4, 4)), np.eye(4), atol=1e-12):
        raise ProbeError(
            f"{path}: this probe currently requires identity IMU T_BS"
        )


def _select_training_poses(
    poses: tuple[Pose, ...],
    train_end_ns: int,
    duration_s: float,
    stride: int,
    reference_start_ns: int,
) -> tuple[tuple[Pose, ...], int]:
    timestamps = tuple(pose.timestamp_ns for pose in poses)
    end_index = bisect.bisect_left(timestamps, train_end_ns)
    if end_index == len(poses) or timestamps[end_index] != train_end_ns:
        raise ProbeError(f"visual trajectory has no activation pose {train_end_ns}")
    requested_start = train_end_ns - int(round(duration_s * 1e9))
    start_index = bisect.bisect_left(timestamps, requested_start)
    if start_index >= end_index:
        raise ProbeError("requested training duration has no usable interval")
    if timestamps[start_index] < reference_start_ns:
        raise ProbeError(
            "requested training support starts before reference truth; reduce --train-duration-s"
        )
    indices = list(range(start_index, end_index, stride))
    if not indices or indices[-1] != end_index:
        indices.append(end_index)
    selected = tuple(poses[index] for index in indices)
    if len(selected) < 3:
        raise ProbeError("training selection needs at least three poses")
    return selected, end_index


def evaluate_alignment(
    all_poses: tuple[Pose, ...],
    train_end_index: int,
    intervals: tuple[PreintegratedInterval, ...],
    alignment: AlignmentResult,
    references: tuple[ReferenceState, ...],
    reference_timestamps: tuple[int, ...],
) -> EvaluationResult:
    if not intervals:
        raise ValueError("evaluation needs at least one interval")
    velocity = alignment.velocities[-1].copy()
    inertial_errors: list[float] = []
    constant_velocity_errors: list[float] = []
    visual_errors: list[float] = []
    inertial_visual_disagreements: list[float] = []
    constant_velocity_visual_disagreements: list[float] = []
    velocity_errors: list[float] = []
    for offset, interval in enumerate(intervals):
        index = train_end_index + offset
        pose_i = all_poses[index]
        pose_j = all_poses[index + 1]
        if interval.t_i_ns != pose_i.timestamp_ns or interval.t_j_ns != pose_j.timestamp_ns:
            raise ValueError("evaluation interval does not match visual poses")
        ref_i = interpolate_reference(references, reference_timestamps, pose_i.timestamp_ns)
        ref_j = interpolate_reference(references, reference_timestamps, pose_j.timestamp_ns)
        delta_s = float(pose_j.timestamp_ns - pose_i.timestamp_ns) * 1e-9
        truth_body_displacement = ref_i.rotation.T @ (
            ref_j.position - ref_i.position
        )
        inertial_body_displacement = (
            pose_i.rotation.T
            @ (velocity * delta_s + 0.5 * alignment.gravity * delta_s * delta_s)
            + interval.delta_p
        )
        visual_body_displacement = pose_i.rotation.T @ (
            pose_j.position - pose_i.position
        )
        previous_pose = all_poses[index - 1]
        previous_dt_s = float(pose_i.timestamp_ns - previous_pose.timestamp_ns) * 1e-9
        constant_velocity = (pose_i.position - previous_pose.position) / previous_dt_s
        constant_velocity_body_displacement = (
            pose_i.rotation.T @ (constant_velocity * delta_s)
        )
        inertial_errors.append(
            float(np.linalg.norm(inertial_body_displacement - truth_body_displacement))
        )
        constant_velocity_errors.append(
            float(
                np.linalg.norm(
                    constant_velocity_body_displacement - truth_body_displacement
                )
            )
        )
        visual_errors.append(
            float(np.linalg.norm(visual_body_displacement - truth_body_displacement))
        )
        inertial_visual_disagreements.append(
            float(
                np.linalg.norm(
                    inertial_body_displacement - visual_body_displacement
                )
            )
        )
        constant_velocity_visual_disagreements.append(
            float(
                np.linalg.norm(
                    constant_velocity_body_displacement
                    - visual_body_displacement
                )
            )
        )
        velocity_errors.append(
            float(
                np.linalg.norm(
                    pose_i.rotation.T @ velocity
                    - ref_i.rotation.T @ ref_i.velocity
                )
            )
        )
        velocity = (
            velocity
            + alignment.gravity * delta_s
            + pose_i.rotation @ interval.delta_v
        )
    final_pose = all_poses[train_end_index + len(intervals)]
    final_reference = interpolate_reference(
        references, reference_timestamps, final_pose.timestamp_ns
    )
    final_velocity_error = float(
        np.linalg.norm(
            final_pose.rotation.T @ velocity
            - final_reference.rotation.T @ final_reference.velocity
        )
    )

    def rms(values: list[float]) -> float:
        return math.sqrt(sum(value * value for value in values) / float(len(values)))

    return EvaluationResult(
        pair_count=len(intervals),
        duration_s=float(intervals[-1].t_j_ns - intervals[0].t_i_ns) * 1e-9,
        inertial_prediction_rms_m=rms(inertial_errors),
        constant_velocity_prediction_rms_m=rms(constant_velocity_errors),
        visual_observation_rms_m=rms(visual_errors),
        inertial_visual_disagreement_rms_m=rms(
            inertial_visual_disagreements
        ),
        constant_velocity_visual_disagreement_rms_m=rms(
            constant_velocity_visual_disagreements
        ),
        velocity_error_rms_mps=rms(velocity_errors),
        velocity_error_end_mps=final_velocity_error,
    )


def run_probe(arguments: argparse.Namespace) -> dict[str, object]:
    visual_run = arguments.visual_run.resolve()
    fusion_run = arguments.fusion_run.resolve()
    sequence_root = arguments.sequence_root.resolve()
    visual_meta = _load_meta(visual_run / "meta.json")
    fusion_meta = _load_meta(fusion_run / "meta.json")
    visual_config = visual_meta["config"]
    fusion_config = fusion_meta["config"]
    assert isinstance(visual_config, dict)
    assert isinstance(fusion_config, dict)
    if visual_config.get("estimator.enable_imu") is not False:
        raise ProbeError("visual run must be an IMU-off run")
    if fusion_config.get("estimator.enable_imu") is not True:
        raise ProbeError("fusion run must be an IMU-on run")
    if visual_meta.get("sequence") != fusion_meta.get("sequence"):
        raise ProbeError("visual and fusion runs have different sequences")
    gravity_magnitude = fusion_config.get("estimator.imu_gravity")
    if not isinstance(gravity_magnitude, (int, float)):
        raise ProbeError("fusion meta is missing estimator.imu_gravity")
    gravity_magnitude = float(gravity_magnitude)

    _verify_identity_imu_extrinsic(sequence_root / "mav0/imu0/sensor.yaml")
    all_poses = load_tum_poses(visual_run / "est.tum")
    imu_samples = load_imu(sequence_root / "mav0/imu0/data.csv")
    references = load_reference(
        sequence_root / "mav0/state_groundtruth_estimate0/data.csv"
    )
    reference_timestamps = tuple(state.timestamp_ns for state in references)
    _, static_accel_bias = _load_init_biases(fusion_run / "vio_init.csv")
    activation_ns, aligned_gyro_bias = _load_aligned_gyro_bias(
        fusion_run / "vio_state.csv"
    )
    train_end_ns = activation_ns + int(
        round(arguments.train_end_offset_s * 1e9)
    )
    training_poses, train_end_index = _select_training_poses(
        all_poses,
        train_end_ns,
        arguments.train_duration_s,
        arguments.train_stride,
        references[0].timestamp_ns,
    )

    if arguments.acc_bias_mode == "zero":
        accel_bias_at: BiasAt = lambda _: np.zeros(3)
        runtime_eligible = True
    elif arguments.acc_bias_mode == "static":
        accel_bias_at = lambda _: static_accel_bias
        runtime_eligible = True
    elif arguments.acc_bias_mode == "estimated":
        accel_bias_at = lambda _: static_accel_bias
        runtime_eligible = True
    elif arguments.acc_bias_mode == "reference_constant":
        reference_accel_bias = interpolate_reference(
            references, reference_timestamps, train_end_ns
        ).accel_bias
        accel_bias_at = lambda _: reference_accel_bias
        runtime_eligible = False
    else:
        accel_bias_at = lambda timestamp_ns: interpolate_reference(
            references, reference_timestamps, timestamp_ns
        ).accel_bias
        runtime_eligible = False

    training_intervals = build_intervals(
        training_poses, imu_samples, aligned_gyro_bias, accel_bias_at
    )
    if arguments.acc_bias_mode == "estimated":
        alignment = solve_bias_alignment(
            training_poses,
            training_intervals,
            gravity_magnitude,
            static_accel_bias,
        )
        effective_accel_bias = alignment.accel_bias
        accel_bias_at = lambda _: effective_accel_bias
    else:
        alignment = solve_alignment(
            training_poses, training_intervals, gravity_magnitude
        )
        effective_accel_bias = accel_bias_at(train_end_ns)

    requested_eval_end = train_end_ns + int(
        round(arguments.eval_duration_s * 1e9)
    )
    max_eval_end = min(requested_eval_end, references[-1].timestamp_ns)
    eval_end_index = bisect.bisect_right(
        tuple(pose.timestamp_ns for pose in all_poses), max_eval_end
    ) - 1
    if eval_end_index <= train_end_index:
        raise ProbeError("evaluation support has no complete visual interval")
    evaluation_poses = all_poses[train_end_index : eval_end_index + 1]
    evaluation_intervals = build_intervals(
        evaluation_poses, imu_samples, aligned_gyro_bias, accel_bias_at
    )
    evaluation = evaluate_alignment(
        all_poses,
        train_end_index,
        evaluation_intervals,
        alignment,
        references,
        reference_timestamps,
    )

    end_pose = training_poses[-1]
    end_reference = interpolate_reference(
        references, reference_timestamps, end_pose.timestamp_ns
    )
    reference_gravity_world = np.array([0.0, 0.0, -gravity_magnitude])
    gravity_body = end_pose.rotation.T @ alignment.gravity
    reference_gravity_body = end_reference.rotation.T @ reference_gravity_world
    gravity_cosine = float(
        np.clip(
            np.dot(gravity_body, reference_gravity_body)
            / (np.linalg.norm(gravity_body) * np.linalg.norm(reference_gravity_body)),
            -1.0,
            1.0,
        )
    )
    velocity_body = end_pose.rotation.T @ alignment.velocities[-1]
    reference_velocity_body = end_reference.rotation.T @ end_reference.velocity
    rotation_residuals = [
        _rotation_angle(
            interval.delta_rotation.T
            @ (pose_i.rotation.T @ pose_j.rotation)
        )
        for pose_i, pose_j, interval in zip(
            training_poses, training_poses[1:], training_intervals
        )
    ]
    reference_accel_bias_end = end_reference.accel_bias
    result: dict[str, object] = {
        "schema_version": 1,
        "visual_run": str(visual_run),
        "fusion_run": str(fusion_run),
        "sequence_root": str(sequence_root),
        "sequence": visual_meta.get("sequence"),
        "visual_config_hash": visual_meta.get("config_hash"),
        "fusion_config_hash": fusion_meta.get("config_hash"),
        "acc_bias_mode": arguments.acc_bias_mode,
        "runtime_eligible": runtime_eligible,
        "train_requested_duration_s": arguments.train_duration_s,
        "train_actual_duration_s": float(
            training_poses[-1].timestamp_ns - training_poses[0].timestamp_ns
        )
        * 1e-9,
        "train_stride": arguments.train_stride,
        "train_pose_count": len(training_poses),
        "train_interval_count": len(training_intervals),
        "train_start_ns": training_poses[0].timestamp_ns,
        "train_end_ns": training_poses[-1].timestamp_ns,
        "gyro_activation_ns": activation_ns,
        "train_end_offset_s": arguments.train_end_offset_s,
        "gyro_bias_rps": aligned_gyro_bias.tolist(),
        "static_accel_bias_mps2": static_accel_bias.tolist(),
        "effective_accel_bias_mps2": effective_accel_bias.tolist(),
        "effective_accel_bias_error_end_mps2": float(
            np.linalg.norm(effective_accel_bias - reference_accel_bias_end)
        ),
        "reference_accel_bias_end_mps2": reference_accel_bias_end.tolist(),
        "alignment_rank": alignment.rank,
        "alignment_unknown_count": 3 * len(training_poses)
        + (5 if arguments.acc_bias_mode == "estimated" else 3),
        "alignment_raw_condition": alignment.raw_condition,
        "alignment_column_scaled_condition": alignment.column_scaled_condition,
        "training_position_residual_rms_m": alignment.position_residual_rms_m,
        "training_velocity_residual_rms_mps": alignment.velocity_residual_rms_mps,
        "training_rotation_residual_rms_rad": math.sqrt(
            sum(value * value for value in rotation_residuals)
            / float(len(rotation_residuals))
        ),
        "unconstrained_gravity_mps2": alignment.unconstrained_gravity.tolist(),
        "unconstrained_gravity_norm_mps2": float(
            np.linalg.norm(alignment.unconstrained_gravity)
        ),
        "refined_gravity_mps2": alignment.gravity.tolist(),
        "refined_gravity_body_mps2": gravity_body.tolist(),
        "reference_gravity_body_mps2": reference_gravity_body.tolist(),
        "gravity_tilt_error_deg": math.degrees(math.acos(gravity_cosine)),
        "train_end_velocity_body_mps": velocity_body.tolist(),
        "reference_train_end_velocity_body_mps": reference_velocity_body.tolist(),
        "train_end_velocity_error_mps": float(
            np.linalg.norm(velocity_body - reference_velocity_body)
        ),
        "eval_requested_duration_s": arguments.eval_duration_s,
        "eval_actual_duration_s": evaluation.duration_s,
        "eval_pair_count": evaluation.pair_count,
        "eval_inertial_prediction_rms_m": evaluation.inertial_prediction_rms_m,
        "eval_constant_velocity_prediction_rms_m": evaluation.constant_velocity_prediction_rms_m,
        "eval_visual_observation_rms_m": evaluation.visual_observation_rms_m,
        "eval_inertial_visual_disagreement_rms_m": (
            evaluation.inertial_visual_disagreement_rms_m
        ),
        "eval_constant_velocity_visual_disagreement_rms_m": (
            evaluation.constant_velocity_visual_disagreement_rms_m
        ),
        "eval_velocity_error_rms_mps": evaluation.velocity_error_rms_mps,
        "eval_velocity_error_end_mps": evaluation.velocity_error_end_mps,
        "eval_inertial_better_than_constant_velocity": (
            evaluation.inertial_prediction_rms_m
            < evaluation.constant_velocity_prediction_rms_m
        ),
        "eval_inertial_better_than_visual_observation": (
            evaluation.inertial_prediction_rms_m
            < evaluation.visual_observation_rms_m
        ),
    }
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("visual_run", type=Path)
    parser.add_argument("fusion_run", type=Path)
    parser.add_argument("sequence_root", type=Path)
    parser.add_argument("--train-duration-s", type=float, default=20.0)
    parser.add_argument("--eval-duration-s", type=float, default=20.0)
    parser.add_argument("--train-stride", type=int, default=10)
    parser.add_argument("--train-end-offset-s", type=float, default=0.0)
    parser.add_argument(
        "--acc-bias-mode",
        choices=(
            "zero",
            "static",
            "estimated",
            "reference_constant",
            "reference",
        ),
        default="zero",
    )
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[list[str]] = None, stdout: TextIO = sys.stdout) -> int:
    parser = _parser()
    arguments = parser.parse_args(argv)
    if (
        not math.isfinite(arguments.train_duration_s)
        or arguments.train_duration_s <= 0.0
    ):
        parser.error("--train-duration-s must be finite and positive")
    if (
        not math.isfinite(arguments.eval_duration_s)
        or arguments.eval_duration_s <= 0.0
    ):
        parser.error("--eval-duration-s must be finite and positive")
    if arguments.train_stride < 1:
        parser.error("--train-stride must be positive")
    if not math.isfinite(arguments.train_end_offset_s):
        parser.error("--train-end-offset-s must be finite")
    try:
        result = run_probe(arguments)
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if arguments.output is None:
            stdout.write(serialized)
        else:
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(serialized, encoding="utf-8")
    except (ProbeError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"imu translation alignment probe: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Audit full-VIO window-boundary state with rolling PIM counterfactuals."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO

import numpy as np

from imu_translation_alignment_probe import (
    PreintegratedInterval,
    ProbeError as ImuProbeError,
    ReferenceState,
    imu_segment,
    integrate_interval,
    interpolate_reference,
    load_imu,
    load_reference,
    load_tum_poses,
    quaternion_to_matrix,
)


Array = np.ndarray

FULL_STATE_FIELDS = (
    "timestamp_ns",
    "fusion_mode",
    "post_p_x",
    "post_p_y",
    "post_p_z",
    "post_q_x",
    "post_q_y",
    "post_q_z",
    "post_q_w",
    "velocity_x",
    "velocity_y",
    "velocity_z",
    "gravity_x",
    "gravity_y",
    "gravity_z",
    "post_bg_x",
    "post_bg_y",
    "post_bg_z",
    "post_ba_x",
    "post_ba_y",
    "post_ba_z",
)

ARMS = (
    "actual",
    "gt_velocity",
    "gt_gravity",
    "gt_bias",
    "gt_velocity_gravity",
    "model_floor",
)


class ProbeError(RuntimeError):
    """An input or derived value violates the probe contract."""


@dataclass(frozen=True)
class FullState:
    timestamp_ns: int
    position: Array
    rotation: Array
    velocity: Array
    gravity: Array
    gyro_bias: Array
    accel_bias: Array


@dataclass(frozen=True)
class RigidTransform:
    rotation: Array
    translation: Array

    def apply(self, points: Array) -> Array:
        values = np.asarray(points, dtype=float)
        return (self.rotation @ values.T).T + self.translation

    def rmse(self, estimate: Array, reference: Array) -> float:
        residual = self.apply(estimate) - np.asarray(reference, dtype=float)
        return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))


@dataclass(frozen=True)
class CounterfactualSample:
    timestamp_ns: int
    errors: dict[str, float]
    position_error_m: float
    velocity_body_error_mps: float
    gravity_body_error_deg: float
    gyro_bias_error_rps: float
    accel_bias_error_mps2: float


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
) -> Array:
    return np.array([_finite(row[field], field, path, line) for field in fields])


def _rotation_angle_deg(rotation: Array) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _vector_angle_deg(left: Array, right: Array) -> float:
    left_norm = float(np.linalg.norm(left))
    right_norm = float(np.linalg.norm(right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise ProbeError("cannot compare zero-length directions")
    cosine = float(np.clip(np.dot(left, right) / (left_norm * right_norm), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def fit_rigid(estimate: Array, reference: Array) -> RigidTransform:
    estimate_value = np.asarray(estimate, dtype=float)
    reference_value = np.asarray(reference, dtype=float)
    if (
        estimate_value.ndim != 2
        or estimate_value.shape[1:] != (3,)
        or estimate_value.shape != reference_value.shape
        or estimate_value.shape[0] < 3
        or not np.all(np.isfinite(estimate_value))
        or not np.all(np.isfinite(reference_value))
    ):
        raise ProbeError("rigid alignment needs matching finite Nx3 arrays")
    estimate_centered = estimate_value - estimate_value.mean(axis=0)
    reference_centered = reference_value - reference_value.mean(axis=0)
    if np.linalg.matrix_rank(estimate_centered) < 2:
        raise ProbeError("rigid alignment support is degenerate")
    left, _, right_transpose = np.linalg.svd(
        estimate_centered.T @ reference_centered
    )
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = reference_value.mean(axis=0) - rotation @ estimate_value.mean(
        axis=0
    )
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ProbeError("rigid alignment produced a non-finite transform")
    return RigidTransform(rotation, translation)


def load_full_states(path: Path) -> tuple[FullState, ...]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc
    states: list[FullState] = []
    previous: Optional[int] = None
    with input_file:
        reader = csv.DictReader(input_file)
        fields = tuple(reader.fieldnames or ())
        if len(fields) != len(set(fields)):
            raise ProbeError(f"{path}: duplicate sidecar column")
        missing = sorted(set(FULL_STATE_FIELDS) - set(fields))
        if missing:
            raise ProbeError(f"{path}: missing columns: {', '.join(missing)}")
        for line, row in enumerate(reader, start=2):
            try:
                timestamp_ns = int(row["timestamp_ns"])
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid timestamp_ns") from exc
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{line}: timestamps must be strictly increasing"
                )
            previous = timestamp_ns
            if row["fusion_mode"] != "full_visual_inertial":
                continue
            quaternion = np.array(
                [
                    _finite(row["post_q_w"], "post_q_w", path, line),
                    _finite(row["post_q_x"], "post_q_x", path, line),
                    _finite(row["post_q_y"], "post_q_y", path, line),
                    _finite(row["post_q_z"], "post_q_z", path, line),
                ]
            )
            try:
                rotation = quaternion_to_matrix(quaternion)
            except ValueError as exc:
                raise ProbeError(f"{path}:{line}: invalid posterior quaternion") from exc
            state = FullState(
                timestamp_ns=timestamp_ns,
                position=_vector(
                    row, ("post_p_x", "post_p_y", "post_p_z"), path, line
                ),
                rotation=rotation,
                velocity=_vector(
                    row, ("velocity_x", "velocity_y", "velocity_z"), path, line
                ),
                gravity=_vector(
                    row, ("gravity_x", "gravity_y", "gravity_z"), path, line
                ),
                gyro_bias=_vector(
                    row, ("post_bg_x", "post_bg_y", "post_bg_z"), path, line
                ),
                accel_bias=_vector(
                    row, ("post_ba_x", "post_ba_y", "post_ba_z"), path, line
                ),
            )
            if float(np.linalg.norm(state.gravity)) <= 0.0:
                raise ProbeError(f"{path}:{line}: gravity must be non-zero")
            states.append(state)
    if not states:
        raise ProbeError(f"{path}: no full_visual_inertial rows")
    return tuple(states)


def _reference_at(
    states: tuple[ReferenceState, ...],
    timestamps: tuple[int, ...],
    target_ns: int,
    max_bracket_ns: int,
) -> ReferenceState:
    right = bisect.bisect_left(timestamps, target_ns)
    if right < len(states) and timestamps[right] == target_ns:
        return states[right]
    if right == 0 or right == len(states):
        raise ProbeError(f"groundtruth does not bracket {target_ns}")
    span_ns = timestamps[right] - timestamps[right - 1]
    if span_ns > max_bracket_ns:
        raise ProbeError(
            f"groundtruth bracket {span_ns} ns exceeds {max_bracket_ns} ns"
        )
    return interpolate_reference(states, timestamps, target_ns)


def counterfactual_errors(
    candidate: FullState,
    reference_start: ReferenceState,
    reference_end: ReferenceState,
    candidate_interval: PreintegratedInterval,
    reference_interval: PreintegratedInterval,
    *,
    gravity_magnitude: float,
) -> dict[str, float]:
    if not math.isfinite(gravity_magnitude) or gravity_magnitude <= 0.0:
        raise ProbeError("gravity magnitude must be finite and positive")
    if (
        candidate.timestamp_ns != reference_start.timestamp_ns
        or candidate_interval.t_i_ns != candidate.timestamp_ns
        or reference_interval.t_i_ns != candidate.timestamp_ns
        or candidate_interval.t_j_ns != reference_end.timestamp_ns
        or reference_interval.t_j_ns != reference_end.timestamp_ns
    ):
        raise ProbeError("counterfactual timestamps do not match")
    delta_s = float(reference_end.timestamp_ns - candidate.timestamp_ns) * 1e-9
    if delta_s <= 0.0:
        raise ProbeError("counterfactual horizon must be positive")
    true_displacement = reference_start.rotation.T @ (
        reference_end.position - reference_start.position
    )
    candidate_velocity = candidate.rotation.T @ candidate.velocity
    candidate_gravity = candidate.rotation.T @ candidate.gravity
    reference_velocity = reference_start.rotation.T @ reference_start.velocity
    reference_gravity = reference_start.rotation.T @ np.array(
        [0.0, 0.0, -gravity_magnitude]
    )
    predictions = {
        "actual": candidate_velocity * delta_s
        + 0.5 * candidate_gravity * delta_s * delta_s
        + candidate_interval.delta_p,
        "gt_velocity": reference_velocity * delta_s
        + 0.5 * candidate_gravity * delta_s * delta_s
        + candidate_interval.delta_p,
        "gt_gravity": candidate_velocity * delta_s
        + 0.5 * reference_gravity * delta_s * delta_s
        + candidate_interval.delta_p,
        "gt_bias": candidate_velocity * delta_s
        + 0.5 * candidate_gravity * delta_s * delta_s
        + reference_interval.delta_p,
        "gt_velocity_gravity": reference_velocity * delta_s
        + 0.5 * reference_gravity * delta_s * delta_s
        + candidate_interval.delta_p,
        "model_floor": reference_velocity * delta_s
        + 0.5 * reference_gravity * delta_s * delta_s
        + reference_interval.delta_p,
    }
    return {
        name: float(np.linalg.norm(value - true_displacement))
        for name, value in predictions.items()
    }


def _trajectory_phase(
    positions: dict[int, Array],
    timestamps: tuple[int, ...],
    references: dict[int, Array],
    first_full_ns: int,
) -> tuple[dict[str, object], RigidTransform]:
    prefix = tuple(timestamp for timestamp in timestamps if timestamp < first_full_ns)
    suffix = tuple(timestamp for timestamp in timestamps if timestamp >= first_full_ns)
    if len(prefix) < 3 or len(suffix) < 3:
        raise ProbeError("phase audit needs at least three prefix and suffix poses")

    def arrays(selected: tuple[int, ...]) -> tuple[Array, Array]:
        return (
            np.stack([positions[timestamp] for timestamp in selected]),
            np.stack([references[timestamp] for timestamp in selected]),
        )

    all_estimate, all_reference = arrays(timestamps)
    prefix_estimate, prefix_reference = arrays(prefix)
    suffix_estimate, suffix_reference = arrays(suffix)
    all_transform = fit_rigid(all_estimate, all_reference)
    prefix_transform = fit_rigid(prefix_estimate, prefix_reference)
    suffix_transform = fit_rigid(suffix_estimate, suffix_reference)
    anchor = suffix_estimate[0]
    phase = {
        "poses": len(timestamps),
        "prefix_poses": len(prefix),
        "suffix_poses": len(suffix),
        "all_self_ate_rmse_m": all_transform.rmse(
            all_estimate, all_reference
        ),
        "prefix_self_ate_rmse_m": prefix_transform.rmse(
            prefix_estimate, prefix_reference
        ),
        "suffix_self_ate_rmse_m": suffix_transform.rmse(
            suffix_estimate, suffix_reference
        ),
        "prefix_fit_suffix_ate_rmse_m": prefix_transform.rmse(
            suffix_estimate, suffix_reference
        ),
        "suffix_fit_prefix_ate_rmse_m": suffix_transform.rmse(
            prefix_estimate, prefix_reference
        ),
        "prefix_suffix_rotation_delta_deg": _rotation_angle_deg(
            suffix_transform.rotation @ prefix_transform.rotation.T
        ),
        "prefix_suffix_anchor_gap_m": float(
            np.linalg.norm(
                prefix_transform.apply(anchor[np.newaxis, :])[0]
                - suffix_transform.apply(anchor[np.newaxis, :])[0]
            )
        ),
        "prefix_suffix_translation_parameter_gap_m": float(
            np.linalg.norm(
                suffix_transform.translation - prefix_transform.translation
            )
        ),
    }
    return phase, prefix_transform


def _rms(values: list[float]) -> float:
    if not values:
        raise ProbeError("cannot summarize empty values")
    return math.sqrt(math.fsum(value * value for value in values) / len(values))


def _summary(values: list[float]) -> dict[str, float]:
    if not values:
        raise ProbeError("cannot summarize empty values")
    return {
        "rmse": _rms(values),
        "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)),
        "max": max(values),
    }


def _summarize_samples(
    samples: list[CounterfactualSample],
) -> dict[str, object]:
    arms = {
        arm: _summary([sample.errors[arm] for sample in samples]) for arm in ARMS
    }
    actual = arms["actual"]["rmse"]
    floor = arms["model_floor"]["rmse"]
    excess = max(0.0, actual - floor)
    explained: dict[str, float] = {}
    for arm in ("gt_velocity", "gt_gravity", "gt_bias", "gt_velocity_gravity"):
        explained[arm] = (
            0.0 if excess <= 0.0 else (actual - arms[arm]["rmse"]) / excess
        )
    return {
        "sample_count": len(samples),
        "arms": arms,
        "actual_minus_model_floor_rmse_m": excess,
        "excess_explained_fraction": explained,
        "state": {
            "position_prefix_fit_error_m": _summary(
                [sample.position_error_m for sample in samples]
            ),
            "velocity_body_error_mps": _summary(
                [sample.velocity_body_error_mps for sample in samples]
            ),
            "gravity_body_error_deg": _summary(
                [sample.gravity_body_error_deg for sample in samples]
            ),
            "gyro_bias_error_rps": _summary(
                [sample.gyro_bias_error_rps for sample in samples]
            ),
            "accel_bias_error_mps2": _summary(
                [sample.accel_bias_error_mps2 for sample in samples]
            ),
        },
    }


def _evaluate_horizon(
    full_states: tuple[FullState, ...],
    references: tuple[ReferenceState, ...],
    imu_samples: tuple,
    prefix_transform: RigidTransform,
    *,
    first_full_ns: int,
    horizon_ns: int,
    bucket_ns: int,
    max_bracket_ns: int,
    gravity_magnitude: float,
) -> dict[str, object]:
    reference_timestamps = tuple(state.timestamp_ns for state in references)
    imu_timestamps = tuple(sample.timestamp_ns for sample in imu_samples)
    samples: list[CounterfactualSample] = []
    for state in full_states:
        end_ns = state.timestamp_ns + horizon_ns
        if (
            state.timestamp_ns < reference_timestamps[0]
            or end_ns > reference_timestamps[-1]
            or state.timestamp_ns < imu_timestamps[0]
            or end_ns > imu_timestamps[-1]
        ):
            continue
        reference_start = _reference_at(
            references,
            reference_timestamps,
            state.timestamp_ns,
            max_bracket_ns,
        )
        reference_end = _reference_at(
            references, reference_timestamps, end_ns, max_bracket_ns
        )
        segment = imu_segment(
            imu_samples, imu_timestamps, state.timestamp_ns, end_ns
        )
        candidate_interval = integrate_interval(
            segment, state.gyro_bias, lambda _: state.accel_bias
        )
        reference_interval = integrate_interval(
            segment,
            reference_start.gyro_bias,
            lambda timestamp_ns: _reference_at(
                references,
                reference_timestamps,
                timestamp_ns,
                max_bracket_ns,
            ).accel_bias,
        )
        errors = counterfactual_errors(
            state,
            reference_start,
            reference_end,
            candidate_interval,
            reference_interval,
            gravity_magnitude=gravity_magnitude,
        )
        state_position = prefix_transform.apply(state.position[np.newaxis, :])[0]
        velocity_error = float(
            np.linalg.norm(
                state.rotation.T @ state.velocity
                - reference_start.rotation.T @ reference_start.velocity
            )
        )
        gravity_error = _vector_angle_deg(
            state.rotation.T @ state.gravity,
            reference_start.rotation.T
            @ np.array([0.0, 0.0, -gravity_magnitude]),
        )
        samples.append(
            CounterfactualSample(
                timestamp_ns=state.timestamp_ns,
                errors=errors,
                position_error_m=float(
                    np.linalg.norm(state_position - reference_start.position)
                ),
                velocity_body_error_mps=velocity_error,
                gravity_body_error_deg=gravity_error,
                gyro_bias_error_rps=float(
                    np.linalg.norm(state.gyro_bias - reference_start.gyro_bias)
                ),
                accel_bias_error_mps2=float(
                    np.linalg.norm(state.accel_bias - reference_start.accel_bias)
                ),
            )
        )
    if not samples:
        raise ProbeError("horizon has no GT/IMU-supported full states")

    buckets: list[dict[str, object]] = []
    bucket_ids = sorted(
        {
            (sample.timestamp_ns - first_full_ns) // bucket_ns
            for sample in samples
        }
    )
    for bucket in bucket_ids:
        selected = [
            sample
            for sample in samples
            if (sample.timestamp_ns - first_full_ns) // bucket_ns == bucket
        ]
        summary = _summarize_samples(selected)
        summary.update(
            {
                "bucket": int(bucket),
                "start_ns": first_full_ns + bucket * bucket_ns,
                "end_ns": first_full_ns + (bucket + 1) * bucket_ns,
            }
        )
        buckets.append(summary)
    result = _summarize_samples(samples)
    result.update(
        {
            "horizon_ns": horizon_ns,
            "horizon_s": horizon_ns * 1e-9,
            "support_start_ns": samples[0].timestamp_ns,
            "support_end_ns": samples[-1].timestamp_ns + horizon_ns,
            "buckets": buckets,
        }
    )
    return result


def _load_meta(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProbeError(f"cannot load {path}: {exc}") from exc
    if not isinstance(value, dict) or not isinstance(value.get("config"), dict):
        raise ProbeError(f"{path}: invalid run metadata")
    return value


def run_probe(
    candidate_run: Path,
    paired_vo_run: Path,
    euroc_root: Path,
    *,
    horizons_s: tuple[float, ...] = (0.05, 0.5, 1.0),
    bucket_duration_s: float = 10.0,
    max_bracket_ns: int = 7_500_000,
) -> dict[str, object]:
    candidate_run = candidate_run.resolve()
    paired_vo_run = paired_vo_run.resolve()
    euroc_root = euroc_root.resolve()
    candidate_meta = _load_meta(candidate_run / "meta.json")
    vo_meta = _load_meta(paired_vo_run / "meta.json")
    candidate_config = candidate_meta["config"]
    vo_config = vo_meta["config"]
    assert isinstance(candidate_config, dict)
    assert isinstance(vo_config, dict)
    if candidate_config.get("estimator.enable_imu") is not True:
        raise ProbeError("candidate run must enable IMU")
    if vo_config.get("estimator.enable_imu") is not False:
        raise ProbeError("paired VO run must disable IMU")
    if candidate_meta.get("sequence") != vo_meta.get("sequence"):
        raise ProbeError("candidate and paired VO sequences differ")
    gravity_value = candidate_config.get("estimator.imu_gravity")
    if not isinstance(gravity_value, (int, float)):
        raise ProbeError("candidate metadata is missing IMU gravity")
    gravity_magnitude = float(gravity_value)
    if not math.isfinite(gravity_magnitude) or gravity_magnitude <= 0.0:
        raise ProbeError("candidate IMU gravity must be finite and positive")
    if max_bracket_ns < 0:
        raise ProbeError("maximum GT bracket must be non-negative")
    if (
        not horizons_s
        or any(not math.isfinite(value) or value <= 0.0 for value in horizons_s)
        or not math.isfinite(bucket_duration_s)
        or bucket_duration_s <= 0.0
    ):
        raise ProbeError("horizons and bucket duration must be finite and positive")

    full_states = load_full_states(candidate_run / "vio_state.csv")
    first_full_ns = full_states[0].timestamp_ns
    candidate_poses = load_tum_poses(candidate_run / "est.tum")
    vo_poses = load_tum_poses(paired_vo_run / "est.tum")
    candidate_by_time = {pose.timestamp_ns: pose for pose in candidate_poses}
    vo_by_time = {pose.timestamp_ns: pose for pose in vo_poses}
    common_timestamps = tuple(
        sorted(candidate_by_time.keys() & vo_by_time.keys())
    )
    references = load_reference(
        euroc_root / "mav0/state_groundtruth_estimate0/data.csv"
    )
    reference_timestamps = tuple(state.timestamp_ns for state in references)
    supported_timestamps: list[int] = []
    reference_positions: dict[int, Array] = {}
    for timestamp in common_timestamps:
        if timestamp < reference_timestamps[0] or timestamp > reference_timestamps[-1]:
            continue
        reference = _reference_at(
            references, reference_timestamps, timestamp, max_bracket_ns
        )
        supported_timestamps.append(timestamp)
        reference_positions[timestamp] = reference.position
    supported = tuple(supported_timestamps)
    candidate_phase, prefix_transform = _trajectory_phase(
        {timestamp: candidate_by_time[timestamp].position for timestamp in supported},
        supported,
        reference_positions,
        first_full_ns,
    )
    vo_phase, _ = _trajectory_phase(
        {timestamp: vo_by_time[timestamp].position for timestamp in supported},
        supported,
        reference_positions,
        first_full_ns,
    )

    imu_samples = load_imu(euroc_root / "mav0/imu0/data.csv")
    bucket_ns = int(round(bucket_duration_s * 1e9))
    horizon_results = []
    for horizon_s in horizons_s:
        horizon_ns = int(round(horizon_s * 1e9))
        if horizon_ns <= 0:
            raise ProbeError("horizon rounds to zero nanoseconds")
        horizon_results.append(
            _evaluate_horizon(
                full_states,
                references,
                imu_samples,
                prefix_transform,
                first_full_ns=first_full_ns,
                horizon_ns=horizon_ns,
                bucket_ns=bucket_ns,
                max_bracket_ns=max_bracket_ns,
                gravity_magnitude=gravity_magnitude,
            )
        )
    return {
        "schema_version": 1,
        "candidate_run": str(candidate_run),
        "paired_vo_run": str(paired_vo_run),
        "sequence_root": str(euroc_root),
        "sequence": candidate_meta.get("sequence"),
        "candidate_config_hash": candidate_meta.get("config_hash"),
        "paired_vo_config_hash": vo_meta.get("config_hash"),
        "first_full_ns": first_full_ns,
        "gravity_magnitude_mps2": gravity_magnitude,
        "max_gt_bracket_ns": max_bracket_ns,
        "bucket_duration_s": bucket_duration_s,
        "phase": {
            "common_poses": len(common_timestamps),
            "gt_supported_poses": len(supported),
            "outside_gt_poses": len(common_timestamps) - len(supported),
            "candidate": candidate_phase,
            "paired_vo": vo_phase,
        },
        "horizons": horizon_results,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("candidate_run", type=Path)
    parser.add_argument("paired_vo_run", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument(
        "--horizon-s",
        type=float,
        action="append",
        dest="horizons_s",
        help="repeat to override the default 0.05/0.5/1.0 s horizons",
    )
    parser.add_argument("--bucket-duration-s", type=float, default=10.0)
    parser.add_argument("--max-gt-bracket-ns", type=int, default=7_500_000)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[list[str]] = None, stdout: TextIO = sys.stdout) -> int:
    arguments = _parser().parse_args(argv)
    horizons = (
        (0.05, 0.5, 1.0)
        if arguments.horizons_s is None
        else tuple(arguments.horizons_s)
    )
    try:
        result = run_probe(
            arguments.candidate_run,
            arguments.paired_vo_run,
            arguments.euroc_root,
            horizons_s=horizons,
            bucket_duration_s=arguments.bucket_duration_s,
            max_bracket_ns=arguments.max_gt_bracket_ns,
        )
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if arguments.output is None:
            stdout.write(serialized)
        else:
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(serialized, encoding="utf-8")
    except (ProbeError, ImuProbeError, ValueError, np.linalg.LinAlgError) as exc:
        print(f"VIO window-history probe: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

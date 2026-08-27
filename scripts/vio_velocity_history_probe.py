#!/usr/bin/env python3
"""Audit velocity-history priors without writing back into the estimator."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO

import numpy as np

from imu_translation_alignment_probe import (
    ProbeError as ImuProbeError,
    ReferenceState,
    imu_segment,
    integrate_interval,
    interpolate_reference,
    load_imu,
    load_reference,
)
from vio_window_history_probe import (
    FullState,
    ProbeError as StateProbeError,
    load_full_states,
)


Array = np.ndarray


class ProbeError(RuntimeError):
    """An input or derived value violates the velocity-shadow contract."""


@dataclass(frozen=True)
class VelocityInterval:
    t_i_ns: int
    t_j_ns: int
    # Fixed-pose ImuFactor equations:
    #   dt * v_i = position_rhs
    #   -v_i + v_j = velocity_rhs
    position_rhs: Array
    velocity_rhs: Array


@dataclass(frozen=True)
class VelocityShadows:
    one_interval: Array
    causal_marginal: Array
    fixed_window: Array
    full_batch: Array
    causal_final_precision: float
    full_batch_normal_residual_inf: float


def _validated_intervals(
    intervals: tuple[VelocityInterval, ...],
) -> tuple[VelocityInterval, ...]:
    if not intervals:
        raise ValueError("velocity solve needs at least one interval")
    previous_j: Optional[int] = None
    for index, interval in enumerate(intervals):
        if interval.t_j_ns <= interval.t_i_ns:
            raise ValueError(f"interval {index} duration must be positive")
        if previous_j is not None and interval.t_i_ns != previous_j:
            raise ValueError(f"interval {index} is not contiguous")
        previous_j = interval.t_j_ns
        for name, value in (
            ("position_rhs", interval.position_rhs),
            ("velocity_rhs", interval.velocity_rhs),
        ):
            array = np.asarray(value, dtype=float)
            if array.shape != (3,) or not np.all(np.isfinite(array)):
                raise ValueError(f"interval {index} {name} must be finite 3D")
    return intervals


def _normal_system(
    intervals: tuple[VelocityInterval, ...],
) -> tuple[Array, Array, Array]:
    values = _validated_intervals(intervals)
    state_count = len(values) + 1
    diagonal = np.zeros(state_count)
    off_diagonal = np.zeros(state_count - 1)
    rhs = np.zeros((state_count, 3))
    for index, interval in enumerate(values):
        dt = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_rhs = np.asarray(interval.position_rhs, dtype=float)
        velocity_rhs = np.asarray(interval.velocity_rhs, dtype=float)
        diagonal[index] += dt * dt + 1.0
        diagonal[index + 1] += 1.0
        off_diagonal[index] = -1.0
        rhs[index] += dt * position_rhs - velocity_rhs
        rhs[index + 1] += velocity_rhs
    return diagonal, off_diagonal, rhs


def _solve_tridiagonal(
    diagonal: Array, off_diagonal: Array, rhs: Array
) -> Array:
    diagonal_value = np.asarray(diagonal, dtype=float).copy()
    off_value = np.asarray(off_diagonal, dtype=float)
    rhs_value = np.asarray(rhs, dtype=float).copy()
    state_count = diagonal_value.shape[0]
    if (
        diagonal_value.shape != (state_count,)
        or off_value.shape != (state_count - 1,)
        or rhs_value.shape != (state_count, 3)
        or not np.all(np.isfinite(diagonal_value))
        or not np.all(np.isfinite(off_value))
        or not np.all(np.isfinite(rhs_value))
    ):
        raise ValueError("invalid tridiagonal velocity system")
    for index in range(1, state_count):
        pivot = diagonal_value[index - 1]
        if not math.isfinite(float(pivot)) or pivot <= 0.0:
            raise ValueError("velocity normal system is not positive definite")
        multiplier = off_value[index - 1] / pivot
        diagonal_value[index] -= multiplier * off_value[index - 1]
        rhs_value[index] -= multiplier * rhs_value[index - 1]
    if diagonal_value[-1] <= 0.0:
        raise ValueError("velocity normal system is not positive definite")
    solution = np.zeros_like(rhs_value)
    solution[-1] = rhs_value[-1] / diagonal_value[-1]
    for index in range(state_count - 2, -1, -1):
        solution[index] = (
            rhs_value[index] - off_value[index] * solution[index + 1]
        ) / diagonal_value[index]
    if not np.all(np.isfinite(solution)):
        raise ValueError("velocity solve produced non-finite state")
    return solution


def solve_batch_velocities(
    intervals: tuple[VelocityInterval, ...],
) -> Array:
    """Solve the fixed-pose batch least-squares problem in O(N)."""
    diagonal, off_diagonal, rhs = _normal_system(intervals)
    return _solve_tridiagonal(diagonal, off_diagonal, rhs)


def _normal_residual_inf(
    intervals: tuple[VelocityInterval, ...], velocities: Array
) -> float:
    diagonal, off_diagonal, rhs = _normal_system(intervals)
    residual = diagonal[:, np.newaxis] * velocities - rhs
    residual[:-1] += off_diagonal[:, np.newaxis] * velocities[1:]
    residual[1:] += off_diagonal[:, np.newaxis] * velocities[:-1]
    return float(np.max(np.abs(residual)))


def solve_velocity_shadows(
    intervals: tuple[VelocityInterval, ...], *, window_size: int
) -> VelocityShadows:
    """Build tuning-free one-edge, causal, window, and batch velocity arms."""
    values = _validated_intervals(intervals)
    if window_size < 2:
        raise ValueError("velocity window size must be at least two")
    state_count = len(values) + 1
    one_interval = np.full((state_count, 3), np.nan)
    causal_marginal = np.full((state_count, 3), np.nan)
    fixed_window = np.full((state_count, 3), np.nan)

    precision = 0.0
    information = np.zeros(3)
    for index, interval in enumerate(values):
        dt = float(interval.t_j_ns - interval.t_i_ns) * 1e-9
        position_rhs = np.asarray(interval.position_rhs, dtype=float)
        velocity_rhs = np.asarray(interval.velocity_rhs, dtype=float)

        one_interval[index + 1] = position_rhs / dt + velocity_rhs

        # Exact Schur complement after eliminating v_i from the accumulated
        # past prior plus the current position/velocity equations.  The scalar
        # precision is shared by x/y/z because the equations are isotropic.
        pivot = precision + dt * dt + 1.0
        eliminated_information = (
            information + dt * position_rhs - velocity_rhs
        )
        precision = 1.0 - 1.0 / pivot
        information = velocity_rhs + eliminated_information / pivot
        if not math.isfinite(precision) or precision <= 0.0:
            raise ValueError("causal velocity marginal lost positive precision")
        causal_marginal[index + 1] = information / precision

    if window_size <= state_count:
        for end in range(window_size - 1, state_count):
            begin = end - window_size + 1
            fixed_window[end] = solve_batch_velocities(values[begin:end])[-1]

    full_batch = solve_batch_velocities(values)
    return VelocityShadows(
        one_interval=one_interval,
        causal_marginal=causal_marginal,
        fixed_window=fixed_window,
        full_batch=full_batch,
        causal_final_precision=precision,
        full_batch_normal_residual_inf=_normal_residual_inf(
            values, full_batch
        ),
    )


def _load_meta(path: Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProbeError(f"cannot load {path}: {exc}") from exc
    if not isinstance(value, dict) or not isinstance(value.get("config"), dict):
        raise ProbeError(f"{path}: invalid run metadata")
    return value


def _reference_at(
    references: tuple[ReferenceState, ...],
    timestamps: tuple[int, ...],
    target_ns: int,
    max_bracket_ns: int,
) -> Optional[ReferenceState]:
    right = bisect.bisect_left(timestamps, target_ns)
    if right < len(references) and timestamps[right] == target_ns:
        return references[right]
    if right == 0 or right == len(references):
        return None
    if timestamps[right] - timestamps[right - 1] > max_bracket_ns:
        raise ProbeError(f"groundtruth bracket is too wide at {target_ns}")
    return interpolate_reference(references, timestamps, target_ns)


def _build_intervals(
    states: tuple[FullState, ...], imu_samples: tuple
) -> tuple[VelocityInterval, ...]:
    imu_timestamps = tuple(sample.timestamp_ns for sample in imu_samples)
    intervals: list[VelocityInterval] = []
    for state_i, state_j in zip(states, states[1:]):
        segment = imu_segment(
            imu_samples,
            imu_timestamps,
            state_i.timestamp_ns,
            state_j.timestamp_ns,
        )
        preintegration = integrate_interval(
            segment,
            state_i.gyro_bias,
            lambda _timestamp_ns, bias=state_i.accel_bias: bias,
        )
        dt = float(state_j.timestamp_ns - state_i.timestamp_ns) * 1e-9
        intervals.append(
            VelocityInterval(
                state_i.timestamp_ns,
                state_j.timestamp_ns,
                state_j.position
                - state_i.position
                - state_i.rotation @ preintegration.delta_p
                - 0.5 * state_i.gravity * dt * dt,
                state_i.rotation @ preintegration.delta_v
                + state_i.gravity * dt,
            )
        )
    return tuple(intervals)


def _summary(values: list[float]) -> dict[str, float]:
    if not values:
        raise ProbeError("cannot summarize empty velocity errors")
    array = np.asarray(values, dtype=float)
    return {
        "rmse_mps": math.sqrt(float(np.mean(array * array))),
        "median_mps": float(np.median(array)),
        "p95_mps": float(np.quantile(array, 0.95)),
        "max_mps": float(np.max(array)),
    }


def run_probe(
    candidate_run: Path,
    euroc_root: Path,
    *,
    bucket_duration_s: float = 10.0,
    max_bracket_ns: int = 7_500_000,
) -> dict[str, object]:
    candidate_run = candidate_run.resolve()
    euroc_root = euroc_root.resolve()
    meta = _load_meta(candidate_run / "meta.json")
    config = meta["config"]
    assert isinstance(config, dict)
    if config.get("estimator.enable_imu") is not True:
        raise ProbeError("candidate run must enable IMU")
    window_value = config.get("estimator.window_size")
    if not isinstance(window_value, int) or window_value < 2:
        raise ProbeError("candidate metadata has invalid estimator.window_size")
    if (
        not math.isfinite(bucket_duration_s)
        or bucket_duration_s <= 0.0
        or max_bracket_ns < 0
    ):
        raise ProbeError("bucket duration and GT bracket must be valid")

    states = load_full_states(candidate_run / "vio_state.csv")
    if len(states) < window_value:
        raise ProbeError("candidate has fewer full states than its window")
    imu_samples = load_imu(euroc_root / "mav0/imu0/data.csv")
    references = load_reference(
        euroc_root / "mav0/state_groundtruth_estimate0/data.csv"
    )
    reference_timestamps = tuple(state.timestamp_ns for state in references)
    intervals = _build_intervals(states, imu_samples)
    shadows = solve_velocity_shadows(intervals, window_size=window_value)

    arm_velocities = {
        "actual": np.stack([state.velocity for state in states]),
        "one_interval": shadows.one_interval,
        "causal_marginal": shadows.causal_marginal,
        "fixed_window": shadows.fixed_window,
        "full_batch": shadows.full_batch,
    }
    first_index = window_value - 1
    samples: list[tuple[int, dict[str, float]]] = []
    for index in range(first_index, len(states)):
        state = states[index]
        reference = _reference_at(
            references,
            reference_timestamps,
            state.timestamp_ns,
            max_bracket_ns,
        )
        if reference is None:
            continue
        truth_body = reference.rotation.T @ reference.velocity
        errors: dict[str, float] = {}
        for name, velocities in arm_velocities.items():
            velocity = velocities[index]
            if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
                raise ProbeError(f"{name} velocity is invalid at {state.timestamp_ns}")
            errors[name] = float(
                np.linalg.norm(state.rotation.T @ velocity - truth_body)
            )
        samples.append((state.timestamp_ns, errors))
    if not samples:
        raise ProbeError("no full states have GT support")

    arm_names = tuple(arm_velocities)
    summaries = {
        name: _summary([errors[name] for _, errors in samples])
        for name in arm_names
    }
    actual_rmse = summaries["actual"]["rmse_mps"]
    for name in arm_names:
        summaries[name]["rmse_ratio_to_actual"] = (
            summaries[name]["rmse_mps"] / actual_rmse
        )
        if name != "actual":
            summaries[name]["improved_fraction"] = float(
                np.mean(
                    [errors[name] < errors["actual"] for _, errors in samples]
                )
            )

    bucket_ns = int(round(bucket_duration_s * 1e9))
    origin_ns = states[0].timestamp_ns
    buckets: list[dict[str, object]] = []
    for bucket in sorted(
        {(timestamp_ns - origin_ns) // bucket_ns for timestamp_ns, _ in samples}
    ):
        selected = [
            errors
            for timestamp_ns, errors in samples
            if (timestamp_ns - origin_ns) // bucket_ns == bucket
        ]
        buckets.append(
            {
                "bucket": int(bucket),
                "start_ns": origin_ns + bucket * bucket_ns,
                "end_ns": origin_ns + (bucket + 1) * bucket_ns,
                "sample_count": len(selected),
                "arms": {
                    name: _summary([errors[name] for errors in selected])
                    for name in arm_names
                },
            }
        )

    return {
        "schema_version": 1,
        "candidate_run": str(candidate_run),
        "sequence_root": str(euroc_root),
        "sequence": meta.get("sequence"),
        "candidate_config_hash": meta.get("config_hash"),
        "window_size": window_value,
        "bucket_duration_s": bucket_duration_s,
        "max_gt_bracket_ns": max_bracket_ns,
        "support": {
            "full_states": len(states),
            "scored_states": len(samples),
            "start_ns": samples[0][0],
            "end_ns": samples[-1][0],
        },
        "contracts": {
            "actual": "candidate posterior velocity",
            "one_interval": "causal; current interval only",
            "causal_marginal": "causal; exact Schur prior from all past intervals",
            "fixed_window": "causal; fixed poses in current window, no incoming prior",
            "full_batch": "non-causal future-leaking diagnostic upper bound",
            "truth": "GT is used only for offline scoring",
        },
        "solver": {
            "causal_final_precision": shadows.causal_final_precision,
            "full_batch_normal_residual_inf": (
                shadows.full_batch_normal_residual_inf
            ),
        },
        "arms": summaries,
        "buckets": buckets,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("candidate_run", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument("--bucket-duration-s", type=float, default=10.0)
    parser.add_argument("--max-gt-bracket-ns", type=int, default=7_500_000)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: Optional[list[str]] = None, stdout: TextIO = sys.stdout) -> int:
    arguments = _parser().parse_args(argv)
    try:
        result = run_probe(
            arguments.candidate_run,
            arguments.euroc_root,
            bucket_duration_s=arguments.bucket_duration_s,
            max_bracket_ns=arguments.max_gt_bracket_ns,
        )
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if arguments.output is None:
            stdout.write(serialized)
        else:
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(serialized, encoding="utf-8")
    except (
        ProbeError,
        ImuProbeError,
        StateProbeError,
        ValueError,
        np.linalg.LinAlgError,
    ) as exc:
        print(f"VIO velocity-history probe: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

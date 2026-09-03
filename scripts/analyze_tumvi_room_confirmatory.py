#!/usr/bin/env python3
"""Analyze the preregistered TUM-VI room confirmatory matrix."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np


SEQUENCES = tuple(f"room{index}" for index in range(1, 7))
ROLES = ("off", "on")
REPEATS = (1, 2, 3)
ARTIFACTS = (
    "est.tum",
    "diag.csv",
    "kf.tum",
    "probe-b.jsonl",
    "meta.json",
    "summary.json",
)
ASSOCIATION_MAX_DT_NS = 4_166_667
ASSOCIATION_MIN_MATCH_RATE = 0.80
ASSOCIATION_MIN_MATCHES = 3
RPE_DELTA_NS = 1_000_000_000
RPE_PARTNER_TOLERANCE_NS = 25_000_000
RANK_TOLERANCE = 1e-8
SUMMARY_ATOL = 1e-10
DIAG_COLUMN_COUNT = 26
DIAG_BOOL_COLUMNS = {5, 14, 18}
DIAG_INT_COLUMNS = {
    0,
    2,
    3,
    4,
    6,
    7,
    10,
    11,
    13,
    15,
    16,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
}
DIAG_FLOAT_COLUMNS = {8, 9, 12, 17}
DIAG_HEADER = (
    "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
    "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
    "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
    "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
    "is_keyframe,num_disparity,unsupported_span_ns,num_retained_observations,"
    "num_seeded_landmarks,num_current_visual_factors,num_mapped_observations,"
    "num_current_mono_visual_factors"
)
DIAG_STATUSES = {
    "ok",
    "initializing",
    "discontinuity",
    "visual_outage",
    "invalid_input",
    "rejected",
    "failed",
}
TASK_PATTERN = re.compile(r"^(off|on)-([123])-(room[1-6])$")
GT_HEADER = "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz"
CONFIRMATORY_PROFILE = "ci-tumvi-room6-confirmatory"
EXPECTED_DATASET_MANIFEST_ID = (
    "sha256:62fad56429774086b675f9203edeece6077f56cd387924633e73c1d245b537b4"
)
EXPECTED_DATASET_CONTENT_ID = (
    "sha256:10462c2f2bec6d1325c2620b0efd24181a200b6d7d8d8fcbb1d12f8f44e42070"
)
EXPECTED_DATASET_ROOT = (
    "/home/lin/Projects/tigerfish/datasets/ci-tumvi-room6-confirmatory/"
    "by-content/sha256/10462c2f2bec6d1325c2620b0efd24181a200b6d7d8d8fcbb1d12f8f44e42070"
)
HEX_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
RFC3339_UTC_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|\+00:00)$"
)
PROTOCOL_RELATIVE_PATH = (
    "docs/research/"
    "2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-protocol.md"
)
ANALYZER_RELATIVE_PATH = "scripts/analyze_tumvi_room_confirmatory.py"
EXPECTED_BATCH_ORDER = (
    ("off", 1),
    ("on", 1),
    ("on", 2),
    ("off", 2),
    ("off", 3),
    ("on", 3),
)
PREFLIGHT_BINARIES = {
    "phad_vo_bench",
    "phad_estimator_tests",
    "phad_apps_tests",
    "phad_io_dataset_tests",
    "phad_eval_tests",
    "phad_bench_tests",
}
PREFLIGHT_RECEIPT_KEYS = {
    "schema_version",
    "profile",
    "created_utc",
    "source_id",
    "dataset",
    "image",
    "inputs",
    "binaries",
    "runtime_linkage",
    "batch_order",
    "benchmark_commands",
    "preflight_receipt_id",
}
PNP_RESULTS = {
    "not_attempted",
    "ransac_failed",
    "insufficient_inliers",
    "proposal_pose_nonfinite",
    "proposal_rms_invalid_correspondence",
    "proposal_rms_cheirality",
    "proposal_rms_nonfinite",
    "proposal_rms_rejected",
    "accepted",
    "accepted_after_guess_retry",
}
PNP_RETRY_OUTCOMES = {
    "not_attempted",
    "solve_failed",
    "proposal_pose_nonfinite",
    "proposal_rms_invalid_correspondence",
    "proposal_rms_cheirality",
    "proposal_rms_nonfinite",
    "proposal_rms_rejected",
    "would_pass_existing_pnp_gates",
}
PROBE_REQUIRED_KEYS = {
    "i",
    "ts_ns",
    "pnp_attempted",
    "pnp_result",
    "pnp_correspondences",
    "pnp_inliers",
    "pnp_guess_retry_outcome",
    "shared",
    "num_obs",
}
PROBE_UINT_KEYS = {
    "i",
    "pnp_correspondences",
    "pnp_inliers",
    "pnp_proposal_inlier_count",
    "pnp_proposal_cheiral_inlier_count",
    "pnp_proposal_min_inlier_landmark_id",
    "pnp_guess_retry_cheiral_inlier_count",
    "culled_ids",
    "zombie_track_n",
    "rejected_block_n",
    "new_lm",
    "shared",
    "num_obs",
    "lm_iterations",
    "res_max_id",
}
PROBE_FLOAT_KEYS = {
    "pnp_proposal_rms_px",
    "pnp_guess_rms_px",
    "pnp_proposal_min_inlier_depth_m",
    "pnp_proposal_guess_translation_delta_m",
    "pnp_proposal_guess_rotation_delta_rad",
    "pnp_guess_retry_proposal_rms_px",
    "pnp_guess_retry_guess_rms_px",
    "pnp_guess_retry_min_inlier_depth_m",
    "pnp_guess_retry_translation_delta_m",
    "pnp_guess_retry_rotation_delta_rad",
    "current_pose_translation_delta_m",
    "current_pose_rotation_delta_rad",
    "current_velocity_delta_mps",
    "current_acc_bias_delta_mps2",
    "current_gyr_bias_delta_radps",
    "primary_graph_error_seed",
    "primary_graph_error_optimized",
    "primary_imu_factor_error_seed",
    "primary_imu_factor_error_optimized",
    "shift_m",
    "res_mean_px",
    "res_max_px",
}
PROBE_ALLOWED_KEYS = (
    PROBE_REQUIRED_KEYS
    | PROBE_UINT_KEYS
    | PROBE_FLOAT_KEYS
    | {"shift_top", "drops_skipped_this_frame"}
)


class AnalysisError(RuntimeError):
    """Invalid or incomplete confirmatory input."""


@dataclass(frozen=True)
class Pose:
    timestamp_ns: int
    position: np.ndarray
    rotation: np.ndarray
    raw_line: str | None = None


@dataclass(frozen=True)
class Association:
    est_indices: np.ndarray
    gt_indices: np.ndarray
    dt_ns: np.ndarray
    est_total: int
    dropped_out_of_range: int
    dropped_over_threshold: int


@dataclass(frozen=True)
class AteReport:
    trans_m: np.ndarray
    rot_deg: np.ndarray
    timestamps_ns: tuple[int, ...]
    association: Association


@dataclass(frozen=True)
class RpeReport:
    trans_m: np.ndarray
    rot_deg: np.ndarray
    keys: tuple[tuple[int, int], ...]
    association: Association
    dropped_no_partner: int


@dataclass(frozen=True)
class RunReport:
    role: str
    repeat: int
    sequence: str
    bench: Path
    meta: dict[str, Any]
    summary: dict[str, Any]
    ate: AteReport | None
    rpe: RpeReport | None
    hashes: dict[str, str]
    exact_hashes: dict[str, str]
    probe: dict[str, int]
    summary_reproduction: dict[str, Any]


@dataclass(frozen=True)
class DiagReport:
    timestamps_ns: tuple[int, ...]
    statuses: tuple[str, ...]
    accepted_keyframe_timestamps_ns: tuple[int, ...]
    all_keyframe_count: int
    segment_count: int
    low_connectivity_count: int
    cheirality_count: int
    pnp_success_count: int
    outliers_culled_total: int


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                AnalysisError(f"non-finite JSON value {value}: {path}")
            ),
        )
    except (OSError, json.JSONDecodeError) as error:
        raise AnalysisError(f"invalid JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"expected JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    except OSError as error:
        raise AnalysisError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _expected_benchmark_command(sequence: str, role: str) -> str:
    activation = (
        "  --estimator-enable-pnp-cheirality-retry \\\n" if role == "on" else ""
    )
    return f"""set -euo pipefail
umask 022
/shared-build/phad_vo_bench /datasets/tum-vi/{sequence} \\
  --dataset-format tum-vi \\
  --gt-tum-vi /datasets/tum-vi/{sequence} \\
  --out /task/artifacts/bench \\
  --sequence-name {sequence} \\
  --config-label default \\
  --repo /src \\
  --max-dt-ms 4.166667 \\
  --min-match-rate 0.80 \\
  --rpe-delta-s 1.0 \\
  --estimator-enable-moving-bootstrap \\
{activation}  --probe-b /task/artifacts/bench/probe-b.jsonl \\
  --force
"""


def _expected_build_command(run: dict[str, Any]) -> str:
    parallelism = _require_nonnegative_int(
        run.get("build_parallelism"), "run.build_parallelism"
    )
    return "\n".join(
        (
            "set -euo pipefail",
            "umask 022",
            "cmake -S /src -B /task/build -G Ninja "
            "-DCMAKE_BUILD_TYPE=Release -DPHAD_BUILD_TESTS=ON "
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            f"cmake --build /task/build --parallel {parallelism}",
        )
    )


def _expected_unit_command(run: dict[str, Any]) -> str:
    parallelism = _require_nonnegative_int(
        run.get("unit_parallelism"), "run.unit_parallelism"
    )
    commands = [
        "set -euo pipefail",
        "umask 022",
        "cmake -E copy_directory /shared-build /task/build",
        f"ctest --test-dir /task/build --output-on-failure -L unit --parallel {parallelism}",
    ]
    for filename in (
        "test_remote_ci.py",
        "test_remote_ci_tumvi_stage.py",
        "test_remote_ci_tumvi_preflight.py",
        "test_analyze_tumvi_room_confirmatory.py",
    ):
        commands.append(
            "/opt/phad-venv/bin/python -m unittest discover "
            f"-s /src/tests/scripts -p '{filename}'"
        )
    return "\n".join(commands)


def _exact_timestamp_ns(text: str) -> int:
    if not text:
        raise AnalysisError("empty timestamp")
    negative = text.startswith("-")
    magnitude = text[1:] if negative else text
    whole, dot, fraction = magnitude.partition(".")
    if not whole.isdigit() or (dot and (not fraction or not fraction.isdigit())):
        raise AnalysisError(f"invalid decimal timestamp: {text!r}")
    if len(fraction) > 9:
        raise AnalysisError(f"sub-nanosecond timestamp: {text!r}")
    value = int(whole) * 1_000_000_000
    if fraction:
        value += int(fraction.ljust(9, "0"))
    return -value if negative else value


def _rotation(
    qx: float, qy: float, qz: float, qw: float, tolerance: float
) -> np.ndarray:
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise AnalysisError("non-finite or zero quaternion")
    if abs(norm - 1.0) > tolerance:
        raise AnalysisError(
            f"quaternion norm {norm:.17g} exceeds tolerance {tolerance:.17g}"
        )
    qw, qx, qy, qz = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def _validate_poses(
    poses: list[Pose], path: Path, *, allow_empty: bool = False
) -> tuple[Pose, ...]:
    if not poses and not allow_empty:
        raise AnalysisError(f"empty trajectory: {path}")
    previous: int | None = None
    for pose in poses:
        if not np.isfinite(pose.position).all() or not np.isfinite(
            pose.rotation
        ).all():
            raise AnalysisError(f"non-finite pose: {path}")
        if previous is not None and pose.timestamp_ns <= previous:
            raise AnalysisError(f"timestamps are not strictly increasing: {path}")
        previous = pose.timestamp_ns
    return tuple(poses)


def read_tum(path: Path, *, allow_empty: bool = False) -> tuple[Pose, ...]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise AnalysisError(f"cannot read trajectory {path}: {error}") from error
    poses: list[Pose] = []
    for line_number, raw in enumerate(lines, 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise AnalysisError(f"{path}:{line_number}: expected 8 TUM fields")
        try:
            values = [float(field) for field in fields[1:]]
        except ValueError as error:
            raise AnalysisError(
                f"{path}:{line_number}: invalid floating-point field"
            ) from error
        if not all(math.isfinite(value) for value in values):
            raise AnalysisError(f"{path}:{line_number}: non-finite pose field")
        poses.append(
            Pose(
                _exact_timestamp_ns(fields[0]),
                np.array(values[:3], dtype=np.float64),
                _rotation(values[3], values[4], values[5], values[6], 1e-3),
                line,
            )
        )
    return _validate_poses(poses, path, allow_empty=allow_empty)


def read_tumvi_groundtruth(sequence_root: Path) -> tuple[Pose, ...]:
    path = sequence_root / "dso" / "gt_imu.csv"
    if not path.is_file():
        raise AnalysisError(f"missing TUM-VI ground truth: {path}")
    poses: list[Pose] = []
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            header = stream.readline().rstrip("\r\n")
            if header != GT_HEADER:
                raise AnalysisError(f"unexpected TUM-VI ground-truth header: {path}")
            reader = csv.reader(stream)
            for line_number, row in enumerate(reader, 2):
                if len(row) != 8:
                    raise AnalysisError(
                        f"{path}:{line_number}: expected 8 CSV columns"
                    )
                try:
                    timestamp_ns = int(row[0])
                    values = [float(value) for value in row[1:]]
                except ValueError as error:
                    raise AnalysisError(
                        f"{path}:{line_number}: invalid pose field"
                    ) from error
                if not all(math.isfinite(value) for value in values):
                    raise AnalysisError(
                        f"{path}:{line_number}: non-finite pose field"
                    )
                poses.append(
                    Pose(
                        timestamp_ns,
                        np.array(values[:3], dtype=np.float64),
                        _rotation(
                            values[4], values[5], values[6], values[3], 1e-6
                        ),
                    )
                )
    except OSError as error:
        raise AnalysisError(f"cannot read ground truth {path}: {error}") from error
    return _validate_poses(poses, path)


def associate(est: tuple[Pose, ...], gt: tuple[Pose, ...]) -> Association:
    gt_timestamps = np.array([pose.timestamp_ns for pose in gt], dtype=np.int64)
    est_indices: list[int] = []
    gt_indices: list[int] = []
    deltas: list[int] = []
    dropped_out_of_range = 0
    dropped_over_threshold = 0
    for est_index, pose in enumerate(est):
        upper = int(np.searchsorted(gt_timestamps, pose.timestamp_ns, side="left"))
        if upper == 0:
            gt_index = 0
        elif upper == len(gt):
            gt_index = len(gt) - 1
        else:
            lower = upper - 1
            upper_gap = int(gt_timestamps[upper] - pose.timestamp_ns)
            lower_gap = int(pose.timestamp_ns - gt_timestamps[lower])
            gt_index = upper if upper_gap < lower_gap else lower
        delta = pose.timestamp_ns - gt[gt_index].timestamp_ns
        if abs(delta) <= ASSOCIATION_MAX_DT_NS:
            est_indices.append(est_index)
            gt_indices.append(gt_index)
            deltas.append(delta)
        elif pose.timestamp_ns < gt_timestamps[0] or pose.timestamp_ns > gt_timestamps[-1]:
            dropped_out_of_range += 1
        else:
            dropped_over_threshold += 1
    result = Association(
        np.array(est_indices, dtype=np.int64),
        np.array(gt_indices, dtype=np.int64),
        np.array(deltas, dtype=np.int64),
        len(est),
        dropped_out_of_range,
        dropped_over_threshold,
    )
    matched = len(result.est_indices)
    if matched < ASSOCIATION_MIN_MATCHES:
        raise AnalysisError(
            f"association has fewer than {ASSOCIATION_MIN_MATCHES} matches"
        )
    if matched / result.est_total < ASSOCIATION_MIN_MATCH_RATE:
        raise AnalysisError("association match rate is below 0.80")
    return result


def _align_se3(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    covariance = (target - target_centroid).T @ (source - source_centroid)
    u, singular_values, vt = np.linalg.svd(covariance, full_matrices=True)
    if singular_values[0] <= 0.0 or singular_values[1] / singular_values[0] < RANK_TOLERANCE:
        raise AnalysisError(
            f"degenerate alignment singular values: {singular_values.tolist()}"
        )
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0.0:
        correction[2, 2] = -1.0
    rotation = u @ correction @ vt
    return rotation, target_centroid - rotation @ source_centroid


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def compute_ate(est: tuple[Pose, ...], gt: tuple[Pose, ...]) -> AteReport:
    association = associate(est, gt)
    source = np.stack([est[int(index)].position for index in association.est_indices])
    target = np.stack([gt[int(index)].position for index in association.gt_indices])
    rotation, translation = _align_se3(source, target)
    trans: list[float] = []
    rot: list[float] = []
    timestamps: list[int] = []
    for est_index, gt_index in zip(
        association.est_indices, association.gt_indices, strict=True
    ):
        estimate = est[int(est_index)]
        reference = gt[int(gt_index)]
        trans.append(
            float(np.linalg.norm(rotation @ estimate.position + translation - reference.position))
        )
        rot.append(_rotation_angle_deg((rotation @ estimate.rotation).T @ reference.rotation))
        timestamps.append(estimate.timestamp_ns)
    return AteReport(
        np.array(trans, dtype=np.float64),
        np.array(rot, dtype=np.float64),
        tuple(timestamps),
        association,
    )


def _find_partner(timestamps: np.ndarray, index: int) -> int | None:
    begin = index + 1
    target = int(timestamps[index]) + RPE_DELTA_NS
    upper = int(np.searchsorted(timestamps[begin:], target, side="left")) + begin
    if upper == len(timestamps):
        if upper == begin:
            return None
        best = upper - 1
    elif upper != begin:
        lower = upper - 1
        best = lower if target - int(timestamps[lower]) < int(timestamps[upper]) - target else upper
    else:
        best = upper
    return best if abs(int(timestamps[best]) - target) <= RPE_PARTNER_TOLERANCE_NS else None


def _relative_motion(first: Pose, second: Pose) -> tuple[np.ndarray, np.ndarray]:
    return (
        first.rotation.T @ second.rotation,
        first.rotation.T @ (second.position - first.position),
    )


def compute_rpe(est: tuple[Pose, ...], gt: tuple[Pose, ...]) -> RpeReport:
    association = associate(est, gt)
    timestamps = np.array(
        [est[int(index)].timestamp_ns for index in association.est_indices],
        dtype=np.int64,
    )
    trans: list[float] = []
    rot: list[float] = []
    keys: list[tuple[int, int]] = []
    dropped_no_partner = 0
    for index in range(len(timestamps)):
        partner = _find_partner(timestamps, index)
        if partner is None:
            dropped_no_partner += 1
            continue
        from_est = est[int(association.est_indices[index])]
        to_est = est[int(association.est_indices[partner])]
        from_gt = gt[int(association.gt_indices[index])]
        to_gt = gt[int(association.gt_indices[partner])]
        est_rotation, est_translation = _relative_motion(from_est, to_est)
        gt_rotation, gt_translation = _relative_motion(from_gt, to_gt)
        error_rotation = gt_rotation.T @ est_rotation
        error_translation = gt_rotation.T @ (est_translation - gt_translation)
        trans.append(float(np.linalg.norm(error_translation)))
        rot.append(_rotation_angle_deg(error_rotation))
        keys.append((from_est.timestamp_ns, to_est.timestamp_ns))
    if not trans:
        raise AnalysisError("RPE has no fixed-delta partners")
    return RpeReport(
        np.array(trans, dtype=np.float64),
        np.array(rot, dtype=np.float64),
        tuple(keys),
        association,
        dropped_no_partner,
    )


def _stats(values: Sequence[float] | np.ndarray) -> dict[str, float | int]:
    array = np.asarray(values, dtype=np.float64)
    if len(array) == 0:
        raise AnalysisError("cannot summarize an empty distribution")
    mean = float(np.mean(array))
    return {
        "count": int(len(array)),
        "rmse": float(math.sqrt(float(np.mean(array * array)))),
        "mean": mean,
        "median": float(np.median(array)),
        "stddev": float(math.sqrt(max(0.0, float(np.mean(array * array)) - mean * mean))),
        "p10": float(np.percentile(array, 10.0, method="linear")),
        "p90": float(np.percentile(array, 90.0, method="linear")),
        "max": float(np.max(array)),
    }


def _delta_stats(values: Sequence[float] | np.ndarray) -> dict[str, float | int]:
    result = _stats(values)
    result.pop("stddev")
    return result


def _association_json(value: Association) -> dict[str, int | float]:
    matched = len(value.est_indices)
    return {
        "est_total": value.est_total,
        "matched": matched,
        "match_rate": matched / value.est_total,
        "dropped_out_of_range": value.dropped_out_of_range,
        "dropped_over_threshold": value.dropped_over_threshold,
    }


def _summary_reproduction(
    summary: dict[str, Any], ate: AteReport, rpe: RpeReport
) -> dict[str, Any]:
    deltas: dict[str, float] = {}
    for section, field, values in (
        ("ate", "trans", ate.trans_m),
        ("ate", "rot_deg", ate.rot_deg),
        ("rpe", "trans", rpe.trans_m),
        ("rpe", "rot_deg", rpe.rot_deg),
    ):
        expected = summary.get(section, {}).get(field)
        if not isinstance(expected, dict):
            raise AnalysisError(f"summary missing {section}.{field}")
        actual = _stats(values)
        for key in ("rmse", "mean", "median", "stddev", "max"):
            try:
                expected_value = float(expected[key])
            except (KeyError, TypeError, ValueError) as error:
                raise AnalysisError(f"summary missing {section}.{field}.{key}") from error
            if not math.isfinite(expected_value):
                raise AnalysisError(f"summary has non-finite {section}.{field}.{key}")
            deltas[f"{section}.{field}.{key}"] = abs(
                float(actual[key]) - expected_value
            )
    maximum = max(deltas.values(), default=0.0)
    if maximum > SUMMARY_ATOL:
        raise AnalysisError(
            f"summary metric reproduction max abs delta {maximum:.17g} "
            f"exceeds {SUMMARY_ATOL:.17g}"
        )
    return {
        "max_abs_delta": maximum,
        "within_atol": True,
        "fields": deltas,
    }


def _reproduce_trajectory_rates(
    trajectory: dict[str, Any],
    diag: DiagReport,
    estimate: tuple[Pose, ...],
    *,
    allow_single_frame: bool = False,
) -> dict[str, float]:
    diag_span = diag.timestamps_ns[-1] - diag.timestamps_ns[0]
    if diag_span <= 0 and not (allow_single_frame and len(diag.timestamps_ns) == 1):
        raise AnalysisError("diag.csv span must be positive")
    estimate_span = (
        estimate[-1].timestamp_ns - estimate[0].timestamp_ns if estimate else 0
    )
    actual = {
        "completion_rate": diag.statuses.count("ok") / len(diag.timestamps_ns),
        "coverage_rate": estimate_span / diag_span if diag_span > 0 else 0.0,
    }
    deltas: dict[str, float] = {}
    for key, actual_value in actual.items():
        if not math.isfinite(actual_value) or not 0.0 <= actual_value <= 1.0:
            raise AnalysisError(f"recomputed trajectory.{key} is outside [0, 1]")
        try:
            summary_value = float(trajectory[key])
        except (KeyError, TypeError, ValueError) as error:
            raise AnalysisError(f"summary missing trajectory.{key}") from error
        if not math.isfinite(summary_value) or not 0.0 <= summary_value <= 1.0:
            raise AnalysisError(f"summary trajectory.{key} is outside [0, 1]")
        delta = abs(summary_value - actual_value)
        if delta > SUMMARY_ATOL:
            raise AnalysisError(
                f"summary trajectory.{key} reproduction delta {delta:.17g} "
                f"exceeds {SUMMARY_ATOL:.17g}"
            )
        deltas[f"trajectory.{key}"] = delta
    return deltas


def _read_probe(path: Path, role: str) -> tuple[dict[str, int], tuple[int, ...]]:
    counts = {"eligible": 0, "attempted": 0, "accepted": 0, "failed": 0, "fallback": 0}
    timestamps: list[int] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise AnalysisError(f"cannot read Probe B artifact {path}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        try:
            row = json.loads(
                line,
                parse_constant=lambda value: (_ for _ in ()).throw(
                    AnalysisError(
                        f"non-finite Probe B value {value}: {path}:{line_number}"
                    )
                ),
            )
        except json.JSONDecodeError as error:
            raise AnalysisError(f"invalid Probe B JSON: {path}:{line_number}") from error
        if not isinstance(row, dict):
            raise AnalysisError(f"expected Probe B object: {path}:{line_number}")
        unknown_keys = sorted(set(row) - PROBE_ALLOWED_KEYS)
        if unknown_keys:
            raise AnalysisError(
                f"Probe B unknown key at {path}:{line_number}: {unknown_keys}"
            )
        missing_keys = sorted(PROBE_REQUIRED_KEYS - set(row))
        if missing_keys:
            raise AnalysisError(
                f"Probe B missing schema keys at {path}:{line_number}: {missing_keys}"
            )
        index = row.get("i")
        if (
            not isinstance(index, int)
            or isinstance(index, bool)
            or index != line_number - 1
        ):
            raise AnalysisError(
                f"Probe B index must equal zero-based row index: {path}:{line_number}"
            )
        timestamp = row.get("ts_ns")
        if not isinstance(timestamp, int) or isinstance(timestamp, bool):
            raise AnalysisError(f"invalid Probe B ts_ns: {path}:{line_number}")
        if timestamps and timestamp <= timestamps[-1]:
            raise AnalysisError(
                f"Probe B timestamps are not strictly increasing: {path}:{line_number}"
            )
        timestamps.append(timestamp)
        result = row.get("pnp_result")
        outcome = row.get("pnp_guess_retry_outcome")
        if result not in PNP_RESULTS:
            raise AnalysisError(f"invalid Probe B pnp_result: {path}:{line_number}")
        if outcome not in PNP_RETRY_OUTCOMES:
            raise AnalysisError(
                f"invalid Probe B pnp_guess_retry_outcome: {path}:{line_number}"
            )
        if not isinstance(row.get("pnp_attempted"), bool):
            raise AnalysisError(
                f"Probe B pnp_attempted must be boolean: {path}:{line_number}"
            )
        for key in PROBE_UINT_KEYS - {"culled_ids"}:
            if key not in row:
                continue
            value = row[key]
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise AnalysisError(
                    f"Probe B {key} must be a non-negative integer: {path}:{line_number}"
                )
        if "culled_ids" in row:
            culled_ids = row["culled_ids"]
            if not isinstance(culled_ids, list) or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
                for value in culled_ids
            ):
                raise AnalysisError(
                    f"Probe B culled_ids must be non-negative integers: {path}:{line_number}"
                )
        for key in PROBE_FLOAT_KEYS:
            if key not in row:
                continue
            value = row[key]
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(float(value))
            ):
                raise AnalysisError(
                    f"Probe B {key} must be finite: {path}:{line_number}"
                )
        if "drops_skipped_this_frame" in row and not isinstance(
            row["drops_skipped_this_frame"], bool
        ):
            raise AnalysisError(
                f"Probe B drops_skipped_this_frame must be boolean: {path}:{line_number}"
            )
        if "shift_top" in row:
            shift_top = row["shift_top"]
            if not isinstance(shift_top, list):
                raise AnalysisError(
                    f"Probe B shift_top must be an array: {path}:{line_number}"
                )
            for entry in shift_top:
                if not isinstance(entry, dict) or set(entry) != {"key", "dt_m"}:
                    raise AnalysisError(
                        f"Probe B shift_top has invalid schema: {path}:{line_number}"
                    )
                key = entry["key"]
                delta = entry["dt_m"]
                if (
                    not isinstance(key, int)
                    or isinstance(key, bool)
                    or key < 0
                    or not isinstance(delta, (int, float))
                    or isinstance(delta, bool)
                    or not math.isfinite(float(delta))
                ):
                    raise AnalysisError(
                        f"Probe B shift_top has invalid value: {path}:{line_number}"
                    )
        eligible = result in {
            "proposal_rms_cheirality",
            "accepted_after_guess_retry",
        }
        attempted = outcome != "not_attempted"
        accepted = result == "accepted_after_guess_retry"
        if row["pnp_attempted"] != (result != "not_attempted"):
            raise AnalysisError(
                f"Probe B pnp_attempted/result invariant violation: "
                f"{path}:{line_number}"
            )
        if accepted and (
            role != "on"
            or not row["pnp_attempted"]
            or not eligible
            or not attempted
            or outcome != "would_pass_existing_pnp_gates"
        ):
            label = "off role" if role == "off" else "accepted invariant"
            raise AnalysisError(f"Probe B {label} violation: {path}:{line_number}")
        if role == "off" and result == "accepted_after_guess_retry":
            raise AnalysisError(
                f"Probe B off role cannot accept retry: {path}:{line_number}"
            )
        if attempted and (
            not row["pnp_attempted"] or not eligible
        ):
            raise AnalysisError(
                f"Probe B attempted retry is not eligible: {path}:{line_number}"
            )
        if eligible:
            counts["eligible"] += 1
        if attempted:
            counts["attempted"] += 1
        if accepted:
            counts["accepted"] += 1
        if attempted and not accepted:
            counts["failed"] += 1
        if row.get("pnp_attempted") is True and result not in {"accepted", "accepted_after_guess_retry"}:
            counts["fallback"] += 1
    return counts, tuple(timestamps)


def _canonical_meta(meta: dict[str, Any]) -> dict[str, Any]:
    value = json.loads(json.dumps(meta))
    value.pop("created_utc", None)
    return value


def _canonical_summary(summary: dict[str, Any]) -> dict[str, Any]:
    value = json.loads(json.dumps(summary))
    value.pop("timing", None)
    return value


def _read_diag(path: Path) -> DiagReport:
    try:
        stream = path.open(encoding="utf-8", newline="")
    except OSError as error:
        raise AnalysisError(f"cannot read diag header {path}: {error}") from error
    with stream:
        reader = csv.reader(stream)
        header = next(reader, None)
        if header is None or ",".join(header) != DIAG_HEADER:
            raise AnalysisError(f"diag.csv header must match exact 26-column schema: {path}")
        timestamps: list[int] = []
        statuses: list[str] = []
        accepted_keyframes: list[int] = []
        all_keyframe_count = 0
        segment_count = 0
        last_ok_segment_id: int | None = None
        low_connectivity_count = 0
        cheirality_count = 0
        pnp_success_count = 0
        outliers_culled_total = 0
        for line_number, row in enumerate(reader, 2):
            if len(row) != DIAG_COLUMN_COUNT:
                raise AnalysisError(
                    f"{path}:{line_number}: expected {DIAG_COLUMN_COUNT} columns"
                )
            try:
                timestamp = int(row[0])
            except ValueError as error:
                raise AnalysisError(
                    f"{path}:{line_number}: invalid timestamp_ns"
                ) from error
            if timestamp < 0:
                raise AnalysisError(f"{path}:{line_number}: negative integer column 1")
            if timestamps and timestamp <= timestamps[-1]:
                raise AnalysisError(
                    f"{path}:{line_number}: timestamps are not strictly increasing"
                )
            if row[1] not in DIAG_STATUSES:
                raise AnalysisError(f"{path}:{line_number}: invalid status {row[1]!r}")
            for index in DIAG_INT_COLUMNS:
                try:
                    value = int(row[index])
                except ValueError as error:
                    raise AnalysisError(
                        f"{path}:{line_number}: invalid diag integer column {index + 1}"
                    ) from error
                if value < 0:
                    raise AnalysisError(
                        f"{path}:{line_number}: negative diag integer column {index + 1}"
                    )
            for index in DIAG_BOOL_COLUMNS:
                if row[index] not in {"0", "1"}:
                    raise AnalysisError(
                        f"{path}:{line_number}: diag boolean column {index + 1} "
                        "must be 0 or 1"
                    )
            for index in DIAG_FLOAT_COLUMNS:
                try:
                    value = float(row[index])
                except ValueError as error:
                    raise AnalysisError(
                        f"{path}:{line_number}: invalid diag float column {index + 1}"
                    ) from error
                if not math.isfinite(value) or value < 0.0:
                    raise AnalysisError(
                        f"{path}:{line_number}: diag float column {index + 1} "
                        "must be finite and non-negative"
                    )
            timestamps.append(timestamp)
            statuses.append(row[1])
            try:
                segment_id = int(row[13])
            except ValueError as error:
                raise AnalysisError(
                    f"{path}:{line_number}: invalid segment_id"
                ) from error
            if segment_id < 0:
                raise AnalysisError(f"{path}:{line_number}: negative segment_id")
            if row[1] == "ok":
                if last_ok_segment_id is None:
                    segment_count = 1
                elif segment_id != last_ok_segment_id:
                    if segment_id <= last_ok_segment_id:
                        raise AnalysisError(
                            f"{path}:{line_number}: ok segment_id must increase"
                        )
                    segment_count += 1
                last_ok_segment_id = segment_id
            low_connectivity_count += int(row[5])
            cheirality_count += int(row[10])
            pnp_success_count += int(row[14])
            outliers_culled_total += int(row[16])
            if row[18] == "1":
                all_keyframe_count += 1
                if row[1] == "ok":
                    accepted_keyframes.append(timestamp)
    if not timestamps:
        raise AnalysisError(f"diag.csv has no data rows: {path}")
    return DiagReport(
        tuple(timestamps),
        tuple(statuses),
        tuple(accepted_keyframes),
        all_keyframe_count,
        segment_count,
        low_connectivity_count,
        cheirality_count,
        pnp_success_count,
        outliers_culled_total,
    )


def _discover(run_root: Path) -> dict[tuple[str, int, str], Path]:
    if not run_root.is_dir():
        raise AnalysisError(f"run root is not a directory: {run_root}")
    benches: dict[tuple[str, int, str], Path] = {}
    tasks = run_root / "tasks"
    if not tasks.is_dir():
        raise AnalysisError(f"missing tasks directory: {tasks}")
    unexpected_names: list[str] = []
    for task in sorted(tasks.iterdir()):
        match = TASK_PATTERN.fullmatch(task.name)
        if match is None:
            if task.is_dir() and task.name not in {"build", "unit"}:
                unexpected_names.append(task.name)
            continue
        role, repeat_text, sequence = match.groups()
        key = (role, int(repeat_text), sequence)
        bench = task / "artifacts" / "bench"
        if key in benches:
            raise AnalysisError(f"duplicate task: {task.name}")
        missing = [name for name in ARTIFACTS if not (bench / name).is_file()]
        if missing:
            raise AnalysisError(f"{task.name} missing artifacts: {', '.join(missing)}")
        benches[key] = bench
    expected = {(role, repeat, sequence) for role in ROLES for repeat in REPEATS for sequence in SEQUENCES}
    missing_keys = sorted(expected - benches.keys())
    missing_names = [f"{role}-{repeat}-{sequence}" for role, repeat, sequence in missing_keys]
    if missing_names or unexpected_names:
        raise AnalysisError(
            f"matrix task mismatch: missing={missing_names}, unexpected={unexpected_names}"
        )
    return benches


META_KEYS = {
    "schema_version",
    "sequence",
    "sequence_root",
    "created_utc",
    "code",
    "config",
    "config_label",
    "config_hash",
    "config_canonical_text",
    "paths",
    "warnings",
}
SUMMARY_KEYS = {
    "schema_version",
    "status",
    "sequence",
    "code",
    "config_label",
    "config_hash",
    "trajectory",
    "ate",
    "rpe",
    "robustness",
    "timing",
    "sync",
    "warnings",
}
TRAJECTORY_KEYS = {
    "image_frames",
    "poses_written",
    "ok",
    "rejected",
    "failed",
    "completion_rate",
    "coverage_rate",
    "segments",
    "total_keyframes",
    "total_track_only_frames",
}
ROBUSTNESS_KEYS = {
    "rejected",
    "failed",
    "low_connectivity",
    "cheirality",
    "reanchors",
    "pnp_successes",
    "pnp_fallbacks",
    "outliers_culled",
    "outliers_culled_unique",
    "outlier_reopts",
    "drops_skipped",
    "deferred_drops",
    "deferred_drop_ids",
    "evictable_marked",
    "tracks_evicted",
    "zombie_age_drops",
    "zombie_age_drop_ids",
}
SYNC_KEYS = {
    "pushed_left",
    "pushed_right",
    "emitted_stereo",
    "dropped_left",
    "dropped_right",
    "dropped_left_overflow",
    "dropped_right_overflow",
    "max_left_queue",
    "max_right_queue",
}


def _require_exact_keys(value: dict[str, Any], expected: set[str], label: str) -> None:
    if set(value) != expected:
        raise AnalysisError(
            f"{label} schema keys mismatch: missing={sorted(expected - set(value))}, "
            f"extra={sorted(set(value) - expected)}"
        )


def _require_nonnegative_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise AnalysisError(f"{label} must be a non-negative integer")
    return value


def _require_nonnegative_number(
    value: Any, label: str, *, nullable: bool = False
) -> float | None:
    if value is None and nullable:
        return None
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise AnalysisError(f"{label} must be finite and non-negative")
    return float(value)


def _config_value_text(value: Any, label: str) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise AnalysisError(f"{label} config value is non-finite")
        return format(value, ".17g")
    if isinstance(value, str):
        return value
    raise AnalysisError(f"{label} config value has unsupported type")


def _config_hash8(text: str) -> str:
    value = 14_695_981_039_346_656_037
    for byte in text.encode("utf-8"):
        value ^= byte
        value = (value * 1_099_511_628_211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"[:8]


def _validate_artifact_schema(
    meta: dict[str, Any], summary: dict[str, Any], bench: Path
) -> None:
    _require_exact_keys(meta, META_KEYS, "meta")
    _require_exact_keys(summary, SUMMARY_KEYS, "summary")
    if meta.get("schema_version") != 2:
        raise AnalysisError(f"meta schema_version must be 2: {bench}")
    if summary.get("schema_version") != 2:
        raise AnalysisError(f"summary schema_version must be 2: {bench}")
    _required_utc(meta.get("created_utc"), f"meta created_utc: {bench}")
    config_label = meta.get("config_label")
    if (
        not isinstance(config_label, str)
        or not config_label
        or summary.get("config_label") != config_label
    ):
        raise AnalysisError(f"meta/summary config_label mismatch: {bench}")
    paths = _required_object(meta.get("paths"), "meta.paths")
    _require_exact_keys(
        paths, {"bench_root", "output_dir", "layout_template"}, "meta.paths"
    )
    expected_paths = {
        "bench_root": "",
        "output_dir": "/task/artifacts/bench",
        "layout_template": "override",
    }
    if paths != expected_paths:
        raise AnalysisError(f"meta.paths identity mismatch: {bench}")
    if summary.get("status") not in {
        "completed",
        "completed_with_warnings",
        "completed_with_failures",
        "failed",
    }:
        raise AnalysisError(f"summary status is not completed: {bench}")
    for owner, value in (("meta", meta), ("summary", summary)):
        warnings = value.get("warnings")
        if not isinstance(warnings, list) or any(
            not isinstance(item, str) for item in warnings
        ):
            raise AnalysisError(f"{owner} warnings schema mismatch: {bench}")
    if summary["warnings"][: len(meta["warnings"])] != meta["warnings"]:
        raise AnalysisError(f"summary warnings do not preserve meta provenance: {bench}")

    config = _required_object(meta.get("config"), "meta.config")
    canonical = "".join(
        f"{key}={_config_value_text(value, key)}\n"
        for key, value in sorted(config.items())
        if isinstance(key, str) and key
    )
    if len(config) != len(canonical.splitlines()):
        raise AnalysisError(f"meta config contains an invalid key: {bench}")
    if meta.get("config_canonical_text") != canonical:
        raise AnalysisError(f"meta config canonical text mismatch: {bench}")
    expected_hash = _config_hash8(canonical)
    if meta.get("config_hash") != expected_hash or summary.get(
        "config_hash"
    ) != expected_hash:
        raise AnalysisError(f"config hash reproduction mismatch: {bench}")

    trajectory = _required_object(summary.get("trajectory"), "summary trajectory")
    _require_exact_keys(trajectory, TRAJECTORY_KEYS, "summary trajectory")
    for key in TRAJECTORY_KEYS - {"completion_rate", "coverage_rate"}:
        _require_nonnegative_int(trajectory.get(key), f"trajectory.{key}")

    robustness = _required_object(summary.get("robustness"), "summary robustness")
    _require_exact_keys(robustness, ROBUSTNESS_KEYS, "summary robustness")
    for key in ROBUSTNESS_KEYS:
        _require_nonnegative_int(robustness.get(key), f"robustness.{key}")
    if summary.get("status") == "failed":
        if robustness["failed"] == 0 or not summary["warnings"]:
            raise AnalysisError(
                f"failed summary lacks typed failure evidence: {bench}"
            )
        expected_status = "failed"
    elif robustness["failed"]:
        expected_status = "completed_with_failures"
    elif summary["warnings"]:
        expected_status = "completed_with_warnings"
    else:
        expected_status = "completed"
    if summary.get("status") != expected_status:
        raise AnalysisError(
            f"summary status mismatch: expected {expected_status}: {bench}"
        )

    timing = _required_object(summary.get("timing"), "summary timing")
    _require_exact_keys(
        timing, {"wall_s", "rtf", "rectify", "frontend", "estimator", "total"},
        "summary timing",
    )
    _require_nonnegative_number(timing.get("wall_s"), "timing.wall_s")
    _require_nonnegative_number(timing.get("rtf"), "timing.rtf", nullable=True)
    for stage in ("rectify", "frontend", "estimator", "total"):
        values = _required_object(timing.get(stage), f"timing.{stage}")
        _require_exact_keys(values, {"mean", "p95", "max"}, f"timing.{stage}")
        for key in ("mean", "p95", "max"):
            _require_nonnegative_number(values.get(key), f"timing.{stage}.{key}")

    for metric in ("ate", "rpe"):
        if summary.get("status") == "failed":
            if summary.get(metric) is not None:
                raise AnalysisError(f"failed summary {metric} must be null: {bench}")
            continue
        report = _required_object(summary.get(metric), f"summary {metric}")
        _require_exact_keys(report, {"trans", "rot_deg"}, f"summary {metric}")
        for component in ("trans", "rot_deg"):
            stats = _required_object(report.get(component), f"{metric}.{component}")
            _require_exact_keys(
                stats, {"rmse", "mean", "median", "stddev", "max"},
                f"{metric}.{component}",
            )
            for key in stats:
                _require_nonnegative_number(
                    stats.get(key), f"{metric}.{component}.{key}"
                )

    sync = _required_object(summary.get("sync"), "summary sync")
    _require_exact_keys(sync, SYNC_KEYS, "summary sync")
    for key in SYNC_KEYS:
        _require_nonnegative_int(sync.get(key), f"sync.{key}")


def _load_run(role: str, repeat: int, sequence: str, bench: Path, gt_root: Path) -> RunReport:
    meta = _read_json(bench / "meta.json")
    summary = _read_json(bench / "summary.json")
    _validate_artifact_schema(meta, summary, bench)
    if meta.get("sequence") != sequence or summary.get("sequence") != sequence:
        raise AnalysisError(f"sequence identity mismatch: {bench}")
    canonical_config = meta.get("config_canonical_text")
    if not isinstance(canonical_config, str) or not canonical_config:
        raise AnalysisError(f"meta missing config_canonical_text: {bench / 'meta.json'}")
    if summary.get("config_hash") != meta.get("config_hash"):
        raise AnalysisError(f"meta/summary config_hash mismatch: {bench}")
    typed_fatal = summary.get("status") == "failed"
    diag = _read_diag(bench / "diag.csv")
    fatal_indices = [
        index
        for index, status in enumerate(diag.statuses)
        if status in {"failed", "invalid_input"}
    ]
    if typed_fatal and (
        len(fatal_indices) != 1 or fatal_indices[0] != len(diag.statuses) - 1
    ):
        raise AnalysisError(
            f"typed fatal diag token must be unique and terminal: {bench}"
        )
    estimate = read_tum(bench / "est.tum", allow_empty=typed_fatal)
    keyframes = read_tum(bench / "kf.tum", allow_empty=typed_fatal)
    probe, probe_timestamps = _read_probe(bench / "probe-b.jsonl", role)
    if probe_timestamps != diag.timestamps_ns:
        raise AnalysisError(f"Probe B timestamps do not match diag.csv: {bench}")
    ok_timestamps = tuple(
        timestamp
        for timestamp, status in zip(diag.timestamps_ns, diag.statuses, strict=True)
        if status == "ok"
    )
    if tuple(pose.timestamp_ns for pose in estimate) != ok_timestamps:
        raise AnalysisError(f"est.tum timestamps do not match ok diag rows: {bench}")
    if (
        tuple(pose.timestamp_ns for pose in keyframes)
        != diag.accepted_keyframe_timestamps_ns
    ):
        raise AnalysisError(f"kf.tum timestamps do not match keyframe diag rows: {bench}")
    estimate_by_timestamp = {pose.timestamp_ns: pose for pose in estimate}
    for keyframe in keyframes:
        estimate_pose = estimate_by_timestamp[keyframe.timestamp_ns]
        if keyframe.raw_line != estimate_pose.raw_line:
            raise AnalysisError(
                f"keyframe pose does not match est.tum at {keyframe.timestamp_ns}: {bench}"
            )
    trajectory = summary.get("trajectory")
    if not isinstance(trajectory, dict):
        raise AnalysisError(f"summary missing trajectory: {bench}")
    expected_counts = {
        "image_frames": len(diag.timestamps_ns),
        "ok": diag.statuses.count("ok"),
        "failed": sum(
            status in {"failed", "invalid_input"} for status in diag.statuses
        ),
        "rejected": diag.statuses.count("rejected"),
        "poses_written": len(estimate),
        "total_keyframes": diag.all_keyframe_count,
        "total_track_only_frames": len(diag.timestamps_ns) - diag.all_keyframe_count,
        "segments": diag.segment_count,
    }
    for key, expected in expected_counts.items():
        if trajectory.get(key) != expected:
            raise AnalysisError(
                f"summary trajectory.{key} mismatch at {bench}: "
                f"expected {expected}, got {trajectory.get(key)!r}"
            )
    robustness = summary.get("robustness")
    if not isinstance(robustness, dict):
        raise AnalysisError(f"summary missing robustness: {bench}")
    for key in ("failed", "rejected"):
        expected = expected_counts[key]
        if robustness.get(key) != expected:
            raise AnalysisError(
                f"summary robustness.{key} mismatch at {bench}: "
                f"expected {expected}, got {robustness.get(key)!r}"
            )
    diag_robustness = {
        "low_connectivity": diag.low_connectivity_count,
        "cheirality": diag.cheirality_count,
        "pnp_successes": diag.pnp_success_count,
        "outliers_culled": diag.outliers_culled_total,
        "reanchors": max(diag.segment_count - 1, 0),
    }
    for key, expected in diag_robustness.items():
        if robustness.get(key) != expected:
            raise AnalysisError(
                f"summary robustness.{key} mismatch at {bench}: "
                f"expected {expected}, got {robustness.get(key)!r}"
            )
    rate_deltas = _reproduce_trajectory_rates(
        trajectory, diag, estimate, allow_single_frame=typed_fatal
    )
    if typed_fatal:
        ate = None
        rpe = None
        reproduction = {
            "max_abs_delta": max(rate_deltas.values(), default=0.0),
            "within_atol": True,
            "fields": dict(rate_deltas),
        }
    else:
        groundtruth = read_tumvi_groundtruth(gt_root / sequence)
        ate = compute_ate(estimate, groundtruth)
        rpe = compute_rpe(estimate, groundtruth)
        reproduction = _summary_reproduction(summary, ate, rpe)
        reproduction["fields"].update(rate_deltas)
        reproduction["max_abs_delta"] = max(
            float(reproduction["max_abs_delta"]), *rate_deltas.values()
        )
    hashes = {name: _sha256(bench / name) for name in ARTIFACTS}
    exact_hashes = dict(hashes)
    exact_hashes["meta.json"] = _canonical_sha256(_canonical_meta(meta))
    exact_hashes["summary.json"] = _canonical_sha256(_canonical_summary(summary))
    return RunReport(
        role,
        repeat,
        sequence,
        bench,
        meta,
        summary,
        ate,
        rpe,
        hashes,
        exact_hashes,
        probe,
        reproduction,
    )


def _common_delta(
    off_keys: Iterable[Any],
    off_values: np.ndarray,
    on_keys: Iterable[Any],
    on_values: np.ndarray,
) -> tuple[dict[str, Any], list[Any]]:
    off = dict(zip(off_keys, off_values, strict=True))
    on = dict(zip(on_keys, on_values, strict=True))
    common = sorted(off.keys() & on.keys())
    if not common:
        raise AnalysisError("paired comparison has no common support")
    deltas = np.array([on[key] - off[key] for key in common], dtype=np.float64)
    tail_index = int(np.argmax(deltas))
    return (
        {
            "counts": {
                "off": len(off),
                "on": len(on),
                "intersection": len(common),
                "off_only": len(off.keys() - on.keys()),
                "on_only": len(on.keys() - off.keys()),
            },
            "delta_on_minus_off": _delta_stats(deltas),
            "top_tail": {"key": common[tail_index], "delta": float(deltas[tail_index])},
        },
        common,
    )


def _native_json(run: RunReport) -> dict[str, Any]:
    if run.ate is None or run.rpe is None:
        ate_json = None
        rpe_json = None
        association_json = None
    else:
        ate_json = {
            "trans_m": _stats(run.ate.trans_m),
            "rot_deg": _stats(run.ate.rot_deg),
        }
        rpe_json = {
            "trans_m": _stats(run.rpe.trans_m),
            "rot_deg": _stats(run.rpe.rot_deg),
            "dropped_no_partner": run.rpe.dropped_no_partner,
        }
        association_json = _association_json(run.ate.association)
    return {
        "ate": ate_json,
        "rpe": rpe_json,
        "association": association_json,
        "summary_reproduction": run.summary_reproduction,
        "trajectory": run.summary.get("trajectory"),
        "robustness": run.summary.get("robustness"),
        "timing": run.summary.get("timing"),
        "probe": run.probe,
        "artifact_path": str(run.bench),
    }


def _median(values: Iterable[float]) -> float:
    return float(np.median(np.asarray(list(values), dtype=np.float64)))


def _nested_number(value: dict[str, Any], *keys: str) -> float:
    current: Any = value
    try:
        for key in keys:
            current = current[key]
        result = float(current)
    except (KeyError, TypeError, ValueError) as error:
        raise AnalysisError(f"missing numeric field: {'.'.join(keys)}") from error
    if not math.isfinite(result):
        raise AnalysisError(f"non-finite numeric field: {'.'.join(keys)}")
    return result


def _role_median(runs: list[RunReport]) -> dict[str, Any]:
    return {
        "ate_trans_rmse_m": _median(_stats(run.ate.trans_m)["rmse"] for run in runs),
        "ate_rot_rmse_deg": _median(_stats(run.ate.rot_deg)["rmse"] for run in runs),
        "rpe_trans_rmse_m": _median(_stats(run.rpe.trans_m)["rmse"] for run in runs),
        "rpe_rot_rmse_deg": _median(_stats(run.rpe.rot_deg)["rmse"] for run in runs),
        "association_count": int(_median(len(run.ate.timestamps_ns) for run in runs)),
        "rpe_pair_count": int(_median(len(run.rpe.keys) for run in runs)),
        "out_of_range": int(_median(run.ate.association.dropped_out_of_range for run in runs)),
        "over_threshold": int(_median(run.ate.association.dropped_over_threshold for run in runs)),
        "dropped_no_partner": int(_median(run.rpe.dropped_no_partner for run in runs)),
        "completion_rate": _median(_nested_number(run.summary, "trajectory", "completion_rate") for run in runs),
        "coverage_rate": _median(_nested_number(run.summary, "trajectory", "coverage_rate") for run in runs),
        "segments": _median(_nested_number(run.summary, "trajectory", "segments") for run in runs),
        "eligible_retry": _median(run.probe["eligible"] for run in runs),
        "attempted_retry": _median(run.probe["attempted"] for run in runs),
        "accepted_retry": _median(run.probe["accepted"] for run in runs),
        "failed_retry": _median(run.probe["failed"] for run in runs),
        "fallback": _median(run.probe["fallback"] for run in runs),
    }


def _repeatability(runs: dict[tuple[str, int, str], RunReport]) -> dict[str, Any]:
    sequences: dict[str, Any] = {}
    for sequence in SEQUENCES:
        sequences[sequence] = {}
        for role in ROLES:
            role_runs = [runs[(role, repeat, sequence)] for repeat in REPEATS]
            hashes = {
                "est.tum": [run.exact_hashes["est.tum"] for run in role_runs],
                "diag.csv": [run.exact_hashes["diag.csv"] for run in role_runs],
                "kf.tum": [run.exact_hashes["kf.tum"] for run in role_runs],
                "probe-b.jsonl": [run.exact_hashes["probe-b.jsonl"] for run in role_runs],
                "meta_without_created_utc": [run.exact_hashes["meta.json"] for run in role_runs],
                "summary_without_timing": [run.exact_hashes["summary.json"] for run in role_runs],
            }
            role_exact = all(len(set(values)) == 1 for values in hashes.values())
            sequences[sequence][role] = {"exact": role_exact, "hashes": hashes}
    return sequences


def _config_identity(runs: dict[tuple[str, int, str], RunReport]) -> tuple[bool, list[str]]:
    reasons: list[str] = []
    for role in ROLES:
        role_hashes = {
            runs[(role, repeat, sequence)].meta.get("config_hash")
            for sequence in SEQUENCES
            for repeat in REPEATS
        }
        role_configs = {
            runs[(role, repeat, sequence)].meta.get("config_canonical_text")
            for sequence in SEQUENCES
            for repeat in REPEATS
        }
        if len(role_hashes) != 1 or None in role_hashes:
            reasons.append(f"{role}: config_hash differs across room/repeat")
        if len(role_configs) != 1:
            reasons.append(f"{role}: canonical config differs across room/repeat")
    for sequence in SEQUENCES:
        for repeat in REPEATS:
            off = runs[("off", repeat, sequence)].meta
            on = runs[("on", repeat, sequence)].meta
            off_config = _required_object(off.get("config"), "off meta.config")
            on_config = _required_object(on.get("config"), "on meta.config")
            off_text = off.get("config_canonical_text")
            on_text = on.get("config_canonical_text")
            if off.get("config_hash") == on.get("config_hash"):
                reasons.append(f"{sequence} repeat {repeat}: role config hashes are equal")
            activation = "estimator.enable_pnp_cheirality_retry=true\n"
            dataset_format = "session.dataset_format=tum_vi\n"
            if (
                dataset_format not in off_text
                or dataset_format not in on_text
                or off_config.get("session.dataset_format") != "tum_vi"
                or on_config.get("session.dataset_format") != "tum_vi"
                or "estimator.enable_pnp_cheirality_retry" in off_config
                or on_config.get("estimator.enable_pnp_cheirality_retry") is not True
                or "estimator.enable_pnp_cheirality_retry=" in off_text
                or on_text.count(activation) != 1
                or on_text.replace(activation, "") != off_text
            ):
                reasons.append(f"{sequence} repeat {repeat}: config diff is not retry-only")
    return not reasons, reasons


def _required_object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AnalysisError(f"missing {label} identity")
    return value


def _required_sha256(value: Any, label: str, *, prefixed: bool = False) -> str:
    if not isinstance(value, str):
        raise AnalysisError(f"missing {label} identity")
    digest = value.removeprefix("sha256:") if prefixed else value
    if (prefixed and not value.startswith("sha256:")) or not HEX_SHA256_PATTERN.fullmatch(
        digest
    ):
        raise AnalysisError(f"invalid {label} identity")
    return value


def _required_utc(value: Any, label: str) -> dt.datetime:
    if not isinstance(value, str) or not RFC3339_UTC_PATTERN.fullmatch(value):
        raise AnalysisError(f"invalid RFC3339 UTC timestamp: {label}")
    try:
        parsed = dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise AnalysisError(f"invalid RFC3339 UTC timestamp: {label}") from error
    if parsed.utcoffset() != dt.timedelta(0):
        raise AnalysisError(f"timestamp is not UTC: {label}")
    return parsed


def _validate_runtime_file(value: Any, label: str) -> None:
    item = _required_object(value, label)
    _require_exact_keys(item, {"path", "realpath", "sha256"}, label)
    for key in ("path", "realpath"):
        path = item.get(key)
        if not isinstance(path, str) or not path.startswith("/"):
            raise AnalysisError(f"{label}.{key} must be an absolute path")
    _required_sha256(item.get("sha256"), f"{label}.sha256")


def _validate_runtime_linkage(
    value: Any, binaries: dict[str, Any]
) -> None:
    linkage = _required_object(value, "runtime linkage")
    _require_exact_keys(
        linkage, {"schema_version", "method", "binaries"}, "runtime linkage"
    )
    if linkage.get("schema_version") != 1 or linkage.get(
        "method"
    ) != "readelf-ldd-sha256":
        raise AnalysisError("runtime linkage identity mismatch")
    linked = _required_object(linkage.get("binaries"), "runtime linkage binaries")
    if set(linked) != PREFLIGHT_BINARIES:
        raise AnalysisError("runtime linkage binary set mismatch")
    for name in sorted(PREFLIGHT_BINARIES):
        item = _required_object(linked.get(name), f"runtime linkage {name}")
        _require_exact_keys(
            item,
            {
                "binary_sha256",
                "interpreter",
                "libraries",
                "virtual",
                "missing",
            },
            f"runtime linkage {name}",
        )
        if item.get("binary_sha256") != binaries[name]:
            raise AnalysisError(f"runtime linkage binary SHA mismatch: {name}")
        _validate_runtime_file(item.get("interpreter"), f"runtime interpreter {name}")
        libraries = item.get("libraries")
        if not isinstance(libraries, list) or not libraries:
            raise AnalysisError(f"runtime linkage libraries missing: {name}")
        keys: list[tuple[str, str, str]] = []
        for index, library in enumerate(libraries):
            entry = _required_object(
                library, f"runtime linkage {name} library {index}"
            )
            _require_exact_keys(
                entry,
                {"soname", "path", "realpath", "sha256"},
                f"runtime linkage {name} library {index}",
            )
            soname = entry.get("soname")
            if not isinstance(soname, str) or not soname:
                raise AnalysisError(f"runtime linkage SONAME invalid: {name}")
            _validate_runtime_file(
                {key: entry[key] for key in ("path", "realpath", "sha256")},
                f"runtime library {name}/{soname}",
            )
            keys.append((soname, entry["path"], entry["realpath"]))
        if keys != sorted(set(keys)):
            raise AnalysisError(f"runtime linkage libraries are not canonical: {name}")
        virtual = item.get("virtual")
        if (
            not isinstance(virtual, list)
            or any(not isinstance(entry, str) or not entry for entry in virtual)
            or virtual != sorted(set(virtual))
        ):
            raise AnalysisError(f"runtime linkage virtual entries invalid: {name}")
        if item.get("missing") != []:
            raise AnalysisError(f"runtime linkage has missing dependencies: {name}")


def _validate_task_statuses(
    run_root: Path,
    run: dict[str, Any],
    receipt: dict[str, Any],
    runs: dict[tuple[str, int, str], RunReport],
) -> None:
    run_status = _read_json(run_root / "run-status.json")
    if run_status.get("schema_version") != 1:
        raise AnalysisError("run status schema_version mismatch")
    if (
        run_status.get("run_id") != run.get("run_id")
        or run_status.get("profile") != CONFIRMATORY_PROFILE
    ):
        raise AnalysisError("run status identity mismatch")
    status_summary = _required_object(run_status.get("tasks"), "run status tasks")
    expected_names = {
        "build",
        "unit",
        *{
            f"{role}-{repeat}-{sequence}"
            for role in ROLES
            for repeat in REPEATS
            for sequence in SEQUENCES
        },
    }
    if set(status_summary) != expected_names:
        raise AnalysisError("run status task set mismatch")
    commands = _required_object(
        receipt.get("benchmark_commands"), "benchmark_commands"
    )
    infrastructure_commands = {
        "build": _expected_build_command(run),
        "unit": _expected_unit_command(run),
    }
    task_states: list[str] = []
    task_times: dict[str, tuple[dt.datetime, dt.datetime]] = {}
    for name in sorted(expected_names):
        status_path = run_root / "tasks" / name / "status.json"
        status = _read_json(status_path)
        if status_summary.get(name) != status:
            raise AnalysisError(f"task status receipt mismatch: {name}")
        if status.get("schema_version") != 1 or status.get("name") != name:
            raise AnalysisError(f"task status identity mismatch: {name}")
        return_code = status.get("return_code")
        if not isinstance(return_code, int) or isinstance(return_code, bool):
            raise AnalysisError(f"task {name} return_code must be an integer")
        command = status.get("command")
        if not isinstance(command, str) or not command:
            raise AnalysisError(f"task {name} command is missing")
        started = _required_utc(status.get("started_utc"), f"task {name}.started_utc")
        finished = _required_utc(
            status.get("finished_utc"), f"task {name}.finished_utc"
        )
        if finished < started:
            raise AnalysisError(f"task {name} timestamp order mismatch")
        task_times[name] = (started, finished)
        if name in {"build", "unit"}:
            if (
                return_code != 0
                or status.get("kind") != name
                or status.get("state") != "passed"
            ):
                raise AnalysisError(f"task {name} kind mismatch")
            if command != infrastructure_commands[name]:
                raise AnalysisError(f"task {name} command mismatch")
            if status.get("failure_reason") is not None:
                raise AnalysisError(f"task {name} failure_reason mismatch")
            task_states.append("passed")
            continue
        match = TASK_PATTERN.fullmatch(name)
        assert match is not None
        role, _repeat, sequence = match.groups()
        if status.get("kind") != "tum-vi":
            raise AnalysisError(f"task {name} kind mismatch")
        report = runs[(role, int(_repeat), sequence)]
        meta_created = _required_utc(
            report.meta.get("created_utc"), f"task {name} meta.created_utc"
        )
        if not started <= meta_created <= finished:
            raise AnalysisError(f"task {name} meta.created_utc is outside task window")
        summary_status = report.summary.get("status")
        robustness = _required_object(
            report.summary.get("robustness"), "summary robustness"
        )
        warnings = report.summary.get("warnings")
        if return_code != 0:
            if (
                summary_status != "failed"
                or robustness.get("failed") == 0
                or status.get("state") != "warning"
                or not warnings
            ):
                raise AnalysisError(f"task {name} has invalid execution failure")
            expected_state = "warning"
            expected_reason = (
                f"benchmark robustness.failed={robustness.get('failed')}, "
                f"robustness.rejected={robustness.get('rejected')}"
            )
        elif robustness.get("failed") != 0 or robustness.get("rejected") != 0:
            expected_state = "warning"
            expected_reason = (
                f"benchmark robustness.failed={robustness.get('failed')}, "
                f"robustness.rejected={robustness.get('rejected')}"
            )
        elif summary_status == "completed":
            expected_state = "passed"
            expected_reason = None
        else:
            expected_state = "warning"
            expected_reason = warnings[-1] if warnings else None
        if status.get("benchmark_status") != summary_status:
            raise AnalysisError(f"task benchmark_status mismatch: {name}")
        if status.get("state") != expected_state:
            raise AnalysisError(f"task state/summary status mismatch: {name}")
        if status.get("failure_reason") != expected_reason:
            raise AnalysisError(f"task failure_reason mismatch: {name}")
        if command != commands[role][sequence]:
            raise AnalysisError(f"task command mismatch: {name}")
        task_states.append(expected_state)
    if "failed" in task_states:
        expected_run_state = "failed"
    elif "warning" in task_states:
        expected_run_state = "warning"
    else:
        expected_run_state = "passed"
    if run_status.get("state") != expected_run_state:
        raise AnalysisError(
            f"run terminal state mismatch: expected {expected_run_state}, "
            f"got {run_status.get('state')!r}"
        )
    receipt_created = _required_utc(
        receipt.get("created_utc"), "preflight receipt.created_utc"
    )
    if task_times["build"][1] > task_times["unit"][0]:
        raise AnalysisError("build/unit timestamp order mismatch")
    if task_times["unit"][1] > receipt_created:
        raise AnalysisError("unit/preflight receipt timestamp order mismatch")
    first_benchmark_started = min(
        task_times[f"{role}-{repeat}-{sequence}"][0]
        for role in ROLES
        for repeat in REPEATS
        for sequence in SEQUENCES
    )
    if receipt_created > first_benchmark_started:
        raise AnalysisError("preflight receipt/benchmark timestamp order mismatch")
    previous_finished: dt.datetime | None = None
    for role, repeat in EXPECTED_BATCH_ORDER:
        starts = [
            task_times[f"{role}-{repeat}-{sequence}"][0]
            for sequence in SEQUENCES
        ]
        finishes = [
            task_times[f"{role}-{repeat}-{sequence}"][1]
            for sequence in SEQUENCES
        ]
        if previous_finished is not None and previous_finished > min(starts):
            raise AnalysisError(f"benchmark batch timestamp overlap: {role}-{repeat}")
        previous_finished = max(finishes)
    run_started = _required_utc(run_status.get("started_utc"), "run started_utc")
    run_updated = _required_utc(run_status.get("updated_utc"), "run updated_utc")
    run_finished = _required_utc(run_status.get("finished_utc"), "run finished_utc")
    if (
        run_started != task_times["build"][0]
        or run_updated < previous_finished
        or run_finished < run_updated
    ):
        raise AnalysisError("run/task timestamp order mismatch")


def _validate_run_identity(
    run_root: Path,
    runs: dict[tuple[str, int, str], RunReport],
    gt_root: Path,
) -> None:
    run = _read_json(run_root / "run.json")
    receipt = _read_json(run_root / "preflight-receipt.json")
    _require_exact_keys(receipt, PREFLIGHT_RECEIPT_KEYS, "preflight receipt")
    if run.get("schema_version") != 1 or receipt.get("schema_version") != 1:
        raise AnalysisError("run/preflight schema_version mismatch")
    if run.get("profile") != CONFIRMATORY_PROFILE or receipt.get(
        "profile"
    ) != CONFIRMATORY_PROFILE:
        raise AnalysisError("run/preflight profile identity mismatch")

    source_manifest = _required_object(run.get("source_manifest"), "source manifest")
    if source_manifest.get("schema_version") != 1:
        raise AnalysisError("source manifest schema_version mismatch")
    expected_source_id = "sha256:" + _canonical_sha256(source_manifest)
    source_id = _required_sha256(run.get("source_id"), "source", prefixed=True)
    if source_id != expected_source_id or receipt.get("source_id") != source_id:
        raise AnalysisError("source identity mismatch")
    _required_sha256(run.get("source_archive_sha256"), "source archive")
    _required_sha256(run.get("source_bundle_sha256"), "source bundle")

    receipt_without_id = dict(receipt)
    receipt_id = receipt_without_id.pop("preflight_receipt_id", None)
    expected_receipt_id = "sha256:" + _canonical_sha256(receipt_without_id)
    if (
        _required_sha256(receipt_id, "preflight receipt", prefixed=True)
        != expected_receipt_id
    ):
        raise AnalysisError("preflight receipt identity mismatch")

    dataset = _required_object(receipt.get("dataset"), "dataset")
    _require_exact_keys(
        dataset,
        {"manifest_id", "content_id", "receipt_sha256", "groundtruth"},
        "preflight dataset",
    )
    if run.get("dataset_format") != "tum-vi":
        raise AnalysisError("dataset format identity mismatch")
    dataset_manifest = _required_object(
        run.get("dataset_manifest"), "dataset manifest"
    )
    expected_manifest_id = "sha256:" + _canonical_sha256(dataset_manifest)
    if run.get("dataset_manifest_id") != expected_manifest_id:
        raise AnalysisError("dataset manifest identity mismatch")
    if expected_manifest_id != EXPECTED_DATASET_MANIFEST_ID:
        raise AnalysisError("dataset manifest differs from frozen protocol")
    for run_key, receipt_key in (
        ("dataset_manifest_id", "manifest_id"),
        ("dataset_content_id", "content_id"),
    ):
        run_value = _required_sha256(run.get(run_key), run_key, prefixed=True)
        receipt_value = _required_sha256(
            dataset.get(receipt_key), f"dataset {receipt_key}", prefixed=True
        )
        if run_value != receipt_value:
            raise AnalysisError(f"dataset {receipt_key} identity mismatch")
    content_id = str(run["dataset_content_id"]).removeprefix("sha256:")
    dataset_root = run.get("dataset_root")
    if (
        not isinstance(dataset_root, str)
        or run.get("dataset_content_id") != EXPECTED_DATASET_CONTENT_ID
        or dataset_root != EXPECTED_DATASET_ROOT
        or not dataset_root.endswith(f"/by-content/sha256/{content_id}")
    ):
        raise AnalysisError("dataset root content identity mismatch")
    _required_sha256(dataset.get("receipt_sha256"), "dataset receipt")
    groundtruth = _required_object(dataset.get("groundtruth"), "dataset groundtruth")
    if set(groundtruth) != set(SEQUENCES):
        raise AnalysisError("dataset groundtruth identity set mismatch")
    for sequence in SEQUENCES:
        item = _required_object(
            groundtruth.get(sequence), f"dataset groundtruth {sequence}"
        )
        _require_exact_keys(
            item, {"path", "sha256"}, f"dataset groundtruth {sequence}"
        )
        expected_path = f"{sequence}/dso/gt_imu.csv"
        if item.get("path") != expected_path:
            raise AnalysisError(f"dataset groundtruth path mismatch: {sequence}")
        expected_digest = _required_sha256(
            item.get("sha256"), f"dataset groundtruth {sequence}"
        )
        actual_digest = _sha256(gt_root / sequence / "dso" / "gt_imu.csv")
        if actual_digest != expected_digest:
            raise AnalysisError(f"dataset groundtruth SHA mismatch: {sequence}")

    run_image = _required_object(run.get("image"), "run image")
    receipt_image = _required_object(receipt.get("image"), "preflight image")
    _require_exact_keys(
        receipt_image,
        {"oci_manifest_id", "oci_config_id", "toolchain_id", "rootfs_layers"},
        "preflight image",
    )
    for key in ("oci_manifest_id", "oci_config_id"):
        run_value = _required_sha256(run_image.get(key), f"run image {key}", prefixed=True)
        receipt_value = _required_sha256(
            receipt_image.get(key), f"preflight image {key}", prefixed=True
        )
        if run_value != receipt_value:
            raise AnalysisError(f"build image identity mismatch: {key}")
    if run_image.get("id") != run_image.get("oci_config_id") or run_image.get(
        "remote_id"
    ) not in {run_image.get("oci_manifest_id"), run_image.get("oci_config_id")}:
        raise AnalysisError("build image immutable identity mismatch")
    toolchain = _required_sha256(run_image.get("toolchain_id"), "run toolchain")
    if receipt_image.get("toolchain_id") != toolchain:
        raise AnalysisError("build toolchain identity mismatch")
    run_layers = run_image.get("rootfs_layers")
    receipt_layers = receipt_image.get("rootfs_layers")
    if not isinstance(run_layers, list) or not run_layers or run_layers != receipt_layers:
        raise AnalysisError("build image rootfs identity mismatch")
    for index, layer in enumerate(run_layers):
        _required_sha256(layer, f"rootfs layer {index}", prefixed=True)
    if len(set(run_layers)) != len(run_layers):
        raise AnalysisError("build image rootfs identity mismatch")

    binaries = _required_object(receipt.get("binaries"), "binary")
    if set(binaries) != PREFLIGHT_BINARIES:
        raise AnalysisError("binary identity set mismatch")
    _required_sha256(
        binaries.get("phad_vo_bench"), "binary identity phad_vo_bench"
    )
    for name, digest in binaries.items():
        if not isinstance(name, str) or not name:
            raise AnalysisError("invalid binary identity name")
        _required_sha256(digest, f"binary {name}")
    _validate_runtime_linkage(receipt.get("runtime_linkage"), binaries)
    inputs = _required_object(receipt.get("inputs"), "preflight input")
    if set(inputs) != {"protocol", "analyzer"}:
        raise AnalysisError("preflight input identity set mismatch")
    expected_input_paths = {
        "protocol": PROTOCOL_RELATIVE_PATH,
        "analyzer": ANALYZER_RELATIVE_PATH,
    }
    repository_root = Path(__file__).resolve().parents[1]
    for name in ("protocol", "analyzer"):
        item = _required_object(inputs[name], f"preflight input {name}")
        _require_exact_keys(item, {"path", "sha256"}, f"preflight input {name}")
        if item.get("path") != expected_input_paths[name]:
            raise AnalysisError(f"preflight input {name} path identity mismatch")
        input_digest = _required_sha256(
            item.get("sha256"), f"preflight input {name}"
        )
        entries = source_manifest.get("entries")
        if not isinstance(entries, list):
            raise AnalysisError("source manifest entries identity is invalid")
        matches = [
            entry
            for entry in entries
            if isinstance(entry, dict) and entry.get("path") == item["path"]
        ]
        if (
            len(matches) != 1
            or matches[0].get("type") != "file"
            or matches[0].get("sha256") != item.get("sha256")
        ):
            raise AnalysisError(
                f"preflight input {name} identity does not match source manifest"
            )
        current_digest = _sha256(repository_root / expected_input_paths[name])
        if current_digest != input_digest:
            raise AnalysisError(f"current {name} SHA does not match preflight")

    expected_batch_order = [list(item) for item in EXPECTED_BATCH_ORDER]
    if run.get("batch_order") != expected_batch_order or receipt.get(
        "batch_order"
    ) != expected_batch_order:
        raise AnalysisError("batch_order identity mismatch")
    if run.get("sequences") != list(SEQUENCES):
        raise AnalysisError("run sequence order identity mismatch")
    commands = _required_object(
        receipt.get("benchmark_commands"), "benchmark_commands"
    )
    if set(commands) != set(ROLES):
        raise AnalysisError("benchmark_commands role set mismatch")
    for role in ROLES:
        role_commands = _required_object(
            commands.get(role), f"benchmark_commands {role}"
        )
        if set(role_commands) != set(SEQUENCES):
            raise AnalysisError(f"benchmark_commands {role} sequence set mismatch")
        for sequence in SEQUENCES:
            if role_commands[sequence] != _expected_benchmark_command(sequence, role):
                raise AnalysisError(
                    f"benchmark_commands mismatch: {role}/{sequence}"
                )

    git = _required_object(source_manifest.get("git"), "source git")
    head = git.get("head")
    tree = git.get("tree")
    branch = git.get("branch")
    dirty = git.get("tracked_dirty")
    if not isinstance(head, str) or not re.fullmatch(r"[0-9a-f]{40}", head):
        raise AnalysisError("invalid source git head identity")
    if not isinstance(tree, str) or not re.fullmatch(r"[0-9a-f]{40}", tree):
        raise AnalysisError("invalid source git tree identity")
    if not isinstance(branch, str) or not branch or not isinstance(dirty, bool):
        raise AnalysisError("invalid source git branch/dirty identity")
    expected_code = {
        "git_commit": head,
        "git_commit_short": head[:7],
        "git_branch": branch,
        "git_dirty": dirty,
    }
    code_values = {
        _canonical_sha256(
            _required_object(run_report.meta.get("code"), "meta.code")
        )
        for run_report in runs.values()
    }
    if len(code_values) != 1:
        raise AnalysisError("meta.code identity differs across 36 tasks")
    for (role, repeat, sequence), run_report in runs.items():
        if run_report.meta.get("code") != expected_code:
            raise AnalysisError(
                f"meta.code identity mismatch: {role}-{repeat}-{sequence}"
            )
        summary_code = _required_object(
            run_report.summary.get("code"), "summary.code"
        )
        if summary_code != {
            "git_commit_short": head[:7],
            "git_dirty": dirty,
        }:
            raise AnalysisError(
                f"summary.code identity mismatch: {role}-{repeat}-{sequence}"
            )

    for sequence in SEQUENCES:
        expected_root = f"/datasets/tum-vi/{sequence}"
        roots = {
            runs[(role, repeat, sequence)].meta.get("sequence_root")
            for role in ROLES
            for repeat in REPEATS
        }
        if roots != {expected_root}:
            raise AnalysisError(
                f"dataset input identity mismatch for {sequence}: {sorted(map(str, roots))}"
            )
    _validate_task_statuses(run_root, run, receipt, runs)


def analyze(run_root: Path, gt_root: Path) -> dict[str, Any]:
    benches = _discover(run_root)
    runs = {
        key: _load_run(*key, bench, gt_root)
        for key, bench in sorted(benches.items())
    }
    _validate_run_identity(run_root, runs, gt_root)
    repeatability = _repeatability(runs)
    config_valid, config_reasons = _config_identity(runs)
    if not config_valid:
        raise AnalysisError("config identity mismatch: " + "; ".join(config_reasons))
    sequence_results: dict[str, Any] = {}
    hard_failures: list[str] = []
    review_reasons: list[str] = []
    inconclusive_reasons: list[str] = []
    ate_ratios: list[float] = []
    rpe_ratios: list[float] = []
    completion_deltas: list[float] = []
    coverage_deltas: list[float] = []
    eligible_total = 0.0
    eligible_observed_total = sum(
        run.probe["eligible"]
        for (role, _repeat, _sequence), run in runs.items()
        if role == "on"
    )
    repeatability_valid = all(
        repeatability[sequence][role]["exact"]
        for sequence in SEQUENCES
        for role in ROLES
    )
    numeric_available = all(
        run.ate is not None and run.rpe is not None for run in runs.values()
    )
    aggregate_sequences = repeatability_valid and numeric_available
    product_aggregation_valid = aggregate_sequences
    if not repeatability_valid:
        inconclusive_reasons.append("same-role exact repeatability failed")

    for sequence in SEQUENCES:
        native: dict[str, Any] = {role: {} for role in ROLES}
        for role in ROLES:
            for repeat in REPEATS:
                run = runs[(role, repeat, sequence)]
                native[role][str(repeat)] = _native_json(run)
                trajectory = run.summary.get("trajectory", {})
                robustness = run.summary.get("robustness", {})
                if trajectory.get("failed") != 0 or trajectory.get("rejected") != 0 or robustness.get("failed") != 0 or robustness.get("rejected") != 0:
                    hard_failures.append(f"{role}-{repeat}-{sequence}: failed/rejected is nonzero")
                if role == "on" and run.probe["eligible"] > run.probe["attempted"]:
                    inconclusive_reasons.append(
                        f"on-{repeat}-{sequence}: eligible retry was not attempted"
                    )

        paired: list[dict[str, Any]] = []
        support_review = False
        segments_ok = True
        for repeat in REPEATS:
            off = runs[("off", repeat, sequence)]
            on = runs[("on", repeat, sequence)]
            if off.ate is None or off.rpe is None or on.ate is None or on.rpe is None:
                paired.append({"repeat": repeat, "status": "not_evaluated"})
            else:
                ate_trans, _ = _common_delta(off.ate.timestamps_ns, off.ate.trans_m, on.ate.timestamps_ns, on.ate.trans_m)
                ate_rot, _ = _common_delta(off.ate.timestamps_ns, off.ate.rot_deg, on.ate.timestamps_ns, on.ate.rot_deg)
                rpe_trans, _ = _common_delta(off.rpe.keys, off.rpe.trans_m, on.rpe.keys, on.rpe.trans_m)
                rpe_rot, _ = _common_delta(off.rpe.keys, off.rpe.rot_deg, on.rpe.keys, on.rpe.rot_deg)
                paired.append(
                    {
                        "repeat": repeat,
                        "status": "evaluated",
                        "support": {
                            "timestamps": ate_trans["counts"],
                            "rpe_pairs": rpe_trans["counts"],
                        },
                        "common": {
                            "ate": {
                                "translation_delta_m": ate_trans["delta_on_minus_off"],
                                "rotation_delta_deg": ate_rot["delta_on_minus_off"],
                            },
                            "rpe": {
                                "translation_delta_m": rpe_trans["delta_on_minus_off"],
                                "rotation_delta_deg": rpe_rot["delta_on_minus_off"],
                            },
                        },
                        "tail": {
                            "ate_translation": ate_trans["top_tail"],
                            "ate_rotation": ate_rot["top_tail"],
                            "rpe_translation": rpe_trans["top_tail"],
                            "rpe_rotation": rpe_rot["top_tail"],
                        },
                    }
                )
                if ate_trans["counts"]["off_only"] or ate_trans["counts"]["on_only"] or rpe_trans["counts"]["off_only"] or rpe_trans["counts"]["on_only"]:
                    support_review = True
            off_segments = _nested_number(off.summary, "trajectory", "segments")
            on_segments = _nested_number(on.summary, "trajectory", "segments")
            if on_segments > off_segments:
                segments_ok = False
                hard_failures.append(f"{sequence} repeat {repeat}: segments increased")

        medians = (
            {
                role: _role_median(
                    [runs[(role, repeat, sequence)] for repeat in REPEATS]
                )
                for role in ROLES
            }
            if aggregate_sequences
            else None
        )
        ate_delta = None
        rpe_delta = None
        completion_delta = None
        coverage_delta = None
        ate_ratio: float | None = None
        rpe_ratio: float | None = None
        if medians is not None:
            off_median = medians["off"]
            on_median = medians["on"]
            ate_delta = on_median["ate_trans_rmse_m"] - off_median["ate_trans_rmse_m"]
            rpe_delta = on_median["rpe_trans_rmse_m"] - off_median["rpe_trans_rmse_m"]
            completion_delta = on_median["completion_rate"] - off_median["completion_rate"]
            coverage_delta = on_median["coverage_rate"] - off_median["coverage_rate"]
            if (
                off_median["ate_trans_rmse_m"] <= 0.0
                or off_median["rpe_trans_rmse_m"] <= 0.0
                or on_median["ate_trans_rmse_m"] <= 0.0
                or on_median["rpe_trans_rmse_m"] <= 0.0
            ):
                product_aggregation_valid = False
                inconclusive_reasons.append(
                    f"{sequence}: zero off/on accuracy denominator"
                )
            else:
                ate_ratio = (
                    on_median["ate_trans_rmse_m"] / off_median["ate_trans_rmse_m"]
                )
                rpe_ratio = (
                    on_median["rpe_trans_rmse_m"] / off_median["rpe_trans_rmse_m"]
                )
        sequence_reviews: list[str] = []
        if aggregate_sequences:
            if ate_delta > max(0.010, 0.10 * off_median["ate_trans_rmse_m"]):
                sequence_reviews.append("ate_tail")
            if rpe_delta > max(0.005, 0.10 * off_median["rpe_trans_rmse_m"]):
                sequence_reviews.append("rpe_tail")
            if completion_delta < -0.010:
                sequence_reviews.append("completion")
            if coverage_delta < -0.010:
                sequence_reviews.append("coverage")
        if support_review:
            sequence_reviews.append("support")
        if sequence_reviews:
            review_reasons.append(f"{sequence}: {', '.join(sequence_reviews)}")
        sequence_results[sequence] = {
            "roles": {
                role: {
                    "repeats": native[role],
                    "native_median": medians[role] if medians is not None else None,
                    "retry": (
                        {
                            "eligible": int(medians[role]["eligible_retry"]),
                            "attempted": int(medians[role]["attempted_retry"]),
                            "accepted": int(medians[role]["accepted_retry"]),
                            "failed": int(medians[role]["failed_retry"]),
                            "fallback": int(medians[role]["fallback"]),
                        }
                        if medians is not None
                        else None
                    ),
                }
                for role in ROLES
            },
            "paired_repeats": paired,
            "comparison": {
                "product_aggregation": (
                    "evaluated" if medians is not None else "not_evaluated"
                ),
                "ate_ratio": ate_ratio if medians is not None else None,
                "rpe_ratio": rpe_ratio if medians is not None else None,
                "ate_delta_m": ate_delta if medians is not None else None,
                "rpe_delta_m": rpe_delta if medians is not None else None,
                "completion_delta": (
                    completion_delta if medians is not None else None
                ),
                "coverage_delta": coverage_delta if medians is not None else None,
                "segments_ok": segments_ok,
                "review_reasons": sequence_reviews,
            },
        }
        if ate_ratio is not None and rpe_ratio is not None:
            ate_ratios.append(ate_ratio)
            rpe_ratios.append(rpe_ratio)
        if completion_delta is not None and coverage_delta is not None:
            completion_deltas.append(completion_delta)
            coverage_deltas.append(coverage_delta)
        if medians is not None:
            eligible_total += medians["on"]["eligible_retry"]

    suite: dict[str, Any] = {
        "eligible_retry_total": (
            int(eligible_total) if product_aggregation_valid else None
        ),
        "config_identity_valid": config_valid,
        "failed_total": int(sum(_nested_number(run.summary, "trajectory", "failed") for run in runs.values())),
        "rejected_total": int(sum(_nested_number(run.summary, "trajectory", "rejected") for run in runs.values())),
        "segments_nonincrease": not any("segments increased" in reason for reason in hard_failures),
    }
    if product_aggregation_valid:
        suite.update(
            {
                "product_aggregation": "evaluated",
                "g_ate": float(
                    math.exp(float(np.mean(np.log(np.asarray(ate_ratios)))))
                ),
                "g_rpe": float(
                    math.exp(float(np.mean(np.log(np.asarray(rpe_ratios)))))
                ),
                "mean_completion_delta": float(np.mean(completion_deltas)),
                "mean_coverage_delta": float(np.mean(coverage_deltas)),
            }
        )
        suite["accuracy_pass"] = (
            suite["g_ate"] <= 1.05 and suite["g_rpe"] <= 1.05
        )
        suite["availability_pass"] = (
            suite["mean_completion_delta"] >= -0.010
            and suite["mean_coverage_delta"] >= -0.010
        )
        if not suite["accuracy_pass"]:
            hard_failures.append("suite accuracy envelope exceeded")
        if not suite["availability_pass"]:
            hard_failures.append("suite availability envelope exceeded")
    else:
        suite.update(
            {
                "product_aggregation": "not_evaluated",
                "g_ate": None,
                "g_rpe": None,
                "mean_completion_delta": None,
                "mean_coverage_delta": None,
                "accuracy_pass": None,
                "availability_pass": None,
            }
        )
    if eligible_observed_total == 0:
        inconclusive_reasons.append("eligible_retry_total is zero")

    if hard_failures:
        verdict = "FAIL"
        reasons = hard_failures + inconclusive_reasons + review_reasons
    elif inconclusive_reasons:
        verdict = "INCONCLUSIVE"
        reasons = inconclusive_reasons + review_reasons
    elif review_reasons:
        verdict = "REVIEW"
        reasons = review_reasons
    else:
        verdict = "PASS"
        reasons = []
    return {
        "schema_version": 1,
        "verdict": verdict,
        "reasons": reasons,
        "repeatability": repeatability,
        "sequences": sequence_results,
        "suite": suite,
    }


def _write_json(value: dict[str, Any], path: Path) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError) as error:
        raise AnalysisError(f"cannot write output {path}: {error}") from error


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path, help="fetched 36-task run root")
    parser.add_argument(
        "--gt-root", type=Path, required=True, help="TUM-VI room dataset root"
    )
    parser.add_argument("--output", type=Path, required=True, help="result JSON path")
    arguments = parser.parse_args(argv)
    try:
        _write_json(analyze(arguments.run_root, arguments.gt_root), arguments.output)
    except (AnalysisError, np.linalg.LinAlgError) as error:
        print(f"analyze_tumvi_room_confirmatory: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

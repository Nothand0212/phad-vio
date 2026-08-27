#!/usr/bin/env python3
"""Compare one-step gyro predictions with accepted VIO posterior poses."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TextIO

from vio_vo_common_support import Metrics, TumPose, evaluate


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

STATE_FIELDS = (
    "timestamp_ns",
    "state_key",
    "prior_key",
    "window_size",
    "is_keyframe",
    "fusion_mode",
    "gyro_factor_n",
    "gyro_alignment_residual_rms_rad",
    "imu_sample_n",
    "imu_t_i_ns",
    "imu_t_j_ns",
    "imu_dt_s",
    "prediction_valid",
    "pred_p_x",
    "pred_p_y",
    "pred_p_z",
    "pred_q_x",
    "pred_q_y",
    "pred_q_z",
    "pred_q_w",
    "pred_bg_x",
    "pred_bg_y",
    "pred_bg_z",
    "graph_init_p_x",
    "graph_init_p_y",
    "graph_init_p_z",
    "graph_init_q_x",
    "graph_init_q_y",
    "graph_init_q_z",
    "graph_init_q_w",
    "post_p_x",
    "post_p_y",
    "post_p_z",
    "post_q_x",
    "post_q_y",
    "post_q_z",
    "post_q_w",
    "post_bg_x",
    "post_bg_y",
    "post_bg_z",
)

FUSED_MODES = ("gyro_visual", "full_visual_inertial")

BUCKET_FIELDS = (
    "bucket",
    "start_index",
    "end_index",
    "start_ns",
    "end_ns",
    "poses",
    "keyframes",
    "prior_advances",
    "pred_ate_rmse_m",
    "graph_init_ate_rmse_m",
    "post_ate_rmse_m",
    "ate_graph_init_minus_pred_m",
    "ate_post_minus_graph_init_m",
    "ate_post_minus_pred_m",
    "pred_rpe_rmse_m",
    "graph_init_rpe_rmse_m",
    "post_rpe_rmse_m",
    "rpe_graph_init_minus_pred_m",
    "rpe_post_minus_graph_init_m",
    "rpe_post_minus_pred_m",
    "matched",
    "rpe_pairs",
    "pose_translation_rms_m",
    "pose_translation_max_m",
    "pose_rotation_rms_deg",
    "pose_rotation_max_deg",
    "gyro_bias_rms_rps",
    "gyro_bias_max_rps",
)


class ProbeError(RuntimeError):
    """A sidecar or evaluator result violates the probe contract."""


@dataclass(frozen=True)
class StateRow:
    timestamp_ns: int
    state_key: int
    prior_key: int
    window_size: int
    is_keyframe: bool
    fusion_mode: str
    gyro_factor_n: int
    gyro_alignment_residual_rms_rad: float
    imu_sample_n: int
    imu_t_i_ns: int
    imu_t_j_ns: int
    imu_dt_s: float
    prediction_valid: bool
    pred_p: Vector3
    pred_q: Quaternion
    pred_bg: Vector3
    graph_init_p: Vector3
    graph_init_q: Quaternion
    post_p: Vector3
    post_q: Quaternion
    post_bg: Vector3


@dataclass(frozen=True)
class CorrectionSummary:
    poses: int
    pose_translation_rms_m: float
    pose_translation_max_m: float
    pose_rotation_rms_deg: float
    pose_rotation_max_deg: float
    gyro_bias_rms_rps: float
    gyro_bias_max_rps: float


@dataclass(frozen=True)
class MetricComparison:
    prediction: Metrics
    graph_initial: Metrics
    posterior: Metrics

    @property
    def ate_graph_init_minus_pred_m(self) -> float:
        return self.graph_initial.ate_rmse_m - self.prediction.ate_rmse_m

    @property
    def ate_post_minus_graph_init_m(self) -> float:
        return self.posterior.ate_rmse_m - self.graph_initial.ate_rmse_m

    @property
    def ate_post_minus_pred_m(self) -> float:
        return self.posterior.ate_rmse_m - self.prediction.ate_rmse_m

    @property
    def rpe_post_minus_pred_m(self) -> float:
        return self.posterior.rpe_rmse_m - self.prediction.rpe_rmse_m

    @property
    def rpe_graph_init_minus_pred_m(self) -> float:
        return self.graph_initial.rpe_rmse_m - self.prediction.rpe_rmse_m

    @property
    def rpe_post_minus_graph_init_m(self) -> float:
        return self.posterior.rpe_rmse_m - self.graph_initial.rpe_rmse_m


@dataclass(frozen=True)
class BucketResult:
    bucket: int
    start_index: int
    end_index: int
    start_ns: int
    end_ns: int
    poses: int
    keyframes: int
    prior_advances: int
    correction: CorrectionSummary
    metrics: Optional[MetricComparison]


@dataclass(frozen=True)
class AnalysisResult:
    accepted_rows: int
    prediction_valid_rows: int
    prediction_invalid_rows: int
    evaluation_rows: int
    outside_groundtruth_rows: int
    full: MetricComparison
    correction: CorrectionSummary
    buckets: tuple[BucketResult, ...]


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


def _boolean(value: str, field: str, path: Path, line: int) -> bool:
    parsed = _integer(value, field, path, line)
    if parsed not in (0, 1):
        raise ProbeError(f"{path}:{line}: {field} must be 0 or 1")
    return bool(parsed)


def _fusion_mode(value: str, path: Path, line: int) -> str:
    if value != "vision_only" and value not in FUSED_MODES:
        raise ProbeError(f"{path}:{line}: invalid fusion_mode")
    return value


def _vector(
    raw: dict[str, str], fields: tuple[str, str, str], path: Path, line: int
) -> Vector3:
    return tuple(_finite(raw[field], field, path, line) for field in fields)  # type: ignore[return-value]


def _quaternion(
    raw: dict[str, str],
    fields: tuple[str, str, str, str],
    path: Path,
    line: int,
) -> Quaternion:
    values = tuple(
        _finite(raw[field], field, path, line) for field in fields
    )
    norm = math.sqrt(sum(value * value for value in values))
    if abs(norm - 1.0) > 1e-9:
        raise ProbeError(f"{path}:{line}: {fields[0][:-1]} unit quaternion required")
    return values  # type: ignore[return-value]


def _validate_interval(row: StateRow, path: Path, line: int) -> None:
    if row.imu_sample_n < 0:
        raise ProbeError(f"{path}:{line}: imu_sample_n must be non-negative")
    if row.imu_dt_s < 0.0:
        raise ProbeError(f"{path}:{line}: imu_dt_s must be non-negative")
    if row.prediction_valid and row.imu_sample_n < 2:
        raise ProbeError(
            f"{path}:{line}: valid prediction needs at least two IMU samples"
        )
    if row.imu_sample_n == 0:
        if (
            row.imu_t_i_ns != 0
            or row.imu_t_j_ns != 0
            or row.imu_dt_s != 0.0
        ):
            raise ProbeError(
                f"{path}:{line}: empty IMU interval must use zero endpoints"
            )
        return
    if row.imu_sample_n == 1:
        if row.imu_t_i_ns != row.imu_t_j_ns or row.imu_dt_s != 0.0:
            raise ProbeError(
                f"{path}:{line}: one-sample IMU interval must have zero duration"
            )
        return
    if row.imu_t_i_ns >= row.imu_t_j_ns:
        raise ProbeError(f"{path}:{line}: IMU interval is not increasing")
    expected_dt = (row.imu_t_j_ns - row.imu_t_i_ns) * 1e-9
    tolerance = max(1e-12, abs(expected_dt) * 1e-12)
    if abs(row.imu_dt_s - expected_dt) > tolerance:
        raise ProbeError(
            f"{path}:{line}: IMU duration does not match endpoints"
        )
    if row.prediction_valid and row.imu_t_j_ns != row.timestamp_ns:
        raise ProbeError(
            f"{path}:{line}: prediction endpoint does not match state timestamp"
        )


def load_rows(path: Path) -> tuple[StateRow, ...]:
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc

    rows: list[StateRow] = []
    previous_timestamp: Optional[int] = None
    previous_state_key: Optional[int] = None
    with input_file:
        reader = csv.DictReader(input_file, skipinitialspace=True)
        fieldnames = tuple(reader.fieldnames or ())
        if (
            len(fieldnames) != len(set(fieldnames))
            or not set(STATE_FIELDS).issubset(fieldnames)
        ):
            raise ProbeError(f"{path}: CSV schema does not match VIO state probe")
        for line, raw in enumerate(reader, start=2):
            if None in raw or any(value is None for value in raw.values()):
                raise ProbeError(f"{path}:{line}: column count mismatch")
            timestamp_ns = _integer(
                raw["timestamp_ns"], "timestamp_ns", path, line
            )
            state_key = _integer(raw["state_key"], "state_key", path, line)
            if timestamp_ns < 0 or state_key < 0:
                raise ProbeError(
                    f"{path}:{line}: timestamp and state key must be non-negative"
                )
            if (
                previous_timestamp is not None
                and timestamp_ns <= previous_timestamp
            ):
                raise ProbeError(
                    f"{path}:{line}: timestamps are not strictly increasing"
                )
            if previous_state_key is not None and state_key <= previous_state_key:
                raise ProbeError(
                    f"{path}:{line}: state keys are not strictly increasing"
                )

            row = StateRow(
                timestamp_ns=timestamp_ns,
                state_key=state_key,
                prior_key=_integer(raw["prior_key"], "prior_key", path, line),
                window_size=_integer(
                    raw["window_size"], "window_size", path, line
                ),
                is_keyframe=_boolean(
                    raw["is_keyframe"], "is_keyframe", path, line
                ),
                fusion_mode=_fusion_mode(raw["fusion_mode"], path, line),
                gyro_factor_n=_integer(
                    raw["gyro_factor_n"], "gyro_factor_n", path, line
                ),
                gyro_alignment_residual_rms_rad=_finite(
                    raw["gyro_alignment_residual_rms_rad"],
                    "gyro_alignment_residual_rms_rad",
                    path,
                    line,
                ),
                imu_sample_n=_integer(
                    raw["imu_sample_n"], "imu_sample_n", path, line
                ),
                imu_t_i_ns=_integer(
                    raw["imu_t_i_ns"], "imu_t_i_ns", path, line
                ),
                imu_t_j_ns=_integer(
                    raw["imu_t_j_ns"], "imu_t_j_ns", path, line
                ),
                imu_dt_s=_finite(raw["imu_dt_s"], "imu_dt_s", path, line),
                prediction_valid=_boolean(
                    raw["prediction_valid"], "prediction_valid", path, line
                ),
                pred_p=_vector(
                    raw, ("pred_p_x", "pred_p_y", "pred_p_z"), path, line
                ),
                pred_q=_quaternion(
                    raw,
                    ("pred_q_x", "pred_q_y", "pred_q_z", "pred_q_w"),
                    path,
                    line,
                ),
                pred_bg=_vector(
                    raw, ("pred_bg_x", "pred_bg_y", "pred_bg_z"), path, line
                ),
                graph_init_p=_vector(
                    raw,
                    ("graph_init_p_x", "graph_init_p_y", "graph_init_p_z"),
                    path,
                    line,
                ),
                graph_init_q=_quaternion(
                    raw,
                    (
                        "graph_init_q_x",
                        "graph_init_q_y",
                        "graph_init_q_z",
                        "graph_init_q_w",
                    ),
                    path,
                    line,
                ),
                post_p=_vector(
                    raw, ("post_p_x", "post_p_y", "post_p_z"), path, line
                ),
                post_q=_quaternion(
                    raw,
                    ("post_q_x", "post_q_y", "post_q_z", "post_q_w"),
                    path,
                    line,
                ),
                post_bg=_vector(
                    raw, ("post_bg_x", "post_bg_y", "post_bg_z"), path, line
                ),
            )
            if (
                row.prior_key < 0
                or row.window_size < 0
                or row.gyro_factor_n < 0
                or row.gyro_alignment_residual_rms_rad < 0.0
            ):
                raise ProbeError(
                    f"{path}:{line}: graph counters must be non-negative"
                )
            if row.fusion_mode == "vision_only" and row.gyro_factor_n != 0:
                raise ProbeError(
                    f"{path}:{line}: vision-only graph has gyro factors"
                )
            _validate_interval(row, path, line)
            rows.append(row)
            previous_timestamp = timestamp_ns
            previous_state_key = state_key
    if not rows:
        raise ProbeError(f"{path}: no state rows")
    return tuple(rows)


def _load_groundtruth_timestamps(euroc_root: Path) -> tuple[int, ...]:
    path = euroc_root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
    try:
        input_file = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {path}: {exc}") from exc

    timestamps: list[int] = []
    previous: Optional[int] = None
    with input_file:
        reader = csv.reader(input_file, skipinitialspace=True)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise ProbeError(f"{path}: empty groundtruth CSV") from exc
        if not header or header[0] != "#timestamp":
            raise ProbeError(f"{path}: invalid groundtruth timestamp header")
        for line, raw in enumerate(reader, start=2):
            if len(raw) != len(header):
                raise ProbeError(f"{path}:{line}: column count mismatch")
            timestamp_ns = _integer(raw[0], "#timestamp", path, line)
            if previous is not None and timestamp_ns <= previous:
                raise ProbeError(
                    f"{path}:{line}: timestamps are not strictly increasing"
                )
            timestamps.append(timestamp_ns)
            previous = timestamp_ns
    if not timestamps:
        raise ProbeError(f"{path}: no groundtruth rows")
    return tuple(timestamps)


def _evaluation_support(
    rows: tuple[StateRow, ...],
    groundtruth_timestamps: tuple[int, ...],
    max_dt_ns: int = 2_500_000,
) -> tuple[tuple[StateRow, ...], int]:
    support: list[StateRow] = []
    outside = 0
    for row in rows:
        if not row.prediction_valid:
            continue
        if (
            row.timestamp_ns < groundtruth_timestamps[0]
            or row.timestamp_ns > groundtruth_timestamps[-1]
        ):
            outside += 1
            continue
        insertion = bisect.bisect_left(groundtruth_timestamps, row.timestamp_ns)
        candidates: list[int] = []
        if insertion < len(groundtruth_timestamps):
            candidates.append(groundtruth_timestamps[insertion])
        if insertion > 0:
            candidates.append(groundtruth_timestamps[insertion - 1])
        nearest = min(
            candidates,
            key=lambda timestamp: (
                abs(timestamp - row.timestamp_ns),
                timestamp,
            ),
        )
        if abs(nearest - row.timestamp_ns) > max_dt_ns:
            raise ProbeError(
                f"{row.timestamp_ns}: no groundtruth within {max_dt_ns} ns"
            )
        support.append(row)
    return tuple(support), outside


def _timestamp_text(timestamp_ns: int) -> str:
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    return f"{seconds}.{nanoseconds:09d}"


def _tum_pose(
    timestamp_ns: int, position: Vector3, quaternion: Quaternion
) -> TumPose:
    values = (*position, *quaternion)
    line = _timestamp_text(timestamp_ns) + " " + " ".join(
        f"{value:.17g}" for value in values
    )
    return TumPose(timestamp_ns=timestamp_ns, line=line)


def tum_poses(
    rows: tuple[StateRow, ...],
) -> tuple[tuple[TumPose, ...], tuple[TumPose, ...], tuple[TumPose, ...]]:
    valid = tuple(
        row
        for row in rows
        if row.prediction_valid and row.fusion_mode in FUSED_MODES
    )
    return (
        tuple(
            _tum_pose(row.timestamp_ns, row.pred_p, row.pred_q)
            for row in valid
        ),
        tuple(
            _tum_pose(
                row.timestamp_ns, row.graph_init_p, row.graph_init_q
            )
            for row in valid
        ),
        tuple(
            _tum_pose(row.timestamp_ns, row.post_p, row.post_q)
            for row in valid
        ),
    )


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _rotation_delta_deg(left: Quaternion, right: Quaternion) -> float:
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return math.degrees(2.0 * math.acos(min(1.0, max(0.0, dot))))


def correction_summary(rows: tuple[StateRow, ...]) -> CorrectionSummary:
    valid = tuple(
        row
        for row in rows
        if row.prediction_valid and row.fusion_mode in FUSED_MODES
    )
    if not valid:
        raise ProbeError("no valid prediction rows")
    translation = [
        _norm(_subtract(row.post_p, row.pred_p)) for row in valid
    ]
    rotation = [
        _rotation_delta_deg(row.post_q, row.pred_q) for row in valid
    ]
    gyro_bias = [_norm(_subtract(row.post_bg, row.pred_bg)) for row in valid]
    return CorrectionSummary(
        poses=len(valid),
        pose_translation_rms_m=_rms(translation),
        pose_translation_max_m=max(translation),
        pose_rotation_rms_deg=_rms(rotation),
        pose_rotation_max_deg=max(rotation),
        gyro_bias_rms_rps=_rms(gyro_bias),
        gyro_bias_max_rps=max(gyro_bias),
    )


def _write_tum(path: Path, poses: tuple[TumPose, ...]) -> None:
    try:
        path.write_text(
            "".join(f"{pose.line}\n" for pose in poses), encoding="utf-8"
        )
    except OSError as exc:
        raise ProbeError(f"cannot write temporary TUM file: {exc}") from exc


def _evaluate_pair(
    rows: tuple[StateRow, ...], euroc_root: Path, evaluator: Path
) -> MetricComparison:
    prediction, graph_initial, posterior = tum_poses(rows)
    if len(prediction) < 22:
        raise ProbeError("trajectory evaluation needs at least 22 poses")
    with tempfile.TemporaryDirectory(prefix="phad-vio-state-") as directory:
        root = Path(directory)
        prediction_path = root / "prediction.tum"
        graph_initial_path = root / "graph_initial.tum"
        posterior_path = root / "posterior.tum"
        _write_tum(prediction_path, prediction)
        _write_tum(graph_initial_path, graph_initial)
        _write_tum(posterior_path, posterior)
        try:
            prediction_metrics = evaluate(
                evaluator, prediction_path, euroc_root
            )
            graph_initial_metrics = evaluate(
                evaluator, graph_initial_path, euroc_root
            )
            posterior_metrics = evaluate(evaluator, posterior_path, euroc_root)
        except RuntimeError as exc:
            raise ProbeError(str(exc)) from exc
    if (
        prediction_metrics.poses != len(prediction)
        or graph_initial_metrics.poses != len(graph_initial)
        or posterior_metrics.poses != len(posterior)
    ):
        raise ProbeError("evaluator pose count does not match sidecar support")
    if not (
        prediction_metrics.matched
        == graph_initial_metrics.matched
        == posterior_metrics.matched
    ):
        raise ProbeError("prediction/graph/posterior matched support differs")
    if not (
        prediction_metrics.rpe_pairs
        == graph_initial_metrics.rpe_pairs
        == posterior_metrics.rpe_pairs
    ):
        raise ProbeError("prediction/graph/posterior RPE support differs")
    return MetricComparison(
        prediction_metrics, graph_initial_metrics, posterior_metrics
    )


def analyze(
    rows: tuple[StateRow, ...],
    euroc_root: Path,
    evaluator: Path,
    bucket_poses: int,
) -> AnalysisResult:
    if bucket_poses < 1:
        raise ProbeError("bucket_poses must be positive")
    valid = tuple(
        row
        for row in rows
        if row.prediction_valid and row.fusion_mode in FUSED_MODES
    )
    evaluable, outside = _evaluation_support(
        valid, _load_groundtruth_timestamps(euroc_root)
    )
    if len(evaluable) < 22:
        raise ProbeError("evaluable prediction support needs at least 22 poses")

    full = _evaluate_pair(evaluable, euroc_root, evaluator)
    buckets: list[BucketResult] = []
    for start in range(0, len(evaluable), bucket_poses):
        chunk = evaluable[start : start + bucket_poses]
        prior_advances = sum(
            1
            for index in range(start, start + len(chunk))
            if index > 0
            and evaluable[index].prior_key != evaluable[index - 1].prior_key
        )
        metrics = (
            _evaluate_pair(chunk, euroc_root, evaluator)
            if len(chunk) >= 22
            else None
        )
        buckets.append(
            BucketResult(
                bucket=len(buckets),
                start_index=start,
                end_index=start + len(chunk),
                start_ns=chunk[0].timestamp_ns,
                end_ns=chunk[-1].timestamp_ns,
                poses=len(chunk),
                keyframes=sum(row.is_keyframe for row in chunk),
                prior_advances=prior_advances,
                correction=correction_summary(chunk),
                metrics=metrics,
            )
        )
    return AnalysisResult(
        accepted_rows=len(rows),
        prediction_valid_rows=len(valid),
        prediction_invalid_rows=len(rows) - len(valid),
        evaluation_rows=len(evaluable),
        outside_groundtruth_rows=outside,
        full=full,
        correction=correction_summary(evaluable),
        buckets=tuple(buckets),
    )


def _format(value: object) -> object:
    return f"{value:.17g}" if isinstance(value, float) else value


def render_bucket_csv(
    buckets: tuple[BucketResult, ...], output: TextIO
) -> None:
    writer = csv.DictWriter(output, fieldnames=BUCKET_FIELDS, lineterminator="\n")
    writer.writeheader()
    for bucket in buckets:
        metrics = bucket.metrics
        row: dict[str, object] = {
            "bucket": bucket.bucket,
            "start_index": bucket.start_index,
            "end_index": bucket.end_index,
            "start_ns": bucket.start_ns,
            "end_ns": bucket.end_ns,
            "poses": bucket.poses,
            "keyframes": bucket.keyframes,
            "prior_advances": bucket.prior_advances,
            "pred_ate_rmse_m": "" if metrics is None else metrics.prediction.ate_rmse_m,
            "graph_init_ate_rmse_m": "" if metrics is None else metrics.graph_initial.ate_rmse_m,
            "post_ate_rmse_m": "" if metrics is None else metrics.posterior.ate_rmse_m,
            "ate_graph_init_minus_pred_m": "" if metrics is None else metrics.ate_graph_init_minus_pred_m,
            "ate_post_minus_graph_init_m": "" if metrics is None else metrics.ate_post_minus_graph_init_m,
            "ate_post_minus_pred_m": "" if metrics is None else metrics.ate_post_minus_pred_m,
            "pred_rpe_rmse_m": "" if metrics is None else metrics.prediction.rpe_rmse_m,
            "graph_init_rpe_rmse_m": "" if metrics is None else metrics.graph_initial.rpe_rmse_m,
            "post_rpe_rmse_m": "" if metrics is None else metrics.posterior.rpe_rmse_m,
            "rpe_graph_init_minus_pred_m": "" if metrics is None else metrics.rpe_graph_init_minus_pred_m,
            "rpe_post_minus_graph_init_m": "" if metrics is None else metrics.rpe_post_minus_graph_init_m,
            "rpe_post_minus_pred_m": "" if metrics is None else metrics.rpe_post_minus_pred_m,
            "matched": "" if metrics is None else metrics.prediction.matched,
            "rpe_pairs": "" if metrics is None else metrics.prediction.rpe_pairs,
            "pose_translation_rms_m": bucket.correction.pose_translation_rms_m,
            "pose_translation_max_m": bucket.correction.pose_translation_max_m,
            "pose_rotation_rms_deg": bucket.correction.pose_rotation_rms_deg,
            "pose_rotation_max_deg": bucket.correction.pose_rotation_max_deg,
            "gyro_bias_rms_rps": bucket.correction.gyro_bias_rms_rps,
            "gyro_bias_max_rps": bucket.correction.gyro_bias_max_rps,
        }
        writer.writerow({key: _format(value) for key, value in row.items()})


def _print_result(result: AnalysisResult) -> None:
    full = result.full
    correction = result.correction
    print(f"accepted_rows={result.accepted_rows}")
    print(f"prediction_valid_rows={result.prediction_valid_rows}")
    print(f"prediction_invalid_rows={result.prediction_invalid_rows}")
    print(f"evaluation_rows={result.evaluation_rows}")
    print(f"outside_groundtruth_rows={result.outside_groundtruth_rows}")
    print(f"matched={full.prediction.matched}")
    print(f"rpe_pairs={full.prediction.rpe_pairs}")
    print(f"pred_ate_rmse_m={full.prediction.ate_rmse_m:.17g}")
    print(f"graph_init_ate_rmse_m={full.graph_initial.ate_rmse_m:.17g}")
    print(f"post_ate_rmse_m={full.posterior.ate_rmse_m:.17g}")
    print(
        "ate_graph_init_minus_pred_m="
        f"{full.ate_graph_init_minus_pred_m:.17g}"
    )
    print(
        "ate_post_minus_graph_init_m="
        f"{full.ate_post_minus_graph_init_m:.17g}"
    )
    print(f"ate_post_minus_pred_m={full.ate_post_minus_pred_m:.17g}")
    print(f"pred_rpe_rmse_m={full.prediction.rpe_rmse_m:.17g}")
    print(f"graph_init_rpe_rmse_m={full.graph_initial.rpe_rmse_m:.17g}")
    print(f"post_rpe_rmse_m={full.posterior.rpe_rmse_m:.17g}")
    print(
        "rpe_graph_init_minus_pred_m="
        f"{full.rpe_graph_init_minus_pred_m:.17g}"
    )
    print(
        "rpe_post_minus_graph_init_m="
        f"{full.rpe_post_minus_graph_init_m:.17g}"
    )
    print(f"rpe_post_minus_pred_m={full.rpe_post_minus_pred_m:.17g}")
    print(
        "correction_rms="
        f"pose_m:{correction.pose_translation_rms_m:.17g},"
        f"rot_deg:{correction.pose_rotation_rms_deg:.17g},"
        f"bg_rps:{correction.gyro_bias_rms_rps:.17g}"
    )
    for bucket in result.buckets:
        if bucket.metrics is None:
            metric_text = "metrics=insufficient_poses"
        else:
            metric_text = (
                f"ate_delta={bucket.metrics.ate_post_minus_pred_m:.9g},"
                f"rpe_delta={bucket.metrics.rpe_post_minus_pred_m:.9g}"
            )
        print(
            f"bucket={bucket.bucket} support={bucket.start_index}:"
            f"{bucket.end_index} poses={bucket.poses} {metric_text}"
        )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sidecar", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument(
        "--traj-eval", type=Path, default=Path("build/phad_traj_eval")
    )
    parser.add_argument("--bucket-poses", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.bucket_poses < 1:
        parser.error("--bucket-poses must be positive")

    try:
        rows = load_rows(args.sidecar)
        result = analyze(
            rows, args.euroc_root, args.traj_eval, args.bucket_poses
        )
        if args.output is not None:
            with args.output.open("w", encoding="utf-8", newline="") as output:
                render_bucket_csv(result.buckets, output)
    except (OSError, ProbeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    _print_result(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

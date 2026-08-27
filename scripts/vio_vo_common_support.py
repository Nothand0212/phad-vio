#!/usr/bin/env python3
"""Compare VIO and VO on their exact common trajectory timestamps."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Optional


class ComparisonError(RuntimeError):
    """Input artifacts or evaluator output violate the probe contract."""


@dataclass(frozen=True)
class TumPose:
    timestamp_ns: int
    line: str


@dataclass(frozen=True)
class Metrics:
    poses: int
    matched: int
    ate_rmse_m: float
    rpe_pairs: int
    rpe_rmse_m: float


_POSES = re.compile(r"^estimate poses (\d+),", re.MULTILINE)
_MATCHED = re.compile(r"^matched (\d+) of \d+", re.MULTILINE)
_ATE = re.compile(
    r"^ATE translation \[m\]\s+rmse ([^\s]+)", re.MULTILINE
)
_RPE_PAIRS = re.compile(r"^RPE over [^\n]+ from (\d+) pose pairs", re.MULTILINE)
_RPE = re.compile(
    r"^RPE translation \[m\]\s+rmse ([^\s]+)", re.MULTILINE
)


def _timestamp_ns(text: str, path: Path, line_number: int) -> int:
    try:
        nanoseconds = Decimal(text) * Decimal(1_000_000_000)
    except InvalidOperation as exc:
        raise ComparisonError(
            f"{path}:{line_number}: invalid timestamp"
        ) from exc
    if nanoseconds != nanoseconds.to_integral_value():
        raise ComparisonError(
            f"{path}:{line_number}: timestamp is not integral nanoseconds"
        )
    return int(nanoseconds)


def load_tum(path: Path) -> tuple[TumPose, ...]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ComparisonError(f"cannot read {path}: {exc}") from exc

    poses: list[TumPose] = []
    previous: Optional[int] = None
    for line_number, line in enumerate(lines, start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ComparisonError(
                f"{path}:{line_number}: expected 8 TUM columns"
            )
        timestamp_ns = _timestamp_ns(fields[0], path, line_number)
        if previous is not None and timestamp_ns <= previous:
            raise ComparisonError(
                f"{path}:{line_number}: timestamps are not strictly increasing"
            )
        poses.append(TumPose(timestamp_ns, stripped))
        previous = timestamp_ns
    if not poses:
        raise ComparisonError(f"{path}: no poses")
    return tuple(poses)


def common_support(
    vio: tuple[TumPose, ...], vo: tuple[TumPose, ...]
) -> tuple[tuple[TumPose, ...], tuple[TumPose, ...]]:
    vio_by_time = {pose.timestamp_ns: pose for pose in vio}
    vo_by_time = {pose.timestamp_ns: pose for pose in vo}
    timestamps = sorted(vio_by_time.keys() & vo_by_time.keys())
    if len(timestamps) < 2:
        raise ComparisonError("VIO and VO have fewer than two common poses")
    return (
        tuple(vio_by_time[timestamp] for timestamp in timestamps),
        tuple(vo_by_time[timestamp] for timestamp in timestamps),
    )


def _write_tum(path: Path, poses: tuple[TumPose, ...]) -> None:
    path.write_text(
        "".join(f"{pose.line}\n" for pose in poses), encoding="utf-8"
    )


def _capture(pattern: re.Pattern[str], output: str, name: str) -> str:
    match = pattern.search(output)
    if match is None:
        raise ComparisonError(f"evaluator output is missing {name}")
    return match.group(1)


def parse_metrics(output: str) -> Metrics:
    try:
        return Metrics(
            poses=int(_capture(_POSES, output, "pose count")),
            matched=int(_capture(_MATCHED, output, "matched count")),
            ate_rmse_m=float(_capture(_ATE, output, "ATE RMSE")),
            rpe_pairs=int(_capture(_RPE_PAIRS, output, "RPE pair count")),
            rpe_rmse_m=float(_capture(_RPE, output, "RPE RMSE")),
        )
    except ValueError as exc:
        raise ComparisonError("evaluator emitted an invalid number") from exc


def evaluate(evaluator: Path, estimate: Path, euroc_root: Path) -> Metrics:
    command = [
        str(evaluator),
        "--est",
        str(estimate),
        "--gt-euroc",
        str(euroc_root),
    ]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        raise ComparisonError(f"cannot run {evaluator}: {exc}") from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise ComparisonError(
            f"evaluator failed with exit {completed.returncode}: {detail}"
        )
    return parse_metrics(completed.stdout)


def compare(
    vio_path: Path,
    vo_path: Path,
    euroc_root: Path,
    evaluator: Path,
    start_common_index: int = 0,
    max_common_poses: Optional[int] = None,
) -> tuple[Metrics, Metrics, int, int, int]:
    vio = load_tum(vio_path)
    vo = load_tum(vo_path)
    common_vio, common_vo = common_support(vio, vo)
    if start_common_index < 0 or start_common_index >= len(common_vio):
        raise ComparisonError("start-common-index is outside common support")
    stop = (
        None
        if max_common_poses is None
        else start_common_index + max_common_poses
    )
    common_vio = common_vio[start_common_index:stop]
    common_vo = common_vo[start_common_index:stop]
    if len(common_vio) < 22:
        raise ComparisonError(
            "selected common support needs at least 22 poses for 1 s RPE"
        )
    with tempfile.TemporaryDirectory(prefix="phad-common-support-") as directory:
        temp_root = Path(directory)
        vio_common_path = temp_root / "vio.tum"
        vo_common_path = temp_root / "vo.tum"
        _write_tum(vio_common_path, common_vio)
        _write_tum(vo_common_path, common_vo)
        vio_metrics = evaluate(evaluator, vio_common_path, euroc_root)
        vo_metrics = evaluate(evaluator, vo_common_path, euroc_root)
    return (
        vio_metrics,
        vo_metrics,
        len(common_vio),
        common_vio[0].timestamp_ns,
        common_vio[-1].timestamp_ns,
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vio_run", type=Path)
    parser.add_argument("vo_run", type=Path)
    parser.add_argument("euroc_root", type=Path)
    parser.add_argument(
        "--traj-eval", type=Path, default=Path("build/phad_traj_eval")
    )
    parser.add_argument(
        "--assert-vio-better",
        action="store_true",
        help="return 1 unless both VIO translation RMSEs are strictly lower",
    )
    parser.add_argument("--start-common-index", type=int, default=0)
    parser.add_argument("--max-common-poses", type=int)
    args = parser.parse_args(argv)

    if args.start_common_index < 0:
        parser.error("--start-common-index must be non-negative")
    if args.max_common_poses is not None and args.max_common_poses < 22:
        parser.error("--max-common-poses must be at least 22")

    try:
        vio, vo, count, start_ns, end_ns = compare(
            args.vio_run / "est.tum",
            args.vo_run / "est.tum",
            args.euroc_root,
            args.traj_eval,
            args.start_common_index,
            args.max_common_poses,
        )
    except ComparisonError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    ate_delta = vio.ate_rmse_m - vo.ate_rmse_m
    rpe_delta = vio.rpe_rmse_m - vo.rpe_rmse_m
    passed = ate_delta < 0.0 and rpe_delta < 0.0
    print(f"common_poses={count}")
    print(f"support_ns={start_ns}:{end_ns}")
    print(f"matched={vio.matched}/{vo.matched}")
    print(f"rpe_pairs={vio.rpe_pairs}/{vo.rpe_pairs}")
    print(f"vio_ate_rmse_m={vio.ate_rmse_m:.17g}")
    print(f"vo_ate_rmse_m={vo.ate_rmse_m:.17g}")
    print(f"ate_delta_m={ate_delta:.17g}")
    print(f"vio_rpe_rmse_m={vio.rpe_rmse_m:.17g}")
    print(f"vo_rpe_rmse_m={vo.rpe_rmse_m:.17g}")
    print(f"rpe_delta_m={rpe_delta:.17g}")
    print(f"verdict={'PASS' if passed else 'FAIL'}")
    if args.assert_vio_better and not passed:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

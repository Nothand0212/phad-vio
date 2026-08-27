#!/usr/bin/env python3
"""Compare accepted-keyframe schedules from phad_vo_bench run directories.

Only ``status=ok && is_keyframe=1`` rows are accepted keyframes.  Runs must
contain the same strictly increasing image timestamp axis; otherwise the
comparison fails instead of silently matching unrelated frames.

The script only depends on the Python standard library.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, TextIO


class ProbeError(RuntimeError):
    """An input run violates the schedule-comparison contract."""


@dataclass(frozen=True)
class RunIdentity:
    sequence: str
    commit: str
    dirty: bool
    config_label: str
    config_hash: str
    ate_trans_rmse: Optional[float]
    rpe_trans_rmse: Optional[float]

    @property
    def label(self) -> str:
        return f"{self.config_label}_{self.config_hash}"


@dataclass(frozen=True)
class RunTrace:
    path: Path
    identity: RunIdentity
    timestamps_ns: tuple[int, ...]
    accepted_frames: tuple[int, ...]
    accepted_rules: tuple[str, ...]


@dataclass(frozen=True)
class MatchReport:
    lag_frames: int
    pairs: tuple[tuple[int, int], ...]
    total_lag_frames: int

    @property
    def matched(self) -> int:
        return len(self.pairs)


def _nested_float(data: dict[str, Any], *keys: str) -> Optional[float]:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _load_identity(run_dir: Path) -> RunIdentity:
    summary_path = run_dir / "summary.json"
    try:
        data = json.loads(summary_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ProbeError(f"cannot read {summary_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ProbeError(f"invalid JSON in {summary_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProbeError(f"summary root is not an object: {summary_path}")

    code = data.get("code") if isinstance(data.get("code"), dict) else {}
    return RunIdentity(
        sequence=str(data.get("sequence") or ""),
        commit=str(code.get("git_commit_short") or "unknown"),
        dirty=bool(code.get("git_dirty", False)),
        config_label=str(data.get("config_label") or "default"),
        config_hash=str(data.get("config_hash") or ""),
        ate_trans_rmse=_nested_float(data, "ate", "trans", "rmse"),
        rpe_trans_rmse=_nested_float(data, "rpe", "trans", "rmse"),
    )


def _parse_binary(value: str, field: str, line: int, path: Path) -> bool:
    if value == "0":
        return False
    if value == "1":
        return True
    raise ProbeError(
        f"{path}:{line}: {field} must be 0 or 1, got {value!r}"
    )


def load_trace(run_dir: Path) -> RunTrace:
    if not run_dir.is_dir():
        raise ProbeError(f"run directory does not exist: {run_dir}")

    identity = _load_identity(run_dir)
    diag_path = run_dir / "diag.csv"
    timestamps: list[int] = []
    accepted_frames: list[int] = []
    accepted_rules: list[str] = []

    try:
        diag_file = diag_path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise ProbeError(f"cannot read {diag_path}: {exc}") from exc

    with diag_file:
        reader = csv.DictReader(diag_file)
        required = {"timestamp_ns", "status", "is_keyframe"}
        fields = set(reader.fieldnames or ())
        missing = sorted(required - fields)
        if missing:
            raise ProbeError(
                f"{diag_path}: missing required columns: {', '.join(missing)}"
            )

        previous_timestamp: Optional[int] = None
        for frame, row in enumerate(reader):
            line = frame + 2
            try:
                timestamp = int(row["timestamp_ns"])
            except (TypeError, ValueError) as exc:
                raise ProbeError(
                    f"{diag_path}:{line}: invalid timestamp_ns"
                ) from exc
            if previous_timestamp is not None and timestamp <= previous_timestamp:
                raise ProbeError(
                    f"{diag_path}:{line}: timestamps are not strictly increasing"
                )
            previous_timestamp = timestamp
            timestamps.append(timestamp)

            is_keyframe = _parse_binary(
                row["is_keyframe"], "is_keyframe", line, diag_path
            )
            if row["status"] == "ok" and is_keyframe:
                accepted_frames.append(frame)
                accepted_rules.append(row.get("kf_rule") or "unavailable")

    if not timestamps:
        raise ProbeError(f"{diag_path}: no frame rows")

    return RunTrace(
        path=run_dir,
        identity=identity,
        timestamps_ns=tuple(timestamps),
        accepted_frames=tuple(accepted_frames),
        accepted_rules=tuple(accepted_rules),
    )


def _score_better(
    candidate_matches: int,
    candidate_lag: int,
    best_matches: int,
    best_lag: int,
) -> bool:
    return candidate_matches > best_matches or (
        candidate_matches == best_matches and candidate_lag < best_lag
    )


def match_schedules(
    reference_frames: tuple[int, ...],
    candidate_frames: tuple[int, ...],
    lag_frames: int,
) -> MatchReport:
    """Maximize one-to-one matches, then minimize total absolute frame lag."""

    if lag_frames < 0:
        raise ValueError("lag_frames must be non-negative")

    n = len(reference_frames)
    m = len(candidate_frames)
    matches = [[0] * (m + 1) for _ in range(n + 1)]
    lag_sum = [[0] * (m + 1) for _ in range(n + 1)]
    choice = [bytearray(m + 1) for _ in range(n + 1)]

    # choice: 1=skip reference, 2=skip candidate, 3=match.
    for i in range(1, n + 1):
        choice[i][0] = 1
    for j in range(1, m + 1):
        choice[0][j] = 2

    for i in range(1, n + 1):
        ref_frame = reference_frames[i - 1]
        for j in range(1, m + 1):
            best_matches = matches[i - 1][j]
            best_lag = lag_sum[i - 1][j]
            best_choice = 1

            cand_matches = matches[i][j - 1]
            cand_lag = lag_sum[i][j - 1]
            if _score_better(
                cand_matches, cand_lag, best_matches, best_lag
            ):
                best_matches = cand_matches
                best_lag = cand_lag
                best_choice = 2

            frame_lag = abs(ref_frame - candidate_frames[j - 1])
            if frame_lag <= lag_frames:
                cand_matches = matches[i - 1][j - 1] + 1
                cand_lag = lag_sum[i - 1][j - 1] + frame_lag
                if _score_better(
                    cand_matches, cand_lag, best_matches, best_lag
                ) or (
                    cand_matches == best_matches and cand_lag == best_lag
                ):
                    best_matches = cand_matches
                    best_lag = cand_lag
                    best_choice = 3

            matches[i][j] = best_matches
            lag_sum[i][j] = best_lag
            choice[i][j] = best_choice

    pairs: list[tuple[int, int]] = []
    i = n
    j = m
    while i > 0 or j > 0:
        selected = choice[i][j]
        if selected == 3:
            pairs.append((i - 1, j - 1))
            i -= 1
            j -= 1
        elif selected == 1:
            i -= 1
        elif selected == 2:
            j -= 1
        else:
            raise AssertionError("invalid schedule matching backtrack")
    pairs.reverse()

    return MatchReport(
        lag_frames=lag_frames,
        pairs=tuple(pairs),
        total_lag_frames=lag_sum[n][m],
    )


def _event(trace: RunTrace, event_index: int) -> Optional[dict[str, Any]]:
    if event_index >= len(trace.accepted_frames):
        return None
    frame = trace.accepted_frames[event_index]
    return {
        "event_ordinal": event_index + 1,
        "frame": frame,
        "timestamp_ns": trace.timestamps_ns[frame],
    }


def _first_order_mismatch(
    reference: RunTrace, candidate: RunTrace
) -> Optional[dict[str, Any]]:
    common = min(len(reference.accepted_frames), len(candidate.accepted_frames))
    mismatch = common
    for index in range(common):
        if reference.accepted_frames[index] != candidate.accepted_frames[index]:
            mismatch = index
            break
    else:
        if len(reference.accepted_frames) == len(candidate.accepted_frames):
            return None
    return {
        "event_ordinal": mismatch + 1,
        "reference": _event(reference, mismatch),
        "candidate": _event(candidate, mismatch),
    }


def _first_set_departure(
    reference: RunTrace, candidate: RunTrace
) -> Optional[dict[str, Any]]:
    reference_set = set(reference.accepted_frames)
    candidate_set = set(candidate.accepted_frames)
    differing = reference_set.symmetric_difference(candidate_set)
    if not differing:
        return None
    frame = min(differing)
    return {
        "frame": frame,
        "timestamp_ns": reference.timestamps_ns[frame],
        "reference_accepted": frame in reference_set,
        "candidate_accepted": frame in candidate_set,
    }


def _interval_report(trace: RunTrace) -> dict[str, Any]:
    gaps = [
        current - previous
        for previous, current in zip(
            trace.accepted_frames, trace.accepted_frames[1:]
        )
    ]
    histogram = Counter(gaps)
    return {
        "count": len(gaps),
        "mean_frames": statistics.fmean(gaps) if gaps else None,
        "median_frames": statistics.median(gaps) if gaps else None,
        "min_frames": min(gaps) if gaps else None,
        "max_frames": max(gaps) if gaps else None,
        "le_6_frames": sum(gap <= 6 for gap in gaps),
        "histogram": {
            str(gap): histogram[gap] for gap in sorted(histogram)
        },
    }


def _rolling_count_delta(
    reference: RunTrace, candidate: RunTrace, window_frames: int
) -> dict[str, Any]:
    if window_frames <= 0:
        raise ValueError("window_frames must be positive")
    frame_count = len(reference.timestamps_ns)
    ref_prefix = [0] * (frame_count + 1)
    cand_prefix = [0] * (frame_count + 1)
    ref_set = set(reference.accepted_frames)
    cand_set = set(candidate.accepted_frames)
    for frame in range(frame_count):
        ref_prefix[frame + 1] = ref_prefix[frame] + (frame in ref_set)
        cand_prefix[frame + 1] = cand_prefix[frame] + (frame in cand_set)

    best_delta = -1
    best_start = 0
    best_signed = 0
    last_start = max(frame_count - window_frames, 0)
    for start in range(last_start + 1):
        end = min(start + window_frames, frame_count)
        ref_count = ref_prefix[end] - ref_prefix[start]
        cand_count = cand_prefix[end] - cand_prefix[start]
        signed = cand_count - ref_count
        if abs(signed) > best_delta:
            best_delta = abs(signed)
            best_start = start
            best_signed = signed

    end = min(best_start + window_frames, frame_count)
    return {
        "window_frames": window_frames,
        "max_absolute_delta": best_delta,
        "candidate_minus_reference": best_signed,
        "start_frame": best_start,
        "end_frame_exclusive": end,
        "start_timestamp_ns": reference.timestamps_ns[best_start],
        "end_timestamp_ns": reference.timestamps_ns[end - 1],
    }


def _rule_breakdown(
    reference: RunTrace, candidate: RunTrace
) -> list[dict[str, Any]]:
    reference_set = set(reference.accepted_frames)
    totals: Counter[str] = Counter()
    common: Counter[str] = Counter()
    candidate_only: Counter[str] = Counter()
    for frame, rule in zip(
        candidate.accepted_frames, candidate.accepted_rules
    ):
        totals[rule] += 1
        if frame in reference_set:
            common[rule] += 1
        else:
            candidate_only[rule] += 1
    return [
        {
            "rule": rule,
            "total": totals[rule],
            "exact_common": common[rule],
            "candidate_only": candidate_only[rule],
        }
        for rule in sorted(totals)
    ]


def _identity_json(trace: RunTrace) -> dict[str, Any]:
    identity = trace.identity
    return {
        "path": str(trace.path),
        "sequence": identity.sequence,
        "commit": identity.commit,
        "dirty": identity.dirty,
        "config_label": identity.config_label,
        "config_hash": identity.config_hash,
        "ate_trans_rmse": identity.ate_trans_rmse,
        "rpe_trans_rmse": identity.rpe_trans_rmse,
    }


def compare_traces(
    reference: RunTrace,
    candidate: RunTrace,
    max_lag_frames: int,
    rolling_window_frames: int,
) -> dict[str, Any]:
    if reference.identity.sequence != candidate.identity.sequence:
        raise ProbeError(
            "sequence mismatch: "
            f"{reference.identity.sequence!r} != {candidate.identity.sequence!r}"
        )
    if reference.timestamps_ns != candidate.timestamps_ns:
        common = min(len(reference.timestamps_ns), len(candidate.timestamps_ns))
        mismatch = common
        for index in range(common):
            if reference.timestamps_ns[index] != candidate.timestamps_ns[index]:
                mismatch = index
                break
        raise ProbeError(
            "image timestamp axis mismatch at frame "
            f"{mismatch}: {reference.path} vs {candidate.path}"
        )

    matches = [
        match_schedules(
            reference.accepted_frames,
            candidate.accepted_frames,
            lag_frames,
        )
        for lag_frames in range(max_lag_frames + 1)
    ]
    reference_count = len(reference.accepted_frames)
    candidate_count = len(candidate.accepted_frames)

    match_json = []
    for report in matches:
        matched = report.matched
        match_json.append(
            {
                "lag_frames": report.lag_frames,
                "matched": matched,
                "reference_match_rate": (
                    matched / reference_count if reference_count else 1.0
                ),
                "candidate_match_rate": (
                    matched / candidate_count if candidate_count else 1.0
                ),
                "mean_matched_lag_frames": (
                    report.total_lag_frames / matched if matched else None
                ),
            }
        )

    return {
        "reference": _identity_json(reference),
        "candidate": _identity_json(candidate),
        "frame_count": len(reference.timestamps_ns),
        "accepted": {
            "reference": reference_count,
            "candidate": candidate_count,
            "delta": candidate_count - reference_count,
        },
        "first_order_mismatch": _first_order_mismatch(reference, candidate),
        "first_set_departure": _first_set_departure(reference, candidate),
        "matches": match_json,
        "reference_intervals": _interval_report(reference),
        "candidate_intervals": _interval_report(candidate),
        "rolling_count_delta": _rolling_count_delta(
            reference, candidate, rolling_window_frames
        ),
        "candidate_rule_breakdown": _rule_breakdown(reference, candidate),
    }


def _fmt_float(value: Optional[float], digits: int = 6) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def _fmt_rate(value: float) -> str:
    return f"{100.0 * value:.1f}%"


def write_markdown(
    reference: RunTrace,
    reports: list[dict[str, Any]],
    out: TextIO,
) -> None:
    print("# Accepted-KF schedule probe", file=out)
    print(file=out)
    print(
        f"Reference: `{reference.identity.label}` / `{reference.path}`",
        file=out,
    )
    if reports:
        reference_intervals = reports[0]["reference_intervals"]
        print(
            "Reference accepted/ATE/RPE/short gaps ≤6: "
            f"`{len(reference.accepted_frames)}` / "
            f"`{_fmt_float(reference.identity.ate_trans_rmse, 9)}` / "
            f"`{_fmt_float(reference.identity.rpe_trans_rmse, 9)}` m / "
            f"`{reference_intervals['le_6_frames']}`.",
            file=out,
        )
    print(file=out)
    print(
        "| candidate | accepted | Δ | exact ref/cand | ±1 ref/cand | "
        "±2 ref/cand | first mismatch | short gaps ≤6 | rolling max Δ |",
        file=out,
    )
    print(
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        file=out,
    )
    for report in reports:
        candidate = report["candidate"]
        accepted = report["accepted"]
        matches = {item["lag_frames"]: item for item in report["matches"]}
        first = report["first_order_mismatch"]
        first_text = "none" if first is None else str(first["event_ordinal"])

        match_cells: list[str] = []
        for lag in (0, 1, 2):
            item = matches.get(lag)
            if item is None:
                match_cells.append("n/a")
            else:
                match_cells.append(
                    f"{_fmt_rate(item['reference_match_rate'])}/"
                    f"{_fmt_rate(item['candidate_match_rate'])}"
                )

        print(
            "| "
            + " | ".join(
                [
                    f"`{candidate['config_label']}_{candidate['config_hash']}`",
                    str(accepted["candidate"]),
                    f"{accepted['delta']:+d}",
                    *match_cells,
                    first_text,
                    str(report["candidate_intervals"]["le_6_frames"]),
                    str(report["rolling_count_delta"]["max_absolute_delta"]),
                ]
            )
            + " |",
            file=out,
        )

    for report in reports:
        candidate = report["candidate"]
        print(file=out)
        print(
            f"## {candidate['config_label']}_{candidate['config_hash']}",
            file=out,
        )
        print(file=out)
        print(
            f"ATE/RPE: `{_fmt_float(candidate['ate_trans_rmse'], 9)}` / "
            f"`{_fmt_float(candidate['rpe_trans_rmse'], 9)}` m.",
            file=out,
        )

        mismatch = report["first_order_mismatch"]
        departure = report["first_set_departure"]
        print(file=out)
        print("- First order mismatch: " + json.dumps(mismatch), file=out)
        print("- First set departure: " + json.dumps(departure), file=out)
        rolling = report["rolling_count_delta"]
        print(
            "- Rolling window: "
            f"{rolling['window_frames']} frames, max |Δ|="
            f"{rolling['max_absolute_delta']} "
            f"(candidate-reference={rolling['candidate_minus_reference']}, "
            f"start frame={rolling['start_frame']}).",
            file=out,
        )

        print(file=out)
        print("| max lag (frames) | matched | reference | candidate | mean lag |", file=out)
        print("|---:|---:|---:|---:|---:|", file=out)
        for item in report["matches"]:
            mean_lag = item["mean_matched_lag_frames"]
            print(
                f"| {item['lag_frames']} | {item['matched']} | "
                f"{_fmt_rate(item['reference_match_rate'])} | "
                f"{_fmt_rate(item['candidate_match_rate'])} | "
                f"{_fmt_float(mean_lag, 3)} |",
                file=out,
            )

        print(file=out)
        print("| candidate rule | total | exact common | candidate-only |", file=out)
        print("|---|---:|---:|---:|", file=out)
        for item in report["candidate_rule_breakdown"]:
            print(
                f"| {item['rule']} | {item['total']} | "
                f"{item['exact_common']} | {item['candidate_only']} |",
                file=out,
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare accepted-keyframe schedules from phad_vo_bench run dirs."
        )
    )
    parser.add_argument("reference_run", type=Path)
    parser.add_argument("candidate_runs", type=Path, nargs="+")
    parser.add_argument(
        "--max-lag-frames",
        type=int,
        default=2,
        help="largest one-to-one matching tolerance (default: 2)",
    )
    parser.add_argument(
        "--rolling-window-frames",
        type=int,
        default=10,
        help="rolling accepted-count window length (default: 10)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="write machine-readable JSON instead of Markdown",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.max_lag_frames < 0:
        parser.error("--max-lag-frames must be non-negative")
    if args.rolling_window_frames <= 0:
        parser.error("--rolling-window-frames must be positive")

    try:
        reference = load_trace(args.reference_run)
        candidates = [load_trace(path) for path in args.candidate_runs]
        reports = [
            compare_traces(
                reference,
                candidate,
                args.max_lag_frames,
                args.rolling_window_frames,
            )
            for candidate in candidates
        ]
    except ProbeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.json:
        json.dump(
            {"reference": _identity_json(reference), "comparisons": reports},
            sys.stdout,
            ensure_ascii=False,
            indent=2,
        )
        print()
    else:
        write_markdown(reference, reports, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

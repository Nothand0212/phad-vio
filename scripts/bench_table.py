#!/usr/bin/env python3
"""把 bench_root 下多个 summary.json 拼成对比表。

只依赖标准库。默认输出 Markdown；`--csv` 输出 CSV。递归查找名为
`summary.json` 的文件，按 (sequence, commit, config) 排序。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class RunRow:
    sequence: str
    commit: str
    dirty: bool
    config_label: str
    config_hash: str
    status: str
    ate_trans_rmse: Optional[float]
    rpe_trans_rmse: Optional[float]
    completion_rate: Optional[float]
    coverage_rate: Optional[float]
    segments: Optional[int]
    reanchors: Optional[int]
    rtf: Optional[float]
    wall_s: Optional[float]
    path: Path

    @property
    def config_key(self) -> str:
        return f"{self.config_label}_{self.config_hash}"

    @property
    def sort_key(self) -> tuple[str, str, str]:
        return (self.sequence, self.commit, self.config_key)


def _nested_get(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def _as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def load_summary(path: Path) -> Optional[RunRow]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"warning: skip {path}: {exc}", file=sys.stderr)
        return None
    if not isinstance(data, dict):
        print(f"warning: skip {path}: root is not an object", file=sys.stderr)
        return None

    code = data.get("code") if isinstance(data.get("code"), dict) else {}
    traj = data.get("trajectory") if isinstance(data.get("trajectory"), dict) else {}
    timing = data.get("timing") if isinstance(data.get("timing"), dict) else {}
    robustness = (
        data.get("robustness") if isinstance(data.get("robustness"), dict) else {}
    )

    sequence = str(data.get("sequence") or "")
    commit = str(code.get("git_commit_short") or data.get("git_commit_short") or "unknown")
    dirty = bool(code.get("git_dirty", data.get("git_dirty", False)))
    config_label = str(data.get("config_label") or "default")
    config_hash = str(data.get("config_hash") or "")

    return RunRow(
        sequence=sequence,
        commit=commit,
        dirty=dirty,
        config_label=config_label,
        config_hash=config_hash,
        status=str(data.get("status") or ""),
        ate_trans_rmse=_as_float(_nested_get(data, "ate", "trans", "rmse")),
        rpe_trans_rmse=_as_float(_nested_get(data, "rpe", "trans", "rmse")),
        completion_rate=_as_float(traj.get("completion_rate")),
        coverage_rate=_as_float(traj.get("coverage_rate")),
        segments=_as_int(traj.get("segments")),
        reanchors=_as_int(robustness.get("reanchors")),
        rtf=_as_float(timing.get("rtf")),
        wall_s=_as_float(timing.get("wall_s")),
        path=path,
    )


def discover_summaries(root: Path) -> list[RunRow]:
    rows: list[RunRow] = []
    for path in sorted(root.rglob("summary.json")):
        row = load_summary(path)
        if row is not None:
            rows.append(row)
    rows.sort(key=lambda row: row.sort_key)
    return rows


def _fmt(value: Optional[float], digits: int = 6) -> str:
    if value is None:
        return ""
    return f"{value:.{digits}g}"


def _fmt_int(value: Optional[int]) -> str:
    if value is None:
        return ""
    return str(value)


def write_markdown(rows: Iterable[RunRow], out) -> None:
    headers = [
        "sequence",
        "commit",
        "dirty",
        "config",
        "status",
        "ate_trans_rmse",
        "rpe_trans_rmse",
        "completion_rate",
        "coverage_rate",
        "segments",
        "reanchors",
        "rtf",
        "wall_s",
    ]
    print("| " + " | ".join(headers) + " |", file=out)
    print("| " + " | ".join("---" for _ in headers) + " |", file=out)
    for row in rows:
        cells = [
            row.sequence,
            row.commit,
            "yes" if row.dirty else "no",
            row.config_key,
            row.status,
            _fmt(row.ate_trans_rmse),
            _fmt(row.rpe_trans_rmse),
            _fmt(row.completion_rate),
            _fmt(row.coverage_rate),
            _fmt_int(row.segments),
            _fmt_int(row.reanchors),
            _fmt(row.rtf),
            _fmt(row.wall_s),
        ]
        print("| " + " | ".join(cells) + " |", file=out)


def write_csv(rows: Iterable[RunRow], out) -> None:
    writer = csv.writer(out)
    writer.writerow(
        [
            "sequence",
            "commit",
            "dirty",
            "config_label",
            "config_hash",
            "status",
            "ate_trans_rmse",
            "rpe_trans_rmse",
            "completion_rate",
            "coverage_rate",
            "segments",
            "reanchors",
            "rtf",
            "wall_s",
            "summary_path",
        ]
    )
    for row in rows:
        writer.writerow(
            [
                row.sequence,
                row.commit,
                "1" if row.dirty else "0",
                row.config_label,
                row.config_hash,
                row.status,
                _fmt(row.ate_trans_rmse),
                _fmt(row.rpe_trans_rmse),
                _fmt(row.completion_rate),
                _fmt(row.coverage_rate),
                _fmt_int(row.segments),
                _fmt_int(row.reanchors),
                _fmt(row.rtf),
                _fmt(row.wall_s),
                str(row.path),
            ]
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect phad_vo_bench summary.json files into a comparison table.",
        epilog=(
            "examples:\n"
            "  python scripts/bench_table.py /path/to/phad-bench\n"
            "  python scripts/bench_table.py $PHAD_BENCH_ROOT --csv\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "bench_root",
        type=Path,
        help="root directory that contains sequence/commit/config run dirs",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="write CSV instead of Markdown",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    root: Path = args.bench_root
    if not root.is_dir():
        print(f"error: bench_root is not a directory: {root}", file=sys.stderr)
        return 2

    rows = discover_summaries(root)
    if not rows:
        print(f"error: no summary.json under {root}", file=sys.stderr)
        return 1

    if args.csv:
        write_csv(rows, sys.stdout)
    else:
        write_markdown(rows, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

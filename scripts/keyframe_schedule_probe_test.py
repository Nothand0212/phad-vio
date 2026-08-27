#!/usr/bin/env python3
"""Unit tests for keyframe_schedule_probe.py."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from keyframe_schedule_probe import (
    ProbeError,
    compare_traces,
    load_trace,
    match_schedules,
)


class KeyframeScheduleProbeTest(unittest.TestCase):
    def make_run(
        self,
        root: Path,
        name: str,
        rows: list[tuple[int, str, int, str]],
        *,
        sequence: str = "MH_01_easy",
    ) -> Path:
        run = root / name
        run.mkdir()
        (run / "summary.json").write_text(
            json.dumps(
                {
                    "sequence": sequence,
                    "config_label": name,
                    "config_hash": "12345678",
                    "code": {
                        "git_commit_short": "abcdef0",
                        "git_dirty": False,
                    },
                    "ate": {"trans": {"rmse": 0.1}},
                    "rpe": {"trans": {"rmse": 0.01}},
                }
            ),
            encoding="utf-8",
        )
        with (run / "diag.csv").open(
            "w", encoding="utf-8", newline=""
        ) as diag:
            writer = csv.writer(diag)
            writer.writerow(
                ["timestamp_ns", "status", "is_keyframe", "kf_rule"]
            )
            writer.writerows(rows)
        return run

    def test_load_trace_counts_only_accepted_keyframes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = self.make_run(
                root,
                "candidate",
                [
                    (10, "rejected", 1, "first"),
                    (20, "ok", 0, "none"),
                    (30, "ok", 1, "parallax"),
                ],
            )
            trace = load_trace(run)

        self.assertEqual(trace.accepted_frames, (2,))
        self.assertEqual(trace.accepted_rules, ("parallax",))

    def test_match_maximizes_count_then_minimizes_lag(self) -> None:
        report = match_schedules((0, 1), (1,), 1)

        self.assertEqual(report.matched, 1)
        self.assertEqual(report.total_lag_frames, 0)
        self.assertEqual(report.pairs, ((1, 0),))

    def test_compare_reports_first_departure_and_rolling_delta(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference_run = self.make_run(
                root,
                "reference",
                [
                    (10, "ok", 1, "unavailable"),
                    (20, "ok", 0, "unavailable"),
                    (30, "ok", 1, "unavailable"),
                    (40, "ok", 0, "unavailable"),
                ],
            )
            candidate_run = self.make_run(
                root,
                "candidate",
                [
                    (10, "ok", 1, "first"),
                    (20, "ok", 1, "parallax"),
                    (30, "ok", 0, "none"),
                    (40, "ok", 1, "parallax"),
                ],
            )
            reference = load_trace(reference_run)
            candidate = load_trace(candidate_run)
            result = compare_traces(reference, candidate, 1, 2)

        self.assertEqual(result["accepted"]["reference"], 2)
        self.assertEqual(result["accepted"]["candidate"], 3)
        self.assertEqual(result["first_order_mismatch"]["event_ordinal"], 2)
        self.assertEqual(result["first_set_departure"]["frame"], 1)
        self.assertEqual(result["matches"][0]["matched"], 1)
        self.assertEqual(
            result["rolling_count_delta"]["max_absolute_delta"], 1
        )

    def test_compare_rejects_different_timestamp_axes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = load_trace(
                self.make_run(root, "reference", [(10, "ok", 1, "first")])
            )
            candidate = load_trace(
                self.make_run(root, "candidate", [(11, "ok", 1, "first")])
            )

            with self.assertRaisesRegex(ProbeError, "timestamp axis mismatch"):
                compare_traces(reference, candidate, 1, 2)


if __name__ == "__main__":
    unittest.main()

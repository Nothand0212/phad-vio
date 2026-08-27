#!/usr/bin/env python3
"""Tests for the M4.4 fixed-lag shadow sidecar analyzer."""

from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from gyro_fixed_lag_shadow_probe import COLUMNS, ProbeError, analyze


def inactive_row(timestamp_ns: int = 0, state_key: int = 0) -> dict[str, object]:
    row: dict[str, object] = {column: 0 for column in COLUMNS}
    row.update(
        {
            "timestamp_ns": timestamp_ns,
            "state_key": state_key,
            "fusion_mode": "vision_only",
            "update_ok": 1,
            "reset_reason": "none",
            "shadow_q_w": 1.0,
        }
    )
    return row


def active_row(
    timestamp_ns: int,
    state_key: int,
    *,
    reset: bool = False,
    reset_reason: str = "none",
    rotation_delta: float = 0.01,
    translation_delta: float = 0.02,
) -> dict[str, object]:
    row = inactive_row(timestamp_ns, state_key)
    row.update(
        {
            "fusion_mode": "gyro_visual",
            "active": 1,
            "reset": int(reset),
            "reset_reason": reset_reason,
            "current_epoch": state_key,
            "cutoff_epoch": max(0, state_key - 6),
            "batch_window_n": 7,
            "smoother_pose_n": 7,
            "smoother_landmark_n": 12,
            "bias_present": 1,
            "bias_fixed": int(state_key >= 8),
            "bias_delta_norm": 0.001 * state_key,
            "user_factor_n": 40,
            "marginalized_pose_n": int(state_key >= 8),
            "retired_landmark_n": 2,
            "new_landmark_generation_n": 3,
            "shadow_p_x": 0.1 * state_key,
            "shadow_q_w": 1.0,
            "newest_rot_delta_rad": rotation_delta,
            "newest_trans_delta_m": translation_delta,
        }
    )
    return row


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


class FixedLagShadowProbeTest(unittest.TestCase):
    def test_summarizes_lifecycle_deltas_and_buckets(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.csv"
            write_rows(
                path,
                [
                    inactive_row(100_000_000, 1),
                    active_row(
                        1_100_000_000,
                        7,
                        reset=True,
                        reset_reason="bootstrap",
                        rotation_delta=0.01,
                        translation_delta=0.02,
                    ),
                    active_row(
                        1_600_000_000,
                        8,
                        rotation_delta=0.03,
                        translation_delta=0.04,
                    ),
                    active_row(
                        2_600_000_000,
                        9,
                        rotation_delta=0.05,
                        translation_delta=0.06,
                    ),
                ],
            )

            result = analyze(path, bucket_s=1.0)

            self.assertEqual(result["row_count"], 4)
            self.assertEqual(result["active_row_count"], 3)
            self.assertEqual(result["inactive_row_count"], 1)
            self.assertEqual(result["reset_counts"], {"bootstrap": 1, "segment": 0})
            self.assertEqual(result["max_smoother_pose_n"], 7)
            self.assertEqual(result["max_smoother_landmark_n"], 12)
            self.assertEqual(result["marginalized_pose_total"], 2)
            self.assertEqual(result["retired_landmark_total"], 6)
            self.assertEqual(result["new_landmark_generation_total"], 9)
            self.assertEqual(result["bias_fixed_row_count"], 2)
            self.assertAlmostEqual(result["rotation_delta_rad"]["p50"], 0.03)
            self.assertAlmostEqual(result["translation_delta_m"]["max"], 0.06)
            self.assertEqual(len(result["buckets"]), 3)
            self.assertTrue(result["lifecycle_gate_pass"])

    def test_rejects_schema_order_lifecycle_and_numeric_violations(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            schema = root / "schema.csv"
            schema.write_text("timestamp_ns,state_key\n1,1\n", encoding="utf-8")
            with self.assertRaises(ProbeError):
                analyze(schema)

            duplicate = root / "duplicate.csv"
            write_rows(
                duplicate,
                [inactive_row(100, 1), inactive_row(100, 2)],
            )
            with self.assertRaises(ProbeError):
                analyze(duplicate)

            orphan = root / "orphan.csv"
            bad_orphan = active_row(
                200, 7, reset=True, reset_reason="bootstrap"
            )
            bad_orphan["timestamp_without_value_n"] = 1
            write_rows(orphan, [bad_orphan])
            with self.assertRaises(ProbeError):
                analyze(orphan)

            pose_count = root / "pose_count.csv"
            bad_pose = active_row(
                200, 7, reset=True, reset_reason="bootstrap"
            )
            bad_pose["smoother_pose_n"] = 6
            write_rows(pose_count, [bad_pose])
            with self.assertRaises(ProbeError):
                analyze(pose_count)

            numeric = root / "numeric.csv"
            bad_numeric = active_row(
                200, 7, reset=True, reset_reason="bootstrap"
            )
            bad_numeric["newest_rot_delta_rad"] = math.nan
            write_rows(numeric, [bad_numeric])
            with self.assertRaises(ProbeError):
                analyze(numeric)

    def test_requires_active_support_and_positive_bucket(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "inactive.csv"
            write_rows(path, [inactive_row(100, 1)])
            with self.assertRaises(ProbeError):
                analyze(path)
            with self.assertRaises(ProbeError):
                analyze(path, bucket_s=0.0)

    def test_allows_landmark_free_segment_reset_frame(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "segment.csv"
            bootstrap = active_row(
                100, 7, reset=True, reset_reason="bootstrap"
            )
            segment = active_row(200, 8, reset=True, reset_reason="segment")
            segment.update(
                {
                    "cutoff_epoch": 8,
                    "batch_window_n": 1,
                    "smoother_pose_n": 1,
                    "smoother_landmark_n": 0,
                    "user_factor_n": 2,
                    "new_landmark_generation_n": 0,
                }
            )
            write_rows(path, [bootstrap, segment])
            result = analyze(path)
            self.assertEqual(result["reset_counts"], {"bootstrap": 1, "segment": 1})
            self.assertEqual(result["max_smoother_landmark_n"], 12)


if __name__ == "__main__":
    unittest.main()

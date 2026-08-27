#!/usr/bin/env python3
"""Unit tests for keyframe_shadow_probe.py."""

from __future__ import annotations

import csv
import io
import tempfile
import unittest
from pathlib import Path

from keyframe_shadow_probe import (
    HEADER,
    ProbeError,
    analyze,
    load_rows,
    render_markdown,
)


class KeyframeShadowProbeTest(unittest.TestCase):
    def base_row(self, frame: int, ts_ns: int) -> dict[str, object]:
        row: dict[str, object] = {field: 0 for field in HEADER}
        row.update(
            {
                "frame": frame,
                "ts_ns": ts_ns,
                "frames_since_kf": -1,
                "prod_selected": 1,
                "prod_rule": "bootstrap",
                "prod_bootstrap": 1,
                "imu_reason": "no_accepted_kf",
                "status": "rejected",
            }
        )
        return row

    def rows(self) -> list[dict[str, object]]:
        rows = [
            self.base_row(0, 100_000_000),
            self.base_row(1, 200_000_000),
            self.base_row(2, 300_000_000),
            self.base_row(3, 400_000_000),
            self.base_row(4, 500_000_000),
        ]
        rows[1].update(
            {
                "status": "ok",
                "epoch_committed": 1,
                "epoch_after": 1,
            }
        )
        rows[2].update(
            {
                "epoch_before": 1,
                "frames_since_kf": 1,
                "status": "ok",
                "epoch_committed": 1,
                "epoch_after": 2,
                "imu_valid": 1,
                "imu_reason": "none",
                "imu_sample_n": 2,
                "imu_t_i_ns": 200_000_000,
                "imu_t_j_ns": 300_000_000,
                "imu_dt_s": 0.1,
                "imu_angle_rad": 0.01,
                "imu_comp_px": 5.0,
                "imu_parallax_n": 10,
            }
        )
        rows[3].update(
            {
                "epoch_before": 2,
                "epoch_after": 2,
                "frames_since_kf": 1,
                "prod_selected": 0,
                "prod_rule": "none",
                "prod_bootstrap": 0,
                "status": "ok",
                "imu_valid": 1,
                "imu_reason": "none",
                "imu_sample_n": 2,
                "imu_t_i_ns": 300_000_000,
                "imu_t_j_ns": 400_000_000,
                "imu_dt_s": 0.1,
                "imu_angle_rad": 0.01,
                "imu_comp_px": 12.0,
                "imu_parallax_n": 10,
                "imu_gt_10": 1,
            }
        )
        rows[4].update(
            {
                "epoch_before": 2,
                "epoch_after": 3,
                "frames_since_kf": 2,
                "prod_selected": 1,
                "prod_rule": "parallax",
                "prod_bootstrap": 0,
                "prod_parallax": 1,
                "pose_comp_px": 31.0,
                "status": "ok",
                "epoch_committed": 1,
                "imu_valid": 1,
                "imu_reason": "none",
                "imu_sample_n": 3,
                "imu_t_i_ns": 300_000_000,
                "imu_t_j_ns": 500_000_000,
                "imu_dt_s": 0.2,
                "imu_angle_rad": 0.02,
                "imu_comp_px": 8.0,
                "imu_parallax_n": 10,
            }
        )
        return rows

    def write(
        self,
        path: Path,
        rows: list[dict[str, object]],
        header: tuple[str, ...] = HEADER,
    ) -> None:
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=header)
            writer.writeheader()
            for row in rows:
                writer.writerow({key: row[key] for key in header})

    def test_load_and_analyze_frozen_epoch_differences(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.csv"
            self.write(path, self.rows())
            rows = load_rows(path)
            result = analyze(rows)
            output = io.StringIO()
            render_markdown(path, result, output)

        self.assertEqual(result["frames"], 5)
        self.assertEqual(result["accepted_epochs"], 3)
        reports = result["thresholds"]
        self.assertIsInstance(reports, list)
        threshold_10 = reports[0]
        self.assertEqual(threshold_10["additions"], [3])
        self.assertEqual(threshold_10["removals"], [4])
        self.assertEqual(threshold_10["first_difference"], 3)
        self.assertEqual(threshold_10["rule4_hits"], 1)
        self.assertEqual(threshold_10["isolated_hits"], 1)
        self.assertEqual(threshold_10["gap_veto_counts"][1], 1)
        markdown = output.getvalue()
        self.assertIn("`no_accepted_kf:2, none:3`", markdown)
        self.assertIn("Rule4 run lengths", markdown)
        self.assertNotIn("\\`", markdown)

    def test_rejects_header_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.csv"
            self.write(path, self.rows(), HEADER[:-1])
            with self.assertRaisesRegex(ProbeError, "header mismatch"):
                load_rows(path)

    def test_rejects_epoch_transaction_mismatch(self) -> None:
        rows = self.rows()
        rows[3]["epoch_before"] = 99
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.csv"
            self.write(path, rows)
            with self.assertRaisesRegex(ProbeError, "epoch_before mismatch"):
                load_rows(path)

    def test_rejects_nonfinite_unreported_field(self) -> None:
        rows = self.rows()
        rows[3]["bias_gyr_z"] = "nan"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "shadow.csv"
            self.write(path, rows)
            with self.assertRaisesRegex(ProbeError, "must be finite"):
                load_rows(path)


if __name__ == "__main__":
    unittest.main()

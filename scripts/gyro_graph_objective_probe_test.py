#!/usr/bin/env python3
"""Tests for the M4.4 gyro graph objective probe."""

from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from gyro_graph_objective_probe import ProbeError, analyze


FIELDS = (
    "timestamp_ns",
    "state_key",
    "fusion_mode",
    "gyro_factor_n",
    "gyro_obj_valid",
    "gyro_obj_factor_n",
    "gyro_obj_interior_n",
    "gyro_obj_stereo_n",
    "gyro_obj_total_init",
    "gyro_obj_total_post",
    "gyro_obj_gyro_init",
    "gyro_obj_gyro_post",
    "gyro_obj_boundary_init",
    "gyro_obj_boundary_post",
    "gyro_obj_boundary_r_init_rad",
    "gyro_obj_boundary_r_post_rad",
    "gyro_obj_boundary_w_init",
    "gyro_obj_boundary_w_post",
    "gyro_obj_interior_init",
    "gyro_obj_interior_post",
    "gyro_obj_new_init",
    "gyro_obj_new_post",
    "gyro_obj_new_r_init_rad",
    "gyro_obj_new_r_post_rad",
    "gyro_obj_new_w_init",
    "gyro_obj_new_w_post",
    "gyro_obj_stereo_init",
    "gyro_obj_stereo_post",
    "gyro_obj_pose_prior_r_post_rad",
    "gyro_obj_pose_prior_t_post_m",
    "gyro_obj_pose_prior_post",
    "gyro_obj_bias_prior_r_post_rps",
    "gyro_obj_bias_prior_post",
)


def objective_row(timestamp_ns: int, state_key: int, *, boundary: float,
                  interior: float, newest: float) -> dict[str, object]:
    gyro = boundary + interior
    return {
        "timestamp_ns": timestamp_ns,
        "state_key": state_key,
        "fusion_mode": "gyro_visual",
        "gyro_factor_n": 3,
        "gyro_obj_valid": 1,
        "gyro_obj_factor_n": 3,
        "gyro_obj_interior_n": 2,
        "gyro_obj_stereo_n": 20,
        "gyro_obj_total_init": 50.0,
        "gyro_obj_total_post": gyro + 12.0,
        "gyro_obj_gyro_init": 20.0,
        "gyro_obj_gyro_post": gyro,
        "gyro_obj_boundary_init": 4.5,
        "gyro_obj_boundary_post": boundary,
        "gyro_obj_boundary_r_init_rad": 0.003,
        "gyro_obj_boundary_r_post_rad": 0.002,
        "gyro_obj_boundary_w_init": 3.0,
        "gyro_obj_boundary_w_post": math.sqrt(2.0 * boundary),
        "gyro_obj_interior_init": 15.5,
        "gyro_obj_interior_post": interior,
        "gyro_obj_new_init": 8.0,
        "gyro_obj_new_post": newest,
        "gyro_obj_new_r_init_rad": 0.004,
        "gyro_obj_new_r_post_rad": 0.001,
        "gyro_obj_new_w_init": 4.0,
        "gyro_obj_new_w_post": math.sqrt(2.0 * newest),
        "gyro_obj_stereo_init": 25.0,
        "gyro_obj_stereo_post": 10.0,
        "gyro_obj_pose_prior_r_post_rad": 1e-6,
        "gyro_obj_pose_prior_t_post_m": 2e-6,
        "gyro_obj_pose_prior_post": 1.0,
        "gyro_obj_bias_prior_r_post_rps": 3e-6,
        "gyro_obj_bias_prior_post": 1.0,
    }


class GyroGraphObjectiveProbeTest(unittest.TestCase):
    def write_rows(self, path: Path, rows: list[dict[str, object]],
                   fields: tuple[str, ...] = FIELDS) -> None:
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fields, extrasaction="ignore"
            )
            writer.writeheader()
            writer.writerows(rows)

    def test_reports_boundary_relative_to_interior_and_newest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            self.write_rows(
                path,
                [
                    objective_row(100, 7, boundary=2.0, interior=8.0,
                                  newest=3.0),
                    objective_row(200, 8, boundary=8.0, interior=4.0,
                                  newest=2.0),
                    objective_row(10_000_000_100, 9, boundary=1.0,
                                  interior=6.0, newest=2.0),
                ],
            )

            result = analyze(path)

        self.assertEqual(result["support"]["gyro_rows"], 3)
        self.assertEqual(result["support"]["comparative_rows"], 3)
        self.assertAlmostEqual(
            result["boundary"]["posterior_cost_to_interior_mean_ratio"]
                  ["median"],
            0.5,
        )
        self.assertAlmostEqual(
            result["boundary"]["greater_than_interior_mean_fraction"],
            1.0 / 3.0,
        )
        self.assertAlmostEqual(
            result["boundary"]["greater_than_newest_fraction"], 1.0 / 3.0
        )
        self.assertEqual(
            [bucket["rows"] for bucket in result["time_buckets"]], [2, 1]
        )
        self.assertAlmostEqual(
            result["time_buckets"][0]["boundary"]
                  ["posterior_cost_to_interior_mean_ratio"]["median"],
            2.25,
        )
        self.assertAlmostEqual(
            result["time_buckets"][1]["boundary"]
                  ["posterior_cost_to_interior_mean_ratio"]["median"],
            1.0 / 3.0,
        )
        self.assertLess(result["identities"]["max_abs_cost_error"], 1e-12)

    def test_rejects_duplicate_or_missing_columns(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            duplicate = root / "duplicate.csv"
            duplicate.write_text("timestamp_ns,timestamp_ns\n1,1\n",
                                 encoding="utf-8")
            with self.assertRaises(ProbeError):
                analyze(duplicate)

            missing = root / "missing.csv"
            self.write_rows(missing, [objective_row(100, 7, boundary=2.0,
                                                    interior=8.0,
                                                    newest=3.0)], FIELDS[:-1])
            with self.assertRaises(ProbeError):
                analyze(missing)

    def test_rejects_nonfinite_and_inconsistent_objective(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            nonfinite = objective_row(100, 7, boundary=2.0, interior=8.0,
                                      newest=3.0)
            nonfinite["gyro_obj_boundary_post"] = "nan"
            nonfinite_path = root / "nonfinite.csv"
            self.write_rows(nonfinite_path, [nonfinite])
            with self.assertRaises(ProbeError):
                analyze(nonfinite_path)

            inconsistent = objective_row(100, 7, boundary=2.0, interior=8.0,
                                         newest=3.0)
            inconsistent["gyro_obj_gyro_post"] = 999.0
            inconsistent_path = root / "inconsistent.csv"
            self.write_rows(inconsistent_path, [inconsistent])
            with self.assertRaises(ProbeError):
                analyze(inconsistent_path)


if __name__ == "__main__":
    unittest.main()

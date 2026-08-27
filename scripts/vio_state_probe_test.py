#!/usr/bin/env python3
"""Unit tests for vio_state_probe.py."""

from __future__ import annotations

import csv
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from vio_state_probe import (
    ProbeError,
    STATE_FIELDS,
    analyze,
    correction_summary,
    load_rows,
    render_bucket_csv,
    tum_poses,
)
from vio_vo_common_support import Metrics


class VioStateProbeTest(unittest.TestCase):
    def make_row(
        self, index: int, *, prediction_valid: bool = True
    ) -> dict[str, str]:
        timestamp_ns = 1_000_000_000 + index * 50_000_000
        row = {field: "0" for field in STATE_FIELDS}
        row.update(
            {
                "timestamp_ns": str(timestamp_ns),
                "state_key": str(10 + index),
                "prior_key": str(2 + index // 2),
                "window_size": "10",
                "is_keyframe": str(index % 2),
                "fusion_mode": "gyro_visual",
                "gyro_factor_n": "9",
                "gyro_alignment_residual_rms_rad": "0.0005",
                "prediction_valid": "1" if prediction_valid else "0",
                "pred_q_w": "1",
                "graph_init_q_w": "1",
                "graph_init_p_x": "2",
                "post_q_w": "1",
                "post_p_x": "3",
                "post_bg_x": "0.3",
                "pred_bg_x": "0.1",
            }
        )
        if prediction_valid:
            row.update(
                {
                    "imu_sample_n": "6",
                    "imu_t_i_ns": str(timestamp_ns - 50_000_000),
                    "imu_t_j_ns": str(timestamp_ns),
                    "imu_dt_s": "0.05",
                }
            )
        return row

    def write_rows(
        self, path: Path, rows: list[dict[str, str]], fields=STATE_FIELDS
    ) -> None:
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(
                output, fieldnames=fields, extrasaction="ignore"
            )
            writer.writeheader()
            writer.writerows(rows)

    def write_groundtruth(self, root: Path, count: int) -> None:
        path = root / "mav0" / "state_groundtruth_estimate0" / "data.csv"
        path.parent.mkdir(parents=True)
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(("#timestamp", "dummy"))
            for index in range(count):
                writer.writerow(
                    (1_000_000_000 + index * 50_000_000, "0")
                )

    def test_loads_strict_schema_and_builds_valid_tum_support(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            self.write_rows(
                path,
                [
                    self.make_row(0),
                    self.make_row(1, prediction_valid=False),
                    self.make_row(2),
                ],
            )
            rows = load_rows(path)
            prediction, graph_initial, posterior = tum_poses(rows)

        self.assertEqual(len(rows), 3)
        self.assertEqual(len(prediction), 2)
        self.assertEqual(len(graph_initial), 2)
        self.assertEqual(
            [pose.timestamp_ns for pose in prediction],
            [1_000_000_000, 1_100_000_000],
        )
        self.assertEqual(
            [pose.timestamp_ns for pose in posterior],
            [1_000_000_000, 1_100_000_000],
        )
        self.assertEqual(prediction[0].line.split()[0], "1.000000000")
        self.assertEqual(prediction[0].line.split()[-1], "1")
        self.assertEqual(graph_initial[0].line.split()[1], "2")
        self.assertEqual(posterior[0].line.split()[1], "3")

    def test_accepts_additional_sidecar_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            row = self.make_row(0)
            row["factor_cost_valid"] = "0"
            self.write_rows(
                path,
                [row],
                fields=STATE_FIELDS + ("factor_cost_valid",),
            )

            rows = load_rows(path)

        self.assertEqual(len(rows), 1)

    def test_loads_full_inertial_rows_for_mixed_mode_sidecars(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            row = self.make_row(0)
            row["fusion_mode"] = "full_visual_inertial"
            row["gyro_factor_n"] = "0"
            self.write_rows(path, [row])

            rows = load_rows(path)

        self.assertEqual(rows[0].fusion_mode, "full_visual_inertial")
        prediction, graph_initial, posterior = tum_poses(rows)
        self.assertEqual(len(prediction), 1)
        self.assertEqual(len(graph_initial), 1)
        self.assertEqual(len(posterior), 1)

    def test_correction_summary_and_bucket_analysis(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            self.write_rows(path, [self.make_row(i) for i in range(25)])
            rows = load_rows(path)
            euroc_root = Path(directory) / "euroc"
            self.write_groundtruth(euroc_root, 25)

            calls: list[tuple[str, int]] = []

            def fake_evaluate(
                evaluator: Path, estimate: Path, euroc_root: Path
            ) -> Metrics:
                del evaluator, euroc_root
                count = len(
                    [
                        line
                        for line in estimate.read_text(
                            encoding="utf-8"
                        ).splitlines()
                        if line
                    ]
                )
                calls.append((estimate.name, count))
                is_prediction = estimate.name == "prediction.tum"
                is_graph_initial = estimate.name == "graph_initial.tum"
                return Metrics(
                    poses=count,
                    matched=count,
                    ate_rmse_m=(
                        0.1 if is_prediction else 0.15 if is_graph_initial else 0.2
                    ),
                    rpe_pairs=count - 20,
                    rpe_rmse_m=(
                        0.01 if is_prediction else 0.02 if is_graph_initial else 0.03
                    ),
                )

            with patch("vio_state_probe.evaluate", side_effect=fake_evaluate):
                result = analyze(
                    rows,
                    euroc_root=euroc_root,
                    evaluator=Path("/traj-eval"),
                    bucket_poses=22,
                )

        correction = correction_summary(rows)
        self.assertEqual(correction.poses, 25)
        self.assertAlmostEqual(correction.pose_translation_rms_m, 3.0)
        self.assertAlmostEqual(correction.gyro_bias_rms_rps, 0.2)
        self.assertAlmostEqual(result.full.ate_post_minus_pred_m, 0.1)
        self.assertAlmostEqual(result.full.ate_graph_init_minus_pred_m, 0.05)
        self.assertAlmostEqual(result.full.ate_post_minus_graph_init_m, 0.05)
        self.assertAlmostEqual(result.full.rpe_post_minus_pred_m, 0.02)
        self.assertEqual(len(result.buckets), 2)
        self.assertIsNotNone(result.buckets[0].metrics)
        self.assertIsNone(result.buckets[1].metrics)
        self.assertEqual(
            calls,
            [
                ("prediction.tum", 25),
                ("graph_initial.tum", 25),
                ("posterior.tum", 25),
                ("prediction.tum", 22),
                ("graph_initial.tum", 22),
                ("posterior.tum", 22),
            ],
        )

        output = io.StringIO()
        render_bucket_csv(result.buckets, output)
        rendered = output.getvalue()
        self.assertIn("ate_post_minus_pred_m", rendered)
        self.assertIn("0.10000000000000001", rendered)

    def test_buckets_use_evaluable_groundtruth_support(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "state.csv"
            self.write_rows(path, [self.make_row(i) for i in range(140)])
            rows = load_rows(path)
            euroc_root = root / "euroc"
            self.write_groundtruth(euroc_root, 118)

            calls: list[int] = []

            def fake_evaluate(
                evaluator: Path, estimate: Path, groundtruth: Path
            ) -> Metrics:
                del evaluator, groundtruth
                count = len(estimate.read_text(encoding="utf-8").splitlines())
                calls.append(count)
                return Metrics(count, count, 0.1, count - 20, 0.01)

            with patch("vio_state_probe.evaluate", side_effect=fake_evaluate):
                result = analyze(
                    rows,
                    euroc_root=euroc_root,
                    evaluator=Path("/traj-eval"),
                    bucket_poses=100,
                )

        self.assertEqual(result.prediction_valid_rows, 140)
        self.assertEqual(result.evaluation_rows, 118)
        self.assertEqual(result.outside_groundtruth_rows, 22)
        self.assertEqual([bucket.poses for bucket in result.buckets], [100, 18])
        self.assertIsNone(result.buckets[1].metrics)
        self.assertEqual(calls, [118, 118, 118, 100, 100, 100])

    def test_rejects_schema_quaternion_and_timestamp_violations(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "state.csv"

            self.write_rows(path, [self.make_row(0)], fields=STATE_FIELDS[:-1])
            with self.assertRaisesRegex(ProbeError, "schema"):
                load_rows(path)

            invalid_q = self.make_row(0)
            invalid_q["pred_q_w"] = "2"
            self.write_rows(path, [invalid_q])
            with self.assertRaisesRegex(ProbeError, "unit quaternion"):
                load_rows(path)

            duplicate = [self.make_row(0), self.make_row(1)]
            duplicate[1]["timestamp_ns"] = duplicate[0]["timestamp_ns"]
            self.write_rows(path, duplicate)
            with self.assertRaisesRegex(ProbeError, "strictly increasing"):
                load_rows(path)


if __name__ == "__main__":
    unittest.main()

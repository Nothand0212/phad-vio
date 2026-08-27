#!/usr/bin/env python3
"""Unit tests for vio_bias_probe.py."""

from __future__ import annotations

import csv
import io
import tempfile
import unittest
from pathlib import Path

from vio_bias_probe import ProbeError, join_bias, load_diag, load_groundtruth, render_csv, summarize


class VioBiasProbeTest(unittest.TestCase):
    def write_diag(self, path: Path) -> None:
        header = (
            "timestamp_ns",
            "status",
            "window_size",
            "prior_key",
            "is_keyframe",
            "bias_gyro_x",
            "bias_gyro_y",
            "bias_gyro_z",
            "bias_acc_x",
            "bias_acc_y",
            "bias_acc_z",
        )
        rows = (
            (1_000_000_000, "ok", 9, 10, 1, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0),
            (1_050_000_000, "rejected", 9, 10, 0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0),
            (1_100_000_000, "ok", 10, 11, 0, 0.2, 0.0, 0.0, 2.0, 0.0, 0.0),
            (1_200_000_000, "ok", 10, 11, 1, 0.3, 0.0, 0.0, 3.0, 0.0, 0.0),
        )
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            output.write(", ".join(header) + "\n")
            writer.writerows(rows)

    def write_groundtruth(self, path: Path) -> None:
        header = (
            "#timestamp",
            "b_w_RS_S_x [rad s^-1]",
            "b_w_RS_S_y [rad s^-1]",
            "b_w_RS_S_z [rad s^-1]",
            "b_a_RS_S_x [m s^-2]",
            "b_a_RS_S_y [m s^-2]",
            "b_a_RS_S_z [m s^-2]",
        )
        rows = (
            (999_000_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (1_099_000_000, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0),
            (1_199_000_000, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (1_201_000_000, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        )
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(header)
            writer.writerows(rows)

    def test_join_summarize_and_render(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            diag_path = root / "diag.csv"
            gt_path = root / "data.csv"
            self.write_diag(diag_path)
            self.write_groundtruth(gt_path)

            samples = join_bias(
                load_diag(diag_path),
                load_groundtruth(gt_path),
                max_dt_ns=2_500_000,
            )
            buckets = summarize(samples, bucket_frames=2)
            output = io.StringIO()
            render_csv(buckets, output)

        self.assertEqual(len(samples), 3)
        self.assertEqual(len(buckets), 2)
        self.assertEqual(buckets[0].frames, 2)
        self.assertEqual(buckets[0].keyframes, 1)
        self.assertEqual(buckets[0].prior_advances, 1)
        self.assertAlmostEqual(buckets[0].acc_err_rms_mps2, (3.25 / 2.0) ** 0.5)
        self.assertAlmostEqual(buckets[0].gyro_err_rms_rps, (0.05 / 2.0) ** 0.5)
        self.assertAlmostEqual(buckets[0].acc_est_drift_mps2, 1.0)
        self.assertIn("acc_err_rms_mps2", output.getvalue())

    def test_rejects_groundtruth_outside_tolerance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            diag_path = root / "diag.csv"
            gt_path = root / "data.csv"
            self.write_diag(diag_path)
            self.write_groundtruth(gt_path)
            diag = load_diag(diag_path)
            gt = load_groundtruth(gt_path)
            with self.assertRaisesRegex(ProbeError, "no groundtruth within"):
                join_bias(diag, gt, max_dt_ns=100)

    def test_rejects_non_monotonic_diag(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "diag.csv"
            self.write_diag(path)
            rows = path.read_text(encoding="utf-8").splitlines()
            rows[-1], rows[-2] = rows[-2], rows[-1]
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ProbeError, "strictly increasing"):
                load_diag(path)


if __name__ == "__main__":
    unittest.main()

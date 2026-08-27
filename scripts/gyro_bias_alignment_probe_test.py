#!/usr/bin/env python3
"""Unit tests for gyro_bias_alignment_probe.py."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

from gyro_bias_alignment_probe import (
    ImuSample,
    ProbeError,
    ReferenceBias,
    RotationPair,
    analyze_rolling,
    estimate_bias,
    load_init_seed,
)


class GyroBiasAlignmentProbeTest(unittest.TestCase):
    def test_recovers_constant_bias_from_visual_rotation(self) -> None:
        expected = (0.012, -0.023, 0.034)
        pairs = tuple(
            RotationPair(
                t_i_ns=index * 50_000_000,
                t_j_ns=(index + 1) * 50_000_000,
                imu=(
                    ImuSample(index * 50_000_000, expected),
                    ImuSample((index + 1) * 50_000_000, expected),
                ),
                visual_delta=(1.0, 0.0, 0.0, 0.0),
            )
            for index in range(20)
        )

        result = estimate_bias(pairs, (0.0, 0.0, 0.0))

        for actual, target in zip(result.bias, expected):
            self.assertAlmostEqual(actual, target, places=10)
        self.assertLess(result.rotation_residual_rms_rad, 1e-10)
        self.assertTrue(math.isfinite(result.normal_condition))
        self.assertAlmostEqual(result.normal_condition, 1.0, places=8)
        self.assertEqual(result.bias_sigma_iid, (0.0, 0.0, 0.0))
        self.assertEqual(result.residual_lag1, (0.0, 0.0, 0.0))

    def test_rejects_empty_pair_set(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least one rotation pair"):
            estimate_bias((), (0.0, 0.0, 0.0))

    def test_rolling_analysis_uses_only_complete_strided_windows(self) -> None:
        expected = (0.012, -0.023, 0.034)
        interval_ns = 50_000_000
        pairs = tuple(
            RotationPair(
                t_i_ns=index * interval_ns,
                t_j_ns=(index + 1) * interval_ns,
                imu=(
                    ImuSample(index * interval_ns, expected),
                    ImuSample((index + 1) * interval_ns, expected),
                ),
                visual_delta=(1.0, 0.0, 0.0, 0.0),
            )
            for index in range(6)
        )
        reference = (
            ReferenceBias(-interval_ns, expected),
            ReferenceBias(7 * interval_ns, expected),
        )

        rows = analyze_rolling(
            pairs,
            (0.0, 0.0, 0.0),
            reference,
            window_pairs=3,
            stride_pairs=2,
        )

        self.assertEqual(len(rows), 2)
        self.assertEqual(
            tuple((row.pair_count, row.start_ns, row.end_ns) for row in rows),
            (
                (3, 0, 3 * interval_ns),
                (3, 2 * interval_ns, 5 * interval_ns),
            ),
        )
        for row in rows:
            for actual, target in zip(row.estimate.bias, expected):
                self.assertAlmostEqual(actual, target, places=10)

    def test_rolling_analysis_rejects_invalid_window_contract(self) -> None:
        pair = RotationPair(
            t_i_ns=0,
            t_j_ns=50_000_000,
            imu=(
                ImuSample(0, (0.0, 0.0, 0.0)),
                ImuSample(50_000_000, (0.0, 0.0, 0.0)),
            ),
            visual_delta=(1.0, 0.0, 0.0, 0.0),
        )
        reference = (
            ReferenceBias(-1, (0.0, 0.0, 0.0)),
            ReferenceBias(100_000_000, (0.0, 0.0, 0.0)),
        )
        with self.assertRaisesRegex(ValueError, "window_pairs"):
            analyze_rolling(
                (pair,),
                (0.0, 0.0, 0.0),
                reference,
                window_pairs=2,
                stride_pairs=1,
            )

    def test_loads_seed_from_exactly_one_init_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vio_init.csv"
            path.write_text(
                "init_bg_x,init_bg_y,init_bg_z\n0.1,-0.2,0.3\n",
                encoding="utf-8",
            )
            self.assertEqual(load_init_seed(path), (0.1, -0.2, 0.3))
            path.write_text(
                "init_bg_x,init_bg_y,init_bg_z\n0,0,0\n1,1,1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ProbeError, "exactly one"):
                load_init_seed(path)


if __name__ == "__main__":
    unittest.main()

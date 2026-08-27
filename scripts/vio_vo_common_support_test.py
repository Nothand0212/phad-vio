#!/usr/bin/env python3
"""Unit tests for vio_vo_common_support.py."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from vio_vo_common_support import (
    ComparisonError,
    common_support,
    load_tum,
    parse_metrics,
)


class VioVoCommonSupportTest(unittest.TestCase):
    def write_tum(self, path: Path, timestamps: tuple[str, ...]) -> None:
        lines = [
            f"{timestamp} 0 0 0 0 0 0 1\n" for timestamp in timestamps
        ]
        path.write_text("# trajectory\n" + "".join(lines), encoding="utf-8")

    def test_load_and_exact_timestamp_intersection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            vio_path = root / "vio.tum"
            vo_path = root / "vo.tum"
            self.write_tum(vio_path, ("1.000000000", "1.05", "1.10"))
            self.write_tum(vo_path, ("1.050000000", "1.1", "1.15"))
            vio, vo = common_support(load_tum(vio_path), load_tum(vo_path))

        self.assertEqual(
            [pose.timestamp_ns for pose in vio],
            [1_050_000_000, 1_100_000_000],
        )
        self.assertEqual(
            [pose.timestamp_ns for pose in vo],
            [1_050_000_000, 1_100_000_000],
        )

    def test_parse_evaluator_metrics(self) -> None:
        output = """\
estimate poses 100, groundtruth poses 1000
matched 98 of 100 (rate 0.98), dropped 2 outside groundtruth span and 0 beyond 2.5 ms
ATE translation [m]   rmse 0.12  mean 0.1
RPE over 1 s from 78 pose pairs, skipped 20 poses without a partner
RPE translation [m]   rmse 0.03  mean 0.02
"""
        metrics = parse_metrics(output)
        self.assertEqual(metrics.poses, 100)
        self.assertEqual(metrics.matched, 98)
        self.assertEqual(metrics.rpe_pairs, 78)
        self.assertEqual(metrics.ate_rmse_m, 0.12)
        self.assertEqual(metrics.rpe_rmse_m, 0.03)

    def test_rejects_duplicate_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.tum"
            self.write_tum(path, ("1.0", "1.0"))
            with self.assertRaisesRegex(
                ComparisonError, "strictly increasing"
            ):
                load_tum(path)

    def test_rejects_missing_metric(self) -> None:
        with self.assertRaisesRegex(ComparisonError, "RPE pair count"):
            parse_metrics(
                "estimate poses 2, groundtruth poses 2\n"
                "matched 2 of 2\n"
                "ATE translation [m] rmse 0.1\n"
            )


if __name__ == "__main__":
    unittest.main()

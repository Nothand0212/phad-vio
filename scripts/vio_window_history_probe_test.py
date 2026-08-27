#!/usr/bin/env python3
"""Tests for the M4.4 window-history counterfactual probe."""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from imu_translation_alignment_probe import (
    PreintegratedInterval,
    ReferenceState,
)
from vio_window_history_probe import (
    FullState,
    ProbeError,
    counterfactual_errors,
    fit_rigid,
    load_full_states,
)


class VioWindowHistoryProbeTest(unittest.TestCase):
    def test_fit_rigid_recovers_rotation_and_translation(self) -> None:
        estimate = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.0, 0.0, 3.0],
            ]
        )
        rotation = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        )
        translation = np.array([3.0, -2.0, 0.5])
        reference = (rotation @ estimate.T).T + translation

        transform = fit_rigid(estimate, reference)

        np.testing.assert_allclose(transform.rotation, rotation, atol=1e-12)
        np.testing.assert_allclose(
            transform.translation, translation, atol=1e-12
        )
        self.assertLess(transform.rmse(estimate, reference), 1e-12)

    def test_gt_velocity_removes_injected_boundary_error(self) -> None:
        identity = np.eye(3)
        candidate = FullState(
            timestamp_ns=0,
            position=np.zeros(3),
            rotation=identity,
            velocity=np.array([1.2, 0.0, 0.0]),
            gravity=np.array([0.0, 0.0, -10.0]),
            gyro_bias=np.zeros(3),
            accel_bias=np.zeros(3),
        )
        reference_start = ReferenceState(
            0,
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.zeros(3),
            np.zeros(3),
        )
        reference_end = ReferenceState(
            1_000_000_000,
            np.array([1.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.zeros(3),
            np.zeros(3),
        )
        interval = PreintegratedInterval(
            0,
            1_000_000_000,
            np.array([0.0, 0.0, 5.0]),
            np.array([0.0, 0.0, 10.0]),
        )

        errors = counterfactual_errors(
            candidate,
            reference_start,
            reference_end,
            interval,
            interval,
            gravity_magnitude=10.0,
        )

        self.assertAlmostEqual(errors["actual"], 0.2)
        self.assertLess(errors["gt_velocity"], 1e-12)
        self.assertAlmostEqual(errors["gt_gravity"], 0.2)
        self.assertAlmostEqual(errors["gt_bias"], 0.2)
        self.assertLess(errors["gt_velocity_gravity"], 1e-12)
        self.assertLess(errors["model_floor"], 1e-12)

    def test_load_full_states_allows_extra_columns_and_rejects_duplicates(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.csv"
            fields = [
                "timestamp_ns",
                "fusion_mode",
                "post_p_x",
                "post_p_y",
                "post_p_z",
                "post_q_x",
                "post_q_y",
                "post_q_z",
                "post_q_w",
                "velocity_x",
                "velocity_y",
                "velocity_z",
                "gravity_x",
                "gravity_y",
                "gravity_z",
                "post_bg_x",
                "post_bg_y",
                "post_bg_z",
                "post_ba_x",
                "post_ba_y",
                "post_ba_z",
                "extra_diagnostic",
            ]
            with path.open("w", encoding="utf-8", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    {
                        "timestamp_ns": "100",
                        "fusion_mode": "gyro_visual",
                        "post_q_w": "1",
                    }
                )
                writer.writerow(
                    {
                        "timestamp_ns": "200",
                        "fusion_mode": "full_visual_inertial",
                        "post_p_x": "1",
                        "post_p_y": "2",
                        "post_p_z": "3",
                        "post_q_x": "0",
                        "post_q_y": "0",
                        "post_q_z": "0",
                        "post_q_w": "1",
                        "velocity_x": "4",
                        "velocity_y": "5",
                        "velocity_z": "6",
                        "gravity_x": "0",
                        "gravity_y": "0",
                        "gravity_z": "-9.81",
                        "post_bg_x": "0.1",
                        "post_bg_y": "0.2",
                        "post_bg_z": "0.3",
                        "post_ba_x": "0.4",
                        "post_ba_y": "0.5",
                        "post_ba_z": "0.6",
                        "extra_diagnostic": "7",
                    }
                )

            states = load_full_states(path)

            self.assertEqual(len(states), 1)
            self.assertEqual(states[0].timestamp_ns, 200)
            np.testing.assert_allclose(states[0].velocity, [4.0, 5.0, 6.0])

            duplicate = Path(directory) / "duplicate.csv"
            duplicate.write_text(
                ",".join(fields + ["timestamp_ns"]) + "\n",
                encoding="utf-8",
            )
            with self.assertRaises(ProbeError):
                load_full_states(duplicate)


if __name__ == "__main__":
    unittest.main()

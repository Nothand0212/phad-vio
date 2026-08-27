#!/usr/bin/env python3
"""Tests for the M4.4 velocity-history shadow probe."""

from __future__ import annotations

import unittest

import numpy as np

from vio_velocity_history_probe import (
    VelocityInterval,
    solve_batch_velocities,
    solve_velocity_shadows,
)


class VioVelocityHistoryProbeTest(unittest.TestCase):
    def test_tridiagonal_batch_matches_dense_least_squares(self) -> None:
        intervals = (
            VelocityInterval(0, 40_000_000, np.array([0.1, -0.2, 0.3]),
                             np.array([0.01, 0.02, -0.03])),
            VelocityInterval(40_000_000, 90_000_000,
                             np.array([-0.4, 0.5, 0.2]),
                             np.array([0.04, -0.01, 0.02])),
            VelocityInterval(90_000_000, 150_000_000,
                             np.array([0.6, 0.1, -0.3]),
                             np.array([-0.02, 0.03, 0.05])),
        )
        matrix = np.zeros((6 * len(intervals), 3 * (len(intervals) + 1)))
        vector = np.zeros(6 * len(intervals))
        identity = np.eye(3)
        for index, interval in enumerate(intervals):
            dt = (interval.t_j_ns - interval.t_i_ns) * 1e-9
            position_rows = slice(6 * index, 6 * index + 3)
            velocity_rows = slice(6 * index + 3, 6 * index + 6)
            velocity_i = slice(3 * index, 3 * index + 3)
            velocity_j = slice(3 * (index + 1), 3 * (index + 1) + 3)
            matrix[position_rows, velocity_i] = dt * identity
            matrix[velocity_rows, velocity_i] = -identity
            matrix[velocity_rows, velocity_j] = identity
            vector[position_rows] = interval.position_rhs
            vector[velocity_rows] = interval.velocity_rhs
        expected, _, rank, _ = np.linalg.lstsq(matrix, vector, rcond=None)
        self.assertEqual(rank, matrix.shape[1])

        actual = solve_batch_velocities(intervals)

        np.testing.assert_allclose(
            actual, expected.reshape((-1, 3)), rtol=1e-12, atol=1e-12
        )

    def test_all_shadows_recover_consistent_motion(self) -> None:
        velocities = np.array(
            [
                [1.0, -0.5, 0.2],
                [1.1, -0.4, 0.25],
                [1.3, -0.2, 0.35],
                [1.6, 0.1, 0.50],
            ]
        )
        timestamps = (0, 50_000_000, 100_000_000, 150_000_000)
        intervals = tuple(
            VelocityInterval(
                timestamps[index],
                timestamps[index + 1],
                0.05 * velocities[index],
                velocities[index + 1] - velocities[index],
            )
            for index in range(len(timestamps) - 1)
        )

        shadows = solve_velocity_shadows(intervals, window_size=3)

        np.testing.assert_allclose(shadows.full_batch, velocities, atol=1e-12)
        np.testing.assert_allclose(shadows.one_interval[1:], velocities[1:])
        np.testing.assert_allclose(shadows.causal_marginal[1:], velocities[1:])
        np.testing.assert_allclose(shadows.fixed_window[2:], velocities[2:])
        self.assertTrue(np.isnan(shadows.one_interval[0]).all())
        self.assertTrue(np.isnan(shadows.fixed_window[:2]).all())

    def test_causal_outputs_do_not_consume_future_interval(self) -> None:
        intervals = (
            VelocityInterval(0, 50_000_000, np.array([0.05, 0.0, 0.0]),
                             np.array([0.1, 0.0, 0.0])),
            VelocityInterval(50_000_000, 100_000_000,
                             np.array([0.06, 0.0, 0.0]),
                             np.array([0.2, 0.0, 0.0])),
            VelocityInterval(100_000_000, 150_000_000,
                             np.array([0.07, 0.0, 0.0]),
                             np.array([0.3, 0.0, 0.0])),
        )
        changed = intervals[:-1] + (
            VelocityInterval(100_000_000, 150_000_000,
                             np.array([7.0, 0.0, 0.0]),
                             np.array([30.0, 0.0, 0.0])),
        )

        original = solve_velocity_shadows(intervals, window_size=2)
        modified = solve_velocity_shadows(changed, window_size=2)

        np.testing.assert_array_equal(
            original.causal_marginal[:3], modified.causal_marginal[:3]
        )
        np.testing.assert_array_equal(
            original.one_interval[:3], modified.one_interval[:3]
        )
        self.assertFalse(
            np.allclose(original.full_batch[:3], modified.full_batch[:3])
        )


if __name__ == "__main__":
    unittest.main()

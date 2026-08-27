#!/usr/bin/env python3
"""Unit tests for imu_translation_alignment_probe.py."""

from __future__ import annotations

import unittest

import numpy as np

from imu_translation_alignment_probe import (
    ImuSample,
    Pose,
    PreintegratedInterval,
    integrate_interval,
    solve_alignment,
    solve_bias_alignment,
)


class ImuTranslationAlignmentProbeTest(unittest.TestCase):
    def test_recovers_metric_gravity_and_velocities(self) -> None:
        gravity = np.array([0.2, -0.3, -9.803443])
        gravity *= 9.81007 / np.linalg.norm(gravity)
        dt_s = 0.1
        dt_ns = 100_000_000
        velocity = np.array([0.4, -0.2, 0.1])
        position = np.zeros(3)
        poses: list[Pose] = [Pose(0, np.eye(3), position.copy())]
        intervals: list[PreintegratedInterval] = []

        for index in range(30):
            world_acceleration = np.array(
                [
                    0.3 * np.sin(0.2 * index),
                    -0.2 * np.cos(0.15 * index),
                    0.1 * np.sin(0.11 * index),
                ]
            )
            specific_force = world_acceleration - gravity
            delta_v = specific_force * dt_s
            delta_p = 0.5 * specific_force * dt_s * dt_s
            intervals.append(
                PreintegratedInterval(
                    index * dt_ns,
                    (index + 1) * dt_ns,
                    delta_p,
                    delta_v,
                )
            )
            position = (
                position
                + velocity * dt_s
                + 0.5 * world_acceleration * dt_s * dt_s
            )
            velocity = velocity + world_acceleration * dt_s
            poses.append(
                Pose((index + 1) * dt_ns, np.eye(3), position.copy())
            )

        result = solve_alignment(tuple(poses), tuple(intervals), 9.81007)

        np.testing.assert_allclose(result.gravity, gravity, atol=1e-9)
        np.testing.assert_allclose(
            result.velocities[0], np.array([0.4, -0.2, 0.1]), atol=1e-9
        )
        np.testing.assert_allclose(result.velocities[-1], velocity, atol=1e-9)
        self.assertLess(result.position_residual_rms_m, 1e-10)
        self.assertLess(result.velocity_residual_rms_mps, 1e-10)
        self.assertEqual(result.rank, 3 * len(poses) + 3)

    def test_integration_subtracts_bias_before_preintegration(self) -> None:
        samples = (
            ImuSample(
                0,
                np.array([0.01, -0.02, 0.03]),
                np.array([1.1, 1.8, 3.3]),
            ),
            ImuSample(
                100_000_000,
                np.array([0.01, -0.02, 0.03]),
                np.array([1.1, 1.8, 3.3]),
            ),
        )

        result = integrate_interval(
            samples,
            gyro_bias=np.array([0.01, -0.02, 0.03]),
            accel_bias_at=lambda _: np.array([0.1, -0.2, 0.3]),
        )

        np.testing.assert_allclose(result.delta_v, [0.1, 0.2, 0.3], atol=1e-12)
        np.testing.assert_allclose(
            result.delta_p, [0.005, 0.01, 0.015], atol=1e-12
        )

    def test_rejects_single_interval_alignment(self) -> None:
        poses = (
            Pose(0, np.eye(3), np.zeros(3)),
            Pose(50_000_000, np.eye(3), np.zeros(3)),
        )
        intervals = (
            PreintegratedInterval(
                0, 50_000_000, np.zeros(3), np.zeros(3)
            ),
        )

        with self.assertRaisesRegex(ValueError, "at least two intervals"):
            solve_alignment(poses, intervals, 9.81007)

    def test_staged_alignment_recovers_constant_accelerometer_bias(self) -> None:
        gravity = np.array([0.1, -0.2, -9.807520])
        gravity *= 9.81007 / np.linalg.norm(gravity)
        expected_bias = np.array([0.08, -0.12, 0.05])
        dt_s = 0.2
        dt_ns = 200_000_000
        velocity = np.array([0.2, -0.1, 0.3])
        position = np.zeros(3)
        poses: list[Pose] = []
        intervals: list[PreintegratedInterval] = []

        for index in range(16):
            roll = 0.07 * index
            pitch = 0.11 * index
            yaw = -0.09 * index
            rotation_x = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, np.cos(roll), -np.sin(roll)],
                    [0.0, np.sin(roll), np.cos(roll)],
                ]
            )
            rotation_y = np.array(
                [
                    [np.cos(pitch), 0.0, np.sin(pitch)],
                    [0.0, 1.0, 0.0],
                    [-np.sin(pitch), 0.0, np.cos(pitch)],
                ]
            )
            rotation_z = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw), 0.0],
                    [np.sin(yaw), np.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            rotation = rotation_z @ rotation_y @ rotation_x
            poses.append(Pose(index * dt_ns, rotation, position.copy()))
            if index == 15:
                break
            world_acceleration = np.array(
                [0.2 * np.sin(index), -0.1 * np.cos(0.4 * index), 0.05]
            )
            corrected_specific_force = rotation.T @ (
                world_acceleration - gravity
            )
            jacobian_v = -dt_s * np.eye(3)
            jacobian_p = -0.5 * dt_s * dt_s * np.eye(3)
            intervals.append(
                PreintegratedInterval(
                    index * dt_ns,
                    (index + 1) * dt_ns,
                    0.5
                    * (corrected_specific_force + expected_bias)
                    * dt_s
                    * dt_s,
                    (corrected_specific_force + expected_bias) * dt_s,
                    np.eye(3),
                    jacobian_p,
                    jacobian_v,
                )
            )
            position = (
                position
                + velocity * dt_s
                + 0.5 * world_acceleration * dt_s * dt_s
            )
            velocity = velocity + world_acceleration * dt_s

        result = solve_bias_alignment(
            tuple(poses),
            tuple(intervals),
            gravity_magnitude=9.81007,
            accel_bias_seed=np.zeros(3),
        )

        np.testing.assert_allclose(result.accel_bias, expected_bias, atol=1e-8)
        np.testing.assert_allclose(result.gravity, gravity, atol=1e-8)
        self.assertLess(result.position_residual_rms_m, 1e-9)
        self.assertLess(result.velocity_residual_rms_mps, 1e-9)


if __name__ == "__main__":
    unittest.main()

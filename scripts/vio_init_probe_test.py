#!/usr/bin/env python3
"""Unit tests for vio_init_probe.py."""

from __future__ import annotations

import csv
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

from vio_init_probe import (
    GT_FIELDS,
    INIT_FIELDS,
    ProbeError,
    analyze,
    load_init,
    render_json,
)


class VioInitProbeTest(unittest.TestCase):
    def make_init_row(self) -> dict[str, str]:
        row = {field: "0" for field in INIT_FIELDS}
        row.update(
            {
                "imu_t_i_ns": "1000000000",
                "imu_t_j_ns": "1500000000",
                "imu_sample_n": "101",
                "imu_dt_s": "0.5",
                "gyro_mean_x": "0.02",
                "gyro_mean_y": "-0.01",
                "gyro_mean_z": "0.03",
                "gyro_std_x": "0",
                "gyro_std_y": "0",
                "gyro_std_z": "0",
                "acc_mean_x": "0.4",
                "acc_mean_y": "-0.2",
                "acc_mean_z": "9.9",
                "acc_std_x": "0",
                "acc_std_y": "0",
                "acc_std_z": "0",
                "gyro_std_limit": "0.01",
                "acc_std_limit": "0.2",
                "init_q_w": "1",
                "init_v_x": "0",
                "init_v_y": "0",
                "init_v_z": "0",
                "init_bg_x": "0.02",
                "init_bg_y": "-0.01",
                "init_bg_z": "0.03",
                "init_ba_x": "0.3",
                "init_ba_y": "-0.1",
                "init_ba_z": "0.2",
                "acc_mean_norm": str(math.sqrt(0.4**2 + 0.2**2 + 9.9**2)),
                "gravity_model_magnitude": "9.81",
            }
        )
        return row

    def write_init(
        self,
        path: Path,
        rows: list[dict[str, str]],
        fields: tuple[str, ...] = INIT_FIELDS,
    ) -> None:
        with path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(
                output, fieldnames=fields, extrasaction="ignore"
            )
            writer.writeheader()
            writer.writerows(rows)

    def write_euroc(
        self,
        root: Path,
        *,
        imu_transform: tuple[float, ...] | None = None,
        t0_ns: int = 1_000_000_000,
        t1_ns: int = 1_500_000_000,
    ) -> None:
        identity = (
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        )
        imu_transform = imu_transform or identity
        for sensor, transform in (
            ("imu0", imu_transform),
            ("state_groundtruth_estimate0", identity),
        ):
            directory = root / "mav0" / sensor
            directory.mkdir(parents=True)
            values = ", ".join(str(value) for value in transform)
            (directory / "sensor.yaml").write_text(
                "sensor_type: test\n"
                "T_BS:\n"
                "  cols: 4\n"
                "  rows: 4\n"
                f"  data: [{values}]\n",
                encoding="utf-8",
            )

        gt_path = (
            root
            / "mav0"
            / "state_groundtruth_estimate0"
            / "data.csv"
        )
        with gt_path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(GT_FIELDS)
            for timestamp_ns, velocity_x in (
                (t0_ns - 1_000_000, 0.03),
                (t0_ns + 1_000_000, 0.03),
                (t1_ns - 1_000_000, 0.13),
                (t1_ns + 1_000_000, 0.13),
            ):
                writer.writerow(
                    (
                        timestamp_ns,
                        1.0,
                        2.0,
                        3.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        velocity_x,
                        0.04,
                        0.0,
                        0.01,
                        -0.02,
                        0.04,
                        0.1,
                        0.2,
                        0.3,
                    )
                )

        imu_path = root / "mav0" / "imu0" / "data.csv"
        with imu_path.open("w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(
                (
                    "#timestamp [ns]",
                    "w_RS_S_x [rad s^-1]",
                    "w_RS_S_y [rad s^-1]",
                    "w_RS_S_z [rad s^-1]",
                    "a_RS_S_x [m s^-2]",
                    "a_RS_S_y [m s^-2]",
                    "a_RS_S_z [m s^-2]",
                )
            )
            for index in range(101):
                writer.writerow(
                    (
                        1_000_000_000 + index * 5_000_000,
                        0.02,
                        -0.01,
                        0.03,
                        0.4,
                        -0.2,
                        9.9,
                    )
                )

    def test_truth_audit_reports_frame_and_bias_decomposition(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            init_path = root / "init.csv"
            self.write_init(init_path, [self.make_init_row()])
            euroc = root / "euroc"
            self.write_euroc(euroc)

            result = analyze(load_init(init_path), euroc)
            output = io.StringIO()
            render_json(result, output)
            rendered = json.loads(output.getvalue())

        self.assertEqual(result.imu_sample_count, 101)
        self.assertTrue(result.raw_imu_stats_verified)
        self.assertEqual(result.gt_t0_left_ns, 999_000_000)
        self.assertEqual(result.gt_t0_right_ns, 1_001_000_000)
        self.assertAlmostEqual(result.gt_t0_alpha, 0.5)
        self.assertEqual(result.gt_t1_left_ns, 1_499_000_000)
        self.assertEqual(result.gt_t1_right_ns, 1_501_000_000)
        self.assertAlmostEqual(result.gt_t1_alpha, 0.5)
        self.assertAlmostEqual(result.gravity_angle_deg, 0.0)
        self.assertAlmostEqual(result.gt_t0_speed_mps, 0.05)
        self.assertAlmostEqual(
            result.gt_t1_speed_mps, math.sqrt(0.13**2 + 0.04**2)
        )
        self.assertAlmostEqual(result.gt_window_rotation_deg, 0.0)
        self.assertAlmostEqual(result.gt_delta_v_over_dt_mps2, 0.2)
        for actual, expected in zip(
            result.gyro_bias_error, (0.01, 0.01, -0.01)
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(
            result.gyro_bias_error_norm_rps, math.sqrt(0.0003)
        )
        for actual, expected in zip(
            result.acc_bias_error, (0.2, -0.3, -0.1)
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result.acc_bias_error_parallel_mps2, -0.1)
        self.assertAlmostEqual(
            result.acc_bias_error_tangent_mps2, math.sqrt(0.13)
        )
        self.assertAlmostEqual(
            result.acc_mean_norm_minus_gravity_model_mps2,
            math.sqrt(0.4**2 + 0.2**2 + 9.9**2) - 9.81,
        )
        for actual, expected in zip(
            result.static_gyro_residual_rps, (0.01, 0.01, -0.01)
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            result.static_acc_residual_mps2, (0.3, -0.4, -0.21)
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(rendered["imu_sample_count"], 101)
        self.assertIn("acc_bias_error_tangent_mps2", rendered)

    def test_rejects_probe_schema_row_count_quaternion_and_interval(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "init.csv"
            row = self.make_init_row()

            self.write_init(path, [row], fields=INIT_FIELDS[:-1])
            with self.assertRaisesRegex(ProbeError, "schema"):
                load_init(path)

            self.write_init(path, [row, row])
            with self.assertRaisesRegex(ProbeError, "exactly one"):
                load_init(path)

            invalid_q = dict(row)
            invalid_q["init_q_w"] = "2"
            self.write_init(path, [invalid_q])
            with self.assertRaisesRegex(ProbeError, "unit quaternion"):
                load_init(path)

            invalid_interval = dict(row)
            invalid_interval["imu_dt_s"] = "0.4"
            self.write_init(path, [invalid_interval])
            with self.assertRaisesRegex(ProbeError, "duration"):
                load_init(path)

    def test_rejects_nonidentity_extrinsics_and_timestamp_miss(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            init_path = root / "init.csv"
            self.write_init(init_path, [self.make_init_row()])
            snapshot = load_init(init_path)

            nonidentity = list(
                (
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.1,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
            )
            euroc = root / "nonidentity"
            self.write_euroc(euroc, imu_transform=tuple(nonidentity))
            with self.assertRaisesRegex(ProbeError, "identity T_BS"):
                analyze(snapshot, euroc)

            missed = root / "missed"
            self.write_euroc(
            missed,
            t0_ns=1_010_000_000,
            t1_ns=1_510_000_000,
        )
            with self.assertRaisesRegex(ProbeError, "outside groundtruth support"):
                analyze(snapshot, missed)

            wide = root / "wide"
            self.write_euroc(wide)
            with self.assertRaisesRegex(ProbeError, "bracket span"):
                analyze(snapshot, wide, max_bracket_ns=1_000_000)

    def test_slerp_uses_shortest_path_and_preserves_tilt(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            init_path = root / "init.csv"
            self.write_init(init_path, [self.make_init_row()])
            euroc = root / "euroc"
            self.write_euroc(euroc)

            gt_path = (
                euroc
                / "mav0"
                / "state_groundtruth_estimate0"
                / "data.csv"
            )
            with gt_path.open("r", encoding="utf-8", newline="") as input_file:
                rows = list(csv.reader(input_file))
            half = math.sqrt(0.5)
            quaternions = (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (half, half, 0.0, 0.0),
                (-half, -half, 0.0, 0.0),
            )
            for row, quaternion in zip(rows[1:], quaternions):
                row[4:8] = [str(value) for value in quaternion]
            with gt_path.open("w", encoding="utf-8", newline="") as output:
                csv.writer(output).writerows(rows)

            result = analyze(load_init(init_path), euroc)

        self.assertAlmostEqual(result.gravity_angle_deg, 90.0)
        self.assertAlmostEqual(result.gt_window_rotation_deg, 0.0)

    def test_rejects_raw_imu_statistics_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            row = self.make_init_row()
            row["gyro_mean_x"] = "0.021"
            init_path = root / "init.csv"
            self.write_init(init_path, [row])
            euroc = root / "euroc"
            self.write_euroc(euroc)

            with self.assertRaisesRegex(ProbeError, "raw IMU gyro mean"):
                analyze(load_init(init_path), euroc)


if __name__ == "__main__":
    unittest.main()

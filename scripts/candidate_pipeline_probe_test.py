#!/usr/bin/env python3
"""Unit tests for candidate_pipeline_probe.py (P2b structural semantics)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from candidate_pipeline_probe import (
    HEADER,
    PROD_DIAG_HEADER,
    ProbeError,
    analyze,
    main,
)


class CandidatePipelineProbeTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def write_meta(self, **overrides: object) -> None:
        meta = {
            "ownership": "full_pipeline",
            "frames": 3,
            "ok": 3,
            "rejected": 0,
            "failed": 0,
            "keyframes": 1,
            "track_only_frames": 2,
            "terminal": 0,
            "wall_s": 0.125,
            "config": {"estimator.window_size": 10},
        }
        meta.update(overrides)
        (self.run_dir / "candidate").mkdir(parents=True, exist_ok=True)
        (self.run_dir / "candidate" / "meta.json").write_text(
            json.dumps(meta) + "\n"
        )

    def write_diag(self, rows: list[dict[str, object]],
                   directory: str = "candidate",
                   header: tuple[str, ...] = HEADER) -> None:
        (self.run_dir / directory).mkdir(parents=True, exist_ok=True)
        path = self.run_dir / directory / "diag.csv"
        with path.open("w", newline="") as stream:
            stream.write(",".join(header) + "\n")
            for row in rows:
                stream.write(
                    ",".join(str(row[field]) for field in header) + "\n"
                )

    def write_tum(self, name: str, content: str,
                  directory: str = "candidate") -> None:
        (self.run_dir / directory).mkdir(parents=True, exist_ok=True)
        (self.run_dir / directory / name).write_text(content)

    def write_run_meta(self, window: int = 10) -> None:
        (self.run_dir / "meta.json").write_text(
            json.dumps({"config": {"estimator.window_size": window}}) + "\n"
        )

    def cand_row(self, frame: int, ts_ns: int, **overrides: object) -> dict:
        row: dict[str, object] = {
            "frame_index": frame,
            "ts_ns": ts_ns,
            "status": "ok",
            "message": "accepted",
            "selected_keyframe": 1 if frame == 1 else 0,
            "keyframe_rule": 1 if frame == 1 else 0,
            "epoch_committed": 1 if frame == 1 else 0,
            "keyframe_epoch": 1 if frame == 1 else 0,
            "segment_id": 0,
            "reset_reason": 1 if frame == 0 else 0,
            "track_count": 20,
            "observation_count": 20,
            "disparity_count": 18,
            "shared_count": 16,
            "landmark_count": 15,
            "culled_count": 0,
            "dropped_track_count": 0,
            "fusion_mode": 1,
            "state_valid": 1,
            "state_tx": -0.1 * frame,
            "state_ty": 0.0,
            "state_tz": 0.0,
            "state_qw": 1.0,
            "state_qx": 0.0,
            "state_qy": 0.0,
            "state_qz": 0.0,
            "bias_gyro_x": 0.0,
            "bias_gyro_y": 0.0,
            "bias_gyro_z": 0.0,
            "graph_valid": 1,
            "graph_active_pose_count": 1,
            "graph_active_factor_count": 4,
            "graph_marginalized_pose_count": 0,
            "graph_retired_landmark_count": 0,
        }
        row.update(overrides)
        return row

    def prod_row(self, frame: int, ts_ns: int, **overrides: object) -> dict:
        row: dict[str, object] = {
            "timestamp_ns": ts_ns,
            "status": "ok",
            "num_obs": 20,
            "num_landmarks": 15,
            "num_shared": 16,
            "low_connectivity": 0,
            "window_size": 2,
            "prior_key": 0,
            "reproj_rms_before_px": 0.1,
            "reproj_rms_after_px": 0.05,
            "num_cheirality": 0,
            "lm_iterations": 3,
            "max_window_pose_shift_m": 0.0,
            "segment_id": 0,
            "pnp_success": 1,
            "pnp_inliers": 20,
            "outliers_culled": 0,
            "reproj_rms_after_cull_px": 0.05,
            "is_keyframe": 1 if frame == 1 else 0,
            "num_disparity": 18,
            "bias_gyro_x": 0.0,
            "bias_gyro_y": 0.0,
            "bias_gyro_z": 0.0,
            "bias_acc_x": 0.0,
            "bias_acc_y": 0.0,
            "bias_acc_z": 0.0,
        }
        row.update(overrides)
        return row

    def write_healthy_run(self) -> None:
        self.write_run_meta()
        self.write_meta()
        rows = [self.cand_row(i, 100 + i * 100) for i in range(3)]
        self.write_diag(rows)
        self.write_diag([self.prod_row(i, 100 + i * 100) for i in range(3)],
                        directory=".", header=PROD_DIAG_HEADER)
        tum = "1 0 0 0 0 0 0 1\n" * 3
        self.write_tum("est.tum", tum)
        self.write_tum("est.tum", tum, directory=".")
        self.write_tum("kf.tum", tum[: len(tum) // 3])

    def test_healthy_run_passes(self) -> None:
        self.write_healthy_run()
        analysis = analyze(self.run_dir)
        self.assertEqual(analysis.frames, 3)
        self.assertEqual(analysis.ok, 3)
        self.assertEqual(analysis.est_pose_count, 3)
        self.assertTrue(analysis.est_finite)
        self.assertEqual(analysis.graph_rows, 3)
        self.assertEqual(analysis.max_active_poses, 1)

    def test_missing_candidate_dir_fails(self) -> None:
        with self.assertRaises(ProbeError):
            analyze(self.run_dir)

    def test_wrong_ownership_fails(self) -> None:
        self.write_meta(ownership="partial")
        with self.assertRaisesRegex(ProbeError, "ownership"):
            analyze(self.run_dir)

    def test_terminal_run_fails(self) -> None:
        self.write_meta(terminal=1, terminal_code=1,
                        terminal_detail="non-gap IMU interval")
        with self.assertRaisesRegex(ProbeError, "terminal"):
            analyze(self.run_dir)

    def test_failed_frames_fail(self) -> None:
        self.write_meta(failed=2)
        with self.assertRaisesRegex(ProbeError, "failed"):
            analyze(self.run_dir)

    def test_diag_row_count_mismatch_fails(self) -> None:
        self.write_meta(frames=5)
        self.write_diag([self.cand_row(0, 100)])
        with self.assertRaisesRegex(ProbeError, "row count"):
            analyze(self.run_dir)

    def test_diag_schema_mismatch_fails(self) -> None:
        self.write_run_meta()
        self.write_meta()
        self.write_diag([self.cand_row(0, 100)], header=HEADER[:-1])
        with self.assertRaisesRegex(ProbeError, "schema"):
            analyze(self.run_dir)

    def test_frame_ts_mapping_mismatch_fails(self) -> None:
        self.write_run_meta()
        self.write_meta()
        self.write_diag([self.cand_row(i, 100 + i * 100) for i in range(3)])
        prod_rows = [self.prod_row(i, 100 + i * 100) for i in range(3)]
        prod_rows[1]["timestamp_ns"] = 999
        self.write_diag(prod_rows, directory=".", header=PROD_DIAG_HEADER)
        with self.assertRaisesRegex(ProbeError, "row 1"):
            analyze(self.run_dir)

    def test_status_allowed_to_differ_from_production(self) -> None:
        # P2b: status/keyframe/bias 由 candidate 拥有，允许与 production
        # 不同；帧对齐（ts）必须一致。
        self.write_run_meta()
        self.write_meta(ok=2, rejected=1)
        cand_rows = [self.cand_row(i, 100 + i * 100) for i in range(3)]
        cand_rows[2]["status"] = "rejected"
        cand_rows[2]["state_valid"] = 0
        self.write_diag(cand_rows)
        prod_rows = [self.prod_row(i, 100 + i * 100) for i in range(3)]
        prod_rows[2]["status"] = "ok"
        prod_rows[2]["is_keyframe"] = 0
        prod_rows[2]["bias_gyro_z"] = 0.5
        self.write_diag(prod_rows, directory=".", header=PROD_DIAG_HEADER)
        tum = "1 0 0 0 0 0 0 1\n" * 2
        self.write_tum("est.tum", tum)
        self.write_tum("est.tum", tum, directory=".")
        self.write_tum("kf.tum", tum[: len(tum) // 2])
        analysis = analyze(self.run_dir)
        self.assertEqual(analysis.ok, 2)
        self.assertEqual(analysis.est_pose_count, 2)

    def test_graph_active_pose_bounds_fail(self) -> None:
        self.write_run_meta()
        self.write_meta()
        rows = [self.cand_row(i, 100 + i * 100) for i in range(3)]
        rows[1]["graph_active_pose_count"] = 99
        self.write_diag(rows)
        self.write_diag([self.prod_row(i, 100 + i * 100) for i in range(3)],
                        directory=".", header=PROD_DIAG_HEADER)
        self.write_tum("est.tum", "1 0 0 0 0 0 0 1\n" * 3)
        with self.assertRaisesRegex(ProbeError, "window"):
            analyze(self.run_dir)

    def test_est_row_count_mismatch_fails(self) -> None:
        self.write_run_meta()
        self.write_meta()
        self.write_diag([self.cand_row(i, 100 + i * 100) for i in range(3)])
        self.write_diag([self.prod_row(i, 100 + i * 100) for i in range(3)],
                        directory=".", header=PROD_DIAG_HEADER)
        self.write_tum("est.tum", "1 0 0 0 0 0 0 1\n")  # 1 行 != ok 3
        with self.assertRaisesRegex(ProbeError, "est.tum"):
            analyze(self.run_dir)

    def test_est_nonfinite_fails(self) -> None:
        self.write_run_meta()
        self.write_meta()
        self.write_diag([self.cand_row(i, 100 + i * 100) for i in range(3)])
        self.write_diag([self.prod_row(i, 100 + i * 100) for i in range(3)],
                        directory=".", header=PROD_DIAG_HEADER)
        tum = "1 0 0 0 0 0 0 1\n"
        self.write_tum("est.tum", tum + "nan 0 0 0 0 0 0 1\n" + tum)
        with self.assertRaisesRegex(ProbeError, "finite"):
            analyze(self.run_dir)

    def test_missing_kf_tum_fails(self) -> None:
        self.write_run_meta()
        self.write_meta()
        self.write_diag([self.cand_row(i, 100 + i * 100) for i in range(3)])
        self.write_diag([self.prod_row(i, 100 + i * 100) for i in range(3)],
                        directory=".", header=PROD_DIAG_HEADER)
        self.write_tum("est.tum", "1 0 0 0 0 0 0 1\n" * 3)
        with self.assertRaisesRegex(ProbeError, "kf.tum"):
            analyze(self.run_dir)

    def test_main_returns_zero_on_healthy_run(self) -> None:
        self.write_healthy_run()
        self.assertEqual(main([str(self.run_dir)]), 0)

    def test_main_returns_one_on_failure(self) -> None:
        self.assertEqual(main([str(self.run_dir)]), 1)


if __name__ == "__main__":
    unittest.main()

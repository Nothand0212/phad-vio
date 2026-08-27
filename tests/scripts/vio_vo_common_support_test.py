import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "vio_vo_common_support.py"


def load_module():
    spec = importlib.util.spec_from_file_location("vio_vo_common_support", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stamp(ts_ns):
    return f"{ts_ns // 1_000_000_000}.{ts_ns % 1_000_000_000:09d}"


def write_tum(path, poses):
    lines = []
    for ts_ns, xyz in poses:
        lines.append(
            f"{stamp(ts_ns)} {xyz[0]} {xyz[1]} {xyz[2]} 0 0 0 1"
        )
    path.write_text("\n".join(lines) + "\n")


def write_summary(path, poses):
    path.write_text(
        json.dumps(
            {
                "trajectory": {
                    "poses_written": poses,
                    "completion_rate": 1.0,
                    "coverage_rate": 1.0,
                    "failed": 0,
                }
            }
        )
    )


class CommonSupportTest(unittest.TestCase):
    def test_exact_common_support_and_rpe_edges_are_auditable(self):
        module = load_module()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            off = root / "off"
            fused = root / "fused"
            gt_root = root / "gt"
            off.mkdir()
            fused.mkdir()
            gt_dir = gt_root / "mav0" / "state_groundtruth_estimate0"
            gt_dir.mkdir(parents=True)

            second = 1_000_000_000
            gt_poses = {
                10 * second: (0.0, 0.0, 0.0),
                11 * second: (1.0, 0.0, 0.0),
                12 * second: (1.0, 1.0, 0.0),
                13 * second: (0.5, 1.5, 0.0),
                14 * second: (0.0, 2.0, 0.0),
                15 * second: (-1.0, 2.0, 0.0),
            }
            off_ts = [10 * second, 11 * second, 12 * second, 13 * second,
                      14 * second]
            fused_ts = [10 * second, 11 * second, 12 * second, 14 * second,
                        15 * second]
            off_poses = []
            for index, ts_ns in enumerate(off_ts):
                xyz = list(gt_poses[ts_ns])
                xyz[1] += 0.08 * (index % 2)
                off_poses.append((ts_ns, tuple(xyz)))
            fused_poses = [(ts_ns, gt_poses[ts_ns]) for ts_ns in fused_ts]
            write_tum(off / "est.tum", off_poses)
            write_tum(fused / "est.tum", fused_poses)
            write_tum(off / "kf.tum", [off_poses[i] for i in (0, 2, 4)])
            write_tum(fused / "kf.tum",
                      [fused_poses[i] for i in (0, 1, 3)])
            write_summary(off / "summary.json", len(off_poses))
            write_summary(fused / "summary.json", len(fused_poses))
            (fused / "gyro_state.csv").write_text(
                "ts_ns,align_ready,gyro_factor_count,gyro_post_valid\n"
                f"{10 * second},0,0,0\n"
                f"{11 * second},1,0,0\n"
                f"{12 * second},1,1,1\n"
                f"{14 * second},1,1,1\n"
                f"{15 * second},1,1,1\n"
            )

            header = (
                "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], "
                "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], "
                "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], "
                "v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], "
                "b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], "
                "b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], "
                "b_a_RS_S_z [m s^-2]"
            )
            rows = [header]
            for ts_ns, xyz in gt_poses.items():
                rows.append(
                    f"{ts_ns},{xyz[0]},{xyz[1]},{xyz[2]},1,0,0,0,"
                    "0,0,0,0,0,0,0,0,0"
                )
            (gt_dir / "data.csv").write_text("\n".join(rows) + "\n")

            report = module.evaluate_runs(off, fused, gt_root)

            self.assertEqual(report["support"]["off_poses"], 5)
            self.assertEqual(report["support"]["fused_poses"], 5)
            self.assertEqual(report["support"]["exact_common"], 4)
            self.assertEqual(report["support"]["gt_matched"], 4)
            self.assertEqual(report["rpe"]["pair_count"], 2)
            self.assertEqual(report["rpe"]["dropped_no_partner"], 2)
            self.assertLess(
                report["ate_m"]["fused"]["rmse"],
                report["ate_m"]["off"]["rmse"],
            )
            self.assertLess(
                report["rpe_m"]["fused"]["rmse"],
                report["rpe_m"]["off"]["rmse"],
            )
            self.assertEqual(
                report["keyframes"]["first_divergence_index"], 1
            )
            self.assertEqual(report["factors"]["posterior_rows"], 3)
            self.assertTrue(report["gate"]["core_pass"])


if __name__ == "__main__":
    unittest.main()

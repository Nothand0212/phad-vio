import csv
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = REPO_ROOT / "scripts" / "validate_m5_initial_root.py"
MH_CONTROL_COMMIT = "ba607c5738b3c5d9bc9cb0423199eee67648b0b3"
MH_CONTROL_CONFIG_HASH = "0337287b"
MH_CONTROL_CANONICAL_TEXT = (
    "estimator.block_culled_rebirth=true\n"
    "estimator.enable_moving_bootstrap=true\n"
    "estimator.enable_outlier_cull=true\n"
    "estimator.enable_outlier_reopt=true\n"
    "estimator.enable_pnp_init=true\n"
    "estimator.far_return_refresh_px=6\n"
    "estimator.hanging_landmark_gate_m=1\n"
    "estimator.huber_k_px=3\n"
    "estimator.max_outlier_reopts=3\n"
    "estimator.min_landmark_observations=2\n"
    "estimator.min_pnp_inliers=10\n"
    "estimator.min_seed_observations=10\n"
    "estimator.min_shared_landmarks=10\n"
    "estimator.min_track_observations_for_seed=1\n"
    "estimator.outlier_avg_reproj_px=4\n"
    "estimator.pnp_confidence=0.98999999999999999\n"
    "estimator.pnp_reproj_px=2\n"
    "estimator.prior_rotation_sigma_rad=0.0001\n"
    "estimator.prior_translation_sigma_m=0.0001\n"
    "estimator.stereo_sigma_px=1\n"
    "estimator.velocity_prior_sigma_mps=0.10000000000000001\n"
    "estimator.window_size=10\n"
    "eval.max_dt_ms=2.5\n"
    "eval.min_match_rate=0.5\n"
    "eval.rpe_delta_s=1\n"
    "session.dataset_format=euroc\n"
    "session.drop_culled_tracks=true\n"
    "session.skip_drop_min_culled=4\n"
    "session.zombie_drop_age=5\n"
    "tracker.forward_backward_px=0.5\n"
    "tracker.lk_pyramid_levels=4\n"
    "tracker.lk_window_px=21\n"
    "tracker.mask_radius_px=20\n"
    "tracker.max_depth_m=25\n"
    "tracker.max_epipolar_px=1.5\n"
    "tracker.max_tracks=200\n"
    "tracker.min_depth_m=0.29999999999999999\n"
    "tracker.min_disparity_px=2\n"
    "tracker.min_distance_px=20\n"
    "tracker.quality_level=0.0030000000000000001\n"
    "tracker.stereo_bidir_px=0.5\n"
    "tracker.stereo_check_bidir=true\n"
    "tracker.stereo_row_tol_px=0\n"
    "tracker.stereo_sad_half_win_px=7\n"
    "tracker.stereo_uniq_ratio=0.5\n"
)


def fnv1a_hash8(text: str) -> str:
    value = 14_695_981_039_346_656_037
    for byte in text.encode():
        value ^= byte
        value = (value * 1_099_511_628_211) & 0xFFFF_FFFF_FFFF_FFFF
    return f"{value:016x}"[:8]


class ValidateM5InitialRootTest(unittest.TestCase):
    def make_v1_fixture(self, root: Path) -> tuple[Path, Path, str]:
        sequence_root = root / "sequence"
        sequence_root.mkdir()
        payload = sequence_root / "dummy.bin"
        payload.write_bytes(b"m5-v1-fixture")
        manifest_text = (
            f"{hashlib.sha256(payload.read_bytes()).hexdigest()}  ./dummy.bin\n"
        )
        manifest = root / "manifest.sha256"
        manifest.write_text(manifest_text, encoding="utf-8")
        manifest_digest = hashlib.sha256(manifest_text.encode()).hexdigest()

        run = root / "run"
        run.mkdir()
        config = {
            "estimator.enable_moving_bootstrap": True,
            "eval.max_dt_ms": 2.5,
            "eval.min_match_rate": 0.5,
            "eval.rpe_delta_s": 1.0,
            "session.max_frames": 118,
        }
        def canonical_value(value: object) -> str:
            if isinstance(value, bool):
                return str(value).lower()
            if isinstance(value, float):
                return format(value, ".17g")
            return str(value)

        canonical_text = "".join(
            f"{key}={canonical_value(config[key])}\n" for key in sorted(config)
        )
        fnv = 14_695_981_039_346_656_037
        for byte in canonical_text.encode():
            fnv ^= byte
            fnv = (fnv * 1_099_511_628_211) & 0xFFFF_FFFF_FFFF_FFFF
        config_hash = f"{fnv:016x}"[:8]
        (run / "meta.json").write_text(
            json.dumps(
                {
                    "sequence": "V1_02_medium",
                    "sequence_root": str(sequence_root),
                    "config": config,
                    "config_canonical_text": canonical_text,
                    "config_hash": config_hash,
                    "code": {"git_commit_short": "abcdef0", "git_dirty": True},
                }
            ),
            encoding="utf-8",
        )
        ok_count = 113
        summary = {
            "status": "completed",
            "sequence": "V1_02_medium",
            "config_hash": config_hash,
            "code": {"git_commit_short": "abcdef0", "git_dirty": True},
            "trajectory": {
                "image_frames": 118,
                "poses_written": ok_count,
                "ok": ok_count,
                "rejected": 0,
                "failed": 0,
                "completion_rate": ok_count / 118.0,
                "coverage_rate": 0.95,
                "segments": 1,
            },
            "sync": {"emitted_stereo": 118},
            "robustness": {"rejected": 0, "failed": 0, "reanchors": 0},
            "ate": {"trans": {"rmse": 0.5}},
            "rpe": {"trans": {"rmse": 0.4}},
        }
        (run / "summary.json").write_text(
            json.dumps(summary), encoding="utf-8"
        )

        timestamps = [50_000_000 * index for index in range(118)]
        with (run / "diag.csv").open("w", newline="", encoding="utf-8") as out:
            writer = csv.DictWriter(
                out, fieldnames=["status", "unused", "timestamp_ns"]
            )
            writer.writeheader()
            for index, timestamp_ns in enumerate(timestamps):
                writer.writerow(
                    {
                        "status": "initializing" if index < 5 else "ok",
                        "unused": index,
                        "timestamp_ns": timestamp_ns,
                    }
                )
        with (run / "est.tum").open("w", encoding="utf-8") as out:
            for timestamp_ns in timestamps[5:]:
                out.write(
                    f"{timestamp_ns / 1e9:.9f} 0 0 0 0 0 0 1\n"
                )
        return run, manifest, manifest_digest

    def make_mh_fixture(self, root: Path) -> tuple[Path, Path, Path]:
        sequence_root = root / "sequence"
        sequence_root.mkdir()
        payload = sequence_root / "dummy.bin"
        payload.write_bytes(b"m5-mh-fixture")
        manifest = root / "manifest.sha256"
        manifest.write_text(
            f"{hashlib.sha256(payload.read_bytes()).hexdigest()}  ./dummy.bin\n",
            encoding="utf-8",
        )

        def write_run(name: str, *, control: bool) -> Path:
            run = root / name
            run.mkdir()
            config = {
                "estimator.enable_moving_bootstrap": True,
                "eval.max_dt_ms": 2.5,
                "eval.min_match_rate": 0.5,
                "eval.rpe_delta_s": 1.0,
            }
            canonical_text = MH_CONTROL_CANONICAL_TEXT
            config_hash = fnv1a_hash8(canonical_text)
            if config_hash != MH_CONTROL_CONFIG_HASH:
                raise AssertionError(
                    f"control config fixture hash {config_hash} is not "
                    f"{MH_CONTROL_CONFIG_HASH}"
                )
            git_commit = MH_CONTROL_COMMIT if control else "abcdef0" * 5 + "abcde"
            git_commit_short = "ba607c5" if control else "abcdef0"
            git_dirty = not control
            (run / "meta.json").write_text(
                json.dumps(
                    {
                        "sequence": "MH_01_easy",
                        "sequence_root": str(sequence_root),
                        "config": config,
                        "config_canonical_text": canonical_text,
                        "config_hash": config_hash,
                        "code": {
                            "git_commit": git_commit,
                            "git_commit_short": git_commit_short,
                            "git_dirty": git_dirty,
                        },
                    }
                ),
                encoding="utf-8",
            )
            summary = {
                "status": "completed",
                "sequence": "MH_01_easy",
                "config_hash": config_hash,
                "code": {
                    "git_commit_short": git_commit_short,
                    "git_dirty": git_dirty,
                },
                "trajectory": {
                    "image_frames": 4,
                    "poses_written": 3,
                    "ok": 3,
                    "rejected": 0,
                    "failed": 0,
                    "completion_rate": 0.75,
                    "coverage_rate": 0.8,
                    "segments": 1,
                },
                "sync": {"emitted_stereo": 4},
                "robustness": {"rejected": 0, "failed": 0, "reanchors": 0},
                "ate": {"trans": {"rmse": 0.1}},
                "rpe": {"trans": {"rmse": 0.2}},
            }
            if not control:
                summary["trajectory"] = {
                    "image_frames": 7,
                    "poses_written": 1,
                    "ok": 1,
                    "rejected": 0,
                    "failed": 0,
                    "completion_rate": 1.0 / 7.0,
                    "coverage_rate": 0.0,
                    "segments": 1,
                }
                summary["sync"] = {"emitted_stereo": 7}
                summary["robustness"] = {"rejected": 0, "failed": 0, "reanchors": 0}
            (run / "summary.json").write_text(
                json.dumps(summary), encoding="utf-8"
            )
            timestamps = (
                [
                    0,
                    50_000_000,
                    100_000_000,
                    150_000_000,
                    200_000_000,
                    250_000_000,
                    300_000_000,
                ]
                if not control
                else [0, 50_000_000, 100_000_000, 150_000_000]
            )
            with (run / "diag.csv").open(
                "w", newline="", encoding="utf-8"
            ) as out:
                writer = csv.DictWriter(
                    out, fieldnames=["timestamp_ns", "status"]
                )
                writer.writeheader()
                for index, timestamp_ns in enumerate(timestamps):
                    writer.writerow(
                        {
                            "timestamp_ns": timestamp_ns,
                            "status": (
                                "initializing"
                                if (not control and index < 6)
                                else "initializing"
                                if control and index == 0
                                else "ok"
                            ),
                        }
                    )
            with (run / "est.tum").open("w", encoding="utf-8") as out:
                start = 6 if not control else 1
                for timestamp_ns in timestamps[start:]:
                    out.write(f"{timestamp_ns / 1e9:.9f} 0 0 0 0 0 0 1\n")
            return run

        return (
            write_run("candidate", control=False),
            write_run("control", control=True),
            manifest,
        )

    def run_validator(self, *args: object) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(VALIDATOR), *(str(arg) for arg in args)],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_v1_healthy_run_passes_by_header_name(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run, manifest, digest = self.make_v1_fixture(Path(temporary))
            completed = self.run_validator(
                "v102-118",
                run,
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                digest,
            )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        verdict = json.loads(completed.stdout)
        self.assertEqual(verdict["mode"], "v102-118")
        self.assertTrue(verdict["passed"])

    def test_v1_manifest_digest_rejects_before_run_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / "manifest.sha256"
            manifest.write_text("not the registered manifest\n", encoding="utf-8")
            completed = self.run_validator(
                "v102-118",
                root / "missing-run",
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                "0" * 64,
            )

        self.assertEqual(completed.returncode, 2)
        self.assertIn("manifest digest mismatch", completed.stderr)
        self.assertNotIn("summary.json", completed.stderr)

    def test_mh_static_healthy_candidate_matches_control(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, control, manifest = self.make_mh_fixture(Path(temporary))
            completed = self.run_validator(
                "mh01-static",
                candidate,
                "--control-dir",
                control,
                "--comparison-tolerance-m",
                "0.000001",
                "--manifest-sha256",
                manifest,
            )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        verdict = json.loads(completed.stdout)
        self.assertEqual(verdict["mode"], "mh01-static")
        self.assertTrue(verdict["passed"])
        self.assertEqual(verdict["candidate_first_ok_index"], 6)
        self.assertEqual(verdict["candidate_first_ok_span_ns"], 300_000_000)
        self.assertEqual(verdict["control_first_ok_index"], 1)
        self.assertLess(
            verdict["candidate"]["completion"],
            verdict["control"]["completion"],
        )
        self.assertLess(
            verdict["candidate"]["coverage"],
            verdict["control"]["coverage"],
        )

    def test_mh_static_candidate_fixture_has_formal_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, _control, _manifest = self.make_mh_fixture(Path(temporary))
            with (candidate / "diag.csv").open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            first_ok = next(index for index, row in enumerate(rows) if row["status"] == "ok")
            self.assertEqual(first_ok, 6, "candidate must first commit at index 6")
            first_ok_ns = int(rows[6]["timestamp_ns"])
            first_ns = int(rows[0]["timestamp_ns"])
            self.assertEqual(first_ok_ns - first_ns, 300_000_000)
            summary = json.loads((candidate / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["trajectory"]["image_frames"], 7)
            self.assertLess(summary["trajectory"]["completion_rate"], 0.75)
            self.assertLess(summary["trajectory"]["coverage_rate"], 0.8)
            self.assertLess(summary["trajectory"]["completion_rate"], 1.0)
            self.assertLess(summary["trajectory"]["coverage_rate"], 1.0)

    def test_mh_static_requires_frozen_comparison_tolerance(self) -> None:
        for tolerance in ("0", "0.000002", "1"):
            with self.subTest(tolerance=tolerance), tempfile.TemporaryDirectory() as temporary:
                candidate, control, manifest = self.make_mh_fixture(Path(temporary))
                completed = self.run_validator(
                    "mh01-static",
                    candidate,
                    "--control-dir",
                    control,
                    "--comparison-tolerance-m",
                    tolerance,
                    "--manifest-sha256",
                    manifest,
                )
                self.assertEqual(completed.returncode, 2, completed.stderr)
                self.assertIn("comparison tolerance", completed.stderr)

    def test_mh_static_control_cadence_is_record_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, control, manifest = self.make_mh_fixture(Path(temporary))
            with (control / "diag.csv").open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            for index, row in enumerate(rows):
                row["status"] = "initializing" if index < 2 else "ok"
            with (control / "diag.csv").open(
                "w", newline="", encoding="utf-8"
            ) as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            with (control / "est.tum").open("w", encoding="utf-8") as out:
                for row in rows[2:]:
                    out.write(f"{int(row['timestamp_ns']) / 1e9:.9f} 0 0 0 0 0 0 1\n")
            summary_path = control / "summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            summary["trajectory"]["poses_written"] = 2
            summary["trajectory"]["ok"] = 2
            summary["trajectory"]["completion_rate"] = 0.5
            summary["trajectory"]["coverage_rate"] = 1.0 / 3.0
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            completed = self.run_validator(
                "mh01-static",
                candidate,
                "--control-dir",
                control,
                "--comparison-tolerance-m",
                "0.000001",
                "--manifest-sha256",
                manifest,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            verdict = json.loads(completed.stdout)
            self.assertEqual(verdict["control_first_ok_index"], 2)
            self.assertEqual(verdict["control_first_ok_span_ns"], 100_000_000)

    def test_mh_record_only_rates_must_be_finite(self) -> None:
        mutations = (
            ("candidate", "completion_rate", float("nan")),
            ("candidate", "coverage_rate", float("inf")),
            ("control", "completion_rate", float("nan")),
            ("control", "coverage_rate", float("inf")),
        )
        for run_name, key, value in mutations:
            with (
                self.subTest(run=run_name, key=key),
                tempfile.TemporaryDirectory() as temporary,
            ):
                candidate, control, manifest = self.make_mh_fixture(Path(temporary))
                run = candidate if run_name == "candidate" else control
                summary_path = run / "summary.json"
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                summary["trajectory"][key] = value
                summary_path.write_text(json.dumps(summary), encoding="utf-8")
                completed = self.run_validator(
                    "mh01-static",
                    candidate,
                    "--control-dir",
                    control,
                    "--comparison-tolerance-m",
                    "0.000001",
                    "--manifest-sha256",
                    manifest,
                )
                self.assertEqual(completed.returncode, 2, completed.stderr)
                self.assertIn("must be finite", completed.stderr)

    def test_mh_precision_tolerance_boundary_is_inclusive(self) -> None:
        boundaries = {
            "ate": 0.100001,
            "rpe": 0.200001,
        }
        for metric, value in boundaries.items():
            with self.subTest(metric=metric), tempfile.TemporaryDirectory() as temporary:
                candidate, control, manifest = self.make_mh_fixture(Path(temporary))
                summary_path = candidate / "summary.json"
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                summary[metric]["trans"]["rmse"] = value
                summary_path.write_text(json.dumps(summary), encoding="utf-8")
                completed = self.run_validator(
                    "mh01-static",
                    candidate,
                    "--control-dir",
                    control,
                    "--comparison-tolerance-m",
                    "0.000001",
                    "--manifest-sha256",
                    manifest,
                )
                self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_mh_static_rejects_early_first_ok_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, control, manifest = self.make_mh_fixture(Path(temporary))
            with (candidate / "diag.csv").open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            for index, row in enumerate(rows):
                row["status"] = "initializing" if index < 5 else "ok"
            with (candidate / "diag.csv").open(
                "w", newline="", encoding="utf-8"
            ) as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            with (candidate / "est.tum").open("w", encoding="utf-8") as out:
                for row in rows[5:]:
                    out.write(f"{int(row['timestamp_ns']) / 1e9:.9f} 0 0 0 0 0 0 1\n")
            summary = json.loads((candidate / "summary.json").read_text(encoding="utf-8"))
            summary["trajectory"]["poses_written"] = 2
            summary["trajectory"]["ok"] = 2
            summary["trajectory"]["completion_rate"] = 2.0 / 7.0
            summary["trajectory"]["coverage_rate"] = 1.0 / 6.0
            (candidate / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
            completed = self.run_validator(
                "mh01-static",
                candidate,
                "--control-dir",
                control,
                "--comparison-tolerance-m",
                "0.000001",
                "--manifest-sha256",
                manifest,
            )
            self.assertEqual(completed.returncode, 1, completed.stderr)
            self.assertIn("candidate first ok index", completed.stderr)

    def test_mh_static_rejects_wrong_span_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, control, manifest = self.make_mh_fixture(Path(temporary))
            with (candidate / "diag.csv").open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            rows[6]["timestamp_ns"] = "299999999"
            with (candidate / "diag.csv").open(
                "w", newline="", encoding="utf-8"
            ) as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            with (candidate / "est.tum").open("w", encoding="utf-8") as out:
                row = rows[6]
                out.write(f"{int(row['timestamp_ns']) / 1e9:.9f} 0 0 0 0 0 0 1\n")
            completed = self.run_validator(
                "mh01-static",
                candidate,
                "--control-dir",
                control,
                "--comparison-tolerance-m",
                "0.000001",
                "--manifest-sha256",
                manifest,
            )
            self.assertEqual(completed.returncode, 1, completed.stderr)
            self.assertIn("candidate first ok span ns", completed.stderr)

    def test_mh_static_requires_exact_control_identity(self) -> None:
        mutations = ("commit", "dirty", "config")
        for mutation in mutations:
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temporary:
                candidate, control, manifest = self.make_mh_fixture(
                    Path(temporary)
                )
                control_meta_path = control / "meta.json"
                control_meta = json.loads(
                    control_meta_path.read_text(encoding="utf-8")
                )
                if mutation == "commit":
                    control_meta["code"]["git_commit"] = "0" * 40
                    control_meta_path.write_text(
                        json.dumps(control_meta), encoding="utf-8"
                    )
                    candidate_summary_path = candidate / "summary.json"
                    candidate_summary = json.loads(
                        candidate_summary_path.read_text(encoding="utf-8")
                    )
                    candidate_summary["ate"]["trans"]["rmse"] = float("nan")
                    candidate_summary_path.write_text(
                        json.dumps(candidate_summary), encoding="utf-8"
                    )
                    expected = "control meta.code.git_commit"
                elif mutation == "dirty":
                    control_meta["code"]["git_dirty"] = True
                    control_meta_path.write_text(
                        json.dumps(control_meta), encoding="utf-8"
                    )
                    control_summary_path = control / "summary.json"
                    control_summary = json.loads(
                        control_summary_path.read_text(encoding="utf-8")
                    )
                    control_summary["code"]["git_dirty"] = True
                    control_summary_path.write_text(
                        json.dumps(control_summary), encoding="utf-8"
                    )
                    expected = "control meta.code.git_dirty"
                else:
                    for run in (candidate, control):
                        meta_path = run / "meta.json"
                        summary_path = run / "summary.json"
                        meta = json.loads(meta_path.read_text(encoding="utf-8"))
                        summary = json.loads(
                            summary_path.read_text(encoding="utf-8")
                        )
                        canonical_text = (
                            meta["config_canonical_text"]
                            + "test.control_identity=wrong\n"
                        )
                        config_hash = fnv1a_hash8(canonical_text)
                        meta["config_canonical_text"] = canonical_text
                        meta["config_hash"] = config_hash
                        summary["config_hash"] = config_hash
                        meta_path.write_text(json.dumps(meta), encoding="utf-8")
                        summary_path.write_text(
                            json.dumps(summary), encoding="utf-8"
                        )
                    expected = "control meta.config_hash"

                completed = self.run_validator(
                    "mh01-static",
                    candidate,
                    "--control-dir",
                    control,
                    "--comparison-tolerance-m",
                    "0.000001",
                    "--manifest-sha256",
                    manifest,
                )

                self.assertEqual(completed.returncode, 1, completed.stderr)
                self.assertIn(expected, completed.stderr)
                if mutation == "commit":
                    self.assertNotIn("must be finite", completed.stderr)

    def test_v1_rejects_invalid_artifact_inputs(self) -> None:
        def missing_key(run: Path) -> None:
            path = run / "summary.json"
            summary = json.loads(path.read_text(encoding="utf-8"))
            del summary["trajectory"]["failed"]
            path.write_text(json.dumps(summary), encoding="utf-8")

        def nonfinite_metric(run: Path) -> None:
            path = run / "summary.json"
            summary = json.loads(path.read_text(encoding="utf-8"))
            summary["ate"]["trans"]["rmse"] = float("nan")
            path.write_text(json.dumps(summary), encoding="utf-8")

        def duplicate_diag(run: Path) -> None:
            path = run / "diag.csv"
            with path.open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            rows[1]["timestamp_ns"] = rows[0]["timestamp_ns"]
            with path.open("w", newline="", encoding="utf-8") as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)

        def duplicate_tum(run: Path) -> None:
            path = run / "est.tum"
            lines = path.read_text(encoding="utf-8").splitlines()
            fields = lines[1].split()
            fields[0] = lines[0].split()[0]
            lines[1] = " ".join(fields)
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        mutations = {
            "missing-key": missing_key,
            "nonfinite": nonfinite_metric,
            "duplicate-diag": duplicate_diag,
            "duplicate-tum": duplicate_tum,
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                run, manifest, digest = self.make_v1_fixture(Path(temporary))
                mutate(run)
                completed = self.run_validator(
                    "v102-118",
                    run,
                    "--manifest-sha256",
                    manifest,
                    "--expected-manifest-digest",
                    digest,
                )
                self.assertEqual(completed.returncode, 2, completed.stderr)

    def test_v1_predicate_and_cadence_failures_report_first_difference(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run, manifest, digest = self.make_v1_fixture(Path(temporary))
            summary_path = run / "summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            summary["ate"]["trans"]["rmse"] = 1.1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            predicate = self.run_validator(
                "v102-118",
                run,
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                digest,
            )
        self.assertEqual(predicate.returncode, 1)
        self.assertIn("ate.trans.rmse", predicate.stderr)
        self.assertIn("expected", predicate.stderr)
        self.assertIn("actual", predicate.stderr)

        with tempfile.TemporaryDirectory() as temporary:
            run, manifest, digest = self.make_v1_fixture(Path(temporary))
            diag_path = run / "diag.csv"
            with diag_path.open(encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            rows[4]["status"] = "ok"
            with diag_path.open("w", newline="", encoding="utf-8") as out:
                writer = csv.DictWriter(out, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            tum_path = run / "est.tum"
            tum_path.write_text(
                "0.200000000 0 0 0 0 0 0 1\n"
                + tum_path.read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            summary_path = run / "summary.json"
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            summary["trajectory"]["poses_written"] = 114
            summary["trajectory"]["ok"] = 114
            summary["trajectory"]["completion_rate"] = 114 / 118.0
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            cadence = self.run_validator(
                "v102-118",
                run,
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                digest,
            )
        self.assertEqual(cadence.returncode, 1)
        self.assertIn("first ok index lower bound", cadence.stderr)

    def test_tum_timestamp_tolerance_is_inclusive_at_500_ns(self) -> None:
        for offset_ns, expected_code in ((500, 0), (501, 1)):
            with self.subTest(offset_ns=offset_ns), tempfile.TemporaryDirectory() as temporary:
                run, manifest, digest = self.make_v1_fixture(Path(temporary))
                tum_path = run / "est.tum"
                lines = tum_path.read_text(encoding="utf-8").splitlines()
                fields = lines[0].split()
                fields[0] = f"{(250_000_000 + offset_ns) / 1e9:.9f}"
                lines[0] = " ".join(fields)
                tum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                completed = self.run_validator(
                    "v102-118",
                    run,
                    "--manifest-sha256",
                    manifest,
                    "--expected-manifest-digest",
                    digest,
                )
                self.assertEqual(
                    completed.returncode, expected_code, completed.stderr
                )
                if expected_code == 1:
                    self.assertIn("trajectory timestamp[0]", completed.stderr)

    def test_mh_relative_predicate_failures_are_nonzero(self) -> None:
        mutations = {
            "ate": ("ate", "trans", "rmse", 0.100002),
            "rpe": ("rpe", "trans", "rmse", 0.200002),
        }
        for name, mutation in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                candidate, control, manifest = self.make_mh_fixture(Path(temporary))
                summary_path = candidate / "summary.json"
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                summary[mutation[0]][mutation[1]][mutation[2]] = mutation[3]
                summary_path.write_text(json.dumps(summary), encoding="utf-8")
                completed = self.run_validator(
                    "mh01-static",
                    candidate,
                    "--control-dir",
                    control,
                    "--comparison-tolerance-m",
                    "0.000001",
                    "--manifest-sha256",
                    manifest,
                )
                self.assertEqual(completed.returncode, 1, completed.stderr)
                self.assertIn(name, completed.stderr)

    def test_manifest_rechecks_files_under_meta_sequence_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run, manifest, digest = self.make_v1_fixture(Path(temporary))
            meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
            (Path(meta["sequence_root"]) / "dummy.bin").write_bytes(b"changed")
            completed = self.run_validator(
                "v102-118",
                run,
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                digest,
            )

        self.assertEqual(completed.returncode, 2)
        self.assertIn("manifest file mismatch", completed.stderr)
        self.assertIn("dummy.bin", completed.stderr)

    def test_v1_rejects_wrong_frozen_config_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run, manifest, digest = self.make_v1_fixture(Path(temporary))
            meta_path = run / "meta.json"
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta["config"]["estimator.enable_moving_bootstrap"] = False
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            completed = self.run_validator(
                "v102-118",
                run,
                "--manifest-sha256",
                manifest,
                "--expected-manifest-digest",
                digest,
            )

        self.assertEqual(completed.returncode, 1, completed.stderr)
        self.assertIn("estimator.enable_moving_bootstrap", completed.stderr)

    def test_meta_sequence_root_type_errors_are_controlled(self) -> None:
        for invalid in (None, ["sequence"], 7):
            with self.subTest(value=invalid), tempfile.TemporaryDirectory() as temporary:
                run, manifest, digest = self.make_v1_fixture(Path(temporary))
                meta_path = run / "meta.json"
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
                meta["sequence_root"] = invalid
                meta_path.write_text(json.dumps(meta), encoding="utf-8")
                completed = self.run_validator(
                    "v102-118",
                    run,
                    "--manifest-sha256",
                    manifest,
                    "--expected-manifest-digest",
                    digest,
                )
                self.assertEqual(completed.returncode, 2, completed.stderr)
                self.assertIn("sequence_root", completed.stderr)
                self.assertNotIn("Traceback", completed.stderr)

    def test_v1_rejects_max_frames_and_canonical_hash_mismatch(self) -> None:
        for mutation in ("max-frames", "hash"):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as temporary:
                run, manifest, digest = self.make_v1_fixture(Path(temporary))
                meta_path = run / "meta.json"
                summary_path = run / "summary.json"
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                if mutation == "max-frames":
                    meta["config"]["session.max_frames"] = 117
                else:
                    meta["config_hash"] = "deadbeef"
                    summary["config_hash"] = "deadbeef"
                meta_path.write_text(json.dumps(meta), encoding="utf-8")
                summary_path.write_text(json.dumps(summary), encoding="utf-8")
                completed = self.run_validator(
                    "v102-118",
                    run,
                    "--manifest-sha256",
                    manifest,
                    "--expected-manifest-digest",
                    digest,
                )
                self.assertEqual(completed.returncode, 1, completed.stderr)
                self.assertIn(
                    "session.max_frames" if mutation == "max-frames" else "config hash",
                    completed.stderr,
                )

    def test_mh_rejects_candidate_control_config_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            candidate, control, manifest = self.make_mh_fixture(Path(temporary))
            meta_path = candidate / "meta.json"
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta["config"]["estimator.enable_moving_bootstrap"] = False
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            completed = self.run_validator(
                "mh01-static",
                candidate,
                "--control-dir",
                control,
                "--comparison-tolerance-m",
                "0.000001",
                "--manifest-sha256",
                manifest,
            )

        self.assertEqual(completed.returncode, 1, completed.stderr)
        self.assertIn("config", completed.stderr)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import argparse
import importlib.util
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "phad_remote_ci", REPO_ROOT / "scripts" / "remote_ci.py"
)
assert SPEC is not None and SPEC.loader is not None
remote_ci = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = remote_ci
SPEC.loader.exec_module(remote_ci)


def git(root: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(root), *arguments], text=True
    ).strip()


class SourceManifestTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        subprocess.run(
            ["git", "init", "--initial-branch=main", str(self.root)], check=True
        )
        git(self.root, "config", "user.email", "remote-ci@example.invalid")
        git(self.root, "config", "user.name", "Remote CI Test")
        (self.root / ".gitignore").write_text("build/\n", encoding="utf-8")
        (self.root / "modified.txt").write_text("base\n", encoding="utf-8")
        (self.root / "deleted.txt").write_text("deleted\n", encoding="utf-8")
        (self.root / "script.sh").write_text("#!/bin/sh\n", encoding="utf-8")
        (self.root / "script.sh").chmod(0o755)
        (self.root / ".codex").mkdir()
        (self.root / ".codex" / "config.toml").write_text(
            "model = \"test\"\n", encoding="utf-8"
        )
        git(self.root, "add", ".")
        git(self.root, "commit", "-m", "base")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def prepare_dirty_tree(self) -> None:
        (self.root / "modified.txt").write_text("dirty\n", encoding="utf-8")
        (self.root / "deleted.txt").unlink()
        (self.root / "untracked.cpp").write_text("int value = 1;\n", encoding="utf-8")
        (self.root / "build").mkdir()
        (self.root / "build" / "ignored.o").write_bytes(b"ignored")
        (self.root / ".codex").mkdir(exist_ok=True)
        (self.root / ".codex" / "session.json").write_text("{}\n", encoding="utf-8")
        artifact = self.root / "artifacts" / "remote-ci"
        artifact.mkdir(parents=True)
        (artifact / "old.log").write_text("old\n", encoding="utf-8")

    def reconstruct_source(
        self, output: Path
    ) -> tuple[dict[str, object], Path]:
        source = remote_ci.create_source_inputs(self.root, output / "inputs")
        reconstructed = output / "repo"
        subprocess.run(
            [
                "git",
                "clone",
                "--no-checkout",
                str(source["bundle_path"]),
                str(reconstructed),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        git(reconstructed, "read-tree", "HEAD")
        tar_path = output / "source.tar"
        subprocess.run(
            [
                "zstd",
                "-d",
                "--force",
                str(source["archive_path"]),
                "-o",
                str(tar_path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        subprocess.run(
            [
                "tar",
                "-xf",
                str(tar_path),
                "-C",
                str(reconstructed),
                "--no-same-owner",
                "--same-permissions",
            ],
            check=True,
        )
        return source, reconstructed

    def test_source_id_is_deterministic_and_captures_dirty_tree(self) -> None:
        self.prepare_dirty_tree()
        first_manifest, first_id = remote_ci.build_source_manifest(self.root)
        second_manifest, second_id = remote_ci.build_source_manifest(self.root)

        self.assertEqual(first_manifest, second_manifest)
        self.assertEqual(first_id, second_id)
        self.assertTrue(first_manifest["git"]["tracked_dirty"])
        self.assertEqual(first_manifest["deleted_tracked"], ["deleted.txt"])
        paths = {entry["path"] for entry in first_manifest["entries"]}
        self.assertIn("modified.txt", paths)
        self.assertIn("untracked.cpp", paths)
        self.assertNotIn("deleted.txt", paths)
        self.assertNotIn("build/ignored.o", paths)
        self.assertNotIn(".codex/session.json", paths)
        self.assertNotIn("artifacts/remote-ci/old.log", paths)

        (self.root / "untracked.cpp").write_text("int value = 2;\n", encoding="utf-8")
        _, changed_id = remote_ci.build_source_manifest(self.root)
        self.assertNotEqual(first_id, changed_id)

    def test_manifest_verifies_git_identity_and_worktree_state(self) -> None:
        self.prepare_dirty_tree()
        manifest, source_id = remote_ci.build_source_manifest(self.root)
        with tempfile.TemporaryDirectory() as manifest_directory:
            manifest_path = Path(manifest_directory) / "source-manifest.json"
            manifest_path.write_text(
                json.dumps({**manifest, "source_id": source_id}), encoding="utf-8"
            )
            verified = remote_ci.verify_source_tree(self.root, manifest_path)

        self.assertEqual(verified["source_id"], source_id)
        self.assertEqual(verified["git_head"], git(self.root, "rev-parse", "HEAD"))
        self.assertEqual(verified["deleted_tracked_count"], 1)

    def test_bundle_and_archive_reconstruct_exact_dirty_snapshot(self) -> None:
        self.prepare_dirty_tree()
        with tempfile.TemporaryDirectory() as output_directory:
            output = Path(output_directory)
            source, reconstructed = self.reconstruct_source(output)
            verified = remote_ci.verify_source_tree(
                reconstructed, source["manifest_path"]
            )

        self.assertEqual(verified["source_id"], source["source_id"])

    def test_bundle_and_archive_reconstruct_clean_snapshot_with_tracked_exclusion(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as output_directory:
            output = Path(output_directory)
            source, reconstructed = self.reconstruct_source(output)
            verified = remote_ci.verify_source_tree(
                reconstructed, source["manifest_path"]
            )

        self.assertEqual(verified["source_id"], source["source_id"])

    def test_worker_rejects_source_tampering_before_starting_any_task(self) -> None:
        self.prepare_dirty_tree()
        with tempfile.TemporaryDirectory() as output_directory:
            output = Path(output_directory)
            source, reconstructed = self.reconstruct_source(output)
            run_root = output / "run"
            run_root.mkdir()
            run = {
                "schema_version": remote_ci.SCHEMA_VERSION,
                "run_id": "tampered-source-test",
                "profile": remote_ci.PROFILE,
                "run_root": str(run_root),
                "source_root": str(reconstructed),
                "source_id": source["source_id"],
                "source_manifest": source["manifest"],
            }
            remote_ci.atomic_write_json(run_root / "run.json", run)
            (reconstructed / "modified.txt").write_text(
                "tampered after publication\n", encoding="utf-8"
            )

            with mock.patch.object(
                remote_ci,
                "_run_remote_task",
                side_effect=AssertionError("task started before source verification"),
            ) as run_task:
                with self.assertRaises(remote_ci.RemoteCiError):
                    remote_ci._remote_worker_impl(run_root)
            run_task.assert_not_called()
            self.assertFalse((run_root / "tasks").exists())

    def test_worker_reverifies_source_after_build_before_unit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run_root = Path(temporary) / "run"
            run_root.mkdir()
            run = {
                "schema_version": remote_ci.SCHEMA_VERSION,
                "run_id": "post-build-source-test",
                "profile": remote_ci.PROFILE,
                "run_root": str(run_root),
                "source_root": "/source",
                "source_id": "sha256:" + "a" * 64,
                "source_manifest": {"git": {}},
                "build_parallelism": 16,
            }
            remote_ci.atomic_write_json(run_root / "run.json", run)
            build_status = {
                "state": "passed",
                "return_code": 0,
                "started_utc": "2026-09-03T00:00:00+00:00",
            }
            with mock.patch.object(
                remote_ci,
                "_verify_run_source_identity",
                side_effect=[{}, remote_ci.RemoteCiError("source changed")],
            ) as verify, mock.patch.object(
                remote_ci, "_run_remote_task", return_value=build_status
            ) as run_task:
                with self.assertRaisesRegex(remote_ci.RemoteCiError, "source changed"):
                    remote_ci._remote_worker_impl(run_root)

            self.assertEqual(verify.call_count, 2)
            self.assertEqual(run_task.call_count, 1)


class InputSafetyTest(unittest.TestCase):
    def test_toolchain_identity_ignores_python_cache_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            context = root / "docker" / "remote-ci"
            cache = context / "__pycache__"
            cache.mkdir(parents=True)
            (context / "Dockerfile").write_text("FROM scratch\n", encoding="utf-8")
            cached = cache / "helper.cpython-312.pyc"
            cached.write_bytes(b"first")
            first, files = remote_ci.toolchain_identity(root)
            cached.write_bytes(b"second")
            second, _ = remote_ci.toolchain_identity(root)

        self.assertEqual(first, second)
        self.assertEqual([path.name for path in files], ["Dockerfile"])

    def test_oci_image_identity_verifies_manifest_and_config_digests(self) -> None:
        config_bytes = b'{"architecture":"amd64","os":"linux"}'
        config_id = f"sha256:{hashlib.sha256(config_bytes).hexdigest()}"
        manifest_bytes = json.dumps(
            {
                "schemaVersion": 2,
                "config": {"digest": config_id},
                "layers": [],
            },
            separators=(",", ":"),
        ).encode()
        manifest_id = f"sha256:{hashlib.sha256(manifest_bytes).hexdigest()}"
        index_bytes = json.dumps(
            {
                "schemaVersion": 2,
                "manifests": [
                    {
                        "digest": manifest_id,
                        "annotations": {
                            "io.containerd.image.name": "docker.io/library/phad:test"
                        },
                    }
                ],
            },
            separators=(",", ":"),
        ).encode()

        identity = remote_ci.parse_oci_image_identity(
            index_bytes,
            manifest_bytes,
            config_bytes,
            tag="phad:test",
            local_native_id=config_id,
        )
        self.assertEqual(identity["oci_manifest_id"], manifest_id)
        self.assertEqual(identity["oci_config_id"], config_id)
        with self.assertRaises(remote_ci.RemoteCiError):
            remote_ci.parse_oci_image_identity(
                index_bytes,
                manifest_bytes,
                b"tampered",
                tag="phad:test",
                local_native_id=config_id,
            )

    def test_ssh_contract_uses_the_pinned_host_key_and_public_key_only(self) -> None:
        client = remote_ci.VerifiedSsh()
        client.known_hosts = Path("/tmp/remote-ci-known-hosts")
        options = client._options()

        self.assertIn("StrictHostKeyChecking=yes", options)
        self.assertIn("BatchMode=yes", options)
        self.assertIn("PreferredAuthentications=publickey", options)
        self.assertIn("PasswordAuthentication=no", options)
        self.assertIn("KbdInteractiveAuthentication=no", options)
        self.assertIn("ForwardAgent=no", options)

    def test_identifiers_and_snapshot_paths_reject_injection(self) -> None:
        for value in ("../run", "run/name", "run name", "run;touch-x", ""):
            with self.subTest(value=value):
                with self.assertRaises(remote_ci.RemoteCiError):
                    remote_ci.validate_identifier(value, "run-id")
        for value in ("../file", "/absolute", "a/../../b", "a/./b"):
            with self.subTest(value=value):
                with self.assertRaises(remote_ci.RemoteCiError):
                    remote_ci._validate_snapshot_path(value)

    def test_result_archive_rejects_parent_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive = root / "unsafe.tar"
            payload = root / "payload"
            payload.write_text("unsafe\n", encoding="utf-8")
            with tarfile.open(archive, "w") as stream:
                stream.add(payload, arcname="../outside")
            with self.assertRaises(remote_ci.RemoteCiError):
                remote_ci._safe_extract_tar(archive, root / "output")

    def test_container_contract_is_readonly_and_uses_immutable_image(self) -> None:
        manifest_id = "sha256:" + "a" * 64
        config_id = "sha256:" + "b" * 64
        run = {
            "run_id": "run-1",
            "remote": {"uid": 1000, "gid": 1000},
            "image": {
                "tag": "phad:mutable-test-tag",
                "remote_id": manifest_id,
                "oci_manifest_id": manifest_id,
                "oci_config_id": config_id,
            },
        }
        command = remote_ci._container_base(
            run,
            "unit",
            cpus=2,
            memory="1g",
            mounts=((Path("/source"), "/src", True),),
        )
        command.extend(
            [
                "--cidfile",
                "/task/cid",
                remote_ci._task_image_reference(run),
                "true",
            ]
        )

        self.assertIn("--read-only", command)
        self.assertIn("none", command[command.index("--network") + 1 :])
        self.assertIn("type=bind,src=/source,dst=/src,readonly", command)
        self.assertLess(command.index("--cidfile"), command.index(manifest_id))
        self.assertNotIn("phad:mutable-test-tag", command)

        run["image"]["remote_id"] = "phad:mutable-test-tag"
        with self.assertRaises(remote_ci.RemoteCiError):
            remote_ci._task_image_reference(run)

    def test_reused_remote_source_is_fully_reverified(self) -> None:
        source_id = "sha256:" + "c" * 64
        source = {
            "source_id": source_id,
            "archive_sha256": "d" * 64,
            "bundle_sha256": "e" * 64,
        }
        ready = subprocess.CompletedProcess(
            [],
            0,
            stdout=json.dumps(
                {
                    "source_id": source_id,
                    "archive_sha256": source["archive_sha256"],
                    "bundle_sha256": source["bundle_sha256"],
                }
            ),
            stderr="",
        )
        verified = subprocess.CompletedProcess(
            [],
            0,
            stdout=json.dumps(
                {
                    "source_id": source_id,
                    "archive_sha256": source["archive_sha256"],
                    "bundle_sha256": source["bundle_sha256"],
                }
            ),
            stderr="",
        )
        ssh = mock.Mock()
        ssh.run.side_effect = [ready, verified]

        result = remote_ci.ensure_remote_source(ssh, source)

        self.assertFalse(result["transferred"])
        self.assertEqual(ssh.run.call_count, 2)
        verify_command = ssh.run.call_args_list[1].args[0]
        self.assertIn("_remote-verify-source", verify_command)
        self.assertIn("--manifest", verify_command)

    def test_remote_image_smoke_uses_verified_immutable_id(self) -> None:
        remote_id = "sha256:" + "1" * 64
        manifest_id = "sha256:" + "2" * 64
        image = {
            "tag": "phad:mutable-test-tag",
            "oci_config_id": remote_id,
            "oci_manifest_id": manifest_id,
            "rootfs_layers": ["sha256:" + "3" * 64],
            "architecture": "amd64",
            "os": "linux",
            "toolchain_id": "4" * 64,
            "versions": {"compiler": "test"},
        }
        inspected = {
            "Id": remote_id,
            "Descriptor": {"digest": manifest_id},
            "RootFS": {"Layers": image["rootfs_layers"]},
            "Architecture": "amd64",
            "Os": "linux",
            "Config": {
                "Labels": {"phad.remote-ci.toolchain-id": image["toolchain_id"]}
            },
        }
        ssh = mock.Mock()
        ssh.run.side_effect = [
            subprocess.CompletedProcess([], 0, stdout="1000\n", stderr=""),
            subprocess.CompletedProcess([], 0, stdout="1000\n", stderr=""),
            subprocess.CompletedProcess(
                [], 0, stdout=json.dumps(image["versions"]), stderr=""
            ),
        ]

        self.assertEqual(
            remote_ci._verify_remote_image(ssh, image, inspected), remote_id
        )
        smoke_command = ssh.run.call_args_list[2].args[0]
        self.assertIn(remote_id, smoke_command)
        self.assertNotIn(image["tag"], smoke_command)
        self.assertIn("zstd --version", smoke_command[-1])


class StatusAndSummaryTest(unittest.TestCase):
    @staticmethod
    def _typed_product_failure_run() -> dict[str, object]:
        return {
            "source_manifest": {
                "git": {
                    "head": "a" * 40,
                    "tree": "b" * 40,
                    "branch": "main",
                    "tracked_dirty": False,
                }
            }
        }

    @staticmethod
    def _write_typed_product_failure_envelope(bench: Path) -> dict[str, object]:
        bench.mkdir(parents=True)
        code = {
            "git_commit": "a" * 40,
            "git_commit_short": "a" * 7,
            "git_dirty": False,
            "git_branch": "main",
        }
        config = {"x": 1}
        config_hash = "06651953"
        summary: dict[str, object] = {
            "schema_version": 2,
            "status": "failed",
            "sequence": "room1",
            "code": {
                "git_commit_short": code["git_commit_short"],
                "git_dirty": code["git_dirty"],
            },
            "config_label": "default",
            "config_hash": config_hash,
            "trajectory": {
                "image_frames": 2,
                "poses_written": 1,
                "ok": 1,
                "rejected": 0,
                "failed": 1,
                "completion_rate": 0.5,
                "coverage_rate": 0.0,
                "segments": 1,
                "total_keyframes": 0,
                "total_track_only_frames": 2,
            },
            "robustness": {
                "rejected": 0,
                "failed": 1,
                "low_connectivity": 0,
                "cheirality": 0,
                "reanchors": 0,
                "pnp_successes": 0,
                "pnp_fallbacks": 0,
                "outliers_culled": 0,
                "outliers_culled_unique": 0,
                "outlier_reopts": 0,
                "drops_skipped": 0,
                "deferred_drops": 0,
                "deferred_drop_ids": 0,
                "evictable_marked": 0,
                "tracks_evicted": 0,
                "zombie_age_drops": 0,
                "zombie_age_drop_ids": 0,
            },
            "ate": None,
            "rpe": None,
            "timing": {
                "wall_s": 1.0,
                "rtf": 1.0,
                "rectify": {"mean": 0.0, "p95": 0.0, "max": 0.0},
                "frontend": {"mean": 0.0, "p95": 0.0, "max": 0.0},
                "estimator": {"mean": 0.0, "p95": 0.0, "max": 0.0},
                "total": {"mean": 0.0, "p95": 0.0, "max": 0.0},
            },
            "sync": {
                "pushed_left": 2,
                "pushed_right": 2,
                "emitted_stereo": 2,
                "dropped_left": 0,
                "dropped_right": 0,
                "dropped_left_overflow": 0,
                "dropped_right_overflow": 0,
                "max_left_queue": 1,
                "max_right_queue": 1,
            },
            "warnings": ["VIO estimator failed: injected terminal result"],
        }
        (bench / "meta.json").write_text(
            json.dumps(
                {
                    "schema_version": 2,
                    "sequence": "room1",
                    "sequence_root": "/datasets/tum-vi/room1",
                    "created_utc": "2026-09-03T00:00:00+00:00",
                    "code": code,
                    "config": config,
                    "config_label": "default",
                    "config_hash": config_hash,
                    "config_canonical_text": "x=1\n",
                    "paths": {
                        "bench_root": "",
                        "output_dir": "/task/artifacts/bench",
                        "layout_template": "override",
                    },
                    "warnings": [],
                }
            ),
            encoding="utf-8",
        )
        (bench / "summary.json").write_text(
            json.dumps(summary), encoding="utf-8"
        )
        header = (
            "timestamp_ns,status,num_obs,num_landmarks,num_shared,"
            "low_connectivity,window_size,prior_key,reproj_rms_before_px,"
            "reproj_rms_after_px,num_cheirality,lm_iterations,"
            "max_window_pose_shift_m,segment_id,pnp_success,pnp_inliers,"
            "outliers_culled,reproj_rms_after_cull_px,is_keyframe,"
            "num_disparity,unsupported_span_ns,num_retained_observations,"
            "num_seeded_landmarks,num_current_visual_factors,"
            "num_mapped_observations,num_current_mono_visual_factors"
        )
        zero_fields = ",".join("0" for _ in range(24))
        (bench / "diag.csv").write_text(
            f"{header}\n100,ok,{zero_fields}\n200,failed,{zero_fields}\n",
            encoding="utf-8",
        )
        (bench / "est.tum").write_text("0.000000100 0 0 0 0 0 0 1\n", encoding="utf-8")
        (bench / "kf.tum").write_text("", encoding="utf-8")
        probe_rows = [
            {
                "i": index,
                "ts_ns": timestamp,
                "pnp_attempted": False,
                "pnp_result": "not_attempted",
                "pnp_correspondences": 0,
                "pnp_inliers": 0,
                "pnp_guess_retry_outcome": "not_attempted",
                "shared": 0,
                "num_obs": 0,
            }
            for index, timestamp in enumerate((100, 200))
        ]
        (bench / "probe-b.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in probe_rows),
            encoding="utf-8",
        )
        return summary

    def test_record_only_benchmark_outcomes_are_explicit(self) -> None:
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0, {"status": "completed_with_failures"}
            ),
            ("warning", None),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0, {"status": "eval_failed", "warnings": ["no matches"]}
            ),
            ("failed", "no matches"),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0,
                {
                    "status": "completed_with_warnings",
                    "warnings": ["first", "canonical warning"],
                },
            ),
            ("warning", "canonical warning"),
        )

    def test_status_aggregation_keeps_sibling_failures_independent(self) -> None:
        self.assertEqual(
            remote_ci.aggregate_run_state(
                {
                    "unit": {"state": "failed"},
                    "MH_01_easy": {"state": "passed"},
                }
            ),
            "failed",
        )
        self.assertEqual(
            remote_ci.aggregate_run_state(
                {
                    "unit": {"state": "passed"},
                    "MH_01_easy": {"state": "warning"},
                }
            ),
            "warning",
        )

    def test_confirmatory_benchmark_requires_zero_failed_and_rejected(self) -> None:
        self.assertIn(
            "require_zero_failed_rejected",
            inspect.signature(remote_ci.classify_benchmark_task).parameters,
        )
        completed = {
            "status": "completed",
            "robustness": {"failed": 0, "rejected": 0},
        }
        failed = {
            "status": "completed_with_failures",
            "robustness": {"failed": 1, "rejected": 0},
        }
        rejected = {
            "status": "completed",
            "robustness": {"failed": 0, "rejected": 2},
        }

        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0, completed, require_zero_failed_rejected=True
            ),
            ("passed", None),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0, failed, require_zero_failed_rejected=True
            ),
            ("warning", "benchmark robustness.failed=1, robustness.rejected=0"),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                0, rejected, require_zero_failed_rejected=True
            ),
            ("warning", "benchmark robustness.failed=0, robustness.rejected=2"),
        )

    def test_confirmatory_benchmark_rejects_malformed_robustness(self) -> None:
        for robustness in (None, {}, {"failed": 0}, {"failed": False, "rejected": 0}):
            with self.subTest(robustness=robustness):
                self.assertEqual(
                    remote_ci.classify_benchmark_task(
                        0,
                        {"status": "completed", "robustness": robustness},
                        require_zero_failed_rejected=True,
                    ),
                    ("failed", "benchmark robustness counters are invalid"),
                )

    def test_typed_product_failure_remains_collectable(self) -> None:
        benchmark = {
            "status": "failed",
            "warnings": ["session entered a failed estimator state"],
            "robustness": {"failed": 1, "rejected": 0},
        }
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                1,
                benchmark,
                require_zero_failed_rejected=True,
            ),
            ("failed", "session entered a failed estimator state"),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                1,
                benchmark,
                require_zero_failed_rejected=True,
                complete_product_failure_envelope=True,
            ),
            ("warning", "benchmark robustness.failed=1, robustness.rejected=0"),
        )
        self.assertEqual(
            remote_ci.classify_benchmark_task(
                1,
                {
                    "status": "eval_failed",
                    "warnings": ["groundtruth association failed"],
                    "robustness": {"failed": 0, "rejected": 0},
                },
                require_zero_failed_rejected=True,
            ),
            ("failed", "groundtruth association failed"),
        )

    def test_typed_product_failure_envelope_is_strict_and_complete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            bench = Path(temporary) / "bench"
            summary = self._write_typed_product_failure_envelope(bench)
            remote_ci._validate_typed_product_failure_envelope(
                bench, summary, self._typed_product_failure_run(), "off-1-room1"
            )

        with tempfile.TemporaryDirectory() as temporary:
            bench = Path(temporary) / "bench"
            summary = self._write_typed_product_failure_envelope(bench)
            summary["trajectory"] = {
                "image_frames": 1,
                "poses_written": 0,
                "ok": 0,
                "rejected": 0,
                "failed": 1,
                "completion_rate": 0.0,
                "coverage_rate": 0.0,
                "segments": 0,
                "total_keyframes": 0,
                "total_track_only_frames": 1,
            }
            summary["warnings"] = [
                "VIO estimator invalid_input: injected terminal result"
            ]
            (bench / "summary.json").write_text(
                json.dumps(summary), encoding="utf-8"
            )
            header = (bench / "diag.csv").read_text(
                encoding="utf-8"
            ).splitlines()[0]
            zero_fields = ",".join("0" for _ in range(24))
            (bench / "diag.csv").write_text(
                f"{header}\n100,invalid_input,{zero_fields}\n",
                encoding="utf-8",
            )
            (bench / "est.tum").write_text("", encoding="utf-8")
            (bench / "probe-b.jsonl").write_text(
                json.dumps(
                    {
                        "i": 0,
                        "ts_ns": 100,
                        "pnp_attempted": False,
                        "pnp_result": "not_attempted",
                        "pnp_correspondences": 0,
                        "pnp_inliers": 0,
                        "pnp_guess_retry_outcome": "not_attempted",
                        "shared": 0,
                        "num_obs": 0,
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            remote_ci._validate_typed_product_failure_envelope(
                bench, summary, self._typed_product_failure_run(), "off-1-room1"
            )

        mutations = {
            "missing kf": lambda bench, summary: (bench / "kf.tum").unlink(),
            "non-null metrics": lambda bench, summary: summary.__setitem__(
                "ate", {"trans": {"rmse": 1.0}}
            ),
            "terminal diag status": lambda bench, summary: (
                bench / "diag.csv"
            ).write_text(
                (bench / "diag.csv").read_text(encoding="utf-8").replace(
                    "200,failed,", "200,ok,"
                ),
                encoding="utf-8",
            ),
            "probe timestamp": lambda bench, summary: (
                bench / "probe-b.jsonl"
            ).write_text(
                '{"i":0,"ts_ns":100}\n{"i":1,"ts_ns":201}\n',
                encoding="utf-8",
            ),
            "pose count": lambda bench, summary: (
                bench / "est.tum"
            ).write_text("", encoding="utf-8"),
            "extra summary schema": lambda bench, summary: summary.__setitem__(
                "unexpected", True
            ),
            "unknown probe token": lambda bench, summary: (
                bench / "probe-b.jsonl"
            ).write_text(
                (bench / "probe-b.jsonl")
                .read_text(encoding="utf-8")
                .replace("{", '{"unexpected":1,', 1),
                encoding="utf-8",
            ),
        }
        for label, mutate in mutations.items():
            with (
                self.subTest(label=label),
                tempfile.TemporaryDirectory() as temporary,
            ):
                bench = Path(temporary) / "bench"
                summary = self._write_typed_product_failure_envelope(bench)
                mutate(bench, summary)
                (bench / "summary.json").write_text(
                    json.dumps(summary), encoding="utf-8"
                )
                with self.assertRaises(remote_ci.RemoteCiError):
                    remote_ci._validate_typed_product_failure_envelope(
                        bench,
                        summary,
                        self._typed_product_failure_run(),
                        "off-1-room1",
                    )

        with tempfile.TemporaryDirectory() as temporary:
            bench = Path(temporary) / "bench"
            summary = self._write_typed_product_failure_envelope(bench)
            mismatched_run = self._typed_product_failure_run()
            mismatched_run["source_manifest"]["git"]["head"] = "c" * 40
            with self.assertRaises(remote_ci.RemoteCiError):
                remote_ci._validate_typed_product_failure_envelope(
                    bench, summary, mismatched_run, "off-1-room1"
                )

    def test_summary_contains_failures_logs_and_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result_root = Path(temporary)
            bench = result_root / "tasks" / "MH_01_easy" / "artifacts" / "bench"
            bench.mkdir(parents=True)
            (bench / "summary.json").write_text(
                json.dumps(
                    {
                        "status": "completed_with_failures",
                        "ate": {"trans": {"rmse": 0.42}},
                        "rpe": {"trans": {"rmse": 0.07}},
                        "trajectory": {
                            "completion_rate": 0.8,
                            "coverage_rate": 0.75,
                            "segments": 3,
                        },
                        "robustness": {"reanchors": 2},
                    }
                ),
                encoding="utf-8",
            )
            (bench.parent / "segment_ate_decomp.txt").write_text(
                "段内加权 RMS = 1.25e-1 m\n", encoding="utf-8"
            )
            run = {
                "run_id": "run-1",
                "profile": remote_ci.PROFILE,
                "source_id": "sha256:source",
                "source_manifest": {
                    "git": {
                        "head": "a" * 40,
                        "tree": "b" * 40,
                        "branch": "main",
                        "tracked_dirty": True,
                    }
                },
                "source_archive_sha256": "c" * 64,
                "source_bundle_sha256": "d" * 64,
                "image": {
                    "tag": "phad:test",
                    "id": "sha256:image",
                    "oci_manifest_id": "sha256:manifest",
                    "oci_config_id": "sha256:config",
                    "remote_id": "sha256:manifest",
                    "archive_sha256": "e" * 64,
                    "toolchain_id": "f" * 64,
                    "dockerfile_sha256": "1" * 64,
                    "gtsam_archive_sha256": "2" * 64,
                    "versions": {"gtsam_commit": remote_ci.GTSAM_COMMIT},
                },
                "remote": {"target": "lin@192.168.110.34"},
                "dataset_root": str(remote_ci.REMOTE_DATA_ROOT),
                "run_root": "/home/lin/Projects/tigerfish/runs/run-1",
                "build_parallelism": 16,
                "unit_parallelism": 8,
                "max_parallel_tasks": 8,
            }
            status = {
                "state": "failed",
                "tasks": {
                    "MH_01_easy": {
                        "kind": "euroc",
                        "state": "failed",
                        "return_code": 2,
                        "failure_reason": "evaluation failed",
                        "remote_log_path": "/home/lin/Projects/tigerfish/runs/run-1/tasks/MH_01_easy/logs/stdout.log",
                        "remote_stderr_path": "/home/lin/Projects/tigerfish/runs/run-1/tasks/MH_01_easy/logs/stderr.log",
                    }
                },
            }

            summary = remote_ci.render_summary(
                run, status, result_root, current_source_matches=False
            )

            self.assertIn("evaluation failed", summary)
            self.assertIn("/home/lin/Projects/tigerfish/runs/run-1/tasks", summary)
            self.assertIn("0.42", summary)
            self.assertIn("0.125", summary)
            self.assertIn("当前工作区匹配：`no`", summary)

    def test_current_index_updates_only_for_matching_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = {
                "run_id": "run-1",
                "profile": remote_ci.PROFILE,
                "source_id": "sha256:source",
            }
            status = {"state": "passed", "finished_utc": "2026-08-26T12:00:00Z"}
            summary_path = root / "run-summary.md"

            first = remote_ci.publish_summary_indexes(
                root,
                run,
                status,
                "first\n",
                summary_path,
                current_source_matches=False,
            )
            self.assertTrue(first["latest"])
            self.assertFalse(first["current"])
            self.assertFalse(
                (root / remote_ci.LOCAL_ARTIFACT_ROOT / "current" / "ci-euroc11.md").exists()
            )

            second = remote_ci.publish_summary_indexes(
                root,
                run,
                status,
                "second\n",
                summary_path,
                current_source_matches=True,
            )
            self.assertTrue(second["current"])
            self.assertEqual(
                (
                    root
                    / remote_ci.LOCAL_ARTIFACT_ROOT
                    / "current"
                    / "ci-euroc11.md"
                ).read_text(encoding="utf-8"),
                "second\n",
            )


class RemoteIdentityConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.saved_environ = {
            key: os.environ.get(key)
            for key in (
                remote_ci.REMOTE_IDENTITY_CONFIG_ENV,
                "XDG_CONFIG_HOME",
            )
        }
        remote_ci.reset_remote_identity_cache()

    def tearDown(self) -> None:
        remote_ci.reset_remote_identity_cache()
        for key, value in self.saved_environ.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        self.temporary.cleanup()

    def write_config(self, path: Path, payload: object) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_missing_file_reports_path_and_example(self) -> None:
        missing = self.root / "missing.json"
        os.environ[remote_ci.REMOTE_IDENTITY_CONFIG_ENV] = str(missing)

        with self.assertRaises(remote_ci.RemoteCiError) as raised:
            remote_ci.remote_identity()

        message = str(raised.exception)
        self.assertIn(str(missing), message)
        self.assertIn("host", message)
        self.assertIn("fingerprint", message)

    def test_extra_keys_and_missing_fields_are_rejected(self) -> None:
        extra = self.write_config(
            self.root / "extra.json",
            {
                "host": "192.168.110.34",
                "user": "lin",
                "fingerprint": "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE",
                "root": "/tmp",
            },
        )
        missing = self.write_config(
            self.root / "partial.json",
            {"host": "192.168.110.34", "user": "lin"},
        )

        with self.assertRaises(remote_ci.RemoteCiError):
            remote_ci.load_remote_identity(extra)
        with self.assertRaises(remote_ci.RemoteCiError):
            remote_ci.load_remote_identity(missing)

    def test_invalid_json_is_rejected(self) -> None:
        path = self.root / "broken.json"
        path.write_text("{not-json", encoding="utf-8")

        with self.assertRaises(remote_ci.RemoteCiError) as raised:
            remote_ci.load_remote_identity(path)

        self.assertIn(str(path), str(raised.exception))

    def test_env_override_reads_host_user_fingerprint(self) -> None:
        path = self.write_config(
            self.root / "identity.json",
            {
                "host": "192.168.110.34",
                "user": "lin",
                "fingerprint": "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE",
            },
        )
        os.environ[remote_ci.REMOTE_IDENTITY_CONFIG_ENV] = str(path)

        identity = remote_ci.remote_identity()

        self.assertEqual(identity.host, "192.168.110.34")
        self.assertEqual(identity.user, "lin")
        self.assertEqual(identity.target, "lin@192.168.110.34")
        self.assertEqual(
            identity.fingerprint,
            "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE",
        )

    def test_xdg_config_home_is_used_without_env_override(self) -> None:
        os.environ.pop(remote_ci.REMOTE_IDENTITY_CONFIG_ENV, None)
        os.environ["XDG_CONFIG_HOME"] = str(self.root)
        self.write_config(
            self.root / "phad-remote-ci" / "config.json",
            {
                "host": "10.0.0.8",
                "user": "ci",
                "fingerprint": "SHA256:abcdefghijklmnopqrstuvwxyz0123456789+/AB",
            },
        )

        identity = remote_ci.load_remote_identity()

        self.assertEqual(identity.target, "ci@10.0.0.8")

    def test_help_does_not_require_identity_config(self) -> None:
        os.environ[remote_ci.REMOTE_IDENTITY_CONFIG_ENV] = str(
            self.root / "missing.json"
        )
        result = subprocess.run(
            ["python3", str(REPO_ROOT / "scripts" / "remote_ci.py"), "--help"],
            check=False,
            capture_output=True,
            text=True,
            env=os.environ.copy(),
        )
        self.assertEqual(result.returncode, 0)
        self.assertIn("doctor", result.stdout)


class TumViConfirmatoryProfileTest(unittest.TestCase):
    PROFILE = "ci-tumvi-room6-confirmatory"
    ROOMS = ("room1", "room2", "room3", "room4", "room5", "room6")
    BATCHES = (
        ("off", 1),
        ("on", 1),
        ("on", 2),
        ("off", 2),
        ("off", 3),
        ("on", 3),
    )
    ARCHIVE_SHA256 = {
        "room1": "20354392eab5cc82c9770ed29cf0130c5acce0db54a257571499008fb34c398f",
        "room2": "70343c5bbd9d674d0a2790bf934ee7d9cc08c1dea7cd77747a5557e4c94973f6",
        "room3": "66ff297e6600e882277d21e21af5ec9fb33864ec0b759dcc0b629cf551a3c824",
        "room4": "2c3633407693988cf24faef5f874cba08bbc3c2d2ec1168c86b6da55ae9f2e68",
        "room5": "be6129b096dc88a7da0a552f1fc74c3309fffaa88bf4528a779e3ecc60a65519",
        "room6": "3183e404e1d24bb8cea29ab6e4e69a69a5f88a68dac6c2ab7f67b6ac86d62827",
    }

    def test_profile_definition_freezes_room_inputs_and_batch_order(self) -> None:
        self.assertTrue(
            callable(getattr(remote_ci, "profile_definition", None)),
            "remote_ci.profile_definition is required",
        )
        definition = remote_ci.profile_definition(self.PROFILE)

        self.assertEqual(definition["name"], self.PROFILE)
        self.assertEqual(tuple(definition["sequences"]), self.ROOMS)
        self.assertEqual(
            tuple(tuple(batch) for batch in definition["batch_order"]),
            self.BATCHES,
        )
        self.assertEqual(definition["max_parallel_tasks"], 6)
        self.assertEqual(
            {
                room: definition["dataset_manifest"]["archives"][room]["sha256"]
                for room in self.ROOMS
            },
            self.ARCHIVE_SHA256,
        )
        self.assertRegex(
            definition["dataset_manifest_id"], r"^sha256:[0-9a-f]{64}$"
        )
        self.assertEqual(
            definition["dataset_manifest_id"],
            "sha256:"
            + hashlib.sha256(
                json.dumps(
                    definition["dataset_manifest"],
                    ensure_ascii=False,
                    separators=(",", ":"),
                    sort_keys=True,
                ).encode("utf-8")
            ).hexdigest(),
        )

    def test_start_parser_accepts_confirmatory_profile(self) -> None:
        parser = remote_ci.build_parser()
        commands = next(
            action
            for action in parser._actions
            if isinstance(action, argparse._SubParsersAction)
        )
        start = commands.choices["start"]
        profile = next(
            action for action in start._actions if action.dest == "profile"
        )
        self.assertIn(self.PROFILE, profile.choices)

    def test_room_script_has_fixed_tum_vi_cli_and_role_only_activation(self) -> None:
        self.assertTrue(
            callable(getattr(remote_ci, "_tum_vi_sequence_script", None)),
            "remote_ci._tum_vi_sequence_script is required",
        )
        off = remote_ci._tum_vi_sequence_script("room4", "off")
        on = remote_ci._tum_vi_sequence_script("room4", "on")

        required = (
            "/shared-build/phad_vo_bench /datasets/tum-vi/room4",
            "--dataset-format tum-vi",
            "--gt-tum-vi /datasets/tum-vi/room4",
            "--out /task/artifacts/bench",
            "--sequence-name room4",
            "--config-label default",
            "--repo /src",
            "--max-dt-ms 4.166667",
            "--min-match-rate 0.80",
            "--rpe-delta-s 1.0",
            "--estimator-enable-moving-bootstrap",
            "--probe-b /task/artifacts/bench/probe-b.jsonl",
            "--force",
        )
        for token in required:
            with self.subTest(token=token):
                self.assertIn(token, off)
                self.assertIn(token, on)
        retry_flag = "--estimator-enable-pnp-cheirality-retry"
        self.assertNotIn(retry_flag, off)
        self.assertEqual(on.count(retry_flag), 1)
        self.assertEqual(on.replace(f"  {retry_flag} \\\n", ""), off)

    def _run_worker_with_fake_tasks(
        self,
        *,
        failing_task: str | None = None,
        warning_task: str | None = None,
        product_failure_task: str | None = None,
    ) -> tuple[int, list[str], list[str], dict[str, int]]:
        with tempfile.TemporaryDirectory() as temporary:
            run_root = Path(temporary)
            run = {
                "schema_version": remote_ci.SCHEMA_VERSION,
                "run_id": "tumvi-confirmatory-test",
                "profile": self.PROFILE,
                "run_root": str(run_root),
                "source_root": "/source",
                "dataset_root": "/datasets/tum-vi",
                "sequences": list(self.ROOMS),
                "batch_order": [list(batch) for batch in self.BATCHES],
                "max_parallel_tasks": 6,
                "build_parallelism": 16,
                "unit_parallelism": 8,
            }
            remote_ci.atomic_write_json(run_root / "run.json", run)
            started: list[str] = []
            violations: list[str] = []
            completed_by_batch = {f"{role}-{repeat}": 0 for role, repeat in self.BATCHES}
            active_by_batch = {f"{role}-{repeat}": 0 for role, repeat in self.BATCHES}
            peak_by_batch = {f"{role}-{repeat}": 0 for role, repeat in self.BATCHES}
            barriers = {
                f"{role}-{repeat}": threading.Barrier(len(self.ROOMS))
                for role, repeat in self.BATCHES
            }
            unit_finished = threading.Event()
            lock = threading.Lock()

            def fake_task(
                task_run: dict[str, object],
                name: str,
                kind: str,
                script: str,
                *,
                cpus: int,
                memory: str,
                extra_mounts: object = (),
            ) -> dict[str, object]:
                del script, cpus, memory, extra_mounts
                with lock:
                    started.append(name)
                if name == "unit":
                    time.sleep(0.03)
                    unit_finished.set()
                elif name != "build":
                    if not unit_finished.is_set():
                        with lock:
                            violations.append(f"{name} started before unit completed")
                    batch = "-".join(name.split("-")[:2])
                    if batch not in completed_by_batch:
                        with lock:
                            violations.append(f"unexpected confirmatory task name: {name}")
                    else:
                        with lock:
                            batch_index = [
                                f"{role}-{repeat}" for role, repeat in self.BATCHES
                            ].index(batch)
                            unfinished = [
                                previous
                                for previous in list(completed_by_batch)[:batch_index]
                                if completed_by_batch[previous] != len(self.ROOMS)
                            ]
                            if unfinished:
                                violations.append(
                                    f"{name} started before batches completed: {unfinished}"
                                )
                            active_by_batch[batch] += 1
                            peak_by_batch[batch] = max(
                                peak_by_batch[batch], active_by_batch[batch]
                            )
                        try:
                            barriers[batch].wait(timeout=1.0)
                        except threading.BrokenBarrierError:
                            with lock:
                                violations.append(
                                    f"{batch} did not run all rooms concurrently"
                                )
                        with lock:
                            active_by_batch[batch] -= 1
                            completed_by_batch[batch] += 1

                state = (
                    "failed"
                    if name == failing_task
                    else "warning"
                    if name in {warning_task, product_failure_task}
                    else "passed"
                )
                status = {
                    "schema_version": remote_ci.SCHEMA_VERSION,
                    "name": name,
                    "kind": kind,
                    "state": state,
                    "return_code": (
                        1
                        if state == "failed" or name == product_failure_task
                        else 0
                    ),
                    "started_utc": "2026-09-03T00:00:00+00:00",
                    "finished_utc": "2026-09-03T00:00:01+00:00",
                }
                task_root = Path(str(task_run["run_root"])) / "tasks" / name
                remote_ci.atomic_write_json(task_root / "status.json", status)
                return status

            with mock.patch.object(
                remote_ci, "_run_remote_task", side_effect=fake_task
            ), mock.patch.object(
                remote_ci,
                "_verify_run_source_identity",
                return_value={},
            ), mock.patch.object(
                remote_ci,
                "_create_results_archive",
                return_value=(run_root / "results.tar.zst", "a" * 64),
            ):
                return_code = remote_ci._remote_worker_impl(run_root)
            return return_code, started, violations, peak_by_batch

    def test_worker_builds_once_then_runs_unit_and_six_serial_batches(self) -> None:
        return_code, started, violations, peak_by_batch = (
            self._run_worker_with_fake_tasks()
        )

        expected_tasks = {
            f"{role}-{repeat}-{room}"
            for role, repeat in self.BATCHES
            for room in self.ROOMS
        }
        self.assertEqual(return_code, 0)
        self.assertEqual(started.count("build"), 1)
        self.assertEqual(started.count("unit"), 1)
        self.assertEqual(set(started) - {"build", "unit"}, expected_tasks)
        self.assertEqual(len(started), 38)
        self.assertEqual(violations, [])
        self.assertEqual(set(peak_by_batch.values()), {6})

    def test_failed_batch_blocks_all_later_batches(self) -> None:
        return_code, started, _, _ = self._run_worker_with_fake_tasks(
            failing_task="off-1-room3"
        )

        self.assertEqual(return_code, 1)
        self.assertTrue({f"off-1-{room}" for room in self.ROOMS} <= set(started))
        self.assertFalse(
            any(
                name.startswith(prefix)
                for prefix in ("on-1-", "on-2-", "off-2-", "off-3-", "on-3-")
                for name in started
            )
        )

    def test_product_hard_warning_preserves_the_complete_matrix(self) -> None:
        return_code, started, violations, _ = self._run_worker_with_fake_tasks(
            warning_task="off-1-room3"
        )

        self.assertEqual(return_code, 0)
        self.assertEqual(len(started), 38)
        self.assertEqual(violations, [])

    def test_typed_product_failure_preserves_the_complete_matrix(self) -> None:
        return_code, started, violations, _ = self._run_worker_with_fake_tasks(
            product_failure_task="off-1-room3"
        )

        self.assertEqual(return_code, 0)
        self.assertEqual(len(started), 38)
        self.assertEqual(violations, [])


if __name__ == "__main__":
    unittest.main()

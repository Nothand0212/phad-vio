from __future__ import annotations

import importlib.util
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest


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

    def test_container_contract_is_readonly_and_image_follows_options(self) -> None:
        run = {
            "run_id": "run-1",
            "remote": {"uid": 1000, "gid": 1000},
            "image": {"tag": "phad:test"},
        }
        command = remote_ci._container_base(
            run,
            "unit",
            cpus=2,
            memory="1g",
            mounts=((Path("/source"), "/src", True),),
        )
        command.extend(["--cidfile", "/task/cid", run["image"]["tag"], "true"])

        self.assertIn("--read-only", command)
        self.assertIn("none", command[command.index("--network") + 1 :])
        self.assertIn("type=bind,src=/source,dst=/src,readonly", command)
        self.assertLess(command.index("--cidfile"), command.index("phad:test"))


class StatusAndSummaryTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

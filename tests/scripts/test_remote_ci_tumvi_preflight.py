from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "phad_remote_ci_tumvi_preflight", REPO_ROOT / "scripts" / "remote_ci.py"
)
assert SPEC is not None and SPEC.loader is not None
remote_ci = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = remote_ci
SPEC.loader.exec_module(remote_ci)

PROTOCOL_PATH = (
    "docs/research/"
    "2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-protocol.md"
)
ANALYZER_PATH = "scripts/analyze_tumvi_room_confirmatory.py"
ROOMS = tuple(f"room{index}" for index in range(1, 7))
BATCH_ORDER = (
    ("off", 1),
    ("on", 1),
    ("on", 2),
    ("off", 2),
    ("off", 3),
    ("on", 3),
)
REQUIRED_BINARIES = (
    "phad_vo_bench",
    "phad_estimator_tests",
    "phad_apps_tests",
    "phad_io_dataset_tests",
    "phad_eval_tests",
    "phad_bench_tests",
)
PREFLIGHT_CREATED_UTC = "2026-09-03T00:00:02+00:00"
ROOTFS_LAYERS = (
    "sha256:" + "8" * 64,
    "sha256:" + "9" * 64,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_id(value: object) -> str:
    payload = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return f"sha256:{hashlib.sha256(payload).hexdigest()}"


class WorkerFixture:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.source_root = root / "source"
        self.dataset_store = root / "dataset-store"
        self.definition = {
            **remote_ci.profile_definition(
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE
            ),
            "dataset_root": str(self.dataset_store),
        }
        self.dataset_root = root / "dataset-staging"
        self.missing_binary: str | None = None
        self.started: list[str] = []
        self.events: list[str] = []
        self.benchmark_receipt_seen: list[bool] = []
        self.runtime_linkage: dict[str, object] | None = None
        self._write_source()
        self._write_dataset()
        self._write_run()

    @property
    def receipt_path(self) -> Path:
        return self.root / "preflight-receipt.json"

    @property
    def build_root(self) -> Path:
        return self.root / "tasks" / "build" / "build"

    def _write_source(self) -> None:
        payloads = {
            PROTOCOL_PATH: b"frozen TUM-VI confirmatory protocol\n",
            ANALYZER_PATH: b"#!/usr/bin/env python3\n# frozen analyzer\n",
        }
        entries = []
        for relative, payload in payloads.items():
            path = self.source_root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            entries.append(
                {
                    "path": relative,
                    "type": "file",
                    "mode": "0644",
                    "size": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            )
        self.source_manifest = {
            "schema_version": remote_ci.SCHEMA_VERSION,
            "git": {
                "head": "1" * 40,
                "tree": "2" * 40,
                "branch": "confirmatory-test",
                "tracked_dirty": False,
            },
            "entries": entries,
            "deleted_tracked": [],
            "included_untracked": 0,
            "excluded_prefixes": [".codex", "artifacts/remote-ci"],
        }
        self.source_id = canonical_id(self.source_manifest)

    def _write_dataset(self) -> None:
        self.dataset_root.mkdir()
        for index, room in enumerate(ROOMS, 1):
            groundtruth = self.dataset_root / room / "dso" / "gt_imu.csv"
            groundtruth.parent.mkdir(parents=True)
            groundtruth.write_text(
                "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz\n"
                f"{index},0,0,0,1,0,0,0\n",
                encoding="utf-8",
            )
        remote_ci._freeze_tumvi_dataset_tree(
            self.dataset_root, include_root=False
        )
        tree = remote_ci.build_tumvi_dataset_manifest(
            self.dataset_root,
            archive_records=self.definition["dataset_manifest"]["archives"],
            calibration=self.definition["dataset_manifest"]["calibration"],
        )
        receipt = remote_ci._tum_vi_stage_receipt(self.definition, tree)
        content_hex = str(tree["dataset_content_id"]).removeprefix("sha256:")
        published = (
            self.dataset_store / "by-content" / "sha256" / content_hex
        )
        published.parent.mkdir(parents=True)
        self.dataset_root.replace(published)
        self.dataset_root = published
        remote_ci.atomic_write_json(
            self.dataset_root / ".phad-dataset.json", receipt
        )
        (self.dataset_root / ".phad-dataset.json").chmod(0o444)
        self.dataset_root.chmod(0o555)
        self.dataset_receipt = receipt

    def _write_run(self) -> None:
        self.run = {
            "schema_version": remote_ci.SCHEMA_VERSION,
            "run_id": "tumvi-preflight-test",
            "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            "run_root": str(self.root),
            "source_root": str(self.source_root),
            "source_id": self.source_id,
            "source_manifest": self.source_manifest,
            "source_archive_sha256": "3" * 64,
            "source_bundle_sha256": "4" * 64,
            "dataset_format": "tum-vi",
            "dataset_root": str(self.dataset_root),
            "dataset_manifest": self.definition["dataset_manifest"],
            "dataset_manifest_id": self.definition["dataset_manifest_id"],
            "dataset_content_id": self.dataset_receipt["dataset_content_id"],
            "image": {
                "oci_manifest_id": "sha256:" + "5" * 64,
                "oci_config_id": "sha256:" + "6" * 64,
                "toolchain_id": "7" * 64,
                "tag": "phad-vio-remote-ci:test",
                "id": "sha256:" + "6" * 64,
                "remote_id": "sha256:" + "6" * 64,
                "rootfs_layers": list(ROOTFS_LAYERS),
            },
            "remote": {"uid": 1000, "gid": 1000},
            "sequences": list(ROOMS),
            "batch_order": [list(item) for item in BATCH_ORDER],
            "max_parallel_tasks": 6,
            "build_parallelism": 16,
            "unit_parallelism": 8,
        }
        self.save_run()

    def save_run(self) -> None:
        remote_ci.atomic_write_json(self.root / "run.json", self.run)

    def expected_receipt(self) -> dict[str, object]:
        assert self.runtime_linkage is not None
        document: dict[str, object] = {
            "schema_version": remote_ci.SCHEMA_VERSION,
            "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            "created_utc": PREFLIGHT_CREATED_UTC,
            "source_id": self.source_id,
            "dataset": {
                "manifest_id": self.run["dataset_manifest_id"],
                "content_id": self.run["dataset_content_id"],
                "receipt_sha256": sha256(
                    self.dataset_root / ".phad-dataset.json"
                ),
                "groundtruth": {
                    room: {
                        "path": f"{room}/dso/gt_imu.csv",
                        "sha256": sha256(
                            self.dataset_root / room / "dso" / "gt_imu.csv"
                        ),
                    }
                    for room in ROOMS
                },
            },
            "image": {
                "oci_manifest_id": self.run["image"]["oci_manifest_id"],
                "oci_config_id": self.run["image"]["oci_config_id"],
                "toolchain_id": self.run["image"]["toolchain_id"],
                "rootfs_layers": list(ROOTFS_LAYERS),
            },
            "inputs": {
                "protocol": {
                    "path": PROTOCOL_PATH,
                    "sha256": sha256(self.source_root / PROTOCOL_PATH),
                },
                "analyzer": {
                    "path": ANALYZER_PATH,
                    "sha256": sha256(self.source_root / ANALYZER_PATH),
                },
            },
            "binaries": {
                name: sha256(self.build_root / name)
                for name in REQUIRED_BINARIES
            },
            "runtime_linkage": self.runtime_linkage,
            "batch_order": [list(item) for item in BATCH_ORDER],
            "benchmark_commands": {
                role: {
                    room: remote_ci._tum_vi_sequence_script(room, role)
                    for room in ROOMS
                }
                for role in ("off", "on")
            },
        }
        return {**document, "preflight_receipt_id": canonical_id(document)}

    def fake_runtime_linkage(
        self, run: dict[str, object], shared_build: Path
    ) -> dict[str, object]:
        self.events.append("runtime-linkage")
        self.assert_shared_build(shared_build)
        binaries = {}
        for index, name in enumerate(REQUIRED_BINARIES, 1):
            binaries[name] = {
                "binary_sha256": sha256(shared_build / name),
                "interpreter": {
                    "path": "/lib64/ld-linux-x86-64.so.2",
                    "realpath": "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2",
                    "sha256": f"{index:064x}",
                },
                "libraries": [
                    {
                        "soname": "libc.so.6",
                        "path": "/lib/x86_64-linux-gnu/libc.so.6",
                        "realpath": "/usr/lib/x86_64-linux-gnu/libc.so.6",
                        "sha256": f"{index + 6:064x}",
                    }
                ],
                "virtual": ["linux-vdso.so.1"],
                "missing": [],
            }
        self.runtime_linkage = {
            "schema_version": remote_ci.SCHEMA_VERSION,
            "method": "readelf-ldd-sha256",
            "binaries": binaries,
        }
        return self.runtime_linkage

    def assert_shared_build(self, shared_build: Path) -> None:
        if shared_build != self.build_root:
            raise AssertionError(f"unexpected shared build: {shared_build}")

    def fake_task(
        self,
        run: dict[str, object],
        name: str,
        kind: str,
        script: str,
        *,
        cpus: int,
        memory: str,
        extra_mounts: object = (),
    ) -> dict[str, object]:
        del script, cpus, memory, extra_mounts
        self.started.append(name)
        self.events.append(name)
        if name == "build":
            self.build_root.mkdir(parents=True)
            for binary in REQUIRED_BINARIES:
                if binary != self.missing_binary:
                    path = self.build_root / binary
                    path.write_bytes(f"ELF fixture: {binary}\n".encode("ascii"))
                    path.chmod(0o755)
        elif kind == "tum-vi":
            self.benchmark_receipt_seen.append(self.receipt_path.is_file())

        if name == "build":
            started_utc = "2026-09-03T00:00:00+00:00"
            finished_utc = "2026-09-03T00:00:00.500000+00:00"
        elif name == "unit":
            started_utc = "2026-09-03T00:00:01+00:00"
            finished_utc = "2026-09-03T00:00:01.500000+00:00"
        else:
            started_utc = "2026-09-03T00:00:03+00:00"
            finished_utc = "2026-09-03T00:00:04+00:00"

        status = {
            "schema_version": remote_ci.SCHEMA_VERSION,
            "name": name,
            "kind": kind,
            "state": "passed",
            "return_code": 0,
            "started_utc": started_utc,
            "finished_utc": finished_utc,
        }
        task_root = Path(str(run["run_root"])) / "tasks" / name
        remote_ci.atomic_write_json(task_root / "status.json", status)
        return status

    def run_worker(self) -> int:
        try:
            with mock.patch.object(
                remote_ci, "profile_definition", return_value=self.definition
            ), mock.patch.object(
                remote_ci,
                "_verify_run_source_identity",
                return_value={},
            ), mock.patch.object(
                remote_ci, "_run_remote_task", side_effect=self.fake_task
            ), mock.patch.object(
                remote_ci,
                "_create_results_archive",
                return_value=(self.root / "results.tar.zst", "8" * 64),
            ), mock.patch.object(
                remote_ci,
                "_collect_tum_vi_runtime_linkage",
                create=True,
                side_effect=self.fake_runtime_linkage,
            ), mock.patch.object(
                remote_ci, "utc_now", return_value=PREFLIGHT_CREATED_UTC
            ):
                return remote_ci.remote_worker(self.root)
        finally:
            self.dataset_root.chmod(0o755)
            sentinel = self.dataset_root / ".phad-dataset.json"
            if sentinel.exists():
                sentinel.chmod(0o644)


class TumViPreflightReceiptTest(unittest.TestCase):
    def test_worker_writes_canonical_receipt_before_first_benchmark(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = WorkerFixture(Path(temporary))

            return_code = fixture.run_worker()

            self.assertEqual(return_code, 0)
            self.assertEqual(fixture.started[:2], ["build", "unit"])
            self.assertEqual(fixture.events[:3], ["build", "unit", "runtime-linkage"])
            self.assertTrue(fixture.receipt_path.is_file())
            self.assertTrue(fixture.benchmark_receipt_seen)
            self.assertTrue(all(fixture.benchmark_receipt_seen))
            self.assertEqual(
                remote_ci.read_json(fixture.receipt_path),
                fixture.expected_receipt(),
            )
            unit_status = remote_ci.read_json(
                fixture.root / "tasks" / "unit" / "status.json"
            )
            first_benchmark_status = remote_ci.read_json(
                fixture.root / "tasks" / "off-1-room1" / "status.json"
            )
            self.assertLessEqual(
                unit_status["finished_utc"], PREFLIGHT_CREATED_UTC
            )
            self.assertLess(
                PREFLIGHT_CREATED_UTC,
                first_benchmark_status["started_utc"],
            )

    def test_preexisting_benchmark_task_prevents_preflight_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = WorkerFixture(Path(temporary))
            (fixture.root / "tasks" / "off-1-room1").mkdir(parents=True)

            return_code = fixture.run_worker()

            self.assertEqual(return_code, 1)
            self.assertEqual(fixture.started, ["build", "unit"])
            self.assertFalse(fixture.receipt_path.exists())
            status = remote_ci.read_json(fixture.root / "run-status.json")
            self.assertEqual(status["state"], "failed")
            self.assertIn("benchmark task", status["failure_reason"])

    def test_runtime_linkage_container_is_exact_and_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = WorkerFixture(Path(temporary))
            task_root = fixture.root / "linkage-task"
            task_root.mkdir()

            command = remote_ci._tum_vi_runtime_linkage_container_command(
                fixture.run, fixture.build_root, task_root
            )

            self.assertIn("--read-only", command)
            self.assertEqual(command[command.index("--network") + 1], "none")
            self.assertIn(
                f"type=bind,src={fixture.source_root},dst=/src,readonly", command
            )
            self.assertIn(
                f"type=bind,src={fixture.build_root},dst=/shared-build,readonly",
                command,
            )
            self.assertIn(
                f"type=bind,src={task_root},dst=/task", command
            )
            self.assertNotIn(
                f"type=bind,src={task_root},dst=/task,readonly", command
            )
            self.assertEqual(
                command[-5:-1],
                [
                    "--rm",
                    fixture.run["image"]["remote_id"],
                    "/opt/phad-venv/bin/python",
                    "-c",
                ],
            )
            self.assertNotIn(fixture.run["image"]["tag"], command)
            collector = command[-1]
            compile(collector, "<tum-vi-runtime-linkage-collector>", "exec")
            self.assertIn('["readelf", "-l", str(binary)]', collector)
            self.assertIn('["ldd", str(binary)]', collector)
            self.assertIn('"missing": sorted(set(missing))', collector)

            mismatched = json.loads(json.dumps(fixture.run))
            mismatched["image"]["remote_id"] = "sha256:" + "a" * 64
            with self.assertRaises(remote_ci.RemoteCiError):
                remote_ci._tum_vi_runtime_linkage_container_command(
                    mismatched, fixture.build_root, task_root
                )

    def test_runtime_linkage_schema_rejects_unbound_or_missing_entries(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = WorkerFixture(Path(temporary))
            fixture.fake_task(
                fixture.run,
                "build",
                "build",
                "build script",
                cpus=16,
                memory="32g",
            )
            valid = fixture.fake_runtime_linkage(fixture.run, fixture.build_root)
            binary_hashes = {
                name: sha256(fixture.build_root / name) for name in REQUIRED_BINARIES
            }
            cases: dict[str, object] = {}
            extra_field = json.loads(json.dumps(valid))
            extra_field["unexpected"] = True
            cases["extra_field"] = extra_field
            missing_library = json.loads(json.dumps(valid))
            missing_library["binaries"]["phad_vo_bench"]["missing"] = [
                "libmissing.so"
            ]
            cases["missing_library"] = missing_library
            missing_binary = json.loads(json.dumps(valid))
            del missing_binary["binaries"]["phad_eval_tests"]
            cases["missing_binary"] = missing_binary
            wrong_binary_hash = json.loads(json.dumps(valid))
            wrong_binary_hash["binaries"]["phad_vo_bench"][
                "binary_sha256"
            ] = "f" * 64
            cases["wrong_binary_hash"] = wrong_binary_hash

            for name, linkage in cases.items():
                with self.subTest(case=name), self.assertRaises(
                    remote_ci.RemoteCiError
                ):
                    remote_ci._validate_tum_vi_runtime_linkage(
                        linkage, binary_hashes
                    )

    def test_rootfs_layer_identity_is_nonempty_typed_and_canonical(self) -> None:
        invalid_layers = (
            None,
            [],
            [7],
            ["not-a-content-id"],
            [ROOTFS_LAYERS[0], ROOTFS_LAYERS[0]],
        )
        for layers in invalid_layers:
            with self.subTest(layers=layers), self.assertRaises(
                remote_ci.RemoteCiError
            ):
                remote_ci._tum_vi_rootfs_layers({"rootfs_layers": layers})

    def test_missing_or_mismatched_preflight_identity_starts_zero_benchmarks(
        self,
    ) -> None:
        cases = (
            "missing_source_id",
            "mismatched_source_id",
            "missing_oci_config_id",
            "mismatched_protocol_sha",
            "missing_analyzer",
            "missing_binary",
            "mismatched_batch_order",
            "mismatched_dataset_manifest_id",
            "mismatched_dataset_receipt",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temporary:
                fixture = WorkerFixture(Path(temporary))
                if case == "missing_source_id":
                    del fixture.run["source_id"]
                    fixture.save_run()
                elif case == "mismatched_source_id":
                    fixture.run["source_id"] = "sha256:" + "9" * 64
                    fixture.save_run()
                elif case == "missing_oci_config_id":
                    del fixture.run["image"]["oci_config_id"]
                    fixture.save_run()
                elif case == "mismatched_protocol_sha":
                    (fixture.source_root / PROTOCOL_PATH).write_text(
                        "changed after source manifest\n", encoding="utf-8"
                    )
                elif case == "missing_analyzer":
                    (fixture.source_root / ANALYZER_PATH).unlink()
                elif case == "missing_binary":
                    fixture.missing_binary = "phad_eval_tests"
                elif case == "mismatched_batch_order":
                    fixture.run["batch_order"] = list(
                        reversed(fixture.run["batch_order"])
                    )
                    fixture.save_run()
                elif case == "mismatched_dataset_manifest_id":
                    fixture.run["dataset_manifest_id"] = "sha256:" + "a" * 64
                    fixture.save_run()
                elif case == "mismatched_dataset_receipt":
                    sentinel = fixture.dataset_root / ".phad-dataset.json"
                    document = remote_ci.read_json(sentinel)
                    document["dataset_content_id"] = "sha256:" + "b" * 64
                    fixture.dataset_root.chmod(0o755)
                    sentinel.chmod(0o644)
                    remote_ci.atomic_write_json(sentinel, document)
                    sentinel.chmod(0o444)
                    fixture.dataset_root.chmod(0o555)

                return_code = fixture.run_worker()

                self.assertEqual(return_code, 1)
                self.assertFalse(
                    any(name not in {"build", "unit"} for name in fixture.started)
                )
                self.assertEqual(fixture.benchmark_receipt_seen, [])
                self.assertFalse(fixture.receipt_path.exists())
                status = remote_ci.read_json(fixture.root / "run-status.json")
                self.assertEqual(status["state"], "failed")
                self.assertTrue(status.get("failure_reason"))

    def test_groundtruth_identity_failure_after_unit_starts_zero_benchmarks(
        self,
    ) -> None:
        cases = (
            "missing_entry",
            "duplicate_entry",
            "crossed_room_path",
            "nonregular_file",
            "sha_mismatch",
            "invalid_sha",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as temporary:
                fixture = WorkerFixture(Path(temporary))
                receipt = json.loads(json.dumps(fixture.dataset_receipt))
                entries = receipt["dataset_tree_manifest"]["entries"]
                expected_path = "room1/dso/gt_imu.csv"
                matches = [
                    entry for entry in entries if entry.get("path") == expected_path
                ]
                self.assertEqual(len(matches), 1)
                if case == "missing_entry":
                    entries.remove(matches[0])
                elif case == "duplicate_entry":
                    entries.append(dict(matches[0]))
                elif case == "crossed_room_path":
                    matches[0]["path"] = "room2/dso/gt_imu.csv"
                elif case == "nonregular_file":
                    groundtruth = fixture.dataset_root / expected_path
                    groundtruth.parent.chmod(0o755)
                    groundtruth.unlink()
                    groundtruth.symlink_to(
                        fixture.dataset_root / "room2" / "dso" / "gt_imu.csv"
                    )
                elif case == "sha_mismatch":
                    matches[0]["sha256"] = "f" * 64
                else:
                    matches[0]["sha256"] = "not-a-sha256"

                with mock.patch.object(
                    remote_ci,
                    "_verify_tum_vi_published_dataset",
                    return_value=receipt,
                ):
                    return_code = fixture.run_worker()

                self.assertEqual(return_code, 1)
                self.assertEqual(fixture.started, ["build", "unit"])
                self.assertEqual(fixture.benchmark_receipt_seen, [])
                self.assertFalse(fixture.receipt_path.exists())
                status = remote_ci.read_json(fixture.root / "run-status.json")
                self.assertEqual(status["state"], "failed")
                self.assertIn("groundtruth", status["failure_reason"])

    def test_tum_unit_script_runs_cpp_and_python_contract_suites(self) -> None:
        script = remote_ci._unit_script(
            {
                "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "unit_parallelism": 8,
            }
        )

        self.assertIn(
            "ctest --test-dir /task/build --output-on-failure -L unit --parallel 8",
            script,
        )
        for filename in (
            "test_remote_ci.py",
            "test_remote_ci_tumvi_stage.py",
            "test_remote_ci_tumvi_preflight.py",
            "test_analyze_tumvi_room_confirmatory.py",
        ):
            with self.subTest(filename=filename):
                self.assertIn(
                    "/opt/phad-venv/bin/python -m unittest discover "
                    f"-s /src/tests/scripts -p '{filename}'",
                    script,
                )


if __name__ == "__main__":
    unittest.main()

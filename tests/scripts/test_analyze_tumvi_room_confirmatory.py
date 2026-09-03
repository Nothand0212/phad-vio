from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from scripts import analyze_tumvi_room_confirmatory as analyzer
from scripts import remote_ci


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYZER = REPO_ROOT / "scripts" / "analyze_tumvi_room_confirmatory.py"
ROOMS = tuple(f"room{index}" for index in range(1, 7))
ROLES = ("off", "on")
REPEATS = (1, 2, 3)

GIT_COMMIT = "0123456789abcdef0123456789abcdef01234567"
GIT_TREE = "123456789abcdef0123456789abcdef012345678"
SOURCE_ARCHIVE_SHA256 = "3" * 64
SOURCE_BUNDLE_SHA256 = "4" * 64
DATASET_CONTENT_ID = analyzer.EXPECTED_DATASET_CONTENT_ID
OCI_MANIFEST_ID = "sha256:" + "7" * 64
OCI_CONFIG_ID = "sha256:" + "8" * 64
TOOLCHAIN_ID = "9" * 64
BINARY_SHA256 = "a" * 64
ROOTFS_LAYERS = ("sha256:" + "b" * 64, "sha256:" + "c" * 64)
PROTOCOL_SHA256 = "b" * 64
ANALYZER_SHA256 = "c" * 64

PREFLIGHT_BINARY_NAMES = (
    "phad_vo_bench",
    "phad_estimator_tests",
    "phad_apps_tests",
    "phad_io_dataset_tests",
    "phad_eval_tests",
    "phad_bench_tests",
)
BATCH_ORDER = (
    ("off", 1),
    ("on", 1),
    ("on", 2),
    ("off", 2),
    ("off", 3),
    ("on", 3),
)
PROTOCOL_PATH = (
    "docs/research/"
    "2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-protocol.md"
)
ANALYZER_PATH = "scripts/analyze_tumvi_room_confirmatory.py"

TIMESTAMPS_NS = tuple(index * 1_000_000_000 for index in range(1, 7))
GT_POINTS = (
    (1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (0.0, 0.0, -1.0),
)

DIAG_HEADER = (
    "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
    "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
    "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
    "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
    "is_keyframe,num_disparity,unsupported_span_ns,num_retained_observations,"
    "num_seeded_landmarks,num_current_visual_factors,num_mapped_observations,"
    "num_current_mono_visual_factors"
)


def error_stats(values: tuple[float, ...]) -> dict[str, float]:
    ordered = sorted(values)
    count = len(values)
    mean = sum(values) / count
    mean_square = sum(value * value for value in values) / count
    if count % 2:
        median = ordered[count // 2]
    else:
        median = (ordered[count // 2 - 1] + ordered[count // 2]) / 2.0
    return {
        "rmse": math.sqrt(mean_square),
        "mean": mean,
        "median": median,
        "stddev": math.sqrt(max(0.0, mean_square - mean * mean)),
        "max": max(values),
    }


def metrics_for_scale(scale_x: float, scale_y: float) -> dict[str, object]:
    errors = tuple(
        ((scale_x - 1.0) * point[0], (scale_y - 1.0) * point[1], 0.0)
        for point in GT_POINTS
    )
    ate = tuple(math.sqrt(x * x + y * y + z * z) for x, y, z in errors)
    rpe = tuple(
        math.sqrt(
            (errors[index + 1][0] - errors[index][0]) ** 2
            + (errors[index + 1][1] - errors[index][1]) ** 2
            + (errors[index + 1][2] - errors[index][2]) ** 2
        )
        for index in range(len(errors) - 1)
    )
    zero_ate = tuple(0.0 for _ in ate)
    zero_rpe = tuple(0.0 for _ in rpe)
    return {
        "ate": {
            "trans": error_stats(ate),
            "rot_deg": error_stats(zero_ate),
        },
        "rpe": {
            "trans": error_stats(rpe),
            "rot_deg": error_stats(zero_rpe),
        },
    }


def canonical_json_sha256(path: Path, excluded_key: str) -> str:
    value = json.loads(path.read_text(encoding="utf-8"))
    value.pop(excluded_key)
    canonical = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_id(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def config_hash8(text: str) -> str:
    value = 14_695_981_039_346_656_037
    for byte in text.encode("utf-8"):
        value ^= byte
        value = (value * 1_099_511_628_211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"[:8]


def benchmark_command(room: str, role: str) -> str:
    activation = (
        "  --estimator-enable-pnp-cheirality-retry \\\n" if role == "on" else ""
    )
    return f"""set -euo pipefail
umask 022
/shared-build/phad_vo_bench /datasets/tum-vi/{room} \\
  --dataset-format tum-vi \\
  --gt-tum-vi /datasets/tum-vi/{room} \\
  --out /task/artifacts/bench \\
  --sequence-name {room} \\
  --config-label default \\
  --repo /src \\
  --max-dt-ms 4.166667 \\
  --min-match-rate 0.80 \\
  --rpe-delta-s 1.0 \\
  --estimator-enable-moving-bootstrap \\
{activation}  --probe-b /task/artifacts/bench/probe-b.jsonl \\
  --force
"""


def build_command() -> str:
    return "\n".join(
        (
            "set -euo pipefail",
            "umask 022",
            "cmake -S /src -B /task/build -G Ninja "
            "-DCMAKE_BUILD_TYPE=Release -DPHAD_BUILD_TESTS=ON "
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            "cmake --build /task/build --parallel 16",
        )
    )


def unit_command() -> str:
    commands = [
        "set -euo pipefail",
        "umask 022",
        "cmake -E copy_directory /shared-build /task/build",
        "ctest --test-dir /task/build --output-on-failure -L unit --parallel 8",
    ]
    for filename in (
        "test_remote_ci.py",
        "test_remote_ci_tumvi_stage.py",
        "test_remote_ci_tumvi_preflight.py",
        "test_analyze_tumvi_room_confirmatory.py",
    ):
        commands.append(
            "/opt/phad-venv/bin/python -m unittest discover "
            f"-s /src/tests/scripts -p '{filename}'"
        )
    return "\n".join(commands)


def groundtruth_payload() -> str:
    rows = ["# timestamp[ns],tx,ty,tz,qw,qx,qy,qz"]
    for timestamp_ns, point in zip(TIMESTAMPS_NS, GT_POINTS):
        rows.append(
            f"{timestamp_ns},{point[0]},{point[1]},{point[2]},1,0,0,0"
        )
    return "\n".join(rows) + "\n"


class ConfirmatoryFixture:
    def __init__(self, root: Path) -> None:
        self.run_root = root / "run"
        self.gt_root = root / "tum-vi"
        self.output = root / "analysis.json"
        self.write()

    def bench(self, room: str, role: str, repeat: int) -> Path:
        return (
            self.run_root
            / "tasks"
            / f"{role}-{repeat}-{room}"
            / "artifacts"
            / "bench"
        )

    def write(self) -> None:
        protocol_sha256 = file_sha256(REPO_ROOT / PROTOCOL_PATH)
        analyzer_sha256 = file_sha256(REPO_ROOT / ANALYZER_PATH)
        gt_payload = groundtruth_payload()
        gt_sha256 = hashlib.sha256(gt_payload.encode("utf-8")).hexdigest()
        source_manifest = {
            "schema_version": 1,
            "git": {
                "head": GIT_COMMIT,
                "tree": GIT_TREE,
                "branch": "codex/m5-product-recut-design",
                "tracked_dirty": False,
            },
            "entries": [
                {
                    "path": PROTOCOL_PATH,
                    "type": "file",
                    "mode": "0644",
                    "sha256": protocol_sha256,
                    "size": (REPO_ROOT / PROTOCOL_PATH).stat().st_size,
                },
                {
                    "path": ANALYZER_PATH,
                    "type": "file",
                    "mode": "0755",
                    "sha256": analyzer_sha256,
                    "size": (REPO_ROOT / ANALYZER_PATH).stat().st_size,
                },
            ],
            "deleted_tracked": [],
            "included_untracked": 0,
            "excluded_prefixes": [],
        }
        source_id = canonical_id(source_manifest)
        dataset_manifest = remote_ci.profile_definition(
            remote_ci.TUM_VI_CONFIRMATORY_PROFILE
        )["dataset_manifest"]
        dataset_manifest_id = canonical_id(dataset_manifest)
        run = {
            "schema_version": 1,
            "run_id": "tumvi-confirmatory-fixture",
            "profile": "ci-tumvi-room6-confirmatory",
            "source_id": source_id,
            "source_manifest": source_manifest,
            "source_archive_sha256": SOURCE_ARCHIVE_SHA256,
            "source_bundle_sha256": SOURCE_BUNDLE_SHA256,
            "dataset_format": "tum-vi",
            "dataset_root": analyzer.EXPECTED_DATASET_ROOT,
            "dataset_manifest": dataset_manifest,
            "dataset_manifest_id": dataset_manifest_id,
            "dataset_content_id": DATASET_CONTENT_ID,
            "sequences": list(ROOMS),
            "batch_order": [list(item) for item in BATCH_ORDER],
            "build_parallelism": 16,
            "unit_parallelism": 8,
            "image": {
                "oci_manifest_id": OCI_MANIFEST_ID,
                "oci_config_id": OCI_CONFIG_ID,
                "id": OCI_CONFIG_ID,
                "remote_id": OCI_MANIFEST_ID,
                "toolchain_id": TOOLCHAIN_ID,
                "rootfs_layers": list(ROOTFS_LAYERS),
            },
        }
        self.run_root.mkdir(parents=True)
        (self.run_root / "run.json").write_text(
            json.dumps(run, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        receipt = {
            "schema_version": 1,
            "profile": "ci-tumvi-room6-confirmatory",
            "created_utc": "2026-09-03T00:00:20Z",
            "source_id": source_id,
            "dataset": {
                "manifest_id": dataset_manifest_id,
                "content_id": DATASET_CONTENT_ID,
                "receipt_sha256": "d" * 64,
                "groundtruth": {
                    room: {
                        "path": f"{room}/dso/gt_imu.csv",
                        "sha256": gt_sha256,
                    }
                    for room in ROOMS
                },
            },
            "image": {
                "oci_manifest_id": OCI_MANIFEST_ID,
                "oci_config_id": OCI_CONFIG_ID,
                "toolchain_id": TOOLCHAIN_ID,
                "rootfs_layers": list(ROOTFS_LAYERS),
            },
            "inputs": {
                "protocol": {
                    "path": PROTOCOL_PATH,
                    "sha256": protocol_sha256,
                },
                "analyzer": {
                    "path": ANALYZER_PATH,
                    "sha256": analyzer_sha256,
                },
            },
            "binaries": {
                name: BINARY_SHA256
                for name in PREFLIGHT_BINARY_NAMES
            },
            "runtime_linkage": {
                "schema_version": 1,
                "method": "readelf-ldd-sha256",
                "binaries": {
                    name: {
                        "binary_sha256": BINARY_SHA256,
                        "interpreter": {
                            "path": "/lib64/ld-linux-x86-64.so.2",
                            "realpath": (
                                "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2"
                            ),
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
                    for index, name in enumerate(PREFLIGHT_BINARY_NAMES, 1)
                },
            },
            "batch_order": [list(item) for item in BATCH_ORDER],
            "benchmark_commands": {
                role: {
                    room: benchmark_command(room, role) for room in ROOMS
                }
                for role in ROLES
            },
        }
        receipt["preflight_receipt_id"] = canonical_id(receipt)
        (self.run_root / "preflight-receipt.json").write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        for room in ROOMS:
            dso = self.gt_root / room / "dso"
            dso.mkdir(parents=True)
            (dso / "gt_imu.csv").write_text(
                gt_payload, encoding="utf-8"
            )

            for role in ROLES:
                for repeat in REPEATS:
                    self._write_task(room, role, repeat)
        self._write_statuses()

    def _write_statuses(self) -> None:
        statuses: dict[str, dict[str, object]] = {}
        for name, kind, command, started, finished in (
            (
                "build",
                "build",
                build_command(),
                "2026-09-03T00:00:00Z",
                "2026-09-03T00:00:10Z",
            ),
            (
                "unit",
                "unit",
                unit_command(),
                "2026-09-03T00:00:10Z",
                "2026-09-03T00:00:20Z",
            ),
        ):
            status = {
                "schema_version": 1,
                "name": name,
                "kind": kind,
                "state": "passed",
                "return_code": 0,
                "command": command,
                "failure_reason": None,
                "started_utc": started,
                "finished_utc": finished,
            }
            task_root = self.run_root / "tasks" / name
            task_root.mkdir(parents=True)
            (task_root / "status.json").write_text(
                json.dumps(status, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            statuses[name] = status
        for role in ROLES:
            for repeat in REPEATS:
                batch_index = BATCH_ORDER.index((role, repeat))
                started = f"2026-09-03T00:00:{21 + 2 * batch_index:02d}Z"
                finished = f"2026-09-03T00:00:{22 + 2 * batch_index:02d}Z"
                for room in ROOMS:
                    name = f"{role}-{repeat}-{room}"
                    status = {
                        "schema_version": 1,
                        "name": name,
                        "kind": "tum-vi",
                        "state": "passed",
                        "return_code": 0,
                        "command": benchmark_command(room, role),
                        "benchmark_status": "completed",
                        "failure_reason": None,
                        "started_utc": started,
                        "finished_utc": finished,
                    }
                    path = self.run_root / "tasks" / name / "status.json"
                    path.write_text(
                        json.dumps(status, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                    statuses[name] = status
        run_status = {
            "schema_version": 1,
            "run_id": "tumvi-confirmatory-fixture",
            "profile": "ci-tumvi-room6-confirmatory",
            "state": "passed",
            "started_utc": "2026-09-03T00:00:00Z",
            "updated_utc": "2026-09-03T00:00:33Z",
            "finished_utc": "2026-09-03T00:00:33Z",
            "tasks": statuses,
        }
        (self.run_root / "run-status.json").write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _write_task(self, room: str, role: str, repeat: int) -> None:
        bench = self.bench(room, role, repeat)
        bench.mkdir(parents=True)

        scale_x, scale_y = (1.10, 0.90) if role == "off" else (1.05, 0.95)
        tum_rows = []
        for timestamp_ns, point in zip(TIMESTAMPS_NS, GT_POINTS):
            tum_rows.append(
                f"{timestamp_ns / 1e9:.9f} {point[0] * scale_x:.9f} "
                f"{point[1] * scale_y:.9f} {point[2]:.9f} 0 0 0 1"
            )
        tum = "\n".join(tum_rows) + "\n"
        (bench / "est.tum").write_text(tum, encoding="utf-8")
        (bench / "kf.tum").write_text(tum, encoding="utf-8")

        diag_rows = [DIAG_HEADER]
        for timestamp_ns in TIMESTAMPS_NS:
            diag_rows.append(
                f"{timestamp_ns},ok,20,20,20,0,10,0,1,0.5,0,2,0,1,0,0,0,"
                "0.5,1,20,0,20,0,20,20,0"
            )
        (bench / "diag.csv").write_text(
            "\n".join(diag_rows) + "\n", encoding="utf-8"
        )

        probe_rows = []
        for index, timestamp_ns in enumerate(TIMESTAMPS_NS):
            eligible = role == "on" and index == 0
            probe_rows.append(
                json.dumps(
                    {
                        "i": index,
                        "ts_ns": timestamp_ns,
                        "pnp_attempted": eligible,
                        "pnp_correspondences": 20 if eligible else 0,
                        "pnp_inliers": 20 if eligible else 0,
                        "pnp_result": (
                            "accepted_after_guess_retry"
                            if eligible
                            else "not_attempted"
                        ),
                        "pnp_guess_retry_outcome": (
                            "would_pass_existing_pnp_gates"
                            if eligible
                            else "not_attempted"
                        ),
                        "shared": 20,
                        "num_obs": 20,
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
        (bench / "probe-b.jsonl").write_text(
            "\n".join(probe_rows) + "\n", encoding="utf-8"
        )

        canonical_config = (
            "estimator.enable_moving_bootstrap=true\n"
            "session.dataset_format=tum_vi\n"
            if role == "off"
            else "estimator.enable_moving_bootstrap=true\n"
            "estimator.enable_pnp_cheirality_retry=true\n"
            "session.dataset_format=tum_vi\n"
        )
        config = {
            "estimator.enable_moving_bootstrap": True,
            "session.dataset_format": "tum_vi",
        }
        if role == "on":
            config["estimator.enable_pnp_cheirality_retry"] = True
        config_hash = config_hash8(canonical_config)
        batch_index = BATCH_ORDER.index((role, repeat))
        meta = {
            "schema_version": 2,
            "created_utc": f"2026-09-03T00:00:{21 + 2 * batch_index:02d}Z",
            "sequence": room,
            "config_label": "default",
            "config_hash": config_hash,
            "config_canonical_text": canonical_config,
            "config": config,
            "code": {
                "git_commit": GIT_COMMIT,
                "git_commit_short": GIT_COMMIT[:7],
                "git_branch": "codex/m5-product-recut-design",
                "git_dirty": False,
            },
            "sequence_root": f"/datasets/tum-vi/{room}",
            "paths": {
                "bench_root": "",
                "output_dir": "/task/artifacts/bench",
                "layout_template": "override",
            },
            "warnings": [],
        }
        (bench / "meta.json").write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        summary = {
            "schema_version": 2,
            "sequence": room,
            "status": "completed",
            "config_label": "default",
            "config_hash": config_hash,
            "code": {
                "git_commit_short": GIT_COMMIT[:7],
                "git_dirty": False,
            },
            **metrics_for_scale(scale_x, scale_y),
            "trajectory": {
                "image_frames": 6,
                "poses_written": 6,
                "ok": 6,
                "completion_rate": 1.0,
                "coverage_rate": 1.0,
                "failed": 0,
                "rejected": 0,
                "segments": 1,
                "total_keyframes": 6,
                "total_track_only_frames": 0,
            },
            "robustness": {
                "failed": 0,
                "rejected": 0,
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
            # Timing differs across repeats but is excluded from exact payload.
            "timing": {
                "wall_s": 10.0 + repeat,
                "rtf": 1.0 + repeat / 10.0,
                "rectify": {"mean": 0.1, "p95": 0.2, "max": 0.3},
                "frontend": {"mean": 0.4, "p95": 0.5, "max": 0.6},
                "estimator": {"mean": 0.7, "p95": 0.8, "max": 0.9},
                "total": {"mean": 2.0, "p95": 3.0, "max": 4.0},
            },
            "sync": {
                "pushed_left": 6,
                "pushed_right": 6,
                "emitted_stereo": 6,
                "dropped_left": 0,
                "dropped_right": 0,
                "dropped_left_overflow": 0,
                "dropped_right_overflow": 0,
                "max_left_queue": 1,
                "max_right_queue": 1,
            },
            "warnings": [],
        }
        (bench / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def run(self) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(ANALYZER),
                str(self.run_root),
                "--gt-root",
                str(self.gt_root),
                "--output",
                str(self.output),
            ],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def result(self) -> dict[str, object]:
        return json.loads(self.output.read_text(encoding="utf-8"))

    def mutate_all(self, role: str, relative: str, mutate) -> None:
        for room in ROOMS:
            for repeat in REPEATS:
                path = self.bench(room, role, repeat) / relative
                mutate(path)

    def sync_summary_metrics(self, room: str, role: str, repeat: int) -> None:
        bench = self.bench(room, role, repeat)
        estimate = analyzer.read_tum(bench / "est.tum")
        groundtruth = analyzer.read_tumvi_groundtruth(self.gt_root / room)
        ate = analyzer.compute_ate(estimate, groundtruth)
        rpe = analyzer.compute_rpe(estimate, groundtruth)
        summary = json.loads((bench / "summary.json").read_text(encoding="utf-8"))

        def summary_stats(values) -> dict[str, float]:
            stats = analyzer._stats(values)
            return {
                key: stats[key]
                for key in ("rmse", "mean", "median", "stddev", "max")
            }

        summary["ate"] = {
            "trans": summary_stats(ate.trans_m),
            "rot_deg": summary_stats(ate.rot_deg),
        }
        summary["rpe"] = {
            "trans": summary_stats(rpe.trans_m),
            "rot_deg": summary_stats(rpe.rot_deg),
        }
        (bench / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def replace_timestamp(
        self, room: str, role: str, repeat: int, old_ns: int, new_ns: int
    ) -> None:
        bench = self.bench(room, role, repeat)
        old_seconds = f"{old_ns / 1e9:.9f} "
        new_seconds = f"{new_ns / 1e9:.9f} "
        for filename in ("est.tum", "kf.tum"):
            path = bench / filename
            path.write_text(
                path.read_text(encoding="utf-8").replace(
                    old_seconds, new_seconds
                ),
                encoding="utf-8",
            )
        diag = bench / "diag.csv"
        diag.write_text(
            diag.read_text(encoding="utf-8").replace(
                f"{old_ns},", f"{new_ns},"
            ),
            encoding="utf-8",
        )
        probe = bench / "probe-b.jsonl"
        rows = []
        for line in probe.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            if row["ts_ns"] == old_ns:
                row["ts_ns"] = new_ns
            rows.append(json.dumps(row, sort_keys=True, separators=(",", ":")))
        probe.write_text("\n".join(rows) + "\n", encoding="utf-8")
        self.sync_summary_metrics(room, role, repeat)

    def mark_last_frame_failed(self, room: str, role: str, repeat: int) -> None:
        bench = self.bench(room, role, repeat)
        diag_path = bench / "diag.csv"
        lines = diag_path.read_text(encoding="utf-8").splitlines()
        fields = lines[-1].split(",")
        fields[1] = "failed"
        fields[18] = "0"
        lines[-1] = ",".join(fields)
        diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        for filename in ("est.tum", "kf.tum"):
            path = bench / filename
            rows = path.read_text(encoding="utf-8").splitlines()
            path.write_text("\n".join(rows[:-1]) + "\n", encoding="utf-8")
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["trajectory"].update(
            {
                "ok": 5,
                "failed": 1,
                "poses_written": 5,
                "total_keyframes": 5,
                "total_track_only_frames": 1,
                "completion_rate": 5.0 / 6.0,
                "coverage_rate": 4.0 / 5.0,
            }
        )
        summary["robustness"]["failed"] = 1
        summary["status"] = "completed_with_failures"
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.sync_summary_metrics(room, role, repeat)
        task_name = f"{role}-{repeat}-{room}"
        task_status_path = self.run_root / "tasks" / task_name / "status.json"
        task_status = json.loads(task_status_path.read_text(encoding="utf-8"))
        task_status.update(
            {
                "state": "warning",
                "benchmark_status": "completed_with_failures",
                "failure_reason": (
                    "benchmark robustness.failed=1, robustness.rejected=0"
                ),
            }
        )
        task_status_path.write_text(
            json.dumps(task_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        run_status_path = self.run_root / "run-status.json"
        run_status = json.loads(run_status_path.read_text(encoding="utf-8"))
        run_status["state"] = "warning"
        run_status["tasks"][task_name] = task_status
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def make_typed_fatal(
        self,
        room: str,
        role: str,
        repeat: int,
        *,
        keep_one_pose: bool,
        fatal_status: str = "failed",
    ) -> None:
        bench = self.bench(room, role, repeat)
        diag_path = bench / "diag.csv"
        original_rows = diag_path.read_text(encoding="utf-8").splitlines()
        data_rows = original_rows[1:2] if not keep_one_pose else original_rows[1:3]
        fatal = data_rows[-1].split(",")
        fatal[1] = fatal_status
        fatal[18] = "0"
        data_rows[-1] = ",".join(fatal)
        diag_path.write_text(
            "\n".join([original_rows[0], *data_rows]) + "\n", encoding="utf-8"
        )
        kept_pose_rows = 1 if keep_one_pose else 0
        for filename in ("est.tum", "kf.tum"):
            path = bench / filename
            rows = path.read_text(encoding="utf-8").splitlines()
            payload = "\n".join(rows[:kept_pose_rows])
            path.write_text(payload + ("\n" if payload else ""), encoding="utf-8")
        probe_path = bench / "probe-b.jsonl"
        probe_rows = probe_path.read_text(encoding="utf-8").splitlines()
        kept_probe_rows = 2 if keep_one_pose else 1
        normalized: list[str] = []
        for line in probe_rows[:kept_probe_rows]:
            row = json.loads(line)
            row.update(
                {
                    "pnp_attempted": False,
                    "pnp_correspondences": 0,
                    "pnp_inliers": 0,
                    "pnp_result": "not_attempted",
                    "pnp_guess_retry_outcome": "not_attempted",
                }
            )
            normalized.append(json.dumps(row, sort_keys=True, separators=(",", ":")))
        probe_path.write_text("\n".join(normalized) + "\n", encoding="utf-8")
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        image_frames = len(data_rows)
        summary.update(
            {
                "status": "failed",
                "ate": None,
                "rpe": None,
                "warnings": ["estimator reported a typed fatal failure"],
            }
        )
        summary["trajectory"].update(
            {
                "image_frames": image_frames,
                "poses_written": kept_pose_rows,
                "ok": kept_pose_rows,
                "failed": 1,
                "rejected": 0,
                "completion_rate": kept_pose_rows / image_frames,
                "coverage_rate": 0.0,
                "segments": 1 if keep_one_pose else 0,
                "total_keyframes": kept_pose_rows,
                "total_track_only_frames": image_frames - kept_pose_rows,
            }
        )
        summary["robustness"].update({"failed": 1, "rejected": 0})
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        task_name = f"{role}-{repeat}-{room}"
        task_path = self.run_root / "tasks" / task_name / "status.json"
        task = json.loads(task_path.read_text(encoding="utf-8"))
        task.update(
            {
                "state": "warning",
                "return_code": 1,
                "benchmark_status": "failed",
                "failure_reason": (
                    "benchmark robustness.failed=1, robustness.rejected=0"
                ),
            }
        )
        task_path.write_text(
            json.dumps(task, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status_path = self.run_root / "run-status.json"
        run_status = json.loads(run_status_path.read_text(encoding="utf-8"))
        run_status["state"] = "warning"
        run_status["tasks"][task_name] = task
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def rewrite_role_scale(self, role: str, scale_x: float, scale_y: float) -> None:
        for room in ROOMS:
            for repeat in REPEATS:
                self.rewrite_run_scale(room, role, repeat, scale_x, scale_y)

    def rewrite_run_scale(
        self,
        room: str,
        role: str,
        repeat: int,
        scale_x: float,
        scale_y: float,
    ) -> None:
        bench = self.bench(room, role, repeat)
        rows = []
        for timestamp_ns, point in zip(TIMESTAMPS_NS, GT_POINTS):
            rows.append(
                f"{timestamp_ns / 1e9:.9f} "
                f"{point[0] * scale_x:.9f} "
                f"{point[1] * scale_y:.9f} {point[2]:.9f} 0 0 0 1"
            )
        payload = "\n".join(rows) + "\n"
        for filename in ("est.tum", "kf.tum"):
            (bench / filename).write_text(payload, encoding="utf-8")
        self.sync_summary_metrics(room, role, repeat)

    def drop_first_pose(self, room: str, role: str, repeat: int) -> None:
        bench = self.bench(room, role, repeat)
        diag_path = bench / "diag.csv"
        diag_lines = diag_path.read_text(encoding="utf-8").splitlines()
        first = diag_lines[1].split(",")
        first[1] = "initializing"
        first[18] = "0"
        diag_lines[1] = ",".join(first)
        diag_path.write_text("\n".join(diag_lines) + "\n", encoding="utf-8")
        for filename in ("est.tum", "kf.tum"):
            path = bench / filename
            rows = path.read_text(encoding="utf-8").splitlines()
            path.write_text("\n".join(rows[1:]) + "\n", encoding="utf-8")
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["trajectory"].update(
            {
                "ok": 5,
                "poses_written": 5,
                "total_keyframes": 5,
                "total_track_only_frames": 1,
                "completion_rate": 5.0 / 6.0,
                "coverage_rate": 4.0 / 5.0,
            }
        )
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.sync_summary_metrics(room, role, repeat)


class TumViRoomConfirmatoryAnalyzerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.fixture = ConfirmatoryFixture(Path(self.temporary.name))

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def assert_completed(self, completed: subprocess.CompletedProcess[str]) -> None:
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue(self.fixture.output.is_file(), completed.stdout)

    def test_recomputes_native_metrics_and_emits_pass_aggregate(self) -> None:
        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(
            set(result),
            {
                "schema_version",
                "verdict",
                "reasons",
                "repeatability",
                "sequences",
                "suite",
            },
        )
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["verdict"], "PASS")
        self.assertEqual(result["reasons"], [])

        expected_hashes = {
            "est.tum": file_sha256(
                self.fixture.bench("room1", "off", 1) / "est.tum"
            ),
            "diag.csv": file_sha256(
                self.fixture.bench("room1", "off", 1) / "diag.csv"
            ),
            "kf.tum": file_sha256(
                self.fixture.bench("room1", "off", 1) / "kf.tum"
            ),
            "probe-b.jsonl": file_sha256(
                self.fixture.bench("room1", "off", 1) / "probe-b.jsonl"
            ),
            "meta_without_created_utc": canonical_json_sha256(
                self.fixture.bench("room1", "off", 1) / "meta.json",
                "created_utc",
            ),
            "summary_without_timing": canonical_json_sha256(
                self.fixture.bench("room1", "off", 1) / "summary.json",
                "timing",
            ),
        }
        repeatability = result["repeatability"]["room1"]["off"]
        self.assertTrue(repeatability["exact"])
        self.assertEqual(
            repeatability["hashes"],
            {key: [value, value, value] for key, value in expected_hashes.items()},
        )

        off = result["sequences"]["room1"]["roles"]["off"]
        on = result["sequences"]["room1"]["roles"]["on"]
        self.assertAlmostEqual(
            off["native_median"]["ate_trans_rmse_m"], math.sqrt(1.0 / 150.0)
        )
        self.assertAlmostEqual(
            on["native_median"]["ate_trans_rmse_m"], math.sqrt(1.0 / 600.0)
        )
        self.assertAlmostEqual(
            off["native_median"]["rpe_trans_rmse_m"], math.sqrt(0.11 / 5.0)
        )
        self.assertAlmostEqual(
            on["native_median"]["rpe_trans_rmse_m"], math.sqrt(0.11 / 20.0)
        )
        for role in (off, on):
            self.assertEqual(role["native_median"]["ate_rot_rmse_deg"], 0.0)
            self.assertEqual(role["native_median"]["rpe_rot_rmse_deg"], 0.0)
            self.assertEqual(role["native_median"]["association_count"], 6)
            self.assertEqual(role["native_median"]["rpe_pair_count"], 5)
            self.assertEqual(role["native_median"]["out_of_range"], 0)
            self.assertEqual(role["native_median"]["over_threshold"], 0)
            self.assertEqual(role["native_median"]["dropped_no_partner"], 1)

        self.assertEqual(
            on["retry"],
            {
                "eligible": 1,
                "attempted": 1,
                "accepted": 1,
                "failed": 0,
                "fallback": 0,
            },
        )
        paired = result["sequences"]["room1"]["paired_repeats"][0]
        self.assertEqual(paired["repeat"], 1)
        self.assertEqual(
            paired["support"]["timestamps"],
            {"off": 6, "on": 6, "intersection": 6, "off_only": 0, "on_only": 0},
        )
        self.assertEqual(
            paired["support"]["rpe_pairs"],
            {"off": 5, "on": 5, "intersection": 5, "off_only": 0, "on_only": 0},
        )
        expected_distribution_keys = {
            "count",
            "rmse",
            "mean",
            "median",
            "p10",
            "p90",
            "max",
        }
        self.assertEqual(
            set(paired["common"]["ate"]["translation_delta_m"]),
            expected_distribution_keys,
        )
        self.assertEqual(
            set(paired["common"]["ate"]["rotation_delta_deg"]),
            expected_distribution_keys,
        )
        self.assertEqual(
            set(paired["common"]["rpe"]["translation_delta_m"]),
            expected_distribution_keys,
        )
        self.assertEqual(
            set(paired["common"]["rpe"]["rotation_delta_deg"]),
            expected_distribution_keys,
        )
        self.assertEqual(paired["common"]["ate"]["translation_delta_m"]["count"], 6)
        self.assertEqual(paired["common"]["rpe"]["translation_delta_m"]["count"], 5)
        self.assertTrue(paired["tail"]["ate_translation"])
        self.assertTrue(paired["tail"]["rpe_translation"])

        suite = result["suite"]
        self.assertAlmostEqual(suite["g_ate"], 0.5)
        self.assertAlmostEqual(suite["g_rpe"], 0.5)
        self.assertAlmostEqual(suite["mean_completion_delta"], 0.0)
        self.assertAlmostEqual(suite["mean_coverage_delta"], 0.0)
        self.assertEqual(suite["failed_total"], 0)
        self.assertEqual(suite["rejected_total"], 0)
        self.assertTrue(suite["segments_nonincrease"])
        self.assertEqual(suite["eligible_retry_total"], 6)

    def test_common_support_only_samples_trigger_review(self) -> None:
        for repeat in REPEATS:
            self.fixture.replace_timestamp(
                "room1", "on", repeat, 6_000_000_000, 6_001_000_000
            )

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "REVIEW")
        for paired in result["sequences"]["room1"]["paired_repeats"]:
            self.assertEqual(
                paired["support"]["timestamps"],
                {
                    "off": 6,
                    "on": 6,
                    "intersection": 5,
                    "off_only": 1,
                    "on_only": 1,
                },
            )
            self.assertGreater(paired["support"]["rpe_pairs"]["off_only"], 0)
            self.assertGreater(paired["support"]["rpe_pairs"]["on_only"], 0)

    def test_suite_availability_regression_is_fail(self) -> None:
        for room in ROOMS:
            for repeat in REPEATS:
                self.fixture.drop_first_pose(room, "on", repeat)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertAlmostEqual(result["suite"]["mean_completion_delta"], -1.0 / 6.0)
        self.assertAlmostEqual(result["suite"]["mean_coverage_delta"], -0.2)

    def test_completion_and_coverage_summary_tamper_is_input_error(self) -> None:
        path = self.fixture.bench("room1", "on", 1) / "summary.json"
        original = path.read_bytes()
        for key in ("completion_rate", "coverage_rate"):
            with self.subTest(key=key):
                summary = json.loads(original)
                summary["trajectory"][key] = 0.99
                path.write_text(
                    json.dumps(summary, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertFalse(self.fixture.output.exists())
                self.assertIn(key, completed.stderr)
                path.write_bytes(original)

    def test_run_task_and_dataset_identities_are_bound(self) -> None:
        meta_path = self.fixture.bench("room2", "on", 2) / "meta.json"
        original_meta = meta_path.read_bytes()
        meta = json.loads(original_meta)
        meta["code"]["git_commit"] = "f" * 40
        meta_path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("code identity", completed.stderr)
        meta_path.write_bytes(original_meta)

        meta = json.loads(original_meta)
        meta["sequence_root"] = "/datasets/tum-vi/room3"
        meta_path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("dataset input identity", completed.stderr)
        meta_path.write_bytes(original_meta)

        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        del receipt["binaries"]["phad_vo_bench"]
        receipt_without_id = dict(receipt)
        receipt_without_id.pop("preflight_receipt_id")
        receipt["preflight_receipt_id"] = canonical_id(receipt_without_id)
        receipt_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("binary identity", completed.stderr)

    def test_source_build_input_and_receipt_identities_are_strict(self) -> None:
        run_path = self.fixture.run_root / "run.json"
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        original_run = run_path.read_bytes()
        original_receipt = receipt_path.read_bytes()

        def write_receipt(value: dict[str, object]) -> None:
            without_id = dict(value)
            without_id.pop("preflight_receipt_id", None)
            value["preflight_receipt_id"] = canonical_id(without_id)
            receipt_path.write_text(
                json.dumps(value, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

        cases = ("source", "build", "input", "preflight")
        for case in cases:
            with self.subTest(case=case):
                run_path.write_bytes(original_run)
                receipt_path.write_bytes(original_receipt)
                if case == "source":
                    run = json.loads(original_run)
                    run["source_id"] = "sha256:" + "f" * 64
                    run_path.write_text(
                        json.dumps(run, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                else:
                    receipt = json.loads(original_receipt)
                    if case == "build":
                        receipt["image"]["toolchain_id"] = "e" * 64
                        write_receipt(receipt)
                    elif case == "input":
                        receipt["inputs"]["analyzer"]["sha256"] = "e" * 64
                        write_receipt(receipt)
                    else:
                        receipt["preflight_receipt_id"] = "sha256:" + "e" * 64
                        receipt_path.write_text(
                            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8",
                        )
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(case, completed.stderr)

    def test_run_and_task_statuses_must_be_successful_and_bound(self) -> None:
        run_status_path = self.fixture.run_root / "run-status.json"
        original_run_status = run_status_path.read_bytes()
        task_name = "on-2-room3"
        task_status_path = self.fixture.run_root / "tasks" / task_name / "status.json"
        original_task_status = task_status_path.read_bytes()

        cases = ("run terminal", "task return_code", "task command")
        for case in cases:
            with self.subTest(case=case):
                run_status_path.write_bytes(original_run_status)
                task_status_path.write_bytes(original_task_status)
                run_status = json.loads(original_run_status)
                if case == "run terminal":
                    run_status["state"] = "failed"
                else:
                    task_status = json.loads(original_task_status)
                    if case == "task return_code":
                        task_status.update({"state": "failed", "return_code": 42})
                    else:
                        task_status["command"] = "unexpected command"
                    task_status_path.write_text(
                        json.dumps(task_status, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                    run_status["tasks"][task_name] = task_status
                run_status_path.write_text(
                    json.dumps(run_status, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(case.split()[0], completed.stderr)

        task_status_path.write_bytes(original_task_status)
        build_path = self.fixture.run_root / "tasks" / "build" / "status.json"
        build = json.loads(build_path.read_text(encoding="utf-8"))
        build.update({"state": "failed", "return_code": 42})
        build_path.write_text(
            json.dumps(build, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status = json.loads(original_run_status)
        run_status["tasks"]["build"] = build
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("build", completed.stderr)

    def test_receipt_batch_and_benchmark_commands_are_canonical(self) -> None:
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        original = receipt_path.read_bytes()
        for case in ("batch_order", "benchmark_commands"):
            with self.subTest(case=case):
                receipt = json.loads(original)
                if case == "batch_order":
                    receipt[case][0] = ["on", 1]
                else:
                    receipt[case]["on"]["room1"] = "unexpected command"
                without_id = dict(receipt)
                without_id.pop("preflight_receipt_id")
                receipt["preflight_receipt_id"] = canonical_id(without_id)
                receipt_path.write_text(
                    json.dumps(receipt, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(case, completed.stderr)
                receipt_path.write_bytes(original)

    def test_current_analyzer_and_groundtruth_content_are_bound(self) -> None:
        gt_path = self.fixture.gt_root / "room1" / "dso" / "gt_imu.csv"
        rows = gt_path.read_text(encoding="utf-8").splitlines()
        translated = [rows[0]]
        for row in rows[1:]:
            fields = row.split(",")
            fields[1] = str(float(fields[1]) + 100.0)
            translated.append(",".join(fields))
        gt_path.write_text("\n".join(translated) + "\n", encoding="utf-8")

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertIn("groundtruth", completed.stderr)

        gt_path.write_text(groundtruth_payload(), encoding="utf-8")
        run_path = self.fixture.run_root / "run.json"
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        run = json.loads(run_path.read_text(encoding="utf-8"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        fake_sha = "e" * 64
        for entry in run["source_manifest"]["entries"]:
            if entry["path"] == ANALYZER_PATH:
                entry["sha256"] = fake_sha
        run["source_id"] = canonical_id(run["source_manifest"])
        receipt["source_id"] = run["source_id"]
        receipt["inputs"]["analyzer"]["sha256"] = fake_sha
        without_id = dict(receipt)
        without_id.pop("preflight_receipt_id")
        receipt["preflight_receipt_id"] = canonical_id(without_id)
        run_path.write_text(
            json.dumps(run, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        receipt_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertIn("current analyzer", completed.stderr)

    def test_preflight_runtime_identity_and_timestamps_are_strict(self) -> None:
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        run_status_path = self.fixture.run_root / "run-status.json"
        original_receipt = receipt_path.read_bytes()
        original_run_status = run_status_path.read_bytes()

        def save_receipt(value: dict[str, object]) -> None:
            document = dict(value)
            document.pop("preflight_receipt_id", None)
            value["preflight_receipt_id"] = canonical_id(document)
            receipt_path.write_text(
                json.dumps(value, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

        for expected, mutate in (
            (
                "RFC3339",
                lambda value: value.update({"created_utc": "not-a-time"}),
            ),
            (
                "rootfs",
                lambda value: value["image"].update(
                    {"rootfs_layers": ["sha256:" + "d" * 64]}
                ),
            ),
            (
                "missing dependencies",
                lambda value: value["runtime_linkage"]["binaries"][
                    "phad_vo_bench"
                ].update({"missing": ["libmissing.so"]}),
            ),
            (
                "runtime library",
                lambda value: value["runtime_linkage"]["binaries"][
                    "phad_vo_bench"
                ]["libraries"][0].update({"sha256": "bad"}),
            ),
        ):
            with self.subTest(expected=expected):
                value = json.loads(original_receipt)
                mutate(value)
                save_receipt(value)
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(expected, completed.stderr)
                receipt_path.write_bytes(original_receipt)

        unit_path = self.fixture.run_root / "tasks" / "unit" / "status.json"
        original_unit = unit_path.read_bytes()
        unit = json.loads(original_unit)
        unit["finished_utc"] = "2026-09-03T00:00:21Z"
        unit_path.write_text(
            json.dumps(unit, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status = json.loads(original_run_status)
        run_status["tasks"]["unit"] = unit
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("unit/preflight", completed.stderr)
        unit_path.write_bytes(original_unit)
        run_status_path.write_bytes(original_run_status)

        task_name = "on-1-room1"
        task_path = self.fixture.run_root / "tasks" / task_name / "status.json"
        status = json.loads(task_path.read_text(encoding="utf-8"))
        status["started_utc"] = "2026-09-03T00:00:21Z"
        task_path.write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status = json.loads(original_run_status)
        run_status["tasks"][task_name] = status
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("batch timestamp overlap", completed.stderr)

    def test_receipt_timestamp_may_equal_first_benchmark_start(self) -> None:
        run_status_path = self.fixture.run_root / "run-status.json"
        run_status = json.loads(run_status_path.read_text(encoding="utf-8"))
        for room in ROOMS:
            name = f"off-1-{room}"
            path = self.fixture.run_root / "tasks" / name / "status.json"
            status = json.loads(path.read_text(encoding="utf-8"))
            status["started_utc"] = "2026-09-03T00:00:20Z"
            path.write_text(
                json.dumps(status, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            run_status["tasks"][name] = status
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assert_completed(completed)

    def test_meta_created_utc_must_be_inside_task_window(self) -> None:
        path = self.fixture.bench("room1", "on", 1) / "meta.json"
        meta = json.loads(path.read_text(encoding="utf-8"))
        meta["created_utc"] = "2026-09-03T00:00:20Z"
        path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertIn("meta.created_utc", completed.stderr)

    def test_dataset_manifest_is_bound_to_the_frozen_protocol(self) -> None:
        run_path = self.fixture.run_root / "run.json"
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        run = json.loads(run_path.read_text(encoding="utf-8"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        run["dataset_manifest"]["dataset"] = "different-holdout"
        changed_id = canonical_id(run["dataset_manifest"])
        run["dataset_manifest_id"] = changed_id
        receipt["dataset"]["manifest_id"] = changed_id
        document = dict(receipt)
        document.pop("preflight_receipt_id")
        receipt["preflight_receipt_id"] = canonical_id(document)
        run_path.write_text(
            json.dumps(run, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        receipt_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertIn("frozen protocol", completed.stderr)

    def test_completed_with_warnings_maps_to_warning_task(self) -> None:
        name = "on-1-room1"
        bench = self.fixture.bench("room1", "on", 1)
        warning = "source worktree is dirty; results are traceable but not clean"
        for filename in ("meta.json", "summary.json"):
            path = bench / filename
            value = json.loads(path.read_text(encoding="utf-8"))
            value["warnings"] = [warning]
            if filename == "summary.json":
                value["status"] = "completed_with_warnings"
            path.write_text(
                json.dumps(value, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        task_path = self.fixture.run_root / "tasks" / name / "status.json"
        task = json.loads(task_path.read_text(encoding="utf-8"))
        task.update(
            {
                "state": "warning",
                "benchmark_status": "completed_with_warnings",
                "failure_reason": warning,
            }
        )
        task_path.write_text(
            json.dumps(task, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        status_path = self.fixture.run_root / "run-status.json"
        status = json.loads(status_path.read_text(encoding="utf-8"))
        status["state"] = "warning"
        status["tasks"][name] = task
        status_path.write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        completed = self.fixture.run()

        self.assert_completed(completed)
        self.assertEqual(self.fixture.result()["verdict"], "INCONCLUSIVE")

    def test_image_identity_and_warning_provenance_are_strict(self) -> None:
        run_path = self.fixture.run_root / "run.json"
        original_run = run_path.read_bytes()
        run = json.loads(original_run)
        run["image"]["id"] = OCI_MANIFEST_ID
        run_path.write_text(
            json.dumps(run, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("immutable", completed.stderr)

        run_path.write_bytes(original_run)
        receipt_path = self.fixture.run_root / "preflight-receipt.json"
        original_receipt = receipt_path.read_bytes()
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        run = json.loads(original_run)
        run["image"]["rootfs_layers"] = [{"not": "hashable"}]
        receipt["image"]["rootfs_layers"] = [{"not": "hashable"}]
        document = dict(receipt)
        document.pop("preflight_receipt_id")
        receipt["preflight_receipt_id"] = canonical_id(document)
        run_path.write_text(
            json.dumps(run, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        receipt_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("rootfs", completed.stderr)

        run_path.write_bytes(original_run)
        receipt_path.write_bytes(original_receipt)
        meta_path = self.fixture.bench("room1", "on", 1) / "meta.json"
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        meta["warnings"] = ["frozen input warning"]
        meta_path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("provenance", completed.stderr)

    def test_probe_schema_tokens_and_role_invariants_are_strict(self) -> None:
        path = self.fixture.bench("room1", "on", 1) / "probe-b.jsonl"
        original = path.read_bytes()
        cases = (
            ("unknown key", lambda row: row.update({"unexpected": 1})),
            ("pnp_result", lambda row: row.update({"pnp_result": "mystery"})),
            (
                "pnp_guess_retry_outcome",
                lambda row: row.update({"pnp_guess_retry_outcome": "mystery"}),
            ),
            ("index", lambda row: row.update({"i": 9})),
            (
                "pnp_attempted/result invariant",
                lambda row: row.update({"pnp_attempted": False}),
            ),
        )
        for expected, mutate in cases:
            with self.subTest(expected=expected):
                rows = original.decode("utf-8").splitlines()
                row = json.loads(rows[0])
                mutate(row)
                rows[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
                path.write_text("\n".join(rows) + "\n", encoding="utf-8")
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(expected, completed.stderr)
                path.write_bytes(original)

        off_path = self.fixture.bench("room1", "off", 1) / "probe-b.jsonl"
        rows = off_path.read_text(encoding="utf-8").splitlines()
        row = json.loads(rows[0])
        row.update(
            {
                "pnp_attempted": True,
                "pnp_result": "accepted_after_guess_retry",
                "pnp_guess_retry_outcome": "would_pass_existing_pnp_gates",
            }
        )
        rows[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
        off_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("off role", completed.stderr)

    def test_off_role_can_record_shadow_retry_without_accepting_it(self) -> None:
        for repeat in REPEATS:
            path = self.fixture.bench("room1", "off", repeat) / "probe-b.jsonl"
            rows = path.read_text(encoding="utf-8").splitlines()
            row = json.loads(rows[0])
            row.update(
                {
                    "pnp_attempted": True,
                    "pnp_correspondences": 20,
                    "pnp_inliers": 20,
                    "pnp_result": "proposal_rms_cheirality",
                    "pnp_guess_retry_outcome": "would_pass_existing_pnp_gates",
                }
            )
            rows[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")

        completed = self.fixture.run()

        self.assert_completed(completed)
        retry = self.fixture.result()["sequences"]["room1"]["roles"]["off"][
            "retry"
        ]
        self.assertEqual(retry["eligible"], 1)
        self.assertEqual(retry["attempted"], 1)
        self.assertEqual(retry["accepted"], 0)

    def test_unattempted_on_role_eligibility_is_inconclusive(self) -> None:
        for room in ROOMS:
            for repeat in REPEATS:
                path = self.fixture.bench(room, "on", repeat) / "probe-b.jsonl"
                rows = path.read_text(encoding="utf-8").splitlines()
                row = json.loads(rows[0])
                row["pnp_result"] = "proposal_rms_cheirality"
                row["pnp_guess_retry_outcome"] = "not_attempted"
                rows[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
                path.write_text("\n".join(rows) + "\n", encoding="utf-8")

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "INCONCLUSIVE")
        self.assertTrue(
            any("eligible retry was not attempted" in reason for reason in result["reasons"])
        )

    def test_artifact_schemas_numeric_types_and_keyframe_poses_are_strict(self) -> None:
        meta_path = self.fixture.bench("room1", "off", 1) / "meta.json"
        summary_path = self.fixture.bench("room1", "off", 1) / "summary.json"
        diag_path = self.fixture.bench("room1", "off", 1) / "diag.csv"
        kf_path = self.fixture.bench("room1", "off", 1) / "kf.tum"
        originals = {
            path: path.read_bytes()
            for path in (meta_path, summary_path, diag_path, kf_path)
        }
        cases = (
            "meta schema_version",
            "summary status",
            "timing schema",
            "robustness schema",
            "diag integer",
            "diag boolean",
            "diag float",
            "diag status",
            "pnp_successes crosscheck",
            "keyframe pose",
        )
        for case in cases:
            with self.subTest(case=case):
                for path, payload in originals.items():
                    path.write_bytes(payload)
                if case == "meta schema_version":
                    value = json.loads(originals[meta_path])
                    value["schema_version"] = 3
                    meta_path.write_text(
                        json.dumps(value, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                elif case == "summary status":
                    value = json.loads(originals[summary_path])
                    value["status"] = "failed"
                    summary_path.write_text(
                        json.dumps(value, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                elif case == "timing schema":
                    value = json.loads(originals[summary_path])
                    del value["timing"]["frontend"]
                    summary_path.write_text(
                        json.dumps(value, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                elif case == "robustness schema":
                    value = json.loads(originals[summary_path])
                    value["robustness"]["pnp_fallbacks"] = 0.5
                    summary_path.write_text(
                        json.dumps(value, indent=2, sort_keys=True) + "\n",
                        encoding="utf-8",
                    )
                elif case == "diag integer":
                    diag_path.write_text(
                        originals[diag_path]
                        .decode("utf-8")
                        .replace(",20,20,20,0,10,", ",-1,20,20,0,10,", 1),
                        encoding="utf-8",
                    )
                elif case == "diag boolean":
                    lines = originals[diag_path].decode("utf-8").splitlines()
                    fields = lines[1].split(",")
                    fields[5] = "2"
                    lines[1] = ",".join(fields)
                    diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                elif case == "diag float":
                    lines = originals[diag_path].decode("utf-8").splitlines()
                    fields = lines[1].split(",")
                    fields[8] = "nan"
                    lines[1] = ",".join(fields)
                    diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                elif case == "diag status":
                    lines = originals[diag_path].decode("utf-8").splitlines()
                    fields = lines[1].split(",")
                    fields[1] = "unknown"
                    lines[1] = ",".join(fields)
                    diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                elif case == "pnp_successes crosscheck":
                    lines = originals[diag_path].decode("utf-8").splitlines()
                    fields = lines[1].split(",")
                    fields[14] = "1"
                    lines[1] = ",".join(fields)
                    diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                else:
                    lines = originals[kf_path].decode("utf-8").splitlines()
                    fields = lines[0].split()
                    fields[1] = str(float(fields[1]) + 0.01)
                    lines[0] = " ".join(fields)
                    kf_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                completed = self.fixture.run()
                self.assertEqual(completed.returncode, 2)
                self.assertIn(case.split()[0], completed.stderr)

    def test_non_ok_keyframes_and_all_diag_status_tokens_are_supported(self) -> None:
        room, role, repeat = "room1", "off", 1
        self.fixture.drop_first_pose(room, role, repeat)
        bench = self.fixture.bench(room, role, repeat)
        diag_path = bench / "diag.csv"
        lines = diag_path.read_text(encoding="utf-8").splitlines()
        fields = lines[1].split(",")
        fields[18] = "1"
        lines[1] = ",".join(fields)
        diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["trajectory"].update(
            {"total_keyframes": 6, "total_track_only_frames": 0}
        )
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assert_completed(completed)
        trajectory = self.fixture.result()["sequences"][room]["roles"][role][
            "repeats"
        ][str(repeat)]["trajectory"]
        self.assertEqual(trajectory["total_keyframes"], 6)

        lines = diag_path.read_text(encoding="utf-8").splitlines()
        fields = lines[1].split(",")
        fields[1] = "visual_outage"
        lines[1] = ",".join(fields)
        diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        completed = self.fixture.run()
        self.assert_completed(completed)

    def test_nonrepeatable_metrics_stop_product_aggregation(self) -> None:
        for room in ROOMS:
            self.fixture.rewrite_run_scale(room, "on", 1, 1.30, 0.70)
            self.fixture.rewrite_run_scale(room, "on", 2, 1.31, 0.69)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "INCONCLUSIVE")
        self.assertIsNone(result["suite"]["g_ate"])
        self.assertIsNone(result["suite"]["g_rpe"])
        self.assertIsNone(result["suite"]["eligible_retry_total"])
        self.assertEqual(result["suite"]["product_aggregation"], "not_evaluated")
        self.assertFalse(any("suite accuracy" in reason for reason in result["reasons"]))
        self.assertIsNone(
            result["sequences"]["room1"]["roles"]["on"]["native_median"]
        )
        self.assertIsNone(
            result["sequences"]["room1"]["comparison"]["ate_delta_m"]
        )

    def test_nonrepeatable_hard_failure_remains_fail(self) -> None:
        self.fixture.mark_last_frame_failed("room2", "on", 2)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertIsNone(result["suite"]["g_ate"])
        self.assertEqual(result["suite"]["product_aggregation"], "not_evaluated")

    def test_rejected_only_run_is_valid_evidence_and_hard_fail(self) -> None:
        room, role, repeat = "room2", "on", 2
        self.fixture.mark_last_frame_failed(room, role, repeat)
        bench = self.fixture.bench(room, role, repeat)
        diag_path = bench / "diag.csv"
        diag_path.write_text(
            diag_path.read_text(encoding="utf-8").replace(
                ",failed,", ",rejected,", 1
            ),
            encoding="utf-8",
        )
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["trajectory"].update({"failed": 0, "rejected": 1})
        summary["robustness"].update({"failed": 0, "rejected": 1})
        summary["status"] = "completed"
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        task_name = f"{role}-{repeat}-{room}"
        task_path = self.fixture.run_root / "tasks" / task_name / "status.json"
        task = json.loads(task_path.read_text(encoding="utf-8"))
        task.update(
            {
                "state": "warning",
                "benchmark_status": "completed",
                "failure_reason": (
                    "benchmark robustness.failed=0, robustness.rejected=1"
                ),
            }
        )
        task_path.write_text(
            json.dumps(task, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status_path = self.fixture.run_root / "run-status.json"
        run_status = json.loads(run_status_path.read_text(encoding="utf-8"))
        run_status["state"] = "warning"
        run_status["tasks"][task_name] = task
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertEqual(result["suite"]["rejected_total"], 1)

    def test_failed_summary_requires_null_metrics(self) -> None:
        room, role, repeat = "room2", "on", 2
        self.fixture.mark_last_frame_failed(room, role, repeat)
        bench = self.fixture.bench(room, role, repeat)
        warning = "estimator reported a typed fatal failure"
        summary_path = bench / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["status"] = "failed"
        summary["warnings"] = [warning]
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        task_name = f"{role}-{repeat}-{room}"
        task_path = self.fixture.run_root / "tasks" / task_name / "status.json"
        task = json.loads(task_path.read_text(encoding="utf-8"))
        task.update(
            {
                "state": "warning",
                "return_code": 1,
                "benchmark_status": "failed",
                "failure_reason": (
                    "benchmark robustness.failed=1, robustness.rejected=0"
                ),
            }
        )
        task_path.write_text(
            json.dumps(task, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        run_status_path = self.fixture.run_root / "run-status.json"
        run_status = json.loads(run_status_path.read_text(encoding="utf-8"))
        run_status["state"] = "warning"
        run_status["tasks"][task_name] = task
        run_status_path.write_text(
            json.dumps(run_status, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertIn("must be null", completed.stderr)

    def test_typed_fatal_null_metrics_skip_numeric_aggregation(self) -> None:
        for fatal_status in ("failed", "invalid_input"):
            for keep_one_pose in (False, True):
                with self.subTest(
                    fatal_status=fatal_status, keep_one_pose=keep_one_pose
                ):
                    with tempfile.TemporaryDirectory() as temporary:
                        fixture = ConfirmatoryFixture(Path(temporary))
                        fixture.make_typed_fatal(
                            "room2",
                            "on",
                            2,
                            keep_one_pose=keep_one_pose,
                            fatal_status=fatal_status,
                        )

                        completed = fixture.run()

                        self.assertEqual(completed.returncode, 0, completed.stderr)
                        result = fixture.result()
                        self.assertEqual(result["verdict"], "FAIL")
                        self.assertIsNone(result["suite"]["g_ate"])
                        self.assertEqual(
                            result["suite"]["product_aggregation"],
                            "not_evaluated",
                        )
                        native = result["sequences"]["room2"]["roles"]["on"][
                            "repeats"
                        ]["2"]
                        self.assertIsNone(native["ate"])
                        self.assertIsNone(native["rpe"])

    def test_typed_fatal_token_must_be_unique_and_terminal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ConfirmatoryFixture(Path(temporary))
            fixture.make_typed_fatal("room2", "on", 2, keep_one_pose=True)
            diag_path = fixture.bench("room2", "on", 2) / "diag.csv"
            lines = diag_path.read_text(encoding="utf-8").splitlines()
            first = lines[1].split(",")
            second = lines[2].split(",")
            first[1], first[18] = "failed", "0"
            second[1], second[18] = "ok", "1"
            lines[1], lines[2] = ",".join(first), ",".join(second)
            diag_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            for filename in ("est.tum", "kf.tum"):
                path = fixture.bench("room2", "on", 2) / filename
                fields = path.read_text(encoding="utf-8").strip().split()
                fields[0] = "2.000000000"
                path.write_text(" ".join(fields) + "\n", encoding="utf-8")

            completed = fixture.run()

            self.assertEqual(completed.returncode, 2)
            self.assertIn("terminal", completed.stderr)

    def test_zero_off_accuracy_denominator_is_typed_inconclusive(self) -> None:
        self.fixture.rewrite_role_scale("off", 1.0, 1.0)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "INCONCLUSIVE")
        self.assertIsNone(result["suite"]["g_ate"])
        self.assertIsNone(result["suite"]["g_rpe"])
        self.assertTrue(any("zero off" in reason for reason in result["reasons"]))

    def test_zero_eligible_retry_is_inconclusive(self) -> None:
        def remove_eligible(path: Path) -> None:
            rows = []
            for line in path.read_text(encoding="utf-8").splitlines():
                row = json.loads(line)
                row["pnp_attempted"] = False
                row["pnp_result"] = "not_attempted"
                row["pnp_guess_retry_outcome"] = "not_attempted"
                rows.append(json.dumps(row, sort_keys=True, separators=(",", ":")))
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")

        self.fixture.mutate_all("on", "probe-b.jsonl", remove_eligible)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "INCONCLUSIVE")
        self.assertEqual(result["suite"]["eligible_retry_total"], 0)

    def test_hard_health_failure_precedes_nonrepeatability(self) -> None:
        self.fixture.mark_last_frame_failed("room2", "on", 2)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertFalse(result["repeatability"]["room2"]["on"]["exact"])
        self.assertEqual(result["suite"]["failed_total"], 1)

    def test_each_exact_payload_participates_in_same_role_hashing(self) -> None:
        mutations = {
            "est.tum": lambda path: path.write_text(
                path.read_text(encoding="utf-8") + "\n", encoding="utf-8"
            ),
            "diag.csv": lambda path: path.write_text(
                path.read_text(encoding="utf-8").replace(
                    ",20,20,20,0,10,", ",21,20,20,0,10,", 1
                ),
                encoding="utf-8",
            ),
            "kf.tum": lambda path: path.write_text(
                path.read_text(encoding="utf-8") + "\n", encoding="utf-8"
            ),
            "probe-b.jsonl": lambda path: path.write_text(
                path.read_text(encoding="utf-8").replace("}\n", "} \n", 1),
                encoding="utf-8",
            ),
            "summary.json": self._change_summary_repeatable_field,
        }
        hash_keys = {
            "est.tum": "est.tum",
            "diag.csv": "diag.csv",
            "kf.tum": "kf.tum",
            "probe-b.jsonl": "probe-b.jsonl",
            "summary.json": "summary_without_timing",
        }
        for filename, mutate in mutations.items():
            with self.subTest(filename=filename):
                path = self.fixture.bench("room3", "on", 2) / filename
                original = path.read_bytes()
                mutate(path)
                completed = self.fixture.run()
                self.assert_completed(completed)
                result = self.fixture.result()
                repeatability = result["repeatability"]["room3"]["on"]
                self.assertFalse(repeatability["exact"])
                self.assertEqual(
                    len(set(repeatability["hashes"][hash_keys[filename]])), 2
                )
                self.assertEqual(result["verdict"], "INCONCLUSIVE")
                path.write_bytes(original)

    def test_association_reports_out_of_range_and_over_threshold_separately(self) -> None:
        cases = (
            (1_000_000_000, 990_000_000, 1, 0),
            (3_000_000_000, 3_010_000_000, 0, 1),
        )
        for old, new, expected_out_of_range, expected_over_threshold in cases:
            with self.subTest(timestamp=new):
                touched: list[tuple[Path, bytes]] = []
                for repeat in REPEATS:
                    bench = self.fixture.bench("room4", "on", repeat)
                    for filename in (
                        "est.tum",
                        "kf.tum",
                        "diag.csv",
                        "probe-b.jsonl",
                        "summary.json",
                    ):
                        path = bench / filename
                        touched.append((path, path.read_bytes()))
                    self.fixture.replace_timestamp(
                        "room4", "on", repeat, old, new
                    )
                completed = self.fixture.run()
                self.assert_completed(completed)
                result = self.fixture.result()
                native = result["sequences"]["room4"]["roles"]["on"][
                    "native_median"
                ]
                self.assertEqual(native["association_count"], 5)
                self.assertEqual(native["out_of_range"], expected_out_of_range)
                self.assertEqual(native["over_threshold"], expected_over_threshold)
                self.assertEqual(result["verdict"], "REVIEW")
                for path, original in touched:
                    path.write_bytes(original)

    def test_task_names_are_exact_and_layout_errors_exit_two(self) -> None:
        expected = self.fixture.run_root / "tasks" / "on-2-room4"
        unexpected = self.fixture.run_root / "tasks" / "candidate-2-room4"
        expected.rename(unexpected)

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertFalse(self.fixture.output.exists())
        self.assertIn("on-2-room4", completed.stderr)
        self.assertIn("candidate-2-room4", completed.stderr)

    def test_summary_metric_tamper_is_input_error(self) -> None:
        path = self.fixture.bench("room1", "on", 1) / "summary.json"
        summary = json.loads(path.read_text(encoding="utf-8"))
        summary["ate"]["trans"]["rmse"] += 1e-6
        path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertFalse(self.fixture.output.exists())
        self.assertIn("reproduction", completed.stderr)

    def test_nonfinite_json_and_diag_summary_mismatch_are_input_errors(self) -> None:
        summary_path = self.fixture.bench("room1", "off", 1) / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["timing"]["wall_s"] = float("nan")
        summary_path.write_text(json.dumps(summary) + "\n", encoding="utf-8")
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("non-finite", completed.stderr)

        summary["timing"]["wall_s"] = 11.0
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        diag = self.fixture.bench("room1", "off", 1) / "diag.csv"
        diag.write_text(
            diag.read_text(encoding="utf-8").replace(",ok,", ",failed,", 1),
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertRegex(completed.stderr, r"est\.tum timestamps|trajectory\.ok mismatch")

    def test_config_identity_requires_canonical_retry_only_diff(self) -> None:
        path = self.fixture.bench("room3", "on", 2) / "meta.json"
        meta = json.loads(path.read_text(encoding="utf-8"))
        meta.pop("config_canonical_text")
        path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("config_canonical_text", completed.stderr)

        meta["config_canonical_text"] = (
            "estimator.enable_moving_bootstrap=true\n"
            "estimator.enable_pnp_cheirality_retry=true\n"
            "session.dataset_format=tum_vi\n"
            "unexpected=true\n"
        )
        path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("config", completed.stderr)

    def test_native_and_estimate_quaternion_tolerances_are_strict(self) -> None:
        gt = self.fixture.gt_root / "room1" / "dso" / "gt_imu.csv"
        original_gt = gt.read_bytes()
        gt.write_text(
            gt.read_text(encoding="utf-8").replace(",1,0,0,0\n", ",1.000002,0,0,0\n", 1),
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("quaternion norm", completed.stderr)
        gt.write_bytes(original_gt)

        for room in ROOMS:
            for repeat in REPEATS:
                for filename in ("est.tum", "kf.tum"):
                    path = self.fixture.bench(room, "on", repeat) / filename
                    path.write_text(
                        path.read_text(encoding="utf-8").replace(
                            " 0 0 0 1\n", " 0 0 0 1.0005\n"
                        ),
                        encoding="utf-8",
                    )
        completed = self.fixture.run()
        self.assert_completed(completed)

        path = self.fixture.bench("room1", "on", 1) / "est.tum"
        path.write_text(
            path.read_text(encoding="utf-8").replace(
                " 0 0 0 1.0005\n", " 0 0 0 1.0011\n", 1
            ),
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("quaternion norm", completed.stderr)

    def test_diag_and_probe_must_match_the_run_timeline(self) -> None:
        diag = self.fixture.bench("room2", "off", 1) / "diag.csv"
        original_diag = diag.read_bytes()
        diag.write_text(
            diag.read_text(encoding="utf-8").replace(DIAG_HEADER, DIAG_HEADER + ",extra", 1),
            encoding="utf-8",
        )
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("26-column", completed.stderr)
        diag.write_bytes(original_diag)

        probe = self.fixture.bench("room2", "off", 1) / "probe-b.jsonl"
        rows = probe.read_text(encoding="utf-8").splitlines()
        row = json.loads(rows[0])
        row["ts_ns"] += 1
        rows[0] = json.dumps(row, sort_keys=True, separators=(",", ":"))
        probe.write_text("\n".join(rows) + "\n", encoding="utf-8")
        completed = self.fixture.run()
        self.assertEqual(completed.returncode, 2)
        self.assertIn("Probe B timestamps", completed.stderr)

    def test_suite_accuracy_regression_is_fail(self) -> None:
        self.fixture.rewrite_role_scale("on", 1.30, 0.70)

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertGreater(result["suite"]["g_ate"], 1.05)
        self.assertGreater(result["suite"]["g_rpe"], 1.05)

    def test_segment_increase_is_fail(self) -> None:
        for room in ROOMS:
            for repeat in REPEATS:
                bench = self.fixture.bench(room, "on", repeat)
                diag = bench / "diag.csv"
                lines = diag.read_text(encoding="utf-8").splitlines()
                fields = lines[-1].split(",")
                fields[13] = "2"
                lines[-1] = ",".join(fields)
                diag.write_text("\n".join(lines) + "\n", encoding="utf-8")
                summary_path = bench / "summary.json"
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                summary["trajectory"]["segments"] = 2
                summary["robustness"]["reanchors"] = 1
                summary_path.write_text(
                    json.dumps(summary, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )

        completed = self.fixture.run()

        self.assert_completed(completed)
        result = self.fixture.result()
        self.assertEqual(result["verdict"], "FAIL")
        self.assertFalse(result["suite"]["segments_nonincrease"])

    def test_wrong_native_groundtruth_header_is_input_error(self) -> None:
        gt = self.fixture.gt_root / "room5" / "dso" / "gt_imu.csv"
        text = gt.read_text(encoding="utf-8")
        gt.write_text(
            text.replace(
                "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz",
                "# timestamp,tx,ty,tz,qx,qy,qz,qw",
            ),
            encoding="utf-8",
        )

        completed = self.fixture.run()

        self.assertEqual(completed.returncode, 2)
        self.assertFalse(self.fixture.output.exists())
        self.assertIn("gt_imu.csv", completed.stderr)
        self.assertIn("header", completed.stderr.lower())

    @staticmethod
    def _change_summary_repeatable_field(path: Path) -> None:
        value = json.loads(path.read_text(encoding="utf-8"))
        value["sync"]["max_left_queue"] = 2
        path.write_text(
            json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )


if __name__ == "__main__":
    unittest.main()

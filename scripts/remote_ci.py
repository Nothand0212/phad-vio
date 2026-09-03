#!/usr/bin/env python3
"""局域网服务器上的隔离 Remote CI / EuRoC 编排入口。"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import csv
import datetime as dt
import hashlib
import math
import json
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
import traceback
from typing import Any, Iterable, Sequence


SCHEMA_VERSION = 1
PROFILE = "ci-euroc11"
TUM_VI_CONFIRMATORY_PROFILE = "ci-tumvi-room6-confirmatory"
REMOTE_IDENTITY_CONFIG_ENV = "PHAD_REMOTE_CI_CONFIG"
REMOTE_IDENTITY_RELATIVE = Path("phad-remote-ci") / "config.json"
REMOTE_IDENTITY_FINGERPRINT = re.compile(r"^SHA256:[A-Za-z0-9+/]+$")
REMOTE_IDENTITY_EXAMPLE = {
    "host": "192.168.110.34",
    "user": "lin",
    "fingerprint": "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE",
}
REMOTE_ROOT = Path("/home/lin/Projects/tigerfish")
REMOTE_DATA_ROOT = Path("/home/lin/data/euroc/native")
REMOTE_TUM_VI_DATA_ROOT = (
    REMOTE_ROOT / "datasets" / TUM_VI_CONFIRMATORY_PROFILE
)
LOCAL_ARTIFACT_ROOT = Path("artifacts/remote-ci")
GTSAM_COMMIT = "2f3e56c0ddbd3a1aa54ed043643b553d26a069f6"
GTSAM_ARCHIVE_SHA256 = (
    "50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a"
)
GTSAM_ARCHIVE_URL = (
    "https://codeload.github.com/borglab/gtsam/tar.gz/" + GTSAM_COMMIT
)

SEQUENCES = (
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
)

TUM_VI_ROOM_SEQUENCES = (
    "room1",
    "room2",
    "room3",
    "room4",
    "room5",
    "room6",
)
TUM_VI_CONFIRMATORY_BATCH_ORDER = (
    ("off", 1),
    ("on", 1),
    ("on", 2),
    ("off", 2),
    ("off", 3),
    ("on", 3),
)
TUM_VI_PREFLIGHT_PROTOCOL_PATH = (
    "docs/research/"
    "2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-protocol.md"
)
TUM_VI_PREFLIGHT_ANALYZER_PATH = "scripts/analyze_tumvi_room_confirmatory.py"
TUM_VI_PREFLIGHT_BINARIES = (
    "phad_vo_bench",
    "phad_estimator_tests",
    "phad_apps_tests",
    "phad_io_dataset_tests",
    "phad_eval_tests",
    "phad_bench_tests",
)
TUM_VI_ROOM_ARCHIVES = {
    "room1": {
        "filename": "dataset-room1_512_16.tar",
        "size": 1_707_110_400,
        "sha256": "20354392eab5cc82c9770ed29cf0130c5acce0db54a257571499008fb34c398f",
        "archive_root": "dataset-room1_512_16",
        "cam0_frames": 2_821,
        "cam1_frames": 2_821,
        "imu_rows": 28_122,
        "gt_rows": 16_541,
        "timestamps": {
            "cam0": [1_520_530_308_199_447_626, 1_520_530_449_203_911_100, 141_004_463_474],
            "cam1": [1_520_530_308_199_447_626, 1_520_530_449_203_911_100, 141_004_463_474],
            "imu": [1_520_530_308_181_901_469, 1_520_530_449_232_329_469, 141_050_428_000],
            "gt": [1_520_530_308_189_679_351, 1_520_530_449_214_678_351, 141_024_999_000],
        },
    },
    "room2": {
        "filename": "dataset-room2_512_16.tar",
        "size": 1_759_395_840,
        "sha256": "70343c5bbd9d674d0a2790bf934ee7d9cc08c1dea7cd77747a5557e4c94973f6",
        "archive_root": "dataset-room2_512_16",
        "cam0_frames": 2_882,
        "cam1_frames": 2_882,
        "imu_rows": 28_732,
        "gt_rows": 15_477,
        "timestamps": {
            "cam0": [1_520_530_731_400_018_325, 1_520_530_875_454_345_641, 144_054_327_316],
            "cam1": [1_520_530_731_400_018_325, 1_520_530_875_454_345_641, 144_054_327_316],
            "imu": [1_520_530_731_372_454_010, 1_520_530_875_479_830_010, 144_107_376_000],
            "gt": [1_520_530_731_382_632_018, 1_520_530_875_449_299_018, 144_066_667_000],
        },
    },
    "room3": {
        "filename": "dataset-room3_512_16.tar",
        "size": 1_742_612_480,
        "sha256": "66ff297e6600e882277d21e21af5ec9fb33864ec0b759dcc0b629cf551a3c824",
        "archive_root": "dataset-room3_512_16",
        "cam0_frames": 2_821,
        "cam1_frames": 2_821,
        "imu_rows": 28_124,
        "gt_rows": 15_200,
        "timestamps": {
            "cam0": [1_520_530_962_750_148_976, 1_520_531_103_754_995_888, 141_004_846_912],
            "cam1": [1_520_530_962_750_148_976, 1_520_531_103_754_995_888, 141_004_846_912],
            "imu": [1_520_530_962_735_370_731, 1_520_531_103_790_106_731, 141_054_736_000],
            "gt": [1_520_530_962_744_806_490, 1_520_531_103_761_473_490, 141_016_667_000],
        },
    },
    "room4": {
        "filename": "dataset-room4_512_16.tar",
        "size": 1_356_206_080,
        "sha256": "2c3633407693988cf24faef5f874cba08bbc3c2d2ec1168c86b6da55ae9f2e68",
        "archive_root": "dataset-room4_512_16",
        "cam0_frames": 2_228,
        "cam1_frames": 2_228,
        "imu_rows": 22_212,
        "gt_rows": 13_075,
        "timestamps": {
            "cam0": [1_520_531_124_150_444_163, 1_520_531_235_504_118_338, 111_353_674_175],
            "cam1": [1_520_531_124_150_444_163, 1_520_531_235_504_118_338, 111_353_674_175],
            "imu": [1_520_531_124_153_717_567, 1_520_531_235_555_397_567, 111_401_680_000],
            "gt": [1_520_531_124_177_875_537, 1_520_531_235_544_541_537, 111_366_666_000],
        },
    },
    "room5": {
        "filename": "dataset-room5_512_16.tar",
        "size": 1_696_430_080,
        "sha256": "be6129b096dc88a7da0a552f1fc74c3309fffaa88bf4528a779e3ecc60a65519",
        "archive_root": "dataset-room5_512_16",
        "cam0_frames": 2_847,
        "cam1_frames": 2_847,
        "imu_rows": 28_381,
        "gt_rows": 16_999,
        "timestamps": {
            "cam0": [1_520_531_467_550_669_082, 1_520_531_609_855_305_345, 142_304_636_263],
            "cam1": [1_520_531_467_550_669_082, 1_520_531_609_855_305_345, 142_304_636_263],
            "imu": [1_520_531_467_566_265_223, 1_520_531_609_911_207_223, 142_344_942_000],
            "gt": [1_520_531_467_575_275_014, 1_520_531_609_891_941_014, 142_316_666_000],
        },
    },
    "room6": {
        "filename": "dataset-room6_512_16.tar",
        "size": 1_598_576_640,
        "sha256": "3183e404e1d24bb8cea29ab6e4e69a69a5f88a68dac6c2ab7f67b6ac86d62827",
        "archive_root": "dataset-room6_512_16",
        "cam0_frames": 2_636,
        "cam1_frames": 2_636,
        "imu_rows": 26_281,
        "gt_rows": 15_244,
        "timestamps": {
            "cam0": [1_520_621_014_586_874_379, 1_520_621_146_341_014_958, 131_754_140_579],
            "cam1": [1_520_621_014_586_874_379, 1_520_621_146_341_014_958, 131_754_140_579],
            "imu": [1_520_621_014_560_423_941, 1_520_621_146_368_776_941, 131_808_353_000],
            "gt": [1_520_621_015_527_175_707, 1_520_621_146_352_175_707, 130_825_000_000],
        },
    },
}

SNAPSHOT_EXCLUDED_PREFIXES = (
    PurePosixPath(".codex"),
    PurePosixPath("artifacts/remote-ci"),
)
TERMINAL_TASK_STATES = {"passed", "warning", "failed", "blocked"}
TERMINAL_RUN_STATES = {"passed", "warning", "failed"}
SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SHA256_CONTENT_ID = re.compile(r"^sha256:[0-9a-f]{64}$")
TUM_VI_CAMERA_HEADER = "#timestamp [ns],filename"
TUM_VI_IMU_HEADER = (
    "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
    "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
    "a_RS_S_z [m s^-2]"
)
TUM_VI_GROUNDTRUTH_HEADER = "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz"
TUM_VI_SKIPPED_REGULAR_PATHS = {
    "mav0/mocap0/data.csv",
    "dso/cam0/times.txt",
    "dso/cam0/camera.txt",
    "dso/cam0/vignette.png",
    "dso/cam0/pcalib.txt",
    "dso/cam1/times.txt",
    "dso/cam1/camera.txt",
    "dso/cam1/vignette.png",
    "dso/cam1/pcalib.txt",
    "dso/imu.txt",
}
TUM_VI_SKIPPED_SYMLINKS = {
    "dso/cam0/images": "../../mav0/cam0/data",
    "dso/cam1/images": "../../mav0/cam1/data",
}
TUM_VI_ALLOWED_DIRECTORIES = {
    "",
    "mav0",
    "mav0/cam0",
    "mav0/cam0/data",
    "mav0/cam1",
    "mav0/cam1/data",
    "mav0/imu0",
    "mav0/mocap0",
    "dso",
    "dso/cam0",
    "dso/cam1",
}


def _canonical_json_id(value: Any) -> str:
    canonical = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return f"sha256:{hashlib.sha256(canonical).hexdigest()}"


def profile_definition(profile: str) -> dict[str, Any]:
    """Return a self-contained, canonical definition for a supported profile."""

    if profile == PROFILE:
        return {
            "name": PROFILE,
            "dataset_format": "euroc",
            "dataset_root": str(REMOTE_DATA_ROOT),
            "sequences": list(SEQUENCES),
            "max_parallel_tasks": 8,
        }
    if profile == TUM_VI_CONFIRMATORY_PROFILE:
        dataset_manifest: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "dataset": "tum-vi-exported-euroc-512_16-room1-room6",
            "archives": TUM_VI_ROOM_ARCHIVES,
            "calibration": {
                "camchain_sha256": "3a0c0285f4d0a3a1678053b2e5912e3e250e78ac09bd009f6d7d8bba669d5703",
                "imu_config_sha256": "e309d3f4b5e6e812580fcb9c9f402ab79f4a09931e41f433e7038301118683a1",
            },
        }
        # JSON round-tripping also prevents callers from mutating module constants.
        result = {
            "name": TUM_VI_CONFIRMATORY_PROFILE,
            "dataset_format": "tum-vi",
            "dataset_root": str(REMOTE_TUM_VI_DATA_ROOT),
            "sequences": list(TUM_VI_ROOM_SEQUENCES),
            "batch_order": [list(batch) for batch in TUM_VI_CONFIRMATORY_BATCH_ORDER],
            "max_parallel_tasks": 6,
            "dataset_manifest": dataset_manifest,
            "dataset_manifest_id": _canonical_json_id(dataset_manifest),
        }
        return json.loads(json.dumps(result))
    raise RemoteCiError(f"unsupported profile: {profile}")


class RemoteCiError(RuntimeError):
    """可向 operator 直接报告的 Remote CI 错误。"""


class RemoteIdentity:
    """本机共享的 SSH 机器身份。"""

    def __init__(self, host: str, user: str, fingerprint: str) -> None:
        self.host = host
        self.user = user
        self.fingerprint = fingerprint

    @property
    def target(self) -> str:
        return f"{self.user}@{self.host}"


_remote_identity: RemoteIdentity | None = None


def remote_identity_config_path() -> Path:
    override = os.environ.get(REMOTE_IDENTITY_CONFIG_ENV)
    if override:
        return Path(override).expanduser()
    xdg = os.environ.get("XDG_CONFIG_HOME")
    if xdg:
        return Path(xdg).expanduser() / REMOTE_IDENTITY_RELATIVE
    return Path.home() / ".config" / REMOTE_IDENTITY_RELATIVE


def reset_remote_identity_cache() -> None:
    global _remote_identity
    _remote_identity = None


def _identity_config_error(path: Path, reason: str) -> RemoteCiError:
    example = json.dumps(REMOTE_IDENTITY_EXAMPLE, indent=2, ensure_ascii=False)
    return RemoteCiError(
        f"remote SSH identity config {reason}: {path}\n"
        "expected JSON object with only host, user, fingerprint, for example:\n"
        f"{example}"
    )


def load_remote_identity(path: Path | None = None) -> RemoteIdentity:
    config_path = path if path is not None else remote_identity_config_path()
    if not config_path.is_file():
        raise _identity_config_error(config_path, "is missing")
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise _identity_config_error(config_path, f"is invalid ({error})") from error
    if not isinstance(raw, dict):
        raise _identity_config_error(config_path, "must be a JSON object")
    extra = sorted(set(raw) - set(REMOTE_IDENTITY_EXAMPLE))
    if extra:
        raise _identity_config_error(config_path, f"has unsupported keys {extra}")
    missing = [key for key in REMOTE_IDENTITY_EXAMPLE if key not in raw]
    if missing:
        raise _identity_config_error(config_path, f"is missing {missing}")
    values: dict[str, str] = {}
    for key in REMOTE_IDENTITY_EXAMPLE:
        value = raw[key]
        if not isinstance(value, str) or not value.strip():
            raise _identity_config_error(
                config_path, f"field {key} must be a non-empty string"
            )
        values[key] = value.strip()
    if REMOTE_IDENTITY_FINGERPRINT.fullmatch(values["fingerprint"]) is None:
        raise _identity_config_error(
            config_path, "fingerprint must look like SHA256:<base64>"
        )
    return RemoteIdentity(
        host=values["host"],
        user=values["user"],
        fingerprint=values["fingerprint"],
    )


def remote_identity() -> RemoteIdentity:
    global _remote_identity
    if _remote_identity is None:
        _remote_identity = load_remote_identity()
    return _remote_identity


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def validate_identifier(value: str, field: str) -> str:
    if SAFE_IDENTIFIER.fullmatch(value) is None:
        raise RemoteCiError(f"invalid {field}: {value!r}")
    return value


def run_command(
    command: Sequence[str],
    *,
    cwd: Path | None = None,
    check: bool = True,
    capture_output: bool = True,
    text: bool = True,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[Any]:
    try:
        return subprocess.run(
            list(command),
            cwd=cwd,
            check=check,
            capture_output=capture_output,
            text=text,
            env=env,
        )
    except FileNotFoundError as error:
        raise RemoteCiError(f"required command is missing: {command[0]}") from error
    except subprocess.CalledProcessError as error:
        detail = ""
        if error.stderr:
            detail = str(error.stderr).strip()
        elif error.stdout:
            detail = str(error.stdout).strip()
        suffix = f": {detail}" if detail else ""
        raise RemoteCiError(
            f"command failed ({error.returncode}): {shlex.join(command)}{suffix}"
        ) from error


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{secrets.token_hex(3)}")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RemoteCiError(f"failed to read JSON: {path}: {error}") from error
    if not isinstance(value, dict):
        raise RemoteCiError(f"expected JSON object: {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_sha256_content_id(value: str, field: str) -> str:
    if SHA256_CONTENT_ID.fullmatch(value) is None:
        raise RemoteCiError(f"{field} must be sha256:<64 lowercase hex digits>")
    return value


def _parse_sha256_content_id(value: str) -> str:
    try:
        return _validate_sha256_content_id(value, "dataset content id")
    except RemoteCiError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _tum_vi_selected_regular_path(relative: str) -> bool:
    if relative in {
        "mav0/cam0/data.csv",
        "mav0/cam1/data.csv",
        "mav0/imu0/data.csv",
        "dso/camchain.yaml",
        "dso/imu_config.yaml",
        "dso/gt_imu.csv",
    }:
        return True
    return re.fullmatch(r"mav0/cam[01]/data/[0-9]+\.png", relative) is not None


def _tum_vi_archive_relative(name: str, archive_root: str) -> str:
    if "\0" in name:
        raise RemoteCiError(f"NUL is not allowed in archive member path: {name!r}")
    if "\\" in name:
        raise RemoteCiError(f"backslash is not allowed in archive member path: {name!r}")
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise RemoteCiError(f"unsafe archive member path: {name!r}")
    canonical = path.as_posix()
    if canonical != name.rstrip("/"):
        raise RemoteCiError(f"non-canonical archive member path: {name!r}")
    if canonical == archive_root:
        return ""
    prefix = archive_root + "/"
    if not canonical.startswith(prefix):
        raise RemoteCiError(
            f"archive member is outside declared root {archive_root!r}: {name!r}"
        )
    relative = canonical.removeprefix(prefix)
    if not relative:
        raise RemoteCiError(f"empty archive member path: {name!r}")
    return relative


def _strict_csv_rows(
    path: Path,
    *,
    expected_header: str,
    columns: int,
    label: str,
) -> tuple[list[list[str]], list[int]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            lines = stream.read().splitlines()
    except (OSError, UnicodeError) as error:
        raise RemoteCiError(f"failed to read {label} CSV: {path}: {error}") from error
    if not lines or lines[0] != expected_header:
        actual = lines[0] if lines else None
        raise RemoteCiError(
            f"{label} header mismatch: expected {expected_header!r}, got {actual!r}"
        )
    rows = list(csv.reader(lines[1:]))
    if not rows:
        raise RemoteCiError(f"{label} CSV has no data rows: {path}")
    timestamps: list[int] = []
    for index, row in enumerate(rows, start=2):
        if len(row) != columns:
            raise RemoteCiError(
                f"{label} CSV row {index} has {len(row)} columns, expected {columns}"
            )
        try:
            timestamp = int(row[0])
        except ValueError as error:
            raise RemoteCiError(
                f"{label} CSV row {index} has invalid integer timestamp: {row[0]!r}"
            ) from error
        if timestamps and timestamp <= timestamps[-1]:
            raise RemoteCiError(
                f"{label} timestamps are not strictly increasing at row {index}"
            )
        timestamps.append(timestamp)
    return rows, timestamps


def _check_expected_count(
    expected: dict[str, Any], field: str, actual: int, label: str
) -> None:
    if field in expected and int(expected[field]) != actual:
        raise RemoteCiError(
            f"{label} count mismatch: expected {expected[field]}, got {actual}"
        )


def _check_expected_timestamps(
    expected: dict[str, Any], label: str, timestamps: Sequence[int]
) -> None:
    expected_domains = expected.get("timestamps")
    if not isinstance(expected_domains, dict) or label not in expected_domains:
        return
    actual = [timestamps[0], timestamps[-1], timestamps[-1] - timestamps[0]]
    declared = expected_domains[label]
    if list(declared) != actual:
        raise RemoteCiError(
            f"{label} timestamp domain mismatch: expected {declared}, got {actual}"
        )


def _validate_extracted_tum_vi_tree(
    root: Path, expected: dict[str, Any]
) -> dict[str, Any]:
    domains: dict[str, list[int]] = {}
    for camera in ("cam0", "cam1"):
        rows, timestamps = _strict_csv_rows(
            root / "mav0" / camera / "data.csv",
            expected_header=TUM_VI_CAMERA_HEADER,
            columns=2,
            label=f"{camera} camera",
        )
        filenames: list[str] = []
        for index, (timestamp, filename) in enumerate(rows, start=2):
            if filename != f"{timestamp}.png" or Path(filename).name != filename:
                raise RemoteCiError(
                    f"{camera} camera CSV row {index} has non-canonical filename: "
                    f"{filename!r}"
                )
            filenames.append(filename)
        images = sorted(
            path.name for path in (root / "mav0" / camera / "data").glob("*.png")
        )
        if sorted(filenames) != images:
            raise RemoteCiError(
                f"{camera} camera CSV/PNG bijection mismatch: "
                f"csv={len(filenames)}, png={len(images)}"
            )
        _check_expected_count(
            expected, f"{camera}_frames", len(rows), f"{camera} frames"
        )
        _check_expected_timestamps(expected, camera, timestamps)
        domains[camera] = timestamps

    imu_rows, imu_timestamps = _strict_csv_rows(
        root / "mav0/imu0/data.csv",
        expected_header=TUM_VI_IMU_HEADER,
        columns=7,
        label="IMU",
    )
    for index, row in enumerate(imu_rows, start=2):
        try:
            values = [float(value) for value in row[1:]]
        except ValueError as error:
            raise RemoteCiError(f"IMU CSV row {index} contains invalid number") from error
        if not all(math.isfinite(value) for value in values):
            raise RemoteCiError(f"IMU CSV row {index} contains non-finite number")
    _check_expected_count(expected, "imu_rows", len(imu_rows), "IMU rows")
    _check_expected_timestamps(expected, "imu", imu_timestamps)
    domains["imu"] = imu_timestamps

    gt_rows, gt_timestamps = _strict_csv_rows(
        root / "dso/gt_imu.csv",
        expected_header=TUM_VI_GROUNDTRUTH_HEADER,
        columns=8,
        label="groundtruth gt_imu",
    )
    for index, row in enumerate(gt_rows, start=2):
        try:
            values = [float(value) for value in row[1:]]
        except ValueError as error:
            raise RemoteCiError(
                f"groundtruth gt_imu row {index} contains invalid number"
            ) from error
        if not all(math.isfinite(value) for value in values):
            raise RemoteCiError(
                f"groundtruth gt_imu row {index} contains non-finite number"
            )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:7]))
        if abs(quaternion_norm - 1.0) > 1e-6:
            raise RemoteCiError(
                f"groundtruth gt_imu row {index} quaternion is not unit length"
            )
    _check_expected_count(expected, "gt_rows", len(gt_rows), "groundtruth rows")
    _check_expected_timestamps(expected, "gt", gt_timestamps)
    domains["gt"] = gt_timestamps

    calibration = {
        "camchain_sha256": sha256_file(root / "dso/camchain.yaml"),
        "imu_config_sha256": sha256_file(root / "dso/imu_config.yaml"),
    }
    for field, actual in calibration.items():
        declared = expected.get(field)
        if declared is not None and str(declared) != actual:
            raise RemoteCiError(
                f"calibration {field} mismatch: expected {declared}, got {actual}"
            )
    return {
        "cam0_frames": len(domains["cam0"]),
        "cam1_frames": len(domains["cam1"]),
        "imu_rows": len(domains["imu"]),
        "gt_rows": len(domains["gt"]),
        "timestamps": {
            label: [values[0], values[-1], values[-1] - values[0]]
            for label, values in domains.items()
        },
        **calibration,
    }


def audit_and_extract_tumvi_archive(
    archive: Path,
    destination: Path,
    *,
    sequence: str,
    expected: dict[str, Any],
) -> dict[str, Any]:
    """Audit one frozen TUM-VI tar and atomically materialize its runtime subset."""

    archive = Path(archive)
    destination = Path(destination)
    try:
        archive_metadata = archive.lstat()
    except OSError as error:
        raise RemoteCiError(f"archive is unavailable: {archive}: {error}") from error
    if not stat.S_ISREG(archive_metadata.st_mode):
        raise RemoteCiError(f"archive is not a regular file: {archive}")
    actual_size = archive_metadata.st_size
    if actual_size != int(expected["size"]):
        raise RemoteCiError(
            f"archive size mismatch: expected {expected['size']}, got {actual_size}"
        )
    actual_sha256 = sha256_file(archive)
    if actual_sha256 != str(expected["sha256"]):
        raise RemoteCiError(
            f"archive sha256 mismatch: expected {expected['sha256']}, "
            f"got {actual_sha256}"
        )
    if destination.exists() or destination.is_symlink():
        raise RemoteCiError(f"staging destination already exists: {destination}")

    archive_root = str(
        expected.get("archive_root", f"dataset-{sequence}_512_16")
    )
    selected: list[tuple[tarfile.TarInfo, str]] = []
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.name}.partial-{os.getpid()}-{secrets.token_hex(3)}"
    )
    try:
        with tarfile.open(archive, "r") as stream:
            seen: set[str] = set()
            for member in stream.getmembers():
                relative = _tum_vi_archive_relative(member.name, archive_root)
                if relative in seen:
                    raise RemoteCiError(
                        f"duplicate archive member path: {member.name!r}"
                    )
                seen.add(relative)
                if member.isdir():
                    if relative not in TUM_VI_ALLOWED_DIRECTORIES:
                        raise RemoteCiError(
                            f"undeclared archive directory member: {member.name!r}"
                        )
                    continue
                if member.issym():
                    target = TUM_VI_SKIPPED_SYMLINKS.get(relative)
                    if target is None or member.linkname != target:
                        raise RemoteCiError(
                            f"unsupported symlink member: {member.name!r} -> "
                            f"{member.linkname!r}"
                        )
                    continue
                if member.islnk():
                    raise RemoteCiError(f"hardlink archive member is forbidden: {member.name!r}")
                if member.ischr() or member.isblk():
                    raise RemoteCiError(f"device archive member is forbidden: {member.name!r}")
                if member.isfifo():
                    raise RemoteCiError(f"FIFO archive member is forbidden: {member.name!r}")
                if not member.isfile():
                    raise RemoteCiError(
                        f"unsupported archive member type: {member.name!r}"
                    )
                if relative in TUM_VI_SKIPPED_REGULAR_PATHS:
                    continue
                if not _tum_vi_selected_regular_path(relative):
                    raise RemoteCiError(
                        f"undeclared regular archive member: {member.name!r}"
                    )
                selected.append((member, relative))

            temporary.mkdir(mode=0o755)
            for member, relative in selected:
                target = temporary / Path(*PurePosixPath(relative).parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                source = stream.extractfile(member)
                if source is None:
                    raise RemoteCiError(
                        f"failed to read regular archive member: {member.name!r}"
                    )
                with source, target.open("xb") as output:
                    shutil.copyfileobj(source, output, length=1024 * 1024)
                target.chmod(stat.S_IMODE(member.mode) & 0o777)

        validation = _validate_extracted_tum_vi_tree(temporary, expected)
        os.replace(temporary, destination)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    return {
        "sequence": sequence,
        "filename": archive.name,
        "archive_root": archive_root,
        "archive_size": actual_size,
        "archive_sha256": actual_sha256,
        **validation,
    }


def build_tumvi_dataset_manifest(
    dataset_root: Path,
    *,
    archive_records: dict[str, Any],
    calibration: dict[str, Any],
) -> dict[str, Any]:
    """Build a canonical regular-file tree manifest and its content ID."""

    root = Path(dataset_root)
    entries: list[dict[str, Any]] = []
    for path in sorted(
        root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()
    ):
        relative = path.relative_to(root).as_posix()
        if relative == ".phad-dataset.json":
            continue
        metadata = path.lstat()
        if stat.S_ISDIR(metadata.st_mode):
            continue
        if not stat.S_ISREG(metadata.st_mode):
            raise RemoteCiError(f"unsupported published dataset entry: {relative}")
        entries.append(
            {
                "path": relative,
                "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
                "size": metadata.st_size,
                "sha256": sha256_file(path),
            }
        )
    archive_digests: dict[str, str] = {}
    for sequence, record in archive_records.items():
        if not isinstance(record, dict):
            raise RemoteCiError(f"invalid archive record for {sequence}")
        digest = record.get("archive_sha256", record.get("sha256"))
        if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            raise RemoteCiError(f"invalid archive sha256 for {sequence}")
        archive_digests[str(sequence)] = digest
    base = {
        "schema_version": SCHEMA_VERSION,
        "archives": json.loads(json.dumps(archive_records, sort_keys=True)),
        "calibration": json.loads(json.dumps(calibration, sort_keys=True)),
        "entries": entries,
    }
    identity = {
        "schema_version": SCHEMA_VERSION,
        "archive_sha256": archive_digests,
        "calibration": base["calibration"],
        "entries": entries,
    }
    return {**base, "dataset_content_id": _canonical_json_id(identity)}


def _tum_vi_stage_receipt(
    definition: dict[str, Any], tree_manifest: dict[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "profile": definition["name"],
        "dataset_manifest_id": definition["dataset_manifest_id"],
        "dataset_manifest": definition["dataset_manifest"],
        "dataset_content_id": tree_manifest["dataset_content_id"],
        "dataset_tree_manifest": tree_manifest,
    }


def _freeze_tumvi_dataset_tree(root: Path, *, include_root: bool) -> None:
    paths = sorted(
        Path(root).rglob("*"),
        key=lambda path: len(path.relative_to(root).parts),
        reverse=True,
    )
    for path in paths:
        metadata = path.lstat()
        if stat.S_ISREG(metadata.st_mode):
            path.chmod(0o444)
        elif stat.S_ISDIR(metadata.st_mode):
            path.chmod(0o555)
        else:
            raise RemoteCiError(
                f"unsupported dataset entry while freezing tree: {path}"
            )
    if include_root:
        Path(root).chmod(0o555)


def _discard_tumvi_staging(root: Path) -> None:
    if not root.exists() and not root.is_symlink():
        return
    if root.is_symlink():
        raise RemoteCiError(f"staging cleanup target is a symlink: {root}")
    for path in sorted(
        root.rglob("*"),
        key=lambda item: len(item.relative_to(root).parts),
    ):
        if path.is_dir() and not path.is_symlink():
            path.chmod(0o755)
    root.chmod(0o755)
    shutil.rmtree(root)


def _verify_tum_vi_published_dataset(
    root: Path,
    *,
    expected_content_id: str,
    expected_manifest_id: str | None = None,
) -> dict[str, Any]:
    content_id = _validate_sha256_content_id(
        expected_content_id, "dataset content id"
    )
    requested_root = Path(root).absolute()
    try:
        root_metadata = requested_root.lstat()
        resolved_root = requested_root.resolve(strict=True)
    except OSError as error:
        raise RemoteCiError(
            f"published dataset root is unavailable: {requested_root}: {error}"
        ) from error
    if not stat.S_ISDIR(root_metadata.st_mode):
        raise RemoteCiError(
            f"published dataset root is not a real directory: {requested_root}"
        )
    if resolved_root != requested_root:
        raise RemoteCiError(
            "published dataset root resolves through a symlink: "
            f"{requested_root} -> {resolved_root}"
        )
    root = requested_root
    sentinel_path = root / ".phad-dataset.json"
    try:
        sentinel = read_json(sentinel_path)
    except RemoteCiError as error:
        raise RemoteCiError(f"dataset sentinel is invalid: {error}") from error
    if sentinel.get("dataset_content_id") != content_id:
        raise RemoteCiError(
            "dataset sentinel content mismatch: "
            f"{sentinel.get('dataset_content_id')!r} != {content_id!r}"
        )
    if (
        expected_manifest_id is not None
        and sentinel.get("dataset_manifest_id") != expected_manifest_id
    ):
        raise RemoteCiError(
            "dataset sentinel manifest mismatch: "
            f"{sentinel.get('dataset_manifest_id')!r} != {expected_manifest_id!r}"
        )
    if stat.S_IMODE(root_metadata.st_mode) != 0o555:
        raise RemoteCiError(f"published dataset root is not read-only: {root}")
    for path in Path(root).rglob("*"):
        metadata = path.lstat()
        if stat.S_ISDIR(metadata.st_mode):
            expected_mode = 0o555
        elif stat.S_ISREG(metadata.st_mode):
            expected_mode = 0o444
        else:
            raise RemoteCiError(f"published dataset has unsupported entry: {path}")
        if stat.S_IMODE(metadata.st_mode) != expected_mode:
            raise RemoteCiError(
                f"published dataset entry is not read-only: {path}: "
                f"{stat.S_IMODE(metadata.st_mode):04o} != {expected_mode:04o}"
            )
    tree_manifest = sentinel.get("dataset_tree_manifest")
    if not isinstance(tree_manifest, dict):
        raise RemoteCiError("dataset sentinel has no tree manifest")
    rebuilt = build_tumvi_dataset_manifest(
        Path(root),
        archive_records=tree_manifest.get("archives", {}),
        calibration=tree_manifest.get("calibration", {}),
    )
    if rebuilt != tree_manifest or rebuilt["dataset_content_id"] != content_id:
        raise RemoteCiError("dataset sentinel tree content does not match published root")
    return sentinel


def stage_remote_dataset(
    source_archives: dict[str, Path],
    dataset_store: Path,
    *,
    definition: dict[str, Any],
    transfer: Any = shutil.copyfile,
) -> dict[str, Any]:
    """Stage and content-address a frozen TUM-VI dataset on the current host."""

    declaration = definition.get("dataset_manifest")
    if not isinstance(declaration, dict):
        raise RemoteCiError("dataset definition has no manifest")
    manifest_id = str(definition.get("dataset_manifest_id", ""))
    if manifest_id != _canonical_json_id(declaration):
        raise RemoteCiError("dataset manifest id does not match its declaration")
    archives = declaration.get("archives")
    calibration = declaration.get("calibration")
    if not isinstance(archives, dict) or not isinstance(calibration, dict):
        raise RemoteCiError("dataset manifest archives/calibration are invalid")
    if set(source_archives) != set(archives):
        raise RemoteCiError(
            f"dataset archive set mismatch: expected {sorted(archives)}, "
            f"got {sorted(source_archives)}"
        )

    store = Path(dataset_store)
    canonical_archives = store / "archives" / "sha256"
    incoming = store / "incoming"
    canonical_archives.mkdir(parents=True, exist_ok=True)
    incoming.mkdir(parents=True, exist_ok=True)
    promoted: dict[str, Path] = {}
    for sequence in sorted(archives):
        expected = archives[sequence]
        digest = str(expected["sha256"])
        final = canonical_archives / f"{digest}.tar"
        partial = incoming / f"{digest}.part"

        def verify_archive(path: Path, label: str) -> None:
            if path.stat().st_size != int(expected["size"]):
                raise RemoteCiError(f"{label} archive size conflict: {path}")
            if sha256_file(path) != digest:
                raise RemoteCiError(f"{label} archive sha256 conflict: {path}")

        final_present = final.exists() or final.is_symlink()
        partial_present = partial.exists() or partial.is_symlink()
        if final_present:
            if final.is_symlink() or not stat.S_ISREG(final.lstat().st_mode):
                raise RemoteCiError(f"canonical archive path is not regular: {final}")
            if partial_present:
                raise RemoteCiError(
                    f"partial archive conflicts with published archive: {partial}"
                )
            verify_archive(final, "canonical")
        elif partial_present:
            if partial.is_symlink() or not stat.S_ISREG(partial.lstat().st_mode):
                raise RemoteCiError(f"partial archive path is not regular: {partial}")
            verify_archive(partial, "partial")
            os.replace(partial, final)
        else:
            source = Path(source_archives[sequence])
            transfer(source, partial)
            if not partial.is_file():
                raise RemoteCiError(f"archive transfer did not create .part file: {partial}")
            verify_archive(partial, "transferred")
            os.replace(partial, final)
        final.chmod(0o444)
        promoted[sequence] = final

    staging_parent = store / "staging"
    staging_parent.mkdir(parents=True, exist_ok=True)
    staging = staging_parent / f"dataset-{os.getpid()}-{secrets.token_hex(4)}"
    newly_written_receipt: Path | None = None
    try:
        staging.mkdir()
        records: dict[str, Any] = {}
        for sequence in sorted(archives):
            expected = {
                **archives[sequence],
                "camchain_sha256": calibration["camchain_sha256"],
                "imu_config_sha256": calibration["imu_config_sha256"],
            }
            record = audit_and_extract_tumvi_archive(
                promoted[sequence],
                staging / sequence,
                sequence=sequence,
                expected=expected,
            )
            record["filename"] = archives[sequence]["filename"]
            record["canonical_archive"] = (
                f"archives/sha256/{archives[sequence]['sha256']}.tar"
            )
            records[sequence] = record
        _freeze_tumvi_dataset_tree(staging, include_root=False)
        tree_manifest = build_tumvi_dataset_manifest(
            staging,
            archive_records=records,
            calibration=calibration,
        )
        receipt = _tum_vi_stage_receipt(definition, tree_manifest)
        content_id = str(tree_manifest["dataset_content_id"])
        content_hex = _validate_sha256_content_id(
            content_id, "dataset content id"
        ).removeprefix("sha256:")
        published = store / "by-content" / "sha256" / content_hex
        receipt_path = store / "receipts" / "sha256" / f"{content_hex}.json"
        if receipt_path.is_symlink():
            raise RemoteCiError(f"dataset receipt path is a symlink: {receipt_path}")
        if receipt_path.exists() and read_json(receipt_path) != receipt:
            raise RemoteCiError(f"dataset receipt conflicts with staged content: {receipt_path}")
        if published.exists() or published.is_symlink():
            if published.is_symlink():
                raise RemoteCiError(f"published dataset root is a symlink: {published}")
            _verify_tum_vi_published_dataset(
                published,
                expected_content_id=content_id,
                expected_manifest_id=manifest_id,
            )
            _discard_tumvi_staging(staging)
        else:
            published.parent.mkdir(parents=True, exist_ok=True)
            sealed_staging = published.parent / (
                f".staging-{os.getpid()}-{secrets.token_hex(4)}"
            )
            os.replace(staging, sealed_staging)
            staging = sealed_staging
            atomic_write_json(staging / ".phad-dataset.json", receipt)
            _freeze_tumvi_dataset_tree(staging, include_root=True)
            os.replace(staging, published)
        if not receipt_path.exists():
            atomic_write_json(receipt_path, receipt)
            newly_written_receipt = receipt_path
        receipt_path.chmod(0o444)
        newly_written_receipt = None
        return receipt
    except Exception:
        with contextlib.suppress(OSError, RemoteCiError):
            _discard_tumvi_staging(staging)
        if newly_written_receipt is not None:
            with contextlib.suppress(OSError):
                newly_written_receipt.unlink()
        raise


def _git_bytes(root: Path, arguments: Sequence[str]) -> bytes:
    result = run_command(
        ["git", "-C", str(root), *arguments], text=False, capture_output=True
    )
    return bytes(result.stdout)


def _git_text(root: Path, arguments: Sequence[str]) -> str:
    result = run_command(["git", "-C", str(root), *arguments])
    return str(result.stdout).strip()


def _split_nul_paths(value: bytes) -> list[str]:
    return [os.fsdecode(item) for item in value.split(b"\0") if item]


def _validate_snapshot_path(relative: str) -> PurePosixPath:
    path = PurePosixPath(relative)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise RemoteCiError(f"unsafe source path returned by Git: {relative!r}")
    if path.as_posix() != relative.replace(os.sep, "/"):
        raise RemoteCiError(f"non-canonical source path returned by Git: {relative!r}")
    return path


def _snapshot_path_is_excluded(path: PurePosixPath) -> bool:
    for prefix in SNAPSHOT_EXCLUDED_PREFIXES:
        if path == prefix or prefix in path.parents:
            return True
    return False


def _snapshot_diff_pathspecs() -> list[str]:
    pathspecs = ["."]
    for prefix in SNAPSHOT_EXCLUDED_PREFIXES:
        value = prefix.as_posix()
        pathspecs.extend((f":(exclude){value}", f":(exclude){value}/**"))
    return pathspecs


def _hash_snapshot_entry(path: Path) -> tuple[str, int, str]:
    metadata = path.lstat()
    mode = stat.S_IMODE(metadata.st_mode)
    if stat.S_ISREG(metadata.st_mode):
        return "file", mode, sha256_file(path)
    if stat.S_ISLNK(metadata.st_mode):
        target = os.fsencode(os.readlink(path))
        return "symlink", mode, hashlib.sha256(target).hexdigest()
    raise RemoteCiError(f"unsupported source entry type: {path}")


def build_source_manifest(root: Path) -> tuple[dict[str, Any], str]:
    """生成不含时间戳的 canonical source manifest 与 source_id。"""

    root = root.resolve()
    head = _git_text(root, ["rev-parse", "HEAD"])
    tree = _git_text(root, ["rev-parse", "HEAD^{tree}"])
    branch = _git_text(root, ["rev-parse", "--abbrev-ref", "HEAD"])

    tracked = set(
        _split_nul_paths(_git_bytes(root, ["ls-files", "-z", "--cached"]))
    )
    candidates = _split_nul_paths(
        _git_bytes(
            root,
            ["ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        )
    )

    entries: list[dict[str, Any]] = []
    deleted_tracked: list[str] = []
    included_untracked = 0
    for relative in sorted(candidates, key=os.fsencode):
        posix_path = _validate_snapshot_path(relative)
        if _snapshot_path_is_excluded(posix_path):
            continue
        absolute = root / Path(*posix_path.parts)
        if not absolute.exists() and not absolute.is_symlink():
            if relative in tracked:
                deleted_tracked.append(posix_path.as_posix())
            continue
        entry_type, mode, digest = _hash_snapshot_entry(absolute)
        entry = {
            "path": posix_path.as_posix(),
            "type": entry_type,
            "mode": f"{mode:04o}",
            "sha256": digest,
        }
        if entry_type == "file":
            entry["size"] = absolute.stat().st_size
        else:
            entry["target"] = os.readlink(absolute)
        entries.append(entry)
        if relative not in tracked:
            included_untracked += 1

    dirty_result = run_command(
        [
            "git",
            "-C",
            str(root),
            "diff",
            "--quiet",
            "HEAD",
            "--",
            *_snapshot_diff_pathspecs(),
        ],
        check=False,
    )
    if dirty_result.returncode not in (0, 1):
        raise RemoteCiError("git diff --quiet HEAD failed while building source manifest")

    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "git": {
            "head": head,
            "tree": tree,
            "branch": branch,
            "tracked_dirty": dirty_result.returncode == 1,
        },
        "entries": entries,
        "deleted_tracked": sorted(deleted_tracked, key=os.fsencode),
        "included_untracked": included_untracked,
        "excluded_prefixes": [path.as_posix() for path in SNAPSHOT_EXCLUDED_PREFIXES],
    }
    canonical = json.dumps(
        manifest, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    source_id = f"sha256:{hashlib.sha256(canonical).hexdigest()}"
    return manifest, source_id


def _normalized_tar_info(info: tarfile.TarInfo, mode: int) -> tarfile.TarInfo:
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    info.mode = mode
    return info


def create_source_inputs(root: Path, output: Path) -> dict[str, Any]:
    manifest, source_id = build_source_manifest(root)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "source-manifest.json"
    atomic_write_json(
        manifest_path,
        {**manifest, "source_id": source_id},
    )

    tar_path = output / "source.tar"
    with tarfile.open(tar_path, mode="w", format=tarfile.PAX_FORMAT) as archive:
        for entry in manifest["entries"]:
            relative = PurePosixPath(entry["path"])
            source = root / Path(*relative.parts)
            info = archive.gettarinfo(str(source), arcname=entry["path"])
            info = _normalized_tar_info(info, int(entry["mode"], 8))
            if entry["type"] == "file":
                with source.open("rb") as stream:
                    archive.addfile(info, stream)
            else:
                archive.addfile(info)

    archive_path = output / "source.tar.zst"
    run_command(
        [
            "zstd",
            "-T0",
            "-3",
            "--no-progress",
            "--force",
            str(tar_path),
            "-o",
            str(archive_path),
        ],
        capture_output=False,
    )

    bundle_path = output / "base.bundle"
    run_command(
        ["git", "-C", str(root), "bundle", "create", str(bundle_path), "HEAD"]
    )
    run_command(["git", "bundle", "verify", str(bundle_path)])
    return {
        "source_id": source_id,
        "manifest": manifest,
        "manifest_path": manifest_path,
        "archive_path": archive_path,
        "archive_sha256": sha256_file(archive_path),
        "bundle_path": bundle_path,
        "bundle_sha256": sha256_file(bundle_path),
    }


def verify_source_tree(source_root: Path, manifest_path: Path) -> dict[str, Any]:
    document = read_json(manifest_path)
    source_id = str(document.pop("source_id", ""))
    canonical = json.dumps(
        document, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    actual_source_id = f"sha256:{hashlib.sha256(canonical).hexdigest()}"
    if source_id != actual_source_id:
        raise RemoteCiError(
            f"source manifest id mismatch: expected {source_id}, got {actual_source_id}"
        )

    for entry in document.get("entries", []):
        relative = _validate_snapshot_path(str(entry["path"]))
        path = source_root / Path(*relative.parts)
        if not path.exists() and not path.is_symlink():
            raise RemoteCiError(f"snapshot entry is missing after extraction: {relative}")
        entry_type, mode, digest = _hash_snapshot_entry(path)
        if entry_type != entry["type"]:
            raise RemoteCiError(f"snapshot type mismatch: {relative}")
        if f"{mode:04o}" != entry["mode"]:
            raise RemoteCiError(
                f"snapshot mode mismatch: {relative}: "
                f"{mode:04o} != {entry['mode']}"
            )
        if digest != entry["sha256"]:
            raise RemoteCiError(f"snapshot digest mismatch: {relative}")

    git = document["git"]
    head = _git_text(source_root, ["rev-parse", "HEAD"])
    tree = _git_text(source_root, ["rev-parse", "HEAD^{tree}"])
    if head != git["head"] or tree != git["tree"]:
        raise RemoteCiError("reconstructed Git HEAD/tree does not match source manifest")

    branch = str(git["branch"])
    if branch != "HEAD":
        run_command(
            ["git", "-C", str(source_root), "check-ref-format", "--branch", branch]
        )
        run_command(
            [
                "git",
                "-C",
                str(source_root),
                "update-ref",
                f"refs/heads/{branch}",
                head,
            ]
        )
        run_command(
            [
                "git",
                "-C",
                str(source_root),
                "symbolic-ref",
                "HEAD",
                f"refs/heads/{branch}",
            ]
        )

    _, reconstructed_source_id = build_source_manifest(source_root)
    if reconstructed_source_id != source_id:
        raise RemoteCiError(
            "reconstructed Git worktree state does not match source manifest: "
            f"{reconstructed_source_id} != {source_id}"
        )

    return {
        "schema_version": SCHEMA_VERSION,
        "source_id": source_id,
        "verified_utc": utc_now(),
        "git_head": head,
        "git_tree": tree,
        "git_branch": branch,
        "entry_count": len(document.get("entries", [])),
        "deleted_tracked_count": len(document.get("deleted_tracked", [])),
    }


def toolchain_identity(root: Path) -> tuple[str, list[Path]]:
    context = root / "docker" / "remote-ci"
    files = sorted(
        path
        for path in context.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and path.suffix not in {".pyc", ".pyo"}
    )
    if not files:
        raise RemoteCiError(f"Docker build context is empty: {context}")
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(context).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    return digest.hexdigest(), files


def ensure_gtsam_archive(root: Path) -> Path:
    cache = (
        root
        / LOCAL_ARTIFACT_ROOT
        / "cache"
        / "toolchains"
        / "gtsam"
        / GTSAM_COMMIT
    )
    archive = cache / "gtsam.tar.gz"
    if archive.exists():
        actual = sha256_file(archive)
        if actual != GTSAM_ARCHIVE_SHA256:
            raise RemoteCiError(
                f"cached GTSAM archive digest mismatch: {actual}"
            )
        return archive

    cache.mkdir(parents=True, exist_ok=True)
    temporary = cache / f".gtsam.tar.gz.partial-{os.getpid()}"
    run_command(
        [
            "curl",
            "--fail",
            "--location",
            "--retry",
            "3",
            "--connect-timeout",
            "15",
            "--max-time",
            "900",
            "--output",
            str(temporary),
            GTSAM_ARCHIVE_URL,
        ],
        capture_output=False,
    )
    actual = sha256_file(temporary)
    if actual != GTSAM_ARCHIVE_SHA256:
        raise RemoteCiError(
            f"downloaded GTSAM archive digest mismatch: {actual}"
        )
    os.replace(temporary, archive)
    return archive


def _inspect_image(reference: str) -> dict[str, Any] | None:
    result = run_command(
        ["docker", "image", "inspect", reference], check=False
    )
    if result.returncode != 0:
        return None
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid docker image inspect output: {error}") from error
    if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], dict):
        raise RemoteCiError("unexpected docker image inspect result")
    return value[0]


def ensure_local_image(root: Path) -> dict[str, Any]:
    toolchain_id, _ = toolchain_identity(root)
    gtsam_archive = ensure_gtsam_archive(root)
    tag = f"phad-vio-remote-ci:{toolchain_id[:16]}"
    image = _inspect_image(tag)
    labels = ((image or {}).get("Config") or {}).get("Labels") or {}
    if image is None or labels.get("phad.remote-ci.toolchain-id") != toolchain_id:
        context = root / "docker" / "remote-ci"
        run_command(
            [
                "docker",
                "build",
                "--platform",
                "linux/amd64",
                "--build-context",
                f"gtsam-source={gtsam_archive.parent}",
                "--label",
                f"phad.remote-ci.toolchain-id={toolchain_id}",
                "--label",
                f"phad.remote-ci.gtsam-commit={GTSAM_COMMIT}",
                "--tag",
                tag,
                str(context),
            ],
            capture_output=False,
        )
        image = _inspect_image(tag)
    if image is None:
        raise RemoteCiError(f"Docker image was not created: {tag}")

    run_command(
        [
            "docker",
            "run",
            "--rm",
            "--pull",
            "never",
            "--platform",
            "linux/amd64",
            "--network",
            "none",
            "--read-only",
            "--cap-drop",
            "ALL",
            "--security-opt",
            "no-new-privileges",
            "--tmpfs",
            "/tmp:rw,nosuid,nodev,size=64m",
            tag,
            "/bin/bash",
            "-lc",
            (
                "/usr/local/bin/phad-remote-ci-smoke && "
                "zstd --version >/dev/null && "
                "/opt/phad-venv/bin/python -c "
                "'import numpy; print(numpy.__version__)'"
            ),
        ],
        capture_output=False,
    )
    versions_result = run_command(
        [
            "docker",
            "run",
            "--rm",
            "--pull",
            "never",
            "--platform",
            "linux/amd64",
            "--network",
            "none",
            "--read-only",
            "--cap-drop",
            "ALL",
            "--security-opt",
            "no-new-privileges",
            tag,
            "cat",
            "/opt/phad-toolchain.json",
        ]
    )
    try:
        versions = json.loads(versions_result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid toolchain version manifest: {error}") from error
    return {
        "toolchain_id": toolchain_id,
        "dockerfile_sha256": sha256_file(
            root / "docker" / "remote-ci" / "Dockerfile"
        ),
        "gtsam_archive_sha256": GTSAM_ARCHIVE_SHA256,
        "tag": tag,
        "id": image["Id"],
        "repo_digests": image.get("RepoDigests") or [],
        "architecture": image.get("Architecture"),
        "os": image.get("Os"),
        "rootfs_layers": (image.get("RootFS") or {}).get("Layers") or [],
        "versions": versions,
    }


def _archive_member(archive: Path, member: str) -> bytes:
    result = run_command(
        [
            "tar",
            "--use-compress-program=unzstd",
            "-xOf",
            str(archive),
            member,
        ],
        text=False,
    )
    return bytes(result.stdout)


def parse_oci_image_identity(
    index_bytes: bytes,
    manifest_bytes: bytes,
    config_bytes: bytes,
    *,
    tag: str,
    local_native_id: str,
) -> dict[str, str]:
    try:
        index = json.loads(index_bytes)
        manifest = json.loads(manifest_bytes)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid OCI image metadata: {error}") from error

    descriptors = index.get("manifests") if isinstance(index, dict) else None
    if not isinstance(descriptors, list) or len(descriptors) != 1:
        raise RemoteCiError("expected one manifest in exported OCI image")
    descriptor = descriptors[0]
    if not isinstance(descriptor, dict):
        raise RemoteCiError("invalid OCI image manifest descriptor")
    annotations = descriptor.get("annotations") or {}
    image_name = annotations.get("io.containerd.image.name")
    reference = annotations.get("org.opencontainers.image.ref.name")
    if image_name not in {tag, f"docker.io/library/{tag}"} and reference != tag.rsplit(
        ":", 1
    )[-1]:
        raise RemoteCiError(f"exported OCI image does not identify tag {tag}")

    manifest_id = str(descriptor.get("digest"))
    actual_manifest_id = f"sha256:{hashlib.sha256(manifest_bytes).hexdigest()}"
    if manifest_id != actual_manifest_id:
        raise RemoteCiError(
            f"OCI manifest digest mismatch: {actual_manifest_id} != {manifest_id}"
        )
    config_descriptor = manifest.get("config") if isinstance(manifest, dict) else None
    if not isinstance(config_descriptor, dict):
        raise RemoteCiError("OCI manifest has no config descriptor")
    config_id = str(config_descriptor.get("digest"))
    actual_config_id = f"sha256:{hashlib.sha256(config_bytes).hexdigest()}"
    if config_id != actual_config_id:
        raise RemoteCiError(
            f"OCI config digest mismatch: {actual_config_id} != {config_id}"
        )
    if local_native_id != config_id:
        raise RemoteCiError(
            f"local Docker image id does not match OCI config: "
            f"{local_native_id} != {config_id}"
        )
    return {"oci_manifest_id": manifest_id, "oci_config_id": config_id}


def inspect_oci_image_archive(
    archive: Path, *, tag: str, local_native_id: str
) -> dict[str, str]:
    index_bytes = _archive_member(archive, "index.json")
    try:
        index = json.loads(index_bytes)
        descriptor = index["manifests"][0]
        manifest_id = str(descriptor["digest"])
    except (json.JSONDecodeError, KeyError, IndexError, TypeError) as error:
        raise RemoteCiError(f"invalid exported OCI image index: {error}") from error
    if re.fullmatch(r"sha256:[0-9a-f]{64}", manifest_id) is None:
        raise RemoteCiError(f"invalid OCI manifest digest: {manifest_id!r}")
    manifest_bytes = _archive_member(
        archive, f"blobs/sha256/{manifest_id.removeprefix('sha256:')}"
    )
    try:
        manifest = json.loads(manifest_bytes)
        config_id = str(manifest["config"]["digest"])
    except (json.JSONDecodeError, KeyError, TypeError) as error:
        raise RemoteCiError(f"invalid exported OCI manifest: {error}") from error
    if re.fullmatch(r"sha256:[0-9a-f]{64}", config_id) is None:
        raise RemoteCiError(f"invalid OCI config digest: {config_id!r}")
    config_bytes = _archive_member(
        archive, f"blobs/sha256/{config_id.removeprefix('sha256:')}"
    )
    return parse_oci_image_identity(
        index_bytes,
        manifest_bytes,
        config_bytes,
        tag=tag,
        local_native_id=local_native_id,
    )


def export_local_image(root: Path, image: dict[str, Any]) -> dict[str, Any]:
    image_hex = str(image["id"]).removeprefix("sha256:")
    cache = root / LOCAL_ARTIFACT_ROOT / "cache" / "images" / image_hex
    cache.mkdir(parents=True, exist_ok=True)
    archive = cache / "image.tar.zst"
    if not archive.exists():
        temporary = cache / f"image.tar.zst.partial-{os.getpid()}"
        save = subprocess.Popen(
            ["docker", "image", "save", str(image["tag"])],
            stdout=subprocess.PIPE,
        )
        assert save.stdout is not None
        compress = subprocess.Popen(
            [
                "zstd",
                "-T0",
                "-3",
                "--no-progress",
                "--force",
                "-o",
                str(temporary),
            ],
            stdin=save.stdout,
        )
        save.stdout.close()
        compress_rc = compress.wait()
        save_rc = save.wait()
        if save_rc != 0 or compress_rc != 0:
            raise RemoteCiError(
                f"docker image export failed: docker={save_rc}, zstd={compress_rc}"
            )
        os.replace(temporary, archive)
    identities = inspect_oci_image_archive(
        archive, tag=str(image["tag"]), local_native_id=str(image["id"])
    )
    return {
        **image,
        **identities,
        "archive_path": archive,
        "archive_sha256": sha256_file(archive),
        "archive_size": archive.stat().st_size,
    }


class VerifiedSsh:
    """用固定指纹构造不依赖用户 SSH config 的批处理连接。"""

    def __init__(self) -> None:
        self._temporary: tempfile.TemporaryDirectory[str] | None = None
        self.known_hosts: Path | None = None

    def __enter__(self) -> "VerifiedSsh":
        self._temporary = tempfile.TemporaryDirectory(prefix="phad-remote-ci-ssh-")
        self.known_hosts = Path(self._temporary.name) / "known_hosts"
        scan = run_command(
            ["ssh-keyscan", "-T", "5", "-t", "ed25519", remote_identity().host],
            text=False,
        )
        lines = sorted(
            {
                line
                for line in bytes(scan.stdout).splitlines()
                if line and not line.startswith(b"#")
            }
        )
        if not lines:
            raise RemoteCiError("ssh-keyscan returned no ED25519 host key")
        self.known_hosts.write_bytes(b"\n".join(lines) + b"\n")
        self.known_hosts.chmod(0o600)
        fingerprints = run_command(
            ["ssh-keygen", "-lf", str(self.known_hosts), "-E", "sha256"]
        )
        found = set(
            re.findall(r"SHA256:[A-Za-z0-9+/]+", str(fingerprints.stdout))
        )
        if found != {remote_identity().fingerprint}:
            raise RemoteCiError(
                "remote ED25519 fingerprint verification failed: "
                f"expected {remote_identity().fingerprint}, got {sorted(found)}"
            )
        return self

    def __exit__(self, *_: object) -> None:
        if self._temporary is not None:
            self._temporary.cleanup()

    def _options(self) -> list[str]:
        if self.known_hosts is None:
            raise RemoteCiError("SSH context is not active")
        return [
            "-F",
            "/dev/null",
            "-o",
            "BatchMode=yes",
            "-o",
            "PreferredAuthentications=publickey",
            "-o",
            "PasswordAuthentication=no",
            "-o",
            "KbdInteractiveAuthentication=no",
            "-o",
            "ForwardAgent=no",
            "-o",
            "ClearAllForwardings=yes",
            "-o",
            "UpdateHostKeys=no",
            "-o",
            "HostKeyAlgorithms=ssh-ed25519",
            "-o",
            "StrictHostKeyChecking=yes",
            "-o",
            f"UserKnownHostsFile={self.known_hosts}",
            "-o",
            "GlobalKnownHostsFile=/dev/null",
            "-o",
            "ConnectTimeout=10",
        ]

    def run(
        self,
        remote_command: Sequence[str],
        *,
        check: bool = True,
        capture_output: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        command = [
            "ssh",
            *self._options(),
            remote_identity().target,
            shlex.join(list(remote_command)),
        ]
        return run_command(
            command, check=check, capture_output=capture_output, text=True
        )

    def upload(self, local: Path, remote: Path) -> None:
        run_command(
            [
                "scp",
                *self._options(),
                str(local),
                f"{remote_identity().target}:{remote}",
            ],
            capture_output=False,
        )

    def download(self, remote: Path, local: Path) -> None:
        local.parent.mkdir(parents=True, exist_ok=True)
        run_command(
            [
                "scp",
                *self._options(),
                f"{remote_identity().target}:{remote}",
                str(local),
            ],
            capture_output=False,
        )


def remote_doctor(
    ssh: VerifiedSsh,
    profile: str = PROFILE,
    *,
    dataset_root: Path | None = None,
    dataset_content_id: str | None = None,
) -> dict[str, Any]:
    definition = profile_definition(profile)
    if profile == TUM_VI_CONFIRMATORY_PROFILE:
        if dataset_content_id is None:
            raise RemoteCiError(
                "TUM-VI confirmatory doctor requires an explicit dataset content id"
            )
        _validate_sha256_content_id(dataset_content_id, "dataset content id")
        required = (
            "mav0/cam0/data.csv",
            "mav0/cam1/data.csv",
            "mav0/imu0/data.csv",
            "dso/camchain.yaml",
            "dso/imu_config.yaml",
            "dso/gt_imu.csv",
        )
    else:
        required = (
            "mav0/cam0/data.csv",
            "mav0/cam0/sensor.yaml",
            "mav0/cam1/data.csv",
            "mav0/cam1/sensor.yaml",
            "mav0/imu0/data.csv",
            "mav0/imu0/sensor.yaml",
            "mav0/state_groundtruth_estimate0/data.csv",
        )
    resolved_dataset_root = (
        Path(definition["dataset_root"])
        if dataset_root is None
        else Path(dataset_root)
    )
    probe = """
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess

requested_root = Path(%s).absolute()
root = requested_root
dataset_path_error = None
try:
    root_metadata = requested_root.lstat()
    resolved_root = requested_root.resolve(strict=True)
    if not stat.S_ISDIR(root_metadata.st_mode):
        dataset_path_error = (
            f'dataset root is not a real directory: {requested_root}')
    elif resolved_root != requested_root:
        dataset_path_error = (
            f'dataset root resolves through a symlink: '
            f'{requested_root} -> {resolved_root}')
except OSError as error:
    dataset_path_error = f'dataset root is unavailable: {requested_root}: {error}'
sequences = %s
required = %s
expected_manifest_id = %s
expected_manifest = %s
expected_content_id = %s
missing = {}
if dataset_path_error is None:
    for sequence in sequences:
        absent = [item for item in required if not os.access(root / sequence / item, os.R_OK)]
        if absent:
            missing[sequence] = absent
manifest_id = None
manifest_error = dataset_path_error
if expected_content_id is not None and manifest_error is None:
    manifest_path = root / '.phad-dataset.json'
    try:
        manifest = json.loads(manifest_path.read_text())
        manifest_id = manifest.get('dataset_manifest_id')
        content_id = manifest.get('dataset_content_id')
        if content_id != expected_content_id:
            manifest_error = (
                f'dataset sentinel content mismatch: {content_id!r} != '
                f'{expected_content_id!r}')
        elif manifest_id != expected_manifest_id:
            manifest_error = (
                f'dataset sentinel manifest mismatch: {manifest_id!r} != '
                f'{expected_manifest_id!r}')
        elif manifest.get('dataset_manifest') != expected_manifest:
            manifest_error = 'dataset sentinel declaration mismatch'
    except (OSError, json.JSONDecodeError, AttributeError) as error:
        manifest_error = f'dataset manifest is unavailable or invalid: {error}'
docker = subprocess.run(
    ['docker', 'version', '--format', '{{json .}}'],
    check=True, capture_output=True, text=True)
info = subprocess.run(
    ['docker', 'info', '--format', '{{json .}}'],
    check=True, capture_output=True, text=True)
docker_version = json.loads(docker.stdout)
docker_info = json.loads(info.stdout)
required_tools = (
    'docker', 'git', 'nohup', 'python3', 'setsid', 'sha256sum', 'tar', 'zstd')
print(json.dumps({
    'uid': os.getuid(),
    'gid': os.getgid(),
    'architecture': os.uname().machine,
    'docker': {
        'client_version': docker_version['Client']['Version'],
        'server_version': docker_version['Server']['Version'],
        'cpus': docker_info['NCPU'],
        'memory_bytes': docker_info['MemTotal'],
        'root_dir': docker_info['DockerRootDir'],
        'security_options': docker_info['SecurityOptions'],
    },
    'missing_tools': [tool for tool in required_tools if shutil.which(tool) is None],
    'project_parent_writable': os.access('/home/lin/Projects', os.W_OK),
    'dataset_root': str(requested_root),
    'dataset_missing': missing,
    'dataset_manifest_id': manifest_id,
    'dataset_manifest_error': manifest_error,
}, sort_keys=True))
""" % (
        repr(str(resolved_dataset_root)),
        repr(tuple(definition["sequences"])),
        repr(required),
        repr(definition.get("dataset_manifest_id")),
        repr(definition.get("dataset_manifest")),
        repr(dataset_content_id),
    )
    result = ssh.run(["python3", "-c", probe])
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"remote doctor returned invalid JSON: {error}") from error
    if value.get("architecture") != "x86_64":
        raise RemoteCiError(f"unsupported remote architecture: {value.get('architecture')}")
    if value.get("dataset_missing"):
        raise RemoteCiError(
            f"remote {definition['dataset_format']} dataset is incomplete: "
            f"{value['dataset_missing']}"
        )
    if value.get("dataset_manifest_error"):
        raise RemoteCiError(str(value["dataset_manifest_error"]))
    if value.get("missing_tools"):
        raise RemoteCiError(f"remote commands are missing: {value['missing_tools']}")
    if not value.get("project_parent_writable"):
        raise RemoteCiError("remote project parent is not writable")
    return value


def _remote_file_sha256(ssh: VerifiedSsh, path: Path) -> str | None:
    result = ssh.run(["sha256sum", str(path)], check=False)
    if result.returncode != 0:
        return None
    fields = str(result.stdout).split()
    return fields[0] if fields else None


def upload_exact(
    ssh: VerifiedSsh, local: Path, remote: Path, expected_sha256: str
) -> None:
    existing = _remote_file_sha256(ssh, remote)
    if existing is not None:
        if existing != expected_sha256:
            raise RemoteCiError(f"remote file exists with another digest: {remote}")
        return

    ssh.run(["mkdir", "-p", str(remote.parent)])
    temporary = remote.with_name(f".{remote.name}.partial-{secrets.token_hex(5)}")
    ssh.upload(local, temporary)
    uploaded = _remote_file_sha256(ssh, temporary)
    if uploaded != expected_sha256:
        raise RemoteCiError(
            f"uploaded checksum mismatch for {remote}: {uploaded} != {expected_sha256}"
        )
    ssh.run(["mv", "-n", str(temporary), str(remote)])
    final = _remote_file_sha256(ssh, remote)
    if final != expected_sha256:
        raise RemoteCiError(f"remote checksum verification failed: {remote}")


def upload_tumvi_archive_part(
    ssh: VerifiedSsh,
    local: Path,
    remote_part: Path,
    expected_sha256: str,
) -> None:
    """Upload one large archive to its deterministic, resumable inspection path."""

    existing = _remote_file_sha256(ssh, remote_part)
    if existing is not None:
        if existing == expected_sha256:
            return
        raise RemoteCiError(
            "remote TUM-VI archive part conflicts with expected digest; "
            f"cleanup target: {remote_part}"
        )

    ssh.run(["mkdir", "-p", str(remote_part.parent)])
    reserve = """
import os
from pathlib import Path
import sys

path = Path(sys.argv[1])
try:
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
except FileExistsError:
    raise SystemExit(17)
else:
    os.close(descriptor)
"""
    reserved = ssh.run(
        ["python3", "-c", reserve, str(remote_part)], check=False
    )
    if reserved.returncode != 0:
        raced = _remote_file_sha256(ssh, remote_part)
        if raced == expected_sha256:
            return
        raise RemoteCiError(
            "remote TUM-VI archive part is owned by another or interrupted "
            f"transfer; cleanup target: {remote_part}"
        )

    try:
        ssh.upload(local, remote_part)
    except Exception as error:
        raise RemoteCiError(
            "TUM-VI archive transfer was interrupted; partial data was kept; "
            f"cleanup target: {remote_part}"
        ) from error
    uploaded = _remote_file_sha256(ssh, remote_part)
    if uploaded != expected_sha256:
        raise RemoteCiError(
            "uploaded TUM-VI archive checksum mismatch; partial data was kept; "
            f"cleanup target: {remote_part}: {uploaded} != {expected_sha256}"
        )


def _validate_local_tumvi_archives(
    local_root: Path, definition: dict[str, Any]
) -> dict[str, Path]:
    declaration = definition.get("dataset_manifest")
    archives = declaration.get("archives") if isinstance(declaration, dict) else None
    if not isinstance(archives, dict):
        raise RemoteCiError("TUM-VI profile has no archive declaration")
    root = Path(local_root)
    result: dict[str, Path] = {}
    failures: list[str] = []
    for sequence in sorted(archives):
        expected = archives[sequence]
        path = root / str(expected["filename"])
        try:
            metadata = path.lstat()
        except OSError as error:
            failures.append(f"{sequence}: archive is unavailable: {path}: {error}")
            continue
        if not stat.S_ISREG(metadata.st_mode):
            failures.append(f"{sequence}: archive is not a regular file: {path}")
            continue
        if metadata.st_size != int(expected["size"]):
            failures.append(
                f"{sequence}: archive size mismatch: "
                f"{metadata.st_size} != {expected['size']}"
            )
            continue
        actual_sha256 = sha256_file(path)
        if actual_sha256 != str(expected["sha256"]):
            failures.append(
                f"{sequence}: archive sha256 mismatch: "
                f"{actual_sha256} != {expected['sha256']}"
            )
            continue
        result[sequence] = path
    if failures:
        raise RemoteCiError("local TUM-VI archive preflight failed: " + "; ".join(failures))
    return result


def _prepare_remote_tumvi_control(
    ssh: VerifiedSsh, definition: dict[str, Any]
) -> tuple[Path, Path]:
    store = Path(str(definition["dataset_root"]))
    manifest_id = str(definition["dataset_manifest_id"])
    _validate_sha256_content_id(manifest_id, "dataset manifest id")
    script = Path(__file__).resolve()
    script_sha256 = sha256_file(script)
    control = (
        store
        / "control"
        / f"{manifest_id.removeprefix('sha256:')}-{script_sha256}"
    )
    remote_script = control / "remote_ci.py"
    remote_manifest = control / "dataset-manifest.json"
    upload_exact(ssh, script, remote_script, script_sha256)
    with tempfile.TemporaryDirectory(prefix="phad-tumvi-manifest-") as temporary:
        local_manifest = Path(temporary) / "dataset-manifest.json"
        atomic_write_json(local_manifest, definition)
        upload_exact(
            ssh,
            local_manifest,
            remote_manifest,
            sha256_file(local_manifest),
        )
    return remote_script, remote_manifest


def _remote_json_object(result: subprocess.CompletedProcess[str], label: str) -> dict[str, Any]:
    try:
        value = json.loads(str(result.stdout))
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"{label} returned invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise RemoteCiError(f"{label} did not return a JSON object")
    return value


def _validate_tumvi_receipt(
    receipt: dict[str, Any], definition: dict[str, Any]
) -> str:
    if receipt.get("dataset_manifest_id") != definition.get("dataset_manifest_id"):
        raise RemoteCiError("remote dataset receipt manifest id mismatch")
    if receipt.get("dataset_manifest") != definition.get("dataset_manifest"):
        raise RemoteCiError("remote dataset receipt declaration mismatch")
    content_id = receipt.get("dataset_content_id")
    if not isinstance(content_id, str):
        raise RemoteCiError("remote dataset receipt has no content id")
    return _validate_sha256_content_id(content_id, "dataset content id")


def stage_remote_tumvi_dataset(
    local_root: Path,
    *,
    profile: str = TUM_VI_CONFIRMATORY_PROFILE,
) -> dict[str, Any]:
    """Upload frozen archives and execute the audited staging transaction remotely."""

    if profile != TUM_VI_CONFIRMATORY_PROFILE:
        raise RemoteCiError(f"unsupported dataset staging profile: {profile}")
    definition = profile_definition(profile)
    local_archives = _validate_local_tumvi_archives(Path(local_root), definition)
    store = Path(str(definition["dataset_root"]))
    with VerifiedSsh() as ssh:
        remote_script, remote_manifest = _prepare_remote_tumvi_control(
            ssh, definition
        )
        remote_archives: dict[str, Path] = {}
        for sequence in sorted(local_archives):
            expected = definition["dataset_manifest"]["archives"][sequence]
            digest = str(expected["sha256"])
            canonical = store / "archives" / "sha256" / f"{digest}.tar"
            canonical_digest = _remote_file_sha256(ssh, canonical)
            if canonical_digest is not None:
                if canonical_digest != digest:
                    raise RemoteCiError(
                        f"remote canonical archive digest conflict: {canonical}"
                    )
                remote_archives[sequence] = canonical
                continue
            partial = store / "incoming" / f"{digest}.part"
            upload_tumvi_archive_part(
                ssh, local_archives[sequence], partial, digest
            )
            remote_archives[sequence] = partial

        command = [
            "python3",
            str(remote_script),
            "_remote-stage-tumvi-dataset",
            "--profile-definition",
            str(remote_manifest),
            "--dataset-store",
            str(store),
        ]
        for sequence in sorted(remote_archives):
            command.extend(
                ["--archive", f"{sequence}={remote_archives[sequence]}"]
            )
        receipt = _remote_json_object(
            ssh.run(command), "remote TUM-VI dataset staging"
        )
        content_id = _validate_tumvi_receipt(receipt, definition)
        content_hex = content_id.removeprefix("sha256:")
        remote_receipt = store / "receipts" / "sha256" / f"{content_hex}.json"
        with tempfile.TemporaryDirectory(prefix="phad-tumvi-receipt-") as temporary:
            local_receipt = Path(temporary) / "receipt.json"
            ssh.download(remote_receipt, local_receipt)
            downloaded = read_json(local_receipt)
        if downloaded != receipt:
            raise RemoteCiError("downloaded dataset receipt differs from remote result")
        return receipt


def verify_remote_tumvi_dataset(
    profile: str, dataset_content_id: str
) -> dict[str, Any]:
    """Run a full remote tree verification for one explicitly bound content root."""

    if profile != TUM_VI_CONFIRMATORY_PROFILE:
        raise RemoteCiError(f"unsupported dataset verification profile: {profile}")
    definition = profile_definition(profile)
    content_hex = _validate_sha256_content_id(
        dataset_content_id, "dataset content id"
    ).removeprefix("sha256:")
    store = Path(str(definition["dataset_root"]))
    dataset_root = store / "by-content" / "sha256" / content_hex
    with VerifiedSsh() as ssh:
        remote_script, remote_manifest = _prepare_remote_tumvi_control(
            ssh, definition
        )
        result = ssh.run(
            [
                "python3",
                str(remote_script),
                "_remote-verify-tumvi-dataset",
                "--profile-definition",
                str(remote_manifest),
                "--dataset-root",
                str(dataset_root),
                "--dataset-content-id",
                dataset_content_id,
            ]
        )
    receipt = _remote_json_object(result, "remote TUM-VI dataset verification")
    _validate_tumvi_receipt(receipt, definition)
    return receipt


def _remote_image_inspect(
    ssh: VerifiedSsh, reference: str
) -> dict[str, Any] | None:
    result = ssh.run(["docker", "image", "inspect", reference], check=False)
    if result.returncode != 0:
        return None
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid remote image inspect output: {error}") from error
    if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], dict):
        raise RemoteCiError("unexpected remote image inspect result")
    return value[0]


def _verify_remote_image(
    ssh: VerifiedSsh, image: dict[str, Any], inspected: dict[str, Any]
) -> str:
    remote_id = str(inspected.get("Id"))
    expected_ids = {str(image["oci_config_id"]), str(image["oci_manifest_id"])}
    if remote_id not in expected_ids:
        raise RemoteCiError(
            f"remote image identity mismatch: {remote_id} not in "
            f"{sorted(expected_ids)}"
        )
    descriptor = inspected.get("Descriptor") or {}
    descriptor_digest = descriptor.get("digest")
    if descriptor_digest is not None and descriptor_digest != image["oci_manifest_id"]:
        raise RemoteCiError(
            f"remote OCI descriptor mismatch: {descriptor_digest} != "
            f"{image['oci_manifest_id']}"
        )
    remote_layers = (inspected.get("RootFS") or {}).get("Layers") or []
    if remote_layers != image["rootfs_layers"]:
        raise RemoteCiError("remote image RootFS layer digests do not match")
    if inspected.get("Architecture") != image["architecture"] or inspected.get(
        "Os"
    ) != image["os"]:
        raise RemoteCiError("remote image platform does not match")
    labels = (inspected.get("Config") or {}).get("Labels") or {}
    if labels.get("phad.remote-ci.toolchain-id") != image["toolchain_id"]:
        raise RemoteCiError("remote image toolchain label does not match")

    remote_uid = str(ssh.run(["id", "-u"]).stdout).strip()
    remote_gid = str(ssh.run(["id", "-g"]).stdout).strip()
    smoke = ssh.run(
        [
            "docker",
            "run",
            "--rm",
            "--pull",
            "never",
            "--platform",
            "linux/amd64",
            "--network",
            "none",
            "--read-only",
            "--cap-drop",
            "ALL",
            "--security-opt",
            "no-new-privileges",
            "--user",
            f"{remote_uid}:{remote_gid}",
            "--tmpfs",
            "/tmp:rw,nosuid,nodev,size=64m",
            remote_id,
            "/bin/bash",
            "-lc",
            "/usr/local/bin/phad-remote-ci-smoke >/dev/null && "
            "zstd --version >/dev/null && "
            "cat /opt/phad-toolchain.json",
        ]
    )
    try:
        remote_versions = json.loads(smoke.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid remote toolchain manifest: {error}") from error
    if remote_versions != image["versions"]:
        raise RemoteCiError("remote toolchain version manifest does not match")
    return remote_id


def ensure_remote_image(ssh: VerifiedSsh, image: dict[str, Any]) -> dict[str, Any]:
    inspected = _remote_image_inspect(ssh, str(image["tag"]))
    if inspected is not None:
        remote_id = _verify_remote_image(ssh, image, inspected)
        return {**image, "remote_id": remote_id, "transferred": False}

    for candidate in (str(image["oci_manifest_id"]), str(image["oci_config_id"])):
        inspected = _remote_image_inspect(ssh, candidate)
        if inspected is not None:
            ssh.run(["docker", "image", "tag", candidate, str(image["tag"])])
            remote_id = _verify_remote_image(ssh, image, inspected)
            return {**image, "remote_id": remote_id, "transferred": False}

    image_hex = str(image["id"]).removeprefix("sha256:")
    remote_archive = REMOTE_ROOT / "images" / image_hex / "image.tar.zst"
    upload_exact(
        ssh,
        Path(image["archive_path"]),
        remote_archive,
        str(image["archive_sha256"]),
    )
    ssh.run(
        ["docker", "image", "load", "--input", str(remote_archive)],
        capture_output=False,
    )
    inspected = _remote_image_inspect(ssh, str(image["tag"]))
    if inspected is None:
        raise RemoteCiError("remote docker load did not create the expected tag")
    remote_id = _verify_remote_image(ssh, image, inspected)
    return {
        **image,
        "remote_id": remote_id,
        "transferred": True,
        "remote_archive": str(remote_archive),
    }


def ensure_remote_source(
    ssh: VerifiedSsh, source: dict[str, Any]
) -> dict[str, Any]:
    source_hex = str(source["source_id"]).removeprefix("sha256:")
    source_root = REMOTE_ROOT / "sources" / source_hex
    ready_path = source_root / "ready.json"
    ready_result = ssh.run(["cat", str(ready_path)], check=False)
    if ready_result.returncode == 0:
        try:
            ready = json.loads(ready_result.stdout)
        except json.JSONDecodeError as error:
            raise RemoteCiError(f"invalid remote source sentinel: {ready_path}") from error
        if (
            ready.get("source_id") != source["source_id"]
            or ready.get("archive_sha256") != source["archive_sha256"]
            or ready.get("bundle_sha256") != source["bundle_sha256"]
        ):
            raise RemoteCiError(f"remote source sentinel mismatch: {source_root}")
        remote_script = source_root / "repo" / "scripts" / "remote_ci.py"
        verify = ssh.run(
            [
                "python3",
                str(remote_script),
                "_remote-verify-source",
                "--source-root",
                str(source_root / "repo"),
                "--manifest",
                str(source_root / "inputs" / "source-manifest.json"),
                "--ready",
                str(ready_path),
                "--archive-sha256",
                str(source["archive_sha256"]),
                "--bundle-sha256",
                str(source["bundle_sha256"]),
            ]
        )
        try:
            verified = json.loads(verify.stdout)
        except json.JSONDecodeError as error:
            raise RemoteCiError(
                f"invalid remote source verification result: {source_root}"
            ) from error
        if (
            not isinstance(verified, dict)
            or verified.get("source_id") != source["source_id"]
            or verified.get("archive_sha256") != source["archive_sha256"]
            or verified.get("bundle_sha256") != source["bundle_sha256"]
        ):
            raise RemoteCiError(
                f"remote source re-verification returned another identity: {source_root}"
            )
        return {**source, "remote_source_root": str(source_root), "transferred": False}

    exists = ssh.run(["test", "-e", str(source_root)], check=False)
    if exists.returncode == 0:
        raise RemoteCiError(f"partial remote source directory requires inspection: {source_root}")

    inputs = source_root / "inputs"
    repo = source_root / "repo"
    ssh.run(["mkdir", "-p", str(inputs)])
    remote_manifest = inputs / "source-manifest.json"
    remote_archive = inputs / "source.tar.zst"
    remote_bundle = inputs / "base.bundle"
    upload_exact(
        ssh,
        Path(source["manifest_path"]),
        remote_manifest,
        sha256_file(Path(source["manifest_path"])),
    )
    upload_exact(
        ssh,
        Path(source["archive_path"]),
        remote_archive,
        str(source["archive_sha256"]),
    )
    upload_exact(
        ssh,
        Path(source["bundle_path"]),
        remote_bundle,
        str(source["bundle_sha256"]),
    )
    ssh.run(["git", "clone", "--no-checkout", str(remote_bundle), str(repo)])
    ssh.run(["git", "-C", str(repo), "read-tree", "HEAD"])
    extraction = (
        f"zstd -dc {shlex.quote(str(remote_archive))} | "
        f"tar -xf - -C {shlex.quote(str(repo))} "
        "--no-same-owner --same-permissions"
    )
    ssh.run(["/bin/bash", "-lc", extraction])
    remote_script = repo / "scripts" / "remote_ci.py"
    verify = ssh.run(
        [
            "python3",
            str(remote_script),
            "_remote-verify-source",
            "--source-root",
            str(repo),
            "--manifest",
            str(remote_manifest),
            "--ready",
            str(ready_path),
            "--archive-sha256",
            str(source["archive_sha256"]),
            "--bundle-sha256",
            str(source["bundle_sha256"]),
        ]
    )
    verified = json.loads(verify.stdout)
    if verified.get("source_id") != source["source_id"]:
        raise RemoteCiError("remote source verification returned another source id")
    return {**source, "remote_source_root": str(source_root), "transferred": True}


def _new_run_id(source_id: str, profile: str = PROFILE) -> str:
    source_short = source_id.removeprefix("sha256:")[:12]
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{profile}-{source_short}-{secrets.token_hex(3)}"


def _read_remote_json(ssh: VerifiedSsh, path: Path) -> dict[str, Any]:
    result = ssh.run(["cat", str(path)])
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"invalid remote JSON at {path}: {error}") from error
    if not isinstance(value, dict):
        raise RemoteCiError(f"expected remote JSON object at {path}")
    return value


def _remote_run_paths(run_id: str) -> tuple[Path, Path]:
    validate_identifier(run_id, "run-id")
    run_root = REMOTE_ROOT / "runs" / run_id
    return run_root, run_root / "run.json"


def launch_remote_worker(
    ssh: VerifiedSsh, run_root: Path, remote_script: Path
) -> dict[str, Any]:
    child = (
        f"printf '%s\\n' \"$$\" > {shlex.quote(str(run_root / 'worker.pid'))}; "
        f"exec python3 {shlex.quote(str(remote_script))} _remote-worker "
        f"--run-root {shlex.quote(str(run_root))}"
    )
    launch = (
        f"cd {shlex.quote(str(run_root))} && "
        f"nohup setsid --fork /bin/bash -c {shlex.quote(child)} "
        f"> {shlex.quote(str(run_root / 'orchestration.log'))} 2>&1 "
        "< /dev/null"
    )
    ssh.run(["/bin/bash", "-lc", launch])

    probe = """
import json
import os
from pathlib import Path
import sys

run_root = Path(sys.argv[1])
expected_script = sys.argv[2]
pid = int((run_root / 'worker.pid').read_text().strip())
command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\\0')
arguments = [item.decode(errors='replace') for item in command if item]
if os.getsid(pid) != pid:
    raise SystemExit(3)
if expected_script not in arguments or '_remote-worker' not in arguments:
    raise SystemExit(4)
print(json.dumps({'pid': pid, 'session_id': os.getsid(pid), 'arguments': arguments}))
"""
    last_error = "worker pid was not ready"
    for _ in range(20):
        result = ssh.run(
            ["python3", "-c", probe, str(run_root), str(remote_script)],
            check=False,
        )
        if result.returncode == 0:
            try:
                value = json.loads(result.stdout)
            except json.JSONDecodeError as error:
                raise RemoteCiError(
                    f"detached worker probe returned invalid JSON: {error}"
                ) from error
            value["detached"] = True
            return value
        last_error = str(result.stderr).strip() or f"probe rc={result.returncode}"
        time.sleep(0.25)
    raise RemoteCiError(f"detached worker verification failed: {last_error}")


def start_remote_run(
    root: Path,
    profile: str,
    *,
    dataset_content_id: str | None = None,
) -> dict[str, Any]:
    definition = profile_definition(profile)
    dataset_root = Path(definition["dataset_root"])
    if profile == TUM_VI_CONFIRMATORY_PROFILE:
        if dataset_content_id is None:
            raise RemoteCiError(
                "TUM-VI confirmatory start requires --dataset-content-id"
            )
        content_hex = _validate_sha256_content_id(
            dataset_content_id, "dataset content id"
        ).removeprefix("sha256:")
        dataset_root = dataset_root / "by-content" / "sha256" / content_hex
    elif dataset_content_id is not None:
        raise RemoteCiError(
            "--dataset-content-id is only valid for the TUM-VI confirmatory profile"
        )
    for command in (
        "curl",
        "docker",
        "git",
        "scp",
        "ssh",
        "ssh-keyscan",
        "ssh-keygen",
        "tar",
        "unzstd",
        "zstd",
    ):
        if shutil.which(command) is None:
            raise RemoteCiError(f"required local command is missing: {command}")

    image = export_local_image(root, ensure_local_image(root))
    with tempfile.TemporaryDirectory(prefix="phad-remote-ci-source-") as temporary:
        source = create_source_inputs(root, Path(temporary))
        with VerifiedSsh() as ssh:
            doctor = remote_doctor(
                ssh,
                profile,
                dataset_root=dataset_root,
                dataset_content_id=dataset_content_id,
            )
            ssh.run(
                [
                    "mkdir",
                    "-p",
                    str(REMOTE_ROOT / "images"),
                    str(REMOTE_ROOT / "sources"),
                    str(REMOTE_ROOT / "runs"),
                ]
            )
            remote_image = ensure_remote_image(ssh, image)
            remote_source = ensure_remote_source(ssh, source)
            run_id = _new_run_id(str(source["source_id"]), profile)
            run_root, run_json = _remote_run_paths(run_id)
            run_config = {
                "schema_version": SCHEMA_VERSION,
                "run_id": run_id,
                "profile": profile,
                "created_utc": utc_now(),
                "remote_root": str(REMOTE_ROOT),
                "run_root": str(run_root),
                "source_root": str(Path(remote_source["remote_source_root"]) / "repo"),
                "source_id": source["source_id"],
                "source_manifest": source["manifest"],
                "source_archive_sha256": source["archive_sha256"],
                "source_bundle_sha256": source["bundle_sha256"],
                "image": {
                    key: remote_image[key]
                    for key in (
                        "toolchain_id",
                        "dockerfile_sha256",
                        "gtsam_archive_sha256",
                        "tag",
                        "id",
                        "oci_manifest_id",
                        "oci_config_id",
                        "remote_id",
                        "repo_digests",
                        "architecture",
                        "os",
                        "rootfs_layers",
                        "archive_sha256",
                        "versions",
                    )
                },
                "remote": {
                    "target": remote_identity().target,
                    "host_fingerprint": remote_identity().fingerprint,
                    "uid": doctor["uid"],
                    "gid": doctor["gid"],
                    "architecture": doctor["architecture"],
                },
                "dataset_format": definition["dataset_format"],
                "dataset_root": doctor["dataset_root"],
                "sequences": list(definition["sequences"]),
                "max_parallel_tasks": definition["max_parallel_tasks"],
                "build_parallelism": 16,
                "unit_parallelism": 8,
                "record_only": True,
            }
            if "batch_order" in definition:
                run_config["batch_order"] = definition["batch_order"]
            if "dataset_manifest" in definition:
                run_config["dataset_manifest"] = definition["dataset_manifest"]
                run_config["dataset_manifest_id"] = definition[
                    "dataset_manifest_id"
                ]
                run_config["dataset_content_id"] = dataset_content_id
            with tempfile.TemporaryDirectory(prefix="phad-remote-ci-run-") as run_temp:
                local_run_json = Path(run_temp) / "run.json"
                atomic_write_json(local_run_json, run_config)
                ssh.run(["mkdir", str(run_root)])
                upload_exact(
                    ssh,
                    local_run_json,
                    run_json,
                    sha256_file(local_run_json),
                )
            remote_script = Path(run_config["source_root"]) / "scripts" / "remote_ci.py"
            worker = launch_remote_worker(ssh, run_root, remote_script)
            return {
                "run_id": run_id,
                "profile": profile,
                "source_id": source["source_id"],
                "image_id": remote_image["oci_manifest_id"],
                "image_config_id": remote_image["oci_config_id"],
                "remote_image_id": remote_image["remote_id"],
                "image_tag": image["tag"],
                "run_root": str(run_root),
                "worker_pid": worker["pid"],
                "worker_detached": worker["detached"],
            }


def remote_status(ssh: VerifiedSsh, run_id: str) -> dict[str, Any]:
    run_root, run_json = _remote_run_paths(run_id)
    config = _read_remote_json(ssh, run_json)
    remote_script = Path(str(config["source_root"])) / "scripts" / "remote_ci.py"
    result = ssh.run(
        [
            "python3",
            str(remote_script),
            "_remote-status",
            "--run-root",
            str(run_root),
        ]
    )
    try:
        status = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"remote status returned invalid JSON: {error}") from error
    return status


def _status_signature(status: dict[str, Any]) -> tuple[Any, ...]:
    tasks = status.get("tasks") or {}
    return (
        status.get("state"),
        *sorted((name, item.get("state")) for name, item in tasks.items()),
    )


def print_status(status: dict[str, Any]) -> None:
    print(
        f"run={status.get('run_id')} state={status.get('state')} "
        f"updated={status.get('updated_utc', 'n/a')}"
    )
    tasks = status.get("tasks") or {}
    for name in sorted(tasks):
        item = tasks[name]
        rc = item.get("return_code")
        rc_text = "-" if rc is None else str(rc)
        print(f"  {name:<18} {item.get('state', 'unknown'):<8} rc={rc_text}")


def _safe_extract_tar(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    destination_resolved = destination.resolve()
    with tarfile.open(archive, "r") as stream:
        for member in stream.getmembers():
            member_path = (destination / member.name).resolve()
            if destination_resolved not in member_path.parents and member_path != destination_resolved:
                raise RemoteCiError(f"unsafe path in results archive: {member.name}")
            if member.issym() or member.islnk():
                raise RemoteCiError(f"links are not allowed in results archive: {member.name}")
        stream.extractall(destination, filter="data")


def _nested(value: Any, *keys: str) -> Any:
    current = value
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _format_metric(value: Any, digits: int = 6) -> str:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{float(value):.{digits}g}"
    return "—"


def _markdown_cell(value: Any) -> str:
    text = "—" if value in (None, "") else str(value)
    return text.replace("|", "\\|").replace("\n", "<br>")


def _parse_segment_rms(path: Path) -> float | None:
    if not path.exists():
        return None
    match = re.search(
        r"加权 RMS\s*=\s*([0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)\s*m",
        path.read_text(encoding="utf-8", errors="replace"),
    )
    return float(match.group(1)) if match else None


def render_summary(
    run: dict[str, Any],
    status: dict[str, Any],
    remote_result_root: Path,
    *,
    current_source_matches: bool,
) -> str:
    dataset_format = str(run.get("dataset_format", "euroc"))
    profile_label = (
        "TUM-VI room1–6 confirmatory"
        if dataset_format == "tum-vi"
        else "EuRoC 11/11 record-only"
    )
    tasks: dict[str, Any] = {}
    task_root = remote_result_root / "tasks"
    if task_root.exists():
        for status_path in sorted(task_root.glob("*/status.json")):
            tasks[status_path.parent.name] = read_json(status_path)
    if status.get("tasks"):
        for name, item in status["tasks"].items():
            tasks[name] = item

    lines = [
        f"# Remote CI：{run['run_id']}",
        "",
        f"- Profile：`{run['profile']}`（{profile_label}）",
        f"- 状态：`{status.get('state', 'unknown')}`",
        f"- Source ID：`{run['source_id']}`",
        f"- Git：`{run['source_manifest']['git']['head']}` / "
        f"tree `{run['source_manifest']['git']['tree']}` / "
        f"branch `{run['source_manifest']['git']['branch']}` / "
        f"dirty `{str(run['source_manifest']['git']['tracked_dirty']).lower()}`",
        f"- 镜像：`{run['image']['tag']}`",
        f"- OCI manifest ID：`{run['image']['oci_manifest_id']}`",
        f"- OCI config ID：`{run['image']['oci_config_id']}`",
        f"- Docker native ID：local `{run['image']['id']}` / remote "
        f"`{run['image']['remote_id']}`",
        f"- 镜像 archive SHA-256：`{run['image']['archive_sha256']}`",
        f"- 服务器：`{run['remote']['target']}`；数据：`{run['dataset_root']}`（只读）",
        f"- 远端 run：`{run['run_root']}`",
        f"- 当前工作区匹配：`{'yes' if current_source_matches else 'no'}`",
        "",
        "## 任务状态",
        "",
        "| 任务 | 类型 | 状态 | rc | 失败原因 | 远端 stdout / stderr |",
        "|---|---|---:|---:|---|---|",
    ]
    for name in sorted(tasks):
        task = tasks[name]
        lines.append(
            "| "
            + " | ".join(
                _markdown_cell(value)
                for value in (
                    name,
                    task.get("kind"),
                    task.get("state"),
                    task.get("return_code"),
                    task.get("failure_reason"),
                    (
                        f"{task.get('remote_log_path', '—')} ; "
                        f"{task.get('remote_stderr_path', '—')}"
                    ),
                )
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Benchmark 指标",
            "",
            "| 序列 | bench 状态 | ATE m | RPE m | completion | coverage | segments | reanchors | 段内 RMS m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    if dataset_format == "tum-vi":
        benchmark_tasks = [
            f"{role}-{repeat}-{sequence}"
            for role, repeat in run.get("batch_order", [])
            for sequence in run.get("sequences", [])
        ]
    else:
        benchmark_tasks = list(run.get("sequences", SEQUENCES))
    for sequence in benchmark_tasks:
        base = task_root / sequence / "artifacts"
        summary_path = base / "bench" / "summary.json"
        summary = read_json(summary_path) if summary_path.exists() else {}
        weighted = _parse_segment_rms(base / "segment_ate_decomp.txt")
        values = (
            sequence,
            summary.get("status"),
            _format_metric(_nested(summary, "ate", "trans", "rmse")),
            _format_metric(_nested(summary, "rpe", "trans", "rmse")),
            _format_metric(_nested(summary, "trajectory", "completion_rate")),
            _format_metric(_nested(summary, "trajectory", "coverage_rate")),
            _nested(summary, "trajectory", "segments"),
            _nested(summary, "robustness", "reanchors"),
            _format_metric(weighted),
        )
        lines.append("| " + " | ".join(_markdown_cell(v) for v in values) + " |")

    lines.extend(
        [
            "",
            "## 可复现身份",
            "",
            f"- Source archive SHA-256：`{run['source_archive_sha256']}`",
            f"- Git bundle SHA-256：`{run['source_bundle_sha256']}`",
            f"- Toolchain ID：`{run['image']['toolchain_id']}`",
            f"- Dockerfile SHA-256：`{run['image']['dockerfile_sha256']}`",
            f"- GTSAM：`{run['image']['versions']['gtsam_commit']}`；archive SHA-256 "
            f"`{run['image']['gtsam_archive_sha256']}`",
            f"- 并行度：build `{run['build_parallelism']}`；unit `{run['unit_parallelism']}`；sequence `{run['max_parallel_tasks']}`",
            "- 本结果是 CI / record-only 账本，不替代 Slice 1a 的顺序产品门。",
            "",
        ]
    )
    return "\n".join(lines)


def _publish_index(
    directory: Path,
    profile: str,
    summary: str,
    metadata: dict[str, Any],
) -> bool:
    metadata_path = directory / f"{profile}.json"
    if metadata_path.exists():
        existing = read_json(metadata_path)
        if str(existing.get("finished_utc", "")) > str(metadata.get("finished_utc", "")):
            return False
    atomic_write_text(directory / f"{profile}.md", summary)
    atomic_write_json(metadata_path, metadata)
    return True


def publish_summary_indexes(
    root: Path,
    run: dict[str, Any],
    status: dict[str, Any],
    summary: str,
    summary_path: Path,
    *,
    current_source_matches: bool,
) -> dict[str, bool]:
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run["run_id"],
        "profile": run["profile"],
        "source_id": run["source_id"],
        "state": status.get("state"),
        "finished_utc": status.get("finished_utc", ""),
        "summary": str(summary_path),
    }
    latest = _publish_index(
        root / LOCAL_ARTIFACT_ROOT / "latest", run["profile"], summary, metadata
    )
    current = False
    if current_source_matches:
        current = _publish_index(
            root / LOCAL_ARTIFACT_ROOT / "current", run["profile"], summary, metadata
        )
    return {"latest": latest, "current": current}


def fetch_remote_run(root: Path, run_id: str) -> Path:
    run_root, run_json_path = _remote_run_paths(run_id)
    local_run_root = root / LOCAL_ARTIFACT_ROOT / "runs" / run_id
    local_remote_root = local_run_root / "remote"
    local_run_root.mkdir(parents=True, exist_ok=True)

    with VerifiedSsh() as ssh:
        run = _read_remote_json(ssh, run_json_path)
        status = remote_status(ssh, run_id)
        if status.get("state") not in TERMINAL_RUN_STATES:
            raise RemoteCiError(
                f"run is not terminal: {run_id}: {status.get('state')}"
            )
        remote_archive = run_root / "results.tar.zst"
        remote_checksum = run_root / "results.sha256"
        for _ in range(24):
            if ssh.run(["test", "-r", str(remote_archive)], check=False).returncode == 0:
                break
            time.sleep(5)
        else:
            raise RemoteCiError(f"remote results archive is not ready: {remote_archive}")

        local_archive = local_run_root / "results.tar.zst"
        local_checksum = local_run_root / "results.sha256"
        ssh.download(remote_archive, local_archive)
        ssh.download(remote_checksum, local_checksum)
        expected = local_checksum.read_text(encoding="utf-8").split()[0]
        actual = sha256_file(local_archive)
        if actual != expected:
            raise RemoteCiError(
                f"downloaded results checksum mismatch: {actual} != {expected}"
            )

        with tempfile.TemporaryDirectory(prefix="phad-remote-ci-results-") as temporary:
            tar_path = Path(temporary) / "results.tar"
            run_command(
                [
                    "zstd",
                    "-d",
                    "--no-progress",
                    "--force",
                    str(local_archive),
                    "-o",
                    str(tar_path),
                ],
                capture_output=False,
            )
            _safe_extract_tar(tar_path, local_remote_root)
        ssh.download(run_json_path, local_remote_root / "run.json")
        ssh.download(run_root / "run-status.json", local_remote_root / "run-status.json")

    current_manifest, current_source_id = build_source_manifest(root)
    del current_manifest
    current_matches = current_source_id == run["source_id"]
    final_status = read_json(local_remote_root / "run-status.json")
    summary = render_summary(
        run,
        final_status,
        local_remote_root,
        current_source_matches=current_matches,
    )
    summary_path = local_run_root / "summary.md"
    atomic_write_text(summary_path, summary)
    publish_summary_indexes(
        root,
        run,
        final_status,
        summary,
        summary_path,
        current_source_matches=current_matches,
    )
    return summary_path


def _task_directories(task_root: Path) -> None:
    for name in ("home", "ros_home", "build", "cache", "tmp", "logs", "artifacts"):
        (task_root / name).mkdir(parents=True, exist_ok=True)


def _container_base(
    run: dict[str, Any],
    task_name: str,
    *,
    cpus: int,
    memory: str,
    mounts: Iterable[tuple[Path, str, bool]],
) -> list[str]:
    uid = run["remote"]["uid"]
    gid = run["remote"]["gid"]
    container_name = re.sub(
        r"[^A-Za-z0-9_.-]", "-", f"phad-{run['run_id']}-{task_name}"
    )
    command = [
        "docker",
        "run",
        "--pull",
        "never",
        "--platform",
        "linux/amd64",
        "--name",
        container_name,
        "--label",
        f"phad.remote-ci.run-id={run['run_id']}",
        "--label",
        f"phad.remote-ci.task={task_name}",
        "--network",
        "none",
        "--read-only",
        "--cap-drop",
        "ALL",
        "--security-opt",
        "no-new-privileges",
        "--pids-limit",
        "2048",
        "--cpus",
        str(cpus),
        "--memory",
        memory,
        "--user",
        f"{uid}:{gid}",
        "--init",
        "--tmpfs",
        "/run:rw,nosuid,nodev,size=64m",
        "--env",
        "HOME=/task/home",
        "--env",
        "ROS_HOME=/task/ros_home",
        "--env",
        "XDG_CACHE_HOME=/task/cache",
        "--env",
        "TMPDIR=/task/tmp",
        "--env",
        "OMP_NUM_THREADS=1",
        "--env",
        "OPENBLAS_NUM_THREADS=1",
        "--env",
        "MKL_NUM_THREADS=1",
        "--workdir",
        "/src",
    ]
    for source, destination, readonly in mounts:
        specification = f"type=bind,src={source},dst={destination}"
        if readonly:
            specification += ",readonly"
        command.extend(["--mount", specification])
    return command


def _task_image_reference(run: dict[str, Any]) -> str:
    image = run.get("image")
    if not isinstance(image, dict):
        raise RemoteCiError("task image identity is missing")
    remote_id = image.get("remote_id")
    manifest_id = image.get("oci_manifest_id")
    config_id = image.get("oci_config_id")
    if not all(
        isinstance(value, str) for value in (remote_id, manifest_id, config_id)
    ):
        raise RemoteCiError("task immutable image identity is missing")
    _validate_sha256_content_id(remote_id, "task remote image ID")
    _validate_sha256_content_id(manifest_id, "task OCI manifest ID")
    _validate_sha256_content_id(config_id, "task OCI config ID")
    if remote_id not in {manifest_id, config_id}:
        raise RemoteCiError("task remote image ID differs from its OCI identity")
    return remote_id


def _stderr_failure(path: Path) -> str:
    if not path.exists():
        return "task exited without stderr"
    lines = [line.strip() for line in path.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines() if line.strip()]
    return lines[-1][:500] if lines else "task returned a non-zero status"


_FATAL_META_KEYS = {
    "schema_version", "sequence", "sequence_root", "created_utc", "code",
    "config", "config_label", "config_hash", "config_canonical_text",
    "paths", "warnings",
}
_FATAL_SUMMARY_KEYS = {
    "schema_version", "status", "sequence", "code", "config_label",
    "config_hash", "trajectory", "ate", "rpe", "robustness", "timing",
    "sync", "warnings",
}
_FATAL_TRAJECTORY_KEYS = {
    "image_frames", "poses_written", "ok", "rejected", "failed",
    "completion_rate", "coverage_rate", "segments", "total_keyframes",
    "total_track_only_frames",
}
_FATAL_ROBUSTNESS_KEYS = {
    "rejected", "failed", "low_connectivity", "cheirality", "reanchors",
    "pnp_successes", "pnp_fallbacks", "outliers_culled",
    "outliers_culled_unique", "outlier_reopts", "drops_skipped",
    "deferred_drops", "deferred_drop_ids", "evictable_marked",
    "tracks_evicted", "zombie_age_drops", "zombie_age_drop_ids",
}
_FATAL_SYNC_KEYS = {
    "pushed_left", "pushed_right", "emitted_stereo", "dropped_left",
    "dropped_right", "dropped_left_overflow", "dropped_right_overflow",
    "max_left_queue", "max_right_queue",
}
_FATAL_DIAG_HEADER = (
    "timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,"
    "window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,"
    "num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,"
    "pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,"
    "is_keyframe,num_disparity,unsupported_span_ns,"
    "num_retained_observations,num_seeded_landmarks,"
    "num_current_visual_factors,num_mapped_observations,"
    "num_current_mono_visual_factors"
)
_FATAL_DIAG_STATUSES = {
    "ok", "initializing", "discontinuity", "visual_outage",
    "invalid_input", "rejected", "failed",
}
_FATAL_DIAG_INT_COLUMNS = {
    0, 2, 3, 4, 6, 7, 10, 11, 13, 15, 16, 19, 20, 21, 22, 23, 24, 25,
}
_FATAL_DIAG_BOOL_COLUMNS = {5, 14, 18}
_FATAL_DIAG_FLOAT_COLUMNS = {8, 9, 12, 17}
_FATAL_PNP_RESULTS = {
    "not_attempted", "ransac_failed", "insufficient_inliers",
    "proposal_pose_nonfinite", "proposal_rms_invalid_correspondence",
    "proposal_rms_cheirality", "proposal_rms_nonfinite",
    "proposal_rms_rejected", "accepted", "accepted_after_guess_retry",
}
_FATAL_RETRY_OUTCOMES = {
    "not_attempted", "solve_failed", "proposal_pose_nonfinite",
    "proposal_rms_invalid_correspondence", "proposal_rms_cheirality",
    "proposal_rms_nonfinite", "proposal_rms_rejected",
    "would_pass_existing_pnp_gates",
}
_FATAL_PROBE_REQUIRED = {
    "i", "ts_ns", "pnp_attempted", "pnp_result", "pnp_correspondences",
    "pnp_inliers", "pnp_guess_retry_outcome", "shared", "num_obs",
}
_FATAL_PROBE_UINT = {
    "i", "pnp_correspondences", "pnp_inliers",
    "pnp_proposal_inlier_count", "pnp_proposal_cheiral_inlier_count",
    "pnp_proposal_min_inlier_landmark_id",
    "pnp_guess_retry_cheiral_inlier_count", "zombie_track_n",
    "rejected_block_n", "new_lm", "shared", "num_obs", "lm_iterations",
    "res_max_id",
}
_FATAL_PROBE_FLOAT = {
    "pnp_proposal_rms_px", "pnp_guess_rms_px",
    "pnp_proposal_min_inlier_depth_m",
    "pnp_proposal_guess_translation_delta_m",
    "pnp_proposal_guess_rotation_delta_rad",
    "pnp_guess_retry_proposal_rms_px", "pnp_guess_retry_guess_rms_px",
    "pnp_guess_retry_min_inlier_depth_m",
    "pnp_guess_retry_translation_delta_m",
    "pnp_guess_retry_rotation_delta_rad",
    "current_pose_translation_delta_m", "current_pose_rotation_delta_rad",
    "current_velocity_delta_mps", "current_acc_bias_delta_mps2",
    "current_gyr_bias_delta_radps", "primary_graph_error_seed",
    "primary_graph_error_optimized", "primary_imu_factor_error_seed",
    "primary_imu_factor_error_optimized", "shift_m", "res_mean_px",
    "res_max_px",
}
_FATAL_PROBE_ALLOWED = (
    _FATAL_PROBE_REQUIRED | _FATAL_PROBE_UINT | _FATAL_PROBE_FLOAT |
    {"culled_ids", "shift_top", "drops_skipped_this_frame"}
)


def _fatal_exact_keys(value: Any, expected: set[str], label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != expected:
        raise RemoteCiError(f"typed product-failure {label} schema mismatch")
    return value


def _fatal_nonnegative_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise RemoteCiError(f"typed product-failure {label} is invalid")
    return value


def _fatal_nonnegative_number(value: Any, label: str, *, nullable: bool = False) -> float | None:
    if value is None and nullable:
        return None
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise RemoteCiError(f"typed product-failure {label} is invalid")
    return float(value)


def _fatal_config_text(config: dict[str, Any]) -> str:
    lines: list[str] = []
    for key, value in sorted(config.items()):
        if not isinstance(key, str) or not key:
            raise RemoteCiError("typed product-failure config key is invalid")
        if isinstance(value, bool):
            text = "true" if value else "false"
        elif isinstance(value, int):
            text = str(value)
        elif isinstance(value, float) and math.isfinite(value):
            text = format(value, ".17g")
        elif isinstance(value, str):
            text = value
        else:
            raise RemoteCiError(
                f"typed product-failure config value is invalid: {key}"
            )
        lines.append(f"{key}={text}\n")
    return "".join(lines)


def _fatal_config_hash(text: str) -> str:
    value = 14_695_981_039_346_656_037
    for byte in text.encode("utf-8"):
        value ^= byte
        value = (value * 1_099_511_628_211) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"[:8]


def _read_fatal_diag(path: Path) -> dict[str, Any]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise RemoteCiError(f"cannot read typed product-failure diag: {error}") from error
    if not lines or lines[0] != _FATAL_DIAG_HEADER:
        raise RemoteCiError("typed product-failure diag header mismatch")
    rows = list(csv.reader(lines[1:]))
    if not rows:
        raise RemoteCiError("typed product-failure diag has no rows")
    timestamps: list[int] = []
    statuses: list[str] = []
    accepted_keyframes: list[int] = []
    segments = 0
    last_ok_segment: int | None = None
    for line_number, row in enumerate(rows, 2):
        if len(row) != 26:
            raise RemoteCiError(
                f"typed product-failure diag row {line_number} column mismatch"
            )
        for index in _FATAL_DIAG_INT_COLUMNS:
            try:
                value = int(row[index])
            except ValueError as error:
                raise RemoteCiError(
                    f"typed product-failure diag row {line_number} integer invalid"
                ) from error
            if value < 0:
                raise RemoteCiError(
                    f"typed product-failure diag row {line_number} integer negative"
                )
        for index in _FATAL_DIAG_BOOL_COLUMNS:
            if row[index] not in {"0", "1"}:
                raise RemoteCiError(
                    f"typed product-failure diag row {line_number} boolean invalid"
                )
        for index in _FATAL_DIAG_FLOAT_COLUMNS:
            try:
                value = float(row[index])
            except ValueError as error:
                raise RemoteCiError(
                    f"typed product-failure diag row {line_number} float invalid"
                ) from error
            if not math.isfinite(value) or value < 0.0:
                raise RemoteCiError(
                    f"typed product-failure diag row {line_number} float invalid"
                )
        timestamp = int(row[0])
        if timestamps and timestamp <= timestamps[-1]:
            raise RemoteCiError("typed product-failure diag timestamps are not increasing")
        if row[1] not in _FATAL_DIAG_STATUSES:
            raise RemoteCiError("typed product-failure diag status is invalid")
        timestamps.append(timestamp)
        statuses.append(row[1])
        if row[1] == "ok":
            segment = int(row[13])
            if last_ok_segment is None:
                segments = 1
            elif segment != last_ok_segment:
                if segment <= last_ok_segment:
                    raise RemoteCiError(
                        "typed product-failure diag segment is not increasing"
                    )
                segments += 1
            last_ok_segment = segment
            if row[18] == "1":
                accepted_keyframes.append(timestamp)
    fatal_indices = [
        index for index, status in enumerate(statuses)
        if status in {"failed", "invalid_input"}
    ]
    if fatal_indices != [len(rows) - 1]:
        raise RemoteCiError("typed product-failure terminal diag status is invalid")
    return {
        "rows": rows,
        "timestamps": timestamps,
        "statuses": statuses,
        "accepted_keyframes": accepted_keyframes,
        "segments": segments,
    }


def _exact_timestamp_ns(text: str) -> int:
    negative = text.startswith("-")
    magnitude = text[1:] if negative else text
    whole, dot, fraction = magnitude.partition(".")
    if not whole.isdigit() or (dot and (not fraction or not fraction.isdigit())):
        raise RemoteCiError(f"typed product-failure TUM timestamp is invalid: {text!r}")
    if len(fraction) > 9:
        raise RemoteCiError("typed product-failure TUM timestamp is sub-nanosecond")
    value = int(whole) * 1_000_000_000
    if fraction:
        value += int(fraction.ljust(9, "0"))
    return -value if negative else value


def _read_fatal_tum(path: Path) -> list[tuple[int, str]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise RemoteCiError(f"cannot read typed product-failure TUM: {error}") from error
    poses: list[tuple[int, str]] = []
    for line_number, raw in enumerate(lines, 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise RemoteCiError(
                f"typed product-failure TUM row {line_number} field mismatch"
            )
        try:
            timestamp = _exact_timestamp_ns(fields[0])
            values = [float(field) for field in fields[1:]]
        except ValueError as error:
            raise RemoteCiError(
                f"typed product-failure TUM row {line_number} number invalid"
            ) from error
        if not all(math.isfinite(value) for value in values):
            raise RemoteCiError(
                f"typed product-failure TUM row {line_number} number non-finite"
            )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:7]))
        if abs(quaternion_norm - 1.0) > 1e-6:
            raise RemoteCiError(
                f"typed product-failure TUM row {line_number} quaternion invalid"
            )
        if poses and timestamp <= poses[-1][0]:
            raise RemoteCiError("typed product-failure TUM timestamps are not increasing")
        poses.append((timestamp, line))
    return poses


def _read_fatal_probe(path: Path, role: str) -> list[int]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise RemoteCiError(f"cannot read typed product-failure probe: {error}") from error
    timestamps: list[int] = []
    for index, line in enumerate(lines):
        try:
            row = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(
                RemoteCiError(f"typed product-failure probe non-finite: {value}")
            ))
        except json.JSONDecodeError as error:
            raise RemoteCiError("typed product-failure probe JSON is invalid") from error
        if not isinstance(row, dict) or set(row) - _FATAL_PROBE_ALLOWED:
            raise RemoteCiError("typed product-failure probe schema is invalid")
        if _FATAL_PROBE_REQUIRED - set(row):
            raise RemoteCiError("typed product-failure probe required keys are missing")
        if row.get("i") != index or isinstance(row.get("i"), bool):
            raise RemoteCiError("typed product-failure probe index is invalid")
        timestamp = row.get("ts_ns")
        if not isinstance(timestamp, int) or isinstance(timestamp, bool):
            raise RemoteCiError("typed product-failure probe timestamp is invalid")
        if timestamps and timestamp <= timestamps[-1]:
            raise RemoteCiError("typed product-failure probe timestamps are not increasing")
        timestamps.append(timestamp)
        result = row.get("pnp_result")
        outcome = row.get("pnp_guess_retry_outcome")
        if result not in _FATAL_PNP_RESULTS or outcome not in _FATAL_RETRY_OUTCOMES:
            raise RemoteCiError("typed product-failure probe PnP token is invalid")
        if not isinstance(row.get("pnp_attempted"), bool):
            raise RemoteCiError("typed product-failure probe pnp_attempted is invalid")
        for key in _FATAL_PROBE_UINT:
            if key in row:
                _fatal_nonnegative_int(row[key], f"probe.{key}")
        for key in _FATAL_PROBE_FLOAT:
            if key in row and (
                not isinstance(row[key], (int, float))
                or isinstance(row[key], bool)
                or not math.isfinite(float(row[key]))
            ):
                raise RemoteCiError(f"typed product-failure probe.{key} is invalid")
        culled = row.get("culled_ids")
        if culled is not None and (
            not isinstance(culled, list)
            or any(
                not isinstance(value, int) or isinstance(value, bool) or value < 0
                for value in culled
            )
        ):
            raise RemoteCiError("typed product-failure probe culled_ids is invalid")
        shift_top = row.get("shift_top")
        if shift_top is not None:
            if not isinstance(shift_top, list):
                raise RemoteCiError("typed product-failure probe shift_top is invalid")
            for entry in shift_top:
                if (
                    not isinstance(entry, dict)
                    or set(entry) != {"key", "dt_m"}
                    or not isinstance(entry["key"], int)
                    or isinstance(entry["key"], bool)
                    or entry["key"] < 0
                    or not isinstance(entry["dt_m"], (int, float))
                    or isinstance(entry["dt_m"], bool)
                    or not math.isfinite(float(entry["dt_m"]))
                ):
                    raise RemoteCiError(
                        "typed product-failure probe shift_top entry is invalid"
                    )
        if "drops_skipped_this_frame" in row and not isinstance(
            row["drops_skipped_this_frame"], bool
        ):
            raise RemoteCiError(
                "typed product-failure probe drops_skipped flag is invalid"
            )
        attempted = outcome != "not_attempted"
        eligible = result in {"proposal_rms_cheirality", "accepted_after_guess_retry"}
        if row["pnp_attempted"] != (result != "not_attempted"):
            raise RemoteCiError("typed product-failure probe PnP invariant failed")
        if attempted and (not row["pnp_attempted"] or not eligible):
            raise RemoteCiError("typed product-failure probe retry invariant failed")
        if result == "accepted_after_guess_retry" and (
            role != "on" or outcome != "would_pass_existing_pnp_gates"
        ):
            raise RemoteCiError("typed product-failure probe role invariant failed")
    return timestamps


def _validate_typed_product_failure_envelope(
    bench_root: Path,
    benchmark: dict[str, Any],
    run: dict[str, Any],
    task_name: str,
) -> None:
    match = re.fullmatch(r"(off|on)-([123])-(room[1-6])", task_name)
    if match is None:
        raise RemoteCiError(
            f"typed product-failure task identity is invalid: {task_name!r}"
        )
    role, repeat_text, sequence = match.groups()
    persisted_summary = read_json(bench_root / "summary.json")
    if persisted_summary != benchmark:
        raise RemoteCiError(
            "typed product-failure summary.json changed during validation"
        )

    required_artifacts = (
        "meta.json", "summary.json", "diag.csv", "est.tum", "kf.tum",
        "probe-b.jsonl",
    )
    missing = [name for name in required_artifacts if not (bench_root / name).is_file()]
    if missing:
        raise RemoteCiError(
            "typed product-failure artifacts are missing: " + ", ".join(missing)
        )

    meta = _fatal_exact_keys(
        read_json(bench_root / "meta.json"), _FATAL_META_KEYS, "meta"
    )
    summary = _fatal_exact_keys(benchmark, _FATAL_SUMMARY_KEYS, "summary")
    if meta.get("schema_version") != 2 or summary.get("schema_version") != 2:
        raise RemoteCiError("typed product-failure schema_version is invalid")
    created_utc = meta.get("created_utc")
    if not isinstance(created_utc, str):
        raise RemoteCiError("typed product-failure created_utc is invalid")
    try:
        created = dt.datetime.fromisoformat(created_utc.replace("Z", "+00:00"))
    except ValueError as error:
        raise RemoteCiError(
            "typed product-failure created_utc is invalid"
        ) from error
    if created.utcoffset() != dt.timedelta(0):
        raise RemoteCiError("typed product-failure created_utc is not UTC")

    source_manifest = run.get("source_manifest")
    git = source_manifest.get("git") if isinstance(source_manifest, dict) else None
    if not isinstance(git, dict):
        raise RemoteCiError("typed product-failure run source identity is missing")
    head = git.get("head")
    branch = git.get("branch")
    dirty = git.get("tracked_dirty")
    if (
        not isinstance(head, str)
        or re.fullmatch(r"[0-9a-f]{40}", head) is None
        or not isinstance(branch, str)
        or not isinstance(dirty, bool)
    ):
        raise RemoteCiError("typed product-failure run Git identity is invalid")
    expected_meta_code = {
        "git_commit": head,
        "git_commit_short": head[:7],
        "git_dirty": dirty,
        "git_branch": None if branch == "HEAD" else branch,
    }
    expected_summary_code = {
        "git_commit_short": head[:7],
        "git_dirty": dirty,
    }
    if meta.get("code") != expected_meta_code:
        raise RemoteCiError("typed product-failure meta/run code identity mismatch")
    if summary.get("code") != expected_summary_code:
        raise RemoteCiError("typed product-failure summary/run code identity mismatch")
    if meta.get("sequence") != sequence or summary.get("sequence") != sequence:
        raise RemoteCiError("typed product-failure sequence identity mismatch")
    if meta.get("sequence_root") != f"/datasets/tum-vi/{sequence}":
        raise RemoteCiError("typed product-failure dataset path identity mismatch")
    paths = _fatal_exact_keys(
        meta.get("paths"), {"bench_root", "output_dir", "layout_template"},
        "meta.paths",
    )
    if paths != {
        "bench_root": "",
        "output_dir": "/task/artifacts/bench",
        "layout_template": "override",
    }:
        raise RemoteCiError("typed product-failure output path identity mismatch")
    if meta.get("config_label") != "default" or summary.get(
        "config_label"
    ) != "default":
        raise RemoteCiError("typed product-failure config label mismatch")
    config = meta.get("config")
    if not isinstance(config, dict):
        raise RemoteCiError("typed product-failure config schema mismatch")
    canonical_config = _fatal_config_text(config)
    if meta.get("config_canonical_text") != canonical_config:
        raise RemoteCiError("typed product-failure canonical config mismatch")
    config_hash = _fatal_config_hash(canonical_config)
    if meta.get("config_hash") != config_hash or summary.get(
        "config_hash"
    ) != config_hash:
        raise RemoteCiError("typed product-failure config hash mismatch")
    retry_enabled = (
        config.get("estimator.enable_pnp_cheirality_retry", False)
    )
    if retry_enabled is not (role == "on"):
        raise RemoteCiError("typed product-failure role/config identity mismatch")

    for owner, value in (("meta", meta), ("summary", summary)):
        warnings = value.get("warnings")
        if not isinstance(warnings, list) or any(
            not isinstance(item, str) for item in warnings
        ):
            raise RemoteCiError(
                f"typed product-failure {owner} warnings schema mismatch"
            )
    if summary["warnings"][: len(meta["warnings"])] != meta["warnings"]:
        raise RemoteCiError("typed product-failure warning provenance mismatch")
    if summary.get("status") != "failed" or not summary["warnings"]:
        raise RemoteCiError("typed product-failure summary status is invalid")
    if summary.get("ate") is not None or summary.get("rpe") is not None:
        raise RemoteCiError("typed product-failure metrics must be null")

    trajectory = _fatal_exact_keys(
        summary.get("trajectory"), _FATAL_TRAJECTORY_KEYS,
        "summary.trajectory",
    )
    for key in _FATAL_TRAJECTORY_KEYS - {"completion_rate", "coverage_rate"}:
        _fatal_nonnegative_int(trajectory.get(key), f"trajectory.{key}")
    robustness = _fatal_exact_keys(
        summary.get("robustness"), _FATAL_ROBUSTNESS_KEYS,
        "summary.robustness",
    )
    for key in _FATAL_ROBUSTNESS_KEYS:
        _fatal_nonnegative_int(robustness.get(key), f"robustness.{key}")
    timing = _fatal_exact_keys(
        summary.get("timing"),
        {"wall_s", "rtf", "rectify", "frontend", "estimator", "total"},
        "summary.timing",
    )
    _fatal_nonnegative_number(timing.get("wall_s"), "timing.wall_s")
    _fatal_nonnegative_number(timing.get("rtf"), "timing.rtf", nullable=True)
    for stage in ("rectify", "frontend", "estimator", "total"):
        stage_values = _fatal_exact_keys(
            timing.get(stage), {"mean", "p95", "max"}, f"timing.{stage}"
        )
        for key in ("mean", "p95", "max"):
            _fatal_nonnegative_number(
                stage_values.get(key), f"timing.{stage}.{key}"
            )
    sync = _fatal_exact_keys(summary.get("sync"), _FATAL_SYNC_KEYS, "summary.sync")
    for key in _FATAL_SYNC_KEYS:
        _fatal_nonnegative_int(sync.get(key), f"sync.{key}")

    diag = _read_fatal_diag(bench_root / "diag.csv")
    probe_timestamps = _read_fatal_probe(bench_root / "probe-b.jsonl", role)
    if probe_timestamps != diag["timestamps"]:
        raise RemoteCiError("typed product-failure probe/diag identity mismatch")
    estimate = _read_fatal_tum(bench_root / "est.tum")
    keyframes = _read_fatal_tum(bench_root / "kf.tum")
    ok_timestamps = [
        timestamp for timestamp, status in zip(
            diag["timestamps"], diag["statuses"], strict=True
        ) if status == "ok"
    ]
    if [timestamp for timestamp, _ in estimate] != ok_timestamps:
        raise RemoteCiError("typed product-failure est/diag identity mismatch")
    if [timestamp for timestamp, _ in keyframes] != diag["accepted_keyframes"]:
        raise RemoteCiError("typed product-failure kf/diag identity mismatch")
    estimate_lines = dict(estimate)
    if any(estimate_lines[timestamp] != line for timestamp, line in keyframes):
        raise RemoteCiError("typed product-failure kf/est pose mismatch")

    rows = diag["rows"]
    statuses = diag["statuses"]
    all_keyframes = sum(row[18] == "1" for row in rows)
    expected_trajectory = {
        "image_frames": len(rows),
        "poses_written": len(estimate),
        "ok": statuses.count("ok"),
        "rejected": statuses.count("rejected"),
        "failed": 1,
        "segments": diag["segments"],
        "total_keyframes": all_keyframes,
        "total_track_only_frames": len(rows) - all_keyframes,
    }
    for key, expected in expected_trajectory.items():
        if trajectory.get(key) != expected:
            raise RemoteCiError(
                f"typed product-failure trajectory.{key} mismatch"
            )
    diag_span = diag["timestamps"][-1] - diag["timestamps"][0]
    estimate_span = estimate[-1][0] - estimate[0][0] if estimate else 0
    expected_rates = {
        "completion_rate": statuses.count("ok") / len(rows),
        "coverage_rate": estimate_span / diag_span if diag_span > 0 else 0.0,
    }
    for key, expected in expected_rates.items():
        actual = _fatal_nonnegative_number(trajectory.get(key), f"trajectory.{key}")
        if actual is None or actual > 1.0 or abs(actual - expected) > 1e-10:
            raise RemoteCiError(f"typed product-failure trajectory.{key} mismatch")

    expected_robustness = {
        "failed": 1,
        "rejected": statuses.count("rejected"),
        "low_connectivity": sum(int(row[5]) for row in rows),
        "cheirality": sum(int(row[10]) for row in rows),
        "pnp_successes": sum(int(row[14]) for row in rows),
        "outliers_culled": sum(int(row[16]) for row in rows),
        "reanchors": max(diag["segments"] - 1, 0),
    }
    for key, expected in expected_robustness.items():
        if robustness.get(key) != expected:
            raise RemoteCiError(
                f"typed product-failure robustness.{key} mismatch"
            )

    terminal_status = statuses[-1]
    terminal_prefix = f"VIO estimator {terminal_status}: "
    terminal_warnings = [
        warning for warning in summary["warnings"]
        if warning.startswith(terminal_prefix)
    ]
    if len(terminal_warnings) != 1 or terminal_warnings[0] == terminal_prefix:
        raise RemoteCiError("typed product-failure warning/status mismatch")


def classify_benchmark_task(
    return_code: int,
    benchmark: dict[str, Any],
    *,
    require_zero_failed_rejected: bool = False,
    complete_product_failure_envelope: bool = False,
) -> tuple[str, str | None]:
    benchmark_status = benchmark.get("status")
    warnings_value = benchmark.get("warnings")
    if warnings_value is None:
        warnings: list[str] = []
    elif not isinstance(warnings_value, list) or any(
        not isinstance(item, str) for item in warnings_value
    ):
        return "failed", "benchmark warnings are invalid"
    else:
        warnings = warnings_value
    reason = str(warnings[-1]) if warnings else None
    if require_zero_failed_rejected:
        robustness = benchmark.get("robustness")
        if not isinstance(robustness, dict):
            return "failed", "benchmark robustness counters are invalid"
        failed = robustness.get("failed")
        rejected = robustness.get("rejected")
        if any(
            not isinstance(value, int) or isinstance(value, bool) or value < 0
            for value in (failed, rejected)
        ):
            return "failed", "benchmark robustness counters are invalid"
        if return_code != 0:
            if (
                benchmark_status == "failed"
                and failed != 0
                and complete_product_failure_envelope
            ):
                return (
                    "warning",
                    f"benchmark robustness.failed={failed}, "
                    f"robustness.rejected={rejected}",
                )
            return "failed", reason
        if failed != 0 or rejected != 0:
            return (
                "warning",
                f"benchmark robustness.failed={failed}, "
                f"robustness.rejected={rejected}",
            )
    elif return_code != 0:
        return "failed", reason
    if benchmark_status == "completed":
        return "passed", None
    if benchmark_status in {
        "completed_with_failures",
        "completed_with_warnings",
    }:
        return "warning", reason
    if benchmark_status in {"eval_failed", "failed"}:
        return "failed", reason or f"benchmark status is {benchmark_status}"
    return "failed", f"unknown benchmark status: {benchmark_status!r}"


def _run_remote_task_impl(
    run: dict[str, Any],
    name: str,
    kind: str,
    script: str,
    *,
    cpus: int,
    memory: str,
    extra_mounts: Sequence[tuple[Path, str, bool]] = (),
) -> dict[str, Any]:
    validate_identifier(name, "task name")
    run_root = Path(run["run_root"])
    task_root = run_root / "tasks" / name
    _task_directories(task_root)
    stdout_path = task_root / "logs" / "stdout.log"
    stderr_path = task_root / "logs" / "stderr.log"
    status_path = task_root / "status.json"
    cid_path = task_root / "container.cid"
    mounts = [
        (Path(run["source_root"]), "/src", True),
        (task_root, "/task", False),
        *extra_mounts,
    ]
    docker = _container_base(
        run, name, cpus=cpus, memory=memory, mounts=mounts
    )
    docker.extend(
        [
            "--cidfile",
            str(cid_path),
            _task_image_reference(run),
            "/bin/bash",
            "-lc",
            script,
        ]
    )
    status: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "kind": kind,
        "state": "running",
        "return_code": None,
        "container_id": None,
        "started_utc": utc_now(),
        "finished_utc": None,
        "command": script,
        "remote_log_path": str(stdout_path),
        "remote_stderr_path": str(stderr_path),
        "failure_reason": None,
    }
    atomic_write_json(status_path, status)
    try:
        with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
            "w", encoding="utf-8"
        ) as stderr:
            process = subprocess.Popen(docker, stdout=stdout, stderr=stderr, text=True)
            for _ in range(50):
                if cid_path.exists():
                    status["container_id"] = cid_path.read_text(
                        encoding="utf-8"
                    ).strip()
                    atomic_write_json(status_path, status)
                    break
                if process.poll() is not None:
                    break
                time.sleep(0.1)
            return_code = process.wait()
    except (OSError, subprocess.SubprocessError) as error:
        return_code = 1
        status["failure_reason"] = str(error)

    status["return_code"] = return_code
    status["finished_utc"] = utc_now()
    if return_code == 0:
        status["state"] = "passed"
    else:
        status["state"] = "failed"
        status["failure_reason"] = status["failure_reason"] or _stderr_failure(
            stderr_path
        )

    if kind in {"euroc", "tum-vi"}:
        benchmark_path = task_root / "artifacts" / "bench" / "summary.json"
        if not benchmark_path.exists():
            status["state"] = "failed"
            status["failure_reason"] = (
                status["failure_reason"] or "benchmark summary.json is missing"
            )
        else:
            benchmark = read_json(benchmark_path)
            benchmark_status = benchmark.get("status")
            status["benchmark_status"] = benchmark_status
            complete_product_failure_envelope = False
            envelope_error: str | None = None
            if (
                kind == "tum-vi"
                and return_code != 0
                and benchmark_status == "failed"
            ):
                try:
                    _validate_typed_product_failure_envelope(
                        benchmark_path.parent, benchmark, run, name
                    )
                except RemoteCiError as error:
                    envelope_error = str(error)
                else:
                    complete_product_failure_envelope = True
            task_state, benchmark_reason = classify_benchmark_task(
                return_code,
                benchmark,
                require_zero_failed_rejected=(kind == "tum-vi"),
                complete_product_failure_envelope=(
                    complete_product_failure_envelope
                ),
            )
            if envelope_error is not None:
                task_state = "failed"
                benchmark_reason = envelope_error
            status["state"] = task_state
            if benchmark_reason:
                status["failure_reason"] = benchmark_reason
    atomic_write_json(status_path, status)
    return status


def _run_remote_task(
    run: dict[str, Any],
    name: str,
    kind: str,
    script: str,
    *,
    cpus: int,
    memory: str,
    extra_mounts: Sequence[tuple[Path, str, bool]] = (),
) -> dict[str, Any]:
    """运行单任务，并把编排层异常也收口为该任务的终态证据。"""

    try:
        return _run_remote_task_impl(
            run,
            name,
            kind,
            script,
            cpus=cpus,
            memory=memory,
            extra_mounts=extra_mounts,
        )
    except Exception as error:
        validate_identifier(name, "task name")
        task_root = Path(run["run_root"]) / "tasks" / name
        _task_directories(task_root)
        status_path = task_root / "status.json"
        stdout_path = task_root / "logs" / "stdout.log"
        stderr_path = task_root / "logs" / "stderr.log"
        error_path = task_root / "logs" / "worker-error.log"
        atomic_write_text(error_path, traceback.format_exc())
        with stderr_path.open("a", encoding="utf-8") as stream:
            stream.write(
                f"remote task worker failed: {type(error).__name__}: {error}\n"
            )
        existing: dict[str, Any] = {}
        with contextlib.suppress(RemoteCiError):
            existing = read_json(status_path)
        failed = {
            **existing,
            "schema_version": SCHEMA_VERSION,
            "name": name,
            "kind": kind,
            "state": "failed",
            "return_code": 1,
            "container_id": existing.get("container_id"),
            "started_utc": existing.get("started_utc") or utc_now(),
            "finished_utc": utc_now(),
            "command": script,
            "remote_log_path": str(stdout_path),
            "remote_stderr_path": str(stderr_path),
            "remote_worker_error_path": str(error_path),
            "failure_reason": f"{type(error).__name__}: {error}",
        }
        atomic_write_json(status_path, failed)
        return failed


def _write_blocked_task(run: dict[str, Any], name: str, kind: str, reason: str) -> None:
    task_root = Path(run["run_root"]) / "tasks" / name
    _task_directories(task_root)
    stderr_path = task_root / "logs" / "stderr.log"
    atomic_write_text(stderr_path, reason + "\n")
    atomic_write_json(
        task_root / "status.json",
        {
            "schema_version": SCHEMA_VERSION,
            "name": name,
            "kind": kind,
            "state": "blocked",
            "return_code": None,
            "container_id": None,
            "started_utc": None,
            "finished_utc": utc_now(),
            "command": None,
            "remote_log_path": str(task_root / "logs" / "stdout.log"),
            "remote_stderr_path": str(stderr_path),
            "failure_reason": reason,
        },
    )


def _build_script(run: dict[str, Any]) -> str:
    parallelism = int(run["build_parallelism"])
    return "\n".join(
        (
            "set -euo pipefail",
            "umask 022",
            "cmake -S /src -B /task/build -G Ninja "
            "-DCMAKE_BUILD_TYPE=Release -DPHAD_BUILD_TESTS=ON "
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            f"cmake --build /task/build --parallel {parallelism}",
        )
    )


def _unit_script(run: dict[str, Any]) -> str:
    parallelism = int(run["unit_parallelism"])
    commands = [
        "set -euo pipefail",
        "umask 022",
        "cmake -E copy_directory /shared-build /task/build",
        f"ctest --test-dir /task/build --output-on-failure -L unit --parallel {parallelism}",
    ]
    if run.get("profile") == TUM_VI_CONFIRMATORY_PROFILE:
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


def _sequence_script(sequence: str) -> str:
    quoted = shlex.quote(sequence)
    return f"""set +e
umask 022
/shared-build/phad_vo_bench /datasets/euroc/{quoted} \\
  --out /task/artifacts/bench \\
  --sequence-name {quoted} \\
  --repo /src \\
  --estimator-enable-moving-bootstrap
bench_rc=$?
decomp_rc=0
if [ -f /task/artifacts/bench/summary.json ] && \\
   [ -f /task/artifacts/bench/diag.csv ] && \\
   [ -f /task/artifacts/bench/est.tum ]; then
  /opt/phad-venv/bin/python /src/scripts/segment_ate_decomp.py \\
    /task/artifacts/bench /datasets/euroc/{quoted} \\
    --phad_traj_eval /shared-build/phad_traj_eval \\
    > /task/artifacts/segment_ate_decomp.txt 2>&1
  decomp_rc=$?
else
  echo "segment ATE inputs are missing" > /task/artifacts/segment_ate_decomp.txt
  decomp_rc=1
fi
/opt/phad-venv/bin/python -c \\
  'import json, pathlib, sys; pathlib.Path("/task/artifacts/stages.json").write_text(json.dumps({{"bench_rc": int(sys.argv[1]), "decomp_rc": int(sys.argv[2])}}, indent=2) + "\\n")' \\
  "$bench_rc" "$decomp_rc"
if [ "$bench_rc" -ne 0 ]; then exit "$bench_rc"; fi
exit "$decomp_rc"
"""


def _tum_vi_sequence_script(sequence: str, role: str) -> str:
    if sequence not in TUM_VI_ROOM_SEQUENCES:
        raise RemoteCiError(f"unsupported TUM-VI room sequence: {sequence}")
    if role not in {"off", "on"}:
        raise RemoteCiError(f"unsupported TUM-VI confirmatory role: {role}")
    quoted = shlex.quote(sequence)
    activation = (
        "  --estimator-enable-pnp-cheirality-retry \\\n" if role == "on" else ""
    )
    return f"""set -euo pipefail
umask 022
/shared-build/phad_vo_bench /datasets/tum-vi/{quoted} \\
  --dataset-format tum-vi \\
  --gt-tum-vi /datasets/tum-vi/{quoted} \\
  --out /task/artifacts/bench \\
  --sequence-name {quoted} \\
  --config-label default \\
  --repo /src \\
  --max-dt-ms 4.166667 \\
  --min-match-rate 0.80 \\
  --rpe-delta-s 1.0 \\
  --estimator-enable-moving-bootstrap \\
{activation}  --probe-b /task/artifacts/bench/probe-b.jsonl \\
  --force
"""


def _required_mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RemoteCiError(f"TUM-VI preflight {field} is missing or invalid")
    return value


def _required_sha256_hex(value: Any, field: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise RemoteCiError(
            f"TUM-VI preflight {field} must be 64 lowercase SHA-256 hex digits"
        )
    return value


def _source_input_identity(
    source_root: Path,
    source_manifest: dict[str, Any],
    relative: str,
    label: str,
) -> dict[str, str]:
    entries = source_manifest.get("entries")
    if not isinstance(entries, list):
        raise RemoteCiError("TUM-VI preflight source manifest entries are invalid")
    matches = [
        entry
        for entry in entries
        if isinstance(entry, dict) and entry.get("path") == relative
    ]
    if len(matches) != 1:
        raise RemoteCiError(
            f"TUM-VI preflight source manifest must contain exactly one {label}: "
            f"{relative}"
        )
    entry = matches[0]
    if entry.get("type") != "file":
        raise RemoteCiError(f"TUM-VI preflight {label} is not a regular source file")
    path = source_root / Path(*PurePosixPath(relative).parts)
    if not path.is_file():
        raise RemoteCiError(f"TUM-VI preflight {label} is missing: {path}")
    actual_sha256 = sha256_file(path)
    expected_sha256 = _required_sha256_hex(entry.get("sha256"), f"{label} SHA")
    if actual_sha256 != expected_sha256:
        raise RemoteCiError(
            f"TUM-VI preflight {label} SHA mismatch: "
            f"{actual_sha256} != {expected_sha256}"
        )
    expected_size = entry.get("size")
    if not isinstance(expected_size, int) or path.stat().st_size != expected_size:
        raise RemoteCiError(
            f"TUM-VI preflight {label} size mismatch: "
            f"{path.stat().st_size} != {expected_size!r}"
        )
    return {"path": relative, "sha256": actual_sha256}


def _tum_vi_groundtruth_identities(
    dataset_root: Path, dataset_receipt: dict[str, Any]
) -> dict[str, dict[str, str]]:
    tree_manifest = _required_mapping(
        dataset_receipt.get("dataset_tree_manifest"), "dataset tree manifest"
    )
    entries = tree_manifest.get("entries")
    if not isinstance(entries, list):
        raise RemoteCiError("TUM-VI preflight dataset tree entries are invalid")

    groundtruth: dict[str, dict[str, str]] = {}
    for room in TUM_VI_ROOM_SEQUENCES:
        relative = f"{room}/dso/gt_imu.csv"
        matches = [
            entry
            for entry in entries
            if isinstance(entry, dict) and entry.get("path") == relative
        ]
        if len(matches) != 1:
            raise RemoteCiError(
                "TUM-VI preflight dataset tree must contain exactly one "
                f"regular groundtruth entry: {relative}"
            )
        digest = _required_sha256_hex(
            matches[0].get("sha256"), f"groundtruth SHA for {room}"
        )
        path = dataset_root / Path(*PurePosixPath(relative).parts)
        try:
            metadata = path.lstat()
        except OSError as error:
            raise RemoteCiError(
                f"TUM-VI preflight groundtruth is unavailable: {path}: {error}"
            ) from error
        if not stat.S_ISREG(metadata.st_mode):
            raise RemoteCiError(
                f"TUM-VI preflight groundtruth is not a regular file: {path}"
            )
        actual_digest = sha256_file(path)
        if actual_digest != digest:
            raise RemoteCiError(
                f"TUM-VI preflight groundtruth SHA mismatch for {room}: "
                f"{actual_digest} != {digest}"
            )
        groundtruth[room] = {"path": relative, "sha256": digest}
    return groundtruth


def _tum_vi_rootfs_layers(image: dict[str, Any]) -> list[str]:
    layers = image.get("rootfs_layers")
    if not isinstance(layers, list) or not layers:
        raise RemoteCiError("TUM-VI preflight image rootfs layers are missing")
    validated: list[str] = []
    for index, layer in enumerate(layers):
        if not isinstance(layer, str):
            raise RemoteCiError(
                f"TUM-VI preflight image rootfs layer {index} is invalid"
            )
        validated.append(
            _validate_sha256_content_id(
                layer, f"TUM-VI preflight image rootfs layer {index}"
            )
        )
    if len(set(validated)) != len(validated):
        raise RemoteCiError("TUM-VI preflight image rootfs layers contain duplicates")
    return validated


def _runtime_linkage_file_identity(value: Any, label: str) -> None:
    item = _required_mapping(value, label)
    if set(item) != {"path", "realpath", "sha256"}:
        raise RemoteCiError(f"TUM-VI preflight {label} identity fields mismatch")
    for field in ("path", "realpath"):
        path = item.get(field)
        if not isinstance(path, str) or not path.startswith("/"):
            raise RemoteCiError(
                f"TUM-VI preflight {label} {field} must be an absolute path"
            )
    _required_sha256_hex(item.get("sha256"), f"{label} SHA")


def _validate_tum_vi_runtime_linkage(
    value: Any, binaries: dict[str, str]
) -> dict[str, Any]:
    linkage = _required_mapping(value, "runtime linkage")
    if set(linkage) != {"schema_version", "method", "binaries"}:
        raise RemoteCiError("TUM-VI preflight runtime linkage fields mismatch")
    if linkage.get("schema_version") != SCHEMA_VERSION:
        raise RemoteCiError("TUM-VI preflight runtime linkage schema mismatch")
    if linkage.get("method") != "readelf-ldd-sha256":
        raise RemoteCiError("TUM-VI preflight runtime linkage method mismatch")
    linked_binaries = _required_mapping(
        linkage.get("binaries"), "runtime linkage binaries"
    )
    if set(linked_binaries) != set(TUM_VI_PREFLIGHT_BINARIES):
        raise RemoteCiError("TUM-VI preflight runtime linkage binary set mismatch")

    for name in TUM_VI_PREFLIGHT_BINARIES:
        item = _required_mapping(
            linked_binaries.get(name), f"runtime linkage binary {name}"
        )
        if set(item) != {
            "binary_sha256",
            "interpreter",
            "libraries",
            "virtual",
            "missing",
        }:
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage fields mismatch for {name}"
            )
        if item.get("binary_sha256") != binaries[name]:
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage binary SHA mismatch for {name}"
            )
        _runtime_linkage_file_identity(
            item.get("interpreter"), f"runtime linkage interpreter for {name}"
        )
        libraries = item.get("libraries")
        if not isinstance(libraries, list) or not libraries:
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage libraries are missing for {name}"
            )
        library_keys: list[tuple[str, str, str]] = []
        for index, library in enumerate(libraries):
            mapping = _required_mapping(
                library, f"runtime linkage library {index} for {name}"
            )
            if set(mapping) != {"soname", "path", "realpath", "sha256"}:
                raise RemoteCiError(
                    "TUM-VI preflight runtime linkage library fields mismatch "
                    f"for {name}"
                )
            soname = mapping.get("soname")
            if not isinstance(soname, str) or not soname:
                raise RemoteCiError(
                    f"TUM-VI preflight runtime linkage SONAME is invalid for {name}"
                )
            _runtime_linkage_file_identity(
                {key: mapping[key] for key in ("path", "realpath", "sha256")},
                f"runtime linkage library {soname} for {name}",
            )
            library_keys.append(
                (soname, str(mapping["path"]), str(mapping["realpath"]))
            )
        if library_keys != sorted(set(library_keys)):
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage libraries are not canonical for {name}"
            )
        virtual = item.get("virtual")
        if (
            not isinstance(virtual, list)
            or any(not isinstance(entry, str) or not entry for entry in virtual)
            or virtual != sorted(set(virtual))
        ):
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage virtual entries are invalid for {name}"
            )
        if item.get("missing") != []:
            raise RemoteCiError(
                f"TUM-VI preflight runtime linkage has missing libraries for {name}"
            )
    return linkage


def _tum_vi_runtime_linkage_collector_source() -> str:
    binaries = repr(list(TUM_VI_PREFLIGHT_BINARIES))
    return f'''import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

BINARIES = {binaries}
OUTPUT = Path("/task/artifacts/runtime-linkage.json")

def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()

def identity(path_text):
    path = Path(path_text)
    realpath = Path(os.path.realpath(path))
    if not path.is_absolute() or not realpath.is_file():
        raise RuntimeError(f"runtime library is unavailable: {{path_text}}")
    return {{
        "path": path.as_posix(),
        "realpath": realpath.as_posix(),
        "sha256": sha256(realpath),
    }}

result = {{
    "schema_version": 1,
    "method": "readelf-ldd-sha256",
    "binaries": {{}},
}}
for name in BINARIES:
    binary = Path("/shared-build") / name
    if not binary.is_file():
        raise RuntimeError(f"runtime binary is unavailable: {{binary}}")
    elf = subprocess.run(
        ["readelf", "-l", str(binary)],
        check=True,
        capture_output=True,
        text=True,
    )
    matches = re.findall(r"Requesting program interpreter: ([^]]+)", elf.stdout)
    if len(matches) != 1:
        raise RuntimeError(f"expected one ELF interpreter for {{name}}, got {{matches}}")

    traced = subprocess.run(
        ["ldd", str(binary)],
        check=False,
        capture_output=True,
        text=True,
    )
    if traced.returncode != 0:
        raise RuntimeError(
            f"ldd failed for {{name}} (rc={{traced.returncode}}): {{traced.stderr.strip()}}"
        )
    libraries = []
    virtual = []
    missing = []
    for raw_line in traced.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if "=>" in line:
            soname, resolved = (part.strip() for part in line.split("=>", 1))
            if resolved.startswith("not found"):
                missing.append(soname)
                continue
            path_text = resolved.split(" (", 1)[0].strip()
        elif line.startswith("/"):
            path_text = line.split(" (", 1)[0].strip()
            soname = Path(path_text).name
        else:
            soname = line.split(None, 1)[0]
            if soname.startswith("linux-vdso"):
                virtual.append(soname)
                continue
            raise RuntimeError(f"unrecognized ldd output for {{name}}: {{line}}")
        libraries.append({{"soname": soname, **identity(path_text)}})

    libraries.sort(key=lambda item: (item["soname"], item["path"], item["realpath"]))
    result["binaries"][name] = {{
        "binary_sha256": sha256(binary),
        "interpreter": identity(matches[0]),
        "libraries": libraries,
        "virtual": sorted(set(virtual)),
        "missing": sorted(set(missing)),
    }}

OUTPUT.write_text(
    json.dumps(result, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\\n",
    encoding="utf-8",
)
'''


def _tum_vi_runtime_linkage_container_command(
    run: dict[str, Any], shared_build: Path, task_root: Path
) -> list[str]:
    image = _required_mapping(run.get("image"), "image identity")
    remote_image_id = image.get("remote_id")
    if not isinstance(remote_image_id, str):
        raise RemoteCiError("TUM-VI preflight remote image ID is missing")
    _validate_sha256_content_id(
        remote_image_id, "TUM-VI preflight remote image ID"
    )
    immutable_image_ids = {
        image_id
        for image_id in (
            image.get("oci_manifest_id"),
            image.get("oci_config_id"),
        )
        if isinstance(image_id, str)
    }
    if remote_image_id not in immutable_image_ids:
        raise RemoteCiError(
            "TUM-VI preflight remote image ID differs from its OCI identity"
        )
    command = _container_base(
        run,
        "preflight-linkage",
        cpus=1,
        memory="1g",
        mounts=(
            (Path(str(run["source_root"])), "/src", True),
            (shared_build, "/shared-build", True),
            (task_root, "/task", False),
        ),
    )
    command.extend(
        [
            "--rm",
            remote_image_id,
            "/opt/phad-venv/bin/python",
            "-c",
            _tum_vi_runtime_linkage_collector_source(),
        ]
    )
    return command


def _collect_tum_vi_runtime_linkage(
    run: dict[str, Any], shared_build: Path
) -> dict[str, Any]:
    run_root = Path(str(run["run_root"]))
    with tempfile.TemporaryDirectory(
        prefix=".preflight-linkage-", dir=run_root
    ) as temporary:
        task_root = Path(temporary)
        _task_directories(task_root)
        command = _tum_vi_runtime_linkage_container_command(
            run, shared_build, task_root
        )
        run_command(command, capture_output=False)
        output = task_root / "artifacts" / "runtime-linkage.json"
        linkage = read_json(output)
    binaries = {
        name: sha256_file(shared_build / name) for name in TUM_VI_PREFLIGHT_BINARIES
    }
    return _validate_tum_vi_runtime_linkage(linkage, binaries)


def _ensure_no_tum_vi_benchmark_tasks(run: dict[str, Any]) -> None:
    tasks_root = Path(str(run["run_root"])) / "tasks"
    for role, repeat in TUM_VI_CONFIRMATORY_BATCH_ORDER:
        for sequence in TUM_VI_ROOM_SEQUENCES:
            task = tasks_root / f"{role}-{repeat}-{sequence}"
            if task.exists() or task.is_symlink():
                raise RemoteCiError(
                    f"TUM-VI preflight found pre-existing benchmark task: {task.name}"
                )


def _write_tum_vi_preflight_receipt(
    run: dict[str, Any],
    shared_build: Path,
    dataset_receipt: dict[str, Any],
    runtime_linkage: dict[str, Any],
) -> dict[str, Any]:
    _ensure_no_tum_vi_benchmark_tasks(run)
    definition = profile_definition(TUM_VI_CONFIRMATORY_PROFILE)
    if run.get("sequences") != list(TUM_VI_ROOM_SEQUENCES):
        raise RemoteCiError("TUM-VI preflight sequence order mismatch")
    expected_batch_order = [list(batch) for batch in TUM_VI_CONFIRMATORY_BATCH_ORDER]
    if run.get("batch_order") != expected_batch_order:
        raise RemoteCiError("TUM-VI preflight batch order mismatch")

    source_manifest = _required_mapping(
        run.get("source_manifest"), "source manifest"
    )
    source_id = run.get("source_id")
    if not isinstance(source_id, str):
        raise RemoteCiError("TUM-VI preflight source ID is missing")
    _validate_sha256_content_id(source_id, "TUM-VI preflight source ID")
    actual_source_id = _canonical_json_id(source_manifest)
    if source_id != actual_source_id:
        raise RemoteCiError(
            f"TUM-VI preflight source ID mismatch: {source_id} != {actual_source_id}"
        )
    source_root_value = run.get("source_root")
    if not isinstance(source_root_value, str) or not source_root_value:
        raise RemoteCiError("TUM-VI preflight source root is missing")
    source_root = Path(source_root_value)
    if not source_root.is_absolute() or not source_root.is_dir():
        raise RemoteCiError(
            f"TUM-VI preflight source root is invalid: {source_root}"
        )
    inputs = {
        "protocol": _source_input_identity(
            source_root,
            source_manifest,
            TUM_VI_PREFLIGHT_PROTOCOL_PATH,
            "protocol",
        ),
        "analyzer": _source_input_identity(
            source_root,
            source_manifest,
            TUM_VI_PREFLIGHT_ANALYZER_PATH,
            "analyzer",
        ),
    }

    dataset_manifest = _required_mapping(
        run.get("dataset_manifest"), "dataset manifest"
    )
    dataset_manifest_id = run.get("dataset_manifest_id")
    if not isinstance(dataset_manifest_id, str):
        raise RemoteCiError("TUM-VI preflight dataset manifest ID is missing")
    if dataset_manifest_id != _canonical_json_id(dataset_manifest):
        raise RemoteCiError("TUM-VI preflight dataset manifest ID mismatch")
    if (
        dataset_manifest != definition["dataset_manifest"]
        or dataset_manifest_id != definition["dataset_manifest_id"]
    ):
        raise RemoteCiError("TUM-VI preflight dataset declaration mismatch")
    dataset_content_id = run.get("dataset_content_id")
    if not isinstance(dataset_content_id, str):
        raise RemoteCiError("TUM-VI preflight dataset content ID is missing")
    _validate_sha256_content_id(
        dataset_content_id, "TUM-VI preflight dataset content ID"
    )
    if dataset_receipt.get("dataset_manifest_id") != dataset_manifest_id:
        raise RemoteCiError("TUM-VI preflight dataset receipt manifest mismatch")
    if dataset_receipt.get("dataset_manifest") != dataset_manifest:
        raise RemoteCiError("TUM-VI preflight dataset receipt declaration mismatch")
    if dataset_receipt.get("dataset_content_id") != dataset_content_id:
        raise RemoteCiError("TUM-VI preflight dataset receipt content mismatch")
    dataset_receipt_path = Path(str(run["dataset_root"])) / ".phad-dataset.json"
    if not dataset_receipt_path.is_file():
        raise RemoteCiError("TUM-VI preflight dataset receipt is missing")
    groundtruth = _tum_vi_groundtruth_identities(
        Path(str(run["dataset_root"])), dataset_receipt
    )

    image = _required_mapping(run.get("image"), "image identity")
    oci_manifest_id = image.get("oci_manifest_id")
    oci_config_id = image.get("oci_config_id")
    if not isinstance(oci_manifest_id, str) or not isinstance(oci_config_id, str):
        raise RemoteCiError("TUM-VI preflight OCI manifest/config ID is missing")
    _validate_sha256_content_id(oci_manifest_id, "TUM-VI preflight OCI manifest ID")
    _validate_sha256_content_id(oci_config_id, "TUM-VI preflight OCI config ID")
    toolchain_id = _required_sha256_hex(
        image.get("toolchain_id"), "toolchain ID"
    )
    if image.get("id") != oci_config_id:
        raise RemoteCiError("TUM-VI preflight local image/config ID mismatch")
    if image.get("remote_id") not in {oci_manifest_id, oci_config_id}:
        raise RemoteCiError("TUM-VI preflight remote OCI identity mismatch")
    rootfs_layers = _tum_vi_rootfs_layers(image)

    binaries: dict[str, str] = {}
    for name in TUM_VI_PREFLIGHT_BINARIES:
        path = shared_build / name
        if not path.is_file():
            raise RemoteCiError(f"TUM-VI preflight binary is missing: {path}")
        binaries[name] = sha256_file(path)
    runtime_linkage = _validate_tum_vi_runtime_linkage(
        runtime_linkage, binaries
    )

    document: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "profile": TUM_VI_CONFIRMATORY_PROFILE,
        "created_utc": utc_now(),
        "source_id": source_id,
        "dataset": {
            "manifest_id": dataset_manifest_id,
            "content_id": dataset_content_id,
            "receipt_sha256": sha256_file(dataset_receipt_path),
            "groundtruth": groundtruth,
        },
        "image": {
            "oci_manifest_id": oci_manifest_id,
            "oci_config_id": oci_config_id,
            "toolchain_id": toolchain_id,
            "rootfs_layers": rootfs_layers,
        },
        "inputs": inputs,
        "binaries": binaries,
        "runtime_linkage": runtime_linkage,
        "batch_order": expected_batch_order,
        "benchmark_commands": {
            role: {
                room: _tum_vi_sequence_script(room, role)
                for room in TUM_VI_ROOM_SEQUENCES
            }
            for role in ("off", "on")
        },
    }
    receipt = {
        **document,
        "preflight_receipt_id": _canonical_json_id(document),
    }
    receipt_path = Path(str(run["run_root"])) / "preflight-receipt.json"
    if receipt_path.exists():
        if read_json(receipt_path) != receipt:
            raise RemoteCiError(
                f"TUM-VI preflight receipt conflicts with current identity: {receipt_path}"
            )
        return receipt
    atomic_write_json(receipt_path, receipt)
    if read_json(receipt_path) != receipt:
        raise RemoteCiError("TUM-VI preflight receipt read-back mismatch")
    receipt_path.chmod(0o444)
    return receipt


def _collect_task_statuses(run_root: Path) -> dict[str, dict[str, Any]]:
    statuses: dict[str, dict[str, Any]] = {}
    for path in sorted((run_root / "tasks").glob("*/status.json")):
        try:
            statuses[path.parent.name] = read_json(path)
        except RemoteCiError:
            continue
    return statuses


def aggregate_run_state(tasks: dict[str, dict[str, Any]]) -> str:
    states = {task.get("state") for task in tasks.values()}
    if "failed" in states or "blocked" in states:
        return "failed"
    if "warning" in states:
        return "warning"
    if tasks and states <= TERMINAL_TASK_STATES:
        return "passed"
    return "running"


def _create_results_archive(run_root: Path) -> tuple[Path, str]:
    selected: list[Path] = []
    for name in (
        "run.json",
        "run-status.json",
        "preflight-receipt.json",
        "worker.pid",
        "orchestration.log",
        "orchestration-error.log",
    ):
        path = run_root / name
        if path.exists():
            selected.append(path)
    for task in sorted((run_root / "tasks").glob("*")):
        for relative in ("status.json", "container.cid"):
            path = task / relative
            if path.exists():
                selected.append(path)
        for directory in (task / "logs", task / "artifacts"):
            if directory.exists():
                selected.extend(sorted(path for path in directory.rglob("*") if path.is_file()))

    with tempfile.TemporaryDirectory(prefix="phad-remote-ci-pack-") as temporary:
        tar_path = Path(temporary) / "results.tar"
        with tarfile.open(tar_path, "w", format=tarfile.PAX_FORMAT) as archive:
            for path in selected:
                archive.add(path, arcname=path.relative_to(run_root).as_posix(), recursive=False)
        temporary_zst = run_root / f".results.tar.zst.tmp-{os.getpid()}"
        run_command(
            [
                "zstd",
                "-T0",
                "-3",
                "--no-progress",
                "--force",
                str(tar_path),
                "-o",
                str(temporary_zst),
            ],
            capture_output=False,
        )
        final = run_root / "results.tar.zst"
        os.replace(temporary_zst, final)
    digest = sha256_file(final)
    atomic_write_text(run_root / "results.sha256", f"{digest}  {final.name}\n")
    return final, digest


def _verify_run_source_identity(run: dict[str, Any]) -> dict[str, Any]:
    source_root = Path(str(run.get("source_root", "")))
    source_manifest = run.get("source_manifest")
    source_id = run.get("source_id")
    if not isinstance(source_manifest, dict) or not isinstance(source_id, str):
        raise RemoteCiError("run source identity is missing")
    manifest_path = source_root.parent / "inputs" / "source-manifest.json"
    expected_manifest = {**source_manifest, "source_id": source_id}
    if read_json(manifest_path) != expected_manifest:
        raise RemoteCiError("published source manifest differs from run identity")
    verified_source = verify_source_tree(source_root, manifest_path)
    if verified_source.get("source_id") != source_id:
        raise RemoteCiError("published source tree differs from run identity")
    return verified_source


def _remote_worker_impl(
    run_root: Path,
    *,
    tumvi_dataset_receipt: dict[str, Any] | None = None,
) -> int:
    run = read_json(run_root / "run.json")
    if Path(str(run["run_root"])).resolve() != run_root.resolve():
        raise RemoteCiError("run.json run_root does not match worker argument")
    _verify_run_source_identity(run)
    status_path = run_root / "run-status.json"
    atomic_write_json(
        status_path,
        {
            "schema_version": SCHEMA_VERSION,
            "run_id": run["run_id"],
            "profile": run["profile"],
            "state": "building",
            "started_utc": utc_now(),
            "updated_utc": utc_now(),
            "finished_utc": None,
        },
    )
    build = _run_remote_task(
        run,
        "build",
        "build",
        _build_script(run),
        cpus=16,
        memory="32g",
    )
    _verify_run_source_identity(run)
    shared_build = run_root / "tasks" / "build" / "build"
    if build["state"] != "passed":
        reason = f"shared build failed (rc={build.get('return_code')})"
        _write_blocked_task(run, "unit", "unit", reason)
        if run["profile"] == TUM_VI_CONFIRMATORY_PROFILE:
            for role, repeat in run["batch_order"]:
                for sequence in run["sequences"]:
                    _write_blocked_task(
                        run,
                        f"{role}-{repeat}-{sequence}",
                        "tum-vi",
                        reason,
                    )
        else:
            for sequence in run["sequences"]:
                _write_blocked_task(run, sequence, "euroc", reason)
    else:
        atomic_write_json(
            status_path,
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": run["run_id"],
                "profile": run["profile"],
                "state": "running",
                "started_utc": build["started_utc"],
                "updated_utc": utc_now(),
                "finished_utc": None,
            },
        )
        unit_mounts = ((shared_build, "/shared-build", True),)
        if run["profile"] == TUM_VI_CONFIRMATORY_PROFILE:
            unit = _run_remote_task(
                run,
                "unit",
                "unit",
                _unit_script(run),
                cpus=8,
                memory="16g",
                extra_mounts=unit_mounts,
            )
            blocked_reason: str | None = None
            if unit["state"] != "passed":
                blocked_reason = f"unit task failed (rc={unit.get('return_code')})"
            elif tumvi_dataset_receipt is not None:
                _ensure_no_tum_vi_benchmark_tasks(run)
                runtime_linkage = _collect_tum_vi_runtime_linkage(
                    run, shared_build
                )
                _write_tum_vi_preflight_receipt(
                    run,
                    shared_build,
                    tumvi_dataset_receipt,
                    runtime_linkage,
                )
            dataset_mounts = (
                (shared_build, "/shared-build", True),
                (Path(run["dataset_root"]), "/datasets/tum-vi", True),
            )
            for role, repeat in run["batch_order"]:
                batch_names = [
                    f"{role}-{repeat}-{sequence}" for sequence in run["sequences"]
                ]
                if blocked_reason is not None:
                    for name in batch_names:
                        _write_blocked_task(run, name, "tum-vi", blocked_reason)
                    continue
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=int(run["max_parallel_tasks"])
                ) as executor:
                    futures = [
                        executor.submit(
                            _run_remote_task,
                            run,
                            name,
                            "tum-vi",
                            _tum_vi_sequence_script(sequence, str(role)),
                            cpus=3,
                            memory="8g",
                            extra_mounts=dataset_mounts,
                        )
                        for name, sequence in zip(batch_names, run["sequences"])
                    ]
                    batch = [future.result() for future in futures]
                failed = [task for task in batch if task["state"] == "failed"]
                if failed:
                    names = ", ".join(str(task["name"]) for task in failed)
                    blocked_reason = f"earlier confirmatory batch failed: {names}"
        else:
            jobs: list[
                tuple[str, str, str, int, str, Sequence[tuple[Path, str, bool]]]
            ] = [
                (
                    "unit",
                    "unit",
                    _unit_script(run),
                    8,
                    "16g",
                    unit_mounts,
                )
            ]
            for sequence in run["sequences"]:
                jobs.append(
                    (
                        sequence,
                        "euroc",
                        _sequence_script(sequence),
                        3,
                        "8g",
                        (
                            (shared_build, "/shared-build", True),
                            (Path(run["dataset_root"]), "/datasets/euroc", True),
                        ),
                    )
                )
            unit_job = jobs[0]
            sequence_jobs = jobs[1:]
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=1
            ) as unit_executor, concurrent.futures.ThreadPoolExecutor(
                max_workers=int(run["max_parallel_tasks"])
            ) as sequence_executor:
                name, kind, script, cpus, memory, mounts = unit_job
                futures = [
                    unit_executor.submit(
                        _run_remote_task,
                        run,
                        name,
                        kind,
                        script,
                        cpus=cpus,
                        memory=memory,
                        extra_mounts=mounts,
                    )
                ]
                futures.extend(
                    sequence_executor.submit(
                        _run_remote_task,
                        run,
                        name,
                        kind,
                        script,
                        cpus=cpus,
                        memory=memory,
                        extra_mounts=mounts,
                    )
                    for name, kind, script, cpus, memory, mounts in sequence_jobs
                )
                for future in concurrent.futures.as_completed(futures):
                    future.result()

    tasks = _collect_task_statuses(run_root)
    final_state = aggregate_run_state(tasks)
    final_status = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run["run_id"],
        "profile": run["profile"],
        "state": "finalizing",
        "started_utc": build["started_utc"],
        "updated_utc": utc_now(),
        "finished_utc": None,
        "tasks": tasks,
    }
    atomic_write_json(status_path, final_status)
    _, archive_sha256 = _create_results_archive(run_root)
    final_status.update(
        {
            "state": final_state,
            "updated_utc": utc_now(),
            "finished_utc": utc_now(),
            "results_archive_sha256": archive_sha256,
        }
    )
    atomic_write_json(status_path, final_status)
    return 0 if final_state in {"passed", "warning"} else 1


def remote_worker(run_root: Path) -> int:
    try:
        run = read_json(run_root / "run.json")
        tumvi_dataset_receipt: dict[str, Any] | None = None
        if run.get("profile") == TUM_VI_CONFIRMATORY_PROFILE:
            content_id = run.get("dataset_content_id")
            if not isinstance(content_id, str):
                raise RemoteCiError(
                    "TUM-VI confirmatory run has no bound dataset content id"
                )
            content_hex = _validate_sha256_content_id(
                content_id, "dataset content id"
            ).removeprefix("sha256:")
            definition = profile_definition(TUM_VI_CONFIRMATORY_PROFILE)
            expected_dataset_root = (
                Path(str(definition["dataset_root"]))
                / "by-content"
                / "sha256"
                / content_hex
            )
            run_dataset_root = Path(str(run["dataset_root"]))
            if run_dataset_root != expected_dataset_root:
                raise RemoteCiError(
                    "TUM-VI run dataset root does not match its bound content slot: "
                    f"{run_dataset_root} != {expected_dataset_root}"
                )
            tumvi_dataset_receipt = _verify_tum_vi_published_dataset(
                expected_dataset_root,
                expected_content_id=content_id,
                expected_manifest_id=(
                    str(run["dataset_manifest_id"])
                    if "dataset_manifest_id" in run
                    else None
                ),
            )
        return _remote_worker_impl(
            run_root, tumvi_dataset_receipt=tumvi_dataset_receipt
        )
    except Exception as error:  # Worker 必须留下可 fetch 的终态证据。
        error_text = traceback.format_exc()
        atomic_write_text(run_root / "orchestration-error.log", error_text)
        try:
            run = read_json(run_root / "run.json")
            tasks = _collect_task_statuses(run_root)
            failed_status = {
                "schema_version": SCHEMA_VERSION,
                "run_id": run.get("run_id"),
                "profile": run.get("profile"),
                "state": "failed",
                "started_utc": None,
                "updated_utc": utc_now(),
                "finished_utc": utc_now(),
                "failure_reason": f"{type(error).__name__}: {error}",
                "tasks": tasks,
            }
            atomic_write_json(run_root / "run-status.json", failed_status)
            _, archive_sha256 = _create_results_archive(run_root)
            failed_status["results_archive_sha256"] = archive_sha256
            atomic_write_json(run_root / "run-status.json", failed_status)
        except Exception:
            with contextlib.suppress(OSError):
                with (run_root / "orchestration-error.log").open(
                    "a", encoding="utf-8"
                ) as stream:
                    stream.write("\nFailure while finalizing worker error:\n")
                    stream.write(traceback.format_exc())
        return 1


def remote_status_document(run_root: Path) -> dict[str, Any]:
    run = read_json(run_root / "run.json")
    status_path = run_root / "run-status.json"
    if status_path.exists():
        status = read_json(status_path)
    else:
        status = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run["run_id"],
            "profile": run["profile"],
            "state": "queued",
            "updated_utc": run["created_utc"],
        }
    status["tasks"] = _collect_task_statuses(run_root)
    return status


def _command_doctor(_: argparse.Namespace) -> int:
    for command in (
        "curl",
        "docker",
        "git",
        "scp",
        "ssh",
        "ssh-keyscan",
        "ssh-keygen",
        "tar",
        "unzstd",
        "zstd",
    ):
        if shutil.which(command) is None:
            raise RemoteCiError(f"required local command is missing: {command}")
    run_command(["docker", "version"])
    with VerifiedSsh() as ssh:
        value = remote_doctor(ssh)
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def _command_stage_dataset(arguments: argparse.Namespace) -> int:
    receipt = stage_remote_tumvi_dataset(
        arguments.local_root,
        profile=arguments.profile,
    )
    print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


def _command_verify_dataset(arguments: argparse.Namespace) -> int:
    receipt = verify_remote_tumvi_dataset(
        arguments.profile,
        arguments.dataset_content_id,
    )
    if arguments.json:
        print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(
            f"dataset_content_id={receipt['dataset_content_id']} "
            f"dataset_manifest_id={receipt['dataset_manifest_id']}"
        )
    return 0


def _command_start(arguments: argparse.Namespace) -> int:
    dataset_content_id = getattr(arguments, "dataset_content_id", None)
    if (
        arguments.profile == TUM_VI_CONFIRMATORY_PROFILE
        and dataset_content_id is None
    ):
        raise RemoteCiError(
            "TUM-VI confirmatory start requires --dataset-content-id"
        )
    if arguments.profile == TUM_VI_CONFIRMATORY_PROFILE:
        result = start_remote_run(
            repo_root(),
            arguments.profile,
            dataset_content_id=dataset_content_id,
        )
    else:
        result = start_remote_run(repo_root(), arguments.profile)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    print(
        f"status: python3 scripts/remote_ci.py status {result['run_id']}\n"
        f"wait:   python3 scripts/remote_ci.py wait {result['run_id']}"
    )
    return 0


def _command_status(arguments: argparse.Namespace) -> int:
    with VerifiedSsh() as ssh:
        status = remote_status(ssh, arguments.run_id)
    if arguments.json:
        print(json.dumps(status, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print_status(status)
    return 0


def _command_wait(arguments: argparse.Namespace) -> int:
    previous: tuple[Any, ...] | None = None
    while True:
        with VerifiedSsh() as ssh:
            status = remote_status(ssh, arguments.run_id)
        signature = _status_signature(status)
        if signature != previous:
            print_status(status)
            previous = signature
        if status.get("state") in TERMINAL_RUN_STATES:
            break
        time.sleep(arguments.interval)
    if not arguments.no_fetch:
        summary = fetch_remote_run(repo_root(), arguments.run_id)
        print(f"summary={summary}")
    return 0 if status.get("state") in {"passed", "warning"} else 1


def _command_fetch(arguments: argparse.Namespace) -> int:
    summary = fetch_remote_run(repo_root(), arguments.run_id)
    print(f"summary={summary}")
    return 0


def _command_remote_verify(arguments: argparse.Namespace) -> int:
    source_root = Path(arguments.source_root)
    result = verify_source_tree(source_root, Path(arguments.manifest))
    result["archive_sha256"] = arguments.archive_sha256
    result["bundle_sha256"] = arguments.bundle_sha256
    atomic_write_json(Path(arguments.ready), result)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


def _read_tumvi_profile_definition(path: Path) -> dict[str, Any]:
    definition = read_json(path)
    if definition.get("name") != TUM_VI_CONFIRMATORY_PROFILE:
        raise RemoteCiError("remote dataset definition has unsupported profile")
    declaration = definition.get("dataset_manifest")
    if not isinstance(declaration, dict):
        raise RemoteCiError("remote dataset definition has no manifest")
    if definition.get("dataset_manifest_id") != _canonical_json_id(declaration):
        raise RemoteCiError("remote dataset definition manifest id mismatch")
    return definition


def _command_remote_stage_tumvi_dataset(arguments: argparse.Namespace) -> int:
    definition = _read_tumvi_profile_definition(Path(arguments.profile_definition))
    store = Path(arguments.dataset_store)
    if store != Path(str(definition["dataset_root"])):
        raise RemoteCiError("remote dataset store differs from profile definition")
    archives: dict[str, Path] = {}
    for value in arguments.archive:
        sequence, separator, path = value.partition("=")
        if not separator or not sequence or not path or sequence in archives:
            raise RemoteCiError(f"invalid remote archive binding: {value!r}")
        archives[sequence] = Path(path)
    receipt = stage_remote_dataset(
        archives,
        store,
        definition=definition,
    )
    print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
    return 0


def _command_remote_verify_tumvi_dataset(arguments: argparse.Namespace) -> int:
    definition = _read_tumvi_profile_definition(Path(arguments.profile_definition))
    content_id = _validate_sha256_content_id(
        arguments.dataset_content_id, "dataset content id"
    )
    expected_root = (
        Path(str(definition["dataset_root"]))
        / "by-content"
        / "sha256"
        / content_id.removeprefix("sha256:")
    )
    root = Path(arguments.dataset_root)
    if root != expected_root:
        raise RemoteCiError("remote dataset root is not the bound content root")
    receipt = _verify_tum_vi_published_dataset(
        root,
        expected_content_id=content_id,
        expected_manifest_id=str(definition["dataset_manifest_id"]),
    )
    if receipt.get("dataset_manifest") != definition["dataset_manifest"]:
        raise RemoteCiError("remote dataset sentinel declaration mismatch")
    print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
    return 0


def _command_remote_worker(arguments: argparse.Namespace) -> int:
    return remote_worker(Path(arguments.run_root))


def _command_remote_status(arguments: argparse.Namespace) -> int:
    print(
        json.dumps(
            remote_status_document(Path(arguments.run_root)),
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor = subparsers.add_parser("doctor", help="核验本地/远端依赖、SSH 和数据集")
    doctor.set_defaults(handler=_command_doctor)

    stage_dataset = subparsers.add_parser(
        "stage-dataset", help="预验并安全发布远端冻结数据集"
    )
    stage_dataset.add_argument(
        "--profile",
        default=TUM_VI_CONFIRMATORY_PROFILE,
        choices=(TUM_VI_CONFIRMATORY_PROFILE,),
    )
    stage_dataset.add_argument("--local-root", required=True, type=Path)
    stage_dataset.set_defaults(handler=_command_stage_dataset)

    verify_dataset = subparsers.add_parser(
        "verify-dataset", help="复核显式content ID对应的远端完整数据树"
    )
    verify_dataset.add_argument(
        "--profile",
        default=TUM_VI_CONFIRMATORY_PROFILE,
        choices=(TUM_VI_CONFIRMATORY_PROFILE,),
    )
    verify_dataset.add_argument(
        "--dataset-content-id",
        required=True,
        type=_parse_sha256_content_id,
    )
    verify_dataset.add_argument("--json", action="store_true")
    verify_dataset.set_defaults(handler=_command_verify_dataset)

    start = subparsers.add_parser("start", help="构建镜像、上传快照并启动后台 run")
    start.add_argument(
        "--profile",
        default=PROFILE,
        choices=(PROFILE, TUM_VI_CONFIRMATORY_PROFILE),
    )
    start.add_argument(
        "--dataset-content-id",
        type=_parse_sha256_content_id,
        default=None,
        help="显式绑定已发布的 TUM-VI dataset content ID",
    )
    start.set_defaults(handler=_command_start)

    status = subparsers.add_parser("status", help="读取一次远端状态")
    status.add_argument("run_id", type=lambda value: validate_identifier(value, "run-id"))
    status.add_argument("--json", action="store_true")
    status.set_defaults(handler=_command_status)

    wait = subparsers.add_parser("wait", help="低频轮询至终态并拉回结果")
    wait.add_argument("run_id", type=lambda value: validate_identifier(value, "run-id"))
    wait.add_argument("--interval", type=int, default=30)
    wait.add_argument("--no-fetch", action="store_true")
    wait.set_defaults(handler=_command_wait)

    fetch = subparsers.add_parser("fetch", help="拉回终态 run 并生成 Markdown")
    fetch.add_argument("run_id", type=lambda value: validate_identifier(value, "run-id"))
    fetch.set_defaults(handler=_command_fetch)

    verify = subparsers.add_parser("_remote-verify-source", help=argparse.SUPPRESS)
    verify.add_argument("--source-root", required=True)
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--ready", required=True)
    verify.add_argument("--archive-sha256", required=True)
    verify.add_argument("--bundle-sha256", required=True)
    verify.set_defaults(handler=_command_remote_verify)

    remote_stage_dataset = subparsers.add_parser(
        "_remote-stage-tumvi-dataset", help=argparse.SUPPRESS
    )
    remote_stage_dataset.add_argument("--profile-definition", required=True)
    remote_stage_dataset.add_argument("--dataset-store", required=True)
    remote_stage_dataset.add_argument("--archive", action="append", default=[])
    remote_stage_dataset.set_defaults(handler=_command_remote_stage_tumvi_dataset)

    remote_verify_dataset = subparsers.add_parser(
        "_remote-verify-tumvi-dataset", help=argparse.SUPPRESS
    )
    remote_verify_dataset.add_argument("--profile-definition", required=True)
    remote_verify_dataset.add_argument("--dataset-root", required=True)
    remote_verify_dataset.add_argument(
        "--dataset-content-id",
        required=True,
        type=_parse_sha256_content_id,
    )
    remote_verify_dataset.set_defaults(
        handler=_command_remote_verify_tumvi_dataset
    )

    worker = subparsers.add_parser("_remote-worker", help=argparse.SUPPRESS)
    worker.add_argument("--run-root", required=True)
    worker.set_defaults(handler=_command_remote_worker)

    remote_status_parser = subparsers.add_parser(
        "_remote-status", help=argparse.SUPPRESS
    )
    remote_status_parser.add_argument("--run-root", required=True)
    remote_status_parser.set_defaults(handler=_command_remote_status)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    arguments = parser.parse_args(argv)
    if getattr(arguments, "interval", 5) < 5:
        parser.error("--interval must be at least 5 seconds")
    try:
        return int(arguments.handler(arguments))
    except RemoteCiError as error:
        print(f"remote-ci error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

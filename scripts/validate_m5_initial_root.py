#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any


MH_CONTROL_COMMIT = "ba607c5738b3c5d9bc9cb0423199eee67648b0b3"
MH_CONTROL_CONFIG_HASH = "0337287b"
MH_CANDIDATE_FIRST_OK_INDEX = 6
MH_CANDIDATE_FIRST_OK_SPAN_NS = 300_000_000
MH_COMPARISON_TOLERANCE_M = 1e-6


class InputError(Exception):
    pass


class PredicateFailure(Exception):
    pass


@dataclass(frozen=True)
class DiagRow:
    timestamp_ns: int
    status: str


@dataclass(frozen=True)
class TumPose:
    timestamp_ns: Decimal


def fail(predicate: str, expected: object, actual: object) -> None:
    raise PredicateFailure(
        json.dumps(
            {"predicate": predicate, "expected": expected, "actual": actual},
            sort_keys=True,
        )
    )


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise InputError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise InputError(f"JSON root must be an object: {path}")
    return value


def nested(mapping: dict[str, Any], *keys: str) -> Any:
    value: Any = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise InputError(f"missing JSON key: {'.'.join(keys)}")
        value = value[key]
    return value


def finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InputError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise InputError(f"{name} must be finite")
    return result


def integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise InputError(f"{name} must be an integer")
    return value


def string_value(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise InputError(f"{name} must be a non-empty string")
    return value


def boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise InputError(f"{name} must be a boolean")
    return value


def path_value(value: Any, name: str) -> Path:
    return Path(string_value(value, name))


def fnv1a_hash8(text: str) -> str:
    value = 14_695_981_039_346_656_037
    for byte in text.encode("utf-8"):
        value ^= byte
        value = (value * 1_099_511_628_211) & 0xFFFF_FFFF_FFFF_FFFF
    return f"{value:016x}"[:8]


def read_diag(path: Path) -> list[DiagRow]:
    try:
        stream = path.open(newline="", encoding="utf-8")
    except OSError as error:
        raise InputError(f"cannot open diag CSV {path}: {error}") from error
    with stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        required = {"timestamp_ns", "status"}
        if not required.issubset(fields):
            raise InputError(
                f"diag CSV missing headers: {sorted(required - fields)}"
            )
        rows: list[DiagRow] = []
        seen: set[int] = set()
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp_ns = int(row["timestamp_ns"])
            except (KeyError, TypeError, ValueError) as error:
                raise InputError(
                    f"diag timestamp is invalid at line {line_number}"
                ) from error
            if timestamp_ns in seen:
                raise InputError(f"duplicate diag timestamp_ns: {timestamp_ns}")
            seen.add(timestamp_ns)
            status = row.get("status", "")
            if not status:
                raise InputError(f"diag status is empty at line {line_number}")
            rows.append(DiagRow(timestamp_ns, status))
    if not rows:
        raise InputError("diag CSV is empty")
    return rows


def read_tum(path: Path) -> list[TumPose]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise InputError(f"cannot read TUM trajectory {path}: {error}") from error
    poses: list[TumPose] = []
    seen: set[Decimal] = set()
    for line_number, line in enumerate(lines, start=1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise InputError(f"TUM line {line_number} must have 8 fields")
        try:
            timestamp_ns = Decimal(fields[0]) * Decimal(1_000_000_000)
            values = [float(field) for field in fields[1:]]
        except (InvalidOperation, ValueError) as error:
            raise InputError(f"TUM line {line_number} is invalid") from error
        if not timestamp_ns.is_finite() or not all(map(math.isfinite, values)):
            raise InputError(f"TUM line {line_number} is non-finite")
        if timestamp_ns in seen:
            raise InputError(f"duplicate TUM timestamp: {fields[0]}")
        seen.add(timestamp_ns)
        poses.append(TumPose(timestamp_ns))
    if not poses:
        raise InputError("TUM trajectory is empty")
    return poses


def verify_manifest(
    manifest_path: Path, expected_digest: str | None, sequence_root: Path
) -> str:
    try:
        manifest_bytes = manifest_path.read_bytes()
    except OSError as error:
        raise InputError(f"cannot read manifest {manifest_path}: {error}") from error
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    if expected_digest is not None and digest != expected_digest:
        raise InputError(
            f"manifest digest mismatch: expected={expected_digest} actual={digest}"
        )
    try:
        lines = manifest_bytes.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise InputError("manifest is not UTF-8") from error
    if not lines:
        raise InputError("manifest is empty")
    root = sequence_root.resolve()
    for line_number, line in enumerate(lines, start=1):
        pieces = line.split(maxsplit=1)
        if len(pieces) != 2 or len(pieces[0]) != 64:
            raise InputError(f"invalid manifest line {line_number}")
        expected_file_digest, relative_text = pieces
        relative_text = relative_text.strip()
        if relative_text.startswith("*"):
            relative_text = relative_text[1:]
        if relative_text.startswith("./"):
            relative_text = relative_text[2:]
        relative = Path(relative_text)
        if relative.is_absolute() or ".." in relative.parts:
            raise InputError(f"unsafe manifest path at line {line_number}")
        candidate = (root / relative).resolve()
        if candidate != root and root not in candidate.parents:
            raise InputError(f"manifest path escapes sequence root: {relative}")
        try:
            actual_file_digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        except OSError as error:
            raise InputError(f"cannot read manifest file {candidate}: {error}") from error
        if actual_file_digest != expected_file_digest:
            raise InputError(
                "manifest file mismatch: "
                f"path={relative} expected={expected_file_digest} "
                f"actual={actual_file_digest}"
            )
    return digest


def check_pose_mapping(diag: list[DiagRow], poses: list[TumPose]) -> list[DiagRow]:
    ok_rows = [row for row in diag if row.status == "ok"]
    if len(ok_rows) != len(poses):
        fail("ok rows map one-to-one to est.tum", len(ok_rows), len(poses))
    tolerance_ns = Decimal(500)
    for index, (row, pose) in enumerate(zip(ok_rows, poses, strict=True)):
        difference = abs(Decimal(row.timestamp_ns) - pose.timestamp_ns)
        if difference > tolerance_ns:
            fail(
                f"trajectory timestamp[{index}] within 500 ns",
                row.timestamp_ns,
                str(pose.timestamp_ns),
            )
    return ok_rows


def check_cadence(
    diag: list[DiagRow], *, earliest_ok_index: int | None, latest_ok_index: int | None
) -> int:
    try:
        first_ok = next(index for index, row in enumerate(diag) if row.status == "ok")
    except StopIteration as error:
        raise PredicateFailure("no ok row in diag") from error
    if earliest_ok_index is not None and first_ok < earliest_ok_index:
        fail("first ok index lower bound", f">={earliest_ok_index}", first_ok)
    if latest_ok_index is not None and first_ok > latest_ok_index:
        fail("first ok index upper bound", f"<={latest_ok_index}", first_ok)
    for index, row in enumerate(diag):
        expected = "initializing" if index < first_ok else "ok"
        if row.status != expected:
            fail(f"diag cadence row {index}", expected, row.status)
    return first_ok


def validate_v1(arguments: argparse.Namespace) -> dict[str, Any]:
    manifest_path = Path(arguments.manifest_sha256)
    try:
        manifest_bytes = manifest_path.read_bytes()
    except OSError as error:
        raise InputError(f"cannot read manifest {manifest_path}: {error}") from error
    actual_digest = hashlib.sha256(manifest_bytes).hexdigest()
    if actual_digest != arguments.expected_manifest_digest:
        raise InputError(
            "manifest digest mismatch: "
            f"expected={arguments.expected_manifest_digest} actual={actual_digest}"
        )

    run = Path(arguments.run_dir)
    meta = load_json(run / "meta.json")
    sequence_root = path_value(
        nested(meta, "sequence_root"), "meta.sequence_root"
    )
    manifest_digest = verify_manifest(
        manifest_path, arguments.expected_manifest_digest, sequence_root
    )
    summary = load_json(run / "summary.json")
    expected_sequence = "V1_02_medium"
    for source_name, source in (("meta", meta), ("summary", summary)):
        actual_sequence = string_value(
            nested(source, "sequence"), f"{source_name}.sequence"
        )
        if actual_sequence != expected_sequence:
            fail(f"{source_name}.sequence", expected_sequence, actual_sequence)
    config = nested(meta, "config")
    if not isinstance(config, dict):
        raise InputError("meta.config must be an object")
    expected_config: dict[str, object] = {
        "estimator.enable_moving_bootstrap": True,
        "eval.max_dt_ms": 2.5,
        "eval.min_match_rate": 0.5,
        "eval.rpe_delta_s": 1.0,
        "session.max_frames": 118,
    }
    for key, expected in expected_config.items():
        if key not in config:
            raise InputError(f"missing JSON key: meta.config.{key}")
        actual = config[key]
        if isinstance(expected, bool):
            actual = boolean(actual, f"meta.config.{key}")
        elif isinstance(expected, int):
            actual = integer(actual, f"meta.config.{key}")
        else:
            actual = finite_number(actual, f"meta.config.{key}")
        if actual != expected:
            fail(f"meta.config.{key}", expected, actual)
    canonical_text = string_value(
        nested(meta, "config_canonical_text"), "meta.config_canonical_text"
    )
    canonical: dict[str, str] = {}
    for line in canonical_text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            canonical[key] = value
    for key, expected in expected_config.items():
        if key not in canonical:
            raise InputError(f"canonical config key is missing: {key}")
        text = canonical[key]
        if isinstance(expected, bool):
            matches = text == ("true" if expected else "false")
        else:
            try:
                matches = Decimal(text) == Decimal(str(expected))
            except InvalidOperation as error:
                raise InputError(f"canonical config value is invalid: {key}") from error
        if not matches:
            fail(f"canonical config {key}", expected, text)
    meta_hash = string_value(nested(meta, "config_hash"), "meta.config_hash")
    summary_hash = string_value(
        nested(summary, "config_hash"), "summary.config_hash"
    )
    if meta_hash != summary_hash:
        fail("config hash identity", meta_hash, summary_hash)
    computed_hash = fnv1a_hash8(canonical_text)
    if meta_hash != computed_hash:
        fail("canonical config hash", computed_hash, meta_hash)
    meta_commit = string_value(
        nested(meta, "code", "git_commit_short"), "meta.code.git_commit_short"
    )
    summary_commit = string_value(
        nested(summary, "code", "git_commit_short"),
        "summary.code.git_commit_short",
    )
    if meta_commit != summary_commit:
        fail("code commit identity", meta_commit, summary_commit)
    meta_dirty = boolean(nested(meta, "code", "git_dirty"), "meta.code.git_dirty")
    summary_dirty = boolean(
        nested(summary, "code", "git_dirty"), "summary.code.git_dirty"
    )
    if meta_dirty != summary_dirty:
        fail("code dirty identity", meta_dirty, summary_dirty)
    diag = read_diag(run / "diag.csv")
    poses = read_tum(run / "est.tum")
    ok_rows = check_pose_mapping(diag, poses)
    first_ok = check_cadence(
        diag, earliest_ok_index=5, latest_ok_index=None
    )

    if len(diag) != 118:
        fail("diag row count", 118, len(diag))
    if diag[first_ok].timestamp_ns - diag[0].timestamp_ns < 250_000_000:
        fail(
            "first ok input span ns",
            ">=250000000",
            diag[first_ok].timestamp_ns - diag[0].timestamp_ns,
        )
    status = nested(summary, "status")
    if status not in {"completed", "completed_with_warnings"}:
        fail("summary status", "completed[_with_warnings]", status)

    trajectory = nested(summary, "trajectory")
    if not isinstance(trajectory, dict):
        raise InputError("summary.trajectory must be an object")
    expected_integers = {
        "image_frames": 118,
        "poses_written": len(poses),
        "ok": len(ok_rows),
        "rejected": 0,
        "failed": 0,
        "segments": 1,
    }
    for key, expected in expected_integers.items():
        actual = integer(nested(summary, "trajectory", key), f"trajectory.{key}")
        if actual != expected:
            fail(f"trajectory.{key}", expected, actual)
    emitted = integer(nested(summary, "sync", "emitted_stereo"), "sync.emitted_stereo")
    if emitted != 118:
        fail("sync.emitted_stereo", 118, emitted)
    for key in ("rejected", "failed"):
        actual = integer(nested(summary, "robustness", key), f"robustness.{key}")
        if actual != 0:
            fail(f"robustness.{key}", 0, actual)
    for key in ("completion_rate", "coverage_rate"):
        actual = finite_number(nested(summary, "trajectory", key), f"trajectory.{key}")
        if actual < 0.90:
            fail(f"trajectory.{key}", ">=0.90", actual)
    for metric in ("ate", "rpe"):
        actual = finite_number(
            nested(summary, metric, "trans", "rmse"),
            f"{metric}.trans.rmse",
        )
        if actual > 1.0:
            fail(f"{metric}.trans.rmse", "<=1.0", actual)
    if ok_rows[0].timestamp_ns != diag[first_ok].timestamp_ns:
        fail("first ok trajectory timestamp", diag[first_ok].timestamp_ns, ok_rows[0].timestamp_ns)
    if ok_rows[-1].timestamp_ns != diag[-1].timestamp_ns:
        fail("last frame trajectory timestamp", diag[-1].timestamp_ns, ok_rows[-1].timestamp_ns)

    return {
        "mode": "v102-118",
        "passed": True,
        "manifest_digest": manifest_digest,
        "diag_rows": len(diag),
        "poses": len(poses),
        "first_ok_index": first_ok,
    }


def load_common_run(run: Path) -> dict[str, Any]:
    meta = load_json(run / "meta.json")
    summary = load_json(run / "summary.json")
    diag = read_diag(run / "diag.csv")
    poses = read_tum(run / "est.tum")
    ok_rows = check_pose_mapping(diag, poses)
    first_ok = check_cadence(
        diag, earliest_ok_index=None, latest_ok_index=None
    )
    status = nested(summary, "status")
    if status not in {"completed", "completed_with_warnings"}:
        fail("summary status", "completed[_with_warnings]", status)
    expected_integers = {
        "image_frames": len(diag),
        "poses_written": len(poses),
        "ok": len(ok_rows),
        "rejected": 0,
        "failed": 0,
        "segments": 1,
    }
    for key, expected in expected_integers.items():
        actual = integer(nested(summary, "trajectory", key), f"trajectory.{key}")
        if actual != expected:
            fail(f"trajectory.{key}", expected, actual)
    emitted = integer(nested(summary, "sync", "emitted_stereo"), "sync.emitted_stereo")
    if emitted != len(diag):
        fail("sync.emitted_stereo", len(diag), emitted)
    for key in ("rejected", "failed", "reanchors"):
        actual = integer(nested(summary, "robustness", key), f"robustness.{key}")
        if actual != 0:
            fail(f"robustness.{key}", 0, actual)
    metrics = {
        "completion": finite_number(
            nested(summary, "trajectory", "completion_rate"),
            "trajectory.completion_rate",
        ),
        "coverage": finite_number(
            nested(summary, "trajectory", "coverage_rate"),
            "trajectory.coverage_rate",
        ),
        "ate": finite_number(
            nested(summary, "ate", "trans", "rmse"), "ate.trans.rmse"
        ),
        "rpe": finite_number(
            nested(summary, "rpe", "trans", "rmse"), "rpe.trans.rmse"
        ),
    }
    meta_sequence = string_value(nested(meta, "sequence"), "meta.sequence")
    summary_sequence = string_value(
        nested(summary, "sequence"), "summary.sequence"
    )
    if meta_sequence != summary_sequence:
        fail("run sequence identity", meta_sequence, summary_sequence)
    config = nested(meta, "config")
    if not isinstance(config, dict):
        raise InputError("meta.config must be an object")
    canonical_text = string_value(
        nested(meta, "config_canonical_text"), "meta.config_canonical_text"
    )
    meta_hash = string_value(nested(meta, "config_hash"), "meta.config_hash")
    summary_hash = string_value(
        nested(summary, "config_hash"), "summary.config_hash"
    )
    if meta_hash != summary_hash:
        fail("run config hash identity", meta_hash, summary_hash)
    computed_hash = fnv1a_hash8(canonical_text)
    if meta_hash != computed_hash:
        fail("run canonical config hash", computed_hash, meta_hash)
    meta_commit = string_value(
        nested(meta, "code", "git_commit_short"), "meta.code.git_commit_short"
    )
    summary_commit = string_value(
        nested(summary, "code", "git_commit_short"),
        "summary.code.git_commit_short",
    )
    if meta_commit != summary_commit:
        fail("run code commit identity", meta_commit, summary_commit)
    meta_dirty = boolean(nested(meta, "code", "git_dirty"), "meta.code.git_dirty")
    summary_dirty = boolean(
        nested(summary, "code", "git_dirty"), "summary.code.git_dirty"
    )
    if meta_dirty != summary_dirty:
        fail("run code dirty identity", meta_dirty, summary_dirty)
    return {
        "meta": meta,
        "diag": diag,
        "poses": poses,
        "first_ok": first_ok,
        "metrics": metrics,
        "identity": {
            "sequence": meta_sequence,
            "config": config,
            "canonical_text": canonical_text,
            "config_hash": meta_hash,
        },
    }


def validate_mh_control_identity(control_dir: Path) -> None:
    meta = load_json(control_dir / "meta.json")
    commit = string_value(
        nested(meta, "code", "git_commit"), "control meta.code.git_commit"
    )
    if commit != MH_CONTROL_COMMIT:
        fail("control meta.code.git_commit", MH_CONTROL_COMMIT, commit)
    dirty = boolean(
        nested(meta, "code", "git_dirty"), "control meta.code.git_dirty"
    )
    if dirty:
        fail("control meta.code.git_dirty", False, dirty)
    config_hash = string_value(
        nested(meta, "config_hash"), "control meta.config_hash"
    )
    if config_hash != MH_CONTROL_CONFIG_HASH:
        fail(
            "control meta.config_hash",
            MH_CONTROL_CONFIG_HASH,
            config_hash,
        )


def validate_mh(arguments: argparse.Namespace) -> dict[str, Any]:
    tolerance = arguments.comparison_tolerance_m
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise InputError("comparison tolerance must be finite and non-negative")
    if tolerance != MH_COMPARISON_TOLERANCE_M:
        raise InputError("comparison tolerance must equal 0.000001 m")
    control_dir = Path(arguments.control_dir)
    validate_mh_control_identity(control_dir)
    candidate = load_common_run(Path(arguments.run_dir))
    control = load_common_run(control_dir)
    manifest_path = Path(arguments.manifest_sha256)
    candidate_digest = verify_manifest(
        manifest_path,
        None,
        path_value(
            nested(candidate["meta"], "sequence_root"),
            "candidate.meta.sequence_root",
        ),
    )
    control_digest = verify_manifest(
        manifest_path,
        None,
        path_value(
            nested(control["meta"], "sequence_root"),
            "control.meta.sequence_root",
        ),
    )
    if candidate_digest != control_digest:
        raise InputError("candidate/control manifest digests differ")

    expected_sequence = "MH_01_easy"
    for label, run in (("candidate", candidate), ("control", control)):
        sequence = run["identity"]["sequence"]
        if sequence != expected_sequence:
            fail(f"{label} sequence", expected_sequence, sequence)
    for key in ("config", "canonical_text", "config_hash"):
        if candidate["identity"][key] != control["identity"][key]:
            fail(
                f"candidate/control {key} identity",
                control["identity"][key],
                candidate["identity"][key],
            )
    expected_config: dict[str, object] = {
        "estimator.enable_moving_bootstrap": True,
        "eval.max_dt_ms": 2.5,
        "eval.min_match_rate": 0.5,
        "eval.rpe_delta_s": 1.0,
    }
    for key, expected in expected_config.items():
        config = candidate["identity"]["config"]
        if key not in config:
            raise InputError(f"candidate config key is missing: {key}")
        actual = config[key]
        if isinstance(expected, bool):
            actual = boolean(actual, f"candidate.config.{key}")
        else:
            actual = finite_number(actual, f"candidate.config.{key}")
        if actual != expected:
            fail(f"candidate.config.{key}", expected, actual)

    candidate_first_ok = candidate["first_ok"]
    if candidate_first_ok != MH_CANDIDATE_FIRST_OK_INDEX:
        fail(
            "candidate first ok index",
            MH_CANDIDATE_FIRST_OK_INDEX,
            candidate_first_ok,
        )
    candidate_diag = candidate["diag"]
    candidate_span_ns = (
        candidate_diag[candidate_first_ok].timestamp_ns
        - candidate_diag[0].timestamp_ns
    )
    if candidate_span_ns != MH_CANDIDATE_FIRST_OK_SPAN_NS:
        fail(
            "candidate first ok span ns",
            MH_CANDIDATE_FIRST_OK_SPAN_NS,
            candidate_span_ns,
        )

    control_first_ok = control["first_ok"]
    control_diag = control["diag"]
    control_span_ns = (
        control_diag[control_first_ok].timestamp_ns
        - control_diag[0].timestamp_ns
    )
    for metric in ("ate", "rpe"):
        actual = candidate["metrics"][metric]
        limit = control["metrics"][metric] + tolerance
        if actual > limit:
            fail(f"candidate {metric}", f"<={limit}", actual)

    return {
        "mode": "mh01-static",
        "passed": True,
        "manifest_digest": candidate_digest,
        "candidate": candidate["metrics"],
        "control": control["metrics"],
        "candidate_first_ok_index": candidate_first_ok,
        "candidate_first_ok_span_ns": candidate_span_ns,
        "control_first_ok_index": control_first_ok,
        "control_first_ok_span_ns": control_span_ns,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="mode", required=True)
    v1 = subparsers.add_parser("v102-118")
    v1.add_argument("run_dir")
    v1.add_argument("--manifest-sha256", required=True)
    v1.add_argument("--expected-manifest-digest", required=True)
    mh = subparsers.add_parser("mh01-static")
    mh.add_argument("run_dir")
    mh.add_argument("--control-dir", required=True)
    mh.add_argument("--comparison-tolerance-m", required=True, type=float)
    mh.add_argument("--manifest-sha256", required=True)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        if arguments.mode == "v102-118":
            verdict = validate_v1(arguments)
        else:
            verdict = validate_mh(arguments)
    except InputError as error:
        print(json.dumps({"passed": False, "error": str(error)}), file=sys.stderr)
        return 2
    except PredicateFailure as error:
        print(json.dumps({"passed": False, "difference": str(error)}), file=sys.stderr)
        return 1
    print(json.dumps(verdict, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

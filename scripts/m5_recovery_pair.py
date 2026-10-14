#!/usr/bin/env python3
"""#58: local preparation and explicit C -> verify -> S remote entry points.

prepare never opens SSH or builds/runs a benchmark. start/wait are for a later
authorized execution. A completed pair still requires independent J/ATE review.
"""

from __future__ import annotations

import argparse
import csv
from decimal import Decimal
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


SEQUENCE = "V2_03_difficult"
PAIRS = ((433, 459), (701, 809), (929, 937), (1010, 1097), (1157, 1343), (1521, 1563))
OUTAGES = (434, 702, 930, 1011, 1158, 1522, 1631)
PAYLOADS = ("diag.csv", "est.tum", "kf.tum")
LOG_NAME = "recovery-bridge.jsonl"


class PairError(RuntimeError):
    def __init__(self, field: str, expected: Any, actual: Any):
        self.detail = {"field": field, "expected": expected, "actual": actual}
        super().__init__(json.dumps(self.detail, ensure_ascii=False))


def require_equal(field: str, actual: Any, expected: Any) -> None:
    if actual != expected:
        raise PairError(field, expected, actual)


def read_json(path: Path) -> Any:
    return json.loads(path.read_text())


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def load_native(path: Path) -> Any:
    spec = importlib.util.spec_from_file_location("phad_recovery_native", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sequence_script() -> str:
    return f"""set -euo pipefail
umask 022
/shared-build/phad_vo_bench /datasets/euroc/{SEQUENCE} \\
  --out /task/artifacts/bench \\
  --sequence-name {SEQUENCE} --repo /src \\
  --estimator-enable-moving-bootstrap \\
  --recovery-bridge-log /task/artifacts/bench/{LOG_NAME}
"""


def run_tasks(native: Any, run: dict[str, Any]) -> None:
    require_equal("sequences", run["sequences"], [SEQUENCE])
    build_root = Path(run["run_root"]) / "tasks/build/build"
    build = native._run_remote_task(run, "build", "build", native._build_script(run),
                                    cpus=16, memory="32g")
    if build["state"] != "passed":
        native._write_blocked_task(run, "unit", "unit", "build failed")
        native._write_blocked_task(run, SEQUENCE, "euroc", "build failed")
        return
    unit = native._run_remote_task(
        run, "unit", "unit", native._unit_script(run), cpus=8, memory="16g",
        extra_mounts=((build_root, "/shared-build", True),))
    if unit["state"] != "passed":
        native._write_blocked_task(run, SEQUENCE, "euroc", "unit failed")
        return
    native._verify_run_source_identity(run)
    native._run_remote_task(
        run, SEQUENCE, "euroc", sequence_script(), cpus=3, memory="8g",
        extra_mounts=((build_root, "/shared-build", True),
                      (Path(run["dataset_root"]), "/datasets/euroc", True)))


def worker_impl(native: Any, run_root: Path) -> int:
    run = read_json(run_root / "run.json")
    require_equal("run_root", str(run_root.resolve()), str(Path(run["run_root"]).resolve()))
    native._verify_run_source_identity(run)
    verify_dataset(native, Path(run["dataset_root"]) / SEQUENCE,
                   run["m5_recovery_pair"]["dataset_manifest"])
    status_path = run_root / "run-status.json"
    status = {"schema_version": 1, "run_id": run["run_id"], "profile": run["profile"],
              "state": "running", "started_utc": native.utc_now(),
              "updated_utc": native.utc_now(), "finished_utc": None}
    native.atomic_write_json(status_path, status)
    run_tasks(native, run)
    tasks = native._collect_task_statuses(run_root)
    terminal = native.aggregate_run_state(tasks)
    status.update(state="finalizing", tasks=tasks, updated_utc=native.utc_now())
    native.atomic_write_json(status_path, status)
    _, checksum = native._create_results_archive(run_root)
    status.update(state=terminal, finished_utc=native.utc_now(),
                  results_archive_sha256=checksum)
    native.atomic_write_json(status_path, status)
    return 0 if terminal in {"passed", "warning"} else 1


def validate_row_schema(row: dict, index: int) -> None:
    base = f"row[{index}]"

    def fields(value: Any, names: str, path: str) -> None:
        if not isinstance(value, dict) or set(value) != set(names.split()):
            raise PairError(path, names.split(), list(value) if isinstance(value, dict) else value)

    def integer(value: Any, path: str, upper: int, optional: bool = False, lower: int = 0) -> None:
        if optional and value is None:
            return
        if type(value) is not int or not lower <= value <= upper:
            raise PairError(path, f"integer [{lower}, {upper}]" + (" or null" if optional else ""), value)

    def member(value: Any, choices: list, path: str) -> None:
        if type(value) is not type(choices[0]) or value not in choices:
            raise PairError(path, choices, value)

    fields(row, "schema_version i ts_ns status segment_id frontend initialization bridge", base)
    integer(row["schema_version"], base + ".schema_version", 1, lower=1)
    integer(row["i"], base + ".i", 2**64 - 1)
    integer(row["ts_ns"], base + ".ts_ns", 2**63 - 1, lower=-2**63)
    integer(row["segment_id"], base + ".segment_id", 2**32 - 1)
    member(row["status"], ["ok", "initializing", "visual_outage", "failed", "invalid_input", "rejected"], base + ".status")
    init, bridge, frontend = row["initialization"], row["bridge"], row["frontend"]
    fields(init, "path phase reason outcome attempt_id current_graph transaction", base + ".initialization")
    fields(bridge, "origin result reason lineage anchor_ts_ns root_ts_ns attempt_id transaction", base + ".bridge")
    fields(frontend, "is_keyframe observations", base + ".frontend")
    member(frontend["is_keyframe"], [True, False], base + ".frontend.is_keyframe")
    if not isinstance(frontend["observations"], list):
        raise PairError(base + ".frontend.observations", "ordered list", frontend["observations"])
    for n, observation in enumerate(frontend["observations"]):
        path = base + f".frontend.observations[{n}]"
        if not isinstance(observation, list) or len(observation) != 4:
            raise PairError(path, "[id,u,v,disparity]", observation)
        integer(observation[0], path + ".id", 2**64 - 1)
        for value in observation[1:]:
            if type(value) in (int, float) and math.isfinite(value):
                continue
            fields(value, "f64_bits", path)
            bits = value["f64_bits"]
            if (not isinstance(bits, str) or len(bits) != 16 or
                    any(c not in "0123456789abcdef" for c in bits) or
                    int(bits, 16) & 0x7ff0000000000000 != 0x7ff0000000000000):
                raise PairError(path, "nonfinite f64_bits: fixed 16 lowercase hex", value)
    member(init["path"], ["None", "Static", "Moving"], base + ".initialization.path")
    member(init["phase"], list(range(8)), base + ".initialization.phase")
    member(init["reason"], list(range(18)), base + ".initialization.reason")
    member(init["outcome"], ["NotActive", "Pending", "RecoverableRejected", "Committed", "FatalError"], base + ".initialization.outcome")
    member(init["current_graph"], [0, 1, 2], base + ".initialization.current_graph")
    member(bridge["origin"], ["None", "ProcessStart", "VisualOutageRecovery", "MeasurementDiscontinuity"], base + ".bridge.origin")
    member(bridge["result"], ["NotApplicable", "Pending", "Eligible", "Applied", "Cleared", "FatalError"], base + ".bridge.result")
    member(bridge["reason"], ["None", "AwaitingValidatedRoot", "StaticRootSelected", "MeasurementDiscontinuity", "TimestampMismatch", "InvalidAnchor", "YawUndefined", "InvalidTransform", "ContractError", "AnchorExpired"], base + ".bridge.reason")
    for name, value in (("initialization", init), ("bridge", bridge)):
        integer(value["attempt_id"], base + f".{name}.attempt_id", 2**64 - 1, optional=True)
        member(value["transaction"], ["NotEvaluated", "EvidenceCommitted", "RootCommitted", "RolledBack"], base + f".{name}.transaction")
    for name in ("anchor_ts_ns", "root_ts_ns"):
        integer(bridge[name], base + ".bridge." + name, 2**63 - 1, optional=True, lower=-2**63)
    lineage = bridge["lineage"]
    if lineage is not None:
        fields(lineage, "source_segment_id source_frame_index source_ts_ns target_segment_id world_origin_segment_id", base + ".bridge.lineage")
        for name in ("source_segment_id", "target_segment_id", "world_origin_segment_id"):
            integer(lineage[name], base + ".bridge.lineage." + name, 2**32 - 1)
        integer(lineage["source_frame_index"], base + ".bridge.lineage.source_frame_index", 2**64 - 1)
        integer(lineage["source_ts_ns"], base + ".bridge.lineage.source_ts_ns", 2**63 - 1, lower=-2**63)


def validate_lifecycle(rows: list[dict], role: str, horizon_ns: int = 500_000_000) -> None:
    # Consume typed lineage. Internal estimator frame indices are opaque here:
    # check stability throughout one anchor, never equate them with processed i.
    world = None
    anchor = None
    for i, row in enumerate(rows):
        validate_row_schema(row, i)
        init, bridge = row["initialization"], row["bridge"]
        lineage = bridge["lineage"]
        def check(name: str, actual: Any, expected: Any) -> None:
            require_equal(f"row[{i}].{name}", actual, expected)
        check("initialization.committed_at_root", init["outcome"] == "Committed",
              i in (238, *(root for _, root in PAIRS)))
        if row["status"] == "visual_outage":
            if i == 0 or rows[i - 1]["status"] != "ok" or world is None or lineage is None:
                raise PairError(f"row[{i}].bridge.lineage", "anchor from latest committed navigation", lineage)
            check("bridge.lineage.source_segment_id", lineage["source_segment_id"], rows[i - 1]["segment_id"])
            check("bridge.lineage.source_ts_ns", lineage["source_ts_ns"], rows[i - 1]["ts_ns"])
            check("bridge.lineage.target_segment_id", lineage["target_segment_id"], row["segment_id"])
            check("segment_id", row["segment_id"], rows[i - 1]["segment_id"] + 1)
            anchor = dict(lineage, world_origin_segment_id=world)
        if anchor is not None:
            if lineage is None:
                raise PairError(f"row[{i}].bridge.lineage", "complete anchor lineage", None)
            for key in ("source_segment_id", "source_frame_index", "source_ts_ns", "target_segment_id"):
                check("bridge.lineage." + key, lineage[key], anchor[key])
            check("segment_id", row["segment_id"], anchor["target_segment_id"])
            target_world = (row["segment_id"] if init["transaction"] == "RootCommitted" and
                            bridge["result"] != "Applied" else anchor["world_origin_segment_id"])
            check("bridge.lineage.world_origin_segment_id", lineage["world_origin_segment_id"], target_world)
        else:
            check("bridge.lineage", lineage, None)
            check("bridge.anchor_ts_ns", bridge["anchor_ts_ns"], None)
        if bridge["result"] in ("Cleared", "FatalError") or row["status"] in ("failed", "invalid_input", "rejected"):
            # These events stop this fixed V2_03 matrix; never turn them into eligibility.
            if bridge["result"] == "Cleared":
                check("bridge.origin", bridge["origin"], "MeasurementDiscontinuity")
                check("bridge.reason", bridge["reason"], "MeasurementDiscontinuity")
                check("bridge.transaction", bridge["transaction"], "EvidenceCommitted")
            if bridge["result"] == "FatalError":
                check("bridge.transaction", bridge["transaction"], "RolledBack")
            if row["status"] == "invalid_input":
                check("bridge.transaction", bridge["transaction"], "NotEvaluated")
            raise PairError(f"row[{i}].lifecycle", "unchanged accepted V2_03 cadence", bridge)
        check("bridge.origin", bridge["origin"], "VisualOutageRecovery" if anchor else
              "ProcessStart" if world is None else "None")
        if init["outcome"] == "Committed":
            for name, expected in (("phase", 7), ("reason", 0), ("current_graph", 1), ("transaction", "RootCommitted")):
                check("initialization." + name, init[name], expected)
            check("status", row["status"], "ok")
            if init["path"] not in ("Moving", "Static"):
                raise PairError(f"row[{i}].initialization.path", "Moving/Static", init["path"])
            moving = init["path"] == "Moving"
            if moving and init["attempt_id"] is None:
                raise PairError(f"row[{i}].initialization.attempt_id", "present", None)
            expired = bool(moving and anchor and role == "subject" and
                           row["ts_ns"] - anchor["source_ts_ns"] > horizon_ns)
            result = ("Eligible" if role == "control" else "NotApplicable" if expired else "Applied") if moving and anchor else "NotApplicable"
            check("bridge.result", bridge["result"], result)
            check("bridge.reason", bridge["reason"], "AnchorExpired" if expired else "None" if moving else "StaticRootSelected")
            check("bridge.root_ts_ns", bridge["root_ts_ns"], row["ts_ns"] if anchor or not moving else None)
            check("bridge.transaction", bridge["transaction"], "RootCommitted")
            check("bridge.attempt_id", bridge["attempt_id"], init["attempt_id"])
            if anchor:
                check("bridge.anchor_ts_ns", bridge["anchor_ts_ns"], row["ts_ns"])
            world = anchor["world_origin_segment_id"] if result == "Applied" else row["segment_id"]
            anchor = None
        elif anchor is not None:
            if init["outcome"] == "Pending":
                if init["path"] == "Static":
                    check("initialization.phase", init["phase"], 1)
                    check("initialization.reason", init["reason"], 1)
                    check("initialization.attempt_id", init["attempt_id"], None)
                else:
                    check("initialization.path", init["path"], "Moving")
                    expected_phase = (1 if init["reason"] in (3, 4) else
                                      2 if init["reason"] in (5, 6, 7, 8, 9) else None)
                    if expected_phase is None:
                        raise PairError(f"row[{i}].initialization.reason", "pending evidence reason", init["reason"])
                    check("initialization.phase", init["phase"], expected_phase)
                check("initialization.current_graph", init["current_graph"], 0)
            elif init["outcome"] == "RecoverableRejected":
                expected_phases = ((3, 4) if init["reason"] in (10, 11) else
                                   (5,) if init["reason"] in (12, 13, 14) else
                                   (6,) if init["reason"] in (15, 16) else None)
                if expected_phases is None:
                    raise PairError(f"row[{i}].initialization.reason", "recoverable rejection reason", init["reason"])
                if init["phase"] not in expected_phases:
                    raise PairError(f"row[{i}].initialization.phase", expected_phases, init["phase"])
                check("initialization.current_graph", init["current_graph"], 2 if init["reason"] in (15, 16) else 0)
            check("bridge.result", bridge["result"], "Pending")
            check("bridge.reason", bridge["reason"], "AwaitingValidatedRoot")
            check("bridge.anchor_ts_ns", bridge["anchor_ts_ns"], row["ts_ns"])
            check("bridge.root_ts_ns", bridge["root_ts_ns"], None)
            check("bridge.transaction", bridge["transaction"], "EvidenceCommitted")
            check("bridge.attempt_id", bridge["attempt_id"], init["attempt_id"])
            if row["status"] != "visual_outage":
                if init["outcome"] not in ("Pending", "RecoverableRejected"):
                    raise PairError(f"row[{i}].initialization.outcome", "Pending/RecoverableRejected", init["outcome"])
                check("initialization.transaction", init["transaction"], "EvidenceCommitted")
        else:
            check("bridge.result", bridge["result"], "NotApplicable")
            check("bridge.reason", bridge["reason"], "None")
            check("bridge.root_ts_ns", bridge["root_ts_ns"], None)
            check("bridge.attempt_id", bridge["attempt_id"], init["attempt_id"])
            check("bridge.transaction", bridge["transaction"], init["transaction"])
            if world is not None:
                check("bridge.attempt_id", bridge["attempt_id"], None)
                check("initialization.outcome", init["outcome"], "NotActive")
                check("bridge.transaction", bridge["transaction"], "NotEvaluated")


def control_eligible(rows: list[dict[str, Any]]) -> list[int]:
    validate_lifecycle(rows, "control")
    eligible = []
    for _, index in PAIRS:
        row = rows[index]
        init, bridge = row["initialization"], row["bridge"]
        for key, expected in (("outcome", "Committed"), ("transaction", "RootCommitted")):
            require_equal(f"row[{index}].initialization.{key}", init[key], expected)
        if init["path"] == "Static":
            require_equal(f"row[{index}].bridge.result", bridge["result"], "NotApplicable")
            require_equal(f"row[{index}].bridge.reason", bridge["reason"], "StaticRootSelected")
            continue
        require_equal(f"row[{index}].initialization.path", init["path"], "Moving")
        for key, expected in (("origin", "VisualOutageRecovery"), ("result", "Eligible"),
                              ("reason", "None"), ("transaction", "RootCommitted"),
                              ("anchor_ts_ns", row["ts_ns"]), ("root_ts_ns", row["ts_ns"]),
                              ("attempt_id", init["attempt_id"])):
            require_equal(f"row[{index}].bridge.{key}", bridge[key], expected)
        if init["attempt_id"] is None or bridge["lineage"] is None:
            raise PairError(f"row[{index}].identity", "attempt and lineage present", bridge)
        require_equal(f"row[{index}].target_segment_id",
                      bridge["lineage"]["target_segment_id"], row["segment_id"])
        eligible.append(index)
    if not eligible:
        raise PairError("eligible_count", "> 0", 0)
    return eligible


def load_sidecar(bench: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in (bench / LOG_NAME).read_text().splitlines()]
    with (bench / "diag.csv").open() as stream:
        reader = csv.DictReader(stream)
        require_equal("diag.columns", len(reader.fieldnames), 26)
        diag = list(reader)
    require_equal("processed.rows", len(rows), 1921)
    require_equal("diag.rows", len(diag), len(rows))
    for index, (row, diagnostic) in enumerate(zip(rows, diag)):
        validate_row_schema(row, index)
        for field, expected in (("schema_version", 1), ("i", index),
                                ("ts_ns", int(diagnostic["timestamp_ns"])),
                                ("status", diagnostic["status"]),
                                ("segment_id", int(diagnostic["segment_id"]))):
            require_equal(f"row[{index}].{field}", row[field], expected)
        if type(row["ts_ns"]) is not int or type(row["i"]) is not int:
            raise PairError(f"row[{index}].identity", "integer index/timestamp", row)
        require_equal(f"row[{index}].frontend.is_keyframe",
                      row["frontend"]["is_keyframe"], diagnostic["is_keyframe"] == "1")
    require_equal("outage.indices", [i for i, r in enumerate(rows) if r["status"] == "visual_outage"],
                  list(OUTAGES))
    roots = [i for i, r in enumerate(rows) if r["status"] == "ok" and
             (i == 0 or rows[i - 1]["status"] != "ok")]
    require_equal("root.indices", roots, [238, *(b for _, b in PAIRS)])
    return rows


def verify_run(plan: dict[str, Any], role: str, fetched: Path, validate: bool = True) -> tuple[Path, list]:
    remote = fetched / "remote"
    run, status = read_json(remote / "run.json"), read_json(remote / "run-status.json")
    require_equal("run.source_id", run["source_id"], plan[role]["source_id"])
    require_equal("run.sequences", run["sequences"], [SEQUENCE])
    require_equal("run.pair", run["m5_recovery_pair"], plan)
    require_equal("tasks", sorted(status["tasks"]), sorted(["build", "unit", SEQUENCE]))
    for task in ("build", "unit", SEQUENCE):
        result = status["tasks"][task]
        require_equal(f"{task}.return_code", result["return_code"], 0)
        if result["state"] not in {"passed", "warning"}:
            raise PairError(f"{task}.state", "passed/warning", result["state"])
    bench = remote / "tasks" / SEQUENCE / "artifacts/bench"
    meta, summary = read_json(bench / "meta.json"), read_json(bench / "summary.json")
    for name in ("config", "config_hash", "config_canonical_text"):
        require_equal(f"meta.{name}", meta[name], plan["config"][name])
    for name in ("failed", "rejected"):
        require_equal(f"summary.{name}", summary["robustness"][name], 0)
    require_equal("summary.config_hash", summary["config_hash"], meta["config_hash"])
    for family in ("ate", "rpe"):
        value = summary[family]["trans"]["rmse"]
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            raise PairError(f"summary.{family}", "finite translation RMSE", value)
    rows = load_sidecar(bench)
    if validate:
        validate_lifecycle(rows, role, meta["config"]["estimator.visual_coast_horizon_ns"])
    return bench, rows


def verify_control(plan: dict[str, Any], fetched: Path) -> list[int]:
    bench, rows = verify_run(plan, "control", fetched)
    for filename in PAYLOADS:
        reference = Path(plan["baseline"]) / filename
        if (bench / filename).read_bytes() != reference.read_bytes():
            raise PairError(f"control.{filename}", str(reference), "payload differs")
    return control_eligible(rows)


def first_payload_difference(control: Path, subject: Path, rows: list, stop: int) -> tuple:
    differences = []
    endpoint = rows[stop - 1]["ts_ns"]
    for filename in PAYLOADS:
        def prefix(bench: Path) -> list[bytes]:
            lines = (bench / filename).read_bytes().splitlines(keepends=True)
            if stop == len(rows):
                return lines
            if filename == "diag.csv":
                return lines[:stop + 1]  # header plus rows strictly before first Applied
            return [line for line in lines if not line.strip() or line.startswith(b"#") or
                    Decimal(line.split()[0].decode()) * 1000000000 <= endpoint + 500]
        before, after = prefix(control), prefix(subject)
        for line in range(max(len(before), len(after))):
            expected = before[line] if line < len(before) else b""
            actual = after[line] if line < len(after) else b""
            if expected == actual:
                continue
            if filename == "diag.csv":
                index = line - 1
            else:
                times = [Decimal(value.split()[0].decode()) * 1000000000
                         for value in (expected, actual) if value.strip() and not value.startswith(b"#")]
                timestamp = min(times) if times else rows[0]["ts_ns"]
                index = next((r["i"] for r in rows if r["ts_ns"] + 500 >= timestamp), stop)
            error = PairError(f"pre-applied.{filename}.line[{line + 1}]", expected.decode(), actual.decode())
            error.detail.update(processed_index=index, ts_ns=rows[index]["ts_ns"] if 0 <= index < len(rows) else None)
            differences.append((index, error))
            break
    return min(differences, key=lambda item: item[0]) if differences else (len(rows), None)


def verify_pair(plan: dict[str, Any], control: Path, subject: Path) -> dict[str, Any]:
    eligible = verify_control(plan, control)
    control_bench, control_rows = verify_run(plan, "control", control)
    subject_bench, subject_rows = verify_run(plan, "subject", subject, validate=False)
    control_run = read_json(control / "remote/run.json")
    subject_run = read_json(subject / "remote/run.json")
    for key in ("oci_manifest_id", "oci_config_id", "toolchain_id"):
        require_equal(f"image.{key}", subject_run["image"][key], control_run["image"][key])
    horizon_ns = plan["config"]["config"]["estimator.visual_coast_horizon_ns"]
    applied = [i for i in eligible if control_rows[i]["ts_ns"] -
               control_rows[i]["bridge"]["lineage"]["source_ts_ns"] <= horizon_ns]
    expired = [i for i in eligible if i not in applied]
    difference_index, difference = first_payload_difference(
        control_bench, subject_bench, control_rows, min(applied) if applied else len(control_rows))
    try:
        validate_lifecycle(subject_rows, "subject", horizon_ns)
    except PairError as error:
        # Lifecycle failures use row[i] fields; compare their position with
        # payload and ingestion differences instead of their check order.
        lifecycle_index = int(error.detail["field"].split("]", 1)[0].removeprefix("row["))
        if lifecycle_index < difference_index:
            difference_index, difference = lifecycle_index, error
    for i, (before, after) in enumerate(zip(control_rows, subject_rows)):
        if difference is not None and difference_index <= i:
            raise difference
        for key in ("i", "ts_ns", "status", "segment_id", "initialization"):
            require_equal(f"row[{i}].{key}", after[key], before[key])
        # JSON numeric spelling retains signed zero as well as ordered content.
        require_equal(f"row[{i}].frontend",
                      json.dumps(after["frontend"], sort_keys=True, allow_nan=False),
                      json.dumps(before["frontend"], sort_keys=True, allow_nan=False))
        c_lineage, s_lineage = before["bridge"]["lineage"], after["bridge"]["lineage"]
        require_equal(f"row[{i}].bridge.lineage.present", s_lineage is not None, c_lineage is not None)
        if c_lineage is not None:
            for key in ("source_segment_id", "source_frame_index", "source_ts_ns", "target_segment_id"):
                require_equal(f"row[{i}].bridge.lineage.{key}", s_lineage[key], c_lineage[key])
        if i in eligible:
            bridge = after["bridge"]
            for key, expected in (("result", "NotApplicable" if i in expired else "Applied"),
                                  ("reason", "AnchorExpired" if i in expired else "None"),
                                  ("transaction", "RootCommitted"),
                                  ("origin", "VisualOutageRecovery"),
                                  ("anchor_ts_ns", after["ts_ns"]),
                                  ("root_ts_ns", after["ts_ns"]),
                                  ("attempt_id", after["initialization"]["attempt_id"])):
                require_equal(f"row[{i}].bridge.{key}", bridge[key], expected)
            if bridge["lineage"] is None:
                raise PairError(f"row[{i}].bridge.lineage", "present", None)
            for key in ("source_segment_id", "source_frame_index", "source_ts_ns", "target_segment_id"):
                require_equal(f"row[{i}].bridge.lineage.{key}",
                              bridge["lineage"][key], before["bridge"]["lineage"][key])
        elif after["bridge"]["result"] == "Applied":
            raise PairError(f"row[{i}].bridge.result", "no application outside eligible roots", "Applied")
        if before["initialization"]["path"] == "Static" and before["status"] == "ok":
            require_equal(f"row[{i}].bridge.result", after["bridge"]["result"], "NotApplicable")
            require_equal(f"row[{i}].bridge.reason", after["bridge"]["reason"],
                          before["bridge"]["reason"])
    if difference is not None:
        raise difference
    c_summary = read_json(control_bench / "summary.json")
    s_summary = read_json(subject_bench / "summary.json")
    for key in ("completion_rate", "coverage_rate"):
        actual = s_summary["trajectory"][key]
        expected = c_summary["trajectory"][key]
        if not math.isfinite(actual) or actual < expected:
            raise PairError(key, f">= {expected}", actual)
    return {"status": "execution-complete", "eligible_indices": eligible,
            "applied_indices": applied, "expired_indices": expired,
            "fixed_endpoint_pairs": PAIRS, "product_verdict": "pending independent J/ATE evaluation"}


def verify_dataset(native: Any, root: Path, manifest: dict[str, str]) -> None:
    actual_files = {p.relative_to(root).as_posix(): p for p in root.rglob("*") if p.is_file()}
    require_equal("dataset.files", sorted(actual_files), sorted(manifest))
    for relative, digest in manifest.items():
        require_equal(f"dataset.{relative}", native.sha256_file(actual_files[relative]), digest)


def prepare(args: argparse.Namespace, native: Any) -> dict[str, Any]:
    roots = {role: Path(getattr(args, role)).resolve() for role in ("control", "subject")}
    if roots["control"] == roots["subject"]:
        raise PairError("source roots", "two isolated directories", str(roots["control"]))
    plan = {"schema_version": 1, "sequence": SEQUENCE, "remote_root": str(args.remote_root),
            "local_root": str(args.output.parent.resolve()), "baseline": str(args.baseline.resolve())}
    maps = {}
    for role, root in roots.items():
        manifest, source_id = native.build_source_manifest(root)
        maps[role] = {entry["path"]: entry for entry in manifest["entries"]}
        plan[role] = {"root": str(root), "source_id": source_id}
    differences = sorted(path for path in maps["control"].keys() | maps["subject"].keys()
                         if maps["control"].get(path) != maps["subject"].get(path))
    require_equal("source delta paths", differences, sorted(args.activation_file))
    if not differences:
        raise PairError("source delta", "bridge activation difference", [])
    plan["activation_delta"] = [{"path": path, "control": maps["control"].get(path),
                                 "subject": maps["subject"].get(path)} for path in differences]
    meta = read_json(args.baseline / "meta.json")
    plan["config"] = {key: meta[key] for key in ("config", "config_hash", "config_canonical_text")}
    for key, expected in (("estimator.visual_coast_horizon_ns", 500000000),
                          ("estimator.enable_moving_bootstrap", True)):
        require_equal(key, meta["config"][key], expected)
    plan["dataset_root"] = str(args.dataset_root)
    dataset_manifest = {}
    for line in args.dataset_manifest.read_text().splitlines():
        digest, name = line.split(maxsplit=1)
        name = name.removeprefix("*").removeprefix("./")
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest) or Path(name).is_absolute() or ".." in Path(name).parts or name in dataset_manifest:
            raise PairError("dataset manifest", "unique relative sha256sum records", line)
        dataset_manifest[name] = digest
    if not dataset_manifest:
        raise PairError("dataset manifest", "nonempty", {})
    plan["dataset_manifest"] = dataset_manifest
    if not args.remote_root.is_absolute() or args.remote_root == native.REMOTE_ROOT:
        raise PairError("remote_root", "explicit isolated absolute directory", str(args.remote_root))
    if args.output.exists():
        raise PairError("plan", "new output file", str(args.output))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, plan)
    return plan


def configure_native(native: Any, plan: dict[str, Any]) -> None:
    native.REMOTE_ROOT = Path(plan["remote_root"])
    native.LOCAL_ARTIFACT_ROOT = Path(plan["local_root"]) / "raw"
    original_definition = native.profile_definition

    def definition(profile: str) -> dict[str, Any]:
        value = original_definition(profile)
        if profile == native.PROFILE:
            value.update(sequences=[SEQUENCE], dataset_root=plan["dataset_root"],
                         max_parallel_tasks=1)
        return value

    native.profile_definition = definition


def start_role(native: Any, plan: dict[str, Any], role: str,
               control_run: Path | None = None) -> dict[str, Any]:
    control_identity = None
    if role == "subject":
        if control_run is None:
            raise PairError("control_run", "verified C1 before S1", None)
        verify_control(plan, control_run)
        control_identity = read_json(control_run / "remote/run.json")
    receipt = Path(plan["local_root"]) / f"{role}-start.json"
    if receipt.exists():
        raise PairError(f"{role} starts", "exactly one start; inspect existing receipt", str(receipt))
    configure_native(native, plan)

    def prepare_run_config(run: dict[str, Any]) -> None:
        require_equal(f"{role}.source_id", run["source_id"], plan[role]["source_id"])
        require_equal("sequences", run["sequences"], [SEQUENCE])
        if control_identity:
            for key in ("oci_manifest_id", "oci_config_id", "toolchain_id"):
                require_equal(f"image.{key}", run["image"][key], control_identity["image"][key])
        run.update(unit_parallelism=1, max_parallel_tasks=1,
                   m5_recovery_pair=plan, m5_recovery_role=role)

    original_launch = native.launch_remote_worker

    def launch(ssh: Any, run_root: Path, remote_script: Path) -> dict[str, Any]:
        # Run the wrapper from the already verified role snapshot, not another checkout.
        wrapper = remote_script.with_name("m5_recovery_pair.py")
        return original_launch(ssh, run_root, wrapper)

    native.launch_remote_worker = launch
    result = native.start_remote_run(Path(plan[role]["root"]), native.PROFILE,
                                     prepare_run_config=prepare_run_config)
    write_json(receipt, result)
    return result


def wait_and_fetch(native: Any, plan: dict[str, Any], role: str) -> Path:
    configure_native(native, plan)
    run_id = read_json(Path(plan["local_root"]) / f"{role}-start.json")["run_id"]
    previous = None
    while True:
        with native.VerifiedSsh() as ssh:
            status = native.remote_status(ssh, run_id)
        signature = native._status_signature(status)
        if signature != previous:
            native.print_status(status)
            previous = signature
        if status["state"] in native.TERMINAL_RUN_STATES:
            break
        time.sleep(30)
    summary = native.fetch_remote_run(Path(plan[role]["root"]), run_id)
    return summary.parent


def parser() -> argparse.ArgumentParser:
    command = argparse.ArgumentParser(description=__doc__)
    sub = command.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare", help="local only; freeze a reviewable plan")
    for name in ("control", "subject", "baseline", "remote-root", "dataset-root",
                 "dataset-manifest", "output"):
        prep.add_argument("--" + name, required=True, type=Path)
    prep.add_argument("--activation-file", action="append", required=True,
                      help="reviewed bridge activation delta path; all other source entries must match")
    for action in ("start-control", "start-subject", "wait-control", "wait-subject",
                   "check-control", "check-pair"):
        item = sub.add_parser(action)
        item.add_argument("--plan", required=True, type=Path)
        if action in {"start-subject", "check-control", "check-pair"}:
            item.add_argument("--control-run", required=True, type=Path)
        if action == "check-pair":
            item.add_argument("--subject-run", required=True, type=Path)
    worker = sub.add_parser("_remote-worker")
    worker.add_argument("--run-root", required=True, type=Path)
    return command


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "_remote-worker":
            run = read_json(args.run_root / "run.json")
            native = load_native(Path(run["source_root"]) / "scripts/remote_ci.py")
            native._remote_worker_impl = lambda root, **_: worker_impl(native, root)
            return native.remote_worker(args.run_root)
        native = load_native(Path(__file__).with_name("remote_ci.py"))
        if args.command == "prepare":
            result = prepare(args, native)
        else:
            plan = read_json(args.plan)
            if args.command.startswith("start-"):
                result = start_role(native, plan, args.command.removeprefix("start-"),
                                    getattr(args, "control_run", None))
            elif args.command.startswith("wait-"):
                result = {"fetched": str(wait_and_fetch(native, plan, args.command.removeprefix("wait-")))}
            elif args.command == "check-control":
                result = {"eligible_indices": verify_control(plan, args.control_run)}
            else:
                result = verify_pair(plan, args.control_run, args.subject_run)
        print(json.dumps(result, indent=2))
        return 0
    except (PairError, OSError, ValueError, KeyError, RuntimeError) as error:
        detail = error.detail if isinstance(error, PairError) else {"error": str(error)}
        if getattr(args, "plan", None):
            first = args.plan.parent / "first-failure.json"
            if not first.exists():
                write_json(first, detail)
        print(json.dumps(detail), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

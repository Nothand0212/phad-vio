import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
from unittest.mock import patch
import copy
import json
import csv
import hashlib
import subprocess


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "m5_recovery_pair.py"
spec = importlib.util.spec_from_file_location("m5_recovery_pair", SCRIPT)
pair = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pair)


class RecoveryPairTest(unittest.TestCase):
    def make_pair(self, root, moving_index=459):
        config = {"config": {"estimator.visual_coast_horizon_ns": 500_000_000},
                  "config_hash": "frozen", "config_canonical_text": "frozen"}
        plan = {"control": {"source_id": "c"}, "subject": {"source_id": "s"},
                "config": config, "baseline": str(root / "baseline")}
        for role in ("control", "subject"):
            remote = root / role / "remote"
            bench = remote / "tasks" / pair.SEQUENCE / "artifacts/bench"
            bench.mkdir(parents=True)
            pair.write_json(remote / "run.json", {"source_id": plan[role]["source_id"], "sequences": [pair.SEQUENCE],
                            "m5_recovery_pair": plan, "image": {"oci_manifest_id": "m", "oci_config_id": "c", "toolchain_id": "t"}})
            pair.write_json(remote / "run-status.json", {"tasks": {task: {"state": "passed", "return_code": 0}
                            for task in ("build", "unit", pair.SEQUENCE)}})
            pair.write_json(bench / "meta.json", config)
            pair.write_json(bench / "summary.json", {"config_hash": "frozen", "robustness": {"failed": 0, "rejected": 0},
                            "ate": {"trans": {"rmse": 1.0}}, "rpe": {"trans": {"rmse": 0.1}},
                            "trajectory": {"completion_rate": 0.49, "coverage_rate": 0.72}})
            rows = self.typed_rows((moving_index,), role)
            with (bench / "diag.csv").open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["timestamp_ns", "status", "segment_id", "is_keyframe", *[f"f{i}" for i in range(22)]])
                writer.writerows([[r["ts_ns"], r["status"], r["segment_id"], 1, *([0] * 22)] for r in rows])
            for filename in ("est.tum", "kf.tum"):
                (bench / filename).write_text("1400000000.000000000 0 0 0 0 0 0 1\n")
            payload = rows
            (bench / pair.LOG_NAME).write_text("".join(json.dumps(row) + "\n" for row in payload))
        baseline = Path(plan["baseline"])
        baseline.mkdir()
        cbench = root / "control/remote/tasks" / pair.SEQUENCE / "artifacts/bench"
        for filename in pair.PAYLOADS:
            (baseline / filename).write_bytes((cbench / filename).read_bytes())
        return plan

    def test_pair_keeps_static_endpoints_and_reports_product_pending(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self.make_pair(root)
            result = pair.verify_pair(plan, root / "control", root / "subject")
            self.assertEqual(result["eligible_indices"], [459])
            self.assertEqual(result["applied_indices"], [])
            self.assertEqual(result["expired_indices"], [459])
            self.assertEqual(len(result["fixed_endpoint_pairs"]), 6)
            self.assertEqual(result["status"], "execution-complete")
            self.assertNotIn("PASS", result["product_verdict"])

    def test_first_ordered_ingestion_difference_stops_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self.make_pair(root)
            path = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / pair.LOG_NAME
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[500]["frontend"]["observations"].reverse()
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            with self.assertRaises(pair.PairError) as caught:
                pair.verify_pair(plan, root / "control", root / "subject")
            self.assertEqual(caught.exception.detail["field"], "row[500].frontend")

    def test_pending_numeric_difference_before_first_applied_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self.make_pair(root)
            path = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench/diag.csv"
            lines = path.read_bytes().splitlines(keepends=True)
            lines[435] = lines[435].replace(b",0,0,", b",9,0,", 1)
            path.write_bytes(b"".join(lines))
            with self.assertRaises(pair.PairError) as caught:
                pair.verify_pair(plan, root / "control", root / "subject")
            self.assertEqual(caught.exception.detail["field"], "pre-applied.diag.csv.line[436]")

    def test_static_tum_pose_before_later_first_applied_stops(self):
        for filename in ("est.tum", "kf.tum"):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                plan = self.make_pair(root, moving_index=809)
                for role in ("control", "subject"):
                    path = root / role / "remote/tasks" / pair.SEQUENCE / "artifacts/bench" / filename
                    path.write_bytes(path.read_bytes() + b"1400000022.950000000 0 0 0 0 0 0 1\n")
                cpath = root / "control/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / filename
                (Path(plan["baseline"]) / filename).write_bytes(cpath.read_bytes())
                spath = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / filename
                spath.write_bytes(spath.read_bytes().replace(b"22.950000000 0", b"22.950000000 1"))
                with self.assertRaises(pair.PairError) as caught:
                    pair.verify_pair(plan, root / "control", root / "subject")
                self.assertEqual(caught.exception.detail["field"], f"pre-applied.{filename}.line[2]")

    def test_unit_failure_prevents_sequence(self):
        native = Mock()
        native._build_script.return_value = "build-command"
        native._unit_script.return_value = "unit-command"
        native._run_remote_task.side_effect = [
            {"state": "passed"}, {"state": "failed"}
        ]
        run = {"run_root": "/isolated/runs/control", "dataset_root": "/datasets",
               "sequences": [pair.SEQUENCE]}
        pair.run_tasks(native, run)
        self.assertEqual([call.args[1] for call in native._run_remote_task.call_args_list],
                         ["build", "unit"])
        self.assertEqual(native._write_blocked_task.call_args.args[1], pair.SEQUENCE)

    def test_worker_runs_only_one_sequence_after_unit(self):
        native = Mock()
        native._run_remote_task.return_value = {"state": "passed"}
        run = {"run_root": "/isolated/runs/subject", "dataset_root": "/datasets",
               "sequences": [pair.SEQUENCE]}
        pair.run_tasks(native, run)
        calls = native._run_remote_task.call_args_list
        self.assertEqual([call.args[1] for call in calls], ["build", "unit", pair.SEQUENCE])
        self.assertIn("--recovery-bridge-log", calls[2].args[3])
        self.assertEqual(calls[2].args[3].count("/shared-build/phad_vo_bench"), 1)
        self.assertNotIn("--max-frames", calls[2].args[3])
        self.assertTrue(all(mount[2] for mount in calls[2].kwargs["extra_mounts"]))

    def test_extra_sequence_is_rejected_before_build(self):
        native = Mock()
        with self.assertRaises(pair.PairError):
            pair.run_tasks(native, {"sequences": [pair.SEQUENCE, "MH_01_easy"]})
        native._run_remote_task.assert_not_called()

    @staticmethod
    def typed_rows(moving=(459,), role="control"):
        rows = []
        starts = [238, *(b for _, b in pair.PAIRS)]
        world = 0
        anchor = None
        for i in range(1921):
            segment = sum(i >= outage for outage in pair.OUTAGES)
            ok = segment < len(starts) and starts[segment] <= i < pair.OUTAGES[segment]
            ts = 1400000000000000000 + i * 50000000
            init = {"path": "None", "phase": 0, "reason": 0, "outcome": "NotActive",
                    "attempt_id": None, "current_graph": 0, "transaction": "NotEvaluated"}
            bridge = {"origin": "None" if ok else "ProcessStart", "result": "NotApplicable",
                      "reason": "None", "lineage": None, "anchor_ts_ns": None,
                      "root_ts_ns": None, "attempt_id": None, "transaction": "NotEvaluated"}
            if i in pair.OUTAGES:
                anchor = {"source_segment_id": segment - 1, "source_frame_index": 11 + segment * 20,
                          "source_ts_ns": rows[i - 1]["ts_ns"], "target_segment_id": segment,
                          "world_origin_segment_id": world}
            if anchor is not None:
                bridge.update(origin="VisualOutageRecovery", result="Pending", reason="AwaitingValidatedRoot",
                              lineage=copy.deepcopy(anchor), anchor_ts_ns=ts, transaction="EvidenceCommitted")
            if not ok and i not in pair.OUTAGES:
                init.update(path="Moving", phase=1, reason=3, outcome="Pending", transaction="EvidenceCommitted")
                bridge["transaction"] = "EvidenceCommitted"
            if i in starts:
                is_moving = i in moving
                init.update(path="Moving" if is_moving else "Static", phase=7, reason=0,
                            outcome="Committed", current_graph=1, transaction="RootCommitted",
                            attempt_id=17 + segment if is_moving else None)
                expired = is_moving and role == "subject" and ts - anchor["source_ts_ns"] > 500_000_000
                bridge.update(result=("Eligible" if role == "control" else "NotApplicable" if expired else "Applied") if is_moving else "NotApplicable",
                              reason="AnchorExpired" if expired else "None" if is_moving else "StaticRootSelected", root_ts_ns=ts,
                              attempt_id=init["attempt_id"], transaction="RootCommitted")
                if segment == 0:
                    bridge["origin"] = "ProcessStart"
                world = anchor["world_origin_segment_id"] if bridge["result"] == "Applied" else segment
                if bridge["lineage"] is not None:
                    bridge["lineage"]["world_origin_segment_id"] = world
                anchor = None
            rows.append({"schema_version": 1, "i": i, "ts_ns": ts,
                         "status": "ok" if ok else "visual_outage" if i in pair.OUTAGES else "initializing",
                         "segment_id": segment, "frontend": {"is_keyframe": True,
                         "observations": [[91, 1.0, 2.0, 3.0], [4, 4.0, 5.0, 6.0]]},
                         "initialization": init, "bridge": bridge})
        return rows

    def test_static_events_stay_in_endpoint_set_and_zero_eligible_stops(self):
        with self.assertRaises(pair.PairError) as caught:
            pair.control_eligible(self.typed_rows(()))
        self.assertEqual(caught.exception.detail["field"], "eligible_count")
        self.assertEqual(len(pair.PAIRS), 6)

    def test_control_eligible_uses_typed_committed_identity(self):
        rows = self.typed_rows()
        self.assertEqual(pair.control_eligible(rows), [459])
        rows[459]["bridge"]["attempt_id"] = 19
        with self.assertRaises(pair.PairError) as caught:
            pair.control_eligible(rows)
        self.assertIn("attempt_id", caught.exception.detail["field"])

    def test_producer_pending_and_recoverable_phases_are_accepted(self):
        # Literal legal combinations from initialization_diagnostics.cpp.
        cases = (
            ("Pending", 1, 3, 0), ("Pending", 1, 4, 0),
            ("Pending", 2, 5, 0), ("Pending", 2, 6, 0),
            ("Pending", 2, 7, 0), ("Pending", 2, 8, 0),
            ("Pending", 2, 9, 0),
            ("RecoverableRejected", 3, 10, 0),
            ("RecoverableRejected", 4, 10, 0),
            ("RecoverableRejected", 3, 11, 0),
            ("RecoverableRejected", 4, 11, 0),
            ("RecoverableRejected", 5, 12, 0),
            ("RecoverableRejected", 5, 13, 0),
            ("RecoverableRejected", 5, 14, 0),
            ("RecoverableRejected", 6, 15, 2),
            ("RecoverableRejected", 6, 16, 2),
        )
        for outcome, phase, reason, graph in cases:
            with self.subTest(outcome=outcome, phase=phase, reason=reason):
                rows = self.typed_rows()
                attempt = 17 if outcome == "RecoverableRejected" else None
                rows[450]["initialization"].update(
                    outcome=outcome, phase=phase, reason=reason,
                    current_graph=graph, attempt_id=attempt)
                rows[450]["bridge"]["attempt_id"] = attempt
                self.assertEqual(pair.control_eligible(rows), [459])

    def test_pending_validated_graph_stops(self):
        rows = self.typed_rows()
        rows[450]["initialization"].update(phase=2, reason=7, current_graph=1)
        with self.assertRaises(pair.PairError) as caught:
            pair.control_eligible(rows)
        self.assertEqual(caught.exception.detail["field"],
                         "row[450].initialization.current_graph")

    def test_static_pending_visual_seed_preserves_moving_recovery_eligibility(self):
        rows = self.typed_rows()
        rows[450]["initialization"].update(
            path="Static", phase=1, reason=1, outcome="Pending",
            attempt_id=None, current_graph=0, transaction="EvidenceCommitted")
        rows[450]["bridge"]["attempt_id"] = None
        self.assertEqual(pair.control_eligible(rows), [459])

    def test_bad_typed_control_stops_before_subject_remote_action(self):
        for defect, field in (("graph", "current_graph"), ("lineage", "lineage"), ("world", "world_origin_segment_id")):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                plan = self.make_pair(root)
                path = root / "control/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / pair.LOG_NAME
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                if defect == "graph":
                    rows[459]["initialization"]["current_graph"] = 2
                elif defect == "lineage":
                    rows[459]["bridge"]["lineage"] = {"target_segment_id": 1}
                else:
                    rows[459]["bridge"]["lineage"]["world_origin_segment_id"] = 0
                path.write_text("".join(json.dumps(row) + "\n" for row in rows))
                native = Mock()
                with self.assertRaises(pair.PairError) as caught:
                    pair.start_role(native, plan, "subject", root / "control")
                self.assertIn(field, caught.exception.detail["field"])
                native.start_remote_run.assert_not_called()
                native.VerifiedSsh.assert_not_called()

    def test_subject_world_inherits_short_bridge_after_expired_root_then_static_resets(self):
        rows = self.typed_rows((809, 937), "subject")
        pair.validate_lifecycle(rows, "subject")
        self.assertEqual(rows[809]["bridge"]["lineage"]["source_segment_id"], 1)
        self.assertEqual(rows[809]["bridge"]["lineage"]["world_origin_segment_id"], 2)
        self.assertEqual(rows[937]["bridge"]["lineage"]["world_origin_segment_id"], 2)
        self.assertEqual(rows[1158]["bridge"]["lineage"]["world_origin_segment_id"], 4)
        for index, wrong_world in ((809, 1), (930, 0), (937, 3), (1158, 2)):
            with self.subTest(index=index):
                wrong = copy.deepcopy(rows)
                wrong[index]["bridge"]["lineage"]["world_origin_segment_id"] = wrong_world
                with self.assertRaises(pair.PairError) as caught:
                    pair.validate_lifecycle(wrong, "subject")
                self.assertEqual(caught.exception.detail["field"], f"row[{index}].bridge.lineage.world_origin_segment_id")

    def test_expired_subject_root_commits_new_world_and_retains_six_endpoints(self):
        rows = self.typed_rows((937, 1097), "subject")
        bridge = rows[1097]["bridge"]
        bridge.update(result="NotApplicable", reason="AnchorExpired")
        bridge["lineage"]["world_origin_segment_id"] = 4
        for index in range(1158, 1343):
            rows[index]["bridge"]["lineage"]["world_origin_segment_id"] = 4
        pair.validate_lifecycle(rows, "subject")
        self.assertEqual(bridge["root_ts_ns"] - bridge["lineage"]["source_ts_ns"], 4_350_000_000)
        self.assertEqual(len(pair.PAIRS), 6)

    def test_subject_cannot_expire_a_root_inside_the_horizon(self):
        rows = self.typed_rows((937,), "subject")
        rows[937]["bridge"].update(result="NotApplicable", reason="AnchorExpired")
        rows[937]["bridge"]["lineage"]["world_origin_segment_id"] = 3
        with self.assertRaises(pair.PairError) as caught:
            pair.validate_lifecycle(rows, "subject")
        self.assertEqual(caught.exception.detail["field"], "row[937].bridge.result")

    def test_anchor_internal_frame_identity_is_opaque_but_stable(self):
        rows = self.typed_rows()
        self.assertNotEqual(rows[434]["bridge"]["lineage"]["source_frame_index"], 433)
        pair.control_eligible(rows)
        rows[450]["bridge"]["lineage"]["source_frame_index"] += 1
        with self.assertRaises(pair.PairError) as caught:
            pair.control_eligible(rows)
        self.assertEqual(caught.exception.detail["field"], "row[450].bridge.lineage.source_frame_index")

    def test_consumed_anchor_must_be_absent_on_active_row(self):
        rows = self.typed_rows()
        rows[460]["bridge"]["lineage"] = copy.deepcopy(rows[459]["bridge"]["lineage"])
        with self.assertRaises(pair.PairError) as caught:
            pair.control_eligible(rows)
        self.assertEqual(caught.exception.detail["field"], "row[460].bridge.lineage")

    def test_first_difference_prefers_earlier_ingestion_over_later_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self.make_pair(root)
            bench = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench"
            path = bench / "diag.csv"
            lines = path.read_bytes().splitlines(keepends=True)
            lines[435] = lines[435].replace(b",0,0,", b",9,0,", 1)
            path.write_bytes(b"".join(lines))
            path = bench / pair.LOG_NAME
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            rows[400]["frontend"]["observations"].reverse()
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
            with self.assertRaises(pair.PairError) as caught:
                pair.verify_pair(plan, root / "control", root / "subject")
            self.assertEqual(caught.exception.detail["field"], "row[400].frontend")

    def test_first_difference_compares_lifecycle_and_ingestion_indices(self):
        cases = ((450, 500, "row[450].bridge.lineage.world_origin_segment_id"),
                 (809, 400, "row[400].frontend"))
        for typed_index, input_index, expected_field in cases:
            with self.subTest(typed_index=typed_index), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                plan = self.make_pair(root)
                path = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / pair.LOG_NAME
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                if typed_index == 450:
                    rows[450]["bridge"]["lineage"]["world_origin_segment_id"] = 99
                else:
                    rows[809]["initialization"]["current_graph"] = 2
                rows[input_index]["frontend"]["observations"].reverse()
                path.write_text("".join(json.dumps(row) + "\n" for row in rows))
                with self.assertRaises(pair.PairError) as caught:
                    pair.verify_pair(plan, root / "control", root / "subject")
                self.assertEqual(caught.exception.detail["field"], expected_field)

    def test_first_difference_compares_lifecycle_and_payload_indices(self):
        cases = ((450, 500, "row[450].bridge.lineage.world_origin_segment_id"),
                 (809, 400, "pre-applied.diag.csv.line[402]"))
        for typed_index, payload_index, expected_field in cases:
            with self.subTest(typed_index=typed_index), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                plan = self.make_pair(root, moving_index=809)
                bench = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench"
                path = bench / pair.LOG_NAME
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                if typed_index == 450:
                    rows[450]["bridge"]["lineage"]["world_origin_segment_id"] = 99
                else:
                    rows[809]["initialization"]["current_graph"] = 2
                path.write_text("".join(json.dumps(row) + "\n" for row in rows))
                path = bench / "diag.csv"
                lines = path.read_bytes().splitlines(keepends=True)
                lines[payload_index + 1] = lines[payload_index + 1].replace(b",0,0,", b",9,0,", 1)
                path.write_bytes(b"".join(lines))
                with self.assertRaises(pair.PairError) as caught:
                    pair.verify_pair(plan, root / "control", root / "subject")
                self.assertEqual(caught.exception.detail["field"], expected_field)

    def test_first_applied_payload_is_excluded_from_exact_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = self.make_pair(root, moving_index=937)
            bench = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench"
            path = bench / "diag.csv"
            lines = path.read_bytes().splitlines(keepends=True)
            lines[938] = lines[938].replace(b",0,0,", b",9,0,", 1)
            path.write_bytes(b"".join(lines))
            for filename in ("est.tum", "kf.tum"):
                path = bench / filename
                path.write_bytes(path.read_bytes() + b"1400000046.850000000 1 0 0 0 0 0 1\n")
            self.assertEqual(pair.verify_pair(plan, root / "control", root / "subject")["status"], "execution-complete")

    def test_all_expired_roots_require_identical_complete_payload(self):
        for filename in ("est.tum", "kf.tum"):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                plan = self.make_pair(root)
                path = root / "subject/remote/tasks" / pair.SEQUENCE / "artifacts/bench" / filename
                path.write_bytes(path.read_bytes() + b"1400000200.000000000 1 0 0 0 0 0 1\n")
                with self.assertRaises(pair.PairError) as caught:
                    pair.verify_pair(plan, root / "control", root / "subject")
                self.assertEqual(caught.exception.detail["field"], f"pre-applied.{filename}.line[2]")

    def test_bad_control_prevents_any_subject_remote_action(self):
        native = Mock()
        with patch.object(pair, "verify_control", side_effect=pair.PairError("eligible", ">0", 0)):
            with self.assertRaises(pair.PairError):
                pair.start_role(native, {}, "subject", Path("control"))
        native.start_remote_run.assert_not_called()
        native.VerifiedSsh.assert_not_called()

    def test_prepare_is_local_and_rejects_extra_source_delta(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = root / "baseline"
            baseline.mkdir()
            (baseline / "meta.json").write_text(json.dumps({
                "config": {"estimator.visual_coast_horizon_ns": 500000000,
                           "estimator.enable_moving_bootstrap": True},
                "config_hash": "4e2b30f2", "config_canonical_text": "frozen"}))
            manifest = root / "input.sha256"
            manifest.write_text("a" * 64 + "  ./mav0/cam0/data.csv\n")
            args = pair.parser().parse_args([
                "prepare", "--control", str(root / "control"), "--subject", str(root / "subject"),
                "--baseline", str(baseline), "--remote-root", "/isolated/58",
                "--dataset-root", "/datasets/euroc", "--dataset-manifest", str(manifest),
                "--output", str(root / "pair.json"), "--activation-file", "bridge.cpp"])
            native = Mock()
            native.REMOTE_ROOT = Path("/existing")
            native.build_source_manifest.side_effect = [
                ({"entries": [{"path": "bridge.cpp", "sha256": "off"}]}, "control-id"),
                ({"entries": [{"path": "bridge.cpp", "sha256": "on"}]}, "subject-id")]
            plan = pair.prepare(args, native)
            self.assertEqual(plan["sequence"], pair.SEQUENCE)
            self.assertEqual(plan["control"]["source_id"], "control-id")
            native.VerifiedSsh.assert_not_called()
            native.create_source_inputs.assert_not_called()
            native._run_remote_task.assert_not_called()
            native.build_source_manifest.side_effect = [
                ({"entries": []}, "a"), ({"entries": [{"path": "extra.cpp"}]}, "b")]
            with self.assertRaises(pair.PairError) as caught:
                pair.prepare(args, native)
            self.assertEqual(caught.exception.detail["field"], "source delta paths")


class MemorySsh:
    """Transport fake; native JSON serialization and immutable upload stay real."""

    def __init__(self):
        self.files = {}
        self.uploads = []
        self.publications = []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def run(self, command, *, check=True):
        if command[0] == "mkdir":
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[0] == "sha256sum":
            data = self.files.get(Path(command[1]))
            if data is None:
                return subprocess.CompletedProcess(command, 1, "", "missing")
            digest = hashlib.sha256(data).hexdigest()
            return subprocess.CompletedProcess(command, 0, f"{digest}  {command[1]}\n", "")
        if command[:2] == ["mv", "-n"]:
            source, destination = map(Path, command[2:])
            if destination not in self.files:
                self.files[destination] = self.files.pop(source)
                self.publications.append((destination, self.files[destination]))
            return subprocess.CompletedProcess(command, 0, "", "")
        if command[0] == "cat":
            return subprocess.CompletedProcess(command, 0, self.files[Path(command[1])].decode(), "")
        raise AssertionError(f"unexpected SSH command: {command}")

    def upload(self, local, remote):
        self.files[Path(remote)] = Path(local).read_bytes()
        self.uploads.append(Path(remote))


class NativeLaunchTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.native = pair.load_native(SCRIPT.with_name("remote_ci.py"))
        self.native.REMOTE_ROOT = Path("/isolated/58")
        self.native._remote_identity = self.native.RemoteIdentity("fake.invalid", "test", "SHA256:fixture")
        self.ssh = MemorySsh()
        self.plan = {
            "control": {"source_id": "c", "root": str(self.root / "control-source")},
            "subject": {"source_id": "s", "root": str(self.root / "subject-source")},
            "remote_root": "/isolated/58", "local_root": str(self.root / "local"),
            "dataset_root": "/datasets/euroc",
        }
        Path(self.plan["local_root"]).mkdir()
        self.source = {"source_id": "c", "manifest": {}, "archive_sha256": "archive", "bundle_sha256": "bundle"}
        self.image = {
            "toolchain_id": "t", "dockerfile_sha256": "dockerfile", "gtsam_archive_sha256": "gtsam",
            "tag": "image-tag", "id": "image-id", "oci_manifest_id": "m", "oci_config_id": "c",
            "remote_id": "remote-image-id", "repo_digests": [], "architecture": "amd64",
            "os": "linux", "rootfs_layers": [], "archive_sha256": "image-archive", "versions": {},
        }
        self.external = {}
        for name, value in {
            "VerifiedSsh": self.ssh,
            "ensure_local_image": self.image, "export_local_image": self.image,
            "ensure_remote_image": self.image,
            "create_source_inputs": self.source,
            "ensure_remote_source": {"remote_source_root": "/isolated/58/sources/frozen"},
            "remote_doctor": {"uid": 1000, "gid": 1000, "architecture": "amd64",
                              "dataset_root": "/datasets/euroc"},
            "launch_remote_worker": {"pid": 123, "detached": True},
        }.items():
            patcher = patch.object(self.native, name, return_value=value)
            self.external[name] = patcher.start()
            self.addCleanup(patcher.stop)
        # External command availability is independent of the launch/config contract.
        patcher = patch.object(self.native.shutil, "which", return_value="/fake/bin/tool")
        patcher.start()
        self.addCleanup(patcher.stop)
        self.launched_runs = []

        def launch(ssh, run_root, remote_script):
            self.launched_runs.append(json.loads(ssh.files[run_root / "run.json"]))
            return {"pid": 123, "detached": True}

        self.external["launch_remote_worker"].side_effect = launch

    def test_pair_first_publication_is_complete_and_launches_once(self):
        result = pair.start_role(self.native, self.plan, "control")
        self.assert_pair_launch(result, "control")

    def assert_pair_launch(self, result, role):
        self.assertEqual(len(self.ssh.publications), 1)
        self.assertEqual(len(self.ssh.uploads), 1)
        path, data = self.ssh.publications[0]
        run = json.loads(data)
        self.assertEqual(path, Path(result["run_root"]) / "run.json")
        self.assertEqual(run["unit_parallelism"], 1)
        self.assertEqual(run["max_parallel_tasks"], 1)
        self.assertEqual(run["sequences"], [pair.SEQUENCE])
        self.assertEqual(run["source_id"], self.plan[role]["source_id"])
        self.assertEqual(run["m5_recovery_pair"], self.plan)
        self.assertEqual(run["m5_recovery_role"], role)
        self.assertEqual(self.launched_runs, [run])
        self.external["launch_remote_worker"].assert_called_once_with(
            self.ssh, Path(result["run_root"]),
            Path("/isolated/58/sources/frozen/repo/scripts/m5_recovery_pair.py"))
        self.assertEqual(pair.read_json(Path(self.plan["local_root"]) / f"{role}-start.json"), result)

    def prepare_subject(self):
        artifact_plan = RecoveryPairTest().make_pair(self.root)
        artifact_plan.update(self.plan)
        self.plan = artifact_plan
        run_path = self.root / "control/remote/run.json"
        run = pair.read_json(run_path)
        run["m5_recovery_pair"] = self.plan
        pair.write_json(run_path, run)
        self.source["source_id"] = "s"
        return self.root / "control"

    def assert_not_published_or_launched(self, role):
        self.assertEqual(self.ssh.publications, [])
        self.assertEqual(self.ssh.uploads, [])
        self.external["launch_remote_worker"].assert_not_called()
        self.assertFalse((Path(self.plan["local_root"]) / f"{role}-start.json").exists())

    def test_subject_matching_control_image_publishes_once(self):
        control = self.prepare_subject()
        result = pair.start_role(self.native, self.plan, "subject", control)
        self.assert_pair_launch(result, "subject")

    def test_subject_image_mismatch_stops_before_publication_or_launch(self):
        control = self.prepare_subject()
        for key in ("oci_manifest_id", "oci_config_id", "toolchain_id"):
            with self.subTest(key=key):
                original = self.image[key]
                self.image[key] = "another-image"
                with self.assertRaises(pair.PairError) as caught:
                    pair.start_role(self.native, self.plan, "subject", control)
                self.assertEqual(caught.exception.detail["field"], f"image.{key}")
                self.assert_not_published_or_launched("subject")
                self.image[key] = original

    def test_source_mismatch_stops_before_publication_or_launch(self):
        self.source["source_id"] = "another-source"
        with self.assertRaises(pair.PairError) as caught:
            pair.start_role(self.native, self.plan, "control")
        self.assertEqual(caught.exception.detail["field"], "control.source_id")
        self.assert_not_published_or_launched("control")

    def test_failed_control_gate_never_enters_native_start(self):
        control = self.prepare_subject()
        status_path = control / "remote/run-status.json"
        status = pair.read_json(status_path)
        status["tasks"]["unit"].update(state="failed", return_code=1)
        pair.write_json(status_path, status)
        with self.assertRaises(pair.PairError) as caught:
            pair.start_role(self.native, self.plan, "subject", control)
        self.assertEqual(caught.exception.detail["field"], "unit.return_code")
        for external in self.external.values():
            external.assert_not_called()
        self.assert_not_published_or_launched("subject")

    def test_native_default_publication_keeps_original_profile_and_worker(self):
        result = self.native.start_remote_run(self.root, self.native.PROFILE)
        self.assertEqual(len(self.ssh.publications), 1)
        self.assertEqual(len(self.ssh.uploads), 1)
        run = json.loads(self.ssh.publications[0][1])
        self.assertEqual(run["unit_parallelism"], 8)
        self.assertEqual(run["max_parallel_tasks"], 8)
        self.assertEqual(run["sequences"], list(self.native.SEQUENCES))
        self.assertEqual(len(run["sequences"]), 11)
        self.assertNotIn("m5_recovery_pair", run)
        self.assertNotIn("m5_recovery_role", run)
        self.assertEqual(self.launched_runs, [run])
        self.external["launch_remote_worker"].assert_called_once_with(
            self.ssh, Path(result["run_root"]),
            Path("/isolated/58/sources/frozen/repo/scripts/remote_ci.py"))


if __name__ == "__main__":
    unittest.main()

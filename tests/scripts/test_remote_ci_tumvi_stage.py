from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock
import zlib
import struct


REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "phad_remote_ci_tumvi_stage", REPO_ROOT / "scripts" / "remote_ci.py"
)
assert SPEC is not None and SPEC.loader is not None
remote_ci = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = remote_ci
SPEC.loader.exec_module(remote_ci)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def png_1x1(value: int) -> bytes:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes((0, value))))
        + chunk(b"IEND", b"")
    )


CAMCHAIN = b"cam0:\n  camera_model: pinhole\ncam1:\n  camera_model: pinhole\n"
IMU_CONFIG = b"imu0:\n  update_rate: 200.0\n"


def default_files() -> dict[str, bytes]:
    camera_csv = b"#timestamp [ns],filename\n100,100.png\n200,200.png\n"
    return {
        "mav0/cam0/data.csv": camera_csv,
        "mav0/cam0/data/100.png": png_1x1(1),
        "mav0/cam0/data/200.png": png_1x1(2),
        "mav0/cam1/data.csv": camera_csv,
        "mav0/cam1/data/100.png": png_1x1(3),
        "mav0/cam1/data/200.png": png_1x1(4),
        "mav0/imu0/data.csv": (
            b"#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
            b"w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
            b"a_RS_S_z [m s^-2]\n100,0,0,0,0,0,9.81\n200,0,0,0,0,0,9.81\n"
        ),
        "dso/camchain.yaml": CAMCHAIN,
        "dso/imu_config.yaml": IMU_CONFIG,
        "dso/gt_imu.csv": (
            b"# timestamp[ns],tx,ty,tz,qw,qx,qy,qz\n"
            b"100,0,0,0,1,0,0,0\n"
            b"200,0.1,0,0,1,0,0,0\n"
        ),
    }


def known_skipped_files() -> dict[str, bytes]:
    return {
        "mav0/mocap0/data.csv": b"# timestamp[ns],tx,ty,tz,qw,qx,qy,qz\n",
        "dso/cam0/times.txt": b"100 0.0\n",
        "dso/cam0/camera.txt": b"camera\n",
        "dso/cam0/vignette.png": png_1x1(1),
        "dso/cam0/pcalib.txt": b"response\n",
        "dso/cam1/times.txt": b"100 0.0\n",
        "dso/cam1/camera.txt": b"camera\n",
        "dso/cam1/vignette.png": png_1x1(1),
        "dso/cam1/pcalib.txt": b"response\n",
        "dso/imu.txt": b"100 0 0 0 0 0 9.81\n",
    }


class ArchiveFixture:
    def __init__(self, root: Path, sequence: str = "room1") -> None:
        self.sequence = sequence
        self.archive = root / f"dataset-{sequence}_512_16.tar"
        self.files = default_files()
        self.files.update(known_skipped_files())
        self.symlinks = {
            "dso/cam0/images": "../../mav0/cam0/data",
            "dso/cam1/images": "../../mav0/cam1/data",
        }

    @property
    def archive_root(self) -> str:
        return f"dataset-{self.sequence}_512_16"

    def write(
        self,
        *,
        extra: list[tarfile.TarInfo | tuple[tarfile.TarInfo, bytes]] | None = None,
        file_order: list[str] | None = None,
    ) -> None:
        with tarfile.open(self.archive, "w", format=tarfile.PAX_FORMAT) as stream:
            directories = (
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
            )
            for relative in directories:
                name = (
                    self.archive_root
                    if not relative
                    else f"{self.archive_root}/{relative}"
                )
                member = tarfile.TarInfo(name)
                member.type = tarfile.DIRTYPE
                member.mode = 0o755
                stream.addfile(member)
            names = file_order if file_order is not None else sorted(self.files)
            for relative in names:
                payload = self.files[relative]
                member = tarfile.TarInfo(f"{self.archive_root}/{relative}")
                member.size = len(payload)
                member.mode = 0o644
                member.mtime = 123456789
                stream.addfile(member, io.BytesIO(payload))
            for relative, target in sorted(self.symlinks.items()):
                member = tarfile.TarInfo(f"{self.archive_root}/{relative}")
                member.type = tarfile.SYMTYPE
                member.linkname = target
                member.mode = 0o777
                stream.addfile(member)
            for item in extra or []:
                if isinstance(item, tuple):
                    member, payload = item
                else:
                    member, payload = item, None
                if member.name.startswith("RAW:"):
                    member.name = member.name.removeprefix("RAW:")
                elif not member.name.startswith("/"):
                    member.name = f"{self.archive_root}/{member.name}"
                if payload is not None:
                    member.size = len(payload)
                    stream.addfile(member, io.BytesIO(payload))
                else:
                    stream.addfile(member)

    def expected(self) -> dict[str, object]:
        return {
            "filename": self.archive.name,
            "size": self.archive.stat().st_size,
            "sha256": remote_ci.sha256_file(self.archive),
            "archive_root": self.archive_root,
            "cam0_frames": 2,
            "cam1_frames": 2,
            "imu_rows": 2,
            "gt_rows": 2,
            "camchain_sha256": sha256_bytes(CAMCHAIN),
            "imu_config_sha256": sha256_bytes(IMU_CONFIG),
        }


def regular_member(
    name: str, payload: bytes = b"unexpected\n"
) -> tuple[tarfile.TarInfo, bytes]:
    member = tarfile.TarInfo(name)
    member.mode = 0o644
    return member, payload


class TumViArchivePreflightTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fixture = ArchiveFixture(self.root)
        self.fixture.write()
        self.destination = self.root / "staging" / "room1"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_size_and_sha256_are_checked_before_tar_is_opened(self) -> None:
        for field, value in (("size", 1), ("sha256", "0" * 64)):
            with self.subTest(field=field):
                expected = self.fixture.expected()
                expected[field] = value
                with mock.patch.object(remote_ci.tarfile, "open") as tar_open:
                    with self.assertRaisesRegex(remote_ci.RemoteCiError, field):
                        remote_ci.audit_and_extract_tumvi_archive(
                            self.fixture.archive,
                            self.destination,
                            sequence="room1",
                            expected=expected,
                        )
                tar_open.assert_not_called()
                self.assertFalse(self.destination.exists())


class TumViSafeExtractionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.destination = self.root / "staging" / "room1"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def make_fixture(
        self,
        *,
        extra: list[tarfile.TarInfo | tuple[tarfile.TarInfo, bytes]] | None = None,
    ) -> ArchiveFixture:
        fixture = ArchiveFixture(self.root)
        fixture.write(extra=extra)
        return fixture

    def extract(self, fixture: ArchiveFixture) -> dict[str, object]:
        return remote_ci.audit_and_extract_tumvi_archive(
            fixture.archive,
            self.destination,
            sequence="room1",
            expected=fixture.expected(),
        )

    def test_extracts_only_runtime_tree_and_audits_but_skips_two_exact_links(self) -> None:
        fixture = self.make_fixture()

        result = self.extract(fixture)

        self.assertEqual(result["sequence"], "room1")
        self.assertEqual(result["archive_sha256"], fixture.expected()["sha256"])
        self.assertTrue((self.destination / "mav0/cam0/data/100.png").is_file())
        self.assertTrue((self.destination / "mav0/cam1/data/200.png").is_file())
        self.assertTrue((self.destination / "mav0/imu0/data.csv").is_file())
        self.assertTrue((self.destination / "dso/gt_imu.csv").is_file())
        self.assertFalse((self.destination / "dso/cam0/images").exists())
        self.assertFalse((self.destination / "dso/cam1/images").exists())
        for relative in known_skipped_files():
            with self.subTest(skipped=relative):
                self.assertFalse((self.destination / relative).exists())

    def test_never_calls_extractall(self) -> None:
        fixture = self.make_fixture()
        with mock.patch.object(
            tarfile.TarFile,
            "extractall",
            side_effect=AssertionError("extractall must not be used"),
        ):
            self.extract(fixture)

    def test_rejects_unsafe_member_names(self) -> None:
        unsafe = (
            "/absolute",
            "../escape",
            "mav0/cam0/../../escape",
            r"mav0\cam0\data.csv",
        )
        for index, name in enumerate(unsafe):
            with self.subTest(name=name):
                destination = self.root / f"unsafe-{index}"
                member = regular_member(f"RAW:{name}" if name.startswith("/") else name)
                fixture = ArchiveFixture(self.root, sequence=f"room{index + 1}")
                fixture.write(extra=[member])
                with self.assertRaisesRegex(remote_ci.RemoteCiError, "path|member"):
                    remote_ci.audit_and_extract_tumvi_archive(
                        fixture.archive,
                        destination,
                        sequence=fixture.sequence,
                        expected=fixture.expected(),
                    )
                self.assertFalse(destination.exists())

    def test_rejects_nul_member_name(self) -> None:
        fixture = self.make_fixture()
        original_open = tarfile.open

        class TarWithNul:
            def __enter__(self):
                self.stream = original_open(fixture.archive, "r")
                self.members = self.stream.getmembers()
                self.members.append(tarfile.TarInfo(f"{fixture.archive_root}/bad\0name"))
                return self

            def __exit__(self, *args):
                self.stream.close()

            def getmembers(self):
                return self.members

            def extractfile(self, member):
                return self.stream.extractfile(member)

        with mock.patch.object(remote_ci.tarfile, "open", return_value=TarWithNul()):
            with self.assertRaisesRegex(remote_ci.RemoteCiError, "NUL|path|member"):
                self.extract(fixture)
        self.assertFalse(self.destination.exists())

    def test_rejects_duplicate_member_name(self) -> None:
        duplicate = regular_member("mav0/cam0/data.csv", default_files()["mav0/cam0/data.csv"])
        fixture = self.make_fixture(extra=[duplicate])
        with self.assertRaisesRegex(remote_ci.RemoteCiError, "duplicate"):
            self.extract(fixture)
        self.assertFalse(self.destination.exists())

    def test_rejects_hardlinks_devices_fifos_and_undeclared_members(self) -> None:
        cases: list[
            tuple[str, tarfile.TarInfo | tuple[tarfile.TarInfo, bytes]]
        ] = []
        hardlink = tarfile.TarInfo("mav0/cam0/hardlink")
        hardlink.type = tarfile.LNKTYPE
        hardlink.linkname = "dataset-room1_512_16/mav0/cam0/data.csv"
        cases.append(("hardlink", hardlink))
        device = tarfile.TarInfo("mav0/cam0/device")
        device.type = tarfile.CHRTYPE
        cases.append(("device", device))
        fifo = tarfile.TarInfo("mav0/cam0/fifo")
        fifo.type = tarfile.FIFOTYPE
        cases.append(("FIFO", fifo))
        cases.append(("undeclared", regular_member("unexpected.txt")))

        for index, (label, member) in enumerate(cases):
            with self.subTest(kind=label):
                fixture = ArchiveFixture(self.root, sequence=f"room{index + 1}")
                fixture.write(extra=[member])
                destination = self.root / f"special-{index}"
                with self.assertRaisesRegex(
                    remote_ci.RemoteCiError,
                    "hardlink|device|FIFO|unsupported|undeclared|member",
                ):
                    remote_ci.audit_and_extract_tumvi_archive(
                        fixture.archive,
                        destination,
                        sequence=fixture.sequence,
                        expected=fixture.expected(),
                    )
                self.assertFalse(destination.exists())

    def test_rejects_wrong_target_or_additional_symlink(self) -> None:
        cases = (
            ("dso/cam0/images", "../../mav0/cam1/data"),
            ("dso/other", "../../mav0/cam0/data"),
        )
        for index, (name, target) in enumerate(cases):
            with self.subTest(name=name, target=target):
                fixture = ArchiveFixture(self.root, sequence=f"room{index + 1}")
                fixture.symlinks[name] = target
                if name == "dso/cam0/images":
                    fixture.symlinks.pop("dso/cam0/images", None)
                    fixture.symlinks[name] = target
                fixture.write()
                destination = self.root / f"link-{index}"
                with self.assertRaisesRegex(remote_ci.RemoteCiError, "symlink|link"):
                    remote_ci.audit_and_extract_tumvi_archive(
                        fixture.archive,
                        destination,
                        sequence=fixture.sequence,
                        expected=fixture.expected(),
                    )
                self.assertFalse(destination.exists())


class TumViDatasetValidationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def assert_archive_rejected(
        self, mutate: object, pattern: str
    ) -> None:
        fixture = ArchiveFixture(self.root)
        mutate(fixture)
        fixture.write()
        destination = self.root / "staging" / pattern.replace("\\", "-")
        with self.assertRaisesRegex(remote_ci.RemoteCiError, pattern):
            remote_ci.audit_and_extract_tumvi_archive(
                fixture.archive,
                destination,
                sequence="room1",
                expected=fixture.expected(),
            )
        self.assertFalse(destination.exists())

    def test_rejects_camera_csv_png_bijection_and_expected_count_errors(self) -> None:
        def missing_png(fixture: ArchiveFixture) -> None:
            fixture.files.pop("mav0/cam0/data/200.png")

        self.assert_archive_rejected(missing_png, "PNG|camera|frame")

        def extra_png(fixture: ArchiveFixture) -> None:
            fixture.files["mav0/cam1/data/300.png"] = png_1x1(5)

        self.assert_archive_rejected(extra_png, "PNG|camera|frame")

        fixture = ArchiveFixture(self.root)
        fixture.write()
        expected = fixture.expected()
        expected["cam0_frames"] = 3
        destination = self.root / "staging-count"
        with self.assertRaisesRegex(remote_ci.RemoteCiError, "count|frames"):
            remote_ci.audit_and_extract_tumvi_archive(
                fixture.archive,
                destination,
                sequence="room1",
                expected=expected,
            )
        self.assertFalse(destination.exists())

    def test_rejects_calibration_hash_mismatch(self) -> None:
        fixture = ArchiveFixture(self.root)
        fixture.write()
        expected = fixture.expected()
        expected["camchain_sha256"] = "0" * 64
        with self.assertRaisesRegex(remote_ci.RemoteCiError, "camchain|calibration|sha256"):
            remote_ci.audit_and_extract_tumvi_archive(
                fixture.archive,
                self.root / "staging-calibration",
                sequence="room1",
                expected=expected,
            )

    def test_rejects_noncanonical_groundtruth_header(self) -> None:
        def wrong_header(fixture: ArchiveFixture) -> None:
            fixture.files["dso/gt_imu.csv"] = fixture.files[
                "dso/gt_imu.csv"
            ].replace(
                b"# timestamp[ns],tx,ty,tz,qw,qx,qy,qz",
                b"# timestamp,tx,ty,tz,qx,qy,qz,qw",
            )

        self.assert_archive_rejected(wrong_header, "groundtruth|header|gt_imu")

    def test_groundtruth_quaternion_tolerance_matches_loader_contract(self) -> None:
        def outside_tolerance(fixture: ArchiveFixture) -> None:
            fixture.files["dso/gt_imu.csv"] = fixture.files[
                "dso/gt_imu.csv"
            ].replace(b",1,0,0,0\n", b",1.00001,0,0,0\n")

        self.assert_archive_rejected(outside_tolerance, "quaternion|unit")


class TumViCanonicalManifestTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def materialize(self, root: Path, reverse: bool) -> None:
        files = default_files()
        for name in sorted(files, reverse=reverse):
            path = root / "room1" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(files[name])
            path.chmod(0o644)
            os.utime(path, (2_000_000 if reverse else 1, 2_000_000 if reverse else 1))

    def test_manifest_and_content_id_ignore_creation_order_and_mtime(self) -> None:
        first = self.root / "first"
        second = self.root / "second"
        self.materialize(first, reverse=False)
        self.materialize(second, reverse=True)

        first_manifest = remote_ci.build_tumvi_dataset_manifest(
            first,
            archive_records={"room1": {"sha256": "a" * 64}},
            calibration={
                "camchain_sha256": sha256_bytes(CAMCHAIN),
                "imu_config_sha256": sha256_bytes(IMU_CONFIG),
            },
        )
        second_manifest = remote_ci.build_tumvi_dataset_manifest(
            second,
            archive_records={"room1": {"sha256": "a" * 64}},
            calibration={
                "camchain_sha256": sha256_bytes(CAMCHAIN),
                "imu_config_sha256": sha256_bytes(IMU_CONFIG),
            },
        )

        self.assertEqual(first_manifest, second_manifest)
        self.assertRegex(first_manifest["dataset_content_id"], r"^sha256:[0-9a-f]{64}$")
        self.assertTrue(first_manifest["entries"])
        self.assertEqual(
            [entry["path"] for entry in first_manifest["entries"]],
            sorted(entry["path"] for entry in first_manifest["entries"]),
        )

    def test_content_or_mode_change_changes_content_id(self) -> None:
        dataset = self.root / "dataset"
        self.materialize(dataset, reverse=False)
        arguments = {
            "archive_records": {"room1": {"sha256": "a" * 64}},
            "calibration": {
                "camchain_sha256": sha256_bytes(CAMCHAIN),
                "imu_config_sha256": sha256_bytes(IMU_CONFIG),
            },
        }
        baseline = remote_ci.build_tumvi_dataset_manifest(dataset, **arguments)
        target = dataset / "room1/mav0/cam0/data.csv"
        target.write_bytes(target.read_bytes() + b"300,300.png\n")
        changed_content = remote_ci.build_tumvi_dataset_manifest(dataset, **arguments)
        self.assertNotEqual(
            baseline["dataset_content_id"], changed_content["dataset_content_id"]
        )

        target.write_bytes(default_files()["mav0/cam0/data.csv"])
        target.chmod(0o600)
        changed_mode = remote_ci.build_tumvi_dataset_manifest(dataset, **arguments)
        self.assertNotEqual(
            baseline["dataset_content_id"], changed_mode["dataset_content_id"]
        )

    def test_original_archive_filename_does_not_change_content_id(self) -> None:
        dataset = self.root / "dataset"
        self.materialize(dataset, reverse=False)
        calibration = {
            "camchain_sha256": sha256_bytes(CAMCHAIN),
            "imu_config_sha256": sha256_bytes(IMU_CONFIG),
        }
        first = remote_ci.build_tumvi_dataset_manifest(
            dataset,
            archive_records={
                "room1": {"filename": "original.tar", "sha256": "a" * 64}
            },
            calibration=calibration,
        )
        renamed = remote_ci.build_tumvi_dataset_manifest(
            dataset,
            archive_records={
                "room1": {"filename": "renamed.tar", "sha256": "a" * 64}
            },
            calibration=calibration,
        )

        self.assertEqual(first["dataset_content_id"], renamed["dataset_content_id"])
        self.assertNotEqual(first["archives"], renamed["archives"])


class TumViTransactionalStageTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.fixture = ArchiveFixture(self.source)
        self.fixture.write()
        archive = self.fixture.expected()
        self.definition = {
            "name": "test-tumvi-stage",
            "dataset_manifest": {
                "schema_version": 1,
                "dataset": "test-room1",
                "archives": {"room1": archive},
                "calibration": {
                    "camchain_sha256": archive["camchain_sha256"],
                    "imu_config_sha256": archive["imu_config_sha256"],
                },
            },
        }
        self.definition["dataset_manifest_id"] = remote_ci._canonical_json_id(
            self.definition["dataset_manifest"]
        )
        self.store = self.root / "store"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def stage(self, transfer=shutil.copyfile):
        return remote_ci.stage_remote_dataset(
            {"room1": self.fixture.archive},
            self.store,
            definition=self.definition,
            transfer=transfer,
        )

    def test_success_is_idempotent_and_reuses_verified_archive_and_root(self) -> None:
        first = self.stage()
        transfer = mock.Mock(side_effect=AssertionError("verified inputs must be reused"))
        second = self.stage(transfer=transfer)

        self.assertEqual(first, second)
        transfer.assert_not_called()
        content_hex = first["dataset_content_id"].removeprefix("sha256:")
        published = self.store / "by-content" / "sha256" / content_hex
        sentinel = json.loads((published / ".phad-dataset.json").read_text())
        receipt = json.loads(
            (self.store / "receipts" / "sha256" / f"{content_hex}.json").read_text()
        )
        for document in (sentinel, receipt):
            self.assertEqual(
                document["dataset_manifest_id"],
                self.definition["dataset_manifest_id"],
            )
            self.assertEqual(
                document["dataset_content_id"], first["dataset_content_id"]
            )
        for path in published.rglob("*"):
            mode = stat.S_IMODE(path.lstat().st_mode)
            self.assertEqual(mode, 0o555 if path.is_dir() else 0o444, path)
        self.assertEqual(stat.S_IMODE(published.lstat().st_mode), 0o555)
        self.assertEqual(stat.S_IMODE(
            (self.store / "receipts" / "sha256" / f"{content_hex}.json").lstat().st_mode
        ), 0o444)
        self.assertEqual(
            {entry["mode"] for entry in receipt["dataset_tree_manifest"]["entries"]},
            {"0444"},
        )

    def test_existing_part_archive_or_published_root_conflicts_fail_closed(self) -> None:
        digest = str(self.fixture.expected()["sha256"])
        incoming = self.store / "incoming"
        incoming.mkdir(parents=True)
        (incoming / f"{digest}.part").write_bytes(b"wrong")
        with self.assertRaisesRegex(remote_ci.RemoteCiError, "part|partial|conflict"):
            self.stage()
        self.assertFalse((self.store / "by-content").exists())

        (incoming / f"{digest}.part").unlink()
        archive = self.store / "archives" / "sha256" / f"{digest}.tar"
        archive.parent.mkdir(parents=True, exist_ok=True)
        archive.write_bytes(b"wrong")
        with self.assertRaisesRegex(
            remote_ci.RemoteCiError, "archive|sha256|size|conflict"
        ):
            self.stage()
        self.assertFalse((self.store / "by-content").exists())

    def test_verified_complete_part_is_promoted_without_retransfer(self) -> None:
        digest = str(self.fixture.expected()["sha256"])
        incoming = self.store / "incoming"
        incoming.mkdir(parents=True)
        shutil.copyfile(
            self.fixture.archive,
            incoming / f"{digest}.part",
        )
        transfer = mock.Mock(side_effect=AssertionError("verified .part must be reused"))

        result = self.stage(transfer=transfer)

        transfer.assert_not_called()
        self.assertRegex(result["dataset_content_id"], r"^sha256:[0-9a-f]{64}$")
        self.assertTrue(
            (self.store / "archives" / "sha256" / f"{digest}.tar").is_file()
        )
        self.assertFalse((incoming / f"{digest}.part").exists())

    def test_existing_published_root_must_have_exact_sentinel(self) -> None:
        result = self.stage()
        content_hex = result["dataset_content_id"].removeprefix("sha256:")
        published = self.store / "by-content" / "sha256" / content_hex
        sentinel = published / ".phad-dataset.json"
        document = json.loads(sentinel.read_text())
        document["dataset_content_id"] = "sha256:" + "0" * 64
        published.chmod(0o755)
        sentinel.chmod(0o644)
        sentinel.write_text(json.dumps(document), encoding="utf-8")

        with self.assertRaisesRegex(remote_ci.RemoteCiError, "sentinel|content"):
            self.stage()

    def test_any_failure_does_not_publish_sentinel_or_receipt(self) -> None:
        self.definition["dataset_manifest"]["archives"]["room1"]["gt_rows"] = 3
        self.definition["dataset_manifest_id"] = remote_ci._canonical_json_id(
            self.definition["dataset_manifest"]
        )
        with self.assertRaises(remote_ci.RemoteCiError):
            self.stage()

        self.assertFalse((self.store / "by-content").exists())
        self.assertEqual(list(self.store.rglob(".phad-dataset.json")), [])
        self.assertEqual(list((self.store / "receipts").rglob("*.json")), [])

    def test_publish_visibility_step_receives_a_fully_sealed_tree(self) -> None:
        real_replace = os.replace
        observed_publish = False

        def inspect_replace(source: Path, destination: Path) -> None:
            nonlocal observed_publish
            source_path = Path(source)
            destination_path = Path(destination)
            if re.fullmatch(r"[0-9a-f]{64}", destination_path.name) is None:
                real_replace(source_path, destination_path)
                return
            observed_publish = True
            self.assertFalse(destination_path.exists())
            self.assertEqual(stat.S_IMODE(source_path.lstat().st_mode), 0o555)
            sentinel = source_path / ".phad-dataset.json"
            self.assertTrue(sentinel.is_file())
            self.assertEqual(stat.S_IMODE(sentinel.lstat().st_mode), 0o444)
            for path in source_path.rglob("*"):
                self.assertEqual(
                    stat.S_IMODE(path.lstat().st_mode),
                    0o555 if path.is_dir() else 0o444,
                    path,
                )
            raise OSError("injected publish failure")

        with mock.patch.object(remote_ci.os, "replace", side_effect=inspect_replace):
            with self.assertRaisesRegex(OSError, "injected publish failure"):
                self.stage()

        self.assertTrue(observed_publish)
        publish_parent = self.store / "by-content" / "sha256"
        self.assertEqual(
            [path for path in publish_parent.iterdir() if not path.name.startswith(".")],
            [],
        )
        self.assertEqual(list((self.store / "receipts").rglob("*.json")), [])


class TumViStartBindingTest(unittest.TestCase):
    def test_confirmatory_start_requires_explicit_dataset_content_id(self) -> None:
        parser = remote_ci.build_parser()
        without_id = parser.parse_args(
            ["start", "--profile", remote_ci.TUM_VI_CONFIRMATORY_PROFILE]
        )
        with mock.patch.object(remote_ci, "start_remote_run") as start:
            with mock.patch.object(sys, "stderr", new_callable=io.StringIO):
                return_code = remote_ci.main(
                    ["start", "--profile", remote_ci.TUM_VI_CONFIRMATORY_PROFILE]
                )
        self.assertEqual(return_code, 1)
        start.assert_not_called()
        self.assertIsNone(without_id.dataset_content_id)

        content_id = "sha256:" + "a" * 64
        arguments = parser.parse_args(
            [
                "start",
                "--profile",
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "--dataset-content-id",
                content_id,
            ]
        )
        self.assertEqual(arguments.dataset_content_id, content_id)
        with mock.patch.object(
            remote_ci,
            "start_remote_run",
            return_value={
                "run_id": "test-run",
                "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "source_id": "sha256:" + "b" * 64,
                "image_id": "sha256:" + "c" * 64,
                "image_config_id": "sha256:" + "d" * 64,
                "remote_image_id": "sha256:" + "e" * 64,
                "image_tag": "test",
                "run_root": "/tmp/test-run",
                "worker_pid": 1,
                "worker_detached": True,
            },
        ) as start, mock.patch.object(sys, "stdout", new_callable=io.StringIO):
            remote_ci._command_start(arguments)
        start.assert_called_once_with(
            remote_ci.repo_root(),
            remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            dataset_content_id=content_id,
        )

    def test_worker_sentinel_mismatch_starts_zero_tasks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "dataset"
            dataset.mkdir()
            remote_ci.atomic_write_json(
                dataset / ".phad-dataset.json",
                {"dataset_content_id": "sha256:" + "b" * 64},
            )
            run = {
                "schema_version": remote_ci.SCHEMA_VERSION,
                "run_id": "sentinel-mismatch",
                "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "run_root": str(root),
                "dataset_root": str(dataset),
                "dataset_content_id": "sha256:" + "a" * 64,
                "build_parallelism": 1,
                "unit_parallelism": 1,
            }
            remote_ci.atomic_write_json(root / "run.json", run)

            with mock.patch.object(remote_ci, "_run_remote_task") as run_task, mock.patch.object(
                remote_ci,
                "_create_results_archive",
                return_value=(root / "results.tar.zst", "c" * 64),
            ):
                return_code = remote_ci.remote_worker(root)

            self.assertEqual(return_code, 1)
            run_task.assert_not_called()
            status = remote_ci.read_json(root / "run-status.json")
            self.assertEqual(status["tasks"], {})
            self.assertRegex(status["failure_reason"], "sentinel|content")

    def test_remote_doctor_rejects_symlink_at_bound_content_root(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            real_dataset = root / "real-dataset"
            for relative in (
                "mav0/cam0/data.csv",
                "mav0/cam1/data.csv",
                "mav0/imu0/data.csv",
                "dso/camchain.yaml",
                "dso/imu_config.yaml",
                "dso/gt_imu.csv",
            ):
                path = real_dataset / "room1" / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("fixture\n", encoding="utf-8")
            content_id = "sha256:" + "a" * 64
            manifest: dict[str, object] = {"archives": {}, "calibration": {}}
            manifest_id = remote_ci._canonical_json_id(manifest)
            remote_ci.atomic_write_json(
                real_dataset / ".phad-dataset.json",
                {
                    "dataset_manifest_id": manifest_id,
                    "dataset_manifest": manifest,
                    "dataset_content_id": content_id,
                },
            )
            bound_dataset = root / "by-content" / "sha256" / ("a" * 64)
            bound_dataset.parent.mkdir(parents=True)
            bound_dataset.symlink_to(real_dataset, target_is_directory=True)

            fake_bin = root / "bin"
            fake_bin.mkdir()
            docker = fake_bin / "docker"
            docker.write_text(
                "#!/usr/bin/env python3\n"
                "import json, sys\n"
                "if sys.argv[1] == 'version':\n"
                "    print(json.dumps({'Client': {'Version': 'test'}, "
                "'Server': {'Version': 'test'}}))\n"
                "elif sys.argv[1] == 'info':\n"
                "    print(json.dumps({'NCPU': 1, 'MemTotal': 1, "
                "'DockerRootDir': '/tmp', 'SecurityOptions': []}))\n",
                encoding="utf-8",
            )
            docker.chmod(0o755)

            class LocalSsh:
                calls = 0

                def run(
                    self,
                    command: list[str],
                    *,
                    check: bool = True,
                    capture_output: bool = True,
                ) -> subprocess.CompletedProcess[str]:
                    self.calls += 1
                    environment = dict(os.environ)
                    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
                    return subprocess.run(
                        command,
                        check=check,
                        capture_output=capture_output,
                        text=True,
                        env=environment,
                    )

            definition = {
                "name": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "dataset_format": "tum-vi",
                "dataset_root": str(root),
                "sequences": ("room1",),
                "dataset_manifest": manifest,
                "dataset_manifest_id": manifest_id,
            }
            ssh = LocalSsh()
            with mock.patch.object(
                remote_ci, "profile_definition", return_value=definition
            ):
                with self.assertRaisesRegex(
                    remote_ci.RemoteCiError, "symlink|real directory"
                ):
                    remote_ci.remote_doctor(
                        ssh,
                        remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                        dataset_root=bound_dataset,
                        dataset_content_id=content_id,
                    )

            self.assertEqual(ssh.calls, 1)

    def test_worker_rejects_valid_dataset_outside_bound_content_slot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            store = root / "store"
            alternate = root / "alternate"
            alternate.mkdir()
            definition = remote_ci.profile_definition(
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE
            )
            definition = {**definition, "dataset_root": str(store)}
            tree = remote_ci.build_tumvi_dataset_manifest(
                alternate,
                archive_records=definition["dataset_manifest"]["archives"],
                calibration=definition["dataset_manifest"]["calibration"],
            )
            receipt = remote_ci._tum_vi_stage_receipt(definition, tree)
            remote_ci.atomic_write_json(
                alternate / ".phad-dataset.json", receipt
            )
            (alternate / ".phad-dataset.json").chmod(0o444)
            alternate.chmod(0o555)
            run = {
                "schema_version": remote_ci.SCHEMA_VERSION,
                "run_id": "wrong-dataset-slot",
                "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "run_root": str(root),
                "dataset_root": str(alternate),
                "dataset_content_id": receipt["dataset_content_id"],
                "dataset_manifest_id": definition["dataset_manifest_id"],
            }
            remote_ci.atomic_write_json(root / "run.json", run)
            try:
                with mock.patch.object(
                    remote_ci, "profile_definition", return_value=definition
                ), mock.patch.object(
                    remote_ci, "_remote_worker_impl", return_value=0
                ) as worker_impl, mock.patch.object(
                    remote_ci,
                    "_create_results_archive",
                    return_value=(root / "results.tar.zst", "c" * 64),
                ):
                    return_code = remote_ci.remote_worker(root)

                self.assertEqual(return_code, 1)
                worker_impl.assert_not_called()
                status = remote_ci.read_json(root / "run-status.json")
                self.assertRegex(status["failure_reason"], "dataset root|content")
            finally:
                alternate.chmod(0o755)
                (alternate / ".phad-dataset.json").chmod(0o644)


class TumViRemoteStageCliTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.local_root = Path(self.temporary.name)
        self.fixture = ArchiveFixture(self.local_root)
        self.fixture.write()
        archive = self.fixture.expected()
        manifest = {
            "schema_version": 1,
            "dataset": "test-room1",
            "archives": {"room1": archive},
            "calibration": {
                "camchain_sha256": archive["camchain_sha256"],
                "imu_config_sha256": archive["imu_config_sha256"],
            },
        }
        self.definition = {
            "name": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            "dataset_root": "/remote/tumvi-store",
            "dataset_manifest": manifest,
            "dataset_manifest_id": remote_ci._canonical_json_id(manifest),
        }
        self.content_id = "sha256:" + "c" * 64
        self.receipt = {
            "schema_version": 1,
            "profile": remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            "dataset_manifest_id": self.definition["dataset_manifest_id"],
            "dataset_manifest": manifest,
            "dataset_content_id": self.content_id,
            "dataset_tree_manifest": {
                "schema_version": 1,
                "archives": {},
                "calibration": {},
                "entries": [],
                "dataset_content_id": self.content_id,
            },
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_parser_exposes_stage_and_verify_without_current_pointer(self) -> None:
        parser = remote_ci.build_parser()
        stage = parser.parse_args(
            [
                "stage-dataset",
                "--profile",
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "--local-root",
                str(self.local_root),
            ]
        )
        verify = parser.parse_args(
            [
                "verify-dataset",
                "--profile",
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                "--dataset-content-id",
                self.content_id,
                "--json",
            ]
        )
        self.assertEqual(stage.local_root, self.local_root)
        self.assertEqual(verify.dataset_content_id, self.content_id)
        self.assertTrue(verify.json)

    def test_local_archive_preflight_fails_before_opening_ssh(self) -> None:
        self.fixture.archive.write_bytes(b"wrong")
        with mock.patch.object(
            remote_ci, "profile_definition", return_value=self.definition
        ), mock.patch.object(remote_ci, "VerifiedSsh") as ssh:
            with self.assertRaisesRegex(remote_ci.RemoteCiError, "size|sha256"):
                remote_ci.stage_remote_tumvi_dataset(
                    self.local_root,
                    profile=remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                )
        ssh.assert_not_called()

    def test_stage_uploads_content_addressed_inputs_and_verifies_downloaded_receipt(
        self,
    ) -> None:
        fake_ssh = mock.MagicMock()
        fake_ssh.__enter__.return_value = fake_ssh
        fake_ssh.__exit__.return_value = None
        fake_ssh.run.return_value = subprocess.CompletedProcess(
            [], 0, stdout=json.dumps(self.receipt), stderr=""
        )

        def download(_: Path, local: Path) -> None:
            local.write_text(json.dumps(self.receipt, sort_keys=True) + "\n")

        fake_ssh.download.side_effect = download
        uploaded: list[tuple[Path, Path, str]] = []
        archive_uploads: list[tuple[Path, Path, str]] = []

        def upload_exact(_: object, local: Path, remote: Path, digest: str) -> None:
            uploaded.append((local, remote, digest))

        def upload_archive(
            _: object, local: Path, remote: Path, digest: str
        ) -> None:
            archive_uploads.append((local, remote, digest))

        with mock.patch.object(
            remote_ci, "profile_definition", return_value=self.definition
        ), mock.patch.object(
            remote_ci, "VerifiedSsh", return_value=fake_ssh
        ), mock.patch.object(
            remote_ci, "upload_exact", side_effect=upload_exact
        ), mock.patch.object(
            remote_ci,
            "upload_tumvi_archive_part",
            side_effect=upload_archive,
        ), mock.patch.object(
            remote_ci, "_remote_file_sha256", return_value=None
        ):
            result = remote_ci.stage_remote_tumvi_dataset(
                self.local_root,
                profile=remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
            )

        self.assertEqual(result, self.receipt)
        archive_sha = str(self.fixture.expected()["sha256"])
        remote_paths = [remote.as_posix() for _, remote, _ in uploaded]
        self.assertEqual(
            [remote.as_posix() for _, remote, _ in archive_uploads],
            [f"/remote/tumvi-store/incoming/{archive_sha}.part"],
        )
        self.assertTrue(any(path.endswith("/remote_ci.py") for path in remote_paths))
        self.assertTrue(any(path.endswith("/dataset-manifest.json") for path in remote_paths))
        self.assertFalse(any(path.endswith(".part") for path in remote_paths))
        stage_calls = [
            call.args[0]
            for call in fake_ssh.run.call_args_list
            if "_remote-stage-tumvi-dataset" in call.args[0]
        ]
        self.assertEqual(len(stage_calls), 1)
        self.assertNotIn("current", " ".join(stage_calls[0]))
        receipt_hex = self.content_id.removeprefix("sha256:")
        fake_ssh.download.assert_called_once_with(
            Path(f"/remote/tumvi-store/receipts/sha256/{receipt_hex}.json"),
            mock.ANY,
        )

    def test_archive_part_upload_is_idempotent(self) -> None:
        fake_ssh = mock.MagicMock()
        digest = "a" * 64
        partial = Path(f"/remote/store/incoming/{digest}.part")
        with mock.patch.object(
            remote_ci, "_remote_file_sha256", return_value=digest
        ):
            remote_ci.upload_tumvi_archive_part(
                fake_ssh, self.fixture.archive, partial, digest
            )

        fake_ssh.upload.assert_not_called()
        fake_ssh.run.assert_not_called()

    def test_archive_part_upload_interruption_preserves_exact_cleanup_target(
        self,
    ) -> None:
        fake_ssh = mock.MagicMock()
        fake_ssh.run.return_value = subprocess.CompletedProcess([], 0, "", "")
        fake_ssh.upload.side_effect = OSError("injected transfer interruption")
        digest = "a" * 64
        partial = Path(f"/remote/store/incoming/{digest}.part")
        with mock.patch.object(
            remote_ci, "_remote_file_sha256", return_value=None
        ):
            with self.assertRaisesRegex(
                remote_ci.RemoteCiError, re.escape(str(partial))
            ):
                remote_ci.upload_tumvi_archive_part(
                    fake_ssh, self.fixture.archive, partial, digest
                )

        fake_ssh.upload.assert_called_once_with(self.fixture.archive, partial)
        self.assertNotIn(".partial-", str(fake_ssh.upload.call_args))

    def test_archive_part_upload_race_fails_without_overwriting(self) -> None:
        fake_ssh = mock.MagicMock()
        fake_ssh.run.side_effect = (
            subprocess.CompletedProcess([], 0, "", ""),
            subprocess.CompletedProcess([], 1, "", "already exists"),
        )
        digest = "a" * 64
        partial = Path(f"/remote/store/incoming/{digest}.part")
        with mock.patch.object(
            remote_ci,
            "_remote_file_sha256",
            side_effect=(None, "b" * 64),
        ):
            with self.assertRaisesRegex(
                remote_ci.RemoteCiError, re.escape(str(partial))
            ):
                remote_ci.upload_tumvi_archive_part(
                    fake_ssh, self.fixture.archive, partial, digest
                )

        fake_ssh.upload.assert_not_called()

    def test_verify_invokes_remote_full_tree_check_at_bound_content_root(self) -> None:
        fake_ssh = mock.MagicMock()
        fake_ssh.__enter__.return_value = fake_ssh
        fake_ssh.__exit__.return_value = None
        fake_ssh.run.return_value = subprocess.CompletedProcess(
            [], 0, stdout=json.dumps(self.receipt), stderr=""
        )
        with mock.patch.object(
            remote_ci, "profile_definition", return_value=self.definition
        ), mock.patch.object(
            remote_ci, "VerifiedSsh", return_value=fake_ssh
        ), mock.patch.object(remote_ci, "upload_exact"):
            result = remote_ci.verify_remote_tumvi_dataset(
                remote_ci.TUM_VI_CONFIRMATORY_PROFILE,
                self.content_id,
            )

        self.assertEqual(result, self.receipt)
        commands = [call.args[0] for call in fake_ssh.run.call_args_list]
        verify = next(
            command
            for command in commands
            if "_remote-verify-tumvi-dataset" in command
        )
        text = " ".join(verify)
        self.assertIn(
            f"/remote/tumvi-store/by-content/sha256/{'c' * 64}", text
        )
        self.assertNotIn("current", text)


if __name__ == "__main__":
    unittest.main()

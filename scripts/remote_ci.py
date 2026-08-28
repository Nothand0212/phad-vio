#!/usr/bin/env python3
"""局域网服务器上的隔离 Remote CI / EuRoC 编排入口。"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import datetime as dt
import hashlib
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
REMOTE_HOST = "192.168.110.119"
REMOTE_USER = "lin"
REMOTE_TARGET = f"{REMOTE_USER}@{REMOTE_HOST}"
REMOTE_FINGERPRINT = (
    "SHA256:gVOFWNnhMg035iardU+Z8GxRKunxHIp3ZQrjhgY/9XE"
)
REMOTE_ROOT = Path("/home/lin/Projects/tigerfish")
REMOTE_DATA_ROOT = Path("/home/lin/data/euroc/native")
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

SNAPSHOT_EXCLUDED_PREFIXES = (
    PurePosixPath(".codex"),
    PurePosixPath("artifacts/remote-ci"),
)
TERMINAL_TASK_STATES = {"passed", "warning", "failed", "blocked"}
TERMINAL_RUN_STATES = {"passed", "warning", "failed"}
SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


class RemoteCiError(RuntimeError):
    """可向 operator 直接报告的 Remote CI 错误。"""


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
            ["ssh-keyscan", "-T", "5", "-t", "ed25519", REMOTE_HOST],
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
        if found != {REMOTE_FINGERPRINT}:
            raise RemoteCiError(
                "remote ED25519 fingerprint verification failed: "
                f"expected {REMOTE_FINGERPRINT}, got {sorted(found)}"
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
            REMOTE_TARGET,
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
                f"{REMOTE_TARGET}:{remote}",
            ],
            capture_output=False,
        )

    def download(self, remote: Path, local: Path) -> None:
        local.parent.mkdir(parents=True, exist_ok=True)
        run_command(
            [
                "scp",
                *self._options(),
                f"{REMOTE_TARGET}:{remote}",
                str(local),
            ],
            capture_output=False,
        )


def remote_doctor(ssh: VerifiedSsh) -> dict[str, Any]:
    probe = """
import json
import os
from pathlib import Path
import shutil
import subprocess

root = Path('/home/lin/data/euroc/native')
sequences = %s
required = (
    'mav0/cam0/data.csv', 'mav0/cam0/sensor.yaml',
    'mav0/cam1/data.csv', 'mav0/cam1/sensor.yaml',
    'mav0/imu0/data.csv', 'mav0/imu0/sensor.yaml',
    'mav0/state_groundtruth_estimate0/data.csv',
)
missing = {}
for sequence in sequences:
    absent = [item for item in required if not os.access(root / sequence / item, os.R_OK)]
    if absent:
        missing[sequence] = absent
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
    'dataset_root': str(root),
    'dataset_missing': missing,
}, sort_keys=True))
""" % (repr(SEQUENCES),)
    result = ssh.run(["python3", "-c", probe])
    try:
        value = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RemoteCiError(f"remote doctor returned invalid JSON: {error}") from error
    if value.get("architecture") != "x86_64":
        raise RemoteCiError(f"unsupported remote architecture: {value.get('architecture')}")
    if value.get("dataset_missing"):
        raise RemoteCiError(
            f"remote EuRoC dataset is incomplete: {value['dataset_missing']}"
        )
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
            str(image["tag"]),
            "/bin/bash",
            "-lc",
            "/usr/local/bin/phad-remote-ci-smoke >/dev/null && "
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
        if ready.get("source_id") != source["source_id"]:
            raise RemoteCiError(f"remote source sentinel mismatch: {source_root}")
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


def _new_run_id(source_id: str) -> str:
    source_short = source_id.removeprefix("sha256:")[:12]
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{PROFILE}-{source_short}-{secrets.token_hex(3)}"


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


def start_remote_run(root: Path, profile: str) -> dict[str, Any]:
    if profile != PROFILE:
        raise RemoteCiError(f"unsupported profile: {profile}")
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
            doctor = remote_doctor(ssh)
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
            run_id = _new_run_id(str(source["source_id"]))
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
                    "target": REMOTE_TARGET,
                    "host_fingerprint": REMOTE_FINGERPRINT,
                    "uid": doctor["uid"],
                    "gid": doctor["gid"],
                    "architecture": doctor["architecture"],
                },
                "dataset_root": str(REMOTE_DATA_ROOT),
                "sequences": list(SEQUENCES),
                "max_parallel_tasks": 8,
                "build_parallelism": 16,
                "unit_parallelism": 8,
                "record_only": True,
            }
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
        f"- Profile：`{run['profile']}`（EuRoC 11/11 record-only）",
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
            "## EuRoC 指标",
            "",
            "| 序列 | bench 状态 | ATE m | RPE m | completion | coverage | segments | reanchors | 段内 RMS m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for sequence in SEQUENCES:
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


def _stderr_failure(path: Path) -> str:
    if not path.exists():
        return "task exited without stderr"
    lines = [line.strip() for line in path.read_text(
        encoding="utf-8", errors="replace"
    ).splitlines() if line.strip()]
    return lines[-1][:500] if lines else "task returned a non-zero status"


def classify_benchmark_task(
    return_code: int, benchmark: dict[str, Any]
) -> tuple[str, str | None]:
    benchmark_status = benchmark.get("status")
    warnings = benchmark.get("warnings") or []
    reason = str(warnings[-1]) if warnings else None
    if return_code != 0:
        return "failed", reason
    if benchmark_status == "completed":
        return "passed", None
    if benchmark_status in {
        "completed_with_failures",
        "completed_with_warnings",
    }:
        return "warning", None
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
            run["image"]["tag"],
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

    if kind == "euroc":
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
            task_state, benchmark_reason = classify_benchmark_task(
                return_code, benchmark
            )
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
    return "\n".join(
        (
            "set -euo pipefail",
            "umask 022",
            "cmake -E copy_directory /shared-build /task/build",
            f"ctest --test-dir /task/build --output-on-failure -L unit --parallel {parallelism}",
        )
    )


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


def _remote_worker_impl(run_root: Path) -> int:
    run = read_json(run_root / "run.json")
    if Path(str(run["run_root"])).resolve() != run_root.resolve():
        raise RemoteCiError("run.json run_root does not match worker argument")
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
    shared_build = run_root / "tasks" / "build" / "build"
    if build["state"] != "passed":
        reason = f"shared build failed (rc={build.get('return_code')})"
        _write_blocked_task(run, "unit", "unit", reason)
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
        jobs: list[tuple[str, str, str, int, str, Sequence[tuple[Path, str, bool]]]] = [
            (
                "unit",
                "unit",
                _unit_script(run),
                8,
                "16g",
                ((shared_build, "/shared-build", True),),
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
        return _remote_worker_impl(run_root)
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


def _command_start(arguments: argparse.Namespace) -> int:
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

    start = subparsers.add_parser("start", help="构建镜像、上传快照并启动后台 run")
    start.add_argument("--profile", default=PROFILE, choices=(PROFILE,))
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

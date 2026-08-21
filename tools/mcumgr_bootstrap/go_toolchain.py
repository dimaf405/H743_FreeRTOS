"""Pinned Go toolchain discovery, download, and extraction."""

from __future__ import annotations

import hashlib
import os
import pathlib
import platform
import re
import shutil
import subprocess
import tarfile
import tempfile
import urllib.error
import urllib.request
import zipfile
from typing import NamedTuple

from .errors import BootstrapError


GO_VERSION = "1.24.6"
GO_MINIMUM_VERSION = (1, 18)


class GoArchive(NamedTuple):
    filename: str
    sha256: str
    size: int


GO_ARCHIVES = {
    ("linux", "amd64"): GoArchive(
        "go1.24.6.linux-amd64.tar.gz",
        "bbca37cc395c974ffa4893ee35819ad23ebb27426df87af92e93a9ec66ef8712",
        78_583_176,
    ),
    ("windows", "amd64"): GoArchive(
        "go1.24.6.windows-amd64.zip",
        "4fbc8af2cfca9e5059019b5150a426eb78e1e57718bf08f0e52b1c942a2782bf",
        87_261_212,
    ),
}

GO_DOWNLOAD_BASES = (
    "https://mirrors.aliyun.com/golang",
    "https://golang.google.cn/dl",
    "https://go.dev/dl",
)


def host_platform() -> tuple[str, str]:
    system = platform.system().casefold()
    machine = platform.machine().casefold()
    system_name = {"linux": "linux", "windows": "windows"}.get(system)
    architecture = {
        "amd64": "amd64",
        "x86_64": "amd64",
    }.get(machine)
    if system_name is None or architecture is None:
        raise BootstrapError(
            f"automatic mcumgr bootstrap does not support {system}/{machine}; "
            "set MCUMGR=/absolute/path/to/mcumgr"
        )
    return system_name, architecture


def go_version(executable: pathlib.Path, host_os: str, host_arch: str) -> bool:
    try:
        version = subprocess.run(
            [str(executable), "version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
            text=True,
        )
        environment = subprocess.run(
            [str(executable), "env", "GOOS", "GOARCH"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    match = re.search(r"\bgo([0-9]+)\.([0-9]+)(?:\.[0-9]+)?\b", version.stdout)
    go_host = environment.stdout.splitlines()
    return (
        version.returncode == 0
        and environment.returncode == 0
        and match is not None
        and (int(match.group(1)), int(match.group(2))) >= GO_MINIMUM_VERSION
        and go_host == [host_os, host_arch]
    )


def cached_go_path(
    cache_root: pathlib.Path, host_os: str, host_arch: str
) -> pathlib.Path:
    executable = "go.exe" if host_os == "windows" else "go"
    return (
        cache_root
        / "go"
        / f"go{GO_VERSION}"
        / f"{host_os}-{host_arch}"
        / "go"
        / "bin"
        / executable
    )


def download_go_archive(archive: GoArchive, destination: pathlib.Path) -> None:
    errors: list[str] = []
    for base in GO_DOWNLOAD_BASES:
        url = f"{base}/{archive.filename}"
        digest = hashlib.sha256()
        downloaded = 0
        try:
            print(f"Downloading pinned Go toolchain from {url} ...", flush=True)
            request = urllib.request.Request(
                url, headers={"User-Agent": "dima-rover-host-tools/1"}
            )
            with urllib.request.urlopen(request, timeout=60) as response:
                with destination.open("wb") as output:
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        output.write(chunk)
                        digest.update(chunk)
                        downloaded += len(chunk)
        except (OSError, urllib.error.URLError) as error:
            destination.unlink(missing_ok=True)
            errors.append(f"{url}: {error}")
            continue

        actual_digest = digest.hexdigest()
        if downloaded != archive.size or actual_digest != archive.sha256:
            destination.unlink(missing_ok=True)
            errors.append(
                f"{url}: expected {archive.size} bytes and SHA-256 "
                f"{archive.sha256}, got {downloaded} bytes and {actual_digest}"
            )
            continue
        return

    raise BootstrapError("unable to download Go toolchain:\n  " + "\n  ".join(errors))


def validate_archive_members(
    destination: pathlib.Path, member_names: list[str]
) -> None:
    resolved_destination = destination.resolve()
    for member_name in member_names:
        resolved_member = (destination / member_name).resolve()
        if (
            resolved_member != resolved_destination
            and resolved_destination not in resolved_member.parents
        ):
            raise BootstrapError(f"unsafe path in Go archive: {member_name}")


def unpack_go_archive(archive: pathlib.Path, destination: pathlib.Path) -> None:
    if archive.name.endswith(".tar.gz"):
        with tarfile.open(archive, "r:gz") as package:
            validate_archive_members(destination, [item.name for item in package])
            package.extractall(destination)
    elif archive.name.endswith(".zip"):
        with zipfile.ZipFile(archive) as package:
            validate_archive_members(destination, package.namelist())
            package.extractall(destination)
    else:
        raise BootstrapError(f"unsupported Go archive format: {archive.name}")


def provision_go(
    cache_root: pathlib.Path, host_os: str, host_arch: str
) -> pathlib.Path:
    system_go = shutil.which("go")
    if system_go is not None:
        system_path = pathlib.Path(system_go)
        if go_version(system_path, host_os, host_arch):
            return system_path

    cached_path = cached_go_path(cache_root, host_os, host_arch)
    if cached_path.is_file() and go_version(cached_path, host_os, host_arch):
        return cached_path

    distribution = GO_ARCHIVES.get((host_os, host_arch))
    if distribution is None:
        raise BootstrapError(
            f"no pinned Go archive is available for {host_os}/{host_arch}; "
            "set MCUMGR=/absolute/path/to/mcumgr"
        )

    destination = cached_path.parents[1]
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".go-bootstrap-", dir=destination.parent
    ) as temporary_name:
        temporary = pathlib.Path(temporary_name)
        archive_path = temporary / distribution.filename
        unpack_root = temporary / "unpacked"
        unpack_root.mkdir()
        download_go_archive(distribution, archive_path)
        unpack_go_archive(archive_path, unpack_root)
        unpacked_go = unpack_root / "go"
        if not unpacked_go.is_dir():
            raise BootstrapError("Go archive did not contain the expected go directory")

        if destination.exists():
            if cached_path.is_file() and go_version(cached_path, host_os, host_arch):
                return cached_path
            shutil.rmtree(destination)
        os.replace(unpacked_go, destination)

    if not go_version(cached_path, host_os, host_arch):
        raise BootstrapError("the cached Go toolchain failed its version check")
    return cached_path

#!/usr/bin/env python3
"""Provision a pinned Apache mcumgr CLI in the shared host-tools cache."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request
import zipfile
from typing import NamedTuple


GO_VERSION = "1.24.6"
GO_MINIMUM_VERSION = (1, 18)
MCUMGR_MODULE = "github.com/apache/mynewt-mcumgr-cli"
MCUMGR_PACKAGE = "./mcumgr"
MCUMGR_VERSION = "v0.0.0-20221004073047-5c56bd24066c"
NEWTMGR_MODULE = "mynewt.apache.org/newtmgr"
NEWTMGR_VERSION = "v0.0.0-20201028150837-60b2da78788c"
MCUMGR_BUILD_REVISION = "dima-usb-cdc-v2"
MCUMGR_VERSION_STRING = f"0.0.0-dev+{MCUMGR_BUILD_REVISION}"
DEFAULT_GOPROXY = "https://goproxy.cn,direct"

NEWTMGR_SERIAL_XPORT_SHA256 = (
    "1ea559513ae11a658bc571cc25793995e1528130954a7dba6e7a97a4d050e942"
)
NEWTMGR_IMAGE_XACT_SHA256 = (
    "fea8b780eff391207b6c1b4f7cc90370271d4bc87b98d3b2275232fdd860ce42"
)
NEWTMGR_SERIAL_DELAY = "\t\t\ttime.Sleep(20 * time.Millisecond)\n"
NEWTMGR_USB_CDC_DELAY = (
    "\t\t\t// Real UART transports need pacing between 124-byte NLIP frames.\n"
    "\t\t\t// Dima Rover uses USB CDC at a virtual baud of 921600 or higher,\n"
    "\t\t\t// where this delay only throttles firmware upload throughput.\n"
    "\t\t\tif sx.cfg.Baud < 921600 {\n"
    "\t\t\t\ttime.Sleep(20 * time.Millisecond)\n"
    "\t\t\t}\n"
)
NEWTMGR_FIXED_FRAME = "\t\twriteLen := util.Min(124, totlen-written)\n"
NEWTMGR_CONFIGURED_FRAME = (
    "\t\twriteLen := util.Min(sx.cfg.Mtu-4, totlen-written)\n"
)
NEWTMGR_UNALIGNED_CHUNK_RETURN = "\treturn chunklen, nil\n"
NEWTMGR_ALIGNED_CHUNK_RETURN = (
    "\t// STM32H743 flash words are 32 bytes.  Align every non-final chunk so\n"
    "\t// MCUboot acknowledges exactly the offset that the upload window sent.\n"
    "\tif chunklen < len(data)-off {\n"
    "\t\tchunklen -= chunklen % 32\n"
    "\t}\n\n"
    "\treturn chunklen, nil\n"
)


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


class BootstrapError(RuntimeError):
    """A host-tool bootstrap failure with an actionable diagnostic."""


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


def cached_go_path(cache_root: pathlib.Path, host_os: str, host_arch: str) -> pathlib.Path:
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


def provision_go(cache_root: pathlib.Path, host_os: str, host_arch: str) -> pathlib.Path:
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


def mcumgr_works(executable: pathlib.Path) -> bool:
    try:
        completed = subprocess.run(
            [str(executable), "version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (
        completed.returncode == 0
        and completed.stdout.strip() == f"mcumgr {MCUMGR_VERSION_STRING}"
    )


def go_module_directory(
    go: pathlib.Path, environment: dict[str, str], module: str, version: str
) -> pathlib.Path:
    try:
        completed = subprocess.run(
            [str(go), "mod", "download", "-json", f"{module}@{version}"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=300,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BootstrapError(f"unable to download Go module {module}: {error}") from error

    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip()
        raise BootstrapError(
            f"unable to download Go module {module}@{version}: {details}"
        )
    try:
        metadata = json.loads(completed.stdout)
        directory = pathlib.Path(metadata["Dir"])
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise BootstrapError(
            f"Go returned malformed module metadata for {module}@{version}"
        ) from error
    if not directory.is_dir():
        raise BootstrapError(f"downloaded Go module directory is missing: {directory}")
    return directory


def replace_once(path: pathlib.Path, old: str, new: str, description: str) -> None:
    contents = path.read_text(encoding="utf-8")
    if contents.count(old) != 1:
        raise BootstrapError(
            f"cannot apply {description}; expected source context was not unique in {path}"
        )
    path.write_text(contents.replace(old, new), encoding="utf-8")


def prepare_mcumgr_sources(
    go: pathlib.Path, environment: dict[str, str], destination: pathlib.Path
) -> pathlib.Path:
    mcumgr_source = go_module_directory(
        go, environment, MCUMGR_MODULE, MCUMGR_VERSION
    )
    newtmgr_source = go_module_directory(
        go, environment, NEWTMGR_MODULE, NEWTMGR_VERSION
    )

    mcumgr_build = destination / "mcumgr"
    newtmgr_build = destination / "newtmgr"
    shutil.copytree(mcumgr_source, mcumgr_build, copy_function=shutil.copyfile)
    shutil.copytree(newtmgr_source, newtmgr_build, copy_function=shutil.copyfile)

    serial_xport = newtmgr_build / "nmxact" / "nmserial" / "serial_xport.go"
    digest = hashlib.sha256(serial_xport.read_bytes()).hexdigest()
    if digest != NEWTMGR_SERIAL_XPORT_SHA256:
        raise BootstrapError(
            "Apache newtmgr serial source failed its pinned SHA-256 check: "
            f"expected {NEWTMGR_SERIAL_XPORT_SHA256}, got {digest}"
        )
    replace_once(
        serial_xport,
        NEWTMGR_SERIAL_DELAY,
        NEWTMGR_USB_CDC_DELAY,
        "Dima USB CDC pacing patch",
    )
    replace_once(
        serial_xport,
        NEWTMGR_FIXED_FRAME,
        NEWTMGR_CONFIGURED_FRAME,
        "configurable serial-frame patch",
    )

    image_xact = newtmgr_build / "nmxact" / "xact" / "image.go"
    digest = hashlib.sha256(image_xact.read_bytes()).hexdigest()
    if digest != NEWTMGR_IMAGE_XACT_SHA256:
        raise BootstrapError(
            "Apache newtmgr image-upload source failed its pinned SHA-256 check: "
            f"expected {NEWTMGR_IMAGE_XACT_SHA256}, got {digest}"
        )
    replace_once(
        image_xact,
        NEWTMGR_UNALIGNED_CHUNK_RETURN,
        NEWTMGR_ALIGNED_CHUNK_RETURN,
        "STM32H743 upload-chunk alignment patch",
    )

    replace_once(
        mcumgr_build / "mcumgr" / "mcumgr.go",
        'VersionString: "0.0.0-dev",',
        f'VersionString: "{MCUMGR_VERSION_STRING}",',
        "mcumgr build identity patch",
    )
    go_mod = mcumgr_build / "go.mod"
    with go_mod.open("a", encoding="utf-8") as output:
        output.write(f"\nreplace {NEWTMGR_MODULE} => ../newtmgr\n")
    return mcumgr_build


def ensure_mcumgr(
    cache_root: pathlib.Path,
    target_windows: bool,
    announce_cache_hit: bool = True,
) -> pathlib.Path:
    host_os, host_arch = host_platform()
    target_os = "windows" if target_windows else host_os
    executable_name = "mcumgr.exe" if target_os == "windows" else "mcumgr"
    destination = (
        cache_root
        / "mcumgr"
        / f"{MCUMGR_VERSION.removeprefix('v')}-{MCUMGR_BUILD_REVISION}"
        / f"{target_os}-{host_arch}"
        / executable_name
    )
    if destination.is_file() and mcumgr_works(destination):
        if announce_cache_hit:
            print(f"Using cached Apache mcumgr: {destination}", flush=True)
        return destination

    go = provision_go(cache_root, host_os, host_arch)
    workspace = cache_root / "go-workspace"
    workspace.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.pop("GOBIN", None)
    environment.update(
        {
            "CGO_ENABLED": "0",
            "GOARCH": host_arch,
            "GOOS": target_os,
            "GOPATH": str(workspace),
            "GOTOOLCHAIN": "local",
            "GOPROXY": environment.get("DIMA_GOPROXY")
            or environment.get("GOPROXY")
            or DEFAULT_GOPROXY,
        }
    )

    print(
        f"Building pinned Apache mcumgr {MCUMGR_VERSION} "
        f"({MCUMGR_BUILD_REVISION}) for {target_os}/{host_arch} ...",
        flush=True,
    )
    cache_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".mcumgr-build-", dir=cache_root
    ) as temporary_name:
        build_root = pathlib.Path(temporary_name)
        source_root = prepare_mcumgr_sources(go, environment, build_root)
        built_binary = build_root / executable_name
        try:
            completed = subprocess.run(
                [
                    str(go),
                    "build",
                    "-trimpath",
                    "-o",
                    str(built_binary),
                    MCUMGR_PACKAGE,
                ],
                cwd=source_root,
                env=environment,
                timeout=600,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise BootstrapError(f"unable to build Apache mcumgr: {error}") from error
        if completed.returncode != 0:
            raise BootstrapError(
                f"Apache mcumgr build failed with exit status {completed.returncode}"
            )
        if not built_binary.is_file():
            raise BootstrapError(
                f"Go did not produce the expected binary: {built_binary}"
            )

        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_destination = destination.with_name(
            f".{destination.name}.tmp.{os.getpid()}"
        )
        try:
            shutil.copy2(built_binary, temporary_destination)
            temporary_destination.chmod(0o755)
            os.replace(temporary_destination, destination)
        finally:
            temporary_destination.unlink(missing_ok=True)

    if not mcumgr_works(destination):
        raise BootstrapError("the cached Apache mcumgr failed its version check")
    print(f"Cached Apache mcumgr: {destination}", flush=True)
    return destination


def is_wsl() -> bool:
    try:
        return "microsoft" in pathlib.Path("/proc/sys/kernel/osrelease").read_text(
            encoding="utf-8"
        ).casefold()
    except OSError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--target", choices=("auto", "linux", "windows"), default="auto")
    arguments = parser.parse_args()

    target_windows = is_wsl() if arguments.target == "auto" else arguments.target == "windows"
    executable = ensure_mcumgr(arguments.cache_root.expanduser(), target_windows)
    print(executable)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapError as error:
        print(f"mcumgr bootstrap failed: {error}", file=sys.stderr)
        raise SystemExit(1)

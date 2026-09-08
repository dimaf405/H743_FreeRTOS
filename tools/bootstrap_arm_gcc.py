#!/usr/bin/env python3
"""在 Windows/Linux 本机缓存中准备固定版本的原生 Arm GNU 工具链。"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request
import zipfile
from dataclasses import dataclass


TOOLCHAIN_VERSION = "10.3.1-2.3"
TOOLCHAIN_URL_PREFIXES = (
    "https://ghfast.top/https://github.com/xpack-dev-tools/"
    "arm-none-eabi-gcc-xpack/releases/download/v10.3.1-2.3/",
    "https://gh-proxy.com/https://github.com/xpack-dev-tools/"
    "arm-none-eabi-gcc-xpack/releases/download/v10.3.1-2.3/",
    "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/"
    "releases/download/v10.3.1-2.3/",
)


class BootstrapError(RuntimeError):
    """A reproducible host-toolchain provisioning failure."""


@dataclass(frozen=True)
class ToolchainPackage:
    host: str
    archive: str
    size: int
    sha256: str
    compiler: str


def host_package() -> ToolchainPackage:
    """按 Python 实际主机选择发行包；WSL 的 Linux Python 不下载 Windows exe。"""
    if platform.machine().casefold() not in ("amd64", "x86_64"):
        raise BootstrapError("automatic Arm GNU provisioning currently supports x86_64 hosts")
    system = platform.system().casefold()
    if system == "windows":
        return ToolchainPackage(
            "windows-amd64", "xpack-arm-none-eabi-gcc-10.3.1-2.3-win32-x64.zip",
            194_580_962,
            "169744f784fb04ae10c60bc6a2cd69cff93cff0bf5657e9333776036f347f9c4",
            "arm-none-eabi-gcc.exe",
        )
    if system == "linux":
        # 大小与 SHA-256 对照 xPack 同一 v10.3.1-2.3 release 的 Linux x64 归档及 .sha。
        return ToolchainPackage(
            "linux-amd64", "xpack-arm-none-eabi-gcc-10.3.1-2.3-linux-x64.tar.gz",
            175_646_740,
            "559dcf1c2dfddac513110fe23da0ef254032c09967eaa901f075515d51818719",
            "arm-none-eabi-gcc",
        )
    raise BootstrapError(f"unsupported Arm GNU host: {platform.system()}")


def log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def validate_compiler(executable: pathlib.Path) -> bool:
    if not executable.is_file():
        return False
    try:
        completed = subprocess.run(
            [str(executable), "--version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    first_line = completed.stdout.splitlines()[:1]
    return (
        completed.returncode == 0
        and bool(first_line)
        and "arm-none-eabi-gcc" in first_line[0]
        and "10.3.1" in first_line[0]
    )


def download_archive(destination: pathlib.Path, package: ToolchainPackage) -> None:
    errors: list[str] = []
    for prefix in TOOLCHAIN_URL_PREFIXES:
        url = prefix + package.archive
        digest = hashlib.sha256()
        downloaded = 0
        try:
            log(f"Downloading pinned Arm GNU toolchain from {url} ...")
            request = urllib.request.Request(
                url,
                headers={"User-Agent": "dima-rover-host-tools/1"},
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
        if (
            downloaded != package.size
            or actual_digest != package.sha256
        ):
            destination.unlink(missing_ok=True)
            errors.append(
                f"{url}: expected {package.size} bytes and SHA-256 "
                f"{package.sha256}, got {downloaded} bytes and "
                f"{actual_digest}"
            )
            continue
        return

    raise BootstrapError(
        "unable to download the pinned Arm GNU toolchain:\n  "
        + "\n  ".join(errors)
    )


def validate_archive_members(
    destination: pathlib.Path, member_names: list[str]
) -> None:
    resolved_destination = destination.resolve()
    for member_name in member_names:
        member = pathlib.PurePosixPath(member_name)
        if member.is_absolute() or ".." in member.parts:
            raise BootstrapError(
                f"unsafe path in Arm GNU archive: {member_name}"
            )
        resolved_member = (destination / pathlib.Path(*member.parts)).resolve()
        if (
            resolved_member != resolved_destination
            and resolved_destination not in resolved_member.parents
        ):
            raise BootstrapError(
                f"unsafe path in Arm GNU archive: {member_name}"
            )


def unpack_archive(
    archive: pathlib.Path, destination: pathlib.Path, package: ToolchainPackage
) -> pathlib.Path:
    try:
        if package.archive.endswith(".zip"):
            with zipfile.ZipFile(archive) as source:
                validate_archive_members(destination, source.namelist())
                source.extractall(destination)
        else:
            with tarfile.open(archive, "r:gz") as source:
                members = source.getmembers()
                validate_archive_members(destination, [member.name for member in members])
                for member in members:
                    # Linux 工具链需要保留执行权限和内部链接；链接目标必须仍在解包根内，
                    # 设备/FIFO 等非工具链文件拒绝解包，不能借 tar 覆盖主机其他路径。
                    if member.issym() or member.islnk():
                        base = (destination / member.name).parent if member.issym() else destination
                        target = (base / member.linkname).resolve()
                        if target != destination.resolve() and destination.resolve() not in target.parents:
                            raise BootstrapError(f"unsafe link in Arm GNU archive: {member.name}")
                    elif not member.isfile() and not member.isdir():
                        raise BootstrapError(f"unsupported entry in Arm GNU archive: {member.name}")
                source.extractall(destination)
    except (OSError, zipfile.BadZipFile, tarfile.TarError) as error:
        raise BootstrapError(f"unable to extract Arm GNU archive: {error}") from error
    matches = list(destination.rglob(f"bin/{package.compiler}"))
    if len(matches) != 1:
        raise BootstrapError(
            f"Arm GNU archive did not contain one bin/{package.compiler}"
        )
    return matches[0].parent.parent


def install_toolchain(
    cache_root: pathlib.Path, announce_cache_hit: bool = True
) -> pathlib.Path:
    package = host_package()

    installation = (
        cache_root
        / "arm-none-eabi-gcc"
        / TOOLCHAIN_VERSION
        / package.host
    )
    executable = installation / "bin" / package.compiler
    if validate_compiler(executable):
        if announce_cache_hit:
            log(f"Using cached Arm GNU toolchain: {installation}")
        return installation

    cache_root.mkdir(parents=True, exist_ok=True)
    installation.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".gcc-", dir=cache_root
    ) as temporary_name:
        temporary = pathlib.Path(temporary_name)
        archive = temporary / package.archive
        unpacked = temporary / "unpacked"
        unpacked.mkdir()
        download_archive(archive, package)
        candidate = unpack_archive(archive, unpacked, package)
        if not validate_compiler(candidate / "bin" / package.compiler):
            raise BootstrapError(
                "the extracted Arm GNU compiler failed its version check"
            )

        if installation.exists():
            shutil.rmtree(installation)
        try:
            shutil.copytree(candidate, installation)
        except OSError as error:
            if installation.exists():
                shutil.rmtree(installation, ignore_errors=True)
            raise BootstrapError(
                f"unable to install the Arm GNU toolchain cache: {error}"
            ) from error

    if not validate_compiler(executable):
        raise BootstrapError(
            "the cached Arm GNU compiler failed its final version check"
        )
    log(f"Cached Arm GNU toolchain: {installation}")
    return installation


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--quiet-cache", action="store_true")
    arguments = parser.parse_args()
    installation = install_toolchain(
        arguments.cache_root.expanduser(),
        announce_cache_hit=not arguments.quiet_cache,
    )
    print((installation / "bin").resolve().as_posix())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapError as error:
        print(f"Arm GNU bootstrap failed: {error}", file=sys.stderr)
        raise SystemExit(1)

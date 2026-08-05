#!/usr/bin/env python3
"""Provision the pinned Windows Arm GNU toolchain in the shared host cache."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import platform
import shutil
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
import zipfile


TOOLCHAIN_VERSION = "10.3.1-2.3"
TOOLCHAIN_ARCHIVE = (
    "xpack-arm-none-eabi-gcc-10.3.1-2.3-win32-x64.zip"
)
TOOLCHAIN_ARCHIVE_SIZE = 194_580_962
TOOLCHAIN_ARCHIVE_SHA256 = (
    "169744f784fb04ae10c60bc6a2cd69cff93cff0bf5657e9333776036f347f9c4"
)
TOOLCHAIN_URLS = (
    "https://ghfast.top/https://github.com/xpack-dev-tools/"
    "arm-none-eabi-gcc-xpack/releases/download/v10.3.1-2.3/"
    + TOOLCHAIN_ARCHIVE,
    "https://gh-proxy.com/https://github.com/xpack-dev-tools/"
    "arm-none-eabi-gcc-xpack/releases/download/v10.3.1-2.3/"
    + TOOLCHAIN_ARCHIVE,
    "https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/"
    "releases/download/v10.3.1-2.3/"
    + TOOLCHAIN_ARCHIVE,
)


class BootstrapError(RuntimeError):
    """A reproducible host-toolchain provisioning failure."""


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


def download_archive(destination: pathlib.Path) -> None:
    errors: list[str] = []
    for url in TOOLCHAIN_URLS:
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
            downloaded != TOOLCHAIN_ARCHIVE_SIZE
            or actual_digest != TOOLCHAIN_ARCHIVE_SHA256
        ):
            destination.unlink(missing_ok=True)
            errors.append(
                f"{url}: expected {TOOLCHAIN_ARCHIVE_SIZE} bytes and SHA-256 "
                f"{TOOLCHAIN_ARCHIVE_SHA256}, got {downloaded} bytes and "
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
    archive: pathlib.Path, destination: pathlib.Path
) -> pathlib.Path:
    try:
        with zipfile.ZipFile(archive) as package:
            validate_archive_members(destination, package.namelist())
            package.extractall(destination)
    except (OSError, zipfile.BadZipFile) as error:
        raise BootstrapError(f"unable to extract Arm GNU archive: {error}") from error
    matches = list(destination.rglob("bin/arm-none-eabi-gcc.exe"))
    if len(matches) != 1:
        raise BootstrapError(
            "Arm GNU archive did not contain one bin/arm-none-eabi-gcc.exe"
        )
    return matches[0].parent.parent


def install_toolchain(
    cache_root: pathlib.Path, announce_cache_hit: bool = True
) -> pathlib.Path:
    if platform.system().casefold() != "windows":
        raise BootstrapError(
            "automatic Arm GNU provisioning must run in native Windows Python"
        )

    installation = (
        cache_root
        / "arm-none-eabi-gcc"
        / TOOLCHAIN_VERSION
        / "windows-amd64"
    )
    executable = installation / "bin" / "arm-none-eabi-gcc.exe"
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
        archive = temporary / TOOLCHAIN_ARCHIVE
        unpacked = temporary / "unpacked"
        unpacked.mkdir()
        download_archive(archive)
        candidate = unpack_archive(archive, unpacked)
        if not validate_compiler(candidate / "bin" / "arm-none-eabi-gcc.exe"):
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

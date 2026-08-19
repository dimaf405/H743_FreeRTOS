#!/usr/bin/env python3
"""Provision the pinned pymavlink generator in the shared host cache."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request

sys.dont_write_bytecode = True


class BootstrapError(RuntimeError):
    """A deterministic, actionable generator provisioning failure."""


def load_lock(lock_path: pathlib.Path) -> dict:
    try:
        return json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise BootstrapError(f"unable to read MAVLink lock file: {error}") from error


def file_sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_source(source_root: pathlib.Path, version: str) -> bool:
    required = (
        source_root / "__init__.py",
        source_root / "generator" / "mavgen.py",
        source_root / "generator" / "mavgen_c.py",
        source_root / "generator" / "mavparse.py",
    )
    if not all(path.is_file() for path in required):
        return False
    try:
        version_text = required[0].read_text(encoding="utf-8")
    except OSError:
        return False
    match = re.search(r"__version__\s*=\s*['\"]([^'\"]+)['\"]", version_text)
    return match is not None and match.group(1) == version


def download_archive(archive: dict, destination: pathlib.Path) -> None:
    errors: list[str] = []
    for url in archive["urls"]:
        digest = hashlib.sha256()
        downloaded = 0
        try:
            print(f"Downloading pinned pymavlink from {url} ...", file=sys.stderr)
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

        actual_sha = digest.hexdigest()
        if downloaded != archive["size"] or actual_sha != archive["sha256"]:
            destination.unlink(missing_ok=True)
            errors.append(
                f"{url}: expected {archive['size']} bytes and SHA-256 "
                f"{archive['sha256']}, got {downloaded} bytes and {actual_sha}"
            )
            continue
        return

    raise BootstrapError(
        "unable to download the pinned pymavlink archive:\n  "
        + "\n  ".join(errors)
    )


def safe_extract(archive_path: pathlib.Path, destination: pathlib.Path) -> pathlib.Path:
    resolved_destination = destination.resolve()
    try:
        with tarfile.open(archive_path, "r:gz") as package:
            members = package.getmembers()
            for member in members:
                if member.issym() or member.islnk():
                    raise BootstrapError(
                        f"pymavlink archive contains a link: {member.name}"
                    )
                resolved_member = (destination / member.name).resolve()
                if (
                    resolved_member != resolved_destination
                    and resolved_destination not in resolved_member.parents
                ):
                    raise BootstrapError(
                        f"unsafe path in pymavlink archive: {member.name}"
                    )
            package.extractall(destination)
    except (OSError, tarfile.TarError) as error:
        raise BootstrapError(f"unable to extract pymavlink: {error}") from error

    roots = [path for path in destination.iterdir() if path.is_dir()]
    if len(roots) != 1:
        raise BootstrapError("pymavlink archive did not contain one source root")
    return roots[0]


def provision_pymavlink(
    cache_root: pathlib.Path,
    lock_path: pathlib.Path,
    source_override: pathlib.Path | None = None,
) -> pathlib.Path:
    lock = load_lock(lock_path)
    package = lock["pymavlink"]
    version = package["version"]
    commit = package["commit"]

    if source_override is not None:
        source = source_override.expanduser().resolve()
        if source.name != "pymavlink" or not validate_source(source, version):
            raise BootstrapError(
                f"PYMAVLINK_ROOT must be a package directory named pymavlink "
                f"at version {version}: {source}"
            )
        return source

    installation = cache_root / "pymavlink" / f"{version}-{commit}" / "pymavlink"
    if validate_source(installation, version):
        return installation

    installation.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".pymavlink-", dir=installation.parent
    ) as temporary_name:
        temporary = pathlib.Path(temporary_name)
        archive_path = temporary / package["archive"]["filename"]
        unpack_root = temporary / "unpacked"
        unpack_root.mkdir()
        download_archive(package["archive"], archive_path)
        source = safe_extract(archive_path, unpack_root)
        if not validate_source(source, version):
            raise BootstrapError(
                f"downloaded pymavlink source did not report version {version}"
            )
        candidate = temporary / "source"
        shutil.copytree(source, candidate)
        if installation.exists():
            shutil.rmtree(installation)
        os.replace(candidate, installation)

    if not validate_source(installation, version):
        raise BootstrapError("cached pymavlink failed final validation")
    return installation


def main() -> int:
    script_dir = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lock", type=pathlib.Path, default=script_dir / "mavlink.lock.json"
    )
    parser.add_argument(
        "--cache-root",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--source-root", type=pathlib.Path)
    arguments = parser.parse_args()
    source = provision_pymavlink(
        arguments.cache_root.expanduser(), arguments.lock, arguments.source_root
    )
    print(source.resolve().as_posix())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapError as error:
        print(f"pymavlink bootstrap failed: {error}", file=sys.stderr)
        raise SystemExit(1)

#!/usr/bin/env python3
"""Atomically update a key identity stamp when path or content changes."""

from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import tempfile


def key_identity(key_file: pathlib.Path) -> bytes:
    canonical = key_file.expanduser().resolve(strict=True)
    digest = hashlib.sha256()
    with canonical.open("rb") as key:
        for chunk in iter(lambda: key.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"{canonical}\n{digest.hexdigest()}\n".encode("utf-8")


def stamp_is_current(key_file: pathlib.Path, stamp: pathlib.Path) -> bool:
    try:
        identity = key_identity(key_file)
        return stamp.read_bytes() == identity
    except FileNotFoundError:
        return False


def update_stamp(key_file: pathlib.Path, stamp: pathlib.Path) -> bool:
    identity = key_identity(key_file)
    try:
        if stamp.read_bytes() == identity:
            return False
    except FileNotFoundError:
        pass

    stamp.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=stamp.parent, prefix=f".{stamp.name}.tmp."
    )
    temporary = pathlib.Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(identity)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, stamp)
    finally:
        temporary.unlink(missing_ok=True)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", required=True, type=pathlib.Path)
    parser.add_argument("--stamp", required=True, type=pathlib.Path)
    parser.add_argument(
        "--status",
        action="store_true",
        help="print current or stale without modifying the stamp",
    )
    arguments = parser.parse_args()
    if arguments.status:
        print("current" if stamp_is_current(arguments.key, arguments.stamp) else "stale")
        return 0
    update_stamp(arguments.key, arguments.stamp)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

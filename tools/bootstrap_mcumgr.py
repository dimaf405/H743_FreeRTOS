#!/usr/bin/env python3
"""Provision a pinned Apache mcumgr CLI in the shared host-tools cache."""

from __future__ import annotations

import argparse
import pathlib
import sys

from mcumgr_bootstrap import BootstrapError, ensure_mcumgr


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
    parser.add_argument(
        "--target", choices=("auto", "linux", "windows"), default="auto"
    )
    arguments = parser.parse_args()

    target_windows = (
        is_wsl() if arguments.target == "auto" else arguments.target == "windows"
    )
    executable = ensure_mcumgr(arguments.cache_root.expanduser(), target_windows)
    print(executable)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapError as error:
        print(f"mcumgr bootstrap failed: {error}", file=sys.stderr)
        raise SystemExit(1)

#!/usr/bin/env python3
"""Generate or verify a file-level SHA-256 manifest for vendored sources."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
import sys


class ManifestError(RuntimeError):
    """A vendored source closure is missing, stale or malformed."""


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_files(root: pathlib.Path, output: pathlib.Path) -> dict[str, str]:
    """动态扫描整个上游快照，不在代码里维护脚本、模板或消息列表。"""
    files = {
        path.relative_to(root).as_posix(): sha256(path)
        for path in sorted(root.rglob("*"), key=lambda item: item.as_posix())
        if path.is_file()
        and path.resolve() != output.resolve()
        and "__pycache__" not in path.parts
        and path.suffix != ".pyc"
    }
    if not files:
        raise ManifestError(f"vendored source closure is empty: {root}")
    return files


def document(arguments: argparse.Namespace) -> dict:
    root = arguments.root.resolve()
    if not root.is_dir():
        raise ManifestError(f"vendored source root does not exist: {root}")
    if re.fullmatch(r"[0-9a-f]{40}", arguments.commit) is None:
        raise ManifestError("source commit must be a full 40-character SHA-1")
    return {
        "format": 1,
        "project": arguments.project,
        "commit": arguments.commit,
        "files": source_files(root, arguments.output.resolve()),
    }


def run(arguments: argparse.Namespace) -> None:
    expected = document(arguments)
    output = arguments.output.resolve()
    if arguments.verify:
        try:
            actual = json.loads(output.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ManifestError(f"cannot read source manifest {output}: {error}") from error
        if actual != expected:
            raise ManifestError(f"vendored source manifest is stale: {output}")
        print(f"source manifest verification passed: {output}")
        return

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(expected, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"generated source manifest: {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--project", required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    try:
        run(parse_args())
    except ManifestError as error:
        print(f"source manifest failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

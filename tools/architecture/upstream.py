"""Shared verification for vendored upstream file-level source manifests."""

from __future__ import annotations

import hashlib
import json
import pathlib

from architecture.common import Violation


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_source_manifest(
    root: pathlib.Path,
    expected_project: str,
    expected_commit: str,
    rule: str,
    violations: list[Violation],
) -> dict[str, str] | None:
    """动态核对目录闭包，不在门禁中复制上游文件列表。"""
    manifest_path = root / "SOURCE_MANIFEST.json"
    try:
        document = json.loads(manifest_path.read_text(encoding="utf-8"))
        declared = document["files"]
        if (
            document.get("format") != 1
            or document.get("project") != expected_project
            or document.get("commit") != expected_commit
            or not isinstance(declared, dict)
            or not declared
        ):
            raise TypeError("source identity or files object is invalid")
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        violations.append(Violation(
            manifest_path, 1, rule, f"invalid upstream source manifest: {error}",
        ))
        return None

    actual_paths = {
        path.relative_to(root).as_posix(): path
        for path in root.rglob("*")
        if path.is_file()
        and path != manifest_path
        and "__pycache__" not in path.parts
        and path.suffix != ".pyc"
    }
    if set(actual_paths) != set(declared):
        violations.append(Violation(
            manifest_path, 1, rule,
            "vendored upstream file closure differs from its source manifest",
        ))
        return None
    changed = [
        relative for relative, path in actual_paths.items()
        if not isinstance(declared[relative], str)
        or _sha256(path) != declared[relative]
    ]
    if changed:
        violations.append(Violation(
            root / changed[0], 1, rule,
            f"vendored upstream file hash mismatch: {changed}",
        ))
        return None
    return declared

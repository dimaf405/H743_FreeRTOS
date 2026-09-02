#!/usr/bin/env python3
"""调用锁定的 PX4 v1.17 uORB 生成器并安装官方产物。"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


UPSTREAM_RELATIVE_ROOT = Path("tools/upstream/uorb_v1_17")
PINNED_UPSTREAM_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
SOURCE_MANIFEST_TOOL = (
    Path(__file__).resolve().parents[1] / "generation/source_manifest.py"
)
PASCAL_MESSAGE_NAME = re.compile(r"^[A-Z][A-Za-z0-9]*$")
ORB_DECLARE = re.compile(r"\bORB_DECLARE\(([a-z][a-z0-9_]*)\);")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate uORB topics with the unmodified PX4 v1.17 tools"
    )
    parser.add_argument("--schemas", required=True, type=Path)
    parser.add_argument(
        "--output", type=Path, default=Path("build/generated/uORB")
    )
    parser.add_argument(
        "--compat-output", type=Path, default=Path("build/generated/messages")
    )
    parser.add_argument("--upstream-root", type=Path, default=UPSTREAM_RELATIVE_ROOT)
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def require_file(path: Path, description: str) -> Path:
    resolved = path.resolve()
    if not resolved.is_file():
        raise RuntimeError(f"missing {description}: {path}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repository_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def schema_files(directory: Path) -> list[Path]:
    directory = directory.resolve()
    if not directory.is_dir():
        raise RuntimeError(f"uORB schema directory does not exist: {directory}")
    files = sorted(directory.glob("*.msg"), key=lambda path: path.name)
    if not files:
        raise RuntimeError("uORB schema directory is empty")

    for path in files:
        if not PASCAL_MESSAGE_NAME.fullmatch(path.stem):
            raise RuntimeError(
                f"PX4 uORB schema names must be PascalCase: {path.name}"
            )
        text = path.read_text(encoding="utf-8")
        extension = re.search(r"(?m)^\s*@[A-Za-z_]", text)
        if extension:
            raise RuntimeError(
                f"local uORB schema extension is forbidden: {path.name}:"
                f"{text[:extension.start()].count(chr(10)) + 1}"
            )
    return files


def verify_upstream_source(root: Path) -> None:
    """生成前验证完整上游快照，避免只校验入口脚本而遗漏模板或 helper。"""
    resolved_root = root.resolve()
    verifier = require_file(SOURCE_MANIFEST_TOOL, "source manifest verifier")
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONUTF8"] = "1"
    subprocess.run(
        [
            sys.executable,
            str(verifier),
            "--root",
            str(resolved_root),
            "--output",
            str(resolved_root / "SOURCE_MANIFEST.json"),
            "--project",
            "PX4-Autopilot",
            "--commit",
            PINNED_UPSTREAM_COMMIT,
            "--verify",
        ],
        check=True,
        env=environment,
    )


def upstream_paths(root: Path) -> tuple[Path, Path, Path, list[Path]]:
    root = root.resolve()
    topic_script = require_file(
        root / "Tools/msg/px_generate_uorb_topic_files.py",
        "PX4 uORB generator",
    )
    fields_script = require_file(
        root / "Tools/msg/px_generate_uorb_compressed_fields.py",
        "PX4 compressed uORB fields generator",
    )
    require_file(
        root / "src/lib/heatshrink/heatshrink_encode.py",
        "PX4 heatshrink encoder",
    )
    template_dir = root / "Tools/msg/templates/uorb"
    if not template_dir.is_dir():
        raise RuntimeError(f"missing PX4 uORB templates: {template_dir}")
    dependencies = sorted(
        path
        for dependency_root in (
            root / "Tools/msg",
            root / "src/lib/heatshrink",
        )
        for path in dependency_root.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    )
    if not dependencies:
        raise RuntimeError("PX4 uORB upstream dependency closure is empty")
    return topic_script, fields_script, template_dir, dependencies


def run_upstream(
    topic_script: Path,
    fields_script: Path,
    template_dir: Path,
    schemas: list[Path],
    topics_dir: Path,
) -> None:
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONUTF8"] = "1"
    for mode in ("--headers", "--sources", "--json"):
        # 上游脚本和 EmPy 模板原样执行；本地层不解析字段、不计算布局或消息 hash。
        subprocess.run(
            [
                sys.executable,
                str(topic_script),
                mode,
                "-f",
                *(str(path) for path in schemas),
                "-e",
                str(template_dir),
                "-o",
                str(topics_dir),
            ],
            check=True,
            env=environment,
        )

    json_files = sorted(topics_dir.glob("*.json"), key=lambda path: path.name)
    if not json_files:
        raise RuntimeError("PX4 uORB generator produced no JSON field contracts")
    # ULog F 消息必须和 Topic 头、ID、hash 来自同一次官方生成；这里直接调用
    # PX4 原始压缩字段脚本，不在本地维护字段 token、依赖或消息清单。
    subprocess.run(
        [
            sys.executable,
            str(fields_script),
            "-f",
            *(str(path) for path in json_files),
            "--source-output-file",
            str(topics_dir / "uORBMessageFieldsGenerated.cpp"),
            "--header-output-file",
            str(topics_dir / "uORBMessageFieldsGenerated.hpp"),
        ],
        check=True,
        env=environment,
    )


def generate_forwarders(topics_dir: Path, destination: Path) -> list[Path]:
    """从官方头的 ORB_DECLARE 派生旧 include 转发层，包括 Topic alias。"""
    destination.mkdir(parents=True, exist_ok=True)
    owners: dict[str, str] = {}
    headers = sorted(topics_dir.glob("*.h"), key=lambda path: path.name)
    if not headers:
        raise RuntimeError("PX4 uORB generator produced no message headers")

    for header in headers:
        text = header.read_text(encoding="utf-8")
        topics = ORB_DECLARE.findall(text)
        if not topics:
            raise RuntimeError(f"official uORB header has no topic: {header.name}")
        for topic in topics:
            previous = owners.setdefault(topic, header.name)
            if previous != header.name:
                raise RuntimeError(
                    f"uORB topic {topic} is declared by {previous} and {header.name}"
                )

    outputs: list[Path] = []
    for topic, owner in sorted(owners.items()):
        output = destination / f"{topic}.hpp"
        output.write_text(
            "#pragma once\n"
            "// 兼容旧包含路径；消息结构和 Topic alias 均来自 PX4 官方生成头。\n"
            f"#include <uORB/topics/{owner}>\n",
            encoding="utf-8",
            newline="\n",
        )
        outputs.append(output)
    return outputs


def make_fragment(
    output: Path,
    staged_topics_dir: Path,
    compat_output: Path,
    staged_compat_dir: Path,
) -> str:
    target_topics_dir = output / "topics"
    headers = [
        target_topics_dir / path.name
        for pattern in ("*.h", "*.hpp")
        for path in sorted(staged_topics_dir.glob(pattern))
    ]
    sources = [
        target_topics_dir / path.name
        for path in sorted(staged_topics_dir.glob("*.cpp"))
    ]
    json_files = [
        target_topics_dir / path.name
        for path in sorted(staged_topics_dir.glob("*.json"))
    ]
    compat_headers = [
        compat_output / path.name
        for path in sorted(staged_compat_dir.glob("*.hpp"))
    ]
    for required in (
        staged_topics_dir / "uORBTopics.hpp",
        staged_topics_dir / "uORBTopics.cpp",
        staged_topics_dir / "uORBMessageFieldsGenerated.hpp",
        staged_topics_dir / "uORBMessageFieldsGenerated.cpp",
    ):
        if not required.is_file():
            raise RuntimeError(f"official uORB aggregate output is missing: {required}")

    def assignment(name: str, paths: list[Path]) -> list[str]:
        if not paths:
            raise RuntimeError(f"generated uORB make variable {name} is empty")
        rows = [f"{name} := \\"]
        rows.extend(
            f"\t{repository_path(path)}" + (" \\" if index + 1 < len(paths) else "")
            for index, path in enumerate(paths)
        )
        return rows

    lines = [
        "# Generated by the PX4 v1.17 uORB orchestration layer. DO NOT EDIT.",
        *assignment("DIMA_UORB_GENERATED_HEADERS", headers),
        *assignment("DIMA_UORB_GENERATED_SOURCES", sources),
        *assignment("DIMA_UORB_GENERATED_JSON", json_files),
        *assignment("DIMA_UORB_COMPAT_HEADERS", compat_headers),
        "DIMA_UORB_GENERATED_STAMP := "
        + repository_path(output / ".generated.json"),
        "DIMA_UORB_GENERATED_OUTPUTS := \\",
        "\t$(DIMA_UORB_GENERATED_HEADERS) \\",
        "\t$(DIMA_UORB_GENERATED_SOURCES) \\",
        "\t$(DIMA_UORB_GENERATED_JSON) \\",
        "\t$(DIMA_UORB_COMPAT_HEADERS) \\",
        "\t$(DIMA_UORB_GENERATED_STAMP)",
        "",
    ]
    return "\n".join(lines)


def write_stamp(
    path: Path,
    schemas: list[Path],
    generation_inputs: list[Path],
    output_root: Path,
    compat_root: Path,
) -> None:
    outputs = sorted(
        [item for item in output_root.rglob("*") if item.is_file() and item != path]
        + [item for item in compat_root.rglob("*") if item.is_file()]
    )
    document = {
        "format": 1,
        "generator": "PX4-Autopilot v1.17.0",
        "source_commit": PINNED_UPSTREAM_COMMIT,
        "inputs": {
            repository_path(item): sha256(item)
            for item in [*schemas, *generation_inputs]
        },
        "outputs": {
            (
                "uORB/" + item.relative_to(output_root).as_posix()
                if item.is_relative_to(output_root)
                else "compat/" + item.relative_to(compat_root).as_posix()
            ): sha256(item)
            for item in outputs
        },
    }
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def make_staging_directory(target: Path) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    return Path(
        tempfile.mkdtemp(prefix=f".{target.name}.tmp-", dir=str(target.parent))
    )


def files_by_relative_path(root: Path) -> dict[Path, Path]:
    return {
        path.relative_to(root): path
        for path in root.rglob("*")
        if path.is_file()
    }


def verify_tree(expected: Path, actual: Path) -> None:
    expected_files = files_by_relative_path(expected)
    actual_files = files_by_relative_path(actual) if actual.is_dir() else {}
    if set(expected_files) != set(actual_files):
        raise RuntimeError(
            f"generated uORB file set differs for {actual}: "
            f"missing={sorted(set(expected_files) - set(actual_files))}, "
            f"unexpected={sorted(set(actual_files) - set(expected_files))}"
        )
    changed = [
        relative
        for relative in expected_files
        if expected_files[relative].read_bytes() != actual_files[relative].read_bytes()
    ]
    if changed:
        raise RuntimeError(f"generated uORB files differ for {actual}: {changed}")


def install_directories(pairs: list[tuple[Path, Path]]) -> None:
    """stamp 发布前逐文件切换；失败时恢复全部旧产物。"""
    backup_roots: list[Path] = []
    backups: list[tuple[Path, Path]] = []
    installed: list[Path] = []
    try:
        for staged, target in pairs:
            target.mkdir(parents=True, exist_ok=True)
            backup_root = Path(
                tempfile.mkdtemp(
                    prefix=f".{target.name}.old-{os.getpid()}-",
                    dir=str(target.parent),
                )
            )
            backup_roots.append(backup_root)
            for old_file in sorted(path for path in target.rglob("*") if path.is_file()):
                backup_file = backup_root / old_file.relative_to(target)
                backup_file.parent.mkdir(parents=True, exist_ok=True)
                os.replace(old_file, backup_file)
                backups.append((old_file, backup_file))
            for staged_file in sorted(
                path for path in staged.rglob("*") if path.is_file()
            ):
                target_file = target / staged_file.relative_to(staged)
                target_file.parent.mkdir(parents=True, exist_ok=True)
                os.replace(staged_file, target_file)
                installed.append(target_file)
    except Exception:
        for target_file in reversed(installed):
            if target_file.exists():
                target_file.unlink()
        for target_file, backup_file in reversed(backups):
            if backup_file.exists():
                target_file.parent.mkdir(parents=True, exist_ok=True)
                os.replace(backup_file, target_file)
        for backup_root in backup_roots:
            if backup_root.exists():
                shutil.rmtree(backup_root)
        raise
    else:
        for backup_root in backup_roots:
            shutil.rmtree(backup_root)


def main() -> int:
    args = parse_args()
    schemas = schema_files(args.schemas)
    verify_upstream_source(args.upstream_root)
    topic_script, fields_script, template_dir, dependencies = upstream_paths(
        args.upstream_root
    )
    output = args.output.resolve()
    compat_output = args.compat_output.resolve()
    output_stage = make_staging_directory(output)
    compat_stage = make_staging_directory(compat_output)

    try:
        topics_dir = output_stage / "topics"
        run_upstream(
            topic_script, fields_script, template_dir, schemas, topics_dir
        )
        generate_forwarders(topics_dir, compat_stage)

        makefile = output_stage / "uorb_sources.mk"
        makefile.write_text(
            make_fragment(output, topics_dir, compat_output, compat_stage),
            encoding="utf-8",
            newline="\n",
        )
        write_stamp(
            output_stage / ".generated.json",
            schemas,
            [
                *dependencies,
                Path(__file__).resolve(),
                SOURCE_MANIFEST_TOOL.resolve(),
                args.upstream_root.resolve() / "SOURCE_MANIFEST.json",
            ],
            output_stage,
            compat_stage,
        )

        if args.verify:
            verify_tree(output_stage, output)
            verify_tree(compat_stage, compat_output)
            print(f"uORB generation verification passed: {output}")
        else:
            install_directories(
                [(output_stage, output), (compat_stage, compat_output)]
            )
            print(
                f"generated {len(schemas)} PX4 uORB message types in {output}"
            )
    finally:
        for stage in (output_stage, compat_stage):
            if stage.exists():
                shutil.rmtree(stage)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

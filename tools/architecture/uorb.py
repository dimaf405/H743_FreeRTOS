"""PX4 v1.17 uORB schema、上游来源和官方派生产物闭包门禁。"""

from __future__ import annotations

import hashlib
import json
import pathlib
import re

from architecture.common import ROOT, Violation, line_for
from architecture.upstream import validate_source_manifest


PINNED_UORB_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
UPSTREAM_ROOT = ROOT / "tools/upstream/uorb_v1_17"
SCHEMA_ROOT = ROOT / "Dima/messages/schemas"
GENERATED_ROOT = ROOT / "build/generated/uORB"
COMPAT_ROOT = ROOT / "build/generated/messages"
PASCAL_MESSAGE_NAME = re.compile(r"^[A-Z][A-Za-z0-9]*$")
LOCAL_EXTENSION_RE = re.compile(r"(?m)^\s*@[A-Za-z_]")
ORB_DECLARE_RE = re.compile(r"\bORB_DECLARE\(([a-z][a-z0-9_]*)\);")


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repository_path(path: pathlib.Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def _schema_files(violations: list[Violation]) -> list[pathlib.Path]:
    """只接受生成器会读取的单层 PascalCase schema，不复制消息名称目录。"""
    all_schemas = sorted(SCHEMA_ROOT.rglob("*.msg"))
    schemas = [path for path in all_schemas if path.parent == SCHEMA_ROOT]
    if not schemas:
        violations.append(Violation(
            SCHEMA_ROOT, 1, "R333", "uORB schema directory is empty",
        ))
        return []
    for path in all_schemas:
        if path.parent != SCHEMA_ROOT:
            violations.append(Violation(
                path, 1, "R333",
                "nested uORB schema is ignored by the official generator entry",
            ))
            continue
        if not PASCAL_MESSAGE_NAME.fullmatch(path.stem):
            violations.append(Violation(
                path, 1, "R333", "PX4 uORB schema name must be PascalCase",
            ))
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            violations.append(Violation(
                path, 1, "R333", f"cannot read uORB schema: {error}",
            ))
            continue
        extension = LOCAL_EXTENSION_RE.search(text)
        if extension:
            violations.append(Violation(
                path, line_for(text, extension.group(0)), "R333",
                "local @ directive is forbidden; use PX4 native msg syntax",
            ))
    return schemas


def _scan_upstream_schema_identity(
    schemas: list[pathlib.Path], violations: list[Violation]
) -> None:
    """凡上游存在唯一同名消息就逐字节对照；Dima 专用消息仍走同一生成器。"""
    reference_root = UPSTREAM_ROOT / "msg"
    references: dict[str, list[pathlib.Path]] = {}
    for reference in sorted(reference_root.rglob("*.msg")):
        references.setdefault(reference.name, []).append(reference)

    matched = 0
    for schema in schemas:
        candidates = references.get(schema.name, [])
        if len(candidates) > 1:
            violations.append(Violation(
                schema, 1, "R333",
                "upstream schema identity is ambiguous: "
                + ", ".join(_repository_path(path) for path in candidates),
            ))
        elif len(candidates) == 1:
            matched += 1
            if schema.read_bytes() != candidates[0].read_bytes():
                violations.append(Violation(
                    schema, 1, "R333",
                    "PX4-named schema differs from the pinned upstream file",
                ))
    if schemas and matched == 0:
        violations.append(Violation(
            SCHEMA_ROOT, 1, "R333",
            "no product schema can be traced to the pinned PX4 message snapshot",
        ))


def _expected_inputs(schemas: list[pathlib.Path]) -> dict[str, str]:
    upstream_dependencies = sorted(
        path
        for dependency_root in (
            UPSTREAM_ROOT / "Tools/msg",
            UPSTREAM_ROOT / "src/lib/heatshrink",
        )
        for path in dependency_root.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    )
    orchestration_inputs = (
        ROOT / "tools/uorb/generate_messages.py",
        ROOT / "tools/generation/source_manifest.py",
        UPSTREAM_ROOT / "SOURCE_MANIFEST.json",
    )
    return {
        _repository_path(path): _sha256(path)
        for path in [*schemas, *upstream_dependencies, *orchestration_inputs]
    }


def _actual_outputs() -> dict[str, pathlib.Path]:
    catalog_path = GENERATED_ROOT / ".generated.json"
    outputs = {
        "uORB/" + path.relative_to(GENERATED_ROOT).as_posix(): path
        for path in GENERATED_ROOT.rglob("*")
        if path.is_file() and path != catalog_path
    }
    outputs.update({
        "compat/" + path.relative_to(COMPAT_ROOT).as_posix(): path
        for path in COMPAT_ROOT.rglob("*")
        if path.is_file()
    })
    return outputs


def _catalog_output_path(relative: str) -> str | None:
    if relative.startswith("uORB/"):
        return "build/generated/uORB/" + relative.removeprefix("uORB/")
    if relative.startswith("compat/"):
        return "build/generated/messages/" + relative.removeprefix("compat/")
    return None


def _scan_topic_outputs(violations: list[Violation]) -> None:
    """动态核对官方头/源/JSON 三元组及其中由模板生成的 ID 与 hash。"""
    topics = GENERATED_ROOT / "topics"
    headers = {path.stem: path for path in topics.glob("*.h")}
    sources = {
        path.stem: path
        for path in topics.glob("*.cpp")
        if path.name not in {
            "uORBTopics.cpp", "uORBMessageFieldsGenerated.cpp",
        }
    }
    json_files = {path.stem: path for path in topics.glob("*.json")}
    if not headers or set(headers) != set(sources) or set(headers) != set(json_files):
        violations.append(Violation(
            topics, 1, "R333",
            "official uORB header/source/JSON topic sets differ",
        ))

    for path in sources.values():
        text = path.read_text(encoding="utf-8")
        if "ORB_DEFINE(" not in text or "ORB_ID::" not in text:
            violations.append(Violation(
                path, 1, "R333",
                "official topic source lacks generated hash or ORB_ID binding",
            ))

    for path in json_files.values():
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
            orb_ids = document["orb_ids"]
            header = headers.get(path.stem)
            declared_topics = (
                ORB_DECLARE_RE.findall(header.read_text(encoding="utf-8"))
                if header is not None else []
            )
            main_orb_id = document.get("main_orb_id")
            ids_are_valid = (
                isinstance(orb_ids, list)
                and bool(orb_ids)
                and all(isinstance(value, int) for value in orb_ids)
            )
            # PX4 官方 msg.json.em 对只有 TOPICS alias、没有结构体同名 Topic
            # 的消息生成 main_orb_id=-1。用同批官方头的 ORB_DECLARE 数量和
            # 名称交叉验证这一合法情形，不能宽泛放行任意负 ID。
            alias_only_identity = (
                ids_are_valid
                and main_orb_id == -1
                and bool(declared_topics)
                and path.stem not in declared_topics
                and len(declared_topics) == len(orb_ids)
            )
            valid = (
                isinstance(document.get("name"), str)
                and isinstance(document.get("fields"), str)
                and ids_are_valid
                and (main_orb_id in orb_ids or alias_only_identity)
            )
            if not valid:
                raise TypeError("topic identity or field catalog is invalid")
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
            violations.append(Violation(
                path, 1, "R333", f"invalid official uORB JSON: {error}",
            ))

    # 压缩格式和 Topic 三元组同属一次官方生成事务，但它不是一个 Topic，不能
    # 混入 header/source/JSON 集合相等性判断。
    fields_header = topics / "uORBMessageFieldsGenerated.hpp"
    fields_source = topics / "uORBMessageFieldsGenerated.cpp"
    try:
        fields_header_text = fields_header.read_text(encoding="utf-8")
        fields_source_text = fields_source.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            fields_header, 1, "R333",
            f"generated compressed uORB fields are unavailable: {error}",
        ))
    else:
        if not all(token in fields_header_text for token in (
            "orb_compressed_message_formats",
            "orb_tokenized_fields_max_length",
        )) or "compressed_fields[]" not in fields_source_text:
            violations.append(Violation(
                fields_header, 1, "R333",
                "generated compressed uORB field contract is incomplete",
            ))

    aggregate_header = topics / "uORBTopics.hpp"
    aggregate_source = topics / "uORBTopics.cpp"
    try:
        header_text = aggregate_header.read_text(encoding="utf-8")
        source_text = aggregate_source.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            aggregate_header, 1, "R333",
            f"official aggregate Topic registry is unavailable: {error}",
        ))
    else:
        if not all(token in header_text for token in (
            "ORB_TOPICS_COUNT", "orb_topics_count()", "enum class ORB_ID",
        )) or not all(token in source_text for token in (
            "uorb_topics_list", "orb_get_topics()", "get_orb_meta(ORB_ID",
        )):
            violations.append(Violation(
                aggregate_header, 1, "R333",
                "official aggregate Topic ID/registry output is incomplete",
            ))


def _scan_fragment(
    declared_outputs: dict[str, str], violations: list[Violation]
) -> None:
    fragment_path = GENERATED_ROOT / "uorb_sources.mk"
    try:
        text = fragment_path.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            fragment_path, 1, "R333", f"cannot read uORB Make fragment: {error}",
        ))
        return
    required_variables = (
        "DIMA_UORB_GENERATED_HEADERS",
        "DIMA_UORB_GENERATED_SOURCES",
        "DIMA_UORB_GENERATED_JSON",
        "DIMA_UORB_COMPAT_HEADERS",
        "DIMA_UORB_GENERATED_STAMP",
        "DIMA_UORB_GENERATED_OUTPUTS",
    )
    if any(variable not in text for variable in required_variables):
        violations.append(Violation(
            fragment_path, 1, "R333",
            "generated uORB Make fragment lacks an official output class",
        ))

    listed = set(re.findall(
        r"\bbuild/generated/(?:uORB|messages)/[A-Za-z0-9_./-]+", text
    ))
    expected = {
        converted
        for relative in declared_outputs
        if (converted := _catalog_output_path(relative)) is not None
        and converted != "build/generated/uORB/uorb_sources.mk"
    }
    # stamp 不能递归记录自己的 hash，但 Make 必须把它列为完整生成事务的完成标记。
    expected.add("build/generated/uORB/.generated.json")
    if listed != expected:
        violations.append(Violation(
            fragment_path, 1, "R333",
            "generated uORB Make fragment differs from the output hash closure",
        ))


def _scan_generated_closure(
    schemas: list[pathlib.Path], violations: list[Violation]
) -> None:
    catalog_path = GENERATED_ROOT / ".generated.json"
    try:
        catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
        declared_outputs = catalog["outputs"]
        if (
            catalog.get("format") != 1
            or catalog.get("generator") != "PX4-Autopilot v1.17.0"
            or catalog.get("source_commit") != PINNED_UORB_COMMIT
            or catalog.get("inputs") != _expected_inputs(schemas)
            or not isinstance(declared_outputs, dict)
            or not declared_outputs
        ):
            raise TypeError("identity, inputs or outputs differ from the source closure")
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        violations.append(Violation(
            catalog_path, 1, "R333", f"invalid generated uORB catalog: {error}",
        ))
        return

    actual_outputs = _actual_outputs()
    if set(actual_outputs) != set(declared_outputs):
        violations.append(Violation(
            catalog_path, 1, "R333",
            "generated uORB file set differs from its output hash closure",
        ))
        return
    changed = [
        relative
        for relative, path in actual_outputs.items()
        if not isinstance(declared_outputs[relative], str)
        or _sha256(path) != declared_outputs[relative]
    ]
    if changed:
        violations.append(Violation(
            actual_outputs[changed[0]], 1, "R333",
            f"generated uORB output hash mismatch: {changed}",
        ))

    _scan_topic_outputs(violations)
    _scan_fragment(declared_outputs, violations)

    for path in sorted(COMPAT_ROOT.glob("*.hpp")):
        text = path.read_text(encoding="utf-8")
        if not re.fullmatch(
            r"#pragma once\n//[^\n]*\n#include <uORB/topics/[A-Za-z0-9_]+\.h>\n",
            text,
        ):
            violations.append(Violation(
                path, 1, "R333",
                "compatibility header must remain a generated include-only forwarder",
            ))


def _scan_legacy_implementations(violations: list[Violation]) -> None:
    legacy_paths = (
        ROOT / "Dima/messages/uorb_abi.lock.json",
        ROOT / "tools/uorb/parser.py",
        ROOT / "tools/uorb/layout.py",
        ROOT / "tools/uorb/renderer.py",
    )
    for path in legacy_paths:
        if path.exists():
            violations.append(Violation(
                path, 1, "R333",
                "local uORB ABI/parser/layout/renderer must be retired",
            ))

    generator_path = ROOT / "tools/uorb/generate_messages.py"
    generator_text = generator_path.read_text(encoding="utf-8")
    local_generator = re.search(
        r"\bclass\s+(?:Primitive|FieldLayout|Layout)\b|"
        r"\bdef\s+(?:parse_schema|struct_layout|resolve_layouts|"
        r"generate_header|generate_catalog)\s*\(",
        generator_text,
    )
    if local_generator:
        violations.append(Violation(
            generator_path, line_for(generator_text, local_generator.group(0)),
            "R333", "local uORB parser/layout/renderer re-entered the wrapper",
        ))

    project_text = (ROOT / "make/project.mk").read_text(encoding="utf-8")
    old_make = re.search(r"\bMESSAGE_ABI_LOCK\b|\bMESSAGE_GENERATED_SOURCE\b", project_text)
    if old_make:
        violations.append(Violation(
            ROOT / "make/project.mk", line_for(project_text, old_make.group(0)),
            "R333", "legacy uORB ABI lock or hand-written source wiring remains",
        ))

    scan_roots = (
        ROOT / "Dima", ROOT / "Boards", ROOT / "Linker", ROOT / "make",
        ROOT / "tools/elf_support",
    )
    for scan_root in scan_roots:
        for path in scan_root.rglob("*"):
            if not path.is_file() or path.suffix.lower() not in {
                ".c", ".cpp", ".h", ".hpp", ".ld", ".mk", ".py",
            }:
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (OSError, UnicodeError):
                continue
            obsolete = re.search(r"\.dima_orb_meta\b|\bMetadataRegistrar\b", text)
            if obsolete:
                violations.append(Violation(
                    path, line_for(text, obsolete.group(0)), "R333",
                    "retired linker-section uORB registry remains",
                ))


def _scan_make_wiring(violations: list[Violation]) -> None:
    path = ROOT / "make/project.mk"
    text = path.read_text(encoding="utf-8")
    required = (
        "SOURCE_MANIFEST_TOOL := tools/generation/source_manifest.py",
        "UORB_SOURCE_MANIFEST := $(UORB_UPSTREAM_ROOT)/SOURCE_MANIFEST.json",
        "include $(MESSAGE_GENERATED_MAKEFILE)",
        "$(DIMA_UORB_GENERATED_SOURCES)",
        "uorb-generated-verify:",
    )
    for literal in required:
        if literal not in text:
            violations.append(Violation(
                path, 1, "R333", f"official uORB build wiring is missing: {literal}",
            ))


def scan_uorb_generation_contract(violations: list[Violation]) -> None:
    """汇总来源、权威 schema、官方生成结果和薄兼容层的单向闭包。"""
    validate_source_manifest(
        UPSTREAM_ROOT,
        "PX4-Autopilot",
        PINNED_UORB_COMMIT,
        "R333",
        violations,
    )
    schemas = _schema_files(violations)
    _scan_upstream_schema_identity(schemas, violations)
    _scan_generated_closure(schemas, violations)
    _scan_legacy_implementations(violations)
    _scan_make_wiring(violations)

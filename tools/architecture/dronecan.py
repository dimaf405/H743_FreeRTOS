"""DroneCAN manifest、DSDL 输入与生成目录一致性门禁。"""

from __future__ import annotations

import hashlib
import json
import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    line_for,
    repository_files_named,
    require_literals,
)


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: pathlib.Path, violations: list[Violation], label: str):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R349", f"cannot read {label}: {error}",
        ))
        return None


def _unique_dima_file(
    filename: str,
    violations: list[Violation],
    label: str,
) -> pathlib.Path | None:
    """按唯一文件身份定位权威 manifest，不在门禁中复制其目录位置。"""
    dima_root = ROOT / "Dima"
    matches = tuple(
        path for path in repository_files_named(filename)
        if path.is_relative_to(dima_root)
    )
    if len(matches) == 1:
        return matches[0]
    violations.append(Violation(
        dima_root, 1, "R349",
        f"expected one {label} named {filename}, found {len(matches)}",
    ))
    return None


def _scan_generated_catalog(
    manifest_path: pathlib.Path,
    manifest: dict,
    violations: list[Violation],
) -> None:
    """比对 manifest、生成文件闭包和逐文件哈希，拒绝陈旧或手改产物。"""
    generated = ROOT / "build/generated/dronecan"
    catalog_path = generated / ".generated.json"
    catalog = _load_json(catalog_path, violations, "generated DroneCAN catalog")
    if catalog is None:
        return
    if (catalog.get("contract") != manifest.get("contract") or
            catalog.get("manifest_sha256") != _sha256(manifest_path) or
            catalog.get("upstream") != manifest.get("upstream")):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN catalog is stale or uses different inputs",
        ))
    expected_hashes = catalog.get("output_sha256")
    if not isinstance(expected_hashes, dict):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN catalog lacks its output hash closure",
        ))
        return
    actual_paths = {
        path.relative_to(generated).as_posix(): path
        for path in generated.rglob("*")
        if path.is_file() and path != catalog_path
    }
    if set(actual_paths) != set(expected_hashes):
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN file closure differs from its catalog",
        ))
        return
    changed = [
        relative for relative, path in actual_paths.items()
        if _sha256(path) != expected_hashes[relative]
    ]
    if changed:
        violations.append(Violation(
            generated / changed[0], 1, "R349",
            f"generated DroneCAN output hash mismatch: {changed}",
        ))

    messages = catalog.get("messages")
    roles = {
        message.get("role") for message in messages
        if isinstance(message, dict)
    } if isinstance(messages, list) else set()
    manifest_files = manifest.get("dsdl", {}).get("files", [])
    manifest_roles = {
        item.get("role") for item in manifest_files
        if isinstance(item, dict) and "role" in item
    } if isinstance(manifest_files, list) else set()
    if roles != manifest_roles:
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated subscription roles differ from the manifest",
        ))

    parameters = catalog.get("parameters")
    manifest_parameters = manifest.get("parameters", [])
    manifest_parameter_names = [
        parameter.get("name") for parameter in manifest_parameters
        if isinstance(parameter, dict)
    ] if isinstance(manifest_parameters, list) else []
    if not isinstance(parameters, list) or parameters != manifest_parameter_names:
        violations.append(Violation(
            catalog_path, 1, "R349",
            "generated DroneCAN parameter binding differs from the manifest",
        ))


def scan_dronecan_contract(violations: list[Violation]) -> None:
    """核对唯一 manifest、DSDL pin、生成目录及禁止手写派生定义的边界。"""
    manifest_path = _unique_dima_file(
        "dronecan_contract.json", violations, "DroneCAN manifest"
    )
    if manifest_path is None:
        return
    manifest = _load_json(manifest_path, violations, "DroneCAN manifest")
    if not isinstance(manifest, dict):
        return

    dsdl = manifest.get("dsdl")
    if not isinstance(dsdl, dict):
        violations.append(Violation(
            manifest_path, 1, "R349", "DroneCAN manifest lacks a DSDL object",
        ))
        return
    namespace_root = dsdl.get("namespace_root")
    files = dsdl.get("files")
    if not isinstance(namespace_root, str) or not isinstance(files, list):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN DSDL namespace and file list are invalid",
        ))
        return
    namespace = ROOT / namespace_root
    roles = [
        item.get("role") for item in files
        if isinstance(item, dict) and isinstance(item.get("role"), str)
    ]
    paths = [
        item.get("path") for item in files
        if isinstance(item, dict) and isinstance(item.get("path"), str)
    ]
    invalid_roles = any(
        not isinstance(item, dict) or
        ("role" in item and
         (not isinstance(item.get("role"), str) or not item["role"]))
        for item in files
    )
    if (invalid_roles or len(set(roles)) != len(roles) or
            len(paths) != len(files) or len(set(paths)) != len(paths)):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN DSDL paths and any declared roles must be unique",
        ))

    listed_paths: set[pathlib.Path] = set()
    for item in files:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            continue
        source = namespace / item["path"]
        listed_paths.add(source)
        if not source.is_file() or _sha256(source) != item.get("sha256"):
            violations.append(Violation(
                source, 1, "R349",
                "vendored DSDL input is missing or differs from its manifest pin",
            ))
    actual_paths = {
        path for path in namespace.rglob("*.uavcan") if path.is_file()
    } if namespace.is_dir() else set()
    if actual_paths != listed_paths:
        violations.append(Violation(
            namespace, 1, "R349",
            "vendored DSDL inputs must be exactly the manifest closure",
        ))

    parameters = manifest.get("parameters", [])
    if not isinstance(parameters, list):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN parameter bindings must be a list",
        ))
        parameters = []
    parameter_names = [
        parameter.get("name") for parameter in parameters
        if isinstance(parameter, dict)
    ]
    parameter_roles = [
        parameter.get("role") for parameter in parameters
        if isinstance(parameter, dict)
    ]
    if (len(parameter_names) != len(parameters) or
            any(not isinstance(name, str) or not name
                for name in parameter_names) or
            len(parameter_names) != len(set(parameter_names)) or
            any(not isinstance(role, str) or not role
                for role in parameter_roles) or
            len(parameter_roles) != len(set(parameter_roles))):
        violations.append(Violation(
            manifest_path, 1, "R349",
            "DroneCAN parameter names and roles must be non-empty and unique",
        ))

    project_path = ROOT / "make/project.mk"
    require_literals(
        project_path,
        (
            ("DRONECAN_CONTRACT_GENERATOR := tools/dronecan/generate_contract.py",
             "R349", "build must invoke the DroneCAN generator"),
            ("--print-runtime-sources", "R349",
             "first-party DroneCAN sources must be discovered by the generator"),
            ("include $(DRONECAN_GENERATED_MAKEFILE)", "R349",
             "build must consume the generated source catalog"),
            ("$(DIMA_DRONECAN_GENERATED_C_SOURCES)", "R349",
             "DSDL C sources must come from the generated catalog"),
            ("$(DIMA_DRONECAN_PARAMETER_YAML)", "R349",
             "DroneCAN parameter YAML must come from the generated catalog"),
            ("$(sort $(wildcard $(PARAMETER_DEFINITION_DIR)/module_*.yaml))",
             "R349", "parameter YAML inputs must be discovered, not listed"),
        ),
        violations,
    )
    project_text = project_path.read_text(encoding="utf-8")
    forbidden_make = re.search(
        r"Middlewares/Third_Party/dronecan_dsdl/(?:include|src)|"
        r"uavcan\.(?:protocol|equipment)\.[^\s\\]+\.[ch]",
        project_text,
    )
    if forbidden_make:
        violations.append(Violation(
            project_path, line_for(project_text, forbidden_make.group(0)),
            "R349", "Make must not handwrite DroneCAN message files",
        ))

    runtime_sources = tuple(sorted(
        path for path in (ROOT / "Dima").rglob("DroneCan*.cpp")
        if path.is_file()
    ))
    for source in runtime_sources:
        text = source.read_text(encoding="utf-8")
        handwritten_descriptor = re.search(
            r"\bUAVCAN_[A-Z0-9_]+_(?:ID|SIGNATURE)\b", text
        )
        if handwritten_descriptor:
            violations.append(Violation(
                source, line_for(text, handwritten_descriptor.group(0)),
                "R349",
                "first-party consumers must use generated message descriptors",
            ))

    parameter_names_set = set(parameter_names)
    parameter_root = ROOT / "Dima/middleware/parameters/definitions"
    definition_re = re.compile(
        r"\bPARAM_DEFINE_(?:INT32|FLOAT)\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)"
    )
    for parameter_source in sorted(parameter_root.glob("*.c")):
        source_text = parameter_source.read_text(encoding="utf-8")
        for definition in definition_re.finditer(source_text):
            if definition.group(1) not in parameter_names_set:
                continue
            violations.append(Violation(
                parameter_source,
                line_for(source_text, definition.group(0)), "R349",
                "DroneCAN parameters must be generated from the manifest",
            ))

    _scan_generated_catalog(manifest_path, manifest, violations)

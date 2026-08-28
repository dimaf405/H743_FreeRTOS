"""校验 PX4 YAML 参数闭包、官方派生产物与 MAVLink 生成边界。"""

from __future__ import annotations

import json
import pathlib
import re
import xml.etree.ElementTree as ET

from architecture.build_closure import BuildClosureError, load_build_closure
from architecture.common import ROOT, Violation, line_for, sources_under
from architecture.mavlink_protocol import scan_mavlink_protocol_contract
from architecture.upstream import validate_source_manifest


RAW_PARAMETER_DEFINITION_RE = re.compile(
    r"\b(?:PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\("
)
FORBIDDEN_PARAMETER_RE = re.compile(
    r"\b(?:CAL_MAG[12]_(?:ID|ROT)|SENS_DPRES_OFF)\b"
)


def _read_generated_catalogue(
    path: pathlib.Path, violations: list[Violation]
) -> tuple[list[dict], set[str]] | None:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        parameters = document["parameters"]
        if not isinstance(parameters, list) or not parameters:
            raise TypeError("parameters is not a non-empty list")
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R331", f"cannot read generated parameter catalog: {error}"
        ))
        return None

    names: list[str] = []
    for entry in parameters:
        if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
            violations.append(Violation(
                path, 1, "R331",
                "generated parameter catalog contains an invalid entry",
            ))
            continue
        names.append(entry["name"])
    if len(names) != len(set(names)):
        violations.append(Violation(
            path, 1, "R331", "generated parameter catalog contains duplicates"
        ))
    return parameters, set(names)


def _scan_parameter_generation_contract(violations: list[Violation]) -> None:
    """确认 Make 只把 YAML 送入官方链，并校验 XML/JSON/Header 完整闭包。"""
    validate_source_manifest(
        ROOT / "tools/upstream/parameter_yaml_20260827",
        "PX4-Autopilot",
        "1f6b6f61f8f42eaab0269c16a442cb580f954d7c",
        "R331",
        violations,
    )
    try:
        parameter_inputs = sorted(
            load_build_closure(ROOT).parameter_generator_inputs
        )
    except BuildClosureError as error:
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            f"cannot evaluate generated parameter input closure: {error}",
        ))
        return

    if not parameter_inputs:
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            "PARAMETER_YAML_DEFINITIONS resolved to an empty set",
        ))
        return

    input_paths = [ROOT / relative for relative in parameter_inputs]
    for path in input_paths:
        if path.suffix.lower() not in (".yaml", ".yml"):
            violations.append(Violation(
                path, 1, "R331", "parameter generator inputs must be PX4 YAML"
            ))
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R331", "listed parameter YAML does not exist"
            ))

    tracked_yaml = set(
        (ROOT / "Dima/middleware/parameters/definitions").glob("module_*.yaml")
    )
    if not tracked_yaml or not tracked_yaml.issubset(set(input_paths)):
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            "all tracked module_*.yaml files must enter PARAMETER_YAML_DEFINITIONS",
        ))

    # 受版本控制的 module_*.yaml 已由上方通用闭包检查覆盖；这里只保留
    # 仍需从协议 schema 生成后再送入 PX4 参数链的 DroneCAN YAML。
    required_generated_yaml = {
        ROOT / "build/generated/dronecan/module_dronecan.yaml",
    }
    if not required_generated_yaml.issubset(set(input_paths)):
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            "generated DroneCAN YAML must enter the official parameter chain",
        ))

    # 受版本控制的 C/C++ 树不得再保留 PARAM_DEFINE 入口；官方 module_params.c
    # 只允许出现在 build/generated 的中间目录。
    for path in sources_under(("Dima", "Boards")):
        text = path.read_text(encoding="utf-8")
        match = RAW_PARAMETER_DEFINITION_RE.search(text)
        if match:
            violations.append(Violation(
                path, line_for(text, match.group(0)), "R331",
                "tracked source must not contain PARAM_DEFINE parameter entries",
            ))

    legacy_tools = (
        ROOT / "tools/parameters/generate_header.py",
        ROOT / "tools/parameters/process_parameters.py",
        ROOT / "tools/parameters/dima_params",
    )
    for path in legacy_tools:
        if path.exists():
            violations.append(Violation(
                path, 1, "R331",
                "local parameter parser/renderer must be retired",
            ))

    generated_dir = ROOT / "build/generated/parameters"
    json_path = generated_dir / "parameters.json"
    catalogue = _read_generated_catalogue(json_path, violations)
    if catalogue is None:
        return
    _, json_names = catalogue

    xml_path = generated_dir / "parameters.xml"
    try:
        xml_names = [
            element.attrib["name"]
            for element in ET.parse(xml_path).getroot().iter("parameter")
            if "name" in element.attrib
        ]
    except (OSError, ET.ParseError) as error:
        violations.append(Violation(
            xml_path, 1, "R331", f"cannot read official parameter XML: {error}"
        ))
        return
    if len(xml_names) != len(set(xml_names)) or set(xml_names) != json_names:
        violations.append(Violation(
            xml_path, 1, "R331",
            "official parameter XML and JSON catalogues differ",
        ))

    raw_header_path = generated_dir / "px4_parameters.hpp"
    public_header_path = (
        ROOT / "build/generated_include/parameters/px4_parameters.hpp"
    )
    try:
        raw_header = raw_header_path.read_text(encoding="utf-8")
        public_header = public_header_path.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            raw_header_path, 1, "R331",
            f"cannot read generated PX4 parameter header: {error}",
        ))
    else:
        if ("This file is autogenerated from parameters.xml" not in raw_header or
                raw_header not in public_header or
                not public_header.startswith("#pragma once\n")):
            violations.append(Violation(
                public_header_path, 1, "R331",
                "public parameter header must wrap the unmodified PX4 output",
            ))

    generated_files = [
        *input_paths,
        generated_dir / "module_params.c",
        xml_path,
        json_path,
        raw_header_path,
        generated_dir / "parameter_contract.hpp",
        public_header_path,
        ROOT / "build/generated_include/parameters/parameter_contract.hpp",
        ROOT / "build/generated/component_metadata/parameters.json",
    ]
    for path in generated_files:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as error:
            violations.append(Violation(
                path, 1, "R331", f"parameter closure file is unavailable: {error}"
            ))
            continue
        forbidden = FORBIDDEN_PARAMETER_RE.search(text)
        if forbidden:
            violations.append(Violation(
                path, line_for(text, forbidden.group(0)), "R331",
                "removed calibration parameter re-entered the parameter closure",
            ))
        duplicate_catalog = re.search(
            r"\bk(?:MavlinkPublic|QgcRequired)Parameters\b|\bqgc_required\b",
            text,
        )
        if duplicate_catalog:
            violations.append(Violation(
                path, line_for(text, duplicate_catalog.group(0)), "R331",
                "parameter closure must not contain a second QGC/public catalog",
            ))

    project_path = ROOT / "make/project.mk"
    project_text = project_path.read_text(encoding="utf-8")
    required_make_literals = (
        "PARAMETER_UPSTREAM_ROOT := tools/upstream/parameter_yaml_20260827",
        "PARAMETER_YAML_DEFINITIONS :=",
        "--yaml \"$(definition)\"",
        "--upstream-root $(PARAMETER_UPSTREAM_ROOT)",
    )
    for literal in required_make_literals:
        if literal not in project_text:
            violations.append(Violation(
                project_path, 1, "R331",
                f"official PX4 YAML build wiring is missing: {literal}",
            ))
    old_make = re.search(
        r"\bPARAMETER_DEFINITIONS\b|"
        r"\$\(foreach\s+source,\$\(PARAMETER_[^)]+\),--source",
        project_text,
    )
    if old_make:
        violations.append(Violation(
            project_path, line_for(project_text, old_make.group(0)), "R331",
            "legacy C parameter build wiring must be removed",
        ))


def scan_mavlink_contract(violations: list[Violation]) -> None:
    """核对参数生成与 MAVLink XML/方言/文档的单一生成闭包。"""
    generated_source_tree = ROOT / "Dima/lib/mavlink/c_library_v2"
    if generated_source_tree.exists():
        violations.append(Violation(
            generated_source_tree, 1, "R330",
            "generated MAVLink code must remain under build/generated",
        ))
    _scan_parameter_generation_contract(violations)
    scan_mavlink_protocol_contract(violations)

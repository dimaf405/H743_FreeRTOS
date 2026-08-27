"""参数定义闭包、生成目录与 MAVLink 生成输入一致性门禁。"""

from __future__ import annotations

import json
import pathlib
import re

from architecture.build_closure import BuildClosureError, load_build_closure
from architecture.common import (
    ROOT,
    Violation,
    line_for,
    sources_under,
    strip_c_comments,
)
from architecture.mavlink_protocol import scan_mavlink_protocol_contract


PARAMETER_DEFINITION_RE = re.compile(
    r"\bPARAM_DEFINE_(INT32|FLOAT)\s*\(\s*"
    r"([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([^\s,)]+)\s*\)\s*;"
)
RAW_PARAMETER_DEFINITION_RE = re.compile(
    r"\b(?:PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\("
)


def _literal_default(parameter_type: str, token: str) -> int | float:
    """把生成器支持的数值字面量归一化，便于与 JSON 默认值比较。"""
    if parameter_type == "INT32":
        return int(token.rstrip("uUlL"), 0)
    return float(token.rstrip("fFlL"))


def _scan_parameter_generation_contract(violations: list[Violation]) -> None:
    """从 Make 权威输入提取参数，并与生成 JSON 的类型/默认值逐项核对。"""
    try:
        definition_inputs = sorted(
            load_build_closure(ROOT).parameter_generator_inputs
        )
    except BuildClosureError as error:
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            f"cannot evaluate generated parameter input closure: {error}",
        ))
        return

    parameter_paths = [ROOT / relative for relative in definition_inputs]
    definitions: dict[str, tuple[str, int | float, pathlib.Path, int]] = {}
    parsed_count = 0
    raw_count = 0
    for path in parameter_paths:
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R331",
                "listed parameter definition source does not exist",
            ))
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        raw_count += len(RAW_PARAMETER_DEFINITION_RE.findall(code))
        matches = tuple(PARAMETER_DEFINITION_RE.finditer(code))
        parsed_count += len(matches)
        for definition in matches:
            parameter_type, name, token = definition.groups()
            line = line_for(text, definition.group(0))
            try:
                default = _literal_default(parameter_type, token)
            except ValueError:
                violations.append(Violation(
                    path, line, "R331",
                    f"parameter '{name}' has a non-literal default",
                ))
                continue
            if name in definitions:
                violations.append(Violation(
                    path, line, "R331",
                    f"parameter '{name}' has duplicate source definitions",
                ))
                continue
            definitions[name] = (parameter_type, default, path, line)

    listed_parameter_paths = set(parameter_paths)
    for path in sources_under(("Dima/middleware/parameters/definitions",)):
        if path in listed_parameter_paths:
            continue
        code = strip_c_comments(path.read_text(encoding="utf-8"))
        if RAW_PARAMETER_DEFINITION_RE.search(code):
            violations.append(Violation(
                path, 1, "R331",
                "parameter definition exists outside PARAMETER_DEFINITIONS",
            ))
    if parsed_count != raw_count:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "every parameter definition must use a parseable literal type, "
            "name, and default "
            f"(parsed={parsed_count}, raw={raw_count})",
        ))

    generated_path = ROOT / "build/generated/parameters/parameters.json"
    try:
        generated_document = json.loads(
            generated_path.read_text(encoding="utf-8")
        )
        generated_parameters = generated_document["parameters"]
        if not isinstance(generated_parameters, list):
            raise TypeError("parameters is not a list")
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        violations.append(Violation(
            generated_path, 1, "R331",
            f"cannot read generated parameter catalog: {error}",
        ))
        return

    generated_by_name: dict[str, dict] = {}
    for entry in generated_parameters:
        if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
            violations.append(Violation(
                generated_path, 1, "R331",
                "generated parameter catalog contains an invalid entry",
            ))
            continue
        name = entry["name"]
        if name in generated_by_name:
            violations.append(Violation(
                generated_path, 1, "R331",
                f"generated parameter '{name}' is duplicated",
            ))
            continue
        generated_by_name[name] = entry

    if set(generated_by_name) != set(definitions):
        missing = sorted(set(definitions) - set(generated_by_name))
        unexpected = sorted(set(generated_by_name) - set(definitions))
        violations.append(Violation(
            generated_path, 1, "R331",
            "generated parameter names differ from definition inputs: "
            f"missing={missing}, unexpected={unexpected}",
        ))
        return

    generated_types = {"INT32": "Int32", "FLOAT": "Float"}
    for name, (parameter_type, default, source, line) in definitions.items():
        entry = generated_by_name[name]
        if (entry.get("type") != generated_types[parameter_type] or
                entry.get("default") != default):
            violations.append(Violation(
                source, line, "R331",
                f"generated type/default differs for parameter '{name}'",
            ))


def scan_mavlink_contract(violations: list[Violation]) -> None:
    """核对参数定义和 MAVLink XML/lock 到生成物的单向生成闭包。"""
    generated_source_tree = ROOT / "Dima/lib/mavlink/c_library_v2"
    if generated_source_tree.exists():
        violations.append(Violation(
            generated_source_tree, 1, "R330",
            "generated MAVLink code must remain under build/generated",
        ))
    _scan_parameter_generation_contract(violations)
    scan_mavlink_protocol_contract(violations)

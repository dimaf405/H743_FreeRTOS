#!/usr/bin/env python3
"""调用锁定的上游 Parameter YAML 工具，并生成 Dima 薄运行时合同。"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable


UPSTREAM_RELATIVE_ROOT = Path("tools/upstream/parameter_yaml_20260827")
PINNED_UPSTREAM_COMMIT = "1f6b6f61f8f42eaab0269c16a442cb580f954d7c"
SOURCE_MANIFEST_TOOL = (
    Path(__file__).resolve().parents[1] / "generation/source_manifest.py"
)
FORBIDDEN_PARAMETERS = (
    "CAL_MAG1_ID",
    "CAL_MAG1_ROT",
    "CAL_MAG2_ID",
    "CAL_MAG2_ROT",
    "SENS_DPRES_OFF",
)
RC_CALIBRATION_NAME = re.compile(
    r"^RC(?P<channel>[1-9][0-9]*)_(?P<field>[A-Z][A-Z0-9_]*)$"
)
PARAMETER_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")

GENERATED_HEADER = """/****************************************************************************
 * Generated from locked upstream Parameter YAML outputs. DO NOT EDIT.
 ****************************************************************************/
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the Dima parameter catalogue with locked upstream tools"
    )
    parser.add_argument("--yaml", action="append", required=True, type=Path)
    parser.add_argument(
        "--output", type=Path, default=Path("build/generated/parameters")
    )
    parser.add_argument("--include-output", required=True, type=Path)
    parser.add_argument("--upstream-root", type=Path, default=UPSTREAM_RELATIVE_ROOT)
    parser.add_argument("--board", default="dima_rover")
    return parser.parse_args()


def run_tool(arguments: list[str]) -> None:
    """上游工具始终作为独立进程执行，避免本地导入或改写其解析行为。"""
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONUTF8"] = "1"
    subprocess.run(arguments, check=True, env=environment)


def require_file(path: Path, description: str) -> Path:
    resolved = path.resolve()
    if not resolved.is_file():
        raise RuntimeError(f"missing {description}: {path}")
    return resolved


def verify_upstream_source(root: Path) -> None:
    """先核对上游目录闭包，禁止在固定 commit 名下静默替换脚本或模板。"""
    resolved_root = root.resolve()
    run_tool(
        [
            sys.executable,
            str(require_file(SOURCE_MANIFEST_TOOL, "source manifest verifier")),
            "--root",
            str(resolved_root),
            "--output",
            str(resolved_root / "SOURCE_MANIFEST.json"),
            "--project",
            "PX4-Autopilot",
            "--commit",
            PINNED_UPSTREAM_COMMIT,
            "--verify",
        ]
    )


def upstream_tools(root: Path) -> dict[str, Path]:
    root = root.resolve()
    return {
        "validate": require_file(root / "Tools/validate_yaml.py", "upstream YAML validator"),
        "schema": require_file(
            root / "validation/module_schema.yaml", "upstream module schema"
        ),
        "module": require_file(
            root / "Tools/module_config/generate_params.py",
            "upstream module parameter generator",
        ),
        "process": require_file(
            root / "src/lib/parameters/px_process_params.py",
            "upstream parameter metadata generator",
        ),
        "header": require_file(
            root / "src/lib/parameters/px_generate_params.py",
            "upstream parameter header generator",
        ),
    }


def reject_forbidden_names(paths: Iterable[Path], stage: str) -> None:
    """删除参数不得借助 YAML、派生产物或兼容别名重新进入协议目录。"""
    patterns = {
        name: re.compile(rf"(?<![A-Z0-9_]){re.escape(name)}(?![A-Z0-9_])")
        for name in FORBIDDEN_PARAMETERS
    }
    violations: list[str] = []
    for path in paths:
        text = path.read_text(encoding="utf-8")
        for name, pattern in patterns.items():
            if pattern.search(text):
                violations.append(f"{path}:{name}")
    if violations:
        raise RuntimeError(
            f"forbidden removed parameters found in {stage}: " + ", ".join(violations)
        )


def load_catalogue(path: Path) -> list[dict[str, Any]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    parameters = document.get("parameters")
    if not isinstance(parameters, list) or not parameters:
        raise RuntimeError("official upstream JSON has no parameter catalogue")

    names: list[str] = []
    for parameter in parameters:
        if not isinstance(parameter, dict):
            raise RuntimeError("official upstream JSON contains a non-object parameter")
        name = parameter.get("name")
        parameter_type = parameter.get("type")
        if not isinstance(name, str) or not PARAMETER_NAME.fullmatch(name):
            raise RuntimeError(f"invalid parameter name in official JSON: {name!r}")
        if parameter_type not in ("Int32", "Float"):
            raise RuntimeError(
                f"unsupported parameter type in official JSON: {name}={parameter_type!r}"
            )
        names.append(name)

    if len(names) != len(set(names)):
        raise RuntimeError("official upstream JSON contains duplicate parameter names")
    return parameters


def xml_parameter_names(path: Path) -> list[str]:
    names = [
        element.attrib["name"]
        for element in ET.parse(path).getroot().iter("parameter")
        if "name" in element.attrib
    ]
    if not names or len(names) != len(set(names)):
        raise RuntimeError("official upstream XML parameter catalogue is empty or duplicated")
    return names


def fixed_parameter(parameter: dict[str, Any]) -> bool:
    return (
        "min" in parameter
        and "max" in parameter
        and parameter["min"] == parameter["max"]
        and parameter.get("default") == parameter["min"]
    )


def parameter_storage_capacity(catalogue: list[dict[str, Any]]) -> int:
    """计算 BSON 全目录最坏大小：文档 5 B，加逐项类型、键、终止符和值。"""
    return 5 + sum(
        1
        + len(parameter["name"].encode("utf-8"))
        + 1
        + (8 if parameter["type"] == "Float" else 4)
        for parameter in catalogue
    )


def float_literal(value: Any) -> str:
    number = float(value)
    if math.isnan(number):
        return "NAN"
    if math.isinf(number):
        return "INFINITY" if number > 0.0 else "-INFINITY"
    text = repr(number)
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "F"


def snake_to_pascal(value: str) -> str:
    words = [word for word in value.lower().split("_") if word]
    if not words:
        raise RuntimeError("cannot generate an empty C++ identifier")
    identifier = "".join(word[0].upper() + word[1:] for word in words)
    if identifier[0].isdigit():
        identifier = "Role" + identifier
    return identifier


def rc_contract(
    catalogue: list[dict[str, Any]],
) -> tuple[
    list[tuple[int, list[tuple[str, str]]]],
    list[tuple[str, str]],
]:
    """仅从官方 JSON 推导 RC 结构，不回读 YAML 或维护参数名副本。"""
    calibration_by_channel: dict[int, list[tuple[str, str]]] = {}
    mapping: list[tuple[str, str]] = []

    for parameter in catalogue:
        name = parameter["name"]
        match = RC_CALIBRATION_NAME.fullmatch(name)
        if match and parameter.get("group") == "Radio Calibration":
            channel = int(match.group("channel"))
            field = match.group("field").lower()
            calibration_by_channel.setdefault(channel, []).append((field, name))

        if (
            parameter.get("group") == "RC Mapping"
            and name.startswith("RC_MAP_")
            and not fixed_parameter(parameter)
        ):
            mapping.append((snake_to_pascal(name.removeprefix("RC_MAP_")), name))

    channels = sorted(calibration_by_channel)
    if not channels or channels != list(range(1, channels[-1] + 1)):
        raise RuntimeError("RC calibration channels must be contiguous from channel 1")

    field_schema = [field for field, _ in calibration_by_channel[channels[0]]]
    if not field_schema or len(field_schema) != len(set(field_schema)):
        raise RuntimeError("RC calibration field schema is empty or duplicated")
    for channel in channels:
        fields = [field for field, _ in calibration_by_channel[channel]]
        if fields != field_schema:
            raise RuntimeError(
                f"RC{channel} calibration fields differ from RC1: {fields}"
            )

    mapping.sort(key=lambda item: item[0])
    roles = [role for role, _ in mapping]
    if not roles or len(roles) != len(set(roles)):
        raise RuntimeError("RC mapping roles are empty or duplicated")

    by_name = {parameter["name"]: parameter for parameter in catalogue}
    channel_limit = channels[-1]
    channel_count = by_name.get("RC_CHAN_CNT")
    if (
        channel_count is None
        or channel_count.get("min") != 0
        or channel_count.get("max") != channel_limit
    ):
        raise RuntimeError(f"RC_CHAN_CNT range must be 0..{channel_limit}")

    invalid_mapping_ranges = [
        name
        for _, name in mapping
        if by_name[name].get("min") != 0
        or by_name[name].get("max") != channel_limit
    ]
    if invalid_mapping_ranges:
        raise RuntimeError(
            f"RC mapping ranges must be 0..{channel_limit}: "
            f"{invalid_mapping_ranges}"
        )

    calibration = [
        (channel, calibration_by_channel[channel]) for channel in channels
    ]
    return calibration, mapping


def render_parameter_contract(
    catalogue: list[dict[str, Any]], xml_names: list[str]
) -> str:
    """从官方 XML/JSON 派生 Dima 固定值与 RC 绑定，不复制完整参数目录。"""
    json_names = {parameter["name"] for parameter in catalogue}
    if json_names != set(xml_names) or len(catalogue) != len(xml_names):
        raise RuntimeError(
            "official XML/JSON parameter catalogues differ: "
            f"xml={len(xml_names)} json={len(catalogue)}"
        )

    fixed = [parameter for parameter in catalogue if fixed_parameter(parameter)]
    if not fixed:
        raise RuntimeError("Dima fixed-parameter policy resolved to an empty set")
    calibration, mapping = rc_contract(catalogue)

    calibration_fields = [field for field, _ in calibration[0][1]]
    calibration_members = [f"    dima::params {field};" for field in calibration_fields]
    calibration_rows = [
        "    {"
        + ", ".join(f"dima::params::{name}" for _, name in fields)
        + "}"
        for _, fields in calibration
    ]

    mapping_roles = [f"    {role}," for role, _ in mapping]
    mapping_rows = [f"    dima::params::{name}," for _, name in mapping]

    fixed_rows: list[str] = []
    for parameter in fixed:
        if parameter["type"] == "Int32":
            fixed_rows.append(
                "    {dima::params::%s, FixedParameterType::Int32, %d, 0.0F},"
                % (parameter["name"], int(parameter["default"]))
            )
        else:
            fixed_rows.append(
                "    {dima::params::%s, FixedParameterType::Float, 0, %s},"
                % (parameter["name"], float_literal(parameter["default"]))
            )

    lines = [
        GENERATED_HEADER.rstrip(),
        "#pragma once",
        "",
        "#include <parameters/param.h>",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::generated::parameters {",
        "",
        "// 参数总数由官方 XML/JSON 闭包生成，并与 Dima 公开目录长度互证。",
        f"inline constexpr std::size_t kParameterCount = {len(catalogue)}U;",
        "static_assert(kParameterCount ==",
        "              sizeof(dima::parameter_catalog::parameters) /",
        "                  sizeof(dima::parameter_catalog::parameters[0]));",
        "// BSON 公式为 5 + Σ(type byte + UTF-8 name + NUL + wire value)。",
        "inline constexpr std::size_t kParameterStorageMaxBytes = "
        f"{parameter_storage_capacity(catalogue)}U;",
        "",
        "enum class FixedParameterType : std::uint8_t { Int32, Float };",
        "",
        "struct FixedParameterConstraint {",
        "    dima::params parameter;",
        "    FixedParameterType type;",
        "    std::int32_t int32_value;",
        "    float float_value;",
        "};",
        "",
        "// min=max=default 的产品约束从官方 JSON 结构化字段派生，消费者不得另列名称。",
        "inline constexpr FixedParameterConstraint kFixedParameterConstraints[]{",
        *fixed_rows,
        "};",
        "inline constexpr std::size_t kFixedParameterConstraintCount =",
        "    sizeof(kFixedParameterConstraints) / sizeof(kFixedParameterConstraints[0]);",
        "",
        "struct RcCalibrationParameters {",
        *calibration_members,
        "};",
        "",
        "// 每通道字段和通道数量均从官方 JSON 的 Radio Calibration 分组推导。",
        "inline constexpr RcCalibrationParameters kRcCalibrationParameters[]{",
        *[row + "," for row in calibration_rows],
        "};",
        "inline constexpr std::size_t kRcCalibrationChannelCount =",
        "    sizeof(kRcCalibrationParameters) / sizeof(kRcCalibrationParameters[0]);",
        f"inline constexpr std::size_t kRcCalibrationFieldCount = {len(calibration_fields)}U;",
        "",
        "enum class RcMappingRole : std::size_t {",
        *mapping_roles,
        "    Count,",
        "};",
        "",
        "// 枚举与数组由同一组 RC_MAP_* 官方 JSON 条目同时生成，索引不会人工漂移。",
        "inline constexpr dima::params kRcMappingParameters[]{",
        *mapping_rows,
        "};",
        "inline constexpr std::size_t kRcMappingParameterCount =",
        "    sizeof(kRcMappingParameters) / sizeof(kRcMappingParameters[0]);",
        "static_assert(kRcMappingParameterCount ==",
        "              static_cast<std::size_t>(RcMappingRole::Count));",
        "",
        "} // namespace dima::generated::parameters",
        "",
    ]
    return "\n".join(lines)


def write_include_tree(
    destination: Path, official_header: Path, parameter_contract: Path
) -> None:
    parameters = destination / "parameters"
    parameters.mkdir(parents=True, exist_ok=True)

    # 上游原始头仍完整保留在 build/generated；公开安装头只机械替换产品命名
    # 空间并增加一次包含保护。参数枚举、数组、类型和顺序不在适配层重渲染。
    header_text = official_header.read_text(encoding="utf-8")
    namespace_token = "namespace px4"
    if header_text.count(namespace_token) != 2:
        raise RuntimeError(
            "official parameter header has an unexpected namespace layout"
        )
    dima_header = header_text.replace(
        namespace_token, "namespace dima::parameter_catalog"
    )
    (parameters / "dima_parameters.hpp").write_text(
        "#pragma once\n" + dima_header,
        encoding="utf-8",
        newline="\n",
    )
    (parameters / "parameter_contract.hpp").write_bytes(
        parameter_contract.read_bytes()
    )


def make_staging_directory(target: Path) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    return Path(
        tempfile.mkdtemp(prefix=f".{target.name}.tmp-", dir=str(target.parent))
    )


def install_directories_atomically(pairs: list[tuple[Path, Path]]) -> None:
    """逐文件原子切换完整目录；任一切换失败会恢复全部旧文件。"""
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

            # Windows 可能拒绝替换一个刚关闭的目录；文件级 os.replace 仍保证
            # 每份公开合同不会以半写状态出现，stamp 只会在整个事务成功后生成。
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
            if backup_root.exists():
                shutil.rmtree(backup_root)


def main() -> int:
    args = parse_args()
    yaml_files = [require_file(path, "parameter YAML") for path in args.yaml]
    if len(yaml_files) != len(set(yaml_files)):
        raise RuntimeError("duplicate parameter YAML input")
    reject_forbidden_names(yaml_files, "authoritative YAML inputs")

    verify_upstream_source(args.upstream_root)
    tools = upstream_tools(args.upstream_root)
    output = args.output.resolve()
    include_output = args.include_output.resolve()
    output_stage = make_staging_directory(output)
    include_stage = make_staging_directory(include_output)

    try:
        module_params = output_stage / "module_params.c"
        parameters_xml = output_stage / "parameters.xml"
        parameters_json = output_stage / "parameters.json"

        # 正式链逐段调用锁定的上游原脚本；本地代码只负责输入、输出和失败边界。
        run_tool(
            [
                sys.executable,
                str(tools["validate"]),
                *(str(path) for path in yaml_files),
                "--schema-file",
                str(tools["schema"]),
            ]
        )
        run_tool(
            [
                sys.executable,
                str(tools["module"]),
                "--config-files",
                *(str(path) for path in yaml_files),
                "--params-file",
                str(module_params),
                "--board",
                args.board,
            ]
        )
        run_tool(
            [
                sys.executable,
                str(tools["process"]),
                "--src-path",
                str(output_stage),
                "--xml",
                str(parameters_xml),
                "--json",
                str(parameters_json),
                "--board",
                args.board,
            ]
        )
        run_tool(
            [
                sys.executable,
                str(tools["header"]),
                "--xml",
                str(parameters_xml),
                "--dest",
                str(output_stage),
            ]
        )

        official_header = output_stage / "px4_parameters.hpp"
        require_file(official_header, "upstream generated parameter header")
        catalogue = load_catalogue(parameters_json)
        xml_names = xml_parameter_names(parameters_xml)
        contract = output_stage / "parameter_contract.hpp"
        contract.write_text(
            render_parameter_contract(catalogue, xml_names),
            encoding="utf-8",
            newline="\n",
        )
        reject_forbidden_names(
            [module_params, parameters_xml, parameters_json, official_header, contract],
            "official and adapted parameter outputs",
        )
        write_include_tree(include_stage, official_header, contract)

        install_directories_atomically(
            [(output_stage, output), (include_stage, include_output)]
        )
    finally:
        for stage in (output_stage, include_stage):
            if stage.exists():
                shutil.rmtree(stage)

    print(f"generated {len(catalogue)} Dima YAML parameters in {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

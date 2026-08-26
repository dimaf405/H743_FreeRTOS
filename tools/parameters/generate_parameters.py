#!/usr/bin/env python3
"""以参数定义 C 源为权威输入，调用 PX4 parser 生成 Dima 参数目录及派生合同。"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

from generate_header import generate

BSD_HEADER = """/****************************************************************************
 * Generated from PX4 parameter definitions. DO NOT EDIT.
 ****************************************************************************/
"""

PARAMETER_DEFINITION = re.compile(
    r"\bPARAM_DEFINE_(?:INT32|FLOAT)\s*\(\s*"
    r"([A-Z][A-Z0-9_]*)\s*,"
)
RC_CALIBRATION_NAME = re.compile(r"^RC(?P<channel>[1-9][0-9]*)_(?P<field>[A-Z][A-Z0-9_]*)$")


def ordered_json_parameters(path: Path, ordered_names: list[str]) -> list[dict]:
    """把 JSON 重排为生成 handle 顺序，并拒绝名称缺失、重复或额外参数。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    items = data.get("parameters", [])
    if not isinstance(items, list):
        raise RuntimeError("official JSON has no parameters list")
    by_name = {item["name"]: item for item in items}
    if len(by_name) != len(items) or set(by_name) != set(ordered_names):
        raise RuntimeError("JSON catalogue differs from generated handles")
    items = [by_name[name] for name in ordered_names]
    data["parameters"] = items
    path.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return items


def float_literal(value: str) -> str:
    text = value.strip().rstrip("fF")
    if text.lower() in ("nan", "+nan", "-nan"):
        return "NAN"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def write_metadata(path: Path, parameters) -> None:
    rows = []
    for parameter in parameters:
        attributes = parameter.attrib
        parameter_type = attributes["type"]
        if parameter_type == "FLOAT":
            value = f"{{.f = {float_literal(attributes['default'])}}}"
        elif parameter_type == "INT32":
            value = f"{{.i = {attributes['default']}}}"
        else:
            raise RuntimeError(f"unsupported parameter type {parameter_type}")
        rows.append(f'    {{"{attributes["name"]}", PARAM_TYPE_{parameter_type}, {value}}}')

    text = (
        BSD_HEADER
        + '#include "parameters/param.h"\n\n'
        + f"#define PARAM_INFO_COUNT {len(parameters)}\n\n"
        + "const param_info_s param_info[PARAM_INFO_COUNT] = {\n"
        + ",\n".join(rows)
        + "\n};\n\n"
        + "const uint16_t param_info_count = PARAM_INFO_COUNT;\n"
    )
    path.write_text(text, encoding="utf-8")


def fixed_parameter(item: dict) -> bool:
    return (
        "min" in item
        and "max" in item
        and item["min"] == item["max"]
        and item["default"] == item["min"]
    )


def parameter_names_in_source_order(sources: list[Path]) -> list[str]:
    """从权威定义文件读取 PARAM_DEFINE 顺序，不在生成器内复制参数列表。"""
    names: list[str] = []
    for source in sources:
        names.extend(PARAMETER_DEFINITION.findall(source.read_text(encoding="utf-8")))
    if not names or len(names) != len(set(names)):
        raise RuntimeError("parameter definitions are empty or duplicated")
    return names


def qgc_compatibility_names(sources: list[Path]) -> set[str]:
    candidates = [path for path in sources if path.name == "qgc_compat_params.c"]
    if len(candidates) != 1:
        raise RuntimeError(
            "parameter generation requires exactly one qgc_compat_params.c source"
        )
    names = PARAMETER_DEFINITION.findall(candidates[0].read_text(encoding="utf-8"))
    if not names or len(names) != len(set(names)):
        raise RuntimeError("QGC compatibility parameter definitions are empty or duplicated")
    return set(names)


def rc_runtime_contract(
    catalog: list[dict], sources: list[Path]
) -> tuple[list[tuple[int, list[tuple[str, str]]]], list[str]]:
    """从权威定义顺序推导 RC 校准结构和非固定功能映射。

    校准字段名取每个 ``RC<通道>_<字段>`` 的真实后缀，通道 1 的定义顺序是
    结构 schema；所有其余通道必须拥有完全相同的字段序列。映射同样按定义文件
    顺序生成，并自动排除 min=max 的 QGC 固定兼容句柄。
    """
    by_name = {item["name"]: item for item in catalog}
    source_names = parameter_names_in_source_order(sources)
    if set(source_names) != set(by_name):
        raise RuntimeError("parameter source order differs from generated catalogue")

    calibration_by_channel: dict[int, list[tuple[str, str]]] = {}
    mapping_names: list[str] = []
    for name in source_names:
        item = by_name[name]
        match = RC_CALIBRATION_NAME.fullmatch(name)
        if match and item.get("group") == "Radio Calibration":
            channel = int(match.group("channel"))
            calibration_by_channel.setdefault(channel, []).append(
                (match.group("field").lower(), name)
            )
        if (
            item.get("group") == "RC Mapping"
            and name.startswith("RC_MAP_")
            and not fixed_parameter(item)
        ):
            mapping_names.append(name)

    channels = sorted(calibration_by_channel)
    if not channels or channels != list(range(1, channels[-1] + 1)):
        raise RuntimeError("RC calibration channels must be contiguous from channel 1")
    field_schema = [field for field, _ in calibration_by_channel[1]]
    if not field_schema or len(field_schema) != len(set(field_schema)):
        raise RuntimeError("RC calibration field schema is empty or duplicated")
    for channel in channels:
        fields = [field for field, _ in calibration_by_channel[channel]]
        if fields != field_schema:
            raise RuntimeError(
                f"RC{channel} calibration fields differ from channel 1: {fields}"
            )
    if not mapping_names:
        raise RuntimeError("RC runtime mapping contract is empty")
    return [(channel, calibration_by_channel[channel]) for channel in channels], mapping_names


def write_parameter_contract(
    path: Path, parameters, catalog: list[dict], sources: list[Path]
) -> None:
    """从 parser 结果生成 MAVLink public/QGC-required/fixed 集合，不手写参数名。"""
    qgc_required = [
        parameter.attrib["name"]
        for parameter in parameters
        if parameter.attrib.get("qgc_required") == "true"
    ]
    if not qgc_required:
        raise RuntimeError("no @qgc_required parameters were generated")

    fixed = [item for item in catalog if fixed_parameter(item)]
    fixed_names = {item["name"] for item in fixed}
    compatibility_names = qgc_compatibility_names(sources)
    if not compatibility_names.issubset(fixed_names):
        missing = sorted(compatibility_names - fixed_names)
        raise RuntimeError(
            "QGC compatibility parameters must use identical default/min/max "
            f"metadata: {missing}"
        )

    calibration, rc_mapping_names = rc_runtime_contract(catalog, sources)

    # 所有数组都从同一 parser/catalogue 推导，运行期只能消费这份生成合同。
    public_rows = [f"    px4::params::{item['name']}" for item in catalog]
    required_rows = [f"    px4::params::{name}" for name in qgc_required]
    fixed_rows = []
    for item in fixed:
        if item["type"] == "Int32":
            fixed_rows.append(
                "    {px4::params::%s, FixedParameterType::Int32, %d, 0.0F}"
                % (item["name"], int(item["default"]))
            )
        elif item["type"] == "Float":
            fixed_rows.append(
                "    {px4::params::%s, FixedParameterType::Float, 0, %s}"
                % (item["name"], float_literal(str(item["default"])).upper())
            )
        else:
            raise RuntimeError(
                f"unsupported fixed parameter type {item['type']}: {item['name']}"
            )

    calibration_fields = [field for field, _ in calibration[0][1]]
    calibration_member_rows = [
        f"    px4::params {field};" for field in calibration_fields
    ]
    calibration_rows = []
    for _, fields in calibration:
        calibration_rows.append(
            "    {" + ", ".join(
                f"px4::params::{name}" for _, name in fields
            ) + "}"
        )
    rc_mapping_rows = [
        f"    px4::params::{name}" for name in rc_mapping_names
    ]

    text = (
        BSD_HEADER
        + "#pragma once\n\n"
        + "#include <parameters/px4_parameters.hpp>\n\n"
        + "#include <cstddef>\n"
        + "#include <cstdint>\n\n"
        + "namespace dima::generated::parameters {\n\n"
        + "enum class FixedParameterType : std::uint8_t { Int32, Float };\n\n"
        + "struct FixedParameterConstraint {\n"
        + "    px4::params parameter;\n"
        + "    FixedParameterType type;\n"
        + "    std::int32_t int32_value;\n"
        + "    float float_value;\n"
        + "};\n\n"
        + "struct RcCalibrationParameters {\n"
        + "\n".join(calibration_member_rows)
        + "\n};\n\n"
        + "inline constexpr RcCalibrationParameters kRcCalibrationParameters[]{\n"
        + ",\n".join(calibration_rows)
        + "\n};\n\n"
        + "inline constexpr px4::params kRcMappingParameters[]{\n"
        + ",\n".join(rc_mapping_rows)
        + "\n};\n\n"
        + "inline constexpr px4::params kMavlinkPublicParameters[]{\n"
        + ",\n".join(public_rows)
        + "\n};\n\n"
        + "inline constexpr px4::params kQgcRequiredParameters[]{\n"
        + ",\n".join(required_rows)
        + "\n};\n\n"
        + "inline constexpr FixedParameterConstraint kFixedParameterConstraints[]{\n"
        + ",\n".join(fixed_rows)
        + "\n};\n\n"
        + "inline constexpr std::size_t kMavlinkPublicParameterCount =\n"
        + "    sizeof(kMavlinkPublicParameters) / sizeof(kMavlinkPublicParameters[0]);\n"
        + "inline constexpr std::size_t kQgcRequiredParameterCount =\n"
        + "    sizeof(kQgcRequiredParameters) / sizeof(kQgcRequiredParameters[0]);\n"
        + "inline constexpr std::size_t kFixedParameterConstraintCount =\n"
        + "    sizeof(kFixedParameterConstraints) / sizeof(kFixedParameterConstraints[0]);\n\n"
        + "inline constexpr std::size_t kRcCalibrationChannelCount =\n"
        + "    sizeof(kRcCalibrationParameters) / sizeof(kRcCalibrationParameters[0]);\n"
        + f"inline constexpr std::size_t kRcCalibrationFieldCount = {len(calibration_fields)}U;\n"
        + "inline constexpr std::size_t kRcMappingParameterCount =\n"
        + "    sizeof(kRcMappingParameters) / sizeof(kRcMappingParameters[0]);\n\n"
        + "} // namespace dima::generated::parameters\n"
    )
    path.write_text(text, encoding="utf-8")


def write_forward_header(path: Path, include_target: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f'#pragma once\n#include "{include_target}"\n', encoding="utf-8")


def write_include_forwarders(
    include_output: Path, generated_header: Path, generated_contract: Path
) -> None:
    common = include_output / "px4_platform_common"
    write_forward_header(common / "param.h", "parameters/param.h")
    # DrvFS 上的转发 include 会把 Windows 盘符写入 GCC .d 文件并破坏 GNU Make 解析。
    parameter_header = include_output / "parameters" / "px4_parameters.hpp"
    parameter_header.parent.mkdir(parents=True, exist_ok=True)
    parameter_header.write_bytes(generated_header.read_bytes())
    contract_header = include_output / "parameters" / "parameter_contract.hpp"
    contract_header.write_bytes(generated_contract.read_bytes())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", action="append", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("build/generated/parameters"))
    parser.add_argument("--include-output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    """串联 XML/JSON/header/metadata/forwarder 生成，并交叉核对顺序与数量。"""
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    process = Path(__file__).with_name("process_parameters.py")
    xml_path = args.output / "parameters.xml"
    json_path = args.output / "parameters.json"
    subprocess.run([
        sys.executable, str(process), "--src-file",
        *(str(path) for path in args.source),
        "--xml", str(xml_path), "--json", str(json_path), "--board", "dima_rover",
    ], check=True)

    parameters = generate(xml_path, args.output)
    xml_names = [parameter.attrib["name"] for parameter in parameters]
    catalog = ordered_json_parameters(json_path, xml_names)
    generated_json_names = [item["name"] for item in catalog]
    if not parameters or xml_names != generated_json_names:
        raise RuntimeError(
            f"catalog mismatch: xml/header={len(parameters)} json={len(generated_json_names)}"
        )

    write_metadata(args.output / "parameter_metadata.c", parameters)
    contract_path = args.output / "parameter_contract.hpp"
    write_parameter_contract(contract_path, parameters, catalog, args.source)
    write_include_forwarders(
        args.include_output, args.output / "px4_parameters.hpp", contract_path
    )
    print(f"generated {len(parameters)} parameters in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

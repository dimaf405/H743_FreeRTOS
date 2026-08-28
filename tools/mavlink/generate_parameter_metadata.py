#!/usr/bin/env python3
"""从生成参数目录构造只读 PX4/QGC Component Metadata、XZ 载荷与嵌入头。"""

from __future__ import annotations

import argparse
import hashlib
import json
import lzma
import math
import re
from pathlib import Path


GENERAL_URI = "mftp://etc/extras/component_general.json.xz"
PARAMETER_URI = "mftp://etc/extras/parameters.json.xz"
ACTUATOR_URI = "mftp://etc/extras/actuators.json.xz"
PARAMETER_NAME = re.compile(r"^[.\-a-zA-Z0-9_{}]{1,16}$")
PARAMETER_TYPES = {"Int32", "Float"}
CALIBRATION_ROLE_NAME = re.compile(
    r"^CAL_(?P<sensor>ACC|GYRO|MAG)(?P<instance>[0-9]+)_"
    r"(?P<role>ID|ROT)$"
)
CALIBRATION_VALUE_NAME = re.compile(
    r"^CAL_(?P<sensor>ACC|GYRO|MAG)(?P<instance>[0-9]+)_"
    r"(?P<axis>[XYZ])(?P<role>OFF|SCALE)$"
)
PARAMETER_FIELDS = {
    "name", "type", "shortDesc", "longDesc", "units", "default",
    "decimalPlaces", "min", "max", "increment", "rebootRequired",
    "group", "category", "volatile", "values", "bitmask",
}


def validate_metadata_contract_constants() -> None:
    """锁定 QGC URI 与 PX4 CRC 参考向量，防止生成器算法静默漂移。"""
    expected_uris = (
        "mftp://etc/extras/component_general.json.xz",
        "mftp://etc/extras/parameters.json.xz",
        "mftp://etc/extras/actuators.json.xz",
    )
    if (GENERAL_URI, PARAMETER_URI, ACTUATOR_URI) != expected_uris:
        raise RuntimeError("Dima component Metadata URI contract changed")
    if mavlink_crc32(b"123456789") != 0x2DFD2D88:
        raise RuntimeError("PX4 component Metadata CRC32 semantics changed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parameters", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--verify", action="store_true")
    return parser.parse_args()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def json_bytes(value: object) -> bytes:
    """使用稳定排序和紧凑分隔符生成确定性 UTF-8；同一输入必须逐字节相同。"""
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def compress_xz(data: bytes) -> bytes:
    return lzma.compress(
        data, format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC64, preset=9
    )


def mavlink_crc32(data: bytes) -> int:
    """PX4 Component Metadata CRC32：初值为 0，不执行最终异或。"""
    table = []
    for value in range(256):
        current = value
        for _ in range(8):
            if current & 1:
                current ^= 0x1DB710640
            current >>= 1
        table.append(current)

    crc = 0
    for value in data:
        crc = (crc >> 8) ^ table[(crc & 0xFF) ^ value]
    return crc & 0xFFFFFFFF


def validate_embedded_header(header: bytes,
                             general_xz: bytes,
                             parameter_xz: bytes,
                             actuator_xz: bytes,
                             general_crc: int,
                             parameter_crc: int,
                             actuator_crc: int) -> None:
    """Independently decode the generated C++ header contract."""
    text = header.decode("utf-8")
    string_items = re.findall(
        r'inline constexpr char (k\w+)\[\] = "([^"]*)";', text
    )
    strings = dict(string_items)
    expected_strings = {
        "kGeneralUri": GENERAL_URI,
        "kGeneralPath": "/etc/extras/component_general.json.xz",
        "kParameterPath": "/etc/extras/parameters.json.xz",
        "kActuatorPath": "/etc/extras/actuators.json.xz",
    }
    if len(string_items) != len(expected_strings) or strings != expected_strings:
        raise RuntimeError("embedded Metadata URI/path mapping changed")

    crc_items = re.findall(
        r"inline constexpr std::uint32_t (k\w+Crc) = "
        r"0x([0-9a-fA-F]{8})U;",
        text,
    )
    crc_values = {
        name: int(value, 16)
        for name, value in crc_items
    }
    expected_crcs = {
        "kGeneralCrc": general_crc,
        "kParameterCrc": parameter_crc,
        "kActuatorCrc": actuator_crc,
    }
    if len(crc_items) != len(expected_crcs) or crc_values != expected_crcs:
        raise RuntimeError("embedded Metadata CRC mapping changed")

    array_items = re.findall(
        r"alignas\(4\) inline constexpr std::uint8_t "
        r"(k\w+File)\[\]\{(.*?)\n\};",
        text,
        re.DOTALL,
    )
    arrays: dict[str, bytes] = {}
    for name, body in array_items:
        values = re.findall(r"0x([0-9a-fA-F]{2})\s*,", body)
        residue = re.sub(r"0x[0-9a-fA-F]{2}\s*,", "", body)
        if residue.strip():
            raise RuntimeError(f"embedded Metadata array {name} is malformed")
        arrays[name] = bytes(int(value, 16) for value in values)
    expected_arrays = {
        "kGeneralFile": general_xz,
        "kParameterFile": parameter_xz,
        "kActuatorFile": actuator_xz,
    }
    if len(array_items) != len(expected_arrays) or arrays != expected_arrays:
        raise RuntimeError("embedded Metadata file mapping changed")

    size_items = re.findall(
        r"inline constexpr std::size_t (k\w+FileSize) = "
        r"sizeof\((k\w+File)\);",
        text,
    )
    size_mappings = dict(size_items)
    expected_sizes = {
        "kGeneralFileSize": "kGeneralFile",
        "kParameterFileSize": "kParameterFile",
        "kActuatorFileSize": "kActuatorFile",
    }
    if len(size_items) != len(expected_sizes) or size_mappings != expected_sizes:
        raise RuntimeError("embedded Metadata size mapping changed")


def verify_generated_metadata(parameters_path: Path,
                              output: Path) -> None:
    validate_metadata_contract_constants()
    stamp_path = output / ".generated.json"
    stamp = json.loads(stamp_path.read_text(encoding="utf-8"))
    if stamp.get("format") != 1:
        raise RuntimeError("component Metadata stamp format changed")
    records = stamp.get("outputs")
    expected_names = {
        "component_general.json",
        "component_general.json.xz",
        "parameters.json",
        "parameters.json.xz",
        "actuators.json",
        "actuators.json.xz",
        "parameter_metadata_files.hpp",
    }
    if not isinstance(records, dict) or set(records) != expected_names:
        raise RuntimeError("component Metadata stamp output set changed")
    if stamp.get("input_sha256") != sha256(parameters_path.read_bytes()):
        raise RuntimeError("component Metadata input SHA does not match")

    output_data: dict[str, bytes] = {}
    for filename in sorted(expected_names):
        data = (output / filename).read_bytes()
        output_data[filename] = data
        record = records.get(filename)
        if (not isinstance(record, dict) or record.get("size") != len(data) or
                record.get("sha256") != sha256(data)):
            raise RuntimeError(
                f"component Metadata stamp mismatch for {filename}"
            )

    general_json = output_data["component_general.json"]
    general_xz = output_data["component_general.json.xz"]
    parameter_json = output_data["parameters.json"]
    parameter_xz = output_data["parameters.json.xz"]
    actuator_json = output_data["actuators.json"]
    actuator_xz = output_data["actuators.json.xz"]
    if (lzma.decompress(general_xz) != general_json or
            lzma.decompress(parameter_xz) != parameter_json or
            lzma.decompress(actuator_xz) != actuator_json):
        raise RuntimeError("component Metadata XZ content mismatch")
    general_crc = mavlink_crc32(general_json)
    parameter_crc = mavlink_crc32(parameter_xz)
    actuator_crc = mavlink_crc32(actuator_xz)
    if stamp.get("general_crc32") != general_crc or \
            stamp.get("parameter_crc32") != parameter_crc or \
            stamp.get("actuator_crc32") != actuator_crc:
        raise RuntimeError("component Metadata CRC stamp mismatch")

    source = json.loads(parameters_path.read_text(encoding="utf-8"))
    parameters = source.get("parameters")
    if (not isinstance(parameters, list) or
            stamp.get("public_parameter_count") != len(parameters)):
        raise RuntimeError("component Metadata parameter count mismatch")
    by_name = validate_parameter_catalogue(parameters)
    validate_sensor_metadata(by_name)

    expected_parameter_json = json_bytes({
        "parameters": parameters,
        "version": 1,
    })
    expected_actuator_json = build_actuator_metadata(by_name)
    expected_general_json = json_bytes({
        "metadataTypes": [
            {"fileCrc": parameter_crc, "type": 1, "uri": PARAMETER_URI},
            {"fileCrc": actuator_crc, "type": 5, "uri": ACTUATOR_URI},
        ],
        "version": 1,
    })
    if parameter_json != expected_parameter_json:
        raise RuntimeError("parameter Metadata content contract changed")
    if actuator_json != expected_actuator_json:
        raise RuntimeError("actuator Metadata content contract changed")
    if general_json != expected_general_json:
        raise RuntimeError("General Metadata content contract changed")

    validate_embedded_header(
        output_data["parameter_metadata_files.hpp"],
        general_xz, parameter_xz, actuator_xz,
        general_crc, parameter_crc, actuator_crc,
    )


def is_number(value: object) -> bool:
    return (isinstance(value, (int, float)) and
            not isinstance(value, bool) and math.isfinite(value))


def validate_parameter(parameter: object, index: int) -> None:
    """Validate Dima's supported QGC v1 parameter-object subset."""
    if not isinstance(parameter, dict):
        raise RuntimeError(f"parameter {index} is not an object")

    unexpected = set(parameter) - PARAMETER_FIELDS
    if unexpected:
        raise RuntimeError(
            f"parameter {index} has unsupported fields: {sorted(unexpected)}"
        )

    name = parameter.get("name")
    parameter_type = parameter.get("type")
    if not isinstance(name, str) or PARAMETER_NAME.fullmatch(name) is None:
        raise RuntimeError(f"parameter {index} has an invalid QGC name")
    if parameter_type not in PARAMETER_TYPES:
        raise RuntimeError(
            f"parameter {name} has unsupported type {parameter_type!r}"
        )

    for field in ("shortDesc", "longDesc", "units", "group", "category"):
        if field in parameter and not isinstance(parameter[field], str):
            raise RuntimeError(f"parameter {name} field {field} is not text")
    for field in ("rebootRequired", "volatile"):
        if field in parameter and not isinstance(parameter[field], bool):
            raise RuntimeError(f"parameter {name} field {field} is not boolean")

    decimal_places = parameter.get("decimalPlaces")
    if (decimal_places is not None and
            (not isinstance(decimal_places, int) or
             isinstance(decimal_places, bool) or decimal_places < 0)):
        raise RuntimeError(
            f"parameter {name} has invalid decimalPlaces"
        )

    for field in ("default", "min", "max", "increment"):
        if field in parameter and not is_number(parameter[field]):
            raise RuntimeError(f"parameter {name} field {field} is not finite")
        if (field in parameter and parameter_type == "Int32" and
                (not isinstance(parameter[field], int) or
                 not -(2 ** 31) <= parameter[field] < 2 ** 31)):
            raise RuntimeError(
                f"parameter {name} field {field} is not a valid Int32"
            )

    minimum = parameter.get("min")
    maximum = parameter.get("max")
    default = parameter.get("default")
    if minimum is not None and maximum is not None and minimum > maximum:
        raise RuntimeError(f"parameter {name} has min greater than max")
    if (default is not None and minimum is not None and default < minimum) or (
            default is not None and maximum is not None and default > maximum):
        raise RuntimeError(f"parameter {name} default is outside its range")
    if "increment" in parameter and parameter["increment"] <= 0:
        raise RuntimeError(f"parameter {name} increment must be positive")

    values = parameter.get("values")
    if values is not None:
        if not isinstance(values, list):
            raise RuntimeError(f"parameter {name} values is not an array")
        seen_values: set[int | float] = set()
        for value_index, entry in enumerate(values):
            if (not isinstance(entry, dict) or set(entry) !=
                    {"value", "description"} or
                    not is_number(entry.get("value")) or
                    not isinstance(entry.get("description"), str)):
                raise RuntimeError(
                    f"parameter {name} has invalid enum entry {value_index}"
                )
            enum_value = entry["value"]
            if (parameter_type == "Int32" and
                    (not isinstance(enum_value, int) or
                     not -(2 ** 31) <= enum_value < 2 ** 31)):
                raise RuntimeError(
                    f"parameter {name} enum {enum_value!r} is not Int32"
                )
            if enum_value in seen_values:
                raise RuntimeError(
                    f"parameter {name} repeats enum value {enum_value}"
                )
            seen_values.add(enum_value)
            if ((minimum is not None and enum_value < minimum) or
                    (maximum is not None and enum_value > maximum)):
                raise RuntimeError(
                    f"parameter {name} enum {enum_value} is outside its range"
                )

    bitmask = parameter.get("bitmask")
    if bitmask is not None:
        if not isinstance(bitmask, list):
            raise RuntimeError(f"parameter {name} bitmask is not an array")
        seen_indices: set[int] = set()
        for bit_index, entry in enumerate(bitmask):
            if (not isinstance(entry, dict) or set(entry) !=
                    {"index", "description"} or
                    not isinstance(entry.get("index"), int) or
                    isinstance(entry.get("index"), bool) or
                    not 0 <= entry["index"] <= 31 or
                    not isinstance(entry.get("description"), str)):
                raise RuntimeError(
                    f"parameter {name} has invalid bitmask entry {bit_index}"
                )
            if entry["index"] in seen_indices:
                raise RuntimeError(
                    f"parameter {name} repeats bit index {entry['index']}"
                )
            seen_indices.add(entry["index"])


def validate_parameter_catalogue(parameters: list[object]) -> dict[str, dict]:
    for index, parameter in enumerate(parameters):
        validate_parameter(parameter, index)
    names = [parameter["name"] for parameter in parameters]
    if len(names) != len(set(names)):
        raise RuntimeError("parameter catalogue contains duplicate names")
    return {parameter["name"]: parameter for parameter in parameters}


def validate_sensor_metadata(by_name: dict[str, dict]) -> None:
    # 参数名只承担可解析的结构角色；实际实例集合完全由生成目录发现，新增
    # ACC/GYRO/MAG 实例时无需在 Metadata 工具中同步维护参数清单。
    calibration_roles = [
        (name, parameter, match)
        for name, parameter in by_name.items()
        if (match := CALIBRATION_ROLE_NAME.fullmatch(name)) is not None
    ]
    calibration_values = [
        (name, parameter, match)
        for name, parameter in by_name.items()
        if (match := CALIBRATION_VALUE_NAME.fullmatch(name)) is not None
    ]
    if not calibration_roles or not calibration_values:
        raise RuntimeError("PX4 sensor calibration Metadata roles are empty")

    for name, parameter, match in calibration_roles:
        if (parameter.get("type") != "Int32" or
                parameter.get("group") != "Sensor Calibration" or
                parameter.get("category") != "System"):
            raise RuntimeError(
                f"PX4 sensor calibration Metadata contract invalid for {name}"
            )
    for name, parameter, match in calibration_values:
        if (parameter.get("type") != "Float" or
                parameter.get("group") != "Sensor Calibration" or
                parameter.get("category") != "System" or
                parameter.get("decimalPlaces") != 3 or
                parameter.get("volatile", False)):
            raise RuntimeError(
                f"PX4 calibration value Metadata contract invalid for {name}"
            )
        if match.group("role") == "SCALE" and (
                parameter.get("default") != 1.0 or
                parameter.get("min") != 0.1 or
                parameter.get("max") != 3.0):
            raise RuntimeError(
                f"PX4 calibration scale range invalid for {name}"
            )
        if (match.group("role") == "OFF" and
                parameter.get("default") != 0.0):
            raise RuntimeError(
                f"PX4 calibration offset default invalid for {name}"
            )

    integration = by_name.get("IMU_INTEG_RATE")
    if (integration is None or integration.get("type") != "Int32" or
            integration.get("group") != "Sensors" or
            integration.get("default") != 200 or
            integration.get("min") != 100 or integration.get("max") != 400 or
            {entry.get("value") for entry in integration.get("values", [])} !=
            {100, 200, 250, 400}):
        raise RuntimeError("IMU_INTEG_RATE Metadata contract invalid")

    clipping = by_name.get("SENS_IMU_CLPNOTI")
    if (clipping is None or clipping.get("type") != "Int32" or
            clipping.get("group") != "Sensors" or
            clipping.get("category") != "System" or
            clipping.get("default") != 1 or
            {entry.get("value") for entry in clipping.get("values", [])} !=
            {0, 1}):
        raise RuntimeError("SENS_IMU_CLPNOTI Metadata contract invalid")

    mag_rate = by_name.get("SENS_MAG_RATE")
    if (mag_rate is None or mag_rate.get("type") != "Float" or
            mag_rate.get("group") != "Sensors" or
            mag_rate.get("default") != 15.0 or
            mag_rate.get("min") != 1 or mag_rate.get("max") != 200):
        raise RuntimeError("SENS_MAG_RATE Metadata contract invalid")


def array_lines(data: bytes) -> list[str]:
    rows = []
    for offset in range(0, len(data), 12):
        chunk = data[offset:offset + 12]
        rows.append("    " + ", ".join(f"0x{value:02x}" for value in chunk) + ",")
    return rows


def generated_header(general_xz: bytes, parameter_xz: bytes, actuator_xz: bytes,
                     general_crc: int, parameter_crc: int,
                     actuator_crc: int) -> str:
    lines = [
        "#pragma once",
        "",
        "// Generated parameter Component Metadata. DO NOT EDIT.",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace dima::generated::parameter_metadata {",
        "",
        f'inline constexpr char kGeneralUri[] = "{GENERAL_URI}";',
        'inline constexpr char kGeneralPath[] = "/etc/extras/component_general.json.xz";',
        'inline constexpr char kParameterPath[] = "/etc/extras/parameters.json.xz";',
        'inline constexpr char kActuatorPath[] = "/etc/extras/actuators.json.xz";',
        f"inline constexpr std::uint32_t kGeneralCrc = 0x{general_crc:08x}U;",
        f"inline constexpr std::uint32_t kParameterCrc = 0x{parameter_crc:08x}U;",
        f"inline constexpr std::uint32_t kActuatorCrc = 0x{actuator_crc:08x}U;",
        "",
        "alignas(4) inline constexpr std::uint8_t kGeneralFile[]{",
        *array_lines(general_xz),
        "};",
        "",
        "alignas(4) inline constexpr std::uint8_t kParameterFile[]{",
        *array_lines(parameter_xz),
        "};",
        "",
        "alignas(4) inline constexpr std::uint8_t kActuatorFile[]{",
        *array_lines(actuator_xz),
        "};",
        "",
        "inline constexpr std::size_t kGeneralFileSize = sizeof(kGeneralFile);",
        "inline constexpr std::size_t kParameterFileSize = sizeof(kParameterFile);",
        "inline constexpr std::size_t kActuatorFileSize = sizeof(kActuatorFile);",
        "",
        "} // namespace dima::generated::parameter_metadata",
        "",
    ]
    return "\n".join(lines)


ACTUATOR_FUNCTIONS = [
    (0, "Disabled"),
    (101, "Motor right"),
    (102, "Motor left"),
]
ACTUATOR_FUNCTIONS_EXCLUDE_TEST = {101, 102}


def validate_output_only_actuator_metadata(metadata: dict) -> None:
    """Validate Dima's intentional QGC output-only Actuators adaptation."""
    expected_top_level = {"functions_v1", "mixer_v1", "outputs_v1", "version"}
    mixer = metadata.get("mixer_v1")
    outputs = metadata.get("outputs_v1")
    expected_mixer = {
        "actuator-types": {"DEFAULT": {"values": {"min": 0, "max": 1}}},
        "config": [],
    }
    if (set(metadata) != expected_top_level or metadata.get("version") != 1 or
            mixer != expected_mixer or
            not isinstance(outputs, list) or len(outputs) != 1):
        raise RuntimeError(
            "Dima actuator Metadata must keep an empty mixer and one output"
        )

    subgroups = outputs[0].get("subgroups")
    if (set(outputs[0]) != {"label", "subgroups"} or
            outputs[0].get("label") != "PWM Output" or
            not isinstance(subgroups, list) or len(subgroups) != 1):
        raise RuntimeError("Dima actuator output subgroup contract changed")
    subgroup = subgroups[0]
    expected_channels = [
        {"label": f"S{channel}", "param-index": channel}
        for channel in range(1, 7)
    ]
    if (set(subgroup) != {"channels", "per-channel-parameters"} or
            subgroup.get("channels") != expected_channels):
        raise RuntimeError("Dima actuator output channels must remain S1..S6")

    expected_parameters = [
        {"label": "Function", "name": "PWM_S${i}_FUNC",
         "function": "function"},
        {"label": "Minimum", "name": "PWM_S${i}_MIN",
         "function": "min"},
        {"label": "Center\n(for Servos)", "name": "PWM_S${i}_CENT"},
        {"label": "Maximum", "name": "PWM_S${i}_MAX",
         "function": "max"},
        {"label": "Reversed", "name": "PWM_S${i}_REV"},
    ]
    if subgroup.get("per-channel-parameters") != expected_parameters:
        raise RuntimeError("Dima actuator per-channel parameter contract changed")

    expected_functions = {
        "0": {"label": "Disabled"},
        "101": {"label": "Motor right",
                "exclude-from-actuator-testing": True},
        "102": {"label": "Motor left",
                "exclude-from-actuator-testing": True},
    }
    if metadata.get("functions_v1") != expected_functions:
        raise RuntimeError("Dima actuator function registry changed")


def build_actuator_metadata(by_name: dict) -> bytes:
    """Build the QGC Actuators page without actuator-test capability."""
    channels = []
    per_channel_fields = ("FUNC", "MIN", "CENT", "MAX", "REV")
    for channel in range(1, 7):
        for suffix in per_channel_fields:
            if f"PWM_S{channel}_{suffix}" not in by_name:
                raise RuntimeError(
                    f"actuator Metadata requires PWM_S{channel}_{suffix}"
                )
        func_values = {
            entry["value"]: entry["description"]
            for entry in by_name[f"PWM_S{channel}_FUNC"]["values"]
        }
        advertised = {value: label for value, label in ACTUATOR_FUNCTIONS}
        if func_values != advertised:
            raise RuntimeError(
                f"PWM_S{channel}_FUNC enum {func_values} does not match "
                f"actuator functions_v1 {advertised}"
            )
        channels.append({"label": f"S{channel}", "param-index": channel})

    functions = {}
    for value, label in ACTUATOR_FUNCTIONS:
        entry = {"label": label}
        if value in ACTUATOR_FUNCTIONS_EXCLUDE_TEST:
            entry["exclude-from-actuator-testing"] = True
        functions[str(value)] = entry

    metadata = {
        "functions_v1": functions,
        "mixer_v1": {
            "actuator-types": {"DEFAULT": {"values": {"min": 0, "max": 1}}},
            "config": [],
        },
        "outputs_v1": [{
            "label": "PWM Output",
            "subgroups": [{
                "channels": channels,
                "per-channel-parameters": [
                    {
                        "label": "Function",
                        "name": "PWM_S${i}_FUNC",
                        "function": "function",
                    },
                    {
                        "label": "Minimum",
                        "name": "PWM_S${i}_MIN",
                        "function": "min",
                    },
                    {
                        "label": "Center\n(for Servos)",
                        "name": "PWM_S${i}_CENT",
                    },
                    {
                        "label": "Maximum",
                        "name": "PWM_S${i}_MAX",
                        "function": "max",
                    },
                    {
                        "label": "Reversed",
                        "name": "PWM_S${i}_REV",
                    },
                ],
            }],
        }],
        "version": 1,
    }
    validate_output_only_actuator_metadata(metadata)
    return json_bytes(metadata)


def main() -> int:
    args = parse_args()
    if args.verify:
        verify_generated_metadata(args.parameters, args.output)
        print("component Metadata stamp verification passed")
        return 0
    validate_metadata_contract_constants()
    source = json.loads(args.parameters.read_text(encoding="utf-8"))
    parameters = source.get("parameters")
    if not isinstance(parameters, list):
        raise RuntimeError("parameter catalogue has no parameters array")

    by_name = validate_parameter_catalogue(parameters)
    validate_sensor_metadata(by_name)

    parameter_json = json_bytes({
        "parameters": parameters,
        "version": 1,
    })
    parameter_xz = compress_xz(parameter_json)
    parameter_crc = mavlink_crc32(parameter_xz)

    actuator_json = build_actuator_metadata(by_name)
    actuator_xz = compress_xz(actuator_json)
    actuator_crc = mavlink_crc32(actuator_xz)

    general_json = json_bytes({
        "metadataTypes": [
            {
                "fileCrc": parameter_crc,
                "type": 1,
                "uri": PARAMETER_URI,
            },
            {
                "fileCrc": actuator_crc,
                "type": 5,
                "uri": ACTUATOR_URI,
            },
        ],
        "version": 1,
    })
    general_xz = compress_xz(general_json)
    if lzma.decompress(parameter_xz) != parameter_json:
        raise RuntimeError("parameter Metadata XZ round-trip failed")
    if lzma.decompress(actuator_xz) != actuator_json:
        raise RuntimeError("actuator Metadata XZ round-trip failed")
    if lzma.decompress(general_xz) != general_json:
        raise RuntimeError("General Metadata XZ round-trip failed")
    # PX4 v1.17 advertises the CRC of the uncompressed General JSON even
    # though the URI points at its .xz representation.
    general_crc = mavlink_crc32(general_json)

    args.output.mkdir(parents=True, exist_ok=True)
    outputs = {
        "actuators.json": actuator_json,
        "actuators.json.xz": actuator_xz,
        "component_general.json": general_json,
        "component_general.json.xz": general_xz,
        "parameters.json": parameter_json,
        "parameters.json.xz": parameter_xz,
        "parameter_metadata_files.hpp": generated_header(
            general_xz, parameter_xz, actuator_xz,
            general_crc, parameter_crc, actuator_crc
        ).encode("utf-8"),
    }
    for filename, data in outputs.items():
        (args.output / filename).write_bytes(data)

    stamp = {
        "actuator_crc32": actuator_crc,
        "format": 1,
        "general_crc32": general_crc,
        "input_sha256": sha256(args.parameters.read_bytes()),
        "outputs": {
            filename: {"sha256": sha256(data), "size": len(data)}
            for filename, data in sorted(outputs.items())
        },
        "parameter_crc32": parameter_crc,
        "public_parameter_count": len(parameters),
    }
    (args.output / ".generated.json").write_text(
        json.dumps(stamp, sort_keys=True, separators=(",", ":")),
        encoding="utf-8",
        newline="\n",
    )
    print(
        "generated parameter metadata: "
        f"{len(parameters)} params, "
        f"general={len(general_xz)} bytes, "
        f"parameters={len(parameter_xz)} bytes, "
        f"actuators={len(actuator_xz)} bytes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Generate the read-only PX4/QGC parameter Component Metadata payloads."""

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
INTERNAL_PARAMETERS = {"RC_PORT_CONFIG", "DIMA_SER_VER"}
PARAMETER_NAME = re.compile(r"^[.\-a-zA-Z0-9_{}]{1,16}$")
PARAMETER_TYPES = {"Int32", "Float"}
PARAMETER_FIELDS = {
    "name", "type", "shortDesc", "longDesc", "units", "default",
    "decimalPlaces", "min", "max", "increment", "rebootRequired",
    "group", "category", "volatile", "values", "bitmask",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parameters", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def json_bytes(value: object) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def compress_xz(data: bytes) -> bytes:
    return lzma.compress(
        data, format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC64, preset=9
    )


def mavlink_crc32(data: bytes) -> int:
    """PX4 component-information CRC32: initial zero, no final xor."""
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


def array_lines(data: bytes) -> list[str]:
    rows = []
    for offset in range(0, len(data), 12):
        chunk = data[offset:offset + 12]
        rows.append("    " + ", ".join(f"0x{value:02x}" for value in chunk) + ",")
    return rows


def generated_header(general_xz: bytes, parameter_xz: bytes,
                     general_crc: int, parameter_crc: int) -> str:
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
        f"inline constexpr std::uint32_t kGeneralCrc = 0x{general_crc:08x}U;",
        f"inline constexpr std::uint32_t kParameterCrc = 0x{parameter_crc:08x}U;",
        "",
        "alignas(4) inline constexpr std::uint8_t kGeneralFile[]{",
        *array_lines(general_xz),
        "};",
        "",
        "alignas(4) inline constexpr std::uint8_t kParameterFile[]{",
        *array_lines(parameter_xz),
        "};",
        "",
        "inline constexpr std::size_t kGeneralFileSize = sizeof(kGeneralFile);",
        "inline constexpr std::size_t kParameterFileSize = sizeof(kParameterFile);",
        "",
        "} // namespace dima::generated::parameter_metadata",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    source = json.loads(args.parameters.read_text(encoding="utf-8"))
    parameters = source.get("parameters")
    if not isinstance(parameters, list):
        raise RuntimeError("parameter catalogue has no parameters array")

    for index, parameter in enumerate(parameters):
        validate_parameter(parameter, index)
    names = [parameter.get("name") for parameter in parameters]
    if len(names) != len(set(names)):
        raise RuntimeError("parameter catalogue contains duplicate names")
    if len(parameters) != 205:
        raise RuntimeError(
            f"expected 205 firmware parameters, found {len(parameters)}"
        )

    public_parameters = [
        parameter for parameter in parameters
        if parameter["name"] not in INTERNAL_PARAMETERS
    ]
    if len(public_parameters) + len(INTERNAL_PARAMETERS) != len(parameters):
        raise RuntimeError("internal parameter filter differs from the catalogue")
    if len(public_parameters) != 203:
        raise RuntimeError("public parameter Metadata must contain 203 entries")

    by_name = {parameter["name"]: parameter for parameter in public_parameters}
    for serial in range(1, 9):
        for suffix, expected_values in (("BAUD", 16), ("FUNCTION", 2)):
            name = f"SERIAL{serial}_{suffix}"
            parameter = by_name.get(name)
            if (parameter is None or parameter.get("type") != "Int32" or
                    parameter.get("group") != "Serial" or
                    not parameter.get("shortDesc") or
                    parameter.get("rebootRequired") is not True or
                    len(parameter.get("values", [])) != expected_values):
                raise RuntimeError(
                    f"QGC serial Metadata contract invalid for {name}"
                )

    parameter_json = json_bytes({
        "parameters": public_parameters,
        "version": 1,
    })
    parameter_xz = compress_xz(parameter_json)
    parameter_crc = mavlink_crc32(parameter_xz)

    general_json = json_bytes({
        "metadataTypes": [{
            "fileCrc": parameter_crc,
            "type": 1,
            "uri": PARAMETER_URI,
        }],
        "version": 1,
    })
    general_xz = compress_xz(general_json)
    if lzma.decompress(parameter_xz) != parameter_json:
        raise RuntimeError("parameter Metadata XZ round-trip failed")
    if lzma.decompress(general_xz) != general_json:
        raise RuntimeError("General Metadata XZ round-trip failed")
    # PX4 v1.17 advertises the CRC of the uncompressed General JSON even
    # though the URI points at its .xz representation.
    general_crc = mavlink_crc32(general_json)

    args.output.mkdir(parents=True, exist_ok=True)
    outputs = {
        "component_general.json": general_json,
        "component_general.json.xz": general_xz,
        "parameters.json": parameter_json,
        "parameters.json.xz": parameter_xz,
        "parameter_metadata_files.hpp": generated_header(
            general_xz, parameter_xz, general_crc, parameter_crc
        ).encode("utf-8"),
    }
    for filename, data in outputs.items():
        (args.output / filename).write_bytes(data)

    stamp = {
        "format": 1,
        "general_crc32": general_crc,
        "input_sha256": sha256(args.parameters.read_bytes()),
        "outputs": {
            filename: {"sha256": sha256(data), "size": len(data)}
            for filename, data in sorted(outputs.items())
        },
        "parameter_crc32": parameter_crc,
        "public_parameter_count": len(public_parameters),
    }
    (args.output / ".generated.json").write_text(
        json.dumps(stamp, sort_keys=True, separators=(",", ":")),
        encoding="utf-8",
        newline="\n",
    )
    print(
        "generated parameter metadata: "
        f"{len(public_parameters)} params, "
        f"general={len(general_xz)} bytes, parameters={len(parameter_xz)} bytes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

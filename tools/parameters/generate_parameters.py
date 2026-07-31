#!/usr/bin/env python3
"""Run the imported PX4 parser and generate the Dima parameter catalogue."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from generate_header import generate

BSD_HEADER = """/****************************************************************************
 * Generated from PX4 parameter definitions. DO NOT EDIT.
 ****************************************************************************/
"""


def json_names(path: Path) -> list[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    items = data.get("parameters", [])
    if not isinstance(items, list):
        raise RuntimeError("official JSON has no parameters list")
    items.sort(key=lambda item: item["name"])
    data["parameters"] = items
    path.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return [item["name"] for item in items]


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
        + '#include "Dima/middleware/parameters/param.h"\n\n'
        + f"#define PARAM_INFO_COUNT {len(parameters)}\n\n"
        + "const param_info_s param_info[PARAM_INFO_COUNT] = {\n"
        + ",\n".join(rows)
        + "\n};\n\n"
        + "const uint16_t param_info_count = PARAM_INFO_COUNT;\n"
    )
    path.write_text(text, encoding="utf-8")


def write_forward_header(path: Path, include_target: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f'#pragma once\n#include "{include_target}"\n', encoding="utf-8")


def write_include_forwarders(include_output: Path, generated_header: Path) -> None:
    write_forward_header(include_output / "px4_platform_common" / "param.h", "Dima/middleware/parameters/param.h")
    write_forward_header(include_output / "px4_platform_common" / "param_macros.h", "Dima/middleware/parameters/param_macros.h")
    write_forward_header(include_output / "px4_platform_common" / "module_params.h", "Dima/middleware/parameters/module_params.h")
    # DrvFS 上的转发 include 会把 Windows 盘符写入 GCC .d 文件并破坏 GNU Make 解析。
    parameter_header = include_output / "parameters" / "px4_parameters.hpp"
    parameter_header.parent.mkdir(parents=True, exist_ok=True)
    parameter_header.write_bytes(generated_header.read_bytes())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", action="append", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=Path("build/generated/parameters"))
    parser.add_argument("--include-output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    process = Path(__file__).with_name("process_parameters.py")
    source_dirs = sorted({str(path.parent) for path in args.source})
    xml_path = args.output / "parameters.xml"
    json_path = args.output / "parameters.json"
    subprocess.run([
        sys.executable, str(process), "--src-path", *source_dirs,
        "--xml", str(xml_path), "--json", str(json_path), "--board", "dima_rover",
    ], check=True)

    parameters = generate(xml_path, args.output)
    xml_names = [parameter.attrib["name"] for parameter in parameters]
    generated_json_names = json_names(json_path)
    if not parameters or xml_names != generated_json_names:
        raise RuntimeError(
            f"catalog mismatch: xml/header={len(parameters)} json={len(generated_json_names)}"
        )

    write_metadata(args.output / "parameter_metadata.c", parameters)
    write_include_forwarders(args.include_output, args.output / "px4_parameters.hpp")
    print(f"generated {len(parameters)} parameters in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


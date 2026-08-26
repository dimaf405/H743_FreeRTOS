"""生成参数目录、传感器 consumer 闭包与 QGC 可见性/写入策略门禁。"""

from __future__ import annotations

import json
import pathlib
import re

from architecture.build_closure import BuildClosureError, load_build_closure
from architecture.common import (
    ROOT,
    Violation,
    strip_c_comments,
    sources_under,
    line_for,
    require_literals,
)

from architecture.mavlink_protocol import scan_mavlink_protocol_contract


def _generated_serial_parameter_schema(
        violations: list[Violation]) -> tuple[tuple[str, str, str], ...]:
    """直接从板级 serial manifest 推导参数名和默认值，禁止在门禁中复制第二份清单。"""
    path = ROOT / "Boards/H743/serial_ports.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R331",
            f"cannot derive generated serial parameter schema: {error}",
        ))
        return ()

    schema: list[tuple[str, str, str]] = []
    valid = True
    ports = manifest.get("ports")
    if not isinstance(ports, list):
        ports = []
        valid = False
    for port in ports:
        if not isinstance(port, dict):
            valid = False
            continue
        serial = port.get("serial")
        if serial == 0:
            continue
        baud_name = port.get("parameter")
        function_name = port.get("function_parameter")
        default_baud = port.get("default_baud")
        default_function = port.get("default_function")
        if (not isinstance(serial, int) or
                not isinstance(baud_name, str) or
                not isinstance(function_name, str) or
                type(default_baud) is not int or
                type(default_function) is not int):
            valid = False
            continue
        schema.extend((
            (baud_name, "INT32", str(default_baud)),
            (function_name, "INT32", str(default_function)),
        ))

    gps_parameter = manifest.get("gps_port_parameter")
    if (not isinstance(gps_parameter, dict) or
            not isinstance(gps_parameter.get("name"), str) or
            type(gps_parameter.get("default")) is not int):
        valid = False
    else:
        schema.append((
            gps_parameter["name"], "INT32", str(gps_parameter["default"]),
        ))

    if not valid or len(schema) != 17:
        violations.append(Violation(
            path, 1, "R331",
            "serial manifest must generate 16 SERIALx parameters and one "
            "primary GPS port parameter",
        ))
    return tuple(schema)


def scan_mavlink_contract(violations: list[Violation]) -> None:
    """R330-R336：验证实际交付的裁剪 MAVLink/参数表面及其生成闭包。"""
    forbidden_paths = (
        ROOT / "Dima/lib/mavlink/c_library_v2",
        ROOT / "tools/generate_actuators_metadata.py",
    )
    for path in forbidden_paths:
        if path.exists():
            violations.append(Violation(
                path, 1, "R330",
                "generated source-tree MAVLink or Actuator Metadata code "
                "must remain absent",
            ))
    mavlink_parameters_path, mavlink_parameters_text = (
        _scan_parameter_catalog_contract(violations)
    )
    scan_mavlink_protocol_contract(
        violations, mavlink_parameters_path, mavlink_parameters_text
    )


def _scan_parameter_catalog_contract(
        violations: list[Violation]) -> tuple[pathlib.Path, str]:
    """核对参数定义输入、生成 public/QGC/fixed 集合和运行期消费者完全同源。"""
    forbidden_parameters = {
        "COM_DL_LOSS_T", "COM_ARM_SWISBTN", "COM_RC_ARM_HYST",
        "MAN_ARM_GESTURE", "RTL_RETURN_ALT", "RTL_DESCEND_ALT",
        "RTL_LAND_DELAY", "COM_FLTMODE1", "COM_FLTMODE2",
        "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5",
        "COM_FLTMODE6", "DIMA_SER_VER", "RC_PORT_CONFIG",
    }
    _generated_serial_parameter_schema(violations)
    parameter_definition_re = re.compile(
        r"\bPARAM_DEFINE_(INT32|FLOAT)\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([^\s,)]+)\s*\)\s*;"
    )
    raw_parameter_definition_re = re.compile(
        r"\b(?:PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\("
    )
    documented_parameter_re = re.compile(
        r"/\*\*(?P<documentation>.*?)\*/\s*"
        r"PARAM_DEFINE_(?:INT32|FLOAT)\s*\(\s*"
        r"(?P<name>[A-Z][A-Z0-9_]*)\s*,",
        re.DOTALL,
    )
    qgc_required_tag_re = re.compile(
        r"^\s*\*\s*@qgc_required\s*$", re.MULTILINE
    )
    try:
        definition_inputs = sorted(
            load_build_closure(ROOT).parameter_generator_inputs
        )
    except BuildClosureError as error:
        violations.append(Violation(
            ROOT / "make/project.mk", 1, "R331",
            f"cannot evaluate generated parameter input closure: {error}",
        ))
        definition_inputs = []
    parameter_paths = [ROOT / relative for relative in definition_inputs]
    parameter_definition_count = 0
    raw_parameter_definition_count = 0
    parameter_name_locations: dict[str, list[tuple[pathlib.Path, int]]] = {}
    qgc_required_locations: dict[str, list[tuple[pathlib.Path, int]]] = {}
    for path in parameter_paths:
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R331",
                "listed parameter definition source does not exist",
            ))
            continue
        text = path.read_text(encoding="utf-8")
        for documented in documented_parameter_re.finditer(text):
            if qgc_required_tag_re.search(documented.group("documentation")):
                qgc_required_locations.setdefault(
                    documented.group("name"), []
                ).append((path, line_for(text, documented.group(0))))
        code = strip_c_comments(text)
        raw_parameter_definition_count += len(
            raw_parameter_definition_re.findall(code)
        )
        definitions = tuple(parameter_definition_re.finditer(code))
        parameter_definition_count += len(definitions)
        for definition in definitions:
            parameter = definition.group(2)
            parameter_name_locations.setdefault(parameter, []).append(
                (path, line_for(text, definition.group(0)))
            )
            if parameter in forbidden_parameters:
                violations.append(Violation(
                    path, line_for(text, definition.group(0)), "R331",
                    f"unimplemented parameter '{parameter}' entered the build",
                ))
    listed_parameter_paths = set(parameter_paths)
    for path in sources_under(("Dima/middleware/parameters/definitions",)):
        if path in listed_parameter_paths:
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        if raw_parameter_definition_re.search(code):
            violations.append(Violation(
                path, 1, "R331",
                "parameter definition exists outside PARAMETER_DEFINITIONS",
            ))
    if raw_parameter_definition_count != 233:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "generated parameter source contract must contain exactly 233 "
            f"definitions (found {raw_parameter_definition_count})",
        ))
    if parameter_definition_count != raw_parameter_definition_count:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "every parameter definition must use a parseable literal type, "
            "name, and default "
            f"(parsed={parameter_definition_count}, "
            f"raw={raw_parameter_definition_count})",
        ))
    for name, locations in parameter_name_locations.items():
        if len(locations) > 1:
            path, line = locations[1]
            violations.append(Violation(
                path, line, "R331",
                f"parameter '{name}' has duplicate source definitions",
            ))

    duplicate_qgc_required = sorted(
        name for name, locations in qgc_required_locations.items()
        if len(locations) != 1
    )
    if not qgc_required_locations or duplicate_qgc_required:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "QGC setup Facts must be tagged in their authoritative parameter "
            "definitions and generated as one contract "
            f"(count={len(qgc_required_locations)}, "
            f"duplicates={duplicate_qgc_required})",
        ))
    for name, locations in qgc_required_locations.items():
        if name not in parameter_name_locations:
            path, line = locations[0]
            violations.append(Violation(
                path, line, "R331",
                f"QGC-required parameter '{name}' has no catalogue entry",
            ))

    qgc_compat_path = (
        ROOT / "Dima/middleware/parameters/definitions/qgc_compat_params.c"
    )
    qgc_compat_definitions = []
    qgc_compat_raw_definition_count = 0
    if qgc_compat_path.exists():
        qgc_compat_text = qgc_compat_path.read_text(encoding="utf-8")
        qgc_compat_code = strip_c_comments(qgc_compat_text)
        qgc_compat_raw_definition_count = len(
            raw_parameter_definition_re.findall(qgc_compat_code)
        )
        qgc_compat_definitions = [
            (match.group(2), match.group(1), match.group(3))
            for match in parameter_definition_re.finditer(qgc_compat_code)
        ]
    if (qgc_compat_raw_definition_count == 0 or
            len(qgc_compat_definitions) != qgc_compat_raw_definition_count):
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "qgc_compat_params.c must contain only parseable literal "
            "parameter definitions; fixedness is generated from min/max "
            f"(raw={qgc_compat_raw_definition_count}, "
            f"parsed={len(qgc_compat_definitions)})",
        ))
    qgc_parameter_names = tuple(
        definition[0] for definition in qgc_compat_definitions
    )
    require_literals(
        ROOT / "tools/parameters/generate_parameters.py",
        (
            ("catalog = ordered_json_parameters(json_path, xml_names)",
             "R331", "JSON catalogue must use the generated handle order"),
            ("xml_names != generated_json_names", "R331",
             "JSON and firmware parameter orders must remain identical"),
            ('"--src-file"', "R331",
             "parameter parser must receive the explicit source file set"),
            ("parameters = generate(xml_path, args.output)", "R331",
             "parameter generator must use the current catalogue order"),
            ("write_parameter_contract(contract_path", "R331",
             "MAVLink/QGC parameter contracts must be generated"),
            ("if not qgc_required:", "R331",
             "QGC-required Fact annotations must not generate an empty set"),
            ("compatibility_names.issubset(fixed_names)", "R331",
             "fixed QGC compatibility values must come from generated metadata"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/parameters/process_parameters.py",
        (
            ('"--src-file"', "R331",
             "official parameter parser must accept explicit source files"),
            ("scanner.ScanFile(source_file, parser)", "R331",
             "only listed parameter source files may be scanned"),
        ),
        violations,
    )

    mavlink_parameters_path = (
        ROOT / "Dima/modules/mavlink/MavlinkParameters.cpp"
    )
    mavlink_parameters_text = mavlink_parameters_path.read_text(
        encoding="utf-8"
    )
    qgc_registry_path = (
        ROOT / "Dima/middleware/parameters/QgcCompatibility.hpp"
    )
    if qgc_registry_path.exists():
        violations.append(Violation(
            qgc_registry_path, 1, "R331",
            "hand-maintained QGC parameter registries are forbidden; use the "
            "generated parameter_contract.hpp",
        ))
    require_literals(
        mavlink_parameters_path,
        (
            ("prepare_parameter_catalogue()", "R331",
             "MAVLink must activate the generated public catalogue"),
            ("contract::kMavlinkPublicParameters", "R331",
             "MAVLink public parameters must come from generated handles"),
            ("contract::kQgcRequiredParameters", "R331",
             "QGC setup Facts must be verified from generated handles"),
            ("contract::kFixedParameterConstraints", "R331",
             "fixed parameter writes must use generated constraints"),
        ),
        violations,
    )

    snapshot_codec_path = (
        ROOT / "Dima/modules/parameters/ParameterSnapshotCodec.cpp"
    )
    require_literals(
        snapshot_codec_path,
        (
            ("is_fixed_parameter(name)", "R331",
             "snapshot load must identify generated fixed parameters"),
            ("kFixedParameterConstraints", "R331",
             "snapshot filter must use generated fixed constraints"),
            ("load_mutable_parameter, &filtered", "R331",
              "snapshot decode must filter fixed values before the param layer"),
        ),
        violations,
    )

    common_qgc_owners = {qgc_compat_path}
    expected_qgc_consumers = {
        name: set() for name in qgc_parameter_names
    }
    expected_qgc_consumers.update({
        "MAV_SYS_ID": {ROOT / "Dima/modules/mavlink/MavlinkService.cpp"},
        "NAV_RCL_ACT": {ROOT / "Dima/modules/safety/CommanderSafety.cpp"},
        "NAV_DLL_ACT": {ROOT / "Dima/modules/safety/CommanderSafety.cpp"},
    })
    actual_qgc_consumers = {
        name: set() for name in qgc_parameter_names
    }
    for path in sources_under(("Dima",)):
        if path in common_qgc_owners:
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        for parameter in qgc_parameter_names:
            if re.search(rf"\b{re.escape(parameter)}\b", code):
                actual_qgc_consumers[parameter].add(path)
    for parameter in qgc_parameter_names:
        actual = actual_qgc_consumers[parameter]
        expected = expected_qgc_consumers[parameter]
        if actual != expected:
            unexpected = actual - expected
            location = next(iter(unexpected or expected), qgc_compat_path)
            violations.append(Violation(
                location, 1, "R331",
                f"QGC compatibility parameter '{parameter}' consumer set "
                "differs from the exact whitelist "
                f"(expected={sorted(path.relative_to(ROOT).as_posix() for path in expected)}, "
                f"actual={sorted(path.relative_to(ROOT).as_posix() for path in actual)})",
            ))

    return mavlink_parameters_path, mavlink_parameters_text

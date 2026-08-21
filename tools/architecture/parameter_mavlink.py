"""Parameter catalogue and QGC policy checks."""

from __future__ import annotations

import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    strip_c_comments,
    sources_under,
    line_for,
    require_literals,
)

from architecture.mavlink_protocol import scan_mavlink_protocol_contract


def scan_mavlink_contract(violations: list[Violation]) -> None:
    """R330-R336: enforce the shipped, deliberately trimmed MAVLink surface."""
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
    forbidden_parameters = {
        "COM_DL_LOSS_T", "COM_ARM_SWISBTN", "COM_RC_ARM_HYST",
        "MAN_ARM_GESTURE", "RTL_RETURN_ALT", "RTL_DESCEND_ALT",
        "RTL_LAND_DELAY", "COM_FLTMODE1", "COM_FLTMODE2",
        "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5",
        "COM_FLTMODE6", "DIMA_SER_VER", "RC_PORT_CONFIG",
    }
    qgc_fixed_schema = (
        ("SYS_AUTOSTART", "INT32", "50000"),
        ("SYS_AUTOCONFIG", "INT32", "0"),
        ("MAV_SYS_ID", "INT32", "1"),
        ("CAL_GYRO0_ID", "INT32", "0"),
        ("CAL_ACC0_ID", "INT32", "0"),
        ("CAL_MAG0_ID", "INT32", "0"),
        ("CAL_MAG1_ID", "INT32", "0"),
        ("CAL_MAG2_ID", "INT32", "0"),
        ("NAV_RCL_ACT", "INT32", "6"),
        ("NAV_DLL_ACT", "INT32", "0"),
        ("COM_LOW_BAT_ACT", "INT32", "0"),
    )
    serial_parameter_schema = (
        ("SERIAL1_BAUD", "INT32", "921600"),
        ("SERIAL1_FUNCTION", "INT32", "0"),
        ("SERIAL2_BAUD", "INT32", "0"),
        ("SERIAL2_FUNCTION", "INT32", "0"),
        ("SERIAL3_BAUD", "INT32", "0"),
        ("SERIAL3_FUNCTION", "INT32", "0"),
        ("SERIAL4_BAUD", "INT32", "115200"),
        ("SERIAL4_FUNCTION", "INT32", "0"),
        ("SERIAL5_BAUD", "INT32", "115200"),
        ("SERIAL5_FUNCTION", "INT32", "0"),
        ("SERIAL6_BAUD", "INT32", "0"),
        ("SERIAL6_FUNCTION", "INT32", "1"),
        ("SERIAL7_BAUD", "INT32", "57600"),
        ("SERIAL7_FUNCTION", "INT32", "0"),
        ("SERIAL8_BAUD", "INT32", "115200"),
        ("SERIAL8_FUNCTION", "INT32", "0"),
    )
    expected_qgc_schema = set(qgc_fixed_schema)
    qgc_parameter_names = tuple(entry[0] for entry in qgc_fixed_schema)
    parameter_definition_re = re.compile(
        r"\bPARAM_DEFINE_(INT32|FLOAT)\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([^\s,)]+)\s*\)\s*;"
    )
    raw_parameter_definition_re = re.compile(
        r"\b(?:PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\("
    )
    project_mk_path = ROOT / "make/project.mk"
    project_mk_text = project_mk_path.read_text(encoding="utf-8")
    definition_block = re.search(
        r"^PARAMETER_DEFINITIONS\s*:=\s*\\\s*\n"
        r"(?P<body>(?:^[^\n]*\\\s*\n)*^[^\n]*)",
        project_mk_text,
        re.MULTILINE,
    )
    definition_inputs = [] if definition_block is None else re.findall(
        r"Dima/middleware/parameters/definitions/[^\\\s]+\.[ch]",
        definition_block.group("body"),
    )
    parameter_paths = [ROOT / relative for relative in definition_inputs]
    parameter_definition_count = 0
    raw_parameter_definition_count = 0
    parameter_name_locations: dict[str, list[tuple[pathlib.Path, int]]] = {}
    for path in parameter_paths:
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R331",
                "listed parameter definition source does not exist",
            ))
            continue
        text = path.read_text(encoding="utf-8")
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
    serial_manifest_path = ROOT / "Boards/H743/serial_ports.json"
    for name, parameter_type, value in serial_parameter_schema:
        parameter_definition_count += 1
        raw_parameter_definition_count += 1
        parameter_name_locations.setdefault(name, []).append(
            (serial_manifest_path, 1)
        )
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
    if raw_parameter_definition_count != 203:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "generated parameter source contract must contain exactly 203 "
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
    if (qgc_compat_raw_definition_count != 11 or
            len(qgc_compat_definitions) != 11):
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "qgc_compat_params.c must contain exactly 11 fixed parameter "
            "definitions with parseable literal values "
            f"(raw={qgc_compat_raw_definition_count}, "
            f"parsed={len(qgc_compat_definitions)})",
        ))
    actual_qgc_definitions = qgc_compat_definitions
    actual_qgc_schema = set(actual_qgc_definitions)
    if actual_qgc_schema != expected_qgc_schema:
        missing = sorted(expected_qgc_schema - actual_qgc_schema)
        extra = sorted(actual_qgc_schema - expected_qgc_schema)
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "QGC compatibility schema differs from the exact fixed contract "
            f"(missing={missing}, extra={extra})",
        ))
    require_literals(
        ROOT / "tools/parameters/generate_parameters.py",
        (
            ("generated_json_names = json_names(json_path, xml_names)",
             "R331", "JSON catalogue must use the generated handle order"),
            ("xml_names != generated_json_names", "R331",
             "JSON and firmware parameter orders must remain identical"),
            ('"--src-file"', "R331",
             "parameter parser must receive the explicit source file set"),
            ("EXPECTED_PARAMETER_COUNT = 203", "R331",
              "generator must fail closed on a parameter count change"),
            ("parameters = generate(xml_path, args.output)", "R331",
             "parameter generator must use the current catalogue order"),
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
    qgc_registry_text = qgc_registry_path.read_text(encoding="utf-8")
    registry_entry_re = re.compile(
        r'\{\s*"([A-Z][A-Z0-9_]*)"\s*,\s*'
        r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[fF])?)\s*\}'
    )

    def registry_entries(
            array_name: str,
            parameter_type: str) -> list[tuple[str, str, str]]:
        array = re.search(
            rf"{re.escape(array_name)}\[\]\s*\{{(?P<body>.*?)\}};",
            qgc_registry_text,
            re.DOTALL,
        )
        if array is None:
            violations.append(Violation(
                qgc_registry_path, 1, "R331",
                f"QGC fixed-value registry '{array_name}' is missing",
            ))
            return []
        body = array.group("body")
        residue = registry_entry_re.sub("", body)
        if re.sub(r"[\s,]", "", residue):
            violations.append(Violation(
                qgc_registry_path,
                line_for(qgc_registry_text, array.group(0)),
                "R331",
                f"QGC fixed-value registry '{array_name}' contains an "
                "unparseable or non-literal entry",
            ))
        return [
            (match.group(1), parameter_type, match.group(2))
            for match in registry_entry_re.finditer(body)
        ]

    actual_qgc_registry = registry_entries(
        "kQgcFixedInt32Parameters", "INT32"
    )
    expected_qgc_registry = {
        (name, parameter_type,
         value.upper() if parameter_type == "FLOAT" else value)
        for name, parameter_type, value in qgc_fixed_schema
    }
    registry_names = [entry[0] for entry in actual_qgc_registry]
    duplicate_registry_names = sorted({
        name for name in registry_names if registry_names.count(name) > 1
    })
    if (len(actual_qgc_registry) != 11 or duplicate_registry_names or
            set(actual_qgc_registry) != expected_qgc_registry):
        missing = sorted(expected_qgc_registry - set(actual_qgc_registry))
        extra = sorted(set(actual_qgc_registry) - expected_qgc_registry)
        violations.append(Violation(
            qgc_registry_path, 1, "R331",
            "QGC fixed-value registry differs from the exact schema "
            f"(duplicates={duplicate_registry_names}, missing={missing}, "
            f"extra={extra})",
        ))
    require_literals(
        mavlink_parameters_path,
        (("dima::parameters::qgc_fixed_int32_parameter(name)", "R331",
          "MAVLink fixed values bypass the shared QGC registry"),),
        violations,
    )

    parameter_service_path = (
        ROOT / "Dima/modules/parameters/ParameterService.cpp"
    )
    require_literals(
        parameter_service_path,
        (
            ("is_qgc_compatibility_parameter(name)", "R331",
             "snapshot load must identify fixed QGC compatibility values"),
            ("qgc_fixed_int32_parameter(name)", "R331",
             "Journal filter must use the shared QGC registry"),
            ("is_disabled_mode_compatibility_parameter(name)", "R336",
             "disabled RC mode mapping must ignore legacy stored values"),
            ("load_mutable_parameter, &filtered", "R331",
              "snapshot decode must filter fixed values before the param layer"),
        ),
        violations,
    )

    common_qgc_owners = {
        qgc_compat_path,
        qgc_registry_path,
    }
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

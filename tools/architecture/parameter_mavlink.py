"""Parameter catalogue, stable handle, and QGC policy checks."""

from __future__ import annotations

import hashlib
import pathlib
import re

from architecture.common import (
    ROOT,
    Violation,
    strip_c_comments,
    sources_under,
    line_for,
    require_literals,
    owner_texts,
    require_literals_in_owners,
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
        "COM_FLTMODE6",
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
        ("DIMA_SER_VER", "INT32", "0"),
    )
    serial_config_schema = ("RC_PORT_CONFIG", "INT32", "6")
    expected_qgc_schema = set(qgc_fixed_schema)
    qgc_parameter_names = tuple(entry[0] for entry in qgc_fixed_schema)
    expected_qgc_names = set(qgc_parameter_names)
    serial_parameter_names = tuple(
        entry[0] for entry in serial_parameter_schema
    )
    expected_serial_names = set(serial_parameter_names)
    stable_tail_names = qgc_parameter_names + serial_parameter_names
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
    for name, parameter_type, value in (
            serial_config_schema, *serial_parameter_schema):
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
    if raw_parameter_definition_count != 205:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "generated parameter source contract must contain exactly 205 "
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
    actual_qgc_order = tuple(
        name for name, _parameter_type, _value in actual_qgc_definitions
    )
    if actual_qgc_order != qgc_parameter_names:
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "QGC compatibility definitions must retain their stable append "
            f"order (expected={qgc_parameter_names}, "
            f"actual={actual_qgc_order})",
        ))
    actual_stable_tail_order = tuple(
        name for name, _parameter_type, _value in
        (*qgc_compat_definitions, *serial_parameter_schema)
    )
    if actual_stable_tail_order != stable_tail_names:
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "PX4/QGC stable tail order changed "
            f"(expected={stable_tail_names}, actual={actual_stable_tail_order})",
        ))

    require_literals(
        ROOT / "tools/parameters/generate_parameters.py",
        (
            ("for source in args.stable_tail_source", "R331",
             "ordered compatibility sources must define the stable tail"),
            ("parameters = generate(xml_path, args.output, stable_tail_names)",
             "R331", "parameter generator must apply the stable tail"),
            ("generated_json_names = json_names(json_path, xml_names)",
             "R331", "JSON catalogue must use the generated handle order"),
            ("xml_names != generated_json_names", "R331",
             "JSON and firmware parameter orders must remain identical"),
            ('"--src-file"', "R331",
             "parameter parser must receive the explicit source file set"),
            ("EXPECTED_PARAMETER_COUNT = 205", "R331",
             "generator must fail closed on a parameter count change"),
            ("EXPECTED_STABLE_TAIL_COUNT = 28", "R331",
             "generator must fail closed on a compatibility tail change"),
            ('stable_tail_names[0] != "SYS_AUTOSTART"', "R331",
             "SYS_AUTOSTART must preserve prior handle 177"),
            ('stable_tail_names[-1] != "DIMA_SER_VER"', "R331",
             "serial migration version must terminate the stable tail"),
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
    require_literals(
        project_mk_path,
        (
            ("--stable-tail-source Dima/middleware/parameters/definitions/"
             "qgc_compat_params.c", "R331",
             "QGC compatibility source must start the explicit stable tail"),
            ("--stable-tail-source $(SERIAL_BAUD_PARAMETERS)", "R331",
             "generated board serial parameters must finish the stable tail"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/parameters/generate_header.py",
        (
            ("def order_with_stable_tail", "R331",
             "generated handles must support an append-only stable tail"),
            ("+ [by_name[name] for name in tail_names]", "R331",
             "compatibility handles must be appended in source order"),
        ),
        violations,
    )

    all_parameter_names = set(parameter_name_locations)
    prior_parameter_names = sorted(
        all_parameter_names -
        ((expected_qgc_names - {"SYS_AUTOSTART"}) | expected_serial_names)
    )
    prior_parameter_digest = hashlib.sha256(
        ("\n".join(prior_parameter_names) + "\n").encode("ascii")
    ).hexdigest()
    expected_generated_order = (
        sorted(all_parameter_names - expected_qgc_names - expected_serial_names)
        + list(stable_tail_names)
    )
    if (len(prior_parameter_names) != 178 or
            prior_parameter_digest !=
            "10477cbfe796ac845b63069aa0676326a8183a3f27ee02f0f184ce019f9a2449" or
            expected_generated_order[:178] != prior_parameter_names):
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "stable-tail layout must preserve all 178 prior parameter "
            "handles before appending the 28-entry QGC/board-serial tail",
        ))

    mavlink_parameters_path = (
        ROOT / "Dima/modules/mavlink/MavlinkParameters.cpp"
    )
    mavlink_parameter_owners = (
        mavlink_parameters_path,
        ROOT / "Dima/modules/mavlink/MavlinkParameterExt.cpp",
    )
    mavlink_parameters_text = mavlink_parameters_path.read_text(
        encoding="utf-8"
    )
    internal_filter_count = sum(
        text.count("is_internal_parameter(name)")
        for _path, text in owner_texts(mavlink_parameter_owners)
    )
    if internal_filter_count != 3:
        violations.append(Violation(
            mavlink_parameters_path, 1, "R336",
            "internal parameters must be filtered in SET, Classic READ, and Ext READ",
        ))
    registry_entry_re = re.compile(
        r'\{\s*"([A-Z][A-Z0-9_]*)"\s*,\s*'
        r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[fF])?)\s*\}'
    )

    def registry_entries(
            array_name: str,
            parameter_type: str) -> list[tuple[str, str, str]]:
        array = re.search(
            rf"{re.escape(array_name)}\[\]\s*\{{(?P<body>.*?)\}};",
            mavlink_parameters_text,
            re.DOTALL,
        )
        if array is None:
            violations.append(Violation(
                mavlink_parameters_path, 1, "R331",
                f"QGC fixed-value registry '{array_name}' is missing",
            ))
            return []
        body = array.group("body")
        residue = registry_entry_re.sub("", body)
        if re.sub(r"[\s,]", "", residue):
            violations.append(Violation(
                mavlink_parameters_path,
                line_for(mavlink_parameters_text, array.group(0)),
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
            mavlink_parameters_path, 1, "R331",
            "QGC fixed-value registry differs from the exact schema "
            f"(duplicates={duplicate_registry_names}, missing={missing}, "
            f"extra={extra})",
        ))

    parameter_service_path = (
        ROOT / "Dima/modules/parameters/ParameterService.cpp"
    )
    parameter_service_owners = (
        parameter_service_path,
        ROOT / "Dima/modules/parameters/SerialParameterMigration.cpp",
        ROOT / "Dima/modules/parameters/SerialMigrationSchema.hpp",
    )
    parameter_service_text = parameter_service_path.read_text(encoding="utf-8")
    ignored_array = re.search(
        r"kQgcJournalIgnoredParameters\[\]\s*\{(?P<body>.*?)\};",
        parameter_service_text,
        re.DOTALL,
    )
    ignored_name_re = re.compile(r'"([A-Z][A-Z0-9_]*)"')
    ignored_names = [] if ignored_array is None else [
        match.group(1)
        for match in ignored_name_re.finditer(ignored_array.group("body"))
    ]
    if ignored_array is not None:
        ignored_residue = ignored_name_re.sub("", ignored_array.group("body"))
        if re.sub(r"[\s,]", "", ignored_residue):
            violations.append(Violation(
                parameter_service_path,
                line_for(parameter_service_text, ignored_array.group(0)),
                "R331",
                "Journal fixed-parameter filter contains an unparseable or "
                "non-literal entry",
            ))
    duplicate_ignored_names = sorted({
        name for name in ignored_names if ignored_names.count(name) > 1
    })
    if (len(ignored_names) != 11 or duplicate_ignored_names or
            set(ignored_names) != expected_qgc_names):
        violations.append(Violation(
            parameter_service_path, 1, "R331",
            "Journal fixed-parameter filter must match the exact QGC schema "
            f"(duplicates={duplicate_ignored_names}, "
            f"missing={sorted(expected_qgc_names - set(ignored_names))}, "
            f"extra={sorted(set(ignored_names) - expected_qgc_names)})",
        ))
    require_literals_in_owners(
        parameter_service_owners,
        (
            ("is_qgc_compatibility_parameter(name)", "R331",
             "Journal load must identify fixed QGC compatibility values"),
            ("is_disabled_mode_compatibility_parameter(name)", "R336",
             "disabled RC mode mapping must ignore legacy stored values"),
            ("load_mutable_parameter, &filtered", "R331",
             "Journal decode must filter fixed values before the param layer"),
            ("scan_serial_storage, &scan", "R331",
             "Journal load must detect the serial schema before migration"),
            ("migrate_serial_schema_v1()", "R331",
             "serial schema v1 must migrate before direct numbering is used"),
            ("kSchema1ToDirectSerial", "R331",
             "schema v1 migration must preserve physical UART ownership"),
            ("unsupported serial schema=", "R331",
             "invalid or future serial schemas must fail closed"),
            ("migrate_serial_configuration(loaded == 0)", "R331",
             "legacy RC port values must migrate to SERIALx_FUNCTION"),
            ("migrate_legacy_rc_port", "R331",
             "serial migration must use the generated board mapping"),
            ("legacy_serial_for_baud_parameter", "R331",
             "legacy baud values must migrate by physical serial port"),
        ),
        violations,
    )

    common_qgc_owners = {
        qgc_compat_path,
        mavlink_parameters_path,
        parameter_service_path,
    }
    expected_qgc_consumers = {
        name: set() for name in qgc_parameter_names
    }
    expected_qgc_consumers.update({
        "MAV_SYS_ID": {ROOT / "Dima/modules/mavlink/MavlinkService.cpp"},
        "NAV_RCL_ACT": {ROOT / "Dima/modules/safety/CommanderSafety.cpp"},
        "NAV_DLL_ACT": {ROOT / "Dima/modules/safety/CommanderSafety.cpp"},
        **{
            f"COM_FLTMODE{slot}": {
                ROOT / "Dima/modules/rc/RcManualInput.cpp"
            }
            for slot in range(1, 7)
        },
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

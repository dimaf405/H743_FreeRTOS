"""板级串口 manifest、生成参数/资源与 RC/GPS 唯一所有权门禁。"""

from __future__ import annotations

import importlib.util
import json
import re

from architecture.common import (
    ROOT,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
    strip_c_comments,
)


def _scan_generated_serial_contract(
        violations: list[Violation], manifest: dict,
        ports: list[dict]) -> None:
    """R342：内存中重新生成合同并逐字节比对产物，拒绝手改派生文件。"""
    generator_path = ROOT / "tools/serial/generate_config.py"
    spec = importlib.util.spec_from_file_location(
        "dima_serial_contract_generator", generator_path
    )
    if spec is None or spec.loader is None:
        violations.append(Violation(
            generator_path, 1, "R342",
            "serial generator cannot be loaded for contract verification",
        ))
        return
    try:
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        external_ports = [
            port for port in ports
            if isinstance(port, dict) and port.get("serial") != 0
        ]
        parameter_output = generator.generate_serial_parameters(
            manifest, external_ports
        )
        contract_output = generator.generate_serial_contract(
            manifest, external_ports
        )
    except Exception as error:  # The gate must report, not crash, on drift.
        violations.append(Violation(
            generator_path, 1, "R342",
            f"serial generator contract verification failed: {error}",
        ))
        return

    gps_parameter = manifest.get("gps_port_parameter", {})
    expected_parameters = [
        ("INT32", gps_parameter.get("name"),
         str(gps_parameter.get("default"))),
    ]
    for port in external_ports:
        expected_parameters.extend((
            ("INT32", port.get("parameter"),
             str(port.get("default_baud"))),
            ("INT32", port.get("function_parameter"),
             str(port.get("default_function"))),
        ))

    def parameter_metadata(name: object) -> tuple[object, list[tuple[int, str]]]:
        """从生成参数源提取 group/value，不在门禁里复制串口参数名称或枚举。"""
        if not isinstance(name, str):
            return None, []
        block = re.search(
            r"/\*\*(?P<body>(?:(?!/\*\*).)*?)\*/\s*"
            rf"PARAM_DEFINE_INT32\(\s*{re.escape(name)}\b",
            parameter_output,
            re.DOTALL,
        )
        if block is None:
            return None, []
        body = block.group("body")
        group = re.search(r"^\s*\*\s*@group\s+(\S+)\s*$", body,
                          re.MULTILINE)
        values = [
            (int(match.group(1)), match.group(2).strip())
            for match in re.finditer(
                r"^\s*\*\s*@value\s+(-?\d+)\s+(.+?)\s*$",
                body,
                re.MULTILINE,
            )
        ]
        return (None if group is None else group.group(1), values)

    expected_gps_values = [(0, "Disabled")] + [
        (port.get("serial"), f"Serial {port.get('serial')}")
        for port in external_ports
    ]
    gps_group, gps_values = parameter_metadata(gps_parameter.get("name"))
    if (gps_group != gps_parameter.get("group") or
            gps_values != expected_gps_values):
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated primary GPS Metadata differs from the manifest "
            f"(group={gps_group}, values={gps_values})",
        ))

    supported_baudrates = manifest.get("supported_baudrates", [])
    expected_baud_values = [
        (rate, "Auto" if rate == 0 else f"{rate} 8N1")
        for rate in supported_baudrates
    ] if isinstance(supported_baudrates, list) else []
    functions = manifest.get("functions", [])
    expected_function_metadata = [
        (function.get("value"), function.get("name"))
        for function in functions if isinstance(function, dict)
    ]
    for port in external_ports:
        baud_group, baud_values = parameter_metadata(port.get("parameter"))
        if baud_group != "Serial" or baud_values != expected_baud_values:
            violations.append(Violation(
                generator_path, 1, "R342",
                f"generated SERIAL{port.get('serial')} baud Metadata differs "
                "from the manifest",
            ))
        function_group, function_values = parameter_metadata(
            port.get("function_parameter")
        )
        if (function_group != "Serial" or
                function_values != expected_function_metadata):
            violations.append(Violation(
                generator_path, 1, "R342",
                f"generated SERIAL{port.get('serial')} function Metadata "
                "differs from the manifest",
            ))

    definition_re = re.compile(
        r"\bPARAM_DEFINE_(INT32|FLOAT)\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([^\s,)]+)\s*\)\s*;"
    )
    parameter_code = strip_c_comments(parameter_output)
    raw_definition_count = len(re.findall(
        r"\bPARAM_DEFINE_[A-Z0-9_]+\s*\(", parameter_code
    ))
    actual_parameters = [
        (match.group(1), match.group(2), match.group(3))
        for match in definition_re.finditer(parameter_code)
    ]
    if (raw_definition_count != len(expected_parameters) or
            raw_definition_count != len(actual_parameters)):
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated serial parameter output must contain exactly 17 "
            "parseable definitions "
            f"(raw={raw_definition_count}, parsed={len(actual_parameters)})",
        ))
    if actual_parameters != expected_parameters:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated serial parameter name/type/default sequence differs "
            f"from the manifest (expected={expected_parameters}, "
            f"actual={actual_parameters})",
        ))

    contract_code = strip_c_comments(contract_output)
    primary_parameter = re.search(
        r"^#define\s+DIMA_PRIMARY_GPS_PORT_PARAMETER\s+"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*$",
        contract_code,
        re.MULTILINE,
    )
    actual_primary = (
        None if primary_parameter is None else primary_parameter.group(1)
    )
    expected_primary = gps_parameter.get("name")
    if actual_primary != expected_primary:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated primary GPS parameter token differs from the manifest",
        ))

    contract_row_re = re.compile(
        r"\bX\(\s*(\d+)\s*,\s*([A-Za-z_][A-Za-z0-9_]*)\s*,\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*\)"
    )
    actual_rows = [
        (int(match.group(1)), match.group(2), match.group(3))
        for match in contract_row_re.finditer(contract_code)
    ]
    expected_rows = [
        (port.get("serial"), port.get("parameter"),
         port.get("function_parameter"))
        for port in external_ports
    ]
    if actual_rows != expected_rows:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated serial parameter list differs from the manifest "
            f"(expected={expected_rows}, actual={actual_rows})",
        ))

    descriptor_array = re.search(
        r"\bkSerialPorts\[\]\s*\{(?P<body>.*?)\};",
        contract_code,
        re.DOTALL,
    )
    descriptor_re = re.compile(
        r"\{\s*(\d+)\s*,\s*\"([^\"]*)\"\s*,\s*"
        r"\"([^\"]*)\"\s*,\s*\"([^\"]*)\"\s*,\s*"
        r"\"([^\"]*)\"\s*,\s*\"([^\"]*)\"\s*\},"
    )
    actual_descriptors = [] if descriptor_array is None else [
        (int(match.group(1)), *match.groups()[1:])
        for match in descriptor_re.finditer(descriptor_array.group("body"))
    ]
    expected_descriptors = [
        (port.get("serial"), port.get("parameter"),
         port.get("function_parameter"), port.get("role"),
         port.get("peripheral"), port.get("rx"))
        for port in external_ports
    ]
    if actual_descriptors != expected_descriptors:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated serial descriptors differ from the manifest "
            f"(expected={expected_descriptors}, actual={actual_descriptors})",
        ))

    baud_array = re.search(
        r"\bkSupportedBaudrates\[\]\s*\{(?P<body>[^}]*)\};",
        contract_code,
    )
    actual_baudrates = [] if baud_array is None else [
        int(value) for value in re.findall(
            r"\b(\d+)U\b", baud_array.group("body")
        )
    ]
    expected_baudrates = manifest.get("supported_baudrates")
    if actual_baudrates != expected_baudrates:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated baudrate array differs from the manifest "
            f"(expected={expected_baudrates}, actual={actual_baudrates})",
        ))

    expected_function_values = {
        function.get("name"): function.get("value")
        for function in functions if isinstance(function, dict)
    }
    constant_contract = (
        ("kSerialFunctionDisabled", "Disabled"),
        ("kSerialFunctionRcInput", "RC Input"),
        ("kSerialFunctionGps", "GPS"),
    )
    for constant, function_name in constant_contract:
        match = re.search(
            rf"\b{constant}\s*=\s*(-?\d+)\s*;", contract_code
        )
        actual_value = None if match is None else int(match.group(1))
        expected_value = expected_function_values.get(function_name)
        if actual_value != expected_value:
            violations.append(Violation(
                generator_path, 1, "R342",
                f"generated {constant} differs from the manifest "
                f"(expected={expected_value}, actual={actual_value})",
            ))

    function_array = re.search(
        r"\bkSupportedSerialFunctions\[\]\s*\{(?P<body>[^}]*)\};",
        contract_code,
    )
    actual_functions = [] if function_array is None else [
        int(value) for value in re.findall(
            r"(?<![A-Za-z0-9_])-?\d+(?![A-Za-z0-9_])",
            function_array.group("body"),
        )
    ]
    expected_functions = [
        function.get("value") for function in functions
        if isinstance(function, dict)
    ]
    if actual_functions != expected_functions:
        violations.append(Violation(
            generator_path, 1, "R342",
            "generated serial function array differs from the manifest "
            f"(expected={expected_functions}, actual={actual_functions})",
        ))


def scan_board_serial_manifest(violations: list[Violation]) -> None:
    """R196/R342：要求单一板级声明同时驱动串口硬件、参数和运行期所有权。"""
    path = ROOT / "Boards/H743/serial_ports.json"
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R196", f"invalid board serial manifest: {error}"
        ))
        return

    if manifest.get("format_version") != 1:
        violations.append(Violation(
            path, 1, "R196", "board serial manifest must use format version 1",
        ))

    _scan_generated_serial_contract(
        violations, manifest, manifest.get("ports", [])
        if isinstance(manifest.get("ports"), list) else []
    )

    expected_hardware_reference = {
        "source": "H743_FreeRTOS.ioc",
        "schematic": "VCU-H7-低成本版 V1.0 (2025-12-10)",
        "serial_order": "OTG1 USART1 USART2 USART3 UART4 UART5 USART6 UART7 UART8",
    }
    if manifest.get("hardware_reference") != expected_hardware_reference:
        violations.append(Violation(
            path, 1, "R196",
            "serial order must retain its locked VCU-H7 hardware source",
        ))

    expected_ports = [
        (0, "MAVLink USB", "USB_OTG1", None, None),
        (1, "Serial 1", "USART1", "PA9", "PA10"),
        (2, "Serial 2", "USART2", "PD5", "PD6"),
        (3, "Serial 3", "USART3", "PD8", "PD9"),
        (4, "Serial 4", "UART4", "PB9", "PB8"),
        (5, "Serial 5", "UART5", "PB13", "PB12"),
        (6, "SBUS", "USART6", "PC6", "PC7"),
        (7, "Serial 7", "UART7", "PE8", "PE7"),
        (8, "Serial 8", "UART8", "PE1", "PE0"),
    ]
    ports = manifest.get("ports", [])
    actual_ports = [
        (port.get("serial"), port.get("role"), port.get("peripheral"),
         port.get("tx"), port.get("rx"))
        for port in ports if isinstance(port, dict)
    ] if isinstance(ports, list) else []
    if actual_ports != expected_ports:
        violations.append(Violation(
            path, 1, "R196",
            "board serial order must directly follow USART/UART1..8",
        ))

    expected_runtime = [
        (1, "huart1", "DMA_REQUEST_USART1_RX",
         "USART1_IRQn", "GPIOA", "GPIO_PIN_10", "GPIO_AF7_USART1", 10),
        (2, "huart2", "DMA_REQUEST_USART2_RX",
         "USART2_IRQn", "GPIOD", "GPIO_PIN_6", "GPIO_AF7_USART2", 6),
        (3, "huart3", "DMA_REQUEST_USART3_RX",
         "USART3_IRQn", "GPIOD", "GPIO_PIN_9", "GPIO_AF7_USART3", 9),
        (4, "huart4", "DMA_REQUEST_UART4_RX",
         "UART4_IRQn", "GPIOB", "GPIO_PIN_8", "GPIO_AF8_UART4", 8),
        (5, "huart5", "DMA_REQUEST_UART5_RX",
         "UART5_IRQn", "GPIOB", "GPIO_PIN_12", "GPIO_AF14_UART5", 12),
        (6, "huart6", "DMA_REQUEST_USART6_RX",
         "USART6_IRQn", "GPIOC", "GPIO_PIN_7", "GPIO_AF7_USART6", 7),
        (7, "huart7", "DMA_REQUEST_UART7_RX",
         "UART7_IRQn", "GPIOE", "GPIO_PIN_7", "GPIO_AF7_UART7", 7),
        (8, "huart8", "DMA_REQUEST_UART8_RX",
         "UART8_IRQn", "GPIOE", "GPIO_PIN_0", "GPIO_AF8_UART8", 0),
    ]
    actual_runtime = [
        (port.get("serial"), port.get("uart_handle"),
         port.get("dma_request"), port.get("irq"),
         port.get("rx_gpio"), port.get("rx_pin"), port.get("rx_af"),
         port.get("rx_index"))
        for port in ports[1:] if isinstance(port, dict)
    ] if isinstance(ports, list) else []
    if actual_runtime != expected_runtime:
        violations.append(Violation(
            path, 1, "R196",
            "STM32 serial handle/DMA/IRQ/AF mapping changed",
        ))
    expected_default_sources = [
        "VCU-H7 USART1 product default", "VCU-H7 USART2 product default",
        "VCU-H7 USART3 product default", "VCU-H7 UART4 product default",
        "CubeMX UART5 normal 8N1", "VCU-H7 USART6 RC product default",
        "VCU-H7 UART7 product default", "VCU-H7 UART8 product default",
    ]
    actual_default_sources = [
        port.get("default_source") for port in ports[1:]
    ] if isinstance(ports, list) else []
    if actual_default_sources != expected_default_sources:
        violations.append(Violation(
            path, 1, "R196",
            "each SERIALx default must retain an explicit board source",
        ))

    expected_functions = [
        {"value": 0, "name": "Disabled"},
        {"value": 1, "name": "RC Input"},
        {"value": 2, "name": "GPS"},
    ]
    if (manifest.get("default_rc_port") != 6 or
            manifest.get("functions") != expected_functions):
        violations.append(Violation(
            path, 1, "R196",
            "SERIAL6 must be the sole default RC Input function",
        ))

    if manifest.get("gps_port_parameter") != {
        "name": "GPS_1_CONFIG", "default": 0, "group": "GPS",
    }:
        violations.append(Violation(
            path, 1, "R342",
            "primary GPS port parameter must be declared by the serial manifest",
        ))

    expected_baudrates = [
        0, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
        230400, 460800, 500000, 921600, 1000000, 1500000,
        2000000, 3000000,
    ]
    if manifest.get("supported_baudrates") != expected_baudrates:
        violations.append(Violation(
            path, 1, "R196",
            "board serial baud whitelist differs from the approved common-rate subset",
        ))

    if isinstance(ports, list):
        for port in ports[1:]:
            serial = port.get("serial") if isinstance(port, dict) else None
            if (not isinstance(port, dict) or
                    port.get("parameter") != f"SERIAL{serial}_BAUD" or
                    port.get("function_parameter") !=
                    f"SERIAL{serial}_FUNCTION"):
                violations.append(Violation(
                    path, 1, "R196",
                    "each external port needs matching SERIALx_BAUD/FUNCTION names",
                ))
                break
            if (port.get("default_baud") not in expected_baudrates or
                    port.get("default_function") not in
                    {entry["value"] for entry in expected_functions}):
                violations.append(Violation(
                    path, 1, "R196",
                    "serial defaults must use manifest-declared baud and function values",
                ))
                break

    rc_default_owners = [
        port.get("serial") for port in ports[1:]
        if isinstance(port, dict) and port.get("default_function") == 1
    ] if isinstance(ports, list) else []
    if rc_default_owners != [manifest.get("default_rc_port")]:
        violations.append(Violation(
            path, 1, "R196",
            "exactly the manifest default RC port must own RC Input",
        ))

    require_literals(
        ROOT / "tools/serial/generate_config.py",
        (
            ("Tools/serial/generate_config.py", "R196",
             "serial generator must retain its pinned PX4 generation origin"),
            ("SERIAL{serial}_BAUD", "R196",
             "baud parameter names must follow the real board serial index"),
            ("SERIAL{serial}_FUNCTION", "R196",
             "function parameter names must follow the real board serial index"),
            ("serial_baud_params.c", "R196",
             "serial generator must emit parameter definitions"),
            ("SerialContract.hpp", "R196",
             "serial generator must emit the platform-neutral contract"),
            ("BoardUartResources.hpp", "R196",
             "serial generator must emit the STM32H7 resource mapping"),
            ("gps_port_parameter", "R342",
             "serial generator must emit the primary GPS port parameter"),
            ("DIMA_PRIMARY_GPS_PORT_PARAMETER", "R342",
             "serial generator must emit the primary GPS parameter token"),
        ),
        violations,
    )

    definitions_root = (
        ROOT / "Dima/middleware/parameters/definitions"
    )
    topology_definition_re = re.compile(
        r"\bPARAM_DEFINE_[A-Z0-9_]+\s*\(\s*"
        r"(?:SERIAL[1-8]_(?:BAUD|FUNCTION)|GPS_1_CONFIG)\b"
    )
    parameter_source_suffixes = {
        ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx",
    }
    for source in definitions_root.rglob("*"):
        if (not source.is_file() or
                source.suffix.lower() not in parameter_source_suffixes):
            continue
        text = source.read_text(encoding="utf-8")
        match = topology_definition_re.search(text)
        if match:
            violations.append(Violation(
                source, line_for(text, match.group(0)), "R342",
                "board serial topology parameters must be generated from "
                "serial_ports.json",
            ))

    project_mk_path = ROOT / "make/project.mk"
    project_mk = project_mk_path.read_text(encoding="utf-8")

    def make_variable_body(name: str) -> str:
        match = re.search(
            rf"^{re.escape(name)}\s*:=\s*\\\s*\n"
            rf"(?P<body>(?:^[^\n]*\\\s*\n)*^[^\n]*)",
            project_mk,
            re.MULTILINE,
        )
        return "" if match is None else match.group("body")

    if "$(SERIAL_BAUD_PARAMETERS)" not in make_variable_body(
            "PARAMETER_DEFINITIONS"):
        violations.append(Violation(
            project_mk_path, 1, "R342",
            "parameter catalogue must include generated serial definitions",
        ))
    require_literals(
        ROOT / "Dima/modules/serial/SerialConfig.hpp",
        (("DIMA_PRIMARY_GPS_PORT_PARAMETER", "R342",
          "SerialConfig must use the generated primary GPS parameter token"),),
        violations,
    )

    parameter_gate_path = ROOT / "tools/architecture/parameter_mavlink.py"
    parameter_gate = parameter_gate_path.read_text(encoding="utf-8")
    manual_schema = re.search(
        r"[\"']SERIAL[1-8]_(?:BAUD|FUNCTION)[\"']", parameter_gate
    )
    if manual_schema:
        violations.append(Violation(
            parameter_gate_path,
            line_for(parameter_gate, manual_schema.group(0)), "R342",
            "parameter gate must derive serial names and defaults from the manifest",
        ))
    require_literals(
        parameter_gate_path,
        (
            ("_generated_serial_parameter_schema(violations)", "R342",
             "parameter gate must derive the generated schema from the manifest"),
            ('manifest.get("ports")', "R342",
             "parameter gate must enumerate manifest serial ports"),
            ('port.get("parameter")', "R342",
             "parameter gate must read manifest baud parameter names"),
            ('port.get("function_parameter")', "R342",
             "parameter gate must read manifest function parameter names"),
            ('manifest.get("gps_port_parameter")', "R342",
             "parameter gate must read the manifest GPS port parameter"),
        ),
        violations,
    )

    board_init_path = ROOT / "Boards/H743/Src/board_init.c"
    board_init = board_init_path.read_text(encoding="utf-8")
    init_tokens = [
        "MX_USART1_UART_Init();", "MX_USART2_UART_Init();",
        "MX_USART3_UART_Init();", "MX_UART4_Init();",
        "MX_UART5_Init();", "MX_USART6_UART_Init();",
        "MX_UART7_Init();", "MX_UART8_Init();",
    ]
    positions = [board_init.find(token) for token in init_tokens]
    if any(position < 0 for position in positions) or len(set(positions)) != 8:
        violations.append(Violation(
            board_init_path, 1, "R196",
            "all eight external serial peripherals must be initialized",
        ))

    ioc_path = ROOT / "H743_FreeRTOS.ioc"
    ioc_text = ioc_path.read_text(encoding="utf-8")
    usart_header = (ROOT / "Core/Inc/usart.h").read_text(encoding="utf-8")
    for port in ports[1:] if isinstance(ports, list) else []:
        peripheral = port.get("peripheral")
        tx = port.get("tx")
        rx = port.get("rx")
        handle = port.get("uart_handle")
        for token in (f"{tx}.Signal={peripheral}_TX",
                      f"{rx}.Signal={peripheral}_RX"):
            if token not in ioc_text:
                violations.append(Violation(
                    ioc_path, 1, "R196",
                    f"serial manifest pin is absent from CubeMX IOC: {token}",
                ))
        if f"extern UART_HandleTypeDef {handle};" not in usart_header:
            violations.append(Violation(
                ROOT / "Core/Inc/usart.h", 1, "R196",
                f"serial manifest UART handle is absent: {handle}",
            ))

    for source in first_party_sources():
        text = source.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        match = re.search(r"\bSER_(?:TEL[1-4]|GPS[1-3]|RC)_BAUD\b", code)
        if match:
            violations.append(Violation(
                source, line_for(code, match.group(0)), "R196",
                "serial identity must use SERIAL1..SERIAL8, not assigned function names",
            ))
        topology_consumer = re.search(
            r"(?:\bpx4::params::|[\"'])"
            r"(?:SERIAL[1-8]_(?:BAUD|FUNCTION)|GPS_1_CONFIG)\b",
            code,
        )
        if topology_consumer:
            violations.append(Violation(
                source, line_for(code, topology_consumer.group(0)), "R342",
                "serial topology consumers must use the generated contract",
            ))

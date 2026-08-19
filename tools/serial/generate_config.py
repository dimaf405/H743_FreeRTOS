#!/usr/bin/env python3
"""Generate Dima serial parameters and board mappings from one board manifest.

The generation model follows PX4 v1.17.0 Tools/serial/generate_config.py:
the board declares its real serial ports once, and both SER/driver parameters
and runtime port resolution are generated from that declaration.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
PIN_RE = re.compile(r"^P[A-K][0-9]{1,2}$")
EXPECTED_SERIALS = list(range(9))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def require_identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or IDENTIFIER_RE.fullmatch(value) is None:
        raise RuntimeError(f"invalid {field}: {value!r}")
    return value


def load_manifest(path: Path) -> tuple[dict, list[dict]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 2 or data.get("board") != "dimah743":
        raise RuntimeError("unsupported serial manifest identity")
    expected_hardware_reference = {
        "source": "H743_FreeRTOS.ioc",
        "schematic": "VCU-H7-低成本版 V1.0 (2025-12-10)",
        "serial_order": "OTG1 USART1 USART2 USART3 UART4 UART5 USART6 UART7 UART8",
    }
    if data.get("hardware_reference") != expected_hardware_reference:
        raise RuntimeError("board serial order lacks the locked hardware reference")

    ports = data.get("ports")
    if not isinstance(ports, list) or [port.get("serial") for port in ports] != EXPECTED_SERIALS:
        raise RuntimeError("serial manifest must define SERIAL0..SERIAL8 in order")

    rates = data.get("supported_baudrates")
    if (not isinstance(rates, list) or not rates or rates[0] != 0 or
            rates != sorted(set(rates)) or
            any(not isinstance(rate, int) or rate < 0 for rate in rates)):
        raise RuntimeError("supported_baudrates must be a sorted unique integer list starting at Auto/0")

    functions = data.get("functions")
    if functions != [
        {"value": 0, "name": "Disabled"},
        {"value": 1, "name": "RC Input"},
    ]:
        raise RuntimeError("current product serial functions must be exactly Disabled/RC Input")

    configurable = []
    for port in ports:
        serial = port["serial"]
        if serial == 0:
            if port.get("peripheral") != "USB_OTG1" or port.get("configurable_baud") is not False:
                raise RuntimeError("SERIAL0 must remain the MAVLink-only USB OTG1 port")
            continue

        expected_parameter = f"SERIAL{serial}_BAUD"
        if port.get("parameter") != expected_parameter:
            raise RuntimeError(
                f"SERIAL{serial} parameter must be named {expected_parameter}"
            )
        expected_function = f"SERIAL{serial}_FUNCTION"
        if port.get("function_parameter") != expected_function:
            raise RuntimeError(
                f"SERIAL{serial} function must be named {expected_function}"
            )
        if port.get("default_function") not in (0, 1):
            raise RuntimeError(f"unsupported function for SERIAL{serial}")
        default_baud = port.get("default_baud")
        if default_baud not in rates:
            raise RuntimeError(f"unsupported default baud for SERIAL{serial}")
        if not isinstance(port.get("default_source"), str) or not port["default_source"]:
            raise RuntimeError(f"missing default baud source for SERIAL{serial}")
        for field in ("peripheral", "uart_handle", "dma_request", "irq",
                      "rx_gpio", "rx_pin", "rx_af"):
            require_identifier(port.get(field), field)
        if PIN_RE.fullmatch(str(port.get("tx"))) is None or PIN_RE.fullmatch(str(port.get("rx"))) is None:
            raise RuntimeError(f"invalid STM32 pin declaration for SERIAL{serial}")
        if not isinstance(port.get("role"), str) or not port["role"]:
            raise RuntimeError(f"missing role for SERIAL{serial}")
        if port.get("rx_index") != int(str(port["rx"])[2:]):
            raise RuntimeError(f"RX pin/index mismatch for SERIAL{serial}")
        configurable.append(port)

    default_rc_port = data.get("default_rc_port")
    if default_rc_port not in [port["serial"] for port in configurable]:
        raise RuntimeError("default_rc_port is not a real external serial port")
    rc_defaults = [
        port["serial"] for port in configurable
        if port["default_function"] == 1
    ]
    if rc_defaults != [default_rc_port]:
        raise RuntimeError("exactly the real default RC port must own RC Input")

    legacy = data.get("legacy_rc_port_map")
    if not isinstance(legacy, dict):
        raise RuntimeError("legacy_rc_port_map is missing")
    for old_value, new_value in legacy.items():
        if not old_value.isdigit() or not isinstance(new_value, int):
            raise RuntimeError("legacy RC port map must contain integer values")
        if new_value != 0 and new_value not in [port["serial"] for port in configurable]:
            raise RuntimeError("legacy RC port maps to an unavailable serial port")

    legacy_baud = data.get("legacy_baud_parameter_map")
    expected_legacy_baud_names = {
        "SER_RC_BAUD", "SER_TEL1_BAUD", "SER_TEL2_BAUD",
        "SER_TEL3_BAUD", "SER_TEL4_BAUD", "SER_GPS1_BAUD",
        "SER_GPS2_BAUD", "SER_GPS3_BAUD",
    }
    if (not isinstance(legacy_baud, dict) or
            set(legacy_baud) != expected_legacy_baud_names):
        raise RuntimeError("legacy baud parameter map is incomplete")
    for name, serial in legacy_baud.items():
        require_identifier(name, "legacy baud parameter")
        if not isinstance(serial, int) or (
                serial != 0 and
                serial not in [port["serial"] for port in configurable]):
            raise RuntimeError("legacy baud parameter maps to an unavailable port")

    return data, configurable


def baud_value_lines(rates: list[int]) -> list[str]:
    return [
        " * @value 0 Auto" if rate == 0 else f" * @value {rate} {rate} 8N1"
        for rate in rates
    ]


def generate_baud_parameters(data: dict, ports: list[dict]) -> str:
    rates = data["supported_baudrates"]
    sections = [
        "/****************************************************************************",
        " * Generated from Boards/H743/serial_ports.json. DO NOT EDIT.",
        " ****************************************************************************/",
        "",
    ]
    value_lines = baud_value_lines(rates)
    function_value_lines = [
        f" * @value {function['value']} {function['name']}"
        for function in data["functions"]
    ]
    for port in ports:
        serial = port["serial"]
        sections.extend([
            "/**",
            f" * Baudrate for SERIAL{serial}: {port['role']} ({port['peripheral']}).",
            " *",
            f" * Default source: {port['default_source']}.",
            " * Configure the normal 8N1 baudrate for this physical board port.",
            " * A protocol driver may temporarily override framing and baudrate while",
            " * it owns the port.",
            " *",
            *value_lines,
            " * @group Serial",
            " * @reboot_required true",
            " */",
            f"PARAM_DEFINE_INT32({port['parameter']}, {port['default_baud']});",
            "",
            "/**",
            f" * Function assigned to SERIAL{serial}: {port['role']} ({port['peripheral']}).",
            " *",
            " * Only functions with a production data path are selectable.",
            *function_value_lines,
            " * @group Serial",
            " * @reboot_required true",
            " */",
            f"PARAM_DEFINE_INT32({port['function_parameter']}, {port['default_function']});",
            "",
        ])

    sections.extend([
        "/** Internal persisted serial-schema migration version.",
        " * This parameter is consumed by ParameterService and is not exposed to QGC.",
        " */",
        "PARAM_DEFINE_INT32(DIMA_SER_VER, 0);",
        "",
    ])
    return "\n".join(sections)


def generate_driver_parameters(data: dict, ports: list[dict]) -> str:
    default_port = data["default_rc_port"]
    sections = [
        "/****************************************************************************",
        " * Generated from Boards/H743/serial_ports.json. DO NOT EDIT.",
        " ****************************************************************************/",
        "",
        "/**",
        " * Serial Configuration for RC Input Driver.",
        " *",
        " * Deprecated compatibility input used only for one-time migration to",
        " * SERIAL1_FUNCTION..SERIAL8_FUNCTION. New configurations must assign",
        " * RC Input through exactly one SERIALx_FUNCTION parameter.",
        " *",
        f" * @value 0 Board default (SERIAL {default_port})",
    ]
    for port in ports:
        sections.append(
            f" * @value {port['serial']} SERIAL {port['serial']} - "
            f"{port['role']} ({port['peripheral']} RX {port['rx']})"
        )
    sections.extend([
        " * @group Serial",
        " * @reboot_required true",
        " */",
        f"PARAM_DEFINE_INT32(RC_PORT_CONFIG, {default_port});",
        "",
    ])
    return "\n".join(sections)


def macro(name: str, rows: list[str]) -> list[str]:
    output = [f"#define {name}(X) \\"]
    for index, row in enumerate(rows):
        suffix = " \\" if index + 1 < len(rows) else ""
        output.append(f"    X({row}){suffix}")
    return output


def generate_header(data: dict, ports: list[dict]) -> str:
    rates = ", ".join(f"{rate}U" for rate in data["supported_baudrates"])
    port_rows = [
        f"{port['serial']}, {port['parameter']}, {port['function_parameter']}"
        for port in ports
    ]
    stm32_rows = [
        ", ".join([
            str(port["serial"]), port["uart_handle"], port["dma_request"],
            port["irq"], port["rx_gpio"], port["rx_pin"], port["rx_af"],
            str(port["rx_index"]),
        ])
        for port in ports
    ]
    descriptor_rows = [
        "    {" + ", ".join([
            str(port["serial"]), str(port["default_baud"]),
            str(port["default_function"]), json.dumps(port["parameter"]),
            json.dumps(port["function_parameter"]), json.dumps(port["role"]),
            json.dumps(port["peripheral"]), json.dumps(port["tx"]),
            json.dumps(port["rx"]),
        ]) + "},"
        for port in ports
    ]
    migration_cases = [
        f"    case {old_value}: return {new_value};"
        for old_value, new_value in data["legacy_rc_port_map"].items()
    ]
    legacy_baud_lines = [
        f"    if (std::strcmp(name, {json.dumps(old_name)}) == 0) return {serial};"
        for old_name, serial in data["legacy_baud_parameter_map"].items()
    ]
    function_values = ", ".join(
        str(function["value"]) for function in data["functions"]
    )
    function_disabled = next(
        function["value"] for function in data["functions"]
        if function["name"] == "Disabled"
    )
    function_rc = next(
        function["value"] for function in data["functions"]
        if function["name"] == "RC Input"
    )

    lines = [
        "#pragma once",
        "",
        "// Generated from Boards/H743/serial_ports.json. DO NOT EDIT.",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <cstring>",
        "",
        *macro("DIMA_BOARD_SERIAL_PARAMETER_LIST", port_rows),
        "",
        *macro("DIMA_STM32_SERIAL_PORT_LIST", stm32_rows),
        "",
        "namespace dima::board {",
        "",
        "struct SerialPortDescriptor {",
        "    std::int32_t port_id;",
        "    std::uint32_t default_baud;",
        "    std::int32_t default_function;",
        "    const char *parameter_name;",
        "    const char *function_parameter_name;",
        "    const char *role;",
        "    const char *peripheral;",
        "    const char *tx_pin;",
        "    const char *rx_pin;",
        "};",
        "",
        "inline constexpr SerialPortDescriptor kSerialPorts[]{",
        *descriptor_rows,
        "};",
        f"inline constexpr std::int32_t kDefaultRcPort = {data['default_rc_port']};",
        f"inline constexpr std::uint32_t kSerialSchemaVersion = {data['schema_version']}U;",
        f"inline constexpr std::uint32_t kSupportedBaudrates[]{{{rates}}};",
        f"inline constexpr std::int32_t kSerialFunctionDisabled = {function_disabled};",
        f"inline constexpr std::int32_t kSerialFunctionRcInput = {function_rc};",
        f"inline constexpr std::int32_t kSupportedSerialFunctions[]{{{function_values}}};",
        "",
        "constexpr bool serial_port_supported(std::int32_t port) noexcept",
        "{",
        "    for (const SerialPortDescriptor &descriptor : kSerialPorts) {",
        "        if (descriptor.port_id == port) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "constexpr bool serial_baud_supported(std::uint32_t baudrate) noexcept",
        "{",
        "    for (const std::uint32_t candidate : kSupportedBaudrates) {",
        "        if (candidate == baudrate) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "inline bool serial_baud_parameter(const char *name) noexcept",
        "{",
        "    if (name == nullptr) return false;",
        "    for (const SerialPortDescriptor &descriptor : kSerialPorts) {",
        "        if (std::strcmp(name, descriptor.parameter_name) == 0) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "inline bool serial_function_parameter(const char *name) noexcept",
        "{",
        "    if (name == nullptr) return false;",
        "    for (const SerialPortDescriptor &descriptor : kSerialPorts) {",
        "        if (std::strcmp(name, descriptor.function_parameter_name) == 0) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "constexpr bool serial_function_supported(std::int32_t function) noexcept",
        "{",
        "    for (const std::int32_t candidate : kSupportedSerialFunctions) {",
        "        if (candidate == function) return true;",
        "    }",
        "    return false;",
        "}",
        "",
        "constexpr const SerialPortDescriptor *serial_port(std::int32_t port) noexcept",
        "{",
        "    for (const SerialPortDescriptor &descriptor : kSerialPorts) {",
        "        if (descriptor.port_id == port) return &descriptor;",
        "    }",
        "    return nullptr;",
        "}",
        "",
        "constexpr std::int32_t migrate_legacy_rc_port(std::int32_t port) noexcept",
        "{",
        "    switch (port) {",
        *migration_cases,
        "    default: return port;",
        "    }",
        "}",
        "",
        "inline std::int32_t legacy_serial_for_baud_parameter(const char *name) noexcept",
        "{",
        "    if (name == nullptr) return 0;",
        *legacy_baud_lines,
        "    return 0;",
        "}",
        "",
        "} // namespace dima::board",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    data, ports = load_manifest(args.manifest)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "serial_baud_params.c").write_text(
        generate_baud_parameters(data, ports), encoding="utf-8", newline="\n"
    )
    (args.output / "serial_config_params.c").write_text(
        generate_driver_parameters(data, ports), encoding="utf-8", newline="\n"
    )
    (args.output / "board_serial_config.hpp").write_text(
        generate_header(data, ports), encoding="utf-8", newline="\n"
    )
    print(f"generated SERIAL0..SERIAL8 configuration for {data['board']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

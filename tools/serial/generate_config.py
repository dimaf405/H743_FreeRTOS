#!/usr/bin/env python3
"""从单一板级 manifest 生成 Dima 串口参数合同和 STM32H7 私有资源映射。

生成模型固定对照 PX4 v1.17.0 commit
``d6f12ad1c4f70ad3230afd7d86e971421e02fef4`` 的
``Tools/serial/generate_config.py``：板只声明一次真实端口；公共参数/所有权合同与
私有 HAL/CMSIS 资源分别输出，防止 common 模块借生成头反向获得芯片标识符。
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
PIN_RE = re.compile(r"^P[A-K][0-9]{1,2}$")


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
    """校验 manifest 结构、唯一标识及内部引用，不冻结端口表或产品默认值。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    if (data.get("format_version") != 1 or
            not isinstance(data.get("board"), str) or not data["board"]):
        raise RuntimeError("unsupported serial manifest identity")
    hardware_reference = data.get("hardware_reference")
    if (not isinstance(hardware_reference, dict) or
            not isinstance(hardware_reference.get("source"), str) or
            not hardware_reference["source"]):
        raise RuntimeError("serial manifest lacks its hardware source")

    gps_port_parameter = data.get("gps_port_parameter")
    if (not isinstance(gps_port_parameter, dict) or
            not isinstance(gps_port_parameter.get("default"), int) or
            not isinstance(gps_port_parameter.get("group"), str) or
            not gps_port_parameter["group"]):
        raise RuntimeError("invalid primary GPS port parameter declaration")
    require_identifier(gps_port_parameter.get("name"), "gps parameter")

    ports = data.get("ports")
    if not isinstance(ports, list) or not ports:
        raise RuntimeError("serial manifest ports must be a non-empty list")
    serials = [
        port.get("serial") for port in ports if isinstance(port, dict)
    ]
    if (len(serials) != len(ports) or
            any(type(serial) is not int or serial < 0 for serial in serials) or
            len(serials) != len(set(serials)) or 0 not in serials):
        raise RuntimeError("serial port indexes must be unique and include port 0")

    rates = data.get("supported_baudrates")
    if (not isinstance(rates, list) or not rates or
            rates != sorted(set(rates)) or
            any(not isinstance(rate, int) or rate < 0 for rate in rates)):
        raise RuntimeError(
            "supported_baudrates must be a sorted unique non-negative list"
        )

    functions = data.get("functions")
    if not isinstance(functions, list) or not functions:
        raise RuntimeError("serial functions must be a non-empty list")
    function_values = [
        function.get("value") for function in functions
        if isinstance(function, dict)
    ]
    function_names = [
        function.get("name") for function in functions
        if isinstance(function, dict)
    ]
    function_roles = [
        function.get("role") for function in functions
        if isinstance(function, dict)
    ]
    if (len(function_values) != len(functions) or
            any(type(value) is not int for value in function_values) or
            any(not isinstance(name, str) or not name
                for name in function_names) or
            any(not isinstance(role, str) or not role
                for role in function_roles) or
            len(function_values) != len(set(function_values)) or
            len(function_names) != len(set(function_names)) or
            len(function_roles) != len(set(function_roles))):
        raise RuntimeError("serial function values, names and roles must be unique")
    # 现有运行时代码消费这三个语义角色；manifest 可继续增加其他功能项，
    # 生成器不再冻结完整功能列表、数值或顺序。
    required_roles = {"disabled", "rc_input", "gps"}
    if not required_roles.issubset(function_roles):
        raise RuntimeError(
            f"serial functions are missing runtime roles: "
            f"{sorted(required_roles - set(function_roles))}"
        )

    configurable = []
    for port in ports:
        serial = port["serial"]
        if serial == 0:
            continue

        require_identifier(port.get("parameter"), "baud parameter")
        require_identifier(port.get("function_parameter"), "function parameter")
        if port.get("default_function") not in function_values:
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

    return data, configurable


def baud_value_lines(rates: list[int]) -> list[str]:
    return [
        " * @value 0 Auto" if rate == 0 else f" * @value {rate} {rate} 8N1"
        for rate in rates
    ]


def generate_serial_parameters(data: dict, ports: list[dict]) -> str:
    """由 manifest 生成 PARAM_DEFINE 源，参数名、默认值和枚举不在模板外复制。"""
    rates = data["supported_baudrates"]
    gps_port_parameter = data["gps_port_parameter"]
    sections = [
        "/****************************************************************************",
        " * Generated from Boards/H743/serial_ports.json. DO NOT EDIT.",
        " ****************************************************************************/",
        "",
        "/**",
        " * Serial connector used by the primary GPS receiver.",
        " *",
        " * The values are Dima board connector indexes, generated in the same role as",
        " * PX4 GPS_1_CONFIG. A legacy SERIALx_FUNCTION=GPS assignment remains accepted",
        " * when this parameter is Disabled.",
        " *",
        " * @value 0 Disabled",
        *[
            f" * @value {port['serial']} Serial {port['serial']}"
            for port in ports
        ],
        f" * @group {gps_port_parameter['group']}",
        " */",
        "PARAM_DEFINE_INT32({name}, {default});".format(
            **gps_port_parameter
        ),
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
            " */",
            f"PARAM_DEFINE_INT32({port['parameter']}, {port['default_baud']});",
            "",
            "/**",
            f" * Function assigned to SERIAL{serial}: {port['role']} ({port['peripheral']}).",
            " *",
            " * Only functions with a production data path are selectable.",
            *function_value_lines,
            " * @group Serial",
            " */",
            f"PARAM_DEFINE_INT32({port['function_parameter']}, {port['default_function']});",
            "",
        ])

    return "\n".join(sections)


def macro(name: str, rows: list[str]) -> list[str]:
    output = [f"#define {name}(X) \\"]
    for index, row in enumerate(rows):
        suffix = " \\" if index + 1 < len(rows) else ""
        output.append(f"    X({row}){suffix}")
    return output


def generate_serial_contract(data: dict, ports: list[dict]) -> str:
    """生成 HAL 无关的端口描述符、支持集合与唯一参数展开宏。"""
    rates = ", ".join(f"{rate}U" for rate in data["supported_baudrates"])
    port_rows = [
        f"{port['serial']}, {port['parameter']}, {port['function_parameter']}"
        for port in ports
    ]
    descriptor_rows = [
        "    {" + ", ".join([
            str(port["serial"]), json.dumps(port["parameter"]),
            json.dumps(port["function_parameter"]), json.dumps(port["role"]),
            json.dumps(port["peripheral"]), json.dumps(port["rx"]),
        ]) + "},"
        for port in ports
    ]
    function_values = ", ".join(
        str(function["value"]) for function in data["functions"]
    )
    functions_by_role = {
        function["role"]: function["value"] for function in data["functions"]
    }
    function_disabled = functions_by_role["disabled"]
    function_rc = functions_by_role["rc_input"]
    function_gps = functions_by_role["gps"]

    lines = [
        "#pragma once",
        "",
        "// Generated serial contract from Boards/H743/serial_ports.json.",
        "// DO NOT EDIT. This header must remain HAL/CMSIS independent.",
        "#include <cstddef>",
        "#include <cstdint>",
        "#include <cstring>",
        "",
        f"#define DIMA_PRIMARY_GPS_PORT_PARAMETER {data['gps_port_parameter']['name']}",
        "",
        *macro("DIMA_BOARD_SERIAL_PARAMETER_LIST", port_rows),
        "",
        "namespace dima::board {",
        "",
        "struct SerialPortDescriptor {",
        "    std::int32_t port_id;",
        "    const char *parameter_name;",
        "    const char *function_parameter_name;",
        "    const char *role;",
        "    const char *peripheral;",
        "    const char *rx_pin;",
        "};",
        "",
        "inline constexpr SerialPortDescriptor kSerialPorts[]{",
        *descriptor_rows,
        "};",
        f"inline constexpr std::uint32_t kSupportedBaudrates[]{{{rates}}};",
        f"inline constexpr std::int32_t kSerialFunctionDisabled = {function_disabled};",
        f"inline constexpr std::int32_t kSerialFunctionRcInput = {function_rc};",
        f"inline constexpr std::int32_t kSerialFunctionGps = {function_gps};",
        f"inline constexpr std::int32_t kSupportedSerialFunctions[]{{{function_values}}};",
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
        "} // namespace dima::board",
        "",
    ]
    return "\n".join(lines)


def generate_uart_resources(ports: list[dict]) -> str:
    """生成仅 STM32H7 backend 可见的 UART/DMA/IRQ/GPIO 资源宏。"""
    stm32_rows = [
        ", ".join([
            str(port["serial"]), port["uart_handle"], port["dma_request"],
            port["irq"], port["rx_gpio"], port["rx_pin"], port["rx_af"],
            str(port["rx_index"]),
        ])
        for port in ports
    ]
    lines = [
        "#pragma once",
        "",
        "// Generated STM32H7 UART resources from Boards/H743/serial_ports.json.",
        "// DO NOT EDIT. Only the STM32H7 serial backend may include this header.",
        "",
        *macro("DIMA_STM32_SERIAL_PORT_LIST", stm32_rows),
        "",
    ]
    return "\n".join(lines)


def write_generated(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def main() -> int:
    args = parse_args()
    data, ports = load_manifest(args.manifest)
    if args.output.name.lower() != "serial":
        raise RuntimeError(
            "--output must name the generated serial contract directory"
        )
    stm32_output = args.output.parent / "stm32h7" / "serial"
    write_generated(
        args.output / "serial_baud_params.c",
        generate_serial_parameters(data, ports),
    )
    write_generated(
        args.output / "SerialContract.hpp",
        generate_serial_contract(data, ports),
    )
    write_generated(
        stm32_output / "BoardUartResources.hpp",
        generate_uart_resources(ports),
    )
    print(
        f"generated {len(ports)} configurable serial ports and STM32H7 "
        f"resources for {data['board']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

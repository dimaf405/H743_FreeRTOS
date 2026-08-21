"""Board serial manifest and generated serial ownership checks."""

from __future__ import annotations

import json
import re

from architecture.common import (
    ROOT,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
)


def scan_board_serial_manifest(violations: list[Violation]) -> None:
    """R196: one real SERIAL0..8 board declaration drives all serial code."""
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
        (0, "MAVLink USB", "USB_OTG1", None, None, None),
        (1, "Serial 1", "USART1", "PA9", "PA10", "SERIAL1_BAUD"),
        (2, "Serial 2", "USART2", "PD5", "PD6", "SERIAL2_BAUD"),
        (3, "Serial 3", "USART3", "PD8", "PD9", "SERIAL3_BAUD"),
        (4, "Serial 4", "UART4", "PB9", "PB8", "SERIAL4_BAUD"),
        (5, "Serial 5", "UART5", "PB13", "PB12", "SERIAL5_BAUD"),
        (6, "SBUS", "USART6", "PC6", "PC7", "SERIAL6_BAUD"),
        (7, "Serial 7", "UART7", "PE8", "PE7", "SERIAL7_BAUD"),
        (8, "Serial 8", "UART8", "PE1", "PE0", "SERIAL8_BAUD"),
    ]
    ports = manifest.get("ports", [])
    actual_ports = [
        (port.get("serial"), port.get("role"), port.get("peripheral"),
         port.get("tx"), port.get("rx"), port.get("parameter"))
        for port in ports if isinstance(port, dict)
    ] if isinstance(ports, list) else []
    if actual_ports != expected_ports:
        violations.append(Violation(
            path, 1, "R196",
            "board serial order must directly follow USART/UART1..8",
        ))

    expected_runtime = [
        (1, "SERIAL1_FUNCTION", 921600, 0, "huart1", "DMA_REQUEST_USART1_RX",
         "USART1_IRQn", "GPIOA", "GPIO_PIN_10", "GPIO_AF7_USART1", 10),
        (2, "SERIAL2_FUNCTION", 0, 0, "huart2", "DMA_REQUEST_USART2_RX",
         "USART2_IRQn", "GPIOD", "GPIO_PIN_6", "GPIO_AF7_USART2", 6),
        (3, "SERIAL3_FUNCTION", 0, 0, "huart3", "DMA_REQUEST_USART3_RX",
         "USART3_IRQn", "GPIOD", "GPIO_PIN_9", "GPIO_AF7_USART3", 9),
        (4, "SERIAL4_FUNCTION", 115200, 0, "huart4", "DMA_REQUEST_UART4_RX",
         "UART4_IRQn", "GPIOB", "GPIO_PIN_8", "GPIO_AF8_UART4", 8),
        (5, "SERIAL5_FUNCTION", 115200, 0, "huart5", "DMA_REQUEST_UART5_RX",
         "UART5_IRQn", "GPIOB", "GPIO_PIN_12", "GPIO_AF14_UART5", 12),
        (6, "SERIAL6_FUNCTION", 0, 1, "huart6", "DMA_REQUEST_USART6_RX",
         "USART6_IRQn", "GPIOC", "GPIO_PIN_7", "GPIO_AF7_USART6", 7),
        (7, "SERIAL7_FUNCTION", 57600, 0, "huart7", "DMA_REQUEST_UART7_RX",
         "UART7_IRQn", "GPIOE", "GPIO_PIN_7", "GPIO_AF7_UART7", 7),
        (8, "SERIAL8_FUNCTION", 115200, 0, "huart8", "DMA_REQUEST_UART8_RX",
         "UART8_IRQn", "GPIOE", "GPIO_PIN_0", "GPIO_AF8_UART8", 0),
    ]
    actual_runtime = [
        (port.get("serial"), port.get("function_parameter"),
         port.get("default_baud"), port.get("default_function"),
         port.get("uart_handle"), port.get("dma_request"), port.get("irq"),
         port.get("rx_gpio"), port.get("rx_pin"), port.get("rx_af"),
         port.get("rx_index"))
        for port in ports[1:] if isinstance(port, dict)
    ] if isinstance(ports, list) else []
    if actual_runtime != expected_runtime:
        violations.append(Violation(
            path, 1, "R196",
            "serial parameter defaults or STM32 handle/DMA/IRQ/AF mapping changed",
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
    ]
    if (manifest.get("default_rc_port") != 6 or
            manifest.get("functions") != expected_functions):
        violations.append(Violation(
            path, 1, "R196",
            "SERIAL6 must be the sole default RC Input function",
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
                    port.get("function_parameter") !=
                    f"SERIAL{serial}_FUNCTION"):
                violations.append(Violation(
                    path, 1, "R196",
                    "each external port needs matching SERIALx_BAUD/FUNCTION names",
                ))
                break

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
            ("board_serial_config.hpp", "R196",
             "serial generator must emit the runtime mapping"),
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
        match = re.search(r"\bSER_(?:TEL[1-4]|GPS[1-3]|RC)_BAUD\b", text)
        if match:
            violations.append(Violation(
                source, line_for(text, match.group(0)), "R196",
                "serial identity must use SERIAL1..SERIAL8, not assigned function names",
            ))

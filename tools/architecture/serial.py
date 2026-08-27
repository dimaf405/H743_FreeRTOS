"""板级串口 manifest、生成输出与 CubeMX 硬件声明一致性门禁。"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import re

from architecture.common import ROOT, Violation, line_for, require_literals


def _scan_generated_serial_contract(
    manifest_path: pathlib.Path,
    violations: list[Violation],
) -> tuple[dict, list[dict]] | None:
    """调用权威生成器并逐字节比对三个派生输出，不复制端口或参数表。"""
    generator_path = ROOT / "tools/serial/generate_config.py"
    spec = importlib.util.spec_from_file_location(
        "dima_serial_contract_generator", generator_path
    )
    if spec is None or spec.loader is None:
        violations.append(Violation(
            generator_path, 1, "R342",
            "serial generator cannot be loaded for contract verification",
        ))
        return None
    try:
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        manifest, ports = generator.load_manifest(manifest_path)
        expected_outputs = {
            ROOT / "build/generated/serial/serial_baud_params.c":
                generator.generate_serial_parameters(manifest, ports),
            ROOT / "build/generated/serial/SerialContract.hpp":
                generator.generate_serial_contract(manifest, ports),
            ROOT / "build/generated/stm32h7/serial/BoardUartResources.hpp":
                generator.generate_uart_resources(ports),
        }
    except Exception as error:  # 门禁应报告 manifest/生成器错误，而不是回溯退出。
        violations.append(Violation(
            generator_path, 1, "R342",
            f"serial generator contract verification failed: {error}",
        ))
        return None

    for output_path, expected in expected_outputs.items():
        try:
            actual = output_path.read_text(encoding="utf-8")
        except OSError as error:
            violations.append(Violation(
                output_path, 1, "R342",
                f"generated serial output is unavailable: {error}",
            ))
            continue
        if actual != expected:
            violations.append(Violation(
                output_path, 1, "R342",
                "generated serial output differs from serial_ports.json",
            ))
    return manifest, ports


def scan_board_serial_manifest(violations: list[Violation]) -> None:
    """保留格式/唯一性、manifest 到生成物以及 manifest 到 IOC 的一致性。"""
    path = ROOT / "Boards/H743/serial_ports.json"
    try:
        raw_manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        violations.append(Violation(
            path, 1, "R196", f"invalid board serial manifest: {error}"
        ))
        return
    if not isinstance(raw_manifest, dict) or raw_manifest.get("format_version") != 1:
        violations.append(Violation(
            path, 1, "R196", "board serial manifest must use format version 1",
        ))
        return

    generated = _scan_generated_serial_contract(path, violations)
    if generated is None:
        return
    manifest, external_ports = generated

    gps_parameter = manifest.get("gps_port_parameter", {})
    parameter_names = [gps_parameter.get("name")]
    parameter_names.extend(
        name
        for port in external_ports
        for name in (port.get("parameter"), port.get("function_parameter"))
    )
    resource_fields = ("serial", "peripheral", "uart_handle", "irq")
    resources_unique = all(
        len(values) == len(set(values))
        for values in (
            [port.get(field) for port in external_ports]
            for field in resource_fields
        )
    )
    if (any(not isinstance(name, str) or not name for name in parameter_names) or
            len(parameter_names) != len(set(parameter_names)) or
            not resources_unique):
        violations.append(Violation(
            path, 1, "R196",
            "serial parameter names and hardware resource owners must be unique",
        ))

    hardware_reference = manifest.get("hardware_reference", {})
    ioc_source = hardware_reference.get("source")
    if not isinstance(ioc_source, str) or not ioc_source:
        violations.append(Violation(
            path, 1, "R196", "serial manifest lacks its CubeMX IOC source",
        ))
        return
    ioc_path = ROOT / ioc_source
    try:
        ioc_text = ioc_path.read_text(encoding="utf-8")
        usart_header_path = ROOT / "Core/Inc/usart.h"
        usart_header = usart_header_path.read_text(encoding="utf-8")
    except OSError as error:
        violations.append(Violation(
            ioc_path, 1, "R196",
            f"serial hardware declaration is unavailable: {error}",
        ))
        return

    for port in external_ports:
        peripheral = port.get("peripheral")
        tx = port.get("tx")
        rx = port.get("rx")
        handle = port.get("uart_handle")
        if not all(isinstance(value, str) and value
                   for value in (peripheral, tx, rx, handle)):
            continue
        for token in (f"{tx}.Signal={peripheral}_TX",
                      f"{rx}.Signal={peripheral}_RX"):
            if token not in ioc_text:
                violations.append(Violation(
                    ioc_path, 1, "R196",
                    f"serial manifest pin is absent from CubeMX IOC: {token}",
                ))
        if f"extern UART_HandleTypeDef {handle};" not in usart_header:
            violations.append(Violation(
                usart_header_path, 1, "R196",
                f"serial manifest UART handle is absent: {handle}",
            ))

    generated_parameter_names = set(parameter_names)
    definition_re = re.compile(
        r"\bPARAM_DEFINE_[A-Z0-9_]+\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)"
    )
    definitions_root = ROOT / "Dima/middleware/parameters/definitions"
    for source in sorted(definitions_root.glob("*.c")):
        text = source.read_text(encoding="utf-8")
        for definition in definition_re.finditer(text):
            if definition.group(1) not in generated_parameter_names:
                continue
            violations.append(Violation(
                source, line_for(text, definition.group(0)), "R342",
                "serial parameters must be generated from serial_ports.json",
            ))

    require_literals(
        ROOT / "make/project.mk",
        (
            ("SERIAL_CONFIG_GENERATOR := tools/serial/generate_config.py",
             "R342", "build must invoke the serial generator"),
            ("SERIAL_PORT_MANIFEST := Boards/H743/serial_ports.json",
             "R342", "build must use the board serial manifest"),
            ("$(SERIAL_GENERATED_OUTPUTS)", "R342",
             "build must consume generated serial outputs"),
            ("$(SERIAL_BAUD_PARAMETERS)", "R342",
             "parameter generation must consume serial definitions"),
        ),
        violations,
    )

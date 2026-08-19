#!/usr/bin/env python3
"""Stable CLI for the modular Dima architecture checks."""

from __future__ import annotations

from architecture.actuator import scan_active_actuator_contract
from architecture.common import Violation, first_party_sources
from architecture.dependency import (
    scan_build_isolation,
    scan_hardware_ownership,
    scan_include_directions,
    scan_include_style,
    scan_layer_dependencies,
    scan_namespace_convention,
    scan_usb_console_owner,
)
from architecture.layout import (
    scan_debug_console_contract,
    scan_phase5_message_contracts,
    scan_repository_layout,
    scan_rover_root_contract,
)
from architecture.parameter_mavlink import scan_mavlink_contract
from architecture.runtime_safety import (
    scan_clock_contract,
    scan_fault_ownership,
    scan_linker_contract,
    scan_runtime_contracts,
)
from architecture.serial import scan_board_serial_manifest


def main() -> int:
    violations: list[Violation] = []
    scan_include_directions(violations)
    scan_layer_dependencies(violations)
    scan_hardware_ownership(violations)
    scan_build_isolation(violations)
    scan_repository_layout(violations)
    scan_rover_root_contract(violations)
    scan_debug_console_contract(violations)
    scan_phase5_message_contracts(violations)
    scan_board_serial_manifest(violations)
    scan_runtime_contracts(violations)
    scan_fault_ownership(violations)
    scan_clock_contract(violations)
    scan_active_actuator_contract(violations)
    scan_linker_contract(violations)
    scan_include_style(violations)
    scan_namespace_convention(violations)
    scan_usb_console_owner(violations)
    scan_mavlink_contract(violations)
    if violations:
        for violation in sorted(
                violations,
                key=lambda item: (item.path.as_posix(), item.line, item.rule)):
            print(violation.render())
        print(f"architecture check: FAIL ({len(violations)} violations)")
        return 1
    source_count = len(first_party_sources())
    print(f"architecture check: PASS ({source_count} first-party source files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""DroneCAN DSDL 权威输入与构建消费边界门禁。"""

from __future__ import annotations

import pathlib
import re

from architecture.common import ROOT, Violation, line_for, require_literals


def scan_dronecan_contract(violations: list[Violation]) -> None:
    """核对 DSDL 自动发现、生成源码目录及 Make 消费边界。"""
    dsdl_root = (
        ROOT / "Middlewares/Third_Party/dronecan_dsdl/dsdl/uavcan"
    )
    dsdl_files = tuple(sorted(
        path for path in dsdl_root.rglob("*.uavcan") if path.is_file()
    )) if dsdl_root.is_dir() else ()
    if not dsdl_files:
        violations.append(Violation(
            dsdl_root, 1, "R349", "DroneCAN DSDL source closure is empty",
        ))
        return

    project_path = ROOT / "make/project.mk"
    require_literals(
        project_path,
        (
            ("DRONECAN_CONTRACT_GENERATOR := tools/dronecan/generate_contract.py",
             "R349", "build must invoke the DroneCAN generator"),
            ("DRONECAN_DSDL_ROOT := Middlewares/Third_Party/dronecan_dsdl/dsdl/uavcan",
             "R349", "build must identify the DSDL source root"),
            ("--dsdl-root $(DRONECAN_DSDL_ROOT)",
             "R349", "generator must discover inputs from the DSDL source root"),
            ("--print-runtime-sources", "R349",
             "first-party DroneCAN sources must be discovered by the generator"),
            ("include $(DRONECAN_GENERATED_MAKEFILE)", "R349",
             "build must consume the generated source fragment"),
            ("$(DIMA_DRONECAN_GENERATED_C_SOURCES)", "R349",
             "DSDL C sources must come from the generated source fragment"),
            ("$(sort $(wildcard $(PARAMETER_DEFINITION_DIR)/module_*.yaml))",
             "R349", "parameter YAML inputs must be discovered, not listed"),
        ),
        violations,
    )
    project_text = project_path.read_text(encoding="utf-8")
    handwritten_dsdl_source = re.search(
        r"Middlewares/Third_Party/dronecan_dsdl/(?:include|src)|"
        r"uavcan\.(?:protocol|equipment)\.[^\s\\]+\.[ch]",
        project_text,
    )
    if handwritten_dsdl_source:
        violations.append(Violation(
            project_path,
            line_for(project_text, handwritten_dsdl_source.group(0)),
            "R349", "Make must not handwrite DroneCAN generated message files",
        ))

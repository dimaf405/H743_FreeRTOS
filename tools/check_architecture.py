#!/usr/bin/env python3
"""Dima 模块化架构门禁的稳定 CLI；集中汇总全部规则后一次性决定 PASS/FAIL。"""

from __future__ import annotations

import argparse
import pathlib
import re

from architecture_cache import (
    architecture_identity,
    invalidate_stamp,
    update_stamp,
)
from architecture.actuator import scan_active_actuator_contract
from architecture.common import (
    COMMON_INCLUDE_ROOTS,
    ROOT,
    Violation,
    first_party_sources,
)
from architecture.dependency import (
    scan_build_isolation,
    scan_device_policy_boundaries,
    scan_hardware_ownership,
    scan_include_directions,
    scan_layer_dependencies,
    scan_namespace_convention,
    scan_usb_console_owner,
)
from architecture.dronecan import scan_dronecan_contract
from architecture.flashfs import scan_flashfs_contract
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
from architecture.timer import scan_timer_contract


INCLUDE_WITH_DELIMITER_RE = re.compile(
    r'^\s*#\s*include\s*([<"])([^>"]+)[>"]'
)
PROJECT_TREE_ROOTS = (
    "Dima",
    "Boards/H743",
    "Core",
    "USB_DEVICE",
    "Bootloader",
    "tests",
)
PROJECT_INCLUDE_SOURCE_SUFFIXES = frozenset({
    ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inl",
    ".s",
})
NON_OWNED_SOURCE_PATH_PARTS = {
    "c_library_v2",
    "third_party",
    "third-party",
    "vendor",
}
NON_OWNED_TARGET_PATH_PARTS = NON_OWNED_SOURCE_PATH_PARTS | {
    "cubemx",
    "generated",
}
PROJECT_OWNERSHIP_SEARCH_ROOTS = (
    *COMMON_INCLUDE_ROOTS,
    "Dima",
    "Boards/H743",
    "Core",
    "USB_DEVICE",
    "Bootloader",
    "tests",
    ".",
)


def _project_tree_relative(path: pathlib.Path) -> pathlib.Path | None:
    """把候选路径归一到受管工程树；仓库外和非产品目录不参与第一方判定。"""
    try:
        relative = path.resolve().relative_to(ROOT)
    except ValueError:
        return None
    parts = relative.parts
    if not parts:
        return None
    in_project_tree = (
        parts[0] in {
            "Dima", "Core", "USB_DEVICE", "Bootloader", "tests",
        }
        or parts[:2] == ("Boards", "H743")
    )
    return relative if in_project_tree else None


def _is_scanned_project_source(path: pathlib.Path) -> bool:
    """纳入板级生成源码，但排除嵌入式 vendor 子树，防止把上游头误判为第一方。"""
    relative = _project_tree_relative(path)
    if relative is None:
        return False
    lowered_parts = {part.lower() for part in relative.parts}
    return lowered_parts.isdisjoint(NON_OWNED_SOURCE_PATH_PARTS)


def _project_include_sources() -> tuple[pathlib.Path, ...]:
    """枚举 include 深度门禁的源码输入；只按后缀选文件，不维护具体文件清单。"""
    sources: set[pathlib.Path] = set()
    for relative_root in PROJECT_TREE_ROOTS:
        root = ROOT / relative_root
        if not root.is_dir():
            continue
        sources.update(
            path for path in root.rglob("*")
            if path.is_file()
            and path.suffix.lower() in PROJECT_INCLUDE_SOURCE_SUFFIXES
        )
    return tuple(sorted(sources))


def _is_project_owned_target(path: pathlib.Path) -> bool:
    """按解析后的真实目标判定第一方所有权，而不是按 include 字符串拼写猜测。"""
    relative = _project_tree_relative(path)
    if relative is None:
        return False
    parts = relative.parts
    lowered_parts = {part.lower() for part in parts}
    return lowered_parts.isdisjoint(NON_OWNED_TARGET_PATH_PARTS)


def _owned_include_candidates(source: pathlib.Path, include: str,
                              quoted: bool) -> tuple[pathlib.Path, ...]:
    """按真实编译搜索域解析全部第一方候选；同名歧义保留给深度门禁显式报告。"""
    normalized = pathlib.PurePosixPath(include.replace("\\", "/"))
    search_roots = tuple(
        (ROOT / relative).resolve()
        for relative in PROJECT_OWNERSHIP_SEARCH_ROOTS
    )
    if quoted:
        search_roots = (source.parent.resolve(), *search_roots)
    candidates: list[pathlib.Path] = []
    for include_root in search_roots:
        candidate = include_root.joinpath(*normalized.parts).resolve()
        if (candidate.is_file() and _is_project_owned_target(candidate)
                and candidate not in candidates):
            candidates.append(candidate)
    return tuple(candidates)


def scan_first_party_include_depth(violations: list[Violation]) -> None:
    """R302：第一方 include 最多跨一个 domain，防止路径深度固化内部目录布局。"""
    for path in _project_include_sources():
        if not _is_scanned_project_source(path):
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_WITH_DELIMITER_RE.match(line)
            if match is None:
                continue
            quoted = match.group(1) == '"'
            include = match.group(2).replace("\\", "/")
            if len(pathlib.PurePosixPath(include).parts) < 3:
                continue
            targets = _owned_include_candidates(path, include, quoted)
            if not targets:
                continue
            target_list = ", ".join(
                target.relative_to(ROOT).as_posix() for target in targets
            )
            violations.append(Violation(
                path, line_number, "R302",
                f"first-party include '{include}' matches '{target_list}' "
                "and exceeds one domain; use X.hpp or "
                "domain/X.hpp",
            ))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stamp",
        type=pathlib.Path,
        help="atomically record the checked architecture-input identity",
    )
    arguments = parser.parse_args()
    checked_identity = None
    if arguments.stamp is not None:
        # 强制检查开始前先删除旧成功 stamp；失败或中断绝不能留下可复用的历史通过证据。
        checked_identity = architecture_identity(ROOT)
        invalidate_stamp(arguments.stamp)

    violations: list[Violation] = []
    scan_include_directions(violations)
    scan_layer_dependencies(violations)
    scan_hardware_ownership(violations)
    scan_device_policy_boundaries(violations)
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
    scan_flashfs_contract(violations)
    scan_timer_contract(violations)
    scan_dronecan_contract(violations)
    scan_linker_contract(violations)
    scan_first_party_include_depth(violations)
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
    if checked_identity is not None:
        final_identity = architecture_identity(ROOT)
        # 扫描期间输入发生变化时返回 RETRY，不把混合两个工作区时刻的结果标为通过。
        if final_identity != checked_identity:
            print(
                "architecture check: RETRY "
                "(inputs changed while the check was running)"
            )
            return 2
    print(f"architecture check: PASS ({source_count} first-party source files)")
    if arguments.stamp is not None:
        update_stamp(arguments.stamp, ROOT, checked_identity)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

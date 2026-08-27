"""仓库源码闭包、纯算法边界与消息 schema/生成物边界门禁。"""

from __future__ import annotations

from architecture.build_closure import BuildClosureError, load_build_closure
from architecture.common import (
    ROOT,
    Violation,
    line_for,
    require_literals,
    sources_under,
)


def scan_rover_root_contract(violations: list[Violation]) -> None:
    """纯 Rover 算法库不得拥有运行时、uORB 或参数中间件状态。"""
    runtime_tokens = (
        "ModuleBase", "ScheduledWorkItem", "uORB::", "px4::Param",
        "param_find", "param_get", "param_set",
    )
    for path in sources_under(("Dima/lib/rover",)):
        text = path.read_text(encoding="utf-8")
        for token in runtime_tokens:
            if token in text:
                violations.append(Violation(
                    path, line_for(text, token), "R214",
                    "Rover algorithm library owns runtime or middleware state",
                ))


def scan_repository_layout(violations: list[Violation]) -> None:
    """从真实 Make 数据库核对第一方源码闭包及闭包中的缺失输入。"""
    project_make = ROOT / "make/project.mk"
    if not project_make.is_file():
        return
    make_text = project_make.read_text(encoding="utf-8")
    try:
        closure = load_build_closure(ROOT)
    except BuildClosureError as error:
        violations.append(Violation(
            project_make, 1, "R225",
            f"cannot evaluate the real Make build closure: {error}",
        ))
        return

    compiled_sources = closure.sources
    parameter_sources = closure.parameter_generator_inputs
    dima_root = ROOT / "Dima"
    first_party_sources = sorted(
        path
        for source_root in (dima_root, ROOT / "Boards/H743")
        for path in source_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".c", ".cpp"}
    )
    for path in first_party_sources:
        relative = path.relative_to(ROOT).as_posix()
        is_parameter_input = path.is_relative_to(
            ROOT / "Dima/middleware/parameters/definitions"
        )
        expected_sources = (
            parameter_sources if is_parameter_input else compiled_sources
        )
        if relative not in expected_sources:
            violations.append(Violation(
                path, 1, "R225",
                "first-party source is outside its compiled or generator "
                "input closure",
            ))

    for relative in sorted(compiled_sources | parameter_sources):
        if not relative.startswith(("Dima/", "Boards/H743/")):
            continue
        source_path = ROOT / relative
        if source_path.is_file():
            continue
        boot_make = ROOT / "Bootloader/Makefile"
        owner = project_make
        owner_text = make_text
        if relative not in owner_text and boot_make.is_file():
            owner = boot_make
            owner_text = boot_make.read_text(encoding="utf-8")
        violations.append(Violation(
            owner, line_for(owner_text, relative), "R226",
            f"evaluated build closure references missing {relative}",
        ))


def scan_phase5_message_contracts(violations: list[Violation]) -> None:
    """只保留 schema 权威输入及派生 Topic 源不得手写的生成边界。"""
    for legacy in sorted((ROOT / "Dima/messages").glob("*.[ch]pp")):
        violations.append(Violation(
            legacy, 1, "R340",
            "hand-written uORB Topic contract bypasses schema generation",
        ))

    require_literals(
        ROOT / "make/project.mk",
        (
            ("MESSAGE_GENERATOR := tools/uorb/generate_messages.py", "R341",
             "uORB schema generator is not part of the build contract"),
            ("include $(MESSAGE_GENERATED_MAKEFILE)", "R341",
             "generated uORB source manifest is not included"),
            ("$(DIMA_UORB_GENERATED_SOURCES)", "R341",
             "official generated uORB sources are not built"),
        ),
        violations,
    )

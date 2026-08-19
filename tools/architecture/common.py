#!/usr/bin/env python3
"""Shared paths, patterns, and helpers for Dima architecture checks."""

from __future__ import annotations

import functools
import pathlib
import re
from collections.abc import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[2]
SOURCE_SUFFIXES = {".c", ".h", ".cpp", ".hpp"}
C_COMMENT_RE = re.compile(
    r'("(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|'
    r'(/\*.*?\*/|//[^\r\n]*)',
    re.DOTALL,
)
PROTECTED_ROOTS = (
    "Dima/application",
    "Dima/rover",
    "Dima/modules",
    "Dima/middleware",
    "Dima/messages",
    "Dima/lib",
    "Dima/adapters",
    "Dima/platform/api",
)
FREERTOS_ROOT = "Dima/platform/freertos"
STM32_ROOT = "Dima/platform/stm32h7"
COMMON_INCLUDE_ROOTS = (
    "Dima",
    "Dima/application",
    "Dima/rover",
    "Dima/modules",
    "Dima/middleware",
    "Dima/messages",
    "Dima/lib",
    "Dima/adapters",
)
LAYER_ROOTS = (
    ("platform/freertos", ROOT / "Dima/platform/freertos"),
    ("platform/stm32h7", ROOT / "Dima/platform/stm32h7"),
    ("platform/api", ROOT / "Dima/platform/api"),
    ("application", ROOT / "Dima/application"),
    ("rover", ROOT / "Dima/rover"),
    ("modules", ROOT / "Dima/modules"),
    ("middleware", ROOT / "Dima/middleware"),
    ("messages", ROOT / "Dima/messages"),
    ("lib", ROOT / "Dima/lib"),
    ("adapters", ROOT / "Dima/adapters"),
)
ALLOWED_LAYER_DEPENDENCIES = {
    "platform/freertos": {"platform/freertos", "platform/api"},
    "platform/stm32h7": {"platform/stm32h7", "platform/api"},
    "platform/api": {"platform/api"},
    "lib": {"lib", "platform/api"},
    "middleware": {"middleware", "lib", "platform/api"},
    "messages": {"messages", "middleware", "lib", "platform/api"},
    "modules": {
        "modules", "messages", "middleware", "lib", "platform/api",
    },
    "adapters": {"adapters", "platform/api"},
    "rover": {
        "rover", "modules", "messages", "middleware", "lib", "adapters",
        "platform/api",
    },
    "application": {
        "application", "rover", "modules", "messages", "middleware", "lib",
        "adapters", "platform/api",
    },
}

INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
PROTECTED_API_RE = re.compile(
    r"\b(?:xTask\w*|vTask\w*|uxTask\w*|xSemaphore\w*|vSemaphore\w*|"
    r"task(?:ENTER|EXIT|YIELD)\w*|portMAX_DELAY|TickType_t|TaskHandle_t|"
    r"SemaphoreHandle_t|StaticTask_t|StaticSemaphore_t|BaseType_t|"
    r"UBaseType_t|osKernel\w*|HAL_\w+|__HAL_\w+|LL_\w+|NVIC_\w+|"
    r"__disable_irq|__enable_irq|__get_\w+|irqstate_t|"
    r"enter_critical_section|leave_critical_section)\b|\bSCB(?:->|_)\w+"
)
FREERTOS_VENDOR_API_RE = re.compile(
    r"\b(?:HAL_\w+|__HAL_\w+|LL_\w+|NVIC_\w+|SystemCoreClock)\b|"
    r"\b(?:SCB|RCC|FLASH|MPU)(?:->|_)\w+"
)
RTOS_API_RE = re.compile(
    r"\b(?:xTask\w*|vTask\w*|uxTask\w*|xSemaphore\w*|vSemaphore\w*|"
    r"task(?:ENTER|EXIT|YIELD)\w*|portMAX_DELAY|TickType_t|TaskHandle_t|"
    r"SemaphoreHandle_t|StaticTask_t|StaticSemaphore_t|BaseType_t|"
    r"UBaseType_t|osKernel\w*)\b"
)
CACHE_OPERATION_RE = re.compile(
    r"\bSCB_(?:Enable|Disable|Clean|Invalidate|CleanInvalidate)"
    r"\w*Cache\w*\s*\("
)
HAL_DMA_RE = re.compile(r"\bHAL_\w*DMA\w*\s*\(")
HAL_FLASH_RE = re.compile(r"\b(?:HAL_FLASH\w*|__HAL_FLASH\w*)\s*\(")
NONZERO_PWM_PULSE_RE = re.compile(
    r"^TIM(?:5|8)\.Pulse-PWM.*=(?!0$)(.+)$"
)

BOARD_HEADERS = {
    "board_init.h",
    "boot_diagnostics.h",
    "boot_diagnostics_store.h",
    "boot_layout.h",
    "dima_boot_request.h",
    "motor_pwm.h",
    "platform_composition.h",
}
CORE_GENERATED_HEADERS = {
    "dma.h",
    "fdcan.h",
    "gpio.h",
    "i2c.h",
    "main.h",
    "sdmmc.h",
    "spi.h",
    "tim.h",
    "usart.h",
}
MAKE_CONTRACT_PATHS = (
    ROOT / "make/project.mk",
    ROOT / "make/release.mk",
)


class Violation:
    def __init__(self, path: pathlib.Path, line: int, rule: str,
                 message: str) -> None:
        self.path = path
        self.line = line
        self.rule = rule
        self.message = message

    def render(self) -> str:
        relative = self.path.relative_to(ROOT).as_posix()
        return f"{relative}:{self.line}: {self.rule} {self.message}"


def strip_c_comments(text: str) -> str:
    """Remove C/C++ comments while preserving strings and line numbers."""
    def replace(match: re.Match[str]) -> str:
        if match.group(1) is not None:
            return match.group(1)
        return "\n" * match.group(2).count("\n")

    return C_COMMENT_RE.sub(replace, text)


@functools.lru_cache(maxsize=None)
def source_files_under(relative: str) -> tuple[pathlib.Path, ...]:
    base = ROOT / relative
    if not base.exists():
        return ()
    return tuple(
        sorted(
            path
            for path in base.rglob("*")
            if path.is_file()
            and path.suffix.lower() in SOURCE_SUFFIXES
            and not is_vendored(path)
        )
    )


def sources_under(relative_roots: Iterable[str]) -> list[pathlib.Path]:
    files = {
        path
        for relative in relative_roots
        for path in source_files_under(relative)
    }
    return sorted(files)


def is_vendored(path: pathlib.Path) -> bool:
    """True for upstream-generated sources kept verbatim (c_library_v2)."""
    try:
        relative = path.relative_to(ROOT)
    except ValueError:
        return False
    return "c_library_v2" in relative.parts


def first_party_sources() -> list[pathlib.Path]:
    roots = ("Dima", "Boards", "Core", "Bootloader", "USB_DEVICE")
    return sources_under(roots)


def low_level_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "freertos" in lowered
        or "nuttx" in lowered
        or "cmsis" in lowered
        or "stm32" in lowered
        or lowered.startswith(("core/", "boards/", "usb_device/",
                               "drivers/", "middlewares/"))
        or lowered.startswith(("platform/freertos/",
                               "platform/stm32h7/"))
        or basename in BOARD_HEADERS
        or basename in CORE_GENERATED_HEADERS
        or basename.startswith(("usb_device", "usbd_", "core_cm"))
    )


def vendor_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "stm32" in lowered
        or "cmsis" in lowered
        or lowered.startswith(("core/", "boards/", "usb_device/",
                               "drivers/"))
        or lowered.startswith("platform/stm32h7/")
        or basename in BOARD_HEADERS
        or basename in CORE_GENERATED_HEADERS
        or basename.startswith(("usb_device", "usbd_", "core_cm"))
    )


def freertos_include(include: str) -> bool:
    lowered = include.lower()
    basename = pathlib.PurePosixPath(lowered).name
    return (
        "freertos" in lowered
        or lowered.startswith("platform/freertos/")
        or basename in {
            "event_groups.h", "portable.h", "queue.h", "semphr.h",
            "stream_buffer.h", "task.h", "timers.h",
        }
        or basename.startswith("cmsis_os")
    )


def protected_layer(path: pathlib.Path) -> str | None:
    for name, root in LAYER_ROOTS:
        if path.is_relative_to(root):
            return name
    return None


def resolve_common_include(source: pathlib.Path,
                           include: str) -> pathlib.Path | None:
    candidates = [source.parent / include]
    candidates.extend(ROOT / root / include for root in COMMON_INCLUDE_ROOTS)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None

def variable_block(lines: list[str], name: str) -> list[tuple[int, str]]:
    start_re = re.compile(rf"^\s*{re.escape(name)}\s*:?=")
    for index, line in enumerate(lines):
        if not start_re.match(line):
            continue
        block = [(index + 1, line)]
        while block[-1][1].rstrip().endswith("\\"):
            next_index = index + len(block)
            if next_index >= len(lines):
                break
            block.append((next_index + 1, lines[next_index]))
        return block
    return []


def line_for(text: str, needle: str) -> int:
    offset = text.find(needle)
    return 1 if offset < 0 else text.count("\n", 0, offset) + 1


def require_literals(path: pathlib.Path,
                     requirements: Iterable[tuple[str, str, str]],
                     violations: list[Violation]) -> None:
    if not path.is_file():
        violations.append(Violation(
            path, 1, "R040", "required architecture source is missing",
        ))
        return
    text = path.read_text(encoding="utf-8")
    for literal, rule, message in requirements:
        if literal not in text:
            violations.append(Violation(path, 1, rule, message))


def owner_texts(
        paths: Iterable[pathlib.Path]) -> tuple[tuple[pathlib.Path, str], ...]:
    """Read the existing files from one explicit logical-owner set."""
    return tuple(
        (path, path.read_text(encoding="utf-8"))
        for path in paths
        if path.is_file()
    )


def find_literal_owner(
        paths: Iterable[pathlib.Path],
        literal: str) -> tuple[pathlib.Path, str] | None:
    """Return the first explicit owner containing a literal contract."""
    for path, text in owner_texts(paths):
        if literal in text:
            return path, text
    return None


def require_literals_in_owners(
        paths: tuple[pathlib.Path, ...],
        requirements: Iterable[tuple[str, str, str]],
        violations: list[Violation]) -> None:
    """Require each literal in at least one file of a logical-owner set."""
    existing = owner_texts(paths)
    if not existing:
        require_literals(paths[0], requirements, violations)
        return
    missing = tuple(
        (literal, rule, message)
        for literal, rule, message in requirements
        if not any(literal in text for _path, text in existing)
    )
    if missing:
        require_literals(existing[0][0], missing, violations)

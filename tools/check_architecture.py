#!/usr/bin/env python3
"""Enforce the Dima platform dependency and hardware-access boundaries."""

from __future__ import annotations

import functools
import hashlib
import json
import pathlib
import re
from collections.abc import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[1]
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


def scan_include_directions(violations: list[Violation]) -> None:
    roots = PROTECTED_ROOTS + (FREERTOS_ROOT, STM32_ROOT)
    for path in sources_under(roots):
        source_layer = protected_layer(path)
        if source_layer is None:
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            target = resolve_common_include(path, match.group(1))
            target_layer = protected_layer(target) if target else None
            if (target_layer is None or target_layer in
                    ALLOWED_LAYER_DEPENDENCIES[source_layer]):
                continue
            target_relative = target.relative_to(ROOT).as_posix()
            violations.append(Violation(
                path, line_number, "R004",
                f"{source_layer} layer includes disallowed "
                f"{target_layer} header '{target_relative}'",
            ))


def scan_layer_dependencies(violations: list[Violation]) -> None:
    for path in sources_under(PROTECTED_ROOTS):
        is_api = path.is_relative_to(ROOT / "Dima/platform/api")
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if match and low_level_include(match.group(1)):
                violations.append(Violation(
                    path, line_number, "R001",
                    f"protected layer includes low-level header "
                    f"'{match.group(1)}'",
                ))
            if PROTECTED_API_RE.search(line):
                violations.append(Violation(
                    path, line_number, "R002",
                    "protected layer uses a low-level API identifier",
                ))
            if is_api and match:
                include = match.group(1)
                allowed = {
                    "cstddef", "cstdint", "stdbool.h", "stddef.h", "stdint.h",
                    "Platform.hpp", "Time.hpp", "platform_config.h",
                }
                if include not in allowed and not include.startswith(
                        "platform/api/"):
                    violations.append(Violation(
                        path, line_number, "R003",
                        f"platform/api includes non-contract header '{include}'",
                    ))
    for path in sources_under((FREERTOS_ROOT,)):
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if match and vendor_include(match.group(1)):
                violations.append(Violation(
                    path, line_number, "R010",
                    f"FreeRTOS backend includes MCU header '{match.group(1)}'",
                ))
            if FREERTOS_VENDOR_API_RE.search(line):
                violations.append(Violation(
                    path, line_number, "R011",
                    "FreeRTOS backend uses an MCU/vendor identifier",
                ))

    for path in sources_under((STM32_ROOT,)):
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if match and freertos_include(match.group(1)):
                violations.append(Violation(
                    path, line_number, "R012",
                    f"STM32 backend includes RTOS header '{match.group(1)}'",
                ))
            if RTOS_API_RE.search(line):
                violations.append(Violation(
                    path, line_number, "R013",
                    "STM32 backend uses a FreeRTOS API identifier",
                ))


def scan_hardware_ownership(violations: list[Violation]) -> None:
    cache_owners = {
        "Dima/platform/stm32h7/cache.c",
        "Dima/platform/stm32h7/early_memory.c",
    }
    dma_owners = {
        "Dima/platform/stm32h7/SbusUart.cpp",
    }
    flash_owners = {
        "Dima/platform/stm32h7/FlashDevice.cpp",
        "Dima/platform/stm32h7/flash_bank1.c",
        "Bootloader/Src/flash_map_backend.c",
    }

    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if CACHE_OPERATION_RE.search(line) and relative not in cache_owners:
                violations.append(Violation(
                    path, line_number, "R020",
                    "raw SCB cache operation is outside the central cache owner",
                ))
            if (HAL_DMA_RE.search(line) and relative not in dma_owners and
                    not relative.startswith("Core/Src/")):
                violations.append(Violation(
                    path, line_number, "R021",
                    "raw HAL DMA operation is outside an MCU DMA driver",
                ))
            if HAL_FLASH_RE.search(line) and relative not in flash_owners:
                violations.append(Violation(
                    path, line_number, "R022",
                    "raw HAL Flash operation is outside a Flash driver",
                ))


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


def scan_rover_root_contract(violations: list[Violation]) -> None:
    rover_root = ROOT / "Dima/rover"
    legacy_root = ROOT / "Dima/modules/rover"
    if not rover_root.is_dir():
        violations.append(Violation(
            rover_root, 1, "R040", "the unique Rover product root is missing",
        ))
    if legacy_root.exists():
        violations.append(Violation(
            legacy_root, 1, "R041",
            "legacy Dima/modules/rover must not exist",
        ))

    ambiguous_paths = (
        "Dima/modules/rc/ManualControl.cpp",
        "Dima/modules/rc/ManualControl.hpp",
        "Dima/modules/manual_control",
        "Dima/rover/control/ManualMotionAdapter.cpp",
        "Dima/rover/control/ManualMotionAdapter.hpp",
        "Dima/rover/modes/manual",
        "Dima/rover/control/MotorOutput.cpp",
        "Dima/rover/control/MotorOutput.hpp",
        "Dima/rover/control/DifferentialDrive.cpp",
        "Dima/rover/control/DifferentialDrive.hpp",
        "Dima/lib/rover_control/rover_control.cpp",
        "Dima/lib/rover_control/rover_control.hpp",
        "Dima/lib/motor/speed_to_pwm.cpp",
        "Dima/lib/motor/speed_to_pwm.hpp",
    )
    for relative in ambiguous_paths:
        path = ROOT / relative
        if path.exists():
            violations.append(Violation(
                path, 1, "R212",
                "file or directory remains in a retired ambiguous path",
            ))

    required_paths = (
        "Dima/modules/rc/RcManualInput.cpp",
        "Dima/modules/rc/RcManualInput.hpp",
        "Dima/modules/motor/MotorOutput.cpp",
        "Dima/modules/motor/MotorOutput.hpp",
        "Dima/rover/modes/ManualMode.cpp",
        "Dima/rover/modes/ManualMode.hpp",
        "Dima/lib/rover/DifferentialDrive.cpp",
        "Dima/lib/rover/DifferentialDrive.hpp",
    )
    for relative in required_paths:
        path = ROOT / relative
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R212",
                "required source is missing from its unambiguous owner path",
            ))

    roots = ("Dima", "Boards", "Core", "Bootloader", "USB_DEVICE",
             "make", "docs")
    candidates = sources_under(roots)
    for root in roots:
        base = ROOT / root
        if not base.exists():
            continue
        candidates.extend(
            path for path in base.rglob("*.md") if path.is_file()
        )
        candidates.extend(
            path for path in base.rglob("*.mk") if path.is_file()
        )
    candidates.extend((ROOT / "Makefile", ROOT / "GNUmakefile"))
    for path in sorted(set(candidates)):
        if not path.is_file():
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            normalized = line.replace("\\", "/")
            if "Dima/modules/rover" in normalized:
                violations.append(Violation(
                    path, line_number, "R042",
                    "references the removed Dima/modules/rover path",
                ))
            ambiguous_references = (
                "Dima/modules/rc/ManualControl",
                "Dima/modules/manual_control/",
                "Dima/rover/control/ManualMotionAdapter",
                "Dima/rover/modes/manual/",
                "Dima/rover/control/MotorOutput",
                "Dima/rover/control/DifferentialDrive",
                "Dima/lib/rover_control/",
                "Dima/lib/motor/speed_to_pwm",
            )
            if any(token in normalized for token in ambiguous_references):
                violations.append(Violation(
                    path, line_number, "R213",
                    "references a retired ambiguous ownership path",
                ))

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
    expected_ioc = ROOT / "H743_FreeRTOS.ioc"
    ioc_files = list(ROOT.glob("*.ioc"))
    skipped_roots = {
        ".git", ".keys", ".vscode", "Drivers", "Middlewares", "build",
    }
    for child in ROOT.iterdir():
        if (not child.is_dir() or child.name in skipped_roots or
                child.name.startswith("build-")):
            continue
        ioc_files.extend(child.rglob("*.ioc"))
    for path in sorted(ioc_files):
        if path != expected_ioc:
            violations.append(Violation(
                path, 1, "R220",
                "secondary CubeMX project makes the authoritative .ioc "
                "ambiguous",
            ))

    retired_files = (
        ROOT / "newlib_lock_glue.c",
        ROOT / "stm32_lock.h",
        ROOT / "Dima/platform/stm32h7/Backend.hpp",
        ROOT / "Dima/modules/boot_health/boot_health.cpp",
        ROOT / "Dima/modules/boot_health/boot_health.hpp",
    )
    for path in retired_files:
        if path.exists():
            violations.append(Violation(
                path, 1, "R221",
                "retired or misleading source path has returned",
            ))

    dima_root = ROOT / "Dima"
    for directory in sorted(
            path for path in dima_root.rglob("*") if path.is_dir()):
        if is_vendored(directory):
            continue
        files = sorted(path for path in directory.rglob("*") if path.is_file())
        if not files:
            violations.append(Violation(
                directory, 1, "R222",
                "empty directory does not describe an implemented owner",
            ))
        elif all(path.name == "README.md" for path in files):
            violations.append(Violation(
                files[0], 1, "R223",
                "README-only source directory implies an implementation "
                "that does not exist",
            ))

    by_basename: dict[str, list[pathlib.Path]] = {}
    for path in sources_under(("Dima",)):
        if is_vendored(path):
            continue
        by_basename.setdefault(path.name.lower(), []).append(path)
    for paths in by_basename.values():
        if len(paths) < 2:
            continue
        for path in paths:
            violations.append(Violation(
                path, 1, "R224",
                "duplicate Dima source basename makes short includes "
                "ambiguous",
            ))

    project_make = ROOT / "make/project.mk"
    if project_make.is_file():
        text = project_make.read_text(encoding="utf-8")
        dima_sources = sorted(
            path for path in dima_root.rglob("*")
            if path.is_file() and path.suffix.lower() in {".c", ".cpp"}
        )
        for path in dima_sources:
            relative = path.relative_to(ROOT).as_posix()
            if relative not in text:
                violations.append(Violation(
                    path, 1, "R225",
                    "Dima translation unit is absent from make/project.mk",
                ))
        listed = set(re.findall(
            r"Dima/[A-Za-z0-9_./-]+\.(?:cpp|c)\b", text,
        ))
        for relative in sorted(listed):
            path = ROOT / relative
            if not path.is_file():
                violations.append(Violation(
                    project_make, line_for(text, relative), "R226",
                    f"build manifest references missing source {relative}",
                ))

    require_literals(
        ROOT / "Core/Inc/FreeRTOSConfig.h",
        (("#include \"../../Dima/platform/freertos/FreeRTOSConfig.h\"",
          "R227", "CubeMX FreeRTOSConfig shim no longer forwards to the "
          "single Dima configuration"),),
        violations,
    )


def scan_debug_console_contract(violations: list[Violation]) -> None:
    legacy_paths = (
        ROOT / "Dima/modules/hello_world",
        ROOT / "Dima/messages/app_heartbeat.cpp",
        ROOT / "Dima/messages/app_heartbeat.hpp",
        ROOT / "tools/validate_hello_world_interval.py",
    )
    for path in legacy_paths:
        if path.exists():
            violations.append(Violation(
                path, 1, "R043",
                "HelloWorld and app_heartbeat must remain removed",
            ))

    project_make = ROOT / "make/project.mk"
    if project_make.is_file():
        text = project_make.read_text(encoding="utf-8")
        for token in ("APP_HELLO_WORLD", "app_heartbeat", "hello_world"):
            if token in text:
                violations.append(Violation(
                    project_make, line_for(text, token), "R044",
                    f"legacy debug example token '{token}' is still built",
                ))

    require_literals(
        ROOT / "Dima/rover/ApplicationContext.cpp",
        (
            ("dima::events::reset();", "R045",
             "Application Runtime must reset the Event Ring"),
            ("dima::logging::reset();", "R046",
             "Application Runtime must reset the Log Ring"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/modules/logging/LogService.cpp",
        (
            ("dima::events::pop(event)", "R047",
             "USB debug logger must consume structured events"),
            ("kMaxEventsPerRun", "R048",
             "structured event logging must remain bounded"),
            ("Structured logging ready", "R049",
             "structured logger startup record is missing"),
            ("set_structured_sink(nullptr, &LogService::structured_sink)",
             "R049B",
             "structured logger must register the mavlink_log sink"),
            ("mavlink_log_publication_.publish(record)", "R049C",
             "structured logger must publish mavlink_log records"),
            ("enqueue_sbus_data(hrt_absolute_time())", "R189",
             "USB debug logger does not service SBUS data"),
            ("kSbus.data_period_ms", "R190",
             "SBUS USB data output is not rate limited by DebugConfig"),
            ("Source::Sbus, Level::Debug, \"sbus\"", "R191",
             "SBUS data does not use the PX4-style source log path"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/modules/logging/LogService.hpp",
        (
            ("uORB::SubscriptionData<input_rc_s>", "R188",
             "USB debug logger must observe decoded SBUS data through uORB"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/middleware/logging/debug_config.hpp",
        (
            ("kUsbMinimumLevel = Level::Debug", "R184",
             "default USB debug level changed"),
            ("kIcm42688{Level::Off, false, 100U}", "R186",
             "future ICM42688 debug output must default off"),
        ), violations,
    )
    require_literals(
        ROOT / "Dima/middleware/logging/logging.cpp",
        (
            ("if (level == Level::Off || (!raw && !config::enabled(source, level)))", "R187",
             "module logs are not filtered before formatting"),
        ), violations,
    )


def scan_phase5_message_contracts(violations: list[Violation]) -> None:
    requirements = {
        ROOT / "Dima/messages/actuator_motors.hpp": (
            ("MESSAGE_VERSION = 0U", "R140",
             "actuator_motors version contract changed"),
            ("NUM_CONTROLS = 12U", "R141",
             "actuator_motors must retain 12 public controls"),
            ("std::uint16_t reversible_flags", "R142",
             "actuator_motors reversible flags are missing"),
            ("float control[NUM_CONTROLS]", "R143",
             "actuator_motors control array is missing"),
        ),
        ROOT / "Dima/messages/rover_motion_request.hpp": (
            ("SOURCE_MANUAL = 0U", "R144",
             "Manual motion source contract changed"),
            ("SOURCE_NAVIGATION = 1U", "R145",
             "Navigation motion source reservation changed"),
            ("MODE_NORMALIZED_AXES = 0U", "R146",
             "normalized two-axis mode contract changed"),
            ("MODE_SPEED_YAW_RATE = 1U", "R147",
             "navigation speed/yaw-rate mode reservation changed"),
            ("float normalized_longitudinal", "R148",
             "longitudinal motion axis is missing"),
            ("float normalized_steering", "R149",
             "steering motion axis is missing"),
        ),
        ROOT / "Dima/messages/actuator_output_status.hpp": (
            ("NUM_OUTPUTS = 6U", "R150",
             "actuator output status must remain six-channel"),
            ("STATE_HARD_SAFE_OFF = 1U", "R151",
             "hard-safe-off output state is missing"),
            ("STATE_DISARMED_NEUTRAL = 5U", "R151",
             "disarmed-neutral output state is missing"),
            ("configured_output_mask", "R151",
             "configured output mask is missing"),
            ("right_output_mask", "R151",
             "right motor output mask is missing"),
            ("left_output_mask", "R151",
             "left motor output mask is missing"),
            ("STATE_FAULT = 4U", "R152",
             "fault output state is missing"),
            ("std::uint16_t pwm_us[NUM_OUTPUTS]", "R153",
             "per-channel applied PWM status is missing"),
        ),
        ROOT / "Dima/messages/actuator_motors.cpp": (
            ("ORB_DEFINE(actuator_motors, actuator_motors_s, 1U)", "R154",
             "actuator_motors must remain a latest-value Topic"),
        ),
        ROOT / "Dima/messages/rover_motion_request.cpp": (
            ("ORB_DEFINE(rover_motion_request, rover_motion_request_s, 8U)",
             "R155", "motion request queue depth must remain eight"),
        ),
        ROOT / "Dima/messages/actuator_output_status.cpp": (
            ("ORB_DEFINE(actuator_output_status, actuator_output_status_s, 8U)",
             "R156", "output status queue depth must remain eight"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)


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

    if manifest.get("schema_version") != 2:
        violations.append(Violation(
            path, 1, "R196", "board serial manifest must use schema v2",
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
        "Schema v1 USART1 default", "Schema v1 USART2 default",
        "Schema v1 USART3 default", "Schema v1 UART4 default",
        "CubeMX UART5 normal 8N1", "Schema v1 USART6 RC Auto",
        "Schema v1 UART7 default", "Schema v1 UART8 default",
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

    expected_legacy_rc = {
        "0": 0, "1": 4, "2": 7, "3": 8, "4": 2,
        "101": 7, "102": 8, "103": 2, "104": 5,
        "201": 1, "202": 3, "203": 6, "300": 4,
    }
    expected_legacy_baud = {
        "SER_RC_BAUD": 4, "SER_TEL1_BAUD": 7,
        "SER_TEL2_BAUD": 8, "SER_TEL3_BAUD": 2,
        "SER_TEL4_BAUD": 5, "SER_GPS1_BAUD": 1,
        "SER_GPS2_BAUD": 3, "SER_GPS3_BAUD": 6,
    }
    if (manifest.get("legacy_rc_port_map") != expected_legacy_rc or
            manifest.get("legacy_baud_parameter_map") !=
            expected_legacy_baud):
        violations.append(Violation(
            path, 1, "R196",
            "legacy serial migration must preserve physical UART ownership",
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
        if source == ROOT / "Dima/modules/parameters/ParameterService.cpp":
            continue
        text = source.read_text(encoding="utf-8")
        match = re.search(r"\bSER_(?:TEL[1-4]|GPS[1-3]|RC)_BAUD\b", text)
        if match:
            violations.append(Violation(
                source, line_for(text, match.group(0)), "R196",
                "serial identity must use SERIAL1..SERIAL7, not assigned function names",
            ))


def scan_runtime_contracts(violations: list[Violation]) -> None:
    requirements = {
        ROOT / "Dima/middleware/parameters/param.h": (
            ("constexpr Param() noexcept", "R050",
             "Param construction must remain side-effect free"),
            ("bool bind()", "R051", "Param bind contract is missing"),
            ("param_set_used(handle());", "R052",
             "Param bind must register parameter use"),
            ("bool param_shutdown(void)", "R053",
             "Parameter core shutdown declaration is missing"),
        ),
        ROOT / "Dima/middleware/parameters/param.cpp": (
            ("bool param_shutdown(void) noexcept", "R054",
             "Parameter core shutdown implementation is missing"),
            ("g_active.reset();", "R055",
             "Parameter shutdown must invalidate the used cache"),
            ("g_unsaved.reset();", "R056",
             "Parameter shutdown must invalidate the unsaved cache"),
        ),
        ROOT / "Dima/middleware/uorb/uORB.cpp": (
            ("uint64_t g_lifecycle_epoch", "R057",
             "uORB lifecycle epoch storage is missing"),
            ("++g_lifecycle_epoch;", "R058",
             "uORB initialize must advance the lifecycle epoch"),
            ("void shutdown() noexcept", "R059",
             "uORB Runtime shutdown is missing"),
            ("if (newest == 0U)", "R061",
             "uORB must reject empty generation-zero slots"),
            ("generation == 0U || generation > newest", "R062",
             "uORB queued subscriptions must recover from stale generations"),
        ),
        ROOT / "Dima/middleware/uorb/uORB.hpp": (
            ("synchronize_epoch()", "R060",
             "uORB endpoint epoch synchronization is missing"),
        ),
        ROOT / "Dima/middleware/work_queue/WorkQueue.cpp": (
            ("dima::platform::SignalHandle signal", "R063",
             "WorkQueue globals must not own destructed Signal objects"),
            ("g_owner_task", "R064",
             "WorkQueue Runtime owner tracking is missing"),
            ("bool work_queue_shutdown() noexcept", "R065",
             "WorkQueue shutdown contract is missing"),
            ("tasks.destroy(queue.task)", "R066",
             "WorkQueue shutdown must synchronously destroy workers"),
            ("const wq_config_t lp_default{\"wq:lp_default\", 2U, 4096U, false};",
             "R228", "MAVLink/Parameter/Log WorkQueue stack regressed below 4 KiB"),
        ),
        ROOT / "Dima/platform/api/Platform.hpp": (
            ("virtual bool destroy(TaskHandle handle) noexcept", "R067",
             "TaskRuntime destroy must report failure"),
        ),
        ROOT / "Dima/platform/freertos/Backend.cpp": (
            ("native == xTaskGetCurrentTaskHandle()", "R068",
             "TaskRuntime must reject deleting the current task"),
            ("kTaskStackPoolBytes = 48U * 1024U", "R229",
             "fixed task stack pool must retain the 48 KiB budget"),
        ),
        ROOT / "Dima/rover/ApplicationContext.cpp": (
            ("bool ApplicationContext::shutdown() noexcept", "R069",
             "Application Runtime shutdown is missing"),
            ("if (!services_.console.shutdown())", "R070",
             "Application Runtime must release the Console frontend"),
            ("if (!px4::work_queue_shutdown())", "R071",
             "Application Runtime must release WorkQueue"),
            ("sbus_rc_.state() ==", "R217",
             "Application Runtime ignores SBUS UART restore failure"),
            ("serial_config_.state() ==", "R196",
             "Application Runtime ignores serial backend reset failure"),
            ("Board serial configuration invalid; RC input inhibited", "R196",
             "invalid serial parameters must preserve USB/QGC recovery"),
        ),
        ROOT / "Dima/platform/stm32h7/SbusUart.cpp": (
            ("kDmaBufferSize = 64U", "R072",
             "SBUS DMA buffer size contract changed"),
            ("kReceiveRingCapacity = 256U", "R073",
             "SBUS CPU Ring capacity contract changed"),
            ("g_receive_ring", "R074",
             "SBUS CPU-only handoff Ring is missing"),
            ("reset_receive_epoch()", "R075",
             "SBUS receive epoch reset is missing"),
            ("configure_sbus_rx_pin", "R180",
             "SBUS RX pin configuration is missing"),
            ("GPIO_PULLDOWN", "R181",
             "inverted SBUS must bias the physical RX line low"),
            ("restore_normal_uart", "R182",
             "SBUS release must restore the pre-takeover UART state"),
            ("receive_error_flags_", "R183",
             "SBUS UART error detail is missing"),
            ("UART_ADVFEATURE_RXINV_ENABLE", "R192",
             "SBUS must automatically enable hardware RX inversion"),
            ("uart->Init.BaudRate = 100000U", "R204",
             "SBUS baud rate must remain 100000 bit/s"),
            ("uart->Init.WordLength = UART_WORDLENGTH_9B", "R205",
             "SBUS 8E2 word length contract changed"),
            ("uart->Init.StopBits = UART_STOPBITS_2", "R206",
             "SBUS must use two stop bits"),
            ("uart->Init.Parity = UART_PARITY_EVEN", "R207",
             "SBUS must use even parity"),
            ("uart->Init.Mode = UART_MODE_RX", "R208",
             "SBUS UART must remain RX-only"),
            ("DIMA_STM32_SERIAL_PORT_LIST", "R196",
             "STM32 serial mapping must come from the board generator"),
            ("UART5_IRQHandler", "R196",
             "UART5 must have an IRQ handler when exposed as SERIAL5"),
            ("configure_normal_baud", "R196",
             "normal serial baud configuration backend is missing"),
            ("reset_normal_configuration", "R196",
             "Runtime serial configuration reset is missing"),
            ("normal_advanced_init_", "R193",
             "SBUS must preserve the normal UART advanced configuration"),
            ("normal_fifo_mode_", "R199",
             "SBUS must preserve the normal UART FIFO mode"),
            ("normal_tx_fifo_threshold_", "R200",
             "SBUS must preserve the normal UART TX FIFO threshold"),
            ("normal_rx_fifo_threshold_", "R201",
             "SBUS must preserve the normal UART RX FIFO threshold"),
            ("normal_rx_pin_", "R202",
             "SBUS must preserve the pre-takeover RX GPIO state"),
            ("restore_rx_pin(normal_rx_pin_)", "R203",
             "SBUS release must restore the pre-takeover RX GPIO state"),
            ("bool stop() noexcept override", "R194",
             "SBUS stop must report UART restoration failure"),
            ("if (restored) {", "R218",
             "SBUS restore failure discards the retry context"),
        ),
        ROOT / "Dima/middleware/parameters/definitions/rc_params.c": (
            ("* @value 0 Disabled", "R195",
             "RC_INPUT_PROTO disabled value is missing"),
            ("PARAM_DEFINE_INT32(RC_INPUT_PROTO, 2);", "R196",
             "SBUS must remain the default RC input protocol"),
        ),
        ROOT / "Dima/modules/serial/SerialConfig.hpp": (
            ("DIMA_BOARD_SERIAL_PARAMETER_LIST", "R196",
             "SerialConfig parameter members must come from the board table"),
            ("rc_input_port() const", "R196",
             "serial function owner must resolve the RC port"),
        ),
        ROOT / "Dima/modules/serial/SerialConfig.cpp": (
            ("serial_baud_supported", "R196",
             "normal baud values must use the generated whitelist"),
            ("serial_function_supported", "R196",
             "serial function values must use the generated whitelist"),
            ("rc_owner_count > 1U", "R196",
             "multiple RC serial owners must fail closed"),
            ("backend_.configure_normal_baud", "R196",
             "SerialConfig must apply normal 8N1 settings"),
        ),
        ROOT / "Dima/modules/rc/SbusRc.cpp": (
            ("if (protocol == 0)", "R209",
             "disabled RC protocol must remain a normal lifecycle state"),
            ("schedule_signal_timeout()", "R210",
             "SBUS signal loss logging must follow the RC timeout"),
            ("signal lost last_frame_us=", "R211",
             "SBUS signal loss state log is missing"),
            ("SBUS release failed; UART normal configuration not restored",
             "R219", "SBUS module does not surface UART restore failure"),
            ("serial_config_.rc_input_port()", "R196",
             "SBUS port must come from SERIALx_FUNCTION ownership"),
            ("dima::board::serial_port(port)", "R196",
             "SBUS must reject ports absent from the board manifest"),
            ("consecutive_healthy_frames_", "R220",
             "SBUS lock does not count consecutive healthy frames"),
            ("signal_locked_ = true;", "R220",
             "SBUS lock transition is missing"),
        ),
        ROOT / "Dima/modules/rc/SbusRc.hpp": (
            ("kRequiredLockFrames = 3U", "R220",
             "SBUS must require three consecutive healthy frames"),
        ),
        ROOT / "Dima/modules/rc/RCUpdate.hpp": (
            ("kRecoveryStableUs = 100000ULL", "R221",
             "RC recovery must remain stable for 100 ms"),
        ),
        ROOT / "Dima/modules/rc/RcManualInput.hpp": (
            ("kSwitchDebounceUs = 200000ULL", "R222",
             "RC Arm/Kill switches must debounce for 200 ms"),
            ("kRequiredStableSamples = 2U", "R222",
             "RC Arm/Kill debounce must require repeated samples"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    for path in first_party_sources():
        text = path.read_text(encoding="utf-8")
        if "DIMA_SBUS_INV" in text:
            violations.append(Violation(
                path, line_for(text, "DIMA_SBUS_INV"), "R197",
                "manual SBUS inversion control must remain removed",
            ))

    sbus_backend = ROOT / "Dima/platform/stm32h7/SbusUart.cpp"
    if sbus_backend.is_file():
        text = sbus_backend.read_text(encoding="utf-8")
        for token in ("PX4_INFO", "PX4_WARN", "PX4_ERR", "PX4_DEBUG",
                      "write_module(", "printf(", "console_.write"):
            if token in text:
                violations.append(Violation(
                    sbus_backend, line_for(text, token), "R198",
                    "SBUS ISR/backend path performs formatted USB logging",
                ))

    param_header = ROOT / "Dima/middleware/parameters/param.h"
    if param_header.is_file():
        text = param_header.read_text(encoding="utf-8")
        constructor = re.search(
            r"constexpr\s+Param\(\)\s+noexcept\s*\{(.*?)\n\s*\}"
            r"\s*bool\s+bind\(\)", text, re.DOTALL,
        )
        if constructor is None or any(
                token in constructor.group(1)
                for token in ("ParamTraits<T, p>::get", "param_set_used")):
            violations.append(Violation(
                param_header, line_for(text, "constexpr Param() noexcept"),
                "R076", "Param constructor touches the Parameter Core",
            ))


def scan_fault_ownership(violations: list[Violation]) -> None:
    allowed = {
        "Boards/H743/Inc/boot_diagnostics_store.h",
        "Boards/H743/Src/boot_diagnostics_store.c",
        "Bootloader/Src/main.c",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        if relative in allowed:
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if ("boot_diagnostics_store.h" in line or
                    "dima_boot_diagnostics_store_" in line):
                violations.append(Violation(
                    path, line_number, "R080",
                    "Application-side code depends on diagnostics Flash store",
                ))

    require_literals(
        ROOT / "Boards/H743/Src/boot_diagnostics.c",
        (
            ("DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID", "R081",
             "Fault capture valid marker is missing"),
            ("__DMB();", "R082", "Fault capture DMB is missing"),
            ("__DSB();", "R083", "Fault capture DSB is missing"),
            ("__ISB();", "R084", "Fault capture ISB is missing"),
            ("NVIC_SystemReset();", "R085",
             "Fault capture must reset immediately"),
        ), violations,
    )
    require_literals(
        ROOT / "Bootloader/Makefile",
        (("Boards/H743/Src/boot_diagnostics_store.c", "R086",
          "MCUboot must own diagnostics Flash persistence"),),
        violations,
    )

    project_make = ROOT / "make/project.mk"
    if project_make.is_file():
        text = project_make.read_text(encoding="utf-8")
        if "Boards/H743/Src/boot_diagnostics_store.c" in text:
            violations.append(Violation(
                project_make,
                line_for(text, "Boards/H743/Src/boot_diagnostics_store.c"),
                "R087", "Application build links diagnostics Flash store",
            ))


def scan_clock_contract(violations: list[Violation]) -> None:
    requirements = {
        ROOT / "Core/Inc/stm32h7xx_hal_conf.h": (
            ("#define HSE_VALUE    (8000000UL)", "R090",
             "HSE contract must remain 8 MHz"),
        ),
        ROOT / "Core/Src/main.c": (
            ("RCC_OscInitStruct.PLL.PLLM = 2;", "R091",
             "PLL1 M divider changed"),
            ("RCC_OscInitStruct.PLL.PLLN = 240;", "R092",
             "PLL1 N multiplier changed"),
            ("RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;", "R093",
             "CPU clock divider changed"),
            ("RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;", "R094",
             "HCLK divider changed"),
            ("RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;", "R095",
             "APB1 divider changed"),
        ),
        ROOT / "Dima/platform/stm32h7/Clock.cpp": (
            ("kExpectedTimerInputClockHz = 240000000U", "R096",
             "TIM2 input clock contract must remain 240 MHz"),
            ("kTimerFrequencyHz = 1000000U", "R097",
             "TIM2 HRT must remain 1 MHz"),
            ("pclk * 2U", "R098",
             "TIM2 APB prescaler doubling rule is missing"),
        ),
        ROOT / "Core/Src/stm32h7xx_it.c": (
            ("void SysTick_Handler(void)", "R099",
             "strong shared SysTick handler is missing"),
            ("HAL_IncTick();", "R100",
             "SysTick no longer advances the HAL tick"),
            ("xPortSysTickHandler();", "R101",
             "SysTick no longer advances FreeRTOS"),
            ("void TIM2_IRQHandler(void)", "R102",
             "strong TIM2 HRT handler is missing"),
            ("dima_hrt_overflow_isr();", "R103",
             "TIM2 handler no longer advances HRT overflow"),
        ),
        ROOT / "Dima/platform/freertos/FreeRTOSConfig.h": (
            ("#define configUSE_TICKLESS_IDLE                  0", "R104",
             "tickless idle must remain disabled"),
        ),
        ROOT / "H743_FreeRTOS.ioc": (
            ("RCC.HSE_VALUE=8000000", "R105", "CubeMX HSE is not 8 MHz"),
            ("RCC.SYSCLKFreq_VALUE=480000000", "R106",
             "CubeMX SYSCLK is not 480 MHz"),
            ("RCC.HCLKFreq_Value=240000000", "R107",
             "CubeMX HCLK is not 240 MHz"),
            ("RCC.APB1Freq_Value=120000000", "R108",
             "CubeMX APB1 is not 120 MHz"),
            ("RCC.Tim2OutputFreq_Value=240000000", "R109",
             "CubeMX TIM2 input is not 240 MHz"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    for relative in ("Core/Src/main.c", "Core/Src/stm32h7xx_it.c",
                     "Dima/platform/stm32h7/Clock.cpp", "make/project.mk"):
        path = ROOT / relative
        if not path.is_file():
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if "TIM12" in line or "htim12" in line:
                violations.append(Violation(
                    path, line_number, "R110",
                    "TIM12 must remain released from the system timebase",
                ))


def scan_active_actuator_contract(violations: list[Violation]) -> None:
    allowed_motor_calls = {
        "Boards/H743/Inc/motor_pwm.h",
        "Boards/H743/Src/motor_pwm.c",
        "Dima/platform/stm32h7/ActuatorPwm.cpp",
    }
    allowed_rover_differential_owners = {
        "Dima/rover/ApplicationContext.cpp",
        "Dima/rover/ApplicationContext.hpp",
        "Dima/rover/control/RoverDifferential.cpp",
        "Dima/rover/control/RoverDifferential.hpp",
    }
    allowed_actuator_pwm_owners = {
        "Dima/platform/api/Platform.hpp",
        "Dima/platform/stm32h7/ActuatorPwm.cpp",
        "Dima/platform/stm32h7/HardwareServices.hpp",
        "Dima/modules/motor/MotorOutput.cpp",
        "Dima/modules/motor/MotorOutput.hpp",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        is_dima_source = relative.startswith("Dima/")
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            board_call = re.search(
                r"\bboard_motor_pwm_(start|stop|write|started)\s*\(", line,
            )
            board_call_allowed = relative in allowed_motor_calls or (
                relative == "Boards/H743/Src/board_init.c" and
                board_call is not None and board_call.group(1) == "stop"
            )
            if board_call is not None and not board_call_allowed:
                violations.append(Violation(
                    path, line_number, "R120",
                    "board motor PWM escaped the board safe-off or STM32 "
                    "capability owners",
                ))
            if (relative != "Boards/H743/Src/motor_pwm.c" and
                    re.search(
                        r"\bHAL_TIM(?:Ex)?_PWMN?_(?:Start|Stop)\s*\(", line,
                    )):
                violations.append(Violation(
                    path, line_number, "R121",
                    "PWM start/stop is outside the board motor backend",
                ))
            if (is_dima_source and re.search(
                    r"\b(?:Mixer|MixingOutput|FunctionMotors|ActuatorOutput)\b",
                    line)):
                violations.append(Violation(
                    path, line_number, "R122",
                    "an unauthorized actuator consumer is present",
                ))
            if (is_dima_source and
                    "RoverDifferential" in line and
                    relative not in allowed_rover_differential_owners):
                violations.append(Violation(
                    path, line_number, "R122",
                    "RoverDifferential escaped the Rover control boundary",
                ))
            if (is_dima_source and
                    "ActuatorPwm" in line and
                    relative not in allowed_actuator_pwm_owners):
                violations.append(Violation(
                    path, line_number, "R126",
                    "six-channel PWM capability escaped its platform and "
                    "Rover output owners",
                ))

    requirements = {
        ROOT / "Boards/H743/Src/board_init.c": (
            ("board_motor_pwm_stop() != BOARD_MOTOR_PWM_APPLIED", "R161",
             "board initialization must establish motor PWM safe-off"),
        ),
        ROOT / "Boards/H743/Src/platform_composition.cpp": (
            ("&stm32h7::actuator_pwm(),", "R162",
             "platform composition does not inject actuator PWM"),
            ("stm32h7::independent_watchdog(),", "R183",
             "platform composition does not inject the IWDG capability"),
        ),
        ROOT / "Dima/platform/api/Platform.hpp": (
            ("class IndependentWatchdog", "R183",
             "platform API is missing the narrow IWDG capability"),
            ("IndependentWatchdog &watchdog;", "R183",
             "platform Services do not expose the IWDG capability"),
        ),
        ROOT / "Dima/rover/ApplicationContext.cpp": (
            ("boot_health_.bind_motor_output(motor_output_);", "R163",
             "BootHealth is not bound to MotorOutput"),
            ("module_manager_.start(motor_output_)", "R164",
             "Application Runtime does not start MotorOutput"),
            ("module_manager_.stop(motor_output_)", "R165",
             "Application Runtime does not stop MotorOutput"),
            ("motor_output_.safe_off_confirmed()", "R166",
             "MotorOutput lifecycle does not verify physical safe-off"),
        ),
        ROOT / "Dima/rover/ApplicationContext.hpp": (
            ("dima::modules::motor::MotorOutput motor_output_;", "R167",
             "ApplicationContext does not own MotorOutput"),
        ),
        ROOT / "Dima/rover/control/RoverDifferential.hpp": (
            ("#include \"rover/DifferentialDrive.hpp\"", "R215",
             "RoverDifferential does not consume the pure Rover library"),
            ("dima::lib::rover::DifferentialDrive drive_{};", "R216",
             "DifferentialDrive escaped its pure algorithm namespace"),
        ),
        ROOT / "Dima/modules/motor/MotorOutput.hpp": (
            ("px4::params::COM_ACT_LOSS_T", "R168",
             "MotorOutput is missing the actuator command timeout"),
            ("hard_safe_inhibit_observed_", "R180",
             "MotorOutput is missing the asynchronous hard-safe latch"),
        ),
        ROOT / "Dima/modules/motor/MotorOutput.cpp": (
            ("timestamp > safety_.actuator_armed.timestamp", "R175",
             "positive safety recovery does not require a newer snapshot"),
            ("now_us - actuator_motors_.timestamp <= timeout_us", "R169",
             "actuator publication time is not bounded"),
            ("now_us - actuator_motors_.timestamp_sample <= timeout_us", "R170",
             "actuator sample time is not bounded"),
            ("pwm_->stop()", "R171",
             "MotorOutput has no backend safe-off path"),
            ("build_neutral_frame(frame)", "R180",
             "MotorOutput has no disarmed-neutral frame path"),
            ("frame.pulse_us[channel] = config.center_us", "R180",
             "disarmed-neutral output must use each channel center"),
            ("hard_safe_inhibit_observed_ || !parameters_valid_", "R180",
             "asynchronous Kill/Failsafe does not inhibit neutral output"),
        ),
        ROOT / "Dima/modules/safety/Commander.cpp": (
            ("actuator_output_ready_for_arming(now)", "R181",
             "Commander pre-arm does not require output readiness"),
            ("actuator_output_recovered_disarmed(now)", "R181",
             "Commander output-failsafe recovery can self-lock"),
            ("Kill engaged; Rover requires a new Arm action", "R181",
             "Kill must disarm and require a new Arm edge"),
        ),
        ROOT / "Dima/lib/rover/DifferentialDrive.cpp": (
            ("-1.0F / config_.thrust_asymmetry", "R182",
             "reverse asymmetry feasible domain is missing"),
            ("config_.steering_throttle_mix, lower_motor_limit", "R182",
             "axis priority does not use the asymmetric motor domain"),
        ),
        ROOT / "Dima/modules/boot_health/BootHealthService.cpp": (
            ("motor_output_->state()", "R176",
             "BootHealth does not monitor MotorOutput lifecycle"),
            ("confirmation_state_safe()", "R177",
             "BootHealth does not reset its window for unsafe vehicle state"),
            ("actuator_output_status_s::STATE_HARD_SAFE_OFF", "R178",
             "BootHealth does not recognize hard-safe-off output"),
            ("actuator_output_status_s::STATE_DISARMED_NEUTRAL", "R178",
             "BootHealth does not recognize disarmed-neutral output"),
            ("output.active_output_mask == 0U", "R179",
             "BootHealth does not validate a zero-channel hard-safe frame"),
            ("output_frame_valid(output)", "R179",
             "BootHealth does not validate active/neutral PWM frames"),
            ("health_generation_", "R183",
             "BootHealth does not advance a Runtime health generation"),
        ),
        ROOT / "Dima/application/app_main.cpp": (
            ("kWatchdogTimeoutMs = 2048U", "R183",
             "application IWDG timeout changed"),
            ("kWatchdogCheckIntervalMs = 100U", "R183",
             "application IWDG health cadence changed"),
            ("application.watchdog_feed_allowed", "R183",
             "appMain does not gate IWDG feeds on Runtime health"),
            ("services.watchdog.feed();", "R183",
             "appMain is missing the IWDG feed"),
        ),
        ROOT / "Dima/platform/stm32h7/Watchdog.cpp": (
            ("kPrescalerDiv32 = 3U", "R183",
             "STM32 IWDG prescaler contract changed"),
            ("IWDG1->KR = kStartKey;\n        IWDG1->KR = kWriteAccessKey;",
             "R183", "IWDG configuration waits for LSI before starting it"),
            ("DBGMCU_APB4FZ1_DBG_IWDG1", "R183",
             "IWDG must freeze while the debugger is halted"),
        ),
        ROOT / "Bootloader/Inc/mcuboot_config/mcuboot_config.h": (
            ("MCUBOOT_WATCHDOG_FEED()", "R183",
             "MCUboot long loops have no watchdog feed hook"),
            ("boot_watchdog_feed()", "R183",
             "MCUboot watchdog hook is not connected"),
        ),
        ROOT / "Bootloader/Src/main.c": (
            ("boot_watchdog_prepare();", "R183",
             "MCUboot does not extend a watchdog carried across reset"),
            ("dima_boot_diagnostics_mark_application_bridge()", "R183",
             "MCUboot does not preserve reset flags across its bridge reset"),
        ),
        ROOT / "Boards/H743/Src/boot_diagnostics_store.c": (
            ("DIMA_BOOT_DETAIL_APPLICATION_BRIDGE", "R183",
             "MCUboot diagnostics owner cannot mark its bridge reset"),
            ("seed_application_bridge_record(record);", "R183",
             "MCUboot cold boot depends on an Application-initialized D3 record"),
            ("record->reset_flags = reset_flags;", "R183",
             "cold-start bridge seeding does not preserve reset flags"),
            ("record->magic = DIMA_BOOT_DIAGNOSTICS_MAGIC;", "R183",
             "cold-start bridge seeding does not publish a valid D3 record"),
        ),
        ROOT / "Bootloader/Makefile": (
            ("Bootloader/Src/boot_watchdog.c", "R183",
             "MCUboot build does not link the watchdog bridge"),
        ),
        ROOT / "Boards/H743/Src/boot_diagnostics.c": (
            ("DIMA_BOOT_DETAIL_APPLICATION_BRIDGE", "R183",
             "application bridge can overwrite the original reset cause"),
            ("previous_reset_flags", "R183",
             "startup diagnostics do not retain the original reset flags"),
        ),
        ROOT / "make/project.mk": (
            ("Dima/platform/stm32h7/Watchdog.cpp", "R183",
             "Application build does not link the STM32 IWDG backend"),
        ),
        ROOT / "Dima/middleware/parameters/definitions/commander_params.c": (
            ("PARAM_DEFINE_FLOAT(COM_ACT_LOSS_T, 0.10f);", "R172",
             "COM_ACT_LOSS_T definition or default changed"),
            ("* @min 0.02", "R173",
             "COM_ACT_LOSS_T minimum changed"),
            ("* @max 1.00", "R174",
             "COM_ACT_LOSS_T maximum changed"),
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

    watchdog_feed_owners = []
    for path in first_party_sources():
        text = path.read_text(encoding="utf-8")
        if "services.watchdog.feed();" in text:
            watchdog_feed_owners.append(path.relative_to(ROOT).as_posix())
    if watchdog_feed_owners != ["Dima/application/app_main.cpp"]:
        violations.append(Violation(
            ROOT / "Dima/application/app_main.cpp", 1, "R183",
            "appMain must be the unique application IWDG feed owner: "
            f"{watchdog_feed_owners}",
        ))

    timer_source = ROOT / "Core/Src/tim.c"
    require_literals(
        timer_source,
        (
            ("sConfigOC.Pulse = 0;", "R123",
             "CubeMX PWM compare defaults must remain zero"),
            ("sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;", "R127",
             "TIM5 must remain a reset-mode slave"),
            ("sSlaveConfig.InputTrigger = TIM_TS_ITR3;", "R128",
             "TIM5 must remain connected to TIM8 TRGO"),
            ("sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;", "R129",
             "TIM8 must publish its update event as TRGO"),
            ("sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;", "R138",
             "TIM8 complementary outputs must map zero compare to low"),
        ),
        violations,
    )
    if timer_source.is_file():
        text = timer_source.read_text(encoding="utf-8")
        for match in re.finditer(r"sConfigOC\.Pulse\s*=\s*([^;]+);", text):
            if match.group(1).strip() not in {"0", "0U", "0UL"}:
                violations.append(Violation(
                    timer_source, line_for(text, match.group(0)), "R124",
                    "CubeMX timer init writes a non-zero compare",
                ))

    ioc = ROOT / "H743_FreeRTOS.ioc"
    require_literals(
        ioc,
        (
            ("TIM5.TIM_SlaveMode=TIM_SLAVEMODE_RESET", "R139",
             "CubeMX TIM5 reset-slave contract is missing"),
            ("VP_TIM5_VS_ClockSourceITR.Mode=TriggerSource_ITR3", "R157",
             "CubeMX TIM5 ITR3 virtual connection is missing"),
            ("TIM8.OCNPolarity_2=TIM_OCNPOLARITY_LOW", "R158",
             "CubeMX TIM8 CH2N polarity is not safe at zero compare"),
            ("TIM8.OCNPolarity_3=TIM_OCNPOLARITY_LOW", "R159",
             "CubeMX TIM8 CH3N polarity is not safe at zero compare"),
            ("TIM8.TIM_MasterOutputTrigger=TIM_TRGO_UPDATE", "R160",
             "CubeMX TIM8 update TRGO contract is missing"),
        ),
        violations,
    )
    if ioc.is_file():
        for line_number, line in enumerate(
                ioc.read_text(encoding="utf-8").splitlines(), 1):
            if NONZERO_PWM_PULSE_RE.match(line):
                violations.append(Violation(
                    ioc, line_number, "R125",
                    "CubeMX PWM pulse default must remain zero",
                ))


def scan_linker_contract(violations: list[Violation]) -> None:
    require_literals(
        ROOT / "Linker/STM32H743VITx_MCUBOOT_APP.ld",
        (
            ("FLASH (rx)         : ORIGIN = 0x08040400", "R130",
             "Application FLASH origin changed"),
            (".dima_ramfunc", "R131", "RAM function section is missing"),
            (".dima_dma ORIGIN(RAM_DMA)", "R132",
             "DMA section ownership is missing"),
            (".dima_heap (NOLOAD)", "R133", "platform heap section is missing"),
            (".dima_task_pool (NOLOAD)", "R134",
             "task pool section is missing"),
            (".dima_boot_diag (NOLOAD)", "R135",
             "D3 diagnostics section is missing"),
            ("ASSERT(ADDR(.dima_dma) == 0x30040000", "R136",
             "DMA address assertion is missing"),
            ("ASSERT(ADDR(.dima_boot_diag) == 0x38000000", "R137",
             "D3 diagnostics address assertion is missing"),
        ),
        violations,
    )


def scan_build_isolation(violations: list[Violation]) -> None:
    path = ROOT / "make/project.mk"
    lines = path.read_text(encoding="utf-8").splitlines()
    checks = {
        "DIMA_COMMON_INCLUDES": (
            "R030",
            re.compile(r"(?:^|\s)-I\.(?:\s|\\|$)|Core/|Boards/|USB_DEVICE/|"
                       r"Drivers/|Middlewares/|FreeRTOS|CMSIS|STM32"),
            "common include set exposes a low-level search path",
        ),
        "DIMA_FREERTOS_INCLUDES": (
            "R031",
            re.compile(r"Core/|Boards/|USB_DEVICE/|Drivers/|CMSIS|STM32|"
                       r"platform/stm32h7"),
            "FreeRTOS include set exposes an MCU search path",
        ),
        "DIMA_STM32_INCLUDES": (
            "R032",
            re.compile(r"FreeRTOS|platform/freertos"),
            "STM32 include set exposes an RTOS search path",
        ),
    }
    for name, (rule, pattern, message) in checks.items():
        block = variable_block(lines, name)
        if not block:
            violations.append(Violation(
                path, 1, rule, f"missing build include set {name}",
            ))
            continue
        for line_number, line in block:
            if pattern.search(line):
                violations.append(Violation(path, line_number, rule, message))

    required = {
        "$(DIMA_COMMON_OBJECTS): DIMA_PRIVATE_INCLUDES := "
        "$(DIMA_COMMON_INCLUDES)": "R033",
        "$(DIMA_FREERTOS_OBJECTS): DIMA_PRIVATE_INCLUDES := "
        "$(DIMA_FREERTOS_INCLUDES)": "R034",
        "$(DIMA_STM32_OBJECTS): DIMA_PRIVATE_INCLUDES := "
        "$(DIMA_STM32_INCLUDES)": "R035",
        "$(CC) -c $(DIMA_PROJECT_CFLAGS)": "R036",
        "$(CXX) -c $(DIMA_PROJECT_CXXFLAGS)": "R037",
        "override CFLAGS += -Werror": "R038",
        "$(OPT) -Wall -Werror -fdata-sections": "R038",
    }
    text = "\n".join(lines)
    for required_text, rule in required.items():
        if required_text not in text:
            violations.append(Violation(
                path, 1, rule,
                f"missing isolated-build contract '{required_text}'",
            ))

    require_literals(
        ROOT / "Bootloader/Makefile",
        (("-Wall -Wextra -Werror", "R038",
          "MCUboot warnings must fail the formal build"),),
        violations,
    )


def scan_include_style(violations: list[Violation]) -> None:
    """Require first-party includes to use a registered, non-parent path."""
    dima_full_path_re = re.compile(r'^\s*#\s*include\s*[<"]Dima/')
    for path in sources_under(("Dima",)):
        if is_vendored(path):
            continue
        if path.suffix.lower() not in SOURCE_SUFFIXES:
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            if dima_full_path_re.match(line):
                violations.append(Violation(
                    path, line_number, "R300",
                    f"include uses full 'Dima/' path '{match.group(1)}'; "
                    f"use layer-root-relative form instead",
                ))
                continue
            include = match.group(1)
            if include.startswith("./") or ".." in include.split("/"):
                violations.append(Violation(
                    path, line_number, "R301",
                    f"include uses a relative hierarchy '{include}'; "
                    "use a registered include-root spelling instead",
                ))
                continue
            target = resolve_common_include(path, include)
            if target is None or not target.is_relative_to(ROOT / "Dima"):
                continue
            spellings: set[str] = set()
            for root in COMMON_INCLUDE_ROOTS:
                include_root = ROOT / root
                if target.is_relative_to(include_root):
                    spellings.add(target.relative_to(include_root).as_posix())
            if target.is_relative_to(path.parent):
                spellings.add(target.relative_to(path.parent).as_posix())
            if include not in spellings:
                violations.append(Violation(
                    path, line_number, "R301",
                    f"include spelling '{include}' is not canonical; expected "
                    f"one of {sorted(spellings)}",
                ))


def strip_cpp_structure(text: str) -> list[str]:
    """Strip comments and literals while retaining one output line per line."""
    literal_re = re.compile(
        r'"(?:\\.|[^"\\\r\n])*"|\'(?:\\.|[^\'\\\r\n])*\''
    )
    text = literal_re.sub('""', text)
    text = re.sub(
        r"/\*.*?\*/",
        lambda match: "\n" * match.group(0).count("\n"),
        text,
        flags=re.DOTALL,
    )
    output: list[str] = []
    for line in text.splitlines():
        line = re.sub(r"//.*$", "", line)
        output.append(line)
    return output


def scan_namespace_convention(violations: list[Violation]) -> None:
    """R310: declarations must be enclosed by an allowed outer namespace."""
    known_ns_re = re.compile(
        r"^\s*namespace\s+(?:dima(?:\b|::)|px4\b|uORB\b).*\{"
    )
    # PX4 upstream directories that intentionally use C-style or
    # thin forwarding headers without their own namespace declaration.
    ns_exempt_roots = (
        "Dima/messages",
        "Dima/lib/containers",
    )
    # Thin forwarding headers that delegate to a namesake .hpp
    # which already carries the namespace declaration.
    forwarding_header_ok = {
        "Dima/middleware/uorb/Publication.hpp",
        "Dima/middleware/uorb/SubscriptionData.hpp",
        "Dima/middleware/work_queue/ScheduledWorkItem.hpp",
    }
    c_abi_files = {
        "Dima/adapters/usb_console/UsbConsole.cpp",
        "Dima/application/app_bootstrap.cpp",
        "Dima/application/app_main.cpp",
        "Dima/lib/tinybson/tinybson.cpp",
        "Dima/middleware/logging/logging.cpp",
        "Dima/middleware/logging/logging.hpp",
        "Dima/middleware/parameters/autosave.cpp",
        "Dima/middleware/parameters/flashparams/flashparams.cpp",
        "Dima/middleware/parameters/param.cpp",
        "Dima/middleware/perf/perf_counter.cpp",
        "Dima/platform/api/Time.hpp",
        "Dima/platform/freertos/Backend.cpp",
        "Dima/platform/stm32h7/Clock.cpp",
        "Dima/platform/stm32h7/DmaMemory.cpp",
        "Dima/platform/stm32h7/FlashDevice.cpp",
        "Dima/platform/stm32h7/SbusUart.cpp",
        "Dima/platform/stm32h7/SensorInterrupts.cpp",
    }
    for path in sources_under(("Dima",)):
        if is_vendored(path):
            continue
        if path.suffix.lower() not in {".cpp", ".hpp"}:
            continue
        try:
            rel = path.relative_to(ROOT).as_posix()
        except ValueError:
            continue
        if any(rel.startswith(root + "/") for root in ns_exempt_roots):
            continue
        if rel in forwarding_header_ok:
            continue
        if rel in c_abi_files:
            continue
        lines = strip_cpp_structure(path.read_text(encoding="utf-8"))
        brace_depth = 0
        preprocessor_continuation = False
        for line_number, line in enumerate(lines, 1):
            stripped = line.strip()
            if preprocessor_continuation or stripped.startswith("#"):
                preprocessor_continuation = line.rstrip().endswith("\\")
                continue
            preprocessor_continuation = False
            if not stripped:
                continue
            if brace_depth == 0:
                if known_ns_re.match(line):
                    pass
                elif re.match(r"^\s*namespace\s*\{", line):
                    violations.append(Violation(
                        path, line_number, "R310",
                        "anonymous namespace must be nested in an allowed "
                        "outer namespace",
                    ))
                elif re.match(r'^\s*extern\s+""', line):
                    if rel not in c_abi_files:
                        violations.append(Violation(
                            path, line_number, "R310",
                            "global extern C linkage is not allowed for this file",
                        ))
                elif stripped in {";", "}"} or stripped.startswith("static_assert"):
                    pass
                else:
                    violations.append(Violation(
                        path, line_number, "R310",
                        "declaration or definition is outside an allowed outer "
                        "namespace",
                    ))
            brace_depth += line.count("{") - line.count("}")
            if brace_depth < 0:
                brace_depth = 0


def scan_usb_console_owner(violations: list[Violation]) -> None:
    """R320: MavlinkService is the only Application Console data-plane owner."""
    owners = {
        "Dima/modules/mavlink/MavlinkService.cpp",
        "Dima/modules/mavlink/MavlinkService.hpp",
    }
    data_plane_re = re.compile(
        r"\b(?:console_|console|services_\.console)\."
        r"(?:service|read|read_byte|write)\s*\("
    )
    for path in sources_under(("Dima",)):
        relative = path.relative_to(ROOT).as_posix()
        platform_or_adapter = (
            relative.startswith("Dima/platform/") or
            relative.startswith("Dima/adapters/usb_console/")
        )
        if relative in owners or platform_or_adapter:
            continue
        lines = path.read_text(encoding="utf-8").splitlines()
        for line_number, line in enumerate(lines, 1):
            if data_plane_re.search(line):
                violations.append(Violation(
                    path, line_number, "R320",
                    "Application Console data plane is owned by MavlinkService",
                ))
            if re.search(r"\b(?:dima::)?platform::Console\s*[&*]", line):
                violations.append(Violation(
                    path, line_number, "R321",
                    "Application Console capability may only be retained by "
                    "MavlinkService",
                ))


def scan_mavlink_contract(violations: list[Violation]) -> None:
    """R330-R336: enforce the shipped, deliberately trimmed MAVLink surface."""
    forbidden_paths = (
        ROOT / "Dima/lib/mavlink/c_library_v2",
        ROOT / "tools/generate_actuators_metadata.py",
    )
    for path in forbidden_paths:
        if path.exists():
            violations.append(Violation(
                path, 1, "R330",
                "generated source-tree MAVLink or Actuator Metadata code "
                "must remain absent",
            ))

    forbidden_parameters = {
        "COM_DL_LOSS_T", "COM_ARM_SWISBTN", "COM_RC_ARM_HYST",
        "MAN_ARM_GESTURE", "RTL_RETURN_ALT", "RTL_DESCEND_ALT",
        "RTL_LAND_DELAY", "COM_FLTMODE1", "COM_FLTMODE2",
        "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5",
        "COM_FLTMODE6",
    }
    qgc_fixed_schema = (
        ("SYS_AUTOSTART", "INT32", "50000"),
        ("SYS_AUTOCONFIG", "INT32", "0"),
        ("MAV_SYS_ID", "INT32", "1"),
        ("CAL_GYRO0_ID", "INT32", "0"),
        ("CAL_ACC0_ID", "INT32", "0"),
        ("CAL_MAG0_ID", "INT32", "0"),
        ("CAL_MAG1_ID", "INT32", "0"),
        ("CAL_MAG2_ID", "INT32", "0"),
        ("NAV_RCL_ACT", "INT32", "6"),
        ("NAV_DLL_ACT", "INT32", "0"),
        ("COM_LOW_BAT_ACT", "INT32", "0"),
    )
    serial_parameter_schema = (
        ("SERIAL1_BAUD", "INT32", "921600"),
        ("SERIAL1_FUNCTION", "INT32", "0"),
        ("SERIAL2_BAUD", "INT32", "0"),
        ("SERIAL2_FUNCTION", "INT32", "0"),
        ("SERIAL3_BAUD", "INT32", "0"),
        ("SERIAL3_FUNCTION", "INT32", "0"),
        ("SERIAL4_BAUD", "INT32", "115200"),
        ("SERIAL4_FUNCTION", "INT32", "0"),
        ("SERIAL5_BAUD", "INT32", "115200"),
        ("SERIAL5_FUNCTION", "INT32", "0"),
        ("SERIAL6_BAUD", "INT32", "0"),
        ("SERIAL6_FUNCTION", "INT32", "1"),
        ("SERIAL7_BAUD", "INT32", "57600"),
        ("SERIAL7_FUNCTION", "INT32", "0"),
        ("SERIAL8_BAUD", "INT32", "115200"),
        ("SERIAL8_FUNCTION", "INT32", "0"),
        ("DIMA_SER_VER", "INT32", "0"),
    )
    serial_config_schema = ("RC_PORT_CONFIG", "INT32", "6")
    expected_qgc_schema = set(qgc_fixed_schema)
    qgc_parameter_names = tuple(entry[0] for entry in qgc_fixed_schema)
    expected_qgc_names = set(qgc_parameter_names)
    serial_parameter_names = tuple(
        entry[0] for entry in serial_parameter_schema
    )
    expected_serial_names = set(serial_parameter_names)
    stable_tail_names = qgc_parameter_names + serial_parameter_names
    parameter_definition_re = re.compile(
        r"\bPARAM_DEFINE_(INT32|FLOAT)\s*\(\s*"
        r"([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([^\s,)]+)\s*\)\s*;"
    )
    raw_parameter_definition_re = re.compile(
        r"\b(?:PX4_)?PARAM_DEFINE_[A-Z_][A-Z0-9_]*\s*\("
    )
    project_mk_path = ROOT / "make/project.mk"
    project_mk_text = project_mk_path.read_text(encoding="utf-8")
    definition_block = re.search(
        r"^PARAMETER_DEFINITIONS\s*:=\s*\\\s*\n"
        r"(?P<body>(?:^[^\n]*\\\s*\n)*^[^\n]*)",
        project_mk_text,
        re.MULTILINE,
    )
    definition_inputs = [] if definition_block is None else re.findall(
        r"Dima/middleware/parameters/definitions/[^\\\s]+\.[ch]",
        definition_block.group("body"),
    )
    parameter_paths = [ROOT / relative for relative in definition_inputs]
    parameter_definition_count = 0
    raw_parameter_definition_count = 0
    parameter_name_locations: dict[str, list[tuple[pathlib.Path, int]]] = {}
    for path in parameter_paths:
        if not path.is_file():
            violations.append(Violation(
                path, 1, "R331",
                "listed parameter definition source does not exist",
            ))
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        raw_parameter_definition_count += len(
            raw_parameter_definition_re.findall(code)
        )
        definitions = tuple(parameter_definition_re.finditer(code))
        parameter_definition_count += len(definitions)
        for definition in definitions:
            parameter = definition.group(2)
            parameter_name_locations.setdefault(parameter, []).append(
                (path, line_for(text, definition.group(0)))
            )
            if parameter in forbidden_parameters:
                violations.append(Violation(
                    path, line_for(text, definition.group(0)), "R331",
                    f"unimplemented parameter '{parameter}' entered the build",
                ))
    serial_manifest_path = ROOT / "Boards/H743/serial_ports.json"
    for name, parameter_type, value in (
            serial_config_schema, *serial_parameter_schema):
        parameter_definition_count += 1
        raw_parameter_definition_count += 1
        parameter_name_locations.setdefault(name, []).append(
            (serial_manifest_path, 1)
        )
    listed_parameter_paths = set(parameter_paths)
    for path in sources_under(("Dima/middleware/parameters/definitions",)):
        if path in listed_parameter_paths:
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        if raw_parameter_definition_re.search(code):
            violations.append(Violation(
                path, 1, "R331",
                "parameter definition exists outside PARAMETER_DEFINITIONS",
            ))
    if raw_parameter_definition_count != 205:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "generated parameter source contract must contain exactly 205 "
            f"definitions (found {raw_parameter_definition_count})",
        ))
    if parameter_definition_count != raw_parameter_definition_count:
        violations.append(Violation(
            ROOT / "Dima/middleware/parameters/definitions", 1, "R331",
            "every parameter definition must use a parseable literal type, "
            "name, and default "
            f"(parsed={parameter_definition_count}, "
            f"raw={raw_parameter_definition_count})",
        ))
    for name, locations in parameter_name_locations.items():
        if len(locations) > 1:
            path, line = locations[1]
            violations.append(Violation(
                path, line, "R331",
                f"parameter '{name}' has duplicate source definitions",
            ))

    qgc_compat_path = (
        ROOT / "Dima/middleware/parameters/definitions/qgc_compat_params.c"
    )
    qgc_compat_definitions = []
    qgc_compat_raw_definition_count = 0
    if qgc_compat_path.exists():
        qgc_compat_text = qgc_compat_path.read_text(encoding="utf-8")
        qgc_compat_code = strip_c_comments(qgc_compat_text)
        qgc_compat_raw_definition_count = len(
            raw_parameter_definition_re.findall(qgc_compat_code)
        )
        qgc_compat_definitions = [
            (match.group(2), match.group(1), match.group(3))
            for match in parameter_definition_re.finditer(qgc_compat_code)
        ]
    if (qgc_compat_raw_definition_count != 11 or
            len(qgc_compat_definitions) != 11):
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "qgc_compat_params.c must contain exactly 11 fixed parameter "
            "definitions with parseable literal values "
            f"(raw={qgc_compat_raw_definition_count}, "
            f"parsed={len(qgc_compat_definitions)})",
        ))
    actual_qgc_definitions = qgc_compat_definitions
    actual_qgc_schema = set(actual_qgc_definitions)
    if actual_qgc_schema != expected_qgc_schema:
        missing = sorted(expected_qgc_schema - actual_qgc_schema)
        extra = sorted(actual_qgc_schema - expected_qgc_schema)
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "QGC compatibility schema differs from the exact fixed contract "
            f"(missing={missing}, extra={extra})",
        ))
    actual_qgc_order = tuple(
        name for name, _parameter_type, _value in actual_qgc_definitions
    )
    if actual_qgc_order != qgc_parameter_names:
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "QGC compatibility definitions must retain their stable append "
            f"order (expected={qgc_parameter_names}, "
            f"actual={actual_qgc_order})",
        ))
    actual_stable_tail_order = tuple(
        name for name, _parameter_type, _value in
        (*qgc_compat_definitions, *serial_parameter_schema)
    )
    if actual_stable_tail_order != stable_tail_names:
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "PX4/QGC stable tail order changed "
            f"(expected={stable_tail_names}, actual={actual_stable_tail_order})",
        ))

    require_literals(
        ROOT / "tools/parameters/generate_parameters.py",
        (
            ("for source in args.stable_tail_source", "R331",
             "ordered compatibility sources must define the stable tail"),
            ("parameters = generate(xml_path, args.output, stable_tail_names)",
             "R331", "parameter generator must apply the stable tail"),
            ("generated_json_names = json_names(json_path, xml_names)",
             "R331", "JSON catalogue must use the generated handle order"),
            ("xml_names != generated_json_names", "R331",
             "JSON and firmware parameter orders must remain identical"),
            ('"--src-file"', "R331",
             "parameter parser must receive the explicit source file set"),
            ("EXPECTED_PARAMETER_COUNT = 205", "R331",
             "generator must fail closed on a parameter count change"),
            ("EXPECTED_STABLE_TAIL_COUNT = 28", "R331",
             "generator must fail closed on a compatibility tail change"),
            ('stable_tail_names[0] != "SYS_AUTOSTART"', "R331",
             "SYS_AUTOSTART must preserve prior handle 177"),
            ('stable_tail_names[-1] != "DIMA_SER_VER"', "R331",
             "serial migration version must terminate the stable tail"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/parameters/process_parameters.py",
        (
            ('parser.add_argument("--src-file"', "R331",
             "official parameter parser must accept explicit source files"),
            ("scanner.ScanFile(source_file, parser)", "R331",
             "only listed parameter source files may be scanned"),
        ),
        violations,
    )
    require_literals(
        project_mk_path,
        (
            ("--stable-tail-source Dima/middleware/parameters/definitions/"
             "qgc_compat_params.c", "R331",
             "QGC compatibility source must start the explicit stable tail"),
            ("--stable-tail-source $(SERIAL_BAUD_PARAMETERS)", "R331",
             "generated board serial parameters must finish the stable tail"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/parameters/generate_header.py",
        (
            ("def order_with_stable_tail", "R331",
             "generated handles must support an append-only stable tail"),
            ("+ [by_name[name] for name in tail_names]", "R331",
             "compatibility handles must be appended in source order"),
        ),
        violations,
    )

    all_parameter_names = set(parameter_name_locations)
    prior_parameter_names = sorted(
        all_parameter_names -
        ((expected_qgc_names - {"SYS_AUTOSTART"}) | expected_serial_names)
    )
    prior_parameter_digest = hashlib.sha256(
        ("\n".join(prior_parameter_names) + "\n").encode("ascii")
    ).hexdigest()
    expected_generated_order = (
        sorted(all_parameter_names - expected_qgc_names - expected_serial_names)
        + list(stable_tail_names)
    )
    if (len(prior_parameter_names) != 178 or
            prior_parameter_digest !=
            "10477cbfe796ac845b63069aa0676326a8183a3f27ee02f0f184ce019f9a2449" or
            expected_generated_order[:178] != prior_parameter_names):
        violations.append(Violation(
            qgc_compat_path, 1, "R331",
            "stable-tail layout must preserve all 178 prior parameter "
            "handles before appending the 28-entry QGC/board-serial tail",
        ))

    mavlink_parameters_path = (
        ROOT / "Dima/modules/mavlink/MavlinkParameters.cpp"
    )
    mavlink_parameters_text = mavlink_parameters_path.read_text(
        encoding="utf-8"
    )
    if mavlink_parameters_text.count("is_internal_parameter(name)") != 3:
        violations.append(Violation(
            mavlink_parameters_path, 1, "R336",
            "internal parameters must be filtered in SET, Classic READ, and Ext READ",
        ))
    registry_entry_re = re.compile(
        r'\{\s*"([A-Z][A-Z0-9_]*)"\s*,\s*'
        r'([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[fF])?)\s*\}'
    )

    def registry_entries(
            array_name: str,
            parameter_type: str) -> list[tuple[str, str, str]]:
        array = re.search(
            rf"{re.escape(array_name)}\[\]\s*\{{(?P<body>.*?)\}};",
            mavlink_parameters_text,
            re.DOTALL,
        )
        if array is None:
            violations.append(Violation(
                mavlink_parameters_path, 1, "R331",
                f"QGC fixed-value registry '{array_name}' is missing",
            ))
            return []
        body = array.group("body")
        residue = registry_entry_re.sub("", body)
        if re.sub(r"[\s,]", "", residue):
            violations.append(Violation(
                mavlink_parameters_path,
                line_for(mavlink_parameters_text, array.group(0)),
                "R331",
                f"QGC fixed-value registry '{array_name}' contains an "
                "unparseable or non-literal entry",
            ))
        return [
            (match.group(1), parameter_type, match.group(2))
            for match in registry_entry_re.finditer(body)
        ]

    actual_qgc_registry = registry_entries(
        "kQgcFixedInt32Parameters", "INT32"
    )
    expected_qgc_registry = {
        (name, parameter_type,
         value.upper() if parameter_type == "FLOAT" else value)
        for name, parameter_type, value in qgc_fixed_schema
    }
    registry_names = [entry[0] for entry in actual_qgc_registry]
    duplicate_registry_names = sorted({
        name for name in registry_names if registry_names.count(name) > 1
    })
    if (len(actual_qgc_registry) != 11 or duplicate_registry_names or
            set(actual_qgc_registry) != expected_qgc_registry):
        missing = sorted(expected_qgc_registry - set(actual_qgc_registry))
        extra = sorted(set(actual_qgc_registry) - expected_qgc_registry)
        violations.append(Violation(
            mavlink_parameters_path, 1, "R331",
            "QGC fixed-value registry differs from the exact schema "
            f"(duplicates={duplicate_registry_names}, missing={missing}, "
            f"extra={extra})",
        ))

    parameter_service_path = (
        ROOT / "Dima/modules/parameters/ParameterService.cpp"
    )
    parameter_service_text = parameter_service_path.read_text(encoding="utf-8")
    ignored_array = re.search(
        r"kQgcJournalIgnoredParameters\[\]\s*\{(?P<body>.*?)\};",
        parameter_service_text,
        re.DOTALL,
    )
    ignored_name_re = re.compile(r'"([A-Z][A-Z0-9_]*)"')
    ignored_names = [] if ignored_array is None else [
        match.group(1)
        for match in ignored_name_re.finditer(ignored_array.group("body"))
    ]
    if ignored_array is not None:
        ignored_residue = ignored_name_re.sub("", ignored_array.group("body"))
        if re.sub(r"[\s,]", "", ignored_residue):
            violations.append(Violation(
                parameter_service_path,
                line_for(parameter_service_text, ignored_array.group(0)),
                "R331",
                "Journal fixed-parameter filter contains an unparseable or "
                "non-literal entry",
            ))
    duplicate_ignored_names = sorted({
        name for name in ignored_names if ignored_names.count(name) > 1
    })
    if (len(ignored_names) != 11 or duplicate_ignored_names or
            set(ignored_names) != expected_qgc_names):
        violations.append(Violation(
            parameter_service_path, 1, "R331",
            "Journal fixed-parameter filter must match the exact QGC schema "
            f"(duplicates={duplicate_ignored_names}, "
            f"missing={sorted(expected_qgc_names - set(ignored_names))}, "
            f"extra={sorted(set(ignored_names) - expected_qgc_names)})",
        ))
    require_literals(
        parameter_service_path,
        (
            ("is_qgc_compatibility_parameter(name)", "R331",
             "Journal load must identify fixed QGC compatibility values"),
            ("is_disabled_mode_compatibility_parameter(name)", "R336",
             "disabled RC mode mapping must ignore legacy stored values"),
            ("load_mutable_parameter, &filtered", "R331",
             "Journal decode must filter fixed values before the param layer"),
            ("scan_serial_storage, &scan", "R331",
             "Journal load must detect the serial schema before migration"),
            ("migrate_serial_schema_v1()", "R331",
             "serial schema v1 must migrate before direct numbering is used"),
            ("kSchema1ToDirectSerial", "R331",
             "schema v1 migration must preserve physical UART ownership"),
            ("unsupported serial schema=", "R331",
             "invalid or future serial schemas must fail closed"),
            ("migrate_serial_configuration(loaded == 0)", "R331",
             "legacy RC port values must migrate to SERIALx_FUNCTION"),
            ("migrate_legacy_rc_port", "R331",
             "serial migration must use the generated board mapping"),
            ("legacy_serial_for_baud_parameter", "R331",
             "legacy baud values must migrate by physical serial port"),
        ),
        violations,
    )

    common_qgc_owners = {
        qgc_compat_path,
        mavlink_parameters_path,
        parameter_service_path,
    }
    expected_qgc_consumers = {
        name: set() for name in qgc_parameter_names
    }
    expected_qgc_consumers.update({
        "MAV_SYS_ID": {ROOT / "Dima/modules/mavlink/MavlinkService.cpp"},
        "NAV_RCL_ACT": {ROOT / "Dima/modules/safety/Commander.cpp"},
        "NAV_DLL_ACT": {ROOT / "Dima/modules/safety/Commander.cpp"},
        **{
            f"COM_FLTMODE{slot}": {
                ROOT / "Dima/modules/rc/RcManualInput.cpp"
            }
            for slot in range(1, 7)
        },
    })
    actual_qgc_consumers = {
        name: set() for name in qgc_parameter_names
    }
    for path in sources_under(("Dima",)):
        if path in common_qgc_owners:
            continue
        text = path.read_text(encoding="utf-8")
        code = strip_c_comments(text)
        for parameter in qgc_parameter_names:
            if re.search(rf"\b{re.escape(parameter)}\b", code):
                actual_qgc_consumers[parameter].add(path)
    for parameter in qgc_parameter_names:
        actual = actual_qgc_consumers[parameter]
        expected = expected_qgc_consumers[parameter]
        if actual != expected:
            unexpected = actual - expected
            location = next(iter(unexpected or expected), qgc_compat_path)
            violations.append(Violation(
                location, 1, "R331",
                f"QGC compatibility parameter '{parameter}' consumer set "
                "differs from the exact whitelist "
                f"(expected={sorted(path.relative_to(ROOT).as_posix() for path in expected)}, "
                f"actual={sorted(path.relative_to(ROOT).as_posix() for path in actual)})",
            ))

    require_literals(
        ROOT / "Dima/messages/vehicle_command.hpp",
        (("std::uint8_t  source_system;", "R332",
          "vehicle_command must retain the MAVLink source system"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkIdentity.cpp",
        (
            ("MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT", "R333",
             "MAVLink parameter capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE", "R333",
             "MAVLink bytewise parameter encoding is missing"),
            ("MAV_PROTOCOL_CAPABILITY_FTP", "R333",
             "read-only parameter Metadata FTP capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_COMMAND_INT", "R333",
             "COMMAND_INT capability is missing"),
            ("MAV_PROTOCOL_CAPABILITY_MAVLINK2", "R333",
             "MAVLink2 capability is missing"),
        ),
        violations,
    )
    identity_path = ROOT / "Dima/modules/mavlink/MavlinkIdentity.cpp"
    identity_text = identity_path.read_text(encoding="utf-8")
    capability_tokens = set(re.findall(
        r"MAV_PROTOCOL_CAPABILITY_[A-Z0-9_]+", identity_text,
    ))
    expected_capabilities = {
        "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT",
        "MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE",
        "MAV_PROTOCOL_CAPABILITY_FTP",
        "MAV_PROTOCOL_CAPABILITY_COMMAND_INT",
        "MAV_PROTOCOL_CAPABILITY_MAVLINK2",
    }
    if capability_tokens != expected_capabilities:
        violations.append(Violation(
            identity_path, 1, "R333",
            "MAVLink capability set differs from implemented surface: "
            f"{sorted(capability_tokens)}",
        ))
    require_literals(
        ROOT / "make/project.mk",
        (
            ("MAVLINK_GENERATED_DIR := $(BUILD_DIR)/generated/mavlink", "R334",
             "MAVLink generation must stay under build/generated"),
            ("PARAMETER_METADATA_GENERATOR := tools/mavlink/"
             "generate_parameter_metadata.py", "R334",
             "parameter Metadata generator is missing"),
            ("PARAMETER_METADATA_DIR := $(BUILD_DIR)/generated/"
             "component_metadata", "R334",
             "parameter Metadata outputs must stay under build/generated"),
            ("$(PARAMETER_METADATA_STAMP)", "R334",
             "firmware objects must depend on generated parameter Metadata"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/mavlink/build_trimmed_dialect.py",
        (
            ("install_generated_tree(generated, output_dir)", "R334",
             "Windows MAVLink generation must tolerate short directory locks"),
            ('"FILE_TRANSFER_PROTOCOL"', "R337",
             "Metadata FTP message is absent from the dialect"),
            ('"COMPONENT_METADATA"', "R337",
             "modern Component Metadata message is absent"),
            ('"COMPONENT_INFORMATION"', "R337",
             "deprecated Component Information fallback is absent"),
        ),
        violations,
    )
    require_literals(
        ROOT / "tools/mavlink/generate_parameter_metadata.py",
        (
            ('GENERAL_URI = "mftp://etc/extras/'
             'component_general.json.xz"', "R337",
             "General Metadata URI changed"),
            ('PARAMETER_URI = "mftp://etc/extras/parameters.json.xz"',
             "R337", "Parameter Metadata URI changed"),
            ('INTERNAL_PARAMETERS = {"RC_PORT_CONFIG", "DIMA_SER_VER"}',
             "R337", "internal parameters entered public Metadata"),
            ('"type": 1', "R337",
             "General Metadata must advertise parameters only"),
            ('"version": 1', "R337",
             "QGC parameter Metadata version must remain one"),
            ("general_crc = mavlink_crc32(general_json)", "R337",
             "General Metadata CRC must follow PX4 uncompressed semantics"),
            ("parameter_crc = mavlink_crc32(parameter_xz)", "R337",
             "Parameter Metadata CRC must cover the served XZ file"),
            ("validate_parameter(parameter, index)", "R337",
             "QGC parameter object validation is missing"),
            ("lzma.decompress(parameter_xz) != parameter_json", "R337",
             "Parameter Metadata XZ round-trip validation is missing"),
            ("lzma.decompress(general_xz) != general_json", "R337",
             "General Metadata XZ round-trip validation is missing"),
        ),
        violations,
    )
    metadata_generator_text = (
        ROOT / "tools/mavlink/generate_parameter_metadata.py"
    ).read_text(encoding="utf-8")
    for forbidden in (
            "COMP_METADATA_TYPE_ACTUATORS", "actuators.json",
            "events.json", '"type": 4', '"type": 5'):
        if forbidden in metadata_generator_text:
            violations.append(Violation(
                ROOT / "tools/mavlink/generate_parameter_metadata.py",
                line_for(metadata_generator_text, forbidden), "R337",
                "only Parameter Metadata may be generated",
            ))
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.hpp",
        (
            ("SubscriptionData<input_rc_s>", "R335",
             "MAVLink must subscribe to raw input_rc for QGC calibration"),
            ("kRcChannelsIntervalUs = 200000ULL", "R336",
             "QGC raw RC stream must remain fixed at 5 Hz"),
            ("bool rc_stream_active_{false};", "R336",
             "QGC raw RC stream must retain loss/recovery edge state"),
            ("MavlinkMetadataFtp metadata_ftp_", "R337",
             "MavlinkService must own the read-only Metadata FTP state"),
            ("bool transport_was_ready_{false};", "R337",
             "physical USB readiness must own FTP session lifetime"),
            ("bool send_component_metadata()", "R337",
             "modern Component Metadata response is missing"),
            ("bool send_component_information()", "R337",
             "deprecated Component Information fallback is missing"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.cpp",
        (
            ("mavlink_msg_rc_channels_encode", "R335",
             "MAVLink must stream raw RC_CHANNELS"),
            ("if (!rc_sample_streamable(now)) {", "R336",
             "missing or stale raw RC must stop the QGC stream"),
            ("channels.chancount = channel_count;", "R336",
             "RC_CHANNELS must carry the real valid channel count"),
            ("case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:", "R337",
             "FILE_TRANSFER_PROTOCOL dispatch is missing"),
            ("metadata_ftp_.handle_message(&msg, hrt_absolute_time())", "R337",
             "FTP requests must retain Runtime time and source routing"),
            ("if (!metadata_ftp_.service(now))", "R337",
             "pending FTP must block lower-priority parameter/log traffic"),
            ("case MAVLINK_MSG_ID_COMPONENT_METADATA:", "R337",
             "MAV_CMD_REQUEST_MESSAGE cannot serve COMPONENT_METADATA"),
            ("case MAVLINK_MSG_ID_COMPONENT_INFORMATION:", "R337",
             "MAV_CMD_REQUEST_MESSAGE cannot serve fallback information"),
            ("metadata_ftp_.reset();", "R337",
             "Runtime/link loss must clear FTP state"),
            ("if (!transport_ready && transport_was_ready_)", "R337",
             "physical USB falling edge must clear FTP state"),
            ("discard_rx();", "R337",
             "USB falling edge must discard stale Console input"),
            ("reset_parser_state();", "R337",
             "USB falling edge must reset the MAVLink parser"),
        ),
        violations,
    )
    mavlink_service_path = ROOT / "Dima/modules/mavlink/MavlinkService.cpp"
    mavlink_service_text = mavlink_service_path.read_text(encoding="utf-8")
    if mavlink_service_text.count("metadata_ftp_.reset();") < 2:
        violations.append(Violation(
            mavlink_service_path, 1, "R337",
            "FTP state must reset on Runtime reset and USB disconnect",
        ))
    for forbidden in ("rc_invalid_sent_", "channels.chancount = 0"):
        if forbidden in mavlink_service_text:
            violations.append(Violation(
                mavlink_service_path,
                line_for(mavlink_service_text, forbidden),
                "R336",
                "RC loss must stop RC_CHANNELS instead of sending a zero-count "
                "snapshot",
            ))
    for forbidden in ("latest_input_rc_.rc_failsafe",
                      "latest_input_rc_.rc_lost"):
        if forbidden in mavlink_service_text:
            violations.append(Violation(
                mavlink_service_path,
                line_for(mavlink_service_text, forbidden),
                "R336",
                "RC failsafe/lost flags must gate control, not hide a fresh "
                "raw RC_CHANNELS sample from QGC",
            ))
    ftp_path = ROOT / "Dima/modules/mavlink/MavlinkMetadataFtp.hpp"
    require_literals(
        ftp_path,
        (
            ("kMaxDataLength == 239U", "R337",
             "MAVLink FTP data capacity changed"),
            ("sizeof(PayloadHeader) == 12U", "R337",
             "MAVLink FTP header must match PX4/QGC"),
            ("sizeof(Payload) ==", "R337",
             "MAVLink FTP full payload contract is missing"),
            ("kCmdOpenFileRO", "R337", "OpenFileRO support is missing"),
            ("kCmdReadFile", "R337", "ReadFile hole repair is missing"),
            ("kCmdBurstReadFile", "R337", "BurstReadFile support is missing"),
            ("kCmdResetSessions", "R337", "ResetSessions support is missing"),
            ("kCmdTerminateSession", "R337",
             "TerminateSession support is missing"),
            ("kMaxTxRetries = 4U", "R337",
             "FTP transient TX retries must remain bounded"),
            ("kSessionTimeoutUs = 10000000ULL", "R337",
             "stale FTP sessions need a ten-second timeout"),
            ("ErrorCode reset_sessions()", "R337",
             "ResetSessions must retain PX4 reset-all semantics"),
        ),
        violations,
    )
    ftp_implementation_path = (
        ROOT / "Dima/modules/mavlink/MavlinkMetadataFtp.cpp"
    )
    require_literals(
        ftp_implementation_path,
        (
            ("reply.header.burst_complete = burst ? 1U : 0U", "R337",
             "single-packet bursts must explicitly complete"),
            ("path_length == request.header.size", "R337",
             "virtual Metadata paths must use exact length matching"),
            ("session_owned_by(key)", "R337",
             "FTP sessions must remain bound to the requester"),
            ("response.target_system = key.source_system", "R337",
             "FTP replies must be directed back to the request source"),
            ("if (error == EAGAIN &&", "R337",
             "only definitely unsent FTP frames may be actively retried"),
            ("reply_.valid && same_request", "R337",
             "duplicate FTP requests must replay an identical response"),
            ("if (reply_.pending)", "R337",
             "an unsent FTP response must not be overwritten"),
            ("request_fingerprint(request.payload", "R337",
             "duplicate FTP keys must cover the complete request payload"),
        ),
        violations,
    )
    ftp_text = ftp_implementation_path.read_text(encoding="utf-8")
    handled_ftp_opcodes = set(re.findall(
        r"case\s+(kCmd[A-Za-z0-9_]+)\s*:", ftp_text
    ))
    expected_ftp_opcodes = {
        "kCmdNone", "kCmdTerminateSession", "kCmdResetSessions",
        "kCmdOpenFileRO", "kCmdReadFile", "kCmdBurstReadFile",
    }
    if handled_ftp_opcodes != expected_ftp_opcodes:
        violations.append(Violation(
            ftp_implementation_path, 1, "R337",
            "read-only FTP opcode surface changed: "
            f"{sorted(handled_ftp_opcodes)}",
        ))
    require_literals(
        ROOT / "Dima/adapters/usb_console/UsbConsole.cpp",
        (("kTxCapacity = 280U", "R337",
          "USB TX staging cannot fit a maximum MAVLink2 FTP frame"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/HeartbeatPacer.hpp",
        (
            ("kPx4CustomModeManual = 1UL << 16", "R336",
             "PX4 Manual custom_mode encoding changed"),
            ("kPx4CustomModeTermination = 10UL << 16", "R336",
             "PX4 Termination custom_mode encoding changed"),
            ("SubscriptionData<vehicle_control_mode_s>", "R336",
             "HEARTBEAT must project the published control mode"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/HeartbeatPacer.cpp",
        (("MAV_MODE_FLAG_MANUAL_INPUT_ENABLED", "R336",
          "Manual HEARTBEAT base_mode flag is missing"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkIdentity.hpp",
        (("MAV_AUTOPILOT_VALUE = 12", "R336",
          "stock QGC compatibility requires MAV_AUTOPILOT_PX4"),),
        violations,
    )
    require_literals(
        ROOT / "Dima/middleware/parameters/definitions/rc_params.c",
        (
            ("PARAM_DEFINE_INT32(RC_MAP_ROLL, 0);", "R336",
             "Rover Roll mapping must default unassigned"),
            ("PARAM_DEFINE_INT32(RC_MAP_PITCH, 0);", "R336",
             "Rover Pitch mapping must default unassigned"),
            ("PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 1);", "R336",
             "Rover Throttle mapping must default to channel 1"),
            ("PARAM_DEFINE_INT32(RC_MAP_YAW, 2);", "R336",
             "Rover Yaw mapping must default to channel 2"),
            ("Dima has no selectable flight modes", "R336",
             "RC_MAP_FLTMODE must remain an explicit disabled placeholder"),
            ("/** Arm 开关触发阈值；负值表示反向比较。", "R336",
             "Arm switch polarity metadata is missing"),
        ),
        violations,
    )
    require_literals(
        mavlink_parameters_path,
        (
            ("kQgcFixedInt32Parameters", "R336",
             "fixed QGC INT32 registry is missing"),
            ("is_qgc_fixed_parameter(name)", "R336",
             "QGC compatibility parameters must be active before LIST"),
            ("return value == fixed->value;", "R336",
             "unsupported QGC INT32 writes must be rejected"),
            ("std::strcmp(name, \"RC_MAP_FLTMODE\") == 0", "R336",
             "disabled flight-mode mapping compatibility guard is missing"),
            ("return mapping == 0;", "R336",
             "flight-mode mapping writes must remain disabled"),
            ("std::strcmp(name, \"RC_PORT_CONFIG\") == 0", "R336",
             "legacy RC_PORT_CONFIG must remain write-protected"),
            ("return protocol == 0 || protocol == 2;", "R336",
             "RC_INPUT_PROTO writes must be limited to Disabled or SBUS"),
            ("serial_baud_parameter(name)", "R336",
             "SERIALx baud parameters must be active before LIST"),
            ("serial_function_parameter(name)", "R336",
             "SERIALx function parameters must be active before LIST"),
            ("supported_serial_baud(baudrate)", "R336",
             "serial baud writes must be limited to implemented rates"),
            ("serial_function_write_allowed(name, function)", "R336",
             "serial function writes must preserve single RC ownership"),
            ("is_internal_parameter(name)", "R336",
             "migration-only parameters must be hidden from named requests"),
            ("param_foreach(&MavlinkParameters::append_used_parameter",
             "R331", "PARAM_REQUEST_LIST must snapshot the used set"),
            ("msg.param_count = count;", "R331",
             "LIST PARAM_VALUE frames must use the frozen count"),
            ("msg.param_index = index;", "R331",
             "LIST PARAM_VALUE frames must use the frozen index"),
            ("_send_all_snapshot[requested_index]", "R331",
             "indexed PARAM_REQUEST_READ must use the latest LIST snapshot"),
            ("const int snapshot_index = parameter_snapshot_index(param);",
             "R331", "READ/SET replies must reuse frozen LIST metadata"),
            ("void MavlinkParameters::stop_parameter_stream() noexcept", "R331",
             "LIST completion must stop only the send cursor"),
            ("void MavlinkParameters::clear_parameter_snapshot() noexcept", "R331",
             "Runtime and hash reset must invalidate the LIST snapshot"),
        ),
        violations,
    )
    for forbidden in (
            "FixedFloatParameter", "kQgcFixedFloatParameters",
            "RTL_RETURN_ALT", "RTL_DESCEND_ALT", "RTL_LAND_DELAY",
            "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3",
            "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6"):
        if forbidden in mavlink_parameters_text:
            violations.append(Violation(
                mavlink_parameters_path,
                line_for(mavlink_parameters_text, forbidden), "R336",
                f"unimplemented mode parameter path remains: {forbidden}",
            ))
    require_literals(
        ROOT / "Dima/modules/mavlink/MavlinkService.cpp",
        (
            ("param_find(\"MAV_SYS_ID\")", "R336",
             "fixed MAVLink system ID must have a runtime consumer"),
            ("system_id == 1", "R336",
             "MAVLink system ID must remain fixed at one"),
        ),
        violations,
    )
    require_literals(
        ROOT / "Dima/modules/rc/RcManualInput.cpp",
        (
            ("parameter_update_subscription_.update()", "R336",
             "RC switch mapping changes must be observed"),
            ("reset_switch_baseline();", "R336",
             "RC switch configuration changes must establish a safe baseline"),
            ("action_request_s::ACTION_ARM", "R336",
             "two-position Arm ON edge is missing"),
            ("action_request_s::ACTION_DISARM", "R336",
             "two-position Arm OFF edge is missing"),
        ),
        violations,
    )
    rc_manual_path = ROOT / "Dima/modules/rc/RcManualInput.cpp"
    rc_manual_text = rc_manual_path.read_text(encoding="utf-8")
    for forbidden in ("COM_FLTMODE", "SOURCE_RC_MODE_SLOT",
                      "ACTION_SWITCH_MODE", "flight_modes_"):
        if forbidden in rc_manual_text:
            violations.append(Violation(
                rc_manual_path, line_for(rc_manual_text, forbidden), "R336",
                "unimplemented RC flight-mode action path remains",
            ))
    require_literals(
        ROOT / "Dima/modules/safety/Commander.cpp",
        (
            ("case vehicle_command_s::NAV_CMD_PREFLIGHT_CALIBRATION:",
             "R335", "Commander must arbitrate RC calibration"),
            ("vehicle_status_.rc_calibration_in_progress = true;", "R335",
             "Commander must publish RC calibration state"),
            ("param_find(\"NAV_RCL_ACT\")", "R336",
             "RC-loss action compatibility parameter has no consumer"),
            ("param_find(\"NAV_DLL_ACT\")", "R336",
             "data-link-loss compatibility parameter has no consumer"),
            ("kRcLossActionDisarm", "R336",
             "RC loss must remain fixed to Disarm"),
            ("kDataLinkLossActionDisabled", "R336",
             "GCS loss must remain disabled"),
        ),
        violations,
    )
    commander_path = ROOT / "Dima/modules/safety/Commander.cpp"
    commander_text = commander_path.read_text(encoding="utf-8")
    for forbidden in ("ACTION_SWITCH_MODE", "SOURCE_RC_MODE_SLOT",
                      "switch_mode("):
        if forbidden in commander_text:
            violations.append(Violation(
                commander_path, line_for(commander_text, forbidden), "R336",
                "Commander must not claim an unimplemented selectable mode",
            ))
    require_literals(
        ROOT / "Dima/modules/rc/RCUpdate.cpp",
        (("assign(Mapping::Flaps, rc_channels_s::FUNCTION_FLAPS);", "R335",
          "RC_MAP_FLAPS must have a production mapping consumer"),),
        violations,
    )
    rc_update_path = ROOT / "Dima/modules/rc/RCUpdate.cpp"
    rc_update_text = rc_update_path.read_text(encoding="utf-8")
    for forbidden in ("RC_MAP_FLTMODE", "Mapping::FlightMode", "mode_slot()"):
        if forbidden in rc_update_text:
            violations.append(Violation(
                rc_update_path, line_for(rc_update_text, forbidden), "R336",
                "disabled mode compatibility parameter entered RC runtime",
            ))

    lock_path = ROOT / "tools/mavlink/mavlink.lock.json"
    try:
        lock = json.loads(lock_path.read_text(encoding="utf-8"))
        if lock["dialect"]["message_count"] != 24:
            raise ValueError("message_count is not 24")
        forbidden = set(lock["dialect"]["forbidden_messages"])
        if forbidden != {"COMPONENT_INFORMATION_BASIC"}:
            raise ValueError("forbidden message set changed")
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        violations.append(Violation(
            lock_path, 1, "R334", f"invalid MAVLink lock contract: {error}",
        ))


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

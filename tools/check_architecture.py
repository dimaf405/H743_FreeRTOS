#!/usr/bin/env python3
"""Enforce the Dima platform dependency and hardware-access boundaries."""

from __future__ import annotations

import pathlib
import re
import sys
from collections.abc import Iterable


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE_SUFFIXES = {".c", ".h", ".cpp", ".hpp"}
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


def sources_under(relative_roots: Iterable[str]) -> list[pathlib.Path]:
    files: set[pathlib.Path] = set()
    for relative in relative_roots:
        base = ROOT / relative
        if not base.exists():
            continue
        for path in base.rglob("*"):
            if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES:
                if is_vendored(path):
                    continue
                files.add(path)
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
            ("if (!raw && !config::enabled(source, level))", "R187",
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
            ("STATE_SAFE_OFF = 1U", "R151",
             "safe-off output state is missing"),
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
        ),
        ROOT / "Dima/platform/api/Platform.hpp": (
            ("virtual bool destroy(TaskHandle handle) noexcept", "R067",
             "TaskRuntime destroy must report failure"),
        ),
        ROOT / "Dima/platform/freertos/Backend.cpp": (
            ("native == xTaskGetCurrentTaskHandle()", "R068",
             "TaskRuntime must reject deleting the current task"),
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
        ROOT / "Dima/modules/rc/SbusRc.cpp": (
            ("protocol == 0 || (protocol == 2 && port == 0)", "R209",
             "disabled RC protocol must remain a normal lifecycle state"),
            ("schedule_signal_timeout()", "R210",
             "SBUS signal loss logging must follow the RC timeout"),
            ("signal lost last_frame_us=", "R211",
             "SBUS signal loss state log is missing"),
            ("SBUS release failed; UART normal configuration not restored",
             "R219", "SBUS module does not surface UART restore failure"),
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
            if (path.is_relative_to(ROOT / "Dima") and any(
                    token in line for token in (
                        "Mixer", "MixingOutput", "FunctionMotors",
                        "ActuatorOutput",
                    ))):
                violations.append(Violation(
                    path, line_number, "R122",
                    "an unauthorized actuator consumer is present",
                ))
            if (path.is_relative_to(ROOT / "Dima") and
                    "RoverDifferential" in line and
                    relative not in allowed_rover_differential_owners):
                violations.append(Violation(
                    path, line_number, "R122",
                    "RoverDifferential escaped the Rover control boundary",
                ))
            if (path.is_relative_to(ROOT / "Dima") and
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
        ),
        ROOT / "Dima/modules/boot_health/BootHealthService.cpp": (
            ("motor_output_->state()", "R176",
             "BootHealth does not monitor MotorOutput lifecycle"),
            ("confirmation_state_safe()", "R177",
             "BootHealth does not reset its window for unsafe vehicle state"),
            ("actuator_output_status_s::STATE_SAFE_OFF", "R178",
             "BootHealth does not require an observed safe-off output"),
            ("status.active_output_mask != 0U", "R179",
             "BootHealth does not reject active PWM channels"),
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
    }
    text = "\n".join(lines)
    for required_text, rule in required.items():
        if required_text not in text:
            violations.append(Violation(
                path, 1, rule,
                f"missing isolated-build contract '{required_text}'",
            ))


def scan_include_style(violations: list[Violation]) -> None:
    """R300: no Dima/... full-path includes; R301: cross-dir uses layer root."""
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


def scan_namespace_convention(violations: list[Violation]) -> None:
    """R310: C++ sources under Dima/ must live inside a known namespace.

    Accepted patterns:
      - namespace dima / dima::*
      - namespace px4 (upstream middleware)
      - namespace uORB (upstream messaging)
      - anonymous namespace { … }
      - extern "C" linkage wrappers (application entry points)
      - pure C-header wrappers (#ifdef __cplusplus extern "C")
    """
    known_ns_re = re.compile(
        r"^\s*namespace\s+(?:dima\b|dima::|px4\b|uORB\b)", re.MULTILINE,
    )
    anon_ns_re = re.compile(r"^\s*namespace\s*\{", re.MULTILINE)
    extern_c_re = re.compile(r'^\s*extern\s+"C"', re.MULTILINE)
    qualified_ns_re = re.compile(
        r"\b(?:dima|px4|uORB)::", re.MULTILINE,
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
        text = path.read_text(encoding="utf-8")
        if (known_ns_re.search(text) or anon_ns_re.search(text)
                or extern_c_re.search(text)
                or qualified_ns_re.search(text)):
            continue
        violations.append(Violation(
            path, 1, "R310",
            "C++ source does not declare a known namespace "
            "(dima, px4, uORB) or extern \"C\"",
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
    scan_runtime_contracts(violations)
    scan_fault_ownership(violations)
    scan_clock_contract(violations)
    scan_active_actuator_contract(violations)
    scan_linker_contract(violations)
    scan_include_style(violations)
    scan_namespace_convention(violations)
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

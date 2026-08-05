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
                files.add(path)
    return sorted(files)


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
            ("USB debug logging ready", "R049",
             "USB debug logger startup record is missing"),
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
        ),
    }
    for path, required in requirements.items():
        require_literals(path, required, violations)

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


def scan_inactive_actuator_contract(violations: list[Violation]) -> None:
    allowed_motor_owners = {
        "Boards/H743/Inc/motor_pwm.h",
        "Boards/H743/Src/motor_pwm.c",
    }
    allowed_rover_differential_owners = {
        "Dima/rover/ApplicationContext.cpp",
        "Dima/rover/ApplicationContext.hpp",
        "Dima/rover/control/RoverDifferential.cpp",
        "Dima/rover/control/RoverDifferential.hpp",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if (relative not in allowed_motor_owners and
                    re.search(r"\bboard_motor_pwm_(?:start|write)\s*\(", line)):
                violations.append(Violation(
                    path, line_number, "R120",
                    "actuator PWM has a consumer before Rover control stage",
                ))
            if (relative not in allowed_motor_owners and
                    re.search(r"\bHAL_TIM(?:Ex)?_PWMN?_Start\s*\(", line)):
                violations.append(Violation(
                    path, line_number, "R121",
                    "PWM start is outside the dormant board backend",
                ))
            if (path.is_relative_to(ROOT / "Dima") and any(
                    token in line for token in (
                        "MixingOutput", "FunctionMotors", "ActuatorOutput",
                    ))):
                violations.append(Violation(
                    path, line_number, "R122",
                    "actuator consumer exists before the authorized stage",
                ))
            if (path.is_relative_to(ROOT / "Dima") and
                    "RoverDifferential" in line and
                    relative not in allowed_rover_differential_owners):
                violations.append(Violation(
                    path, line_number, "R122",
                    "RoverDifferential escaped the Rover control boundary",
                ))

    timer_source = ROOT / "Core/Src/tim.c"
    require_literals(
        timer_source,
        (("sConfigOC.Pulse = 0;", "R123",
          "CubeMX PWM compare defaults must remain zero"),),
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


def main() -> int:
    violations: list[Violation] = []
    scan_include_directions(violations)
    scan_layer_dependencies(violations)
    scan_hardware_ownership(violations)
    scan_build_isolation(violations)
    scan_rover_root_contract(violations)
    scan_debug_console_contract(violations)
    scan_phase5_message_contracts(violations)
    scan_runtime_contracts(violations)
    scan_fault_ownership(violations)
    scan_clock_contract(violations)
    scan_inactive_actuator_contract(violations)
    scan_linker_contract(violations)
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

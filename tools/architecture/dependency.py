"""Dependency, include, namespace, and hardware ownership checks."""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    SOURCE_SUFFIXES,
    PROTECTED_ROOTS,
    FREERTOS_ROOT,
    STM32_ROOT,
    COMMON_INCLUDE_ROOTS,
    ALLOWED_LAYER_DEPENDENCIES,
    INCLUDE_RE,
    PROTECTED_API_RE,
    FREERTOS_VENDOR_API_RE,
    RTOS_API_RE,
    CACHE_OPERATION_RE,
    HAL_DMA_RE,
    HAL_FLASH_RE,
    Violation,
    sources_under,
    is_vendored,
    first_party_sources,
    low_level_include,
    vendor_include,
    freertos_include,
    protected_layer,
    resolve_common_include,
    transitive_variable_block,
    variable_block,
    require_literals,
    strip_cpp_structure,
    MAKE_CONTRACT_PATHS,
    owner_texts,
)


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
                    "ActuatorPwm.hpp", "ActuatorPwmLimits.h",
                    "BoardIdentity.hpp", "Boot.hpp",
                    "Console.hpp", "Execution.hpp", "Flash.hpp",
                    "Memory.hpp", "ParameterFileStore.hpp",
                    "PlatformTypes.hpp", "SensorInterrupts.hpp",
                    "Serial.hpp", "Services.hpp", "Synchronization.hpp",
                    "TaskRuntime.hpp", "Time.hpp", "platform_config.h",
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
        "Dima/platform/stm32h7/memory/cache.c",
        "Dima/platform/stm32h7/memory/early_memory.c",
    }
    dma_owners = {
        "Dima/platform/stm32h7/serial/SbusUart.cpp",
    }
    flash_owners = {
        "Dima/platform/stm32h7/flash/FlashDevice.cpp",
        "Dima/platform/stm32h7/flash/flash_bank1.c",
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

def scan_build_isolation(violations: list[Violation]) -> None:
    path = MAKE_CONTRACT_PATHS[0]
    make_owners = owner_texts(MAKE_CONTRACT_PATHS)
    checks = {
        "DIMA_COMMON_INCLUDES": (
            "R030",
            re.compile(r"(?:^|\s)-I\.(?:\s|\\|$)|Core/|Boards/|USB_DEVICE/|"
                       r"Drivers/|Middlewares/|FatFs|FreeRTOS|CMSIS|STM32"),
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
        owner_blocks = [
            (
                owner,
                transitive_variable_block(text.splitlines(), name)
                if name == "DIMA_COMMON_INCLUDES"
                else variable_block(text.splitlines(), name),
            )
            for owner, text in make_owners
        ]
        owner_blocks = [
            (owner, block) for owner, block in owner_blocks if block
        ]
        if not owner_blocks:
            violations.append(Violation(
                path, 1, rule, f"missing build include set {name}",
            ))
            continue
        for owner, block in owner_blocks:
            for line_number, line in block:
                if pattern.search(line):
                    violations.append(Violation(
                        owner, line_number, rule, message,
                    ))

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
    for required_text, rule in required.items():
        if not any(required_text in text for _owner, text in make_owners):
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
        "Dima/modules/mavlink/MavlinkChannelState.cpp",
        "Dima/middleware/logging/logging.cpp",
        "Dima/middleware/logging/logging.hpp",
        "Dima/middleware/parameters/autosave.cpp",
        "Dima/middleware/parameters/flashparams/flashparams.cpp",
        "Dima/middleware/parameters/param.cpp",
        "Dima/middleware/parameters/param_storage.cpp",
        "Dima/middleware/perf/perf_counter.cpp",
        "Dima/platform/api/Time.hpp",
        "Dima/platform/freertos/Backend.cpp",
        "Dima/platform/freertos/BackendTimeout.hpp",
        "Dima/platform/freertos/HeapOperators.cpp",
        "Dima/platform/stm32h7/system/Clock.cpp",
        "Dima/platform/stm32h7/memory/DmaMemory.cpp",
        "Dima/platform/stm32h7/flash/FlashDevice.cpp",
        "Dima/platform/stm32h7/serial/SbusUart.cpp",
        "Dima/platform/stm32h7/io/SensorInterrupts.cpp",
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

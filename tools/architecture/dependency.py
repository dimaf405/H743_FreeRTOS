"""依赖方向、include、时钟、命名空间与硬件能力唯一所有权门禁。"""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    PROTECTED_ROOTS,
    FREERTOS_ROOT,
    STM32_ROOT,
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
    is_px4_upstream_algorithm,
    is_px4_middleware_compat,
    first_party_sources,
    low_level_include,
    vendor_include,
    freertos_include,
    protected_layer,
    resolve_common_include,
    require_literals,
    strip_cpp_structure,
    MAKE_CONTRACT_PATHS,
    owner_texts,
)
from architecture.build_closure import BuildClosureError, load_build_closure


def scan_include_directions(violations: list[Violation]) -> None:
    """解析第一方 include 目标并按 ALLOWED_LAYER_DEPENDENCIES 拒绝反向依赖。"""
    roots = PROTECTED_ROOTS + (FREERTOS_ROOT, STM32_ROOT)
    for path in sources_under(roots):
        # PX4 算法继续按真实 include 逐项检查，只把源端归入受限的上游算法层；
        # 这样可允许其官方兼容头，同时仍拒绝对 driver/module/rover 的反向依赖。
        source_layer = (
            "px4-upstream-algorithm"
            if is_px4_upstream_algorithm(path)
            else protected_layer(path)
        )
        if source_layer is None:
            continue
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if not match:
                continue
            target = resolve_common_include(path, match.group(1))
            target_layer = protected_layer(target) if target else None
            middleware_escape = (
                source_layer == "px4-upstream-algorithm"
                and target_layer == "middleware"
                and target is not None
                and not is_px4_middleware_compat(target)
            )
            if (not middleware_escape and
                    (target_layer is None or target_layer in
                     ALLOWED_LAYER_DEPENDENCIES[source_layer])):
                continue
            target_relative = target.relative_to(ROOT).as_posix()
            violations.append(Violation(
                path, line_number, "R004",
                f"{source_layer} layer includes disallowed "
                f"{target_layer} header '{target_relative}'",
            ))


def scan_layer_dependencies(violations: list[Violation]) -> None:
    """拒绝受保护层直接包含或调用 FreeRTOS、CMSIS、HAL、CubeMX 等底层接口。"""
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
                if include not in allowed:
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
    """核对 Flash、DMA、cache、USB 等硬件 API 只存在于指定平台后端。"""
    cache_owners = {
        "Dima/platform/stm32h7/memory/cache.c",
        "Dima/platform/stm32h7/memory/early_memory.c",
    }
    dma_owners = {
        "Dima/platform/stm32h7/serial/UartTimestampedRxEndpoint.cpp",
        "Dima/platform/stm32h7/serial/UartDuplexDmaEndpoint.cpp",
        "Dima/platform/stm32h7/spi/Spi4.cpp",
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

def scan_device_policy_boundaries(violations: list[Violation]) -> None:
    """隔离设备/协议策略、具体驱动和供应商 ABI；R024 有意连注释 token 一并扫描。"""
    device_policy_re = re.compile(
        r"\b(?:sbus|gps|um982|icm42688p?|dronecan|rm3100|mavlink)\b",
        re.IGNORECASE,
    )
    for path in sources_under((STM32_ROOT,)):
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            if device_policy_re.search(line):
                violations.append(Violation(
                    path, line_number, "R024",
                    "STM32H7 backend contains a device/protocol policy token",
                ))

    for path in sources_under(("Dima/modules/sensors",)):
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if match is None:
                continue
            target = resolve_common_include(path, match.group(1))
            if (target is not None and
                    target.is_relative_to(ROOT / "Dima/drivers")):
                violations.append(Violation(
                    path, line_number, "R025",
                    "sensor frontend includes a concrete device driver",
                ))

    vendor_include_re = re.compile(
        r"(?:^|/)(?:canard\.h|uavcan[._/][^/]*\.h)$", re.IGNORECASE,
    )
    vendor_abi_re = re.compile(
        r"\b(?:Canard(?:Instance|RxTransfer|TransferType)|"
        r"uavcan_[A-Za-z0-9_]+)\b"
    )
    for path in sources_under(PROTECTED_ROOTS):
        if path.suffix.lower() not in {".h", ".hpp"}:
            continue
        if (path.is_relative_to(ROOT / FREERTOS_ROOT) or
                path.is_relative_to(ROOT / STM32_ROOT)):
            continue
        text = path.read_text(encoding="utf-8")
        for line_number, line in enumerate(text.splitlines(), 1):
            match = INCLUDE_RE.match(line)
            include = match.group(1) if match is not None else ""
            if vendor_include_re.search(include) or vendor_abi_re.search(line):
                violations.append(Violation(
                    path, line_number, "R026",
                    "public first-party header leaks a DroneCAN vendor ABI",
                ))

def scan_build_isolation(violations: list[Violation]) -> None:
    """从真实 Make 闭包验证目标私有 include 上下文，防止供应商头泄漏到应用层。"""
    path = MAKE_CONTRACT_PATHS[0]
    make_owners = owner_texts(MAKE_CONTRACT_PATHS)
    for forbidden_name in ("DIMA_COMMON_INCLUDES",
                           "DIMA_GENERATED_INCLUDES"):
        for owner, text in make_owners:
            if forbidden_name in text:
                violations.append(Violation(
                    owner, 1, "R030",
                    f"build restores forbidden include union "
                    f"{forbidden_name}",
                ))

    try:
        closure = load_build_closure(ROOT)
    except BuildClosureError as error:
        violations.append(Violation(
            path, 1, "R033",
            f"cannot evaluate isolated object contexts: {error}",
        ))
    else:
        for unit in closure.units:
            source_path = ROOT / unit.source
            owner_path = (
                source_path if source_path.is_file()
                else path if unit.owner == "application"
                else ROOT / "Bootloader/Makefile"
            )
            broad = sorted({
                include for include in unit.includes
                if include in {".", "Dima"}
            })
            if broad:
                violations.append(Violation(
                    owner_path, 1, "R030",
                    f"effective compile context exposes broad include roots "
                    f"{broad}",
                ))
            duplicates = sorted({
                include for include in unit.includes
                if unit.includes.count(include) > 1
            })
            if duplicates:
                violations.append(Violation(
                    owner_path, 1, "R030",
                    f"effective compile context repeats include roots "
                    f"{duplicates}",
                ))

            if unit.source.startswith("Dima/platform/freertos/"):
                forbidden = sorted({
                    include for include in unit.includes
                    if include.startswith((
                        "Core/", "Boards/", "USB_DEVICE/", "Drivers/",
                        "Dima/platform/stm32h7",
                    ))
                    or "cmsis" in include.lower()
                    or "stm32" in include.lower()
                })
                if forbidden:
                    violations.append(Violation(
                        owner_path, 1, "R031",
                        f"FreeRTOS object exposes MCU include roots {forbidden}",
                    ))

            if unit.source.startswith("Dima/platform/stm32h7/"):
                forbidden = sorted({
                    include for include in unit.includes
                    if "freertos" in include.lower()
                    or include.startswith("Dima/platform/freertos")
                })
                if forbidden:
                    violations.append(Violation(
                        owner_path, 1, "R032",
                        f"STM32 object exposes RTOS include roots {forbidden}",
                    ))

    required = {
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


def scan_namespace_convention(violations: list[Violation]) -> None:
    """R310：除显式 C ABI/转发白名单外，声明必须位于允许的最外层命名空间。"""
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
        "Dima/middleware/uORB/Publication.hpp",
        "Dima/middleware/uORB/SubscriptionData.hpp",
        "Dima/middleware/work_queue/ScheduledWorkItem.hpp",
    }
    # 这两个文件逐字节同步 PX4 v1.17 MessageFormatReader；上游把 namespace
    # 名称与左花括号分成两行，不能仅为本地风格门禁改写原件。
    upstream_namespace_files = {
        "Dima/middleware/uORB/uORBMessageFields.cpp",
        "Dima/middleware/uORB/uORBMessageFields.hpp",
    }
    c_abi_files = {
        "Dima/adapters/usb_console/UsbConsole.cpp",
        "Dima/application/app_bootstrap.cpp",
        "Dima/application/app_main.cpp",
        "Dima/lib/tinybson/tinybson.cpp",
        "Dima/adapters/mavlink/MavlinkChannelState.cpp",
        "Dima/middleware/logging/logging.cpp",
        "Dima/middleware/logging/logging.hpp",
        # PX4 生成模板要求 orb_metadata 与打印入口位于全局 C ABI；
        # Runtime 内部仍保持在 uORB 命名空间，此处只放行这两个薄边界文件。
        "Dima/middleware/uORB/uORB.cpp",
        "Dima/middleware/uORB/uORB.hpp",
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
        "Dima/platform/stm32h7/serial/UartTimestampedRxEndpoint.cpp",
        "Dima/platform/stm32h7/serial/UartDuplexDmaEndpoint.cpp",
        "Dima/platform/stm32h7/serial/UartIrqRouter.cpp",
        "Dima/platform/stm32h7/interrupts/SensorInterrupts.cpp",
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
        # matrix/mathlib/EKF 等上游文件原生使用全局、matrix、math 与 estimator
        # 命名空间；逐文件改包裹会偏离 PX4 公式和类型 ABI，因此只豁免该闭包。
        if is_px4_upstream_algorithm(path):
            continue
        if rel in upstream_namespace_files:
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
    """R320/R321：MavlinkService 是应用层 Console 数据面和能力引用的唯一 owner。"""
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

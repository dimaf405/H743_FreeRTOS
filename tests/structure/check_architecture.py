#!/usr/bin/env python3
"""Structural contracts for the post-CubeMX static application architecture."""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]
MAIN_C = ROOT / "Core/Src/main.c"

PERIPHERAL_CONTRACTS = {
    "gpio": (("MX_GPIO_Init",), ()),
    "dma": (("MX_DMA_Init",), ()),
    "fdcan": (("MX_FDCAN1_Init",), (("FDCAN_HandleTypeDef", "hfdcan1"),)),
    "i2c": (("MX_I2C2_Init",), (("I2C_HandleTypeDef", "hi2c2"),)),
    "sdmmc": (("MX_SDMMC1_SD_Init",), (("SD_HandleTypeDef", "hsd1"),)),
    "spi": (
        ("MX_SPI4_Init",),
        (
            ("SPI_HandleTypeDef", "hspi4"),
            ("DMA_HandleTypeDef", "hdma_spi4_rx"),
            ("DMA_HandleTypeDef", "hdma_spi4_tx"),
        ),
    ),
    "tim": (
        ("MX_TIM5_Init", "MX_TIM8_Init"),
        (("TIM_HandleTypeDef", "htim5"), ("TIM_HandleTypeDef", "htim8")),
    ),
    "usart": (
        (
            "MX_UART4_Init",
            "MX_UART5_Init",
            "MX_UART7_Init",
            "MX_UART8_Init",
            "MX_USART1_UART_Init",
            "MX_USART2_UART_Init",
            "MX_USART3_UART_Init",
            "MX_USART6_UART_Init",
        ),
        tuple(("UART_HandleTypeDef", name) for name in (
            "huart4", "huart5", "huart7", "huart8",
            "huart1", "huart2", "huart3", "huart6",
        )),
    ),
}

BOARD_INIT_CALL_ORDER = (
    "MX_GPIO_Init",
    "MX_DMA_Init",
    "MX_FDCAN1_Init",
    "MX_I2C2_Init",
    "MX_SDMMC1_SD_Init",
    "MX_SPI4_Init",
    "MX_UART4_Init",
    "MX_UART5_Init",
    "MX_UART7_Init",
    "MX_UART8_Init",
    "MX_USART1_UART_Init",
    "MX_USART2_UART_Init",
    "MX_USART3_UART_Init",
    "MX_USART6_UART_Init",
    "MX_TIM5_Init",
    "MX_TIM8_Init",
)


def strip_comments(source: str) -> str:
    """Retain source tokens while removing C/C++ comments before searching."""
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def check(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def function_body(source: str, function_name: str) -> str | None:
    """Return a balanced C function body, excluding its outer braces."""
    match = re.search(
        rf"\b(?:static\s+)?(?:void|int|bool|u?int(?:8|16|32)_t)\s+"
        rf"{re.escape(function_name)}\s*"
        rf"\([^;{{}}]*\)\s*\{{",
        source,
    )
    if match is None:
        return None
    opening = source.find("{", match.start())
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    return None


def user_code_block(source: str, marker: str) -> str | None:
    match = re.search(
        rf"/\*\s*USER CODE BEGIN {re.escape(marker)}\s*\*/(.*?)"
        rf"/\*\s*USER CODE END {re.escape(marker)}\s*\*/",
        source,
        flags=re.DOTALL,
    )
    return None if match is None else match.group(1)


def cpp_method_body(source: str, class_name: str, method_name: str) -> str | None:
    """Return a balanced out-of-line C++ method body."""
    match = re.search(
        rf"\b(?:void|bool|u?int(?:8|16|32|64)_t)\s+"
        rf"{re.escape(class_name)}::{re.escape(method_name)}\s*"
        rf"\([^;{{}}]*\)(?:\s+const)?(?:\s+noexcept)?\s*\{{",
        source,
    )
    if match is None:
        return None
    opening = source.find("{", match.start())
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    return None


def check_hello_world(failures: list[str]) -> None:
    header_path = ROOT / "Dima/modules/hello_world/hello_world.hpp"
    source_path = ROOT / "Dima/modules/hello_world/hello_world.cpp"
    project_path = ROOT / "make/project.mk"
    validator_path = ROOT / "tools/validate_hello_world_interval.py"
    host_makefile_path = ROOT / "tests/host/Makefile"
    host_runner_path = ROOT / "tests/run_host_tests.sh"
    required = (header_path, source_path, project_path, validator_path,
                host_makefile_path, host_runner_path)
    for path in required:
        check(path.is_file(),
              f"missing HelloWorld file: {path.relative_to(ROOT)}", failures)
    if not all(path.is_file() for path in required):
        return

    header_raw = header_path.read_text(encoding="utf-8", errors="replace")
    header = strip_comments(header_raw)
    check(re.search(
              r"class\s+HelloWorld\s+final\s*:\s*public\s+"
              r"dima::middleware::lifecycle::ModuleBase",
              header,
          ) is not None
          and "public dima::middleware::scheduling::ScheduledWorkItem" in header
          and "public px4::ScheduledWorkItem" in header,
          "HelloWorld must use the Dima lifecycle and schedulers", failures)
    protected_section = re.search(r"protected\s*:(.*?)(?:private\s*:|$)",
                                  header, flags=re.DOTALL)
    check(protected_section is not None and
          re.search(r"\bvoid\s+Run\s*\(\s*\)\s*override\s*;",
                    protected_section.group(1)) is not None,
          "HelloWorld::Run must be protected and non-public", failures)
    check("APP_HOST_TEST" in header and "HostDependencies" in header
          and "RunForTest" in header,
          "HelloWorld host seams must be guarded by APP_HOST_TEST", failures)
    check("std::function" not in header,
          "HelloWorld must not use std::function", failures)

    source = strip_comments(source_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    run_body = cpp_method_body(source, "HelloWorld", "Run")
    check(run_body is not None, "HelloWorld.cpp must define HelloWorld::Run", failures)
    if run_body is not None:
        production_branch = run_body.split("#else", 1)[1].split("#endif", 1)[0] \
            if "#else" in run_body and "#endif" in run_body else ""
        ordered_tokens = (
            '(void)printf("Hello World\\r\\n")',
            "(void)fflush(stdout)",
            "dima::platform::platform_time_us()",
            "heartbeat_publication_.publish",
        )
        positions = tuple(run_body.find(token) for token in ordered_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "HelloWorld Run must print, flush, sample time, then publish", failures)
        check('(void)printf("Hello World\\r\\n");' in production_branch
              and "(void)fflush(stdout);" in production_branch,
              "HelloWorld production Run must attempt the exact output calls",
              failures)
        check("usb_console_ready" not in run_body,
              "HelloWorld must not gate output or heartbeat on USB readiness",
              failures)
        check("Publication<app_heartbeat_s>" in header
              and "app_heartbeat_s" in run_body and "++sequence_" in run_body,
              "HelloWorld must strongly publish timestamped heartbeats from sequence 1",
              failures)
    for forbidden in (r"\bnew\b", r"\bdelete\b", r"std::function"):
        check(re.search(forbidden, source) is None,
              f"HelloWorld contains forbidden dynamic token {forbidden}", failures)

    project = strip_comments(project_path.read_text(encoding="utf-8"))
    check(project.count("Dima/modules/hello_world/hello_world.cpp") == 1,
          "make/project.mk must own hello_world.cpp exactly once", failures)
    for variable, default in (("APP_HELLO_WORLD_ENABLED", "1"),
                              ("APP_HELLO_WORLD_INTERVAL_MS", "1000")):
        check(re.search(rf"(?m)^{variable}\s*\?=\s*{default}\s*$", project)
              is not None,
              f"make/project.mk must default {variable} to {default}", failures)
    check("-DAPP_HELLO_WORLD_ENABLED=$(APP_HELLO_WORLD_ENABLED)" in project,
          "make/project.mk must pass APP_HELLO_WORLD_ENABLED to production",
          failures)
    check("APP_HELLO_WORLD_INTERVAL_RAW := "
          "$(value APP_HELLO_WORLD_INTERVAL_MS)" in project
          and "unexport APP_HELLO_WORLD_INTERVAL_MS" in project
          and "export APP_HELLO_WORLD_INTERVAL_RAW" in project,
          "make/project.mk must unexport the recursive input and export its raw value",
          failures)
    check("APP_HELLO_WORLD_INTERVAL_VALIDATED := "
          "$(shell APP_HELLO_WORLD_INTERVAL_RAW="
          "$(APP_HELLO_WORLD_INTERVAL_CANONICAL) "
          "$(PYTHON) tools/validate_hello_world_interval.py)" in project,
          "make/project.mk must validate only the Make-whitelisted canonical interval",
          failures)
    check(re.search(r"\$\(shell[^\n]*\$\(APP_HELLO_WORLD_INTERVAL_(?:MS|RAW)\)",
                    project) is None,
          "make/project.mk must not interpolate the raw interval into shell source",
          failures)
    check("APP_HELLO_WORLD_INTERVAL_NON_DIGITS" in project
          and "APP_HELLO_WORLD_INTERVAL_CANONICAL" in project,
          "make/project.mk must whitelist digits and canonicalize before shell use",
          failures)
    check("-DAPP_HELLO_WORLD_INTERVAL_MS="
          "$(APP_HELLO_WORLD_INTERVAL_VALIDATED)" in project,
          "make/project.mk must pass only the validated interval to production",
          failures)
    check("APP_HELLO_WORLD_ENABLED must be 0 or 1" in project,
          "make/project.mk must reject invalid HelloWorld enable values", failures)
    check("APP_HELLO_WORLD_INTERVAL_MS must be in 1..2147483647" in project,
          "make/project.mk must reject unsafe HelloWorld intervals", failures)

    validator = validator_path.read_text(encoding="utf-8", errors="replace")
    check("APP_HELLO_WORLD_INTERVAL_RAW" in validator and "[0-9]" in validator
          and "2147483647" in validator,
          "interval validator must enforce the raw ASCII/range contract", failures)
    for forbidden in ("subprocess", "os.system", "eval(", "exec("):
        check(forbidden not in validator,
              f"interval validator must not use {forbidden}", failures)

    host_makefile = host_makefile_path.read_text(encoding="utf-8")
    host_runner = host_runner_path.read_text(encoding="utf-8")
    check("hello-world-test" in host_makefile,
          "tests/host/Makefile must expose hello-world-test", failures)
    check("hello-world-test" in host_runner,
          "canonical host runner must execute hello-world-test", failures)


def check_boot_health(failures: list[str]) -> None:
    header_path = ROOT / "Dima/modules/boot_health/boot_health.hpp"
    source_path = ROOT / "Dima/modules/boot_health/boot_health.cpp"
    mcuboot_header_path = ROOT / "Dima/adapters/mcuboot/mcuboot_app.h"
    mcuboot_source_path = ROOT / "Dima/adapters/mcuboot/mcuboot_app.c"
    old_header_path = ROOT / "Core/Inc/mcuboot_app.h"
    old_source_path = ROOT / "Core/Src/mcuboot_app.c"
    project_path = ROOT / "make/project.mk"
    host_makefile_path = ROOT / "tests/host/Makefile"
    host_runner_path = ROOT / "tests/run_host_tests.sh"
    docs_path = ROOT / "docs/MCUBOOT_USB_RECOVERY_ZH.md"
    required = (header_path, source_path, mcuboot_header_path,
                mcuboot_source_path, project_path, host_makefile_path,
                host_runner_path, docs_path)
    for path in required:
        check(path.is_file(),
              f"missing BootHealth file: {path.relative_to(ROOT)}", failures)
    check(not old_header_path.exists() and not old_source_path.exists(),
          "legacy Core mcuboot_app files must be removed after service migration",
          failures)
    if not all(path.is_file() for path in required):
        return

    header = strip_comments(header_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    check(re.search(
              r"class\s+BootHealthService\s+final\s*:\s*public\s+"
              r"dima::middleware::lifecycle::ModuleBase",
              header,
          ) is not None
          and "public dima::middleware::scheduling::ScheduledWorkItem" in header
          and "public px4::ScheduledWorkItem" in header,
          "BootHealthService must use the Dima lifecycle and schedulers", failures)
    protected_section = re.search(r"protected\s*:(.*?)(?:private\s*:|$)",
                                  header, flags=re.DOTALL)
    check(protected_section is not None and
          re.search(r"\bvoid\s+Run\s*\(\s*\)\s*override\s*;",
                    protected_section.group(1)) is not None,
          "BootHealthService::Run must be protected and non-public", failures)
    check("APP_HOST_TEST" in header and "HostDependencies" in header
          and "RunForTest" in header,
          "BootHealth host seams must be hidden by APP_HOST_TEST", failures)
    check("Subscription<app_heartbeat_s>" in header,
          "BootHealth must own a strongly typed heartbeat subscription", failures)
    check(re.search(r"kCheckIntervalMs\s*=\s*100U", header) is not None,
          "BootHealth check interval must be 100 ms", failures)
    check(re.search(r"kStableWindowMs\s*=\s*5000(?:U|ULL)", header) is not None,
          "BootHealth stability window must be 5000 ms", failures)

    source = strip_comments(source_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    start_body = cpp_method_body(source, "BootHealthService", "start")
    run_body = cpp_method_body(source, "BootHealthService", "Run")
    check(start_body is not None and "time_ms_" in start_body
          and "ScheduleOnInterval(kCheckIntervalMs)" in start_body,
          "BootHealth start must sample time and schedule a 100 ms interval",
          failures)
    check(run_body is not None, "boot_health.cpp must define BootHealthService::Run",
          failures)
    if run_body is not None:
        ordered_tokens = (
            "heartbeat_subscription_.copy",
            "elapsed_ms",
            "confirmation_attempted_ = true",
            "ScheduleClear()",
            "confirm_running_image_",
        )
        positions = tuple(run_body.find(token) for token in ordered_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "BootHealth must copy heartbeat, check elapsed time, latch, clear, then confirm",
              failures)
        for result_name in (
            "MCUBOOT_CONFIRM_OK", "MCUBOOT_CONFIRM_ALREADY_CONFIRMED",
            "MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE", "MCUBOOT_CONFIRM_FLASH_ERROR",
        ):
            check(result_name in run_body,
                  f"BootHealth must handle {result_name}", failures)
        check("ModuleState::Error" in run_body and "default:" in run_body,
              "BootHealth flash and unknown confirmation results must become Error",
              failures)
    for forbidden in (
        r"\bprintf\s*\(", r"\bputs\s*\(", r"\bfflush\s*\(",
        r"\b_write\s*\(", r"usb_console", r"\bnew\b", r"\bdelete\b",
        r"std::function",
    ):
        check(re.search(forbidden, source) is None,
              f"BootHealth contains forbidden output/dynamic token {forbidden}",
              failures)

    mcuboot_header = strip_comments(mcuboot_header_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    check("__cplusplus" in mcuboot_header and 'extern "C"' in mcuboot_header,
          "migrated mcuboot_app.h must preserve C linkage", failures)
    mcuboot_source = strip_comments(mcuboot_source_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    for token in (
        "MCUBOOT_MAGIC_ADDRESS", "MCUBOOT_IMAGE_OK_ADDRESS", "memcmp",
        "HAL_FLASH_Unlock", "HAL_FLASH_Program", "HAL_FLASH_Lock",
        "__DSB", "__ISB",
    ):
        check(token in mcuboot_source,
              f"migrated mcuboot_app.c lost semantic token {token}", failures)

    project = strip_comments(project_path.read_text(encoding="utf-8"))
    for source_name in (
        "Dima/adapters/mcuboot/mcuboot_app.c",
        "Dima/modules/boot_health/boot_health.cpp",
    ):
        check(project.count(source_name) == 1,
              f"make/project.mk must own {source_name} exactly once", failures)
    for old_name in ("Core/Src/mcuboot_app.c", "Core/Inc/mcuboot_app.h"):
        check(old_name not in project,
              f"make/project.mk must reject legacy MCUboot path {old_name}", failures)

    host_makefile = host_makefile_path.read_text(encoding="utf-8")
    host_runner = host_runner_path.read_text(encoding="utf-8")
    check("boot-health-test" in host_makefile,
          "tests/host/Makefile must expose boot-health-test", failures)
    check("boot-health-test" in host_runner,
          "canonical host runner must execute boot-health-test", failures)
    docs = docs_path.read_text(encoding="utf-8", errors="replace")
    check("Dima/adapters/mcuboot/mcuboot_app.c" in docs
          and "Core/Src/mcuboot_app.c" not in docs,
          "MCUboot recovery docs must reference only the migrated source path",
          failures)
    check("app_heartbeat" in docs and "BootHealth" in docs
          and "不是 `vTaskDelay(5000)`" in docs,
          "MCUboot docs must describe heartbeat-gated BootHealth confirmation",
          failures)


def check_usb_console(failures: list[str]) -> None:
    """Check the static USB CDC console and its CubeMX-preserved wiring."""
    header_path = ROOT / "Dima/adapters/usb_console/usb_console.h"
    source_path = ROOT / "Dima/adapters/usb_console/usb_console.c"
    internal_path = ROOT / "Dima/adapters/usb_console/usb_console_internal.h"
    usb_device_path = ROOT / "USB_DEVICE/App/usb_device.c"
    cdc_path = ROOT / "USB_DEVICE/App/usbd_cdc_if.c"
    cdc_class_path = ROOT / \
        "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
    usb_target_path = ROOT / "USB_DEVICE/Target/usbd_conf.c"
    config_path = ROOT / "Core/Inc/FreeRTOSConfig.h"
    no_heap_path = ROOT / "Dima/platform/freertos/libc/no_heap.c"
    cpp_runtime_path = ROOT / "Dima/platform/freertos/libc/cpp_runtime.c"
    project_path = ROOT / "make/project.mk"
    host_makefile_path = ROOT / "tests/host/Makefile"
    host_runner_path = ROOT / "tests/run_host_tests.sh"

    required = (
        header_path, source_path, internal_path, usb_device_path, cdc_path,
        cdc_class_path, usb_target_path, config_path, no_heap_path,
        cpp_runtime_path, project_path,
        host_makefile_path, host_runner_path,
    )
    for path in required:
        check(path.is_file(), f"missing USB console file: {path.relative_to(ROOT)}",
              failures)
    if not all(path.is_file() for path in required):
        return

    header_raw = header_path.read_text(encoding="utf-8", errors="replace")
    header = strip_comments(header_raw)
    check("__cplusplus" in header and 'extern "C"' in header,
          "usb_console.h must preserve C linkage for C++ callers", failures)
    for include in ("stdbool.h", "stddef.h", "stdint.h"):
        check(re.search(rf"#\s*include\s*<{re.escape(include)}>", header) is not None,
              f"usb_console.h must include <{include}>", failures)
    public_declarations = (
        r"\bvoid\s+usb_console_init\s*\(\s*void\s*\)\s*;",
        r"\bbool\s+usb_console_ready\s*\(\s*void\s*\)\s*;",
        r"\bint\s+usb_console_write\s*\(\s*const\s+uint8_t\s*\*\s*\w+\s*,"
        r"\s*size_t\s+\w+\s*,\s*uint32_t\s+\w+\s*\)\s*;",
        r"\bvoid\s+usb_console_tx_complete_from_isr\s*\(\s*void\s*\)\s*;",
        r"\bint\s+_write\s*\(\s*int\s+\w+\s*,\s*char\s*\*\s*\w+\s*,"
        r"\s*int\s+\w+\s*\)\s*;",
    )
    for declaration in public_declarations:
        check(re.search(declaration, header) is not None,
              "usb_console.h is missing a fixed public C API declaration", failures)
    check("APP_HOST_TEST" in header and "usb_console_test_backend_t" in header
          and "usb_console_tx_result_t" in header
          and "usb_console_test_set_backend" in header,
          "USB portable seams must be guarded by APP_HOST_TEST", failures)
    check("wq:lp_default" in header_raw and "single-task" in header_raw
          and "FILE" in header_raw and "errno" in header_raw,
          "usb_console.h must document the single-task stdio/FILE/errno contract",
          failures)

    source_raw = source_path.read_text(encoding="utf-8", errors="replace")
    source = strip_comments(source_raw)
    check(re.search(r"\buint8_t\s+\w*[Ss]taging\w*\s*\[\s*256U?\s*\]", source)
          is not None,
          "usb_console.c must own one 256-byte static TX staging buffer", failures)
    check(len(re.findall(r"\bStaticSemaphore_t\s+\w+", source)) >= 2,
          "usb_console.c must own static mutex and completion semaphore storage",
          failures)
    for token in (
        "xSemaphoreCreateMutexStatic", "xSemaphoreCreateBinaryStatic",
        "xSemaphoreTake", "xSemaphoreGive", "xSemaphoreGiveFromISR",
        "portYIELD_FROM_ISR", "xTaskGetSchedulerState", "xPortIsInsideInterrupt",
    ):
        check(token in source, f"usb_console.c must use production primitive {token}",
              failures)
    check("setvbuf" in source and "_IONBF" in source,
          "usb_console_init must make stdout unbuffered", failures)
    for forbidden in (
        r"\bpvPortMalloc\s*\(", r"\bvPortFree\s*\(",
        r"\bxSemaphoreCreateMutex\s*\(", r"\bxSemaphoreCreateBinary\s*\(",
        r"\bxTaskCreate\s*\(", r"\bnew\b", r"\bdelete\b",
    ):
        check(re.search(forbidden, source) is None,
              f"usb_console.c contains forbidden dynamic token {forbidden}", failures)

    usb_device_raw = usb_device_path.read_text(encoding="utf-8", errors="replace")
    pre_treatment = user_code_block(usb_device_raw, "USB_DEVICE_Init_PreTreatment")
    check(pre_treatment is not None and "usb_console_init" in pre_treatment,
          "MX_USB_DEVICE_Init PreTreatment must call usb_console_init", failures)
    usb_init_body = function_body(strip_comments(usb_device_raw), "MX_USB_DEVICE_Init")
    check(usb_init_body is not None and
          0 <= usb_init_body.find("usb_console_init") < usb_init_body.find("USBD_Init"),
          "usb_console_init must run before USBD_Init can enable OTG IRQ", failures)
    usb_include_block = user_code_block(usb_device_raw, "Includes")
    check(usb_include_block is not None and "usb_console.h" in usb_include_block,
          "usb_device.c must include usb_console.h in a USER CODE block", failures)

    cdc_raw = cdc_path.read_text(encoding="utf-8", errors="replace")
    cdc = strip_comments(cdc_raw)
    for marker, token in (
        ("INCLUDE", "usb_console_internal.h"),
        ("3", "usb_console_transport_connected"),
        ("4", "usb_console_transport_disconnected"),
        ("13", "usb_console_tx_complete_from_isr"),
    ):
        block = user_code_block(cdc_raw, marker)
        check(block is not None and token in block,
              f"CDC USER CODE {marker} must wire {token}", failures)
    transmit_body = function_body(cdc, "CDC_Transmit_FS")
    check(transmit_body is not None, "CDC_Transmit_FS body is missing", failures)
    if transmit_body is not None:
        ordered_tokens = (
            "Buf", "Len", "USBD_STATE_CONFIGURED", "pClassData",
            "pClassDataCmsit", "TxState", "USBD_CDC_SetTxBuffer",
            "USBD_CDC_TransmitPacket",
        )
        positions = tuple(transmit_body.find(token) for token in ordered_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "CDC_Transmit_FS must validate args/state/class before TxState/set/submit",
              failures)
        check("taskENTER_CRITICAL" in transmit_body
              and "taskEXIT_CRITICAL" in transmit_body,
              "CDC_Transmit_FS check/set/submit must use a short FreeRTOS critical section",
              failures)
        check("USBD_BUSY" in transmit_body and "USBD_FAIL" in transmit_body,
              "CDC_Transmit_FS must distinguish BUSY from FAIL", failures)
    check("usb_console.c" not in cdc_raw,
          "generated CDC glue must include headers, never implementation files", failures)

    cdc_class = strip_comments(cdc_class_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    packet_body = function_body(cdc_class, "USBD_CDC_TransmitPacket")
    check(packet_body is not None,
          "ST CDC class must define USBD_CDC_TransmitPacket", failures)
    if packet_body is not None:
        check(re.search(r"ret\s*=\s*USBD_LL_Transmit\s*\(", packet_body)
              is not None,
              "USBD_CDC_TransmitPacket must propagate the LL transmit result",
              failures)
        check(re.search(r"if\s*\(\s*ret\s*!=\s*USBD_OK\s*\).*?"
                        r"TxState\s*=\s*0U.*?total_length\s*=\s*0U",
                        packet_body, flags=re.DOTALL) is not None,
              "USBD_CDC_TransmitPacket must roll back TxState/length on LL failure",
              failures)

    no_heap = strip_comments(no_heap_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    check(re.search(r"\bvoid\s*\*\s*_sbrk\s*\(\s*ptrdiff_t\s+\w+\s*\)",
                    no_heap) is not None,
          "no_heap.c must provide the newlib _sbrk(ptrdiff_t) hook", failures)
    check("ENOMEM" in no_heap and re.search(r"errno\s*=\s*ENOMEM", no_heap)
          is not None and re.search(r"return\s*\(\s*void\s*\*\s*\).*?-\s*1",
                                    no_heap) is not None,
          "_sbrk must fail closed with errno=ENOMEM and (void *)-1", failures)

    cpp_runtime = strip_comments(cpp_runtime_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    check(re.search(r"\bvoid\s+__cxa_pure_virtual\s*\(\s*void\s*\)",
                    cpp_runtime) is not None,
          "cpp_runtime.c must provide the strong C ABI pure-virtual fail-stop hook",
          failures)
    check("noreturn" in cpp_runtime and re.search(r"for\s*\(\s*;\s*;\s*\)",
                                                   cpp_runtime) is not None,
          "__cxa_pure_virtual must be a non-returning fail-stop loop", failures)
    cpsid_position = cpp_runtime.find('cpsid i')
    loop_match = re.search(r"for\s*\(\s*;\s*;\s*\)", cpp_runtime)
    check("__arm__" in cpp_runtime and "__thumb__" in cpp_runtime
          and "__asm" in cpp_runtime and cpsid_position >= 0
          and loop_match is not None and cpsid_position < loop_match.start(),
          "__cxa_pure_virtual must mask ARM interrupts before its fail-stop loop",
          failures)
    for forbidden in ("printf", "puts", "fflush", "malloc", "free"):
        check(forbidden not in cpp_runtime,
              f"cpp_runtime.c must not depend on {forbidden}", failures)

    target = strip_comments(usb_target_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    config = strip_comments(config_path.read_text(
        encoding="utf-8", errors="replace"
    ))
    irq_priority = re.search(
        r"HAL_NVIC_SetPriority\s*\(\s*OTG_FS_IRQn\s*,\s*(\d+)", target
    )
    max_syscall = re.search(
        r"#\s*define\s+configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY\s+(\d+)",
        config,
    )
    check(irq_priority is not None and max_syscall is not None
          and int(irq_priority.group(1)) >= int(max_syscall.group(1)),
          "OTG_FS IRQ priority must be numerically >= max syscall priority", failures)

    project = strip_comments(project_path.read_text(encoding="utf-8"))
    check(project.count("Dima/adapters/usb_console/usb_console.c") == 1,
          "make/project.mk must own usb_console.c exactly once", failures)
    check(project.count("Dima/platform/freertos/libc/no_heap.c") == 1,
          "make/project.mk must own no_heap.c exactly once", failures)
    check(project.count("Dima/platform/freertos/libc/cpp_runtime.c") == 1,
          "make/project.mk must own cpp_runtime.c exactly once", failures)
    check(re.search(
              r"override\s+LDFLAGS\s*:=\s*"
              r"\$\(filter-out\s+-specs=nano\.specs,\$\(LDFLAGS\)\)",
              project,
          ) is not None,
          "application overlay must select full newlib for static standard streams",
          failures)
    check("newlib_lock_glue.c" not in project,
          "make/project.mk must not compile newlib_lock_glue.c", failures)
    host_makefile = host_makefile_path.read_text(encoding="utf-8")
    for target_name in ("usb-console-test", "usb-console-freertos-backend-test",
                        "usb-cdc-glue-test", "no-heap-test"):
        check(target_name in host_makefile,
              f"tests/host/Makefile must expose {target_name}", failures)
    host_runner = host_runner_path.read_text(encoding="utf-8")
    for target_name in ("usb-console-test", "usb-console-freertos-backend-test",
                        "usb-cdc-glue-test", "no-heap-test"):
        check(target_name in host_runner,
              f"canonical host runner must execute {target_name}", failures)


def check_board_architecture(failures: list[str]) -> None:
    board_header_path = ROOT / "Boards/H743/Inc/board_init.h"
    board_source_path = ROOT / "Boards/H743/Src/board_init.c"
    boot_layout_path = ROOT / "Boards/H743/Inc/boot_layout.h"
    main_raw_source = MAIN_C.read_text(encoding="utf-8")
    main_source = strip_comments(main_raw_source)

    for stem in PERIPHERAL_CONTRACTS:
        for directory, suffix in (("Core/Inc", ".h"), ("Core/Src", ".c")):
            path = ROOT / directory / f"{stem}{suffix}"
            check(path.is_file(),
                  f"missing CubeMX coupled peripheral file: {path.relative_to(ROOT)}",
                  failures)

    board_header = ""
    if board_header_path.is_file():
        board_header = strip_comments(
            board_header_path.read_text(encoding="utf-8", errors="replace")
        )
        for function_name in ("board_vector_table_init", "board_init"):
            check(re.search(rf"\bvoid\s+{function_name}\s*\(\s*void\s*\)\s*;",
                            board_header) is not None,
                  f"Boards/H743/Inc/board_init.h must declare void {function_name}(void)",
                  failures)
        check("__cplusplus" in board_header and 'extern "C"' in board_header,
              "board_init.h must provide C linkage when included from C++", failures)
        check(re.search(r"#\s*define\s+H743_APP_VECTOR_BASE\b", board_header) is None,
              "board_init.h must not duplicate canonical H743_APP_VECTOR_BASE", failures)
        check(re.search(r"#\s*define\s+BOARD_SD_INIT_AT_BOOT\b", board_header) is None,
              "board_init.h must not expose the SD boot build policy", failures)

    board_source = ""
    if board_source_path.is_file():
        board_source = strip_comments(
            board_source_path.read_text(encoding="utf-8", errors="replace")
        )
        check(re.search(
            r"#\s*include\s*[<\"]Boards/H743/Inc/boot_layout\.h[>\"]",
            board_source,
        )
              is not None,
              "board_init.c must include the board-owned boot_layout.h directly",
              failures)
        check(re.search(r"#\s*ifndef\s+BOARD_SD_INIT_AT_BOOT.*?"
                        r"#\s*define\s+BOARD_SD_INIT_AT_BOOT\s+0\b",
                        board_source, flags=re.DOTALL) is not None,
              "board_init.c must default direct non-Make builds to SD init disabled",
              failures)
        vector_body = function_body(board_source, "board_vector_table_init")
        check(vector_body is not None,
              "Boards/H743/Src/board_init.c must define board_vector_table_init", failures)
        if vector_body is not None:
            vector_tokens = ("SCB->VTOR", "H743_APP_VECTOR_BASE", "__DSB", "__ISB")
            positions = tuple(vector_body.find(token) for token in vector_tokens)
            check(all(position >= 0 for position in positions)
                  and list(positions) == sorted(positions),
                  "board_vector_table_init must set VTOR then execute DSB and ISB",
                  failures)

        init_body = function_body(board_source, "board_init")
        check(init_body is not None,
              "Boards/H743/Src/board_init.c must define board_init", failures)
        if init_body is not None:
            actual_calls = tuple(re.findall(r"\b(MX_[A-Za-z0-9_]+_Init)\s*\(", init_body))
            check(actual_calls == BOARD_INIT_CALL_ORDER,
                  "board_init call order must be GPIO, DMA, FDCAN, I2C, conditional "
                  "SDMMC, SPI, eight UARTs, TIM5 and TIM8", failures)
            check(re.search(r"#\s*if\s+BOARD_SD_INIT_AT_BOOT\b.*?"
                            r"MX_SDMMC1_SD_Init\s*\(\s*\)\s*;.*?#\s*endif",
                            init_body, flags=re.DOTALL) is not None,
                  "board_init must conditionally call SDMMC with BOARD_SD_INIT_AT_BOOT",
                  failures)

    boot_layout = ""
    if boot_layout_path.is_file():
        boot_layout = strip_comments(
            boot_layout_path.read_text(encoding="utf-8", errors="replace")
        )
        check(re.search(r"#\s*define\s+H743_APP_VECTOR_BASE\s+"
                        r"\(H743_PRIMARY_SLOT_BASE\s*\+\s*"
                        r"H743_MCUBOOT_HEADER_SIZE\)", boot_layout) is not None,
              "boot_layout.h must own the canonical application vector expression",
              failures)
    macro_owners = []
    for path in (boot_layout_path, board_header_path, board_source_path):
        if path.is_file() and re.search(
            r"#\s*define\s+H743_APP_VECTOR_BASE\b",
            strip_comments(path.read_text(encoding="utf-8", errors="replace")),
        ):
            macro_owners.append(path)
    check(macro_owners == [boot_layout_path],
          "H743_APP_VECTOR_BASE must have exactly one production owner: boot_layout.h",
          failures)

    core_sources = tuple((ROOT / "Core/Src").glob("*.c"))
    for stem, (mx_functions, handles) in PERIPHERAL_CONTRACTS.items():
        header_path = ROOT / "Core/Inc" / f"{stem}.h"
        source_path = ROOT / "Core/Src" / f"{stem}.c"
        header = strip_comments(header_path.read_text(encoding="utf-8", errors="replace")) \
            if header_path.is_file() else ""
        source = strip_comments(source_path.read_text(encoding="utf-8", errors="replace")) \
            if source_path.is_file() else ""
        for function_name in mx_functions:
            check(re.search(rf"\bvoid\s+{function_name}\s*\(\s*void\s*\)\s*;",
                            header) is not None,
                  f"{header_path.relative_to(ROOT)} must declare {function_name}", failures)
            owners = [path for path in core_sources
                      if function_body(strip_comments(path.read_text(
                          encoding="utf-8", errors="replace")), function_name) is not None]
            check(owners == [source_path],
                  f"{function_name} must have one definition owned by "
                  f"{source_path.relative_to(ROOT)}", failures)
            check(re.search(rf"\bstatic\s+void\s+{function_name}\s*\(", source) is None,
                  f"{function_name} must have external linkage", failures)
        for handle_type, handle_name in handles:
            check(re.search(rf"\bextern\s+{handle_type}\s+{handle_name}\s*;", header)
                  is not None,
                  f"{header_path.relative_to(ROOT)} must declare extern {handle_name}", failures)
            definitions: list[tuple[pathlib.Path, str]] = []
            for path in core_sources:
                contents = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
                for match in re.finditer(
                    rf"(?m)^\s*((?:(?:extern|static)\s+)*){handle_type}\s+"
                    rf"{handle_name}\s*;",
                    contents,
                ):
                    if "extern" not in match.group(1).split():
                        definitions.append((path, match.group(1)))
            check(len(definitions) == 1 and definitions[0][0] == source_path,
                  f"{handle_name} must have one definition owned by "
                  f"{source_path.relative_to(ROOT)}", failures)
            check(not definitions or "static" not in definitions[0][1].split(),
                  f"{handle_name} must not be static", failures)

    sdmmc_source_path = ROOT / "Core/Src/sdmmc.c"
    if sdmmc_source_path.is_file():
        sdmmc_source = strip_comments(
            sdmmc_source_path.read_text(encoding="utf-8", errors="replace")
        )
        sdmmc_body = function_body(sdmmc_source, "MX_SDMMC1_SD_Init") or ""
        check("BOARD_SD_INIT_AT_BOOT" not in sdmmc_source,
              "sdmmc.c must not own the boot policy macro", failures)
        check(not re.search(r"\breturn\s*;", sdmmc_body),
              "MX_SDMMC1_SD_Init must not return early", failures)

    main_body = function_body(main_source, "main")
    check(main_body is not None, "Core/Src/main.c must define main", failures)
    if main_body is not None:
        ordered_tokens = (
            "board_vector_table_init", "HAL_Init", "SystemClock_Config",
            "PeriphCommonClock_Config", "board_init", "osKernelInitialize",
        )
        positions = tuple(main_body.find(token) for token in ordered_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "main must set VTOR before HAL_Init and call board_init after both clock setups",
              failures)
        direct_mx_calls = tuple(re.findall(r"\b(MX_[A-Za-z0-9_]+_Init)\s*\(", main_body))
        check(not set(direct_mx_calls).intersection(BOARD_INIT_CALL_ORDER),
              "main must not call board-level MX initializers directly", failures)
        for marker, function_name in (
            ("1", "board_vector_table_init"),
            ("SysInit", "board_init"),
        ):
            user_block = re.search(
                rf"/\*\s*USER CODE BEGIN {marker}\s*\*/(.*?)"
                rf"/\*\s*USER CODE END {marker}\s*\*/",
                main_raw_source,
                flags=re.DOTALL,
            )
            check(user_block is not None and re.search(
                      rf"\b{function_name}\s*\(\s*\)\s*;", user_block.group(1)
                  ) is not None,
                  f"{function_name} must live in CubeMX-preserved USER CODE {marker}",
                  failures)
            check(len(re.findall(rf"\b{function_name}\s*\(", main_body)) == 1,
                  f"main must call {function_name} exactly once", failures)

    mx_definition = re.compile(
        r"(?:static\s+)?void\s+MX_[A-Za-z0-9_]+_Init\s*\([^;{}]*\)\s*\{",
        re.MULTILINE,
    )
    check(not mx_definition.search(main_source),
          "Core/Src/main.c still defines MX_*_Init bodies; move them to coupled files",
          failures)
    check(not re.search(r"\b(?:FDCAN|I2C|SD|SPI|DMA|TIM|UART)_HandleTypeDef\s+h\w+\s*;",
                        main_source),
          "Core/Src/main.c still owns a board peripheral handle", failures)
    check(not re.search(r"\bprintf\s*\(", main_source),
          "Core/Src/main.c still contains a business printf", failures)

    ioc_path = ROOT / "H743_FreeRTOS.ioc"
    root_ioc_files = tuple(ROOT.glob("*.ioc"))
    check(root_ioc_files == (ioc_path,),
          "H743_FreeRTOS.ioc must be the only root-level .ioc configuration", failures)
    if ioc_path.is_file():
        ioc_source = ioc_path.read_text(encoding="utf-8", errors="replace")
        check(re.search(r"(?m)^ProjectManager\.CoupleFile=true$", ioc_source) is not None,
              "CubeMX ProjectManager.CoupleFile must be true", failures)
        function_list_match = re.search(
            r"(?m)^ProjectManager\.functionlistsort=(.*)$", ioc_source
        )
        check(function_list_match is not None,
              "CubeMX functionlistsort entry is missing", failures)
        if function_list_match is not None:
            entries = function_list_match.group(1).split(",")
            for function_name in BOARD_INIT_CALL_ORDER:
                matches = [entry for entry in entries
                           if f"-{function_name}-" in entry]
                check(len(matches) == 1 and matches[0].endswith("-true-HAL-false"),
                      f"CubeMX {function_name} must decode as true-HAL-false", failures)
            for function_name in ("SystemClock_Config", "MX_USB_DEVICE_Init"):
                matches = [entry for entry in entries
                           if f"-{function_name}-" in entry]
                check(len(matches) == 1 and matches[0].endswith("-false-HAL-false"),
                      f"CubeMX {function_name} ownership/call policy drifted", failures)

    root_makefile_path = ROOT / "Makefile"
    project_makefile_path = ROOT / "make/project.mk"
    if root_makefile_path.is_file() and project_makefile_path.is_file():
        root_makefile = strip_comments(root_makefile_path.read_text(encoding="utf-8"))
        project_makefile = strip_comments(project_makefile_path.read_text(encoding="utf-8"))
        expected_cube_sources = tuple(f"Core/Src/{stem}.c" for stem in PERIPHERAL_CONTRACTS)
        for source_path in expected_cube_sources:
            check(root_makefile.count(source_path) == 1,
                  f"CubeMX Makefile must own exactly one {source_path}", failures)
            check(source_path not in project_makefile,
                  f"make/project.mk must not own CubeMX source {source_path}", failures)
        board_source_name = "Boards/H743/Src/board_init.c"
        check(board_source_name not in root_makefile,
              "CubeMX Makefile must not own board_init.c", failures)
        check(project_makefile.count(board_source_name) == 1,
              "make/project.mk must own board_init.c exactly once", failures)
        c_sources_match = re.search(r"(?ms)^C_SOURCES\s*=\s*(.*?)^\s*$", root_makefile)
        if c_sources_match is not None:
            source_paths = re.findall(r"[^\s\\]+\.c", c_sources_match.group(1))
            basenames = [pathlib.PurePosixPath(path).stem for path in source_paths]
            check(len(basenames) == len(set(basenames)),
                  "CubeMX C_SOURCES object basenames must be unique", failures)


def check_static_runtime(failures: list[str]) -> None:
    """Check that startup and the FreeRTOS kernel need no runtime heap."""
    header_path = ROOT / "Dima/application/app_main.h"
    bootstrap_path = ROOT / "Dima/application/app_bootstrap.cpp"
    app_main_path = ROOT / "Dima/application/app_main.cpp"
    application_context_header_path = ROOT / "Dima/product/rover/ApplicationContext.hpp"
    application_context_source_path = ROOT / "Dima/product/rover/ApplicationContext.cpp"
    mcuboot_header_path = ROOT / "Dima/adapters/mcuboot/mcuboot_app.h"
    config_path = ROOT / "Core/Inc/FreeRTOSConfig.h"
    freertos_path = ROOT / "Core/Src/freertos.c"
    root_makefile_path = ROOT / "Makefile"
    project_makefile_path = ROOT / "make/project.mk"
    linker_path = ROOT / "Linker/STM32H743VITx_MCUBOOT_APP.ld"
    ioc_path = ROOT / "H743_FreeRTOS.ioc"
    mxproject_path = ROOT / ".mxproject"
    regenerated_main_fixture_path = (
        ROOT / "tests/structure/fixtures/main_cubemx_regenerated_static_task.c"
    )

    required_paths = (
        header_path, bootstrap_path, app_main_path,
        application_context_header_path, application_context_source_path,
        mcuboot_header_path, config_path, freertos_path,
        root_makefile_path, project_makefile_path, linker_path, ioc_path,
        mxproject_path, regenerated_main_fixture_path,
    )
    for path in required_paths:
        check(path.is_file(),
              f"missing static-runtime file: {path.relative_to(ROOT)}", failures)

    header = strip_comments(header_path.read_text(encoding="utf-8")) \
        if header_path.is_file() else ""
    check(re.search(r"#\s*include\s*<stdbool\.h>", header) is not None,
          "app_main.h must expose C bool through <stdbool.h>", failures)
    check("__cplusplus" in header and 'extern "C"' in header,
          "app_main.h must provide C linkage when included from C++", failures)
    check(re.search(r"\bbool\s+app_bootstrap_create\s*\(\s*void\s*\)\s*;", header)
          is not None,
          "app_main.h must declare bool app_bootstrap_create(void)", failures)
    check(re.search(r"\bvoid\s+app_main_task\s*\(\s*void\s*\*\s*argument\s*\)\s*;",
                    header) is not None,
          "app_main.h must declare void app_main_task(void *argument)", failures)

    mcuboot_header = strip_comments(
        mcuboot_header_path.read_text(encoding="utf-8")
    ) if mcuboot_header_path.is_file() else ""
    check("__cplusplus" in mcuboot_header and 'extern "C"' in mcuboot_header,
          "mcuboot_app.h must preserve C linkage for C++ application callers",
          failures)

    bootstrap = strip_comments(bootstrap_path.read_text(encoding="utf-8")) \
        if bootstrap_path.is_file() else ""
    check(len(re.findall(r"\bxTaskCreateStatic\s*\(", bootstrap)) == 1,
          "app bootstrap must use exactly one xTaskCreateStatic call", failures)
    check(re.search(r"\b(?:k\w*Stack\w*Bytes|\w*stack\w*bytes)\b\s*=\s*2048U?\b",
                    bootstrap, flags=re.IGNORECASE) is not None,
          "app bootstrap stack size must be exactly 2048 bytes", failures)
    check(len(re.findall(r"\bStaticTask_t\s+\w+\s*(?:\{\s*\})?\s*;", bootstrap)) == 1,
          "app bootstrap must own exactly one static task control block", failures)
    check(len(re.findall(r"\bStackType_t\s+\w+\s*\[", bootstrap)) == 1,
          "app bootstrap must own exactly one dedicated static stack", failures)
    check(re.search(
              r"constexpr\s+char\s+kAppMainTaskName\s*\[\s*\]\s*=\s*"
              r'"appMainTask"\s*;', bootstrap,
          ) is not None,
          "app bootstrap must define one constexpr appMainTask name", failures)
    check(bootstrap.count("kAppMainTaskName") >= 3 and
          bootstrap.count('"appMainTask"') == 1,
          "task lookup and creation must share kAppMainTaskName", failures)
    check(re.search(
              r"static_assert\s*\(\s*sizeof\s*\(\s*kAppMainTaskName\s*\)\s*"
              r"<=\s*configMAX_TASK_NAME_LEN\b", bootstrap,
          ) is not None,
          "appMainTask name including NUL must fit configMAX_TASK_NAME_LEN",
          failures)
    check("app_main_task" in bootstrap,
          "app bootstrap must create the app_main_task entry", failures)
    bootstrap_body = function_body(bootstrap, "app_bootstrap_create")
    check(bootstrap_body is not None,
          "app_bootstrap.cpp must define app_bootstrap_create", failures)
    if bootstrap_body is not None:
        adoption_tokens = (
            "uxTaskGetNumberOfTasks", "xTaskGetHandle", "xTaskCreateStatic",
        )
        positions = tuple(bootstrap_body.find(token) for token in adoption_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "bootstrap must count tasks, adopt appMainTask, then create only if absent",
              failures)
        check(re.search(
                  r"if\s*\(\s*uxTaskGetNumberOfTasks\s*\(\s*\)\s*>\s*0U?\s*\)"
                  r"\s*\{.*?xTaskGetHandle\s*\(",
                  bootstrap_body, flags=re.DOTALL,
              ) is not None,
              "xTaskGetHandle must be guarded by a nonzero task count", failures)

    app_main_source = strip_comments(app_main_path.read_text(encoding="utf-8")) \
        if app_main_path.is_file() else ""
    app_main_body = function_body(app_main_source, "app_main_task")
    check(app_main_body is not None, "app_main.cpp must define app_main_task", failures)
    if app_main_body is not None:
        tokens = ("dima::product::rover::application_context", ".init()",
                  ".start()", ".stop()", "vTaskSuspend")
        positions = tuple(app_main_body.find(token) for token in tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "app_main_task must delegate init/start/stop to ApplicationContext, then suspend",
              failures)
        for forbidden in ("MX_USB_DEVICE_Init", "work_queue_init", "uORB::initialize",
                          "ModuleManager", "BootHealthService", "HelloWorld"):
            check(forbidden not in app_main_body,
                  f"app_main_task must not directly own {forbidden}", failures)
        check("vTaskDelay" not in app_main_body and "vTaskDelete" not in app_main_body,
              "app_main_task must suspend without delay or self-deletion", failures)

    context_header = strip_comments(application_context_header_path.read_text(
        encoding="utf-8", errors="replace")) if application_context_header_path.is_file() else ""
    for token in ("namespace dima::product::rover", "class ApplicationContext",
                  "dima::middleware::lifecycle::ModuleManager",
                  "dima::modules::boot_health::BootHealthService",
                  "dima::modules::hello_world::HelloWorld", "ParameterService", "LogService"):
        check(token in context_header, f"ApplicationContext.hpp must own {token}", failures)

    context_source = strip_comments(application_context_source_path.read_text(
        encoding="utf-8", errors="replace")) if application_context_source_path.is_file() else ""
    init_body = cpp_method_body(context_source, "ApplicationContext", "init")
    start_body = cpp_method_body(context_source, "ApplicationContext", "start")
    stop_body = cpp_method_body(context_source, "ApplicationContext", "stop")
    check(init_body is not None and start_body is not None and stop_body is not None,
          "ApplicationContext must define init/start/stop", failures)
    if init_body is not None:
        tokens = ("MX_USB_DEVICE_Init", "px4::work_queue_init", "uORB::initialize",
                  "parameter_service_.init", "register_module(boot_health_)",
                  "register_module(hello_world_)")
        positions = tuple(init_body.find(token) for token in tokens)
        check(all(position >= 0 for position in positions) and list(positions) == sorted(positions),
              "ApplicationContext::init must initialize infrastructure before modules", failures)
    if start_body is not None:
        tokens = ("start(boot_health_)", "start(hello_world_)",
                  "parameter_service_.start", "log_service_.start")
        positions = tuple(start_body.find(token) for token in tokens)
        check(all(position >= 0 for position in positions) and list(positions) == sorted(positions),
              "ApplicationContext::start must start boot, hello, parameter, then log", failures)
    if stop_body is not None:
        tokens = ("log_service_.stop", "parameter_service_.stop", "stop(hello_world_)",
                  "stop(boot_health_)", "module_manager_.reset", "uORB::shutdown",
                  "px4::work_queue_shutdown")
        positions = tuple(stop_body.find(token) for token in tokens)
        check(all(position >= 0 for position in positions) and list(positions) == sorted(positions),
              "ApplicationContext::stop must unwind services in reverse order", failures)

    app_main_directory = ROOT / "Dima/application"
    forbidden_runtime_tokens = {
        r"\bxTaskCreate\s*\(": "xTaskCreate",
        r"\bosThreadNew\s*\(": "osThreadNew",
        r"\bpvPortMalloc\s*\(": "pvPortMalloc",
        r"\bnew\b": "new",
        r"\bdelete\b": "delete",
    }
    if app_main_directory.is_dir():
        for path in app_main_directory.rglob("*"):
            if path.suffix not in {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}:
                continue
            contents = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            for pattern, token in forbidden_runtime_tokens.items():
                check(not re.search(pattern, contents),
                      f"{path.relative_to(ROOT)} uses forbidden {token}", failures)

    main_raw = MAIN_C.read_text(encoding="utf-8") if MAIN_C.is_file() else ""
    main_source = strip_comments(main_raw)
    main_body = function_body(main_source, "main")
    check(main_body is not None, "Core/Src/main.c must define main", failures)
    check(re.search(
        r"#\s*include\s*[<\"]Dima/application/app_main\.h[>\"]",
        main_source,
    )
          is not None,
          "main.c must include the app bootstrap C interface", failures)
    for forbidden_include in ("usb_device.h", "mcuboot_app.h"):
        check(forbidden_include not in main_source,
              f"main.c must not include {forbidden_include}", failures)
    for forbidden_token in ("defaultTaskHandle", "defaultTask_attributes",
                            "StartDefaultTask", "osThreadNew"):
        check(forbidden_token not in main_source,
              f"main.c still owns legacy {forbidden_token}", failures)
    if main_body is not None:
        ordered_tokens = ("osKernelInitialize", "app_bootstrap_create", "osKernelStart")
        positions = tuple(main_body.find(token) for token in ordered_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "main must create the static app bootstrap after kernel init and before start",
              failures)
        check(re.search(r"if\s*\(\s*!\s*app_bootstrap_create\s*\(\s*\)\s*\)\s*\{?\s*"
                        r"Error_Handler\s*\(\s*\)\s*;", main_body) is not None,
              "main must enter Error_Handler when app bootstrap creation fails", failures)
    threads_block = re.search(
        r"/\*\s*USER CODE BEGIN RTOS_THREADS\s*\*/(.*?)"
        r"/\*\s*USER CODE END RTOS_THREADS\s*\*/",
        main_raw, flags=re.DOTALL,
    )
    check(threads_block is not None and
          "app_bootstrap_create" in threads_block.group(1),
          "app bootstrap creation must live in CubeMX-preserved RTOS_THREADS", failures)

    config = strip_comments(config_path.read_text(encoding="utf-8")) \
        if config_path.is_file() else ""
    required_config_values = {
        "configSUPPORT_STATIC_ALLOCATION": "1",
        "configSUPPORT_DYNAMIC_ALLOCATION": "1",
        "configUSE_TIMERS": "1",
        "configUSE_OS2_THREAD_ENUMERATE": "0",
        "configUSE_OS2_TIMER": "0",
        "INCLUDE_xTaskGetHandle": "1",
    }
    for name, value in required_config_values.items():
        check(re.search(rf"#\s*define\s+{name}\s+{value}\b", config) is not None,
              f"FreeRTOSConfig.h must define {name} as {value}", failures)
    defines_block = re.search(
        r"/\*\s*USER CODE BEGIN Defines\s*\*/(.*?)"
        r"/\*\s*USER CODE END Defines\s*\*/",
        config_path.read_text(encoding="utf-8") if config_path.is_file() else "",
        flags=re.DOTALL,
    )
    check(defines_block is not None and re.search(
              r"#\s*define\s+INCLUDE_xTaskGetHandle\s+1\b",
              defines_block.group(1),
          ) is not None,
          "INCLUDE_xTaskGetHandle=1 must live in the persisted Defines USER block",
          failures)
    config_without_defines = re.sub(
        r"/\*\s*USER CODE BEGIN Defines\s*\*/.*?"
        r"/\*\s*USER CODE END Defines\s*\*/",
        "", config_path.read_text(encoding="utf-8") if config_path.is_file() else "",
        flags=re.DOTALL,
    )
    check(re.search(r"#\s*define\s+INCLUDE_xTaskGetHandle\b",
                    config_without_defines) is None,
          "generated FreeRTOSConfig sections must not duplicate INCLUDE_xTaskGetHandle",
          failures)
    check("USE_FreeRTOS_HEAP_4" not in config,
          "FreeRTOSConfig.h must not select heap_4", failures)
    check(re.search(r"#\s*define\s+configTICK_RATE_HZ\s+\(\(TickType_t\)1000\)", config)
          is not None,
          "static runtime migration must preserve the 1000 Hz tick", failures)
    check(re.search(r"#\s*define\s+configMAX_PRIORITIES\s+\(\s*56\s*\)", config)
          is not None,
          "static runtime migration must preserve 56 priorities", failures)

    freertos_raw = freertos_path.read_text(encoding="utf-8") \
        if freertos_path.is_file() else ""
    freertos_source = strip_comments(freertos_raw)
    for callback, stack_depth in (
        ("vApplicationGetIdleTaskMemory", "configMINIMAL_STACK_SIZE"),
        ("vApplicationGetTimerTaskMemory", "configTIMER_TASK_STACK_DEPTH"),
    ):
        check(re.search(rf"(?m)^void\s+{callback}\s*\(", freertos_source) is not None,
              f"freertos.c must provide strong {callback}", failures)
        check(re.search(rf"\bStackType_t\s+\w+\s*\[\s*{stack_depth}\s*\]",
                        freertos_source) is not None,
              f"{callback} must provide a static {stack_depth} stack", failures)
    check(re.search(r"#\s*if\s*\(?\s*configUSE_TIMERS\s*==\s*1\s*\)?", freertos_raw)
          is not None,
          "timer daemon static memory callback must be conditional on configUSE_TIMERS",
          failures)
    application_block = re.search(
        r"/\*\s*USER CODE BEGIN Application\s*\*/(.*?)"
        r"/\*\s*USER CODE END Application\s*\*/",
        freertos_raw, flags=re.DOTALL,
    )
    check(application_block is not None and
          "vApplicationGetIdleTaskMemory" in application_block.group(1) and
          "vApplicationGetTimerTaskMemory" in application_block.group(1),
          "static kernel memory callbacks must live in the FreeRTOS USER CODE block",
          failures)

    root_makefile = strip_comments(root_makefile_path.read_text(encoding="utf-8")) \
        if root_makefile_path.is_file() else ""
    project_makefile = strip_comments(project_makefile_path.read_text(encoding="utf-8")) \
        if project_makefile_path.is_file() else ""
    check("heap_4.c" not in root_makefile,
          "heap_4.c must be absent from the generated application source list",
          failures)
    check("filter-out" in project_makefile and "heap_4.c" in project_makefile,
          "make/project.mk must defensively filter regenerated heap_4 sources and objects",
          failures)
    for source_path in (
        "Dima/application/app_bootstrap.cpp", "Dima/application/app_main.cpp",
        "Dima/product/rover/ApplicationContext.cpp",
    ):
        check(project_makefile.count(source_path) == 1,
              f"make/project.mk must own exactly one {source_path}", failures)
        check(source_path not in root_makefile,
              f"CubeMX Makefile must not own {source_path}", failures)

    linker = strip_comments(linker_path.read_text(encoding="utf-8")) \
        if linker_path.is_file() else ""
    check(re.search(r"_Min_Heap_Size\s*=\s*0x?0\s*;", linker) is not None,
          "application linker minimum heap size must be zero", failures)

    ioc = ioc_path.read_text(encoding="utf-8", errors="replace") \
        if ioc_path.is_file() else ""
    task_match = re.search(r"(?m)^FREERTOS\.Tasks01=(.*)$", ioc)
    check(task_match is not None, "CubeMX FREERTOS.Tasks01 is missing", failures)
    if task_match is not None:
        task_fields = task_match.group(1).split(",")
        check(len(task_fields) >= 7 and task_fields[0] == "appMainTask"
              and task_fields[1] == "24" and task_fields[2] == "512"
              and task_fields[3] == "app_main_task" and task_fields[6] == "Static",
              "CubeMX task must be appMainTask priority 24, 2048 bytes, app_main_task, Static",
              failures)
        check("Dynamic" not in task_fields,
              "CubeMX task metadata must not contain Dynamic", failures)
    check(re.search(r"(?m)^ProjectManager\.HeapSize=0x0$", ioc) is not None,
          "CubeMX ProjectManager.HeapSize must be zero", failures)

    mxproject = mxproject_path.read_text(encoding="utf-8", errors="replace") \
        if mxproject_path.is_file() else ""
    generated_heap_path = (
        r"Middlewares\Third_Party\FreeRTOS\Source\portable\MemMang\heap_4.c"
    )
    check(generated_heap_path not in mxproject,
          ".mxproject generation lists must not restore heap_4.c", failures)

    regenerated_main = regenerated_main_fixture_path.read_text(
        encoding="utf-8", errors="replace"
    ) if regenerated_main_fixture_path.is_file() else ""
    regenerated_body = function_body(
        strip_comments(regenerated_main), "regenerated_main_fixture"
    )
    check(regenerated_body is not None,
          "missing CubeMX static-task regeneration fixture body", failures)
    if regenerated_body is not None:
        regenerated_tokens = (
            "osKernelInitialize", "osThreadNew", "app_bootstrap_create",
            "osKernelStart",
        )
        positions = tuple(regenerated_body.find(token) for token in regenerated_tokens)
        check(all(position >= 0 for position in positions)
              and list(positions) == sorted(positions),
              "regeneration fixture must create appMainTask before the preserved bootstrap",
              failures)
    regenerated_user_block = re.search(
        r"/\*\s*USER CODE BEGIN RTOS_THREADS\s*\*/(.*?)"
        r"/\*\s*USER CODE END RTOS_THREADS\s*\*/",
        regenerated_main, flags=re.DOTALL,
    )
    check(regenerated_user_block is not None and
          "app_bootstrap_create" in regenerated_user_block.group(1) and
          '"appMainTask"' in regenerated_main,
          "regeneration fixture must preserve bootstrap beside CubeMX appMainTask",
          failures)

    host_makefile_path = ROOT / "tests/host/Makefile"
    host_makefile = host_makefile_path.read_text(encoding="utf-8") \
        if host_makefile_path.is_file() else ""
    ordinary_sources = re.search(
        r"(?ms)^HOST_PRODUCTION_SOURCES\s*\?=\s*(.*?)^\s*$", host_makefile
    )
    check(ordinary_sources is not None and
          "Dima/application/app_main.cpp" not in ordinary_sources.group(1),
          "legacy app_main task body must stay out of the ordinary host binary", failures)



def check_no_tracked_app(failures: list[str]) -> None:
    result = subprocess.run(["git", "ls-files", "--", "App"], cwd=ROOT,
                            capture_output=True, text=True, check=False)
    check(result.returncode == 0,
          "git ls-files failed while checking the retired App root", failures)
    if result.returncode == 0:
        tracked = [line for line in result.stdout.splitlines() if line]
        check(not tracked, "tracked App/ paths are forbidden: " + ", ".join(tracked),
              failures)

def main() -> int:
    if sys.argv[1:] not in (
        [], ["--runtime-only"], ["--board-only"], ["--static-runtime-only"],
        ["--usb-console-only"],
    ):
        print("usage: check_architecture.py "
              "[--runtime-only|--board-only|--static-runtime-only|--usb-console-only]",
              file=sys.stderr)
        return 2

    runtime_only = sys.argv[1:] == ["--runtime-only"]
    board_only = sys.argv[1:] == ["--board-only"]
    static_runtime_only = sys.argv[1:] == ["--static-runtime-only"]
    usb_console_only = sys.argv[1:] == ["--usb-console-only"]
    failures: list[str] = []
    check_no_tracked_app(failures)
    if usb_console_only:
        check_usb_console(failures)
        if failures:
            for failure in failures:
                print(f"STRUCTURE RED: {failure}", file=sys.stderr)
            return 1
        print("STRUCTURE PASS: static USB CDC console boundary is satisfied")
        return 0
    runtime_required_paths = (
        "Dima/middleware/lifecycle/module_base.hpp",
        "Dima/middleware/lifecycle/module_manager.hpp",
        "Dima/middleware/lifecycle/module_manager.cpp",
        "Dima/middleware/messaging/topic.hpp",
        "Dima/middleware/scheduling/scheduled_work_item.hpp",
        "Dima/middleware/scheduling/scheduled_work_item.cpp",
        "Dima/middleware/scheduling/work_queue.hpp",
        "Dima/middleware/scheduling/work_queue.cpp",
        "Dima/middleware/scheduling/freertos_work_queue.hpp",
        "Dima/middleware/scheduling/freertos_work_queue.cpp",
        "Dima/platform/freertos/platform_time.hpp",
        "Dima/platform/freertos/platform_time.cpp",
        "Dima/messages/app_heartbeat.hpp",
    )
    application_required_paths = (
        "Dima/application/app_main.h",
        "Dima/application/app_bootstrap.cpp",
        "Dima/application/app_main.cpp",
        "Dima/product/rover/ApplicationContext.hpp",
        "Dima/product/rover/ApplicationContext.cpp",
        "Dima/adapters/usb_console/usb_console.h",
        "Dima/modules/hello_world/hello_world.hpp",
        "Dima/modules/hello_world/hello_world.cpp",
        "Dima/modules/boot_health/boot_health.hpp",
        "Dima/modules/boot_health/boot_health.cpp",
        "Dima/adapters/mcuboot/mcuboot_app.h",
        "Dima/adapters/mcuboot/mcuboot_app.c",
    )
    board_required_paths = (
        "Boards/H743/Inc/board_init.h",
        "Boards/H743/Src/board_init.c",
        *(f"Core/Inc/{stem}.h" for stem in PERIPHERAL_CONTRACTS),
        *(f"Core/Src/{stem}.c" for stem in PERIPHERAL_CONTRACTS),
    )
    required_paths: tuple[str, ...] = ()
    if not board_only and not static_runtime_only:
        required_paths += runtime_required_paths
    if not runtime_only and not board_only and not static_runtime_only:
        required_paths += application_required_paths
    if not runtime_only and not static_runtime_only:
        required_paths += board_required_paths
    for relative_path in required_paths:
        check((ROOT / relative_path).is_file(),
              f"missing required architecture file: {relative_path}", failures)

    if not runtime_only and not static_runtime_only:
        check_board_architecture(failures)

    if not runtime_only and not board_only:
        check_static_runtime(failures)
    if not runtime_only and not board_only and not static_runtime_only:
        check_hello_world(failures)
        check_boot_health(failures)
        check_usb_console(failures)

    if static_runtime_only:
        if failures:
            for failure in failures:
                print(f"STRUCTURE RED: {failure}", file=sys.stderr)
            return 1
        print("STRUCTURE PASS: static FreeRTOS runtime boundary is satisfied")
        return 0

    if runtime_only:
        source_directories = (ROOT / "Dima",)
    elif board_only:
        source_directories = (ROOT / "Boards/H743",)
    else:
        source_directories = (ROOT / "Dima", ROOT / "Boards/H743")
    for directory in source_directories:
        if not directory.exists():
            continue
        for path in directory.rglob("*"):
            if path.suffix not in {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}:
                continue
            contents = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            check(not re.search(r"#\s*include\s*[<\"](?:[A-Za-z0-9_.-]+/)*main\.h[>\"]", contents),
                  f"{path.relative_to(ROOT)} includes main.h; Dima/Boards must not depend on CubeMX main.h",
                  failures)

    if board_only:
        if failures:
            for failure in failures:
                print(f"STRUCTURE RED: {failure}", file=sys.stderr)
            return 1
        print("STRUCTURE PASS: H743 CubeMX/board boundary is satisfied")
        return 0

    no_heap_runtime_directories = (
        ROOT / "Dima/middleware/lifecycle",
        ROOT / "Dima/middleware/messaging",
        ROOT / "Dima/middleware/scheduling",
    )
    forbidden_platform_tokens = {
        r"\bxTaskCreate\s*\(": "xTaskCreate",
        r"\bosThreadNew\s*\(": "osThreadNew",
        r"\bpvPortMalloc\s*\(": "pvPortMalloc",
        r"\bnew\b": "new",
        r"\bdelete\b": "delete",
    }
    forbidden_stl_headers = re.compile(
        r"#\s*include\s*<(?:algorithm|array|atomic|chrono|deque|functional|list|map|memory|"
        r"mutex|new|optional|queue|set|string|string_view|thread|tuple|type_traits|unordered_map|"
        r"unordered_set|utility|vector)>"
    )
    for runtime_directory in no_heap_runtime_directories:
        if not runtime_directory.is_dir():
            continue
        for path in runtime_directory.rglob("*"):
            if path.suffix not in {".cc", ".cpp", ".cxx", ".h", ".hpp"}:
                continue
            contents = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
            for pattern, token in forbidden_platform_tokens.items():
                check(not re.search(pattern, contents),
                      f"{path.relative_to(ROOT)} uses forbidden {token}", failures)
            check(not forbidden_stl_headers.search(contents),
                  f"{path.relative_to(ROOT)} includes a forbidden STL header", failures)

    freertos_queue = ROOT / "Dima/middleware/scheduling/freertos_work_queue.cpp"
    if freertos_queue.is_file():
        queue_source = strip_comments(
            freertos_queue.read_text(encoding="utf-8", errors="replace")
        )
        check('"wq:hp_default"' in queue_source,
              "FreeRTOS work queue must define wq:hp_default", failures)
        check('"wq:lp_default"' in queue_source,
              "FreeRTOS work queue must define wq:lp_default", failures)
        check(len(re.findall(r"\bxTaskCreateStatic\s*\(", queue_source)) >= 2,
              "FreeRTOS work queue must create both workers with xTaskCreateStatic", failures)
        check(re.search(r"\b(?:k\w*Stack\w*Bytes|\w*stack\w*bytes)\b\s*=\s*2048U?\b",
                        queue_source, flags=re.IGNORECASE) is not None,
              "FreeRTOS work queue static stack size must be exactly 2048 bytes", failures)
        check(len(re.findall(r"\bStackType_t\s+\w*[Ss]tack\w*\s*\[", queue_source)) >= 2,
              "FreeRTOS work queue must provide two independent static stacks", failures)
        check(re.search(r"\bulTaskNotifyTake\s*\(|\bxTaskNotifyWait\s*\(", queue_source)
              is not None,
              "FreeRTOS work queue must block on a task notification", failures)

    platform_time = ROOT / "Dima/platform/freertos/platform_time.cpp"
    if platform_time.is_file():
        time_source = strip_comments(
            platform_time.read_text(encoding="utf-8", errors="replace")
        )
        check("Dima/platform/freertos/hrt.hpp" in time_source,
              "platform time must use the Dima high-resolution timer", failures)
        check("return hrt_absolute_time();" in time_source
              and "return hrt_absolute_time_ms();" in time_source,
              "platform time must delegate microsecond and millisecond reads to HRT", failures)

    generic_work_queue = ROOT / "Dima/middleware/scheduling/work_queue.cpp"
    if generic_work_queue.is_file():
        generic_queue_source = strip_comments(
            generic_work_queue.read_text(encoding="utf-8", errors="replace")
        )
        check("APP_HOST_TEST" in generic_queue_source,
              "generic work queue must provide an APP_HOST_TEST no-op critical section",
              failures)
        check("taskENTER_CRITICAL" in generic_queue_source and
              "taskEXIT_CRITICAL" in generic_queue_source,
              "generic work queue state must be protected by ARM task critical sections",
              failures)
        check("configASSERT" in generic_queue_source and
              "xPortIsInsideInterrupt" in generic_queue_source,
              "generic Schedule APIs must reject ISR context in production builds",
              failures)

    scheduled_header = ROOT / "Dima/middleware/scheduling/scheduled_work_item.hpp"
    if scheduled_header.is_file():
        scheduled_source = scheduled_header.read_text(
            encoding="utf-8", errors="replace"
        ).lower()
        check("task context only" in scheduled_source,
              "ScheduledWorkItem Schedule APIs must be documented task-context-only",
              failures)
        check("not a quiescence barrier" in scheduled_source,
              "ScheduleClear must be documented as not being a quiescence barrier",
              failures)
        check("claimed" in scheduled_source and "running" in scheduled_source,
              "ScheduledWorkItem must distinguish Claimed and Running states",
              failures)

    freertos_header = ROOT / "Dima/middleware/scheduling/freertos_work_queue.hpp"
    if freertos_header.is_file():
        freertos_header_source = freertos_header.read_text(
            encoding="utf-8", errors="replace"
        ).lower()
        check("task context only" in freertos_header_source,
              "default work queue initialization must be documented task-context-only",
              failures)
        check("idempotent" in freertos_header_source,
              "default work queue initialization must document idempotence",
              failures)

    project_makefile = ROOT / "make/project.mk"
    if project_makefile.is_file():
        project_source = strip_comments(
            project_makefile.read_text(encoding="utf-8", errors="replace")
        )
        for source_path in (
            "Dima/middleware/lifecycle/module_manager.cpp",
            "Dima/middleware/scheduling/work_queue.cpp",
            "Dima/middleware/scheduling/scheduled_work_item.cpp",
            "Dima/middleware/scheduling/freertos_work_queue.cpp",
            "Dima/platform/freertos/platform_time.cpp",
        ):
            check(source_path in project_source,
                  f"make/project.mk must explicitly list {source_path}", failures)

    if failures:
        for failure in failures:
            print(f"STRUCTURE RED: {failure}", file=sys.stderr)
        return 1

    if runtime_only:
        print("STRUCTURE PASS: runtime core architecture boundary is satisfied")
    else:
        print("STRUCTURE PASS: application architecture boundary is satisfied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Runtime lifecycle, fault, clock, and linker safety checks."""

from __future__ import annotations

import re

from architecture.common import (
    ROOT,
    Violation,
    first_party_sources,
    line_for,
    require_literals,
    MAKE_CONTRACT_PATHS,
    owner_texts,
)


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
        ROOT / "Dima/platform/api/TaskRuntime.hpp": (
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
        ROOT / "Dima/platform/stm32h7/serial/SbusUart.cpp": (
            ("kDmaBufferSize = 64U", "R072",
             "SBUS DMA buffer size contract changed"),
            ("kReceiveRingCapacity = 256U", "R073",
             "SBUS CPU Ring capacity contract changed"),
            ("g_receive_ring", "R074",
             "SBUS CPU-only handoff Ring is missing"),
            ("reset_receive_epoch()", "R075",
             "SBUS receive epoch reset is missing"),
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
        ROOT / "Dima/platform/stm32h7/serial/SbusUartHal.cpp": (
            ("configure_sbus_rx_pin", "R180",
             "SBUS RX pin configuration is missing"),
            ("GPIO_PULLDOWN", "R181",
             "inverted SBUS must bias the physical RX line low"),
        ),
        ROOT / "Dima/middleware/parameters/definitions/rc_input_mapping_params.c": (
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

    sbus_backend_owners = (
        ROOT / "Dima/platform/stm32h7/serial/SbusUart.cpp",
        ROOT / "Dima/platform/stm32h7/serial/SbusUartHal.cpp",
    )
    for sbus_backend, text in owner_texts(sbus_backend_owners):
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

    for project_make, text in owner_texts(MAKE_CONTRACT_PATHS):
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
        ROOT / "Dima/platform/stm32h7/system/Clock.cpp": (
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

    clock_contract_owners = owner_texts((
        ROOT / "Core/Src/main.c",
        ROOT / "Core/Src/stm32h7xx_it.c",
        ROOT / "Dima/platform/stm32h7/system/Clock.cpp",
        *MAKE_CONTRACT_PATHS,
    ))
    for path, text in clock_contract_owners:
        for line_number, line in enumerate(text.splitlines(), 1):
            if "TIM12" in line or "htim12" in line:
                violations.append(Violation(
                    path, line_number, "R110",
                    "TIM12 must remain released from the system timebase",
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

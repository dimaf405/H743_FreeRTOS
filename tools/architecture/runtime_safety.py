"""故障所有权、单调时钟与链接布局安全门禁。"""

from __future__ import annotations

from architecture.common import (
    ROOT,
    MAKE_CONTRACT_PATHS,
    Violation,
    first_party_sources,
    line_for,
    owner_texts,
    require_literals,
)


def scan_fault_ownership(violations: list[Violation]) -> None:
    """限制诊断 Flash 写入只由故障捕获和 Bootloader 持久化 owner 使用。"""
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
    """核对板级时钟、共享 SysTick 和单调 HRT 的硬件合同。"""
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
    """核对应用向量、诊断区、DMA/heap 段及 MCUboot 槽位布局。"""
    require_literals(
        ROOT / "Linker/STM32H743VITx_MCUBOOT_APP.ld",
        (
            ("FLASH (rx)         : ORIGIN = 0x08040400", "R130",
             "Application FLASH origin changed"),
            (".dima_ramfunc", "R131", "RAM function section is missing"),
            (".dima_dma ORIGIN(RAM_DMA)", "R132",
             "DMA section ownership is missing"),
            (".dima_heap (NOLOAD)", "R133",
             "platform heap section is missing"),
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

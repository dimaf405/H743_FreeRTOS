#include "boot_diagnostics.h"

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"

extern uint32_t SystemD2Clock;

volatile dima_boot_diagnostics_t dima_boot_diagnostics
    __attribute__((used, section(".dima_boot_diag"), aligned(32)));

_Static_assert(sizeof(dima_boot_diagnostics_t) <= 512U,
               "boot diagnostics must remain a small fixed SRAM record");
_Static_assert(sizeof(dima_boot_diagnostics_v1_t) == 192U,
               "boot diagnostics v1 layout is a persistent ABI");
_Static_assert(sizeof(dima_boot_diagnostics_t) == 208U,
               "boot diagnostics v2 layout is a persistent ABI");

static int frame_is_readable(const uint32_t *frame)
{
    const uintptr_t begin = (uintptr_t)frame;
    const uintptr_t end = begin + 8U * sizeof(uint32_t);
    if (end < begin) {
        return 0;
    }

    return (begin >= 0x20000000UL && end <= 0x20020000UL) ||
           (begin >= 0x24000000UL && end <= 0x24080000UL) ||
           (begin >= 0x30000000UL && end <= 0x30048000UL) ||
           (begin >= 0x38000000UL && end <= 0x38010000UL);
}

static void clear_record(volatile dima_boot_diagnostics_t *record)
{
    volatile uint32_t *words = (volatile uint32_t *)(void *)record;
    const size_t count = sizeof(*record) / sizeof(uint32_t);
    for (size_t index = 0U; index < count; ++index) {
        words[index] = 0U;
    }
}

void dima_boot_diagnostics_early_init(void)
{
    /* STM32H743 SRAM4 has no run-mode clock gate in RCC_AHB4ENR. */

    const uint32_t previous_version = dima_boot_diagnostics.version;
    const uint32_t previous_size = dima_boot_diagnostics.size;
    const int previous_valid =
        dima_boot_diagnostics.magic == DIMA_BOOT_DIAGNOSTICS_MAGIC &&
        ((previous_version == DIMA_BOOT_DIAGNOSTICS_VERSION_V1 &&
          previous_size == sizeof(dima_boot_diagnostics_v1_t)) ||
         (previous_version == DIMA_BOOT_DIAGNOSTICS_VERSION &&
          previous_size == sizeof(dima_boot_diagnostics_t)));
    const uint32_t previous_boot_count =
        previous_valid ? dima_boot_diagnostics.boot_count : 0U;
    const uint32_t previous_stage =
        previous_valid ? dima_boot_diagnostics.stage : 0U;
    const uint32_t previous_failure =
        previous_valid ? dima_boot_diagnostics.failure_kind : 0U;
    const uint32_t previous_pc =
        previous_valid ? dima_boot_diagnostics.stacked_pc : 0U;
    const uint32_t previous_cfsr =
        previous_valid ? dima_boot_diagnostics.cfsr : 0U;
    const uint32_t previous_abfsr =
        previous_valid &&
                previous_version == DIMA_BOOT_DIAGNOSTICS_VERSION
            ? dima_boot_diagnostics.abfsr
            : 0U;

    clear_record(&dima_boot_diagnostics);
    dima_boot_diagnostics.version = DIMA_BOOT_DIAGNOSTICS_VERSION;
    dima_boot_diagnostics.size = sizeof(dima_boot_diagnostics_t);
    dima_boot_diagnostics.boot_count = previous_boot_count + 1U;
    dima_boot_diagnostics.reset_flags = RCC->RSR;
    dima_boot_diagnostics.stage = DIMA_BOOT_STAGE_SYSTEM_INIT;
    dima_boot_diagnostics.previous_stage = previous_stage;
    dima_boot_diagnostics.previous_failure_kind = previous_failure;
    dima_boot_diagnostics.previous_pc = previous_pc;
    dima_boot_diagnostics.previous_cfsr = previous_cfsr;
    dima_boot_diagnostics.abfsr = SCB->ABFSR;
    dima_boot_diagnostics.scb_ccr = SCB->CCR;
    dima_boot_diagnostics.mpu_ctrl = MPU->CTRL;
    dima_boot_diagnostics.previous_abfsr = previous_abfsr;
    __DMB();
    dima_boot_diagnostics.magic = DIMA_BOOT_DIAGNOSTICS_MAGIC;
    __DSB();
}

void dima_boot_stage_set(uint32_t stage)
{
    dima_boot_diagnostics.stage = stage;
    dima_boot_diagnostics.detail = 0U;
    __DMB();
}

void dima_boot_detail_set(uint32_t detail)
{
    dima_boot_diagnostics.detail = detail;
    __DMB();
}

static void capture_common(uint32_t failure_kind,
                           const uint32_t *stacked_frame,
                           uint32_t exception_return,
                           uint32_t detail,
                           uint32_t auxiliary)
{
    __disable_irq();

    dima_boot_diagnostics.capture_valid = 0U;
    dima_boot_diagnostics.failure_kind = failure_kind;
    dima_boot_diagnostics.exception_return = exception_return;

    if (frame_is_readable(stacked_frame)) {
        dima_boot_diagnostics.stacked_r0 = stacked_frame[0];
        dima_boot_diagnostics.stacked_r1 = stacked_frame[1];
        dima_boot_diagnostics.stacked_r2 = stacked_frame[2];
        dima_boot_diagnostics.stacked_r3 = stacked_frame[3];
        dima_boot_diagnostics.stacked_r12 = stacked_frame[4];
        dima_boot_diagnostics.stacked_lr = stacked_frame[5];
        dima_boot_diagnostics.stacked_pc = stacked_frame[6];
        dima_boot_diagnostics.stacked_xpsr = stacked_frame[7];
    } else {
        dima_boot_diagnostics.stacked_r0 = auxiliary;
        dima_boot_diagnostics.stacked_pc = detail;
    }

    dima_boot_diagnostics.msp = __get_MSP();
    dima_boot_diagnostics.psp = __get_PSP();
    dima_boot_diagnostics.primask = __get_PRIMASK();
    dima_boot_diagnostics.basepri = __get_BASEPRI();
    dima_boot_diagnostics.faultmask = __get_FAULTMASK();
    dima_boot_diagnostics.control = __get_CONTROL();

    dima_boot_diagnostics.cfsr = SCB->CFSR;
    dima_boot_diagnostics.hfsr = SCB->HFSR;
    dima_boot_diagnostics.dfsr = SCB->DFSR;
    dima_boot_diagnostics.afsr = SCB->AFSR;
    dima_boot_diagnostics.mmfar = SCB->MMFAR;
    dima_boot_diagnostics.bfar = SCB->BFAR;
    dima_boot_diagnostics.icsr = SCB->ICSR;
    dima_boot_diagnostics.shcsr = SCB->SHCSR;
    dima_boot_diagnostics.abfsr = SCB->ABFSR;
    dima_boot_diagnostics.scb_ccr = SCB->CCR;
    dima_boot_diagnostics.mpu_ctrl = MPU->CTRL;

    dima_boot_diagnostics.system_core_clock = SystemCoreClock;
    dima_boot_diagnostics.system_d2_clock = SystemD2Clock;
    dima_boot_diagnostics.rcc_cfgr = RCC->CFGR;
    dima_boot_diagnostics.rcc_d1cfgr = RCC->D1CFGR;
    dima_boot_diagnostics.rcc_d2cfgr = RCC->D2CFGR;
    dima_boot_diagnostics.systick_ctrl = SysTick->CTRL;
    dima_boot_diagnostics.systick_load = SysTick->LOAD;
    dima_boot_diagnostics.systick_value = SysTick->VAL;

    if ((RCC->APB1LENR & RCC_APB1LENR_TIM2EN) != 0U) {
        dima_boot_diagnostics.tim2_psc = TIM2->PSC;
        dima_boot_diagnostics.tim2_arr = TIM2->ARR;
        dima_boot_diagnostics.tim2_cnt = TIM2->CNT;
        dima_boot_diagnostics.tim2_sr = TIM2->SR;
    }

    __DMB();
    dima_boot_diagnostics.capture_valid =
        DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID;
    __DSB();
    __ISB();
}

__attribute__((noreturn)) void dima_boot_diagnostics_capture_fault(
    uint32_t failure_kind, const uint32_t *stacked_frame,
    uint32_t exception_return)
{
    capture_common(failure_kind, stacked_frame, exception_return, 0U, 0U);
    NVIC_SystemReset();
    for (;;) {
        __NOP();
    }
}

__attribute__((noreturn)) void dima_boot_diagnostics_panic(
    uint32_t failure_kind, uint32_t detail, uint32_t auxiliary)
{
    capture_common(failure_kind, NULL, 0U, detail, auxiliary);
    NVIC_SystemReset();
    for (;;) {
        __NOP();
    }
}

__attribute__((noreturn)) void dima_freertos_assert_failed(
    const char *file, uint32_t line)
{
    dima_boot_diagnostics_panic(
        DIMA_BOOT_FAILURE_FREERTOS_ASSERT, line,
        (uint32_t)(uintptr_t)file);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    dima_boot_diagnostics_panic(
        DIMA_BOOT_FAILURE_STACK_OVERFLOW,
        (uint32_t)(uintptr_t)task,
        (uint32_t)(uintptr_t)task_name);
}

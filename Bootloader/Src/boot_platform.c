#include "boot_platform.h"

#include "boot_layout.h"
#include "stm32h7xx_hal.h"

static int stack_pointer_is_valid(uint32_t stack_pointer)
{
    return (stack_pointer >= 0x20000000UL && stack_pointer <= 0x20020000UL) ||
           (stack_pointer >= 0x24000000UL && stack_pointer <= 0x24080000UL) ||
           (stack_pointer >= 0x30000000UL && stack_pointer <= 0x30048000UL) ||
           (stack_pointer >= 0x38000000UL && stack_pointer <= 0x38010000UL);
}

int boot_vector_is_valid(uint32_t vector_address)
{
    if (vector_address < H743_PRIMARY_SLOT_BASE ||
        vector_address >= H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE ||
        (vector_address & 0x3FFU) != 0U) {
        return 0;
    }

    const uint32_t stack_pointer = *(const uint32_t *)(uintptr_t)vector_address;
    const uint32_t reset_handler = *(const uint32_t *)(uintptr_t)(vector_address + 4U);

    return stack_pointer_is_valid(stack_pointer) &&
           (reset_handler & 1U) != 0U &&
           (reset_handler & ~1UL) >= vector_address &&
           (reset_handler & ~1UL) < H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE;
}

void boot_jump_to_application(uint32_t vector_address)
{
    const uint32_t stack_pointer = *(const uint32_t *)(uintptr_t)vector_address;
    const uint32_t reset_handler = *(const uint32_t *)(uintptr_t)(vector_address + 4U);

    __disable_irq();
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    for (uint32_t index = 0; index < 8U; ++index) {
        NVIC->ICER[index] = 0xFFFFFFFFUL;
        NVIC->ICPR[index] = 0xFFFFFFFFUL;
    }

    __DSB();
    __ISB();

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache();
        SCB_DisableDCache();
    }
    if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
        SCB_DisableICache();
    }

    SCB->VTOR = vector_address;
    __DSB();
    __ISB();

    __asm volatile(
        "msr msp, %0\n"
        "movs r0, #0\n"
        "msr control, r0\n"
        "isb\n"
        "cpsie i\n"
        "bx %1\n"
        :
        : "r"(stack_pointer), "r"(reset_handler)
        : "r0", "memory");

    __builtin_unreachable();
}

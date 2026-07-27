#include "Boards/H743/Inc/boot_layout.h"
#include "Boards/H743/Inc/board_init.h"
#include "stm32h7xx.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifndef EXPECT_BOARD_SD_INIT_AT_BOOT
#error "EXPECT_BOARD_SD_INIT_AT_BOOT must be defined by the board-init host target"
#endif

SCB_Type board_test_scb;

static const char *observed_calls[16];
static size_t observed_call_count;
static int vector_failure;
static unsigned int barrier_step;

static void record_call(const char *name)
{
    if (observed_call_count < (sizeof(observed_calls) / sizeof(observed_calls[0]))) {
        observed_calls[observed_call_count] = name;
    }
    ++observed_call_count;
}

void board_test_dsb(void)
{
    if (SCB->VTOR != H743_APP_VECTOR_BASE || barrier_step != 0U) {
        vector_failure = 1;
    }
    barrier_step = 1U;
}

void board_test_isb(void)
{
    if (SCB->VTOR != H743_APP_VECTOR_BASE || barrier_step != 1U) {
        vector_failure = 1;
    }
    barrier_step = 2U;
}

#define DEFINE_INIT_STUB(function_name) \
    void function_name(void) { record_call(#function_name); }

DEFINE_INIT_STUB(MX_GPIO_Init)
DEFINE_INIT_STUB(MX_DMA_Init)
DEFINE_INIT_STUB(MX_FDCAN1_Init)
DEFINE_INIT_STUB(MX_I2C2_Init)
DEFINE_INIT_STUB(MX_SDMMC1_SD_Init)
DEFINE_INIT_STUB(MX_SPI4_Init)
DEFINE_INIT_STUB(MX_UART4_Init)
DEFINE_INIT_STUB(MX_UART5_Init)
DEFINE_INIT_STUB(MX_UART7_Init)
DEFINE_INIT_STUB(MX_UART8_Init)
DEFINE_INIT_STUB(MX_USART1_UART_Init)
DEFINE_INIT_STUB(MX_USART2_UART_Init)
DEFINE_INIT_STUB(MX_USART3_UART_Init)
DEFINE_INIT_STUB(MX_USART6_UART_Init)
DEFINE_INIT_STUB(MX_TIM5_Init)
DEFINE_INIT_STUB(MX_TIM8_Init)

int main(void)
{
    static const char *const expected_calls[] = {
        "MX_GPIO_Init",
        "MX_DMA_Init",
        "MX_FDCAN1_Init",
        "MX_I2C2_Init",
#if EXPECT_BOARD_SD_INIT_AT_BOOT
        "MX_SDMMC1_SD_Init",
#endif
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
    };
    size_t index;

    SCB->VTOR = 0U;
    board_vector_table_init();
    if (SCB->VTOR != H743_APP_VECTOR_BASE || barrier_step != 2U || vector_failure != 0) {
        fprintf(stderr, "vector-table initialization order/value mismatch\n");
        return 1;
    }

    board_init();
    if (observed_call_count != (sizeof(expected_calls) / sizeof(expected_calls[0]))) {
        fprintf(stderr, "board init call count mismatch: got %zu\n", observed_call_count);
        return 1;
    }
    for (index = 0; index < observed_call_count; ++index) {
        if (strcmp(observed_calls[index], expected_calls[index]) != 0) {
            fprintf(stderr, "board init order mismatch at %zu: got %s expected %s\n",
                    index, observed_calls[index], expected_calls[index]);
            return 1;
        }
    }

    printf("board_init host test passed (BOARD_SD_INIT_AT_BOOT=%d)\n",
           EXPECT_BOARD_SD_INIT_AT_BOOT);
    return 0;
}

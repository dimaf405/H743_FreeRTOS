#ifndef TESTS_HOST_BOARD_INIT_SHIM_STM32H7XX_H
#define TESTS_HOST_BOARD_INIT_SHIM_STM32H7XX_H

#include <stdint.h>

typedef struct {
    uint32_t VTOR;
} SCB_Type;

extern SCB_Type board_test_scb;

#define SCB (&board_test_scb)
#define __DSB() board_test_dsb()
#define __ISB() board_test_isb()

void board_test_dsb(void);
void board_test_isb(void);

#endif

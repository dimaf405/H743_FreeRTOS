#ifndef DIMA_PLATFORM_STM32H7_FLASH_BANK1_H
#define DIMA_PLATFORM_STM32H7_FLASH_BANK1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool dima_stm32_flash_bank1_program(uint32_t address, const void *source,
                                    size_t length);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_FLASH_BANK1_H */

#ifndef DIMA_PLATFORM_STM32H7_FLASH_FAULT_H
#define DIMA_PLATFORM_STM32H7_FLASH_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int dima_flash_busfault_recover(uint32_t *stacked_frame);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_FLASH_FAULT_H */

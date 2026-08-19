#ifndef DIMA_PLATFORM_STM32H7_FLASH_FAULT_H
#define DIMA_PLATFORM_STM32H7_FLASH_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Core 提供 fail-closed 弱实现，FlashDevice 在安全读窗口内提供强实现。 */
int dima_flash_busfault_recover(uint32_t *stacked_frame);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_FLASH_FAULT_H */

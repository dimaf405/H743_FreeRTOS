#ifndef DIMA_PLATFORM_STM32H7_EARLY_MEMORY_H
#define DIMA_PLATFORM_STM32H7_EARLY_MEMORY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDMMC1 使用独立 D1 缓冲，地址由链接符号提供；两半区各 4 KiB。 */
#define DIMA_SD_DMA_REGION_SIZE    UINT32_C(0x00002000)

#define DIMA_DMA_REGION_BASE       UINT32_C(0x30040000)
#define DIMA_DMA_REGION_SIZE       UINT32_C(0x00008000)
#define DIMA_DIAGNOSTIC_REGION_BASE UINT32_C(0x38000000)
#define DIMA_DIAGNOSTIC_REGION_SIZE UINT32_C(0x00010000)

void dima_stm32_early_memory_init(void);
bool dima_stm32_memory_contract_verify(uint32_t *failure_mask);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_EARLY_MEMORY_H */

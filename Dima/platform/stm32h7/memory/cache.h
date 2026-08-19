#ifndef DIMA_PLATFORM_STM32H7_CACHE_H
#define DIMA_PLATFORM_STM32H7_CACHE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool dima_stm32_dcache_enabled(void);
void dima_stm32_cache_clean_range(const void *address, size_t length);
void dima_stm32_cache_invalidate_range(const void *address, size_t length);
void dima_stm32_cache_clean_invalidate_range(const void *address,
                                              size_t length);
void dima_stm32_cache_disable_for_handoff(void);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_CACHE_H */

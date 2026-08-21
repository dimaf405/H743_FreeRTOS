#ifndef DIMA_PLATFORM_STM32H7_CACHE_H
#define DIMA_PLATFORM_STM32H7_CACHE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void dima_stm32_cache_invalidate_range(const void *address, size_t length);
void dima_stm32_cache_disable_for_handoff(void);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_PLATFORM_STM32H7_CACHE_H */

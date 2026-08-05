#include "cache.h"

#include <stdint.h>

#include "stm32h7xx.h"

#define DIMA_CACHE_LINE_SIZE 32U

static int cache_range(const void *address, size_t length,
                       uintptr_t *aligned_address, int32_t *aligned_length)
{
    if (address == NULL || length == 0U ||
        (SCB->CCR & SCB_CCR_DC_Msk) == 0U) {
        return 0;
    }
    const uintptr_t begin = (uintptr_t)address;
    const uintptr_t end = begin + length;
    if (end < begin) {
        return 0;
    }
    const uintptr_t first = begin & ~(uintptr_t)(DIMA_CACHE_LINE_SIZE - 1U);
    const uintptr_t last =
        (end + DIMA_CACHE_LINE_SIZE - 1U) &
        ~(uintptr_t)(DIMA_CACHE_LINE_SIZE - 1U);
    if (last < first || last - first > INT32_MAX) {
        return 0;
    }
    *aligned_address = first;
    *aligned_length = (int32_t)(last - first);
    return 1;
}

bool dima_stm32_dcache_enabled(void)
{
    return (SCB->CCR & SCB_CCR_DC_Msk) != 0U;
}

void dima_stm32_cache_clean_range(const void *address, size_t length)
{
    uintptr_t aligned_address;
    int32_t aligned_length;
    if (!cache_range(address, length, &aligned_address, &aligned_length)) {
        return;
    }
    SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, aligned_length);
    __DSB();
    __ISB();
}

void dima_stm32_cache_invalidate_range(const void *address, size_t length)
{
    uintptr_t aligned_address;
    int32_t aligned_length;
    if (!cache_range(address, length, &aligned_address, &aligned_length)) {
        return;
    }
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_address, aligned_length);
    __DSB();
    __ISB();
}

void dima_stm32_cache_clean_invalidate_range(const void *address,
                                              size_t length)
{
    uintptr_t aligned_address;
    int32_t aligned_length;
    if (!cache_range(address, length, &aligned_address, &aligned_length)) {
        return;
    }
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)aligned_address,
                                     aligned_length);
    __DSB();
    __ISB();
}

void dima_stm32_cache_disable_for_handoff(void)
{
    __DSB();
    __ISB();
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache();
        SCB_DisableDCache();
    }
    if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
        SCB_DisableICache();
    }
    __DSB();
    __ISB();
}

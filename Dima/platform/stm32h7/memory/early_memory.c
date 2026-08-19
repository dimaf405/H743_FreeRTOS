#include "early_memory.h"

#include "stm32h7xx.h"

#define DIMA_MPU_DMA_REGION        6U
#define DIMA_MPU_DIAGNOSTIC_REGION 7U

#define DIMA_MPU_NORMAL_NONCACHEABLE \
    (MPU_RASR_XN_Msk | (3UL << MPU_RASR_AP_Pos) | \
     (1UL << MPU_RASR_TEX_Pos) | MPU_RASR_S_Msk)

#define DIMA_MPU_DMA_RASR \
    (DIMA_MPU_NORMAL_NONCACHEABLE | (14UL << MPU_RASR_SIZE_Pos) | \
     MPU_RASR_ENABLE_Msk)
#define DIMA_MPU_DIAGNOSTIC_RASR \
    (DIMA_MPU_NORMAL_NONCACHEABLE | (15UL << MPU_RASR_SIZE_Pos) | \
     MPU_RASR_ENABLE_Msk)

static void configure_region(uint32_t number, uint32_t base, uint32_t rasr)
{
    MPU->RNR = number;
    MPU->RBAR = base & MPU_RBAR_ADDR_Msk;
    MPU->RASR = rasr;
}

void dima_stm32_early_memory_init(void)
{
    __DSB();
    __ISB();

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanInvalidateDCache();
        SCB_DisableDCache();
    }
    if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
        SCB_DisableICache();
    }

    __DMB();
    MPU->CTRL = 0U;
    __DSB();
    __ISB();

    const uint32_t region_count =
        (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
    for (uint32_t region = 0U; region < region_count; ++region) {
        MPU->RNR = region;
        MPU->RASR = 0U;
    }

    configure_region(DIMA_MPU_DMA_REGION, DIMA_DMA_REGION_BASE,
                     DIMA_MPU_DMA_RASR);
    configure_region(DIMA_MPU_DIAGNOSTIC_REGION,
                     DIMA_DIAGNOSTIC_REGION_BASE,
                     DIMA_MPU_DIAGNOSTIC_RASR);

    /* Region 7 must remain non-cacheable while HardFault/NMI captures the
     * cross-reset record; otherwise the exception-mode MPU bypass can leave
     * the newest diagnostics only in D-cache. */
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk |
                MPU_CTRL_ENABLE_Msk;
    __DSB();
    __ISB();

    SCB_EnableICache();
    SCB_EnableDCache();
    __DSB();
    __ISB();
}

static bool region_matches(uint32_t number, uint32_t base,
                           uint32_t rasr)
{
    MPU->RNR = number;
    return (MPU->RBAR & MPU_RBAR_ADDR_Msk) ==
               (base & MPU_RBAR_ADDR_Msk) &&
           MPU->RASR == rasr;
}

bool dima_stm32_memory_contract_verify(uint32_t *failure_mask)
{
    uint32_t failures = 0U;
    if ((SCB->CCR & SCB_CCR_IC_Msk) == 0U) {
        failures |= 1UL << 0U;
    }
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) {
        failures |= 1UL << 1U;
    }
    const uint32_t required_control = MPU_CTRL_PRIVDEFENA_Msk |
                                      MPU_CTRL_HFNMIENA_Msk |
                                      MPU_CTRL_ENABLE_Msk;
    if ((MPU->CTRL & required_control) != required_control) {
        failures |= 1UL << 2U;
    }

    const uint32_t region_count =
        (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
    if (region_count <= DIMA_MPU_DIAGNOSTIC_REGION) {
        failures |= 1UL << 3U;
    } else {
        const uint32_t saved_region = MPU->RNR;
        if (!region_matches(DIMA_MPU_DMA_REGION, DIMA_DMA_REGION_BASE,
                            DIMA_MPU_DMA_RASR)) {
            failures |= 1UL << 4U;
        }
        if (!region_matches(DIMA_MPU_DIAGNOSTIC_REGION,
                            DIMA_DIAGNOSTIC_REGION_BASE,
                            DIMA_MPU_DIAGNOSTIC_RASR)) {
            failures |= 1UL << 5U;
        }
        MPU->RNR = saved_region;
    }

    if (failure_mask != NULL) {
        *failure_mask = failures;
    }
    return failures == 0U;
}

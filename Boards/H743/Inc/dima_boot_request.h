#ifndef DIMA_BOOT_REQUEST_H
#define DIMA_BOOT_REQUEST_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

/* BKP31R is reserved for one-shot MCUboot handoffs.  The backup domain
 * survives NVIC_SystemReset(), but not a backup-domain reset. */
#define DIMA_BOOT_REQUEST_RECOVERY_MAGIC UINT32_C(0xD14AB007)
#define DIMA_BOOT_REQUEST_APPLICATION_MAGIC UINT32_C(0xD14A4A50)
#define DIMA_BOOT_REQUEST_ACCESS_ATTEMPTS UINT32_C(1024)

static inline int dima_boot_request_enable_access(void)
{
    __HAL_RCC_RTC_CLK_ENABLE();
    SET_BIT(PWR->CR1, PWR_CR1_DBP);
    for (uint32_t attempt = 0U;
         attempt < DIMA_BOOT_REQUEST_ACCESS_ATTEMPTS;
         ++attempt) {
        if (READ_BIT(PWR->CR1, PWR_CR1_DBP) != 0U) {
            __DSB();
            return 1;
        }
    }
    return 0;
}

static inline uint32_t dima_boot_request_read(void)
{
    return dima_boot_request_enable_access() ? RTC->BKP31R : 0U;
}

static inline int dima_boot_request_write(uint32_t value)
{
    if (!dima_boot_request_enable_access()) {
        return 0;
    }
    RTC->BKP31R = value;
    __DSB();
    return RTC->BKP31R == value;
}

static inline int dima_boot_request_set_recovery(void)
{
    return dima_boot_request_write(DIMA_BOOT_REQUEST_RECOVERY_MAGIC);
}

static inline int dima_boot_request_set_application(void)
{
    return dima_boot_request_write(DIMA_BOOT_REQUEST_APPLICATION_MAGIC);
}

static inline int dima_boot_request_clear(void)
{
    return dima_boot_request_write(0U);
}

#endif /* DIMA_BOOT_REQUEST_H */

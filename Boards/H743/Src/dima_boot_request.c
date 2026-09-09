#include "dima_boot_request.h"
#include "stm32h7xx_hal.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

int dima_boot_request_enable_access(void)
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

uint32_t dima_boot_request_read(void)
{
    return dima_boot_request_enable_access() ? RTC->BKP31R : 0U;
}

int dima_boot_request_write(uint32_t value)
{
    if (!dima_boot_request_enable_access()) {
        return 0;
    }
    RTC->BKP31R = value;
    __DSB();
    return RTC->BKP31R == value;
}

int dima_boot_request_set_recovery(void)
{
    return dima_boot_request_write(DIMA_BOOT_REQUEST_RECOVERY_MAGIC);
}

int dima_boot_request_set_application(void)
{
    return dima_boot_request_write(DIMA_BOOT_REQUEST_APPLICATION_MAGIC);
}

int dima_boot_request_clear(void)
{
    return dima_boot_request_write(0U);
}

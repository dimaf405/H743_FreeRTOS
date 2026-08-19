#include "boot_watchdog.h"

#include "stm32h7xx.h"

#define BOOT_IWDG_WRITE_ACCESS_KEY UINT32_C(0x5555)
#define BOOT_IWDG_RELOAD_KEY UINT32_C(0xAAAA)
#define BOOT_IWDG_PRESCALER_DIV256 (IWDG_PR_PR_2 | IWDG_PR_PR_1)
#define BOOT_IWDG_RELOAD_MAX IWDG_RLR_RL_Msk
#define BOOT_IWDG_REGISTER_UPDATE_POLL_LIMIT UINT32_C(1000000)

void boot_watchdog_prepare(void)
{
    IWDG1->KR = BOOT_IWDG_WRITE_ACCESS_KEY;
    IWDG1->PR = BOOT_IWDG_PRESCALER_DIV256;
    IWDG1->RLR = BOOT_IWDG_RELOAD_MAX;
    IWDG1->WINR = IWDG_WINR_WIN;

    uint32_t polls = 0U;
    while (IWDG1->SR != 0U &&
           polls < BOOT_IWDG_REGISTER_UPDATE_POLL_LIMIT) {
        ++polls;
    }
    IWDG1->KR = BOOT_IWDG_RELOAD_KEY;
}

void boot_watchdog_feed(void)
{
    IWDG1->KR = BOOT_IWDG_RELOAD_KEY;
}

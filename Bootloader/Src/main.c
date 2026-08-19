#include "stm32h7xx_hal.h"

#include <stdint.h>

#include "boot_diagnostics_store.h"
#include "boot_layout.h"
#include "boot_platform.h"
#include "boot_primary.h"
#include "boot_serial/boot_serial.h"
#include "boot_usb.h"
#include "boot_watchdog.h"
#include "bootutil/bootutil.h"
#include "dima_boot_request.h"

#define USB_RECOVERY_WINDOW_MS 3000

static void SystemClock_Config(void);
static void USBClock_Config(void);
void Error_Handler(void);

static const struct boot_uart_funcs usb_console = {
    .read = boot_usb_console_read,
    .write = boot_usb_console_write,
};

int main(void)
{
    struct boot_rsp response;
    FIH_DECLARE(boot_result, FIH_FAILURE);

    /* IWDG cannot be stopped by reset. Give MCUboot a bounded ~32 s window
     * without starting it on an ordinary power-on where it is inactive. */
    boot_watchdog_prepare();

    uint32_t boot_request = dima_boot_request_read();
    const int captured_failure = dima_boot_diagnostics_capture_pending();

    /* Never carry the live USB/PLL state across the application boundary.
     * boot_go() sets this one-shot marker only after it has selected and
     * validated Primary.  The software reset gives this path reset-like
     * clocks and peripherals.  Primary is verified again without processing
     * the trailer, because a second boot_go() would immediately revert an
     * unconfirmed test image. */
    if (!captured_failure &&
        boot_request == DIMA_BOOT_REQUEST_APPLICATION_MAGIC) {
        uint32_t vector_address = 0U;
        if (dima_boot_request_clear() &&
            boot_primary_image_validate(&vector_address)) {
            boot_jump_to_application(vector_address);
        }
    }

    HAL_Init();
    SystemClock_Config();
    dima_boot_diagnostics_store_enable();
    USBClock_Config();

    if (captured_failure) {
        (void)dima_boot_request_clear();
        boot_request = 0U;
        (void)dima_boot_diagnostics_store_pending(1);

        /* A captured application failure must not be followed by another
         * automatic application jump. Keep USB recovery stable; the same
         * record is readable from 0x08020000 through STM32 ROM DFU. */
        if (boot_usb_init() == 0) {
            boot_serial_start(&usb_console);
        }
        for (;;) {
        }
    }

    if (boot_usb_init() == 0) {
        if (boot_request == DIMA_BOOT_REQUEST_RECOVERY_MAGIC) {
            /* Consume the request only after USB is ready.  mcumgr reset then
             * takes the ordinary boot path instead of re-entering recovery. */
            dima_boot_request_clear();
            boot_serial_start(&usb_console);
        } else {
            /* Any valid SMP request during this window keeps the device in
             * MCUboot recovery until mcumgr issues a reset. */
            boot_serial_check_start(&usb_console, USB_RECOVERY_WINDOW_MS);
        }
        boot_usb_deinit();
    }

    FIH_CALL(boot_go, boot_result, &response);
    if (FIH_EQ(boot_result, FIH_SUCCESS)) {
        const uint32_t vector_address = response.br_image_off +
                                        response.br_hdr->ih_hdr_size;
        /* A failed bridge attempt must stop in Recovery instead of creating
         * an endless reset loop.  Only images using this board's fixed header
         * and vector contract may request a new bridge reset. */
        if (boot_request != DIMA_BOOT_REQUEST_APPLICATION_MAGIC &&
            vector_address == H743_APP_VECTOR_BASE &&
            boot_vector_is_valid(vector_address)) {
            if (dima_boot_request_set_application() &&
                dima_boot_diagnostics_mark_application_bridge()) {
                NVIC_SystemReset();
            }

            /* Never fall back to a hot jump.  If the backup-domain marker
             * cannot be stored, keeping Recovery available is safer than
             * handing live USB/PLL/NVIC state to the application. */
            (void)dima_boot_request_clear();
        }
    }

    /* A missing or invalid primary image must never strand the board. The
     * STM32 ROM DFU remains available through BOOT0; MCUboot CDC recovery is
     * also kept alive here for normal signed-image repair. */
    if (boot_usb_init() == 0) {
        boot_serial_start(&usb_console);
    }

    for (;;) {
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 2;
    oscillator.PLL.PLLN = 240;
    oscillator.PLL.PLLP = 2;
    oscillator.PLL.PLLQ = 20;
    oscillator.PLL.PLLR = 2;
    oscillator.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

static void USBClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_USB;
    peripheral_clock.PLL3.PLL3M = 1;
    peripheral_clock.PLL3.PLL3N = 24;
    peripheral_clock.PLL3.PLL3P = 2;
    peripheral_clock.PLL3.PLL3Q = 4;
    peripheral_clock.PLL3.PLL3R = 2;
    peripheral_clock.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    peripheral_clock.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    peripheral_clock.PLL3.PLL3FRACN = 0;
    peripheral_clock.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
    }
}

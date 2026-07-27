#include "stm32h7xx_hal.h"

#include <stdint.h>

#include "boot_platform.h"
#include "boot_serial/boot_serial.h"
#include "boot_usb.h"
#include "bootutil/bootutil.h"

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

    HAL_Init();
    SystemClock_Config();
    USBClock_Config();

    if (boot_usb_init() == 0) {
        /* Any valid SMP request during this window keeps the device in
         * MCUboot recovery until mcumgr issues a reset. */
        boot_serial_check_start(&usb_console, USB_RECOVERY_WINDOW_MS);
        boot_usb_deinit();
    }

    FIH_CALL(boot_go, boot_result, &response);
    if (FIH_EQ(boot_result, FIH_SUCCESS)) {
        const uint32_t vector_address = response.br_image_off +
                                        response.br_hdr->ih_hdr_size;
        if (boot_vector_is_valid(vector_address)) {
            boot_jump_to_application(vector_address);
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

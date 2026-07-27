#include "Boards/H743/Inc/board_init.h"

#include "Boards/H743/Inc/boot_layout.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "stm32h7xx.h"
#include "tim.h"
#include "usart.h"

#ifndef BOARD_SD_INIT_AT_BOOT
#define BOARD_SD_INIT_AT_BOOT 0
#endif

void board_vector_table_init(void)
{
  SCB->VTOR = H743_APP_VECTOR_BASE;
  __DSB();
  __ISB();
}

void board_init(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
#if BOARD_SD_INIT_AT_BOOT
  MX_SDMMC1_SD_Init();
#endif
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
}

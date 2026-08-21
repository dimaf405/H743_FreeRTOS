#include "board_init.h"

#include "boot_diagnostics.h"
#include "boot_layout.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "i2c.h"
#include "motor_pwm.h"
#include "spi.h"
#include "stm32h7xx.h"
#include "tim.h"
#include "usart.h"

void board_vector_table_init(void)
{
  SCB->VTOR = H743_APP_VECTOR_BASE;
  __DSB();
  __ISB();
}

void board_init(void)
{
  dima_boot_stage_set(DIMA_BOOT_STAGE_GPIO);
  MX_GPIO_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_DMA);
  MX_DMA_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_FDCAN1);
  MX_FDCAN1_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_I2C2);
  MX_I2C2_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_SPI4);
  MX_SPI4_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_UART4);
  MX_UART4_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_UART5);
  MX_UART5_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_UART7);
  MX_UART7_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_UART8);
  MX_UART8_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_USART1);
  MX_USART1_UART_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_USART2);
  MX_USART2_UART_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_USART3);
  MX_USART3_UART_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_USART6);
  MX_USART6_UART_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_TIM5);
  MX_TIM5_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_TIM8);
  MX_TIM8_Init();
  dima_boot_stage_set(DIMA_BOOT_STAGE_MOTOR_PWM_SAFE);
  if (board_motor_pwm_stop() != BOARD_MOTOR_PWM_APPLIED)
  {
    Error_Handler();
  }
  dima_boot_stage_set(DIMA_BOOT_STAGE_BOARD_READY);
}

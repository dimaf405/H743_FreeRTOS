/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "boot_diagnostics.h"
#include "FreeRTOS.h"
#include "flash/flash_fault.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static volatile uint32_t g_hal_tick_suspended;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
__attribute__((noreturn)) void NMI_Handler_C(uint32_t *stacked_frame,
                                             uint32_t exception_return);
__attribute__((noreturn)) void HardFault_Handler_C(uint32_t *stacked_frame,
                                                   uint32_t exception_return);
__attribute__((noreturn)) void MemManage_Handler_C(uint32_t *stacked_frame,
                                                   uint32_t exception_return);
void BusFault_Handler_C(uint32_t *stacked_frame, uint32_t exception_return);
__attribute__((noreturn)) void UsageFault_Handler_C(uint32_t *stacked_frame,
                                                    uint32_t exception_return);
void dima_hrt_overflow_isr(void);
void dima_interrupt_sources_exti_isr(uint16_t pending_pins);
void xPortSysTickHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SuspendTick(void)
{
  /* SysTick 同时驱动 FreeRTOS，HAL_SuspendTick 只能暂停 HAL 的逻辑毫秒计数；
   * 不能关闭硬件 SysTick，否则会冻结任务调度与超时。 */
  g_hal_tick_suspended = 1U;
}

void HAL_ResumeTick(void)
{
  g_hal_tick_suspended = 0U;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
__attribute__((naked)) void NMI_Handler(void)
{
  __asm volatile (
      "mov r1, lr\n"
      "tst r1, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "tst r1, #0x10\n"
      "it eq\n"
      "addeq r0, r0, #72\n"
      "b NMI_Handler_C\n");
}

__attribute__((noreturn)) void NMI_Handler_C(uint32_t *stacked_frame,
                                             uint32_t exception_return)
{
  dima_boot_diagnostics_capture_fault(
      DIMA_BOOT_FAILURE_NMI, stacked_frame, exception_return);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile (
      "mov r1, lr\n"
      "tst r1, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "tst r1, #0x10\n"
      "it eq\n"
      "addeq r0, r0, #72\n"
      "b HardFault_Handler_C\n");
}

__attribute__((noreturn)) void HardFault_Handler_C(
    uint32_t *stacked_frame, uint32_t exception_return)
{
  dima_boot_diagnostics_capture_fault(
      DIMA_BOOT_FAILURE_HARDFAULT, stacked_frame, exception_return);
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  __asm volatile (
      "mov r1, lr\n"
      "tst r1, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "tst r1, #0x10\n"
      "it eq\n"
      "addeq r0, r0, #72\n"
      "b MemManage_Handler_C\n");
}

__attribute__((noreturn)) void MemManage_Handler_C(
    uint32_t *stacked_frame, uint32_t exception_return)
{
  dima_boot_diagnostics_capture_fault(
      DIMA_BOOT_FAILURE_MEMMANAGE, stacked_frame, exception_return);
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  /* Only an active, partition-bounded Flash safe-read window may recover.
   * Every other BusFault remains fail-closed. */
  __asm volatile (
      "mov r1, lr\n"
      "tst r1, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "tst r1, #0x10\n"
      "it eq\n"
      "addeq r0, r0, #72\n"
      "b BusFault_Handler_C\n");
}

__attribute__((weak)) int dima_flash_busfault_recover(uint32_t *stacked_frame)
{
  (void)stacked_frame;
  return 0;
}

void BusFault_Handler_C(uint32_t *stacked_frame, uint32_t exception_return)
{
  if (dima_flash_busfault_recover(stacked_frame) != 0)
  {
    return;
  }
  dima_boot_diagnostics_capture_fault(
      DIMA_BOOT_FAILURE_BUSFAULT, stacked_frame, exception_return);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile (
      "mov r1, lr\n"
      "tst r1, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "tst r1, #0x10\n"
      "it eq\n"
      "addeq r0, r0, #72\n"
      "b UsageFault_Handler_C\n");
}

__attribute__((noreturn)) void UsageFault_Handler_C(
    uint32_t *stacked_frame, uint32_t exception_return)
{
  dima_boot_diagnostics_capture_fault(
      DIMA_BOOT_FAILURE_USAGEFAULT, stacked_frame, exception_return);
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles the shared HAL and FreeRTOS 1 ms time base.
  */
void SysTick_Handler(void)
{
  /* Read CTRL to acknowledge COUNTFLAG, matching the CMSIS-RTOS handler. */
  (void)SysTick->CTRL;

  if (g_hal_tick_suspended == 0U)
  {
    HAL_IncTick();
  }

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    xPortSysTickHandler();
  }
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* HAL 负责清中断源并转入已注册回调；此处不进行 DroneCAN 协议解析。 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
  * @brief This function handles the TIM2 HRT overflow interrupt.
  */
void TIM2_IRQHandler(void)
{
  dima_hrt_overflow_isr();
}

/**
  * @brief This function handles EXTI lines 10 through 15.
  */
void EXTI15_10_IRQHandler(void)
{
  /* INT1/INT2 共用 EXTI15_10：先快照并清除全部 pending 位，再一次性交给
   * 角色化中断路由，避免分别回调造成重复唤醒或遗漏同拍事件。 */
  uint16_t pending_pins = 0U;

  if (__HAL_GPIO_EXTI_GET_IT(ICM42688_INT1_Pin) != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(ICM42688_INT1_Pin);
    pending_pins |= ICM42688_INT1_Pin;
  }
  if (__HAL_GPIO_EXTI_GET_IT(ICM42688_INT2_Pin) != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(ICM42688_INT2_Pin);
    pending_pins |= ICM42688_INT2_Pin;
  }
  if (pending_pins != 0U)
  {
    dima_interrupt_sources_exti_isr(pending_pins);
  }
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi4_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles SPI4 global interrupt.
  */
void SPI4_IRQHandler(void)
{
  /* USER CODE BEGIN SPI4_IRQn 0 */

  /* USER CODE END SPI4_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi4);
  /* USER CODE BEGIN SPI4_IRQn 1 */

  /* USER CODE END SPI4_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

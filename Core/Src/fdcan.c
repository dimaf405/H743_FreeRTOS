/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;

#define DIMA_FDCAN1_DEFAULT_BITRATE 500000U
#define DIMA_FDCAN1_KERNEL_CLOCK_HZ 48000000U
#define DIMA_FDCAN1_NOMINAL_TSEG1 13U
#define DIMA_FDCAN1_NOMINAL_TSEG2 2U
#define DIMA_FDCAN1_NOMINAL_SJW 2U
#define DIMA_FDCAN1_NOMINAL_TIME_QUANTA \
  (1U + DIMA_FDCAN1_NOMINAL_TSEG1 + DIMA_FDCAN1_NOMINAL_TSEG2)

/* 经典 CAN 标称速率 = 48 MHz / (prescaler * 16 TQ)，采样点位于
 * (SyncSeg + TSEG1) / 16 = 14/16 = 87.5%，SJW 为 2 TQ。编译期同时锁定
 * 125k/250k/500k/1M 四个受支持分频，时钟树变化时必须显式失败。 */
_Static_assert(DIMA_FDCAN1_KERNEL_CLOCK_HZ /
                   (24U * DIMA_FDCAN1_NOMINAL_TIME_QUANTA) == 125000U,
               "FDCAN1 125 kbit/s timing mismatch");
_Static_assert(DIMA_FDCAN1_KERNEL_CLOCK_HZ /
                   (12U * DIMA_FDCAN1_NOMINAL_TIME_QUANTA) == 250000U,
               "FDCAN1 250 kbit/s timing mismatch");
_Static_assert(DIMA_FDCAN1_KERNEL_CLOCK_HZ /
                   (6U * DIMA_FDCAN1_NOMINAL_TIME_QUANTA) == 500000U,
               "FDCAN1 500 kbit/s timing mismatch");
_Static_assert(DIMA_FDCAN1_KERNEL_CLOCK_HZ /
                   (3U * DIMA_FDCAN1_NOMINAL_TIME_QUANTA) == 1000000U,
               "FDCAN1 1 Mbit/s timing mismatch");

uint32_t DIMA_FDCAN1_NominalPrescaler(uint32_t bitrate)
{
  switch (bitrate)
  {
    case 125000U:  return 24U;
    case 250000U:  return 12U;
    case 500000U:  return 6U;
    case 1000000U: return 3U;
    default:       return 0U;
  }
}

HAL_StatusTypeDef DIMA_FDCAN1_ConfigureBitrate(uint32_t bitrate)
{
  const uint32_t nominal_prescaler =
      DIMA_FDCAN1_NominalPrescaler(bitrate);
  if (nominal_prescaler == 0U)
  {
    return HAL_ERROR;
  }

  /* HAL 不允许在运行态直接改 timing；先完整 DeInit，再用新的离散参数重建。 */
  if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_RESET)
  {
    if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = nominal_prescaler;
  hfdcan1.Init.NominalSyncJumpWidth = DIMA_FDCAN1_NOMINAL_SJW;
  hfdcan1.Init.NominalTimeSeg1 = DIMA_FDCAN1_NOMINAL_TSEG1;
  hfdcan1.Init.NominalTimeSeg2 = DIMA_FDCAN1_NOMINAL_TSEG2;
  hfdcan1.Init.DataPrescaler = nominal_prescaler;
  hfdcan1.Init.DataSyncJumpWidth = DIMA_FDCAN1_NOMINAL_SJW;
  hfdcan1.Init.DataTimeSeg1 = DIMA_FDCAN1_NOMINAL_TSEG1;
  hfdcan1.Init.DataTimeSeg2 = DIMA_FDCAN1_NOMINAL_TSEG2;
  /* 当前使用 Classic CAN，data timing 不参与总线采样，但 HAL 仍要求字段合法。
   * Message RAM 为单实例从偏移 0 使用，扩展帧过滤器服务 DroneCAN。 */
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  return HAL_FDCAN_Init(&hfdcan1);
}

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  if (DIMA_FDCAN1_ConfigureBitrate(DIMA_FDCAN1_DEFAULT_BITRATE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

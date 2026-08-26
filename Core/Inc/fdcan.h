/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void MX_FDCAN1_Init(void);
/* bitrate 仅接受板级合同列出的离散值；返回 0/ HAL_ERROR 表示不支持，调用方
 * 不得用近似分频静默替代。 */
HAL_StatusTypeDef DIMA_FDCAN1_ConfigureBitrate(uint32_t bitrate);
uint32_t DIMA_FDCAN1_NominalPrescaler(uint32_t bitrate);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

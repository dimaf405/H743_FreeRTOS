/*-----------------------------------------------------------------------*/
/* Low level FatFs disk I/O module for STM32H743 SDMMC1                */
/*                                                                       */
/* Copyright (C) 2017, ChaN, all right reserved.                         */
/* Portions Copyright (C) STMicroelectronics, all right reserved.        */
/* Portions Copyright (C) Dima Project, all right reserved.              */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "sdmmc.h"           /* hsd1, HAL_SD handle */
#include "stm32h7xx_hal.h"   /* HAL_SD_xxx, SCB cache */
#include <string.h>           /* memcpy */

/* Private defines ---------------------------------------------------------*/

#define SD_TIMEOUT_MS      500U  /* Stay below the application IWDG window. */
#define SD_BLOCK_SIZE    512U

/*
 * Scratch buffer for callers that are not word-aligned as required by the
 * polling HAL FIFO path. Keep it in the board's aligned, non-cacheable D2 area.
 */
static uint8_t scratch[SD_BLOCK_SIZE]
  __attribute__((aligned(32), section(".dima_dma")));

/* Private variables -------------------------------------------------------*/

static volatile DSTATUS Stat = STA_NOINIT;

/* Private functions -------------------------------------------------------*/

static void SD_ConfigureHandle(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
}

/**
  * @brief  Check SD card presence and update status.
  */
static DSTATUS SD_CheckStatus(void)
{
  Stat = STA_NOINIT;
  if (hsd1.Instance != SDMMC1) {
    return Stat;
  }
  if (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) {
    Stat &= ~STA_NOINIT;
  }
  return Stat;
}

static DSTATUS SD_Reinitialize(void)
{
  Stat = STA_NOINIT;
  if (hsd1.Instance == SDMMC1) {
    (void)HAL_SD_DeInit(&hsd1);
  }
  SD_ConfigureHandle();
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
    return Stat;
  }
  return SD_CheckStatus();
}

static DRESULT SD_WaitReady(void)
{
  const uint32_t started = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
    if ((HAL_GetTick() - started) >= SD_TIMEOUT_MS) {
      Stat = STA_NOINIT;
      return RES_ERROR;
    }
  }
  return RES_OK;
}

/*-----------------------------------------------------------------------*
 * Disk Interface Functions                                               *
 *-----------------------------------------------------------------------*/

/**
  * @brief  Initialize disk drive.
  * @note   Initialization is intentionally non-fatal and retryable so a card
  *         can be inserted after boot without a card-detect GPIO.
  */
DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != 0) return STA_NOINIT;

  if (SD_CheckStatus() == 0U) return 0U;
  return SD_Reinitialize();
}

/**
  * @brief  Get disk status.
  */
DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != 0) return STA_NOINIT;
  return SD_CheckStatus();
}

/**
  * @brief  Read sector(s) from SD card.
  */
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  HAL_StatusTypeDef hret;

  if ((uint32_t)buff & 0x3U) {
    /* Unaligned buffer: use scratch buffer sector-by-sector */
    for (UINT i = 0; i < count; i++) {
      hret = HAL_SD_ReadBlocks(&hsd1, scratch, sector + i, 1, SD_TIMEOUT_MS);
      if (hret != HAL_OK) { Stat = STA_NOINIT; return RES_ERROR; }
      memcpy(buff, scratch, SD_BLOCK_SIZE);
      buff += SD_BLOCK_SIZE;
    }
  } else {
    /* Aligned buffer: direct multi-block read */
    hret = HAL_SD_ReadBlocks(&hsd1, buff, sector, count, SD_TIMEOUT_MS);
    if (hret != HAL_OK) { Stat = STA_NOINIT; return RES_ERROR; }
  }

  return RES_OK;
}

/**
  * @brief  Write sector(s) to SD card.
  */
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  HAL_StatusTypeDef hret;

  if ((uint32_t)buff & 0x3U) {
    /* Unaligned buffer: use scratch buffer sector-by-sector */
    for (UINT i = 0; i < count; i++) {
      memcpy(scratch, buff, SD_BLOCK_SIZE);
      hret = HAL_SD_WriteBlocks(&hsd1, scratch, sector + i, 1, SD_TIMEOUT_MS);
      if (hret != HAL_OK) { Stat = STA_NOINIT; return RES_ERROR; }
      if (SD_WaitReady() != RES_OK) return RES_ERROR;
      buff += SD_BLOCK_SIZE;
    }
  } else {
    /* Aligned buffer: direct multi-block write */
    hret = HAL_SD_WriteBlocks(&hsd1, (uint8_t *)buff, sector, count, SD_TIMEOUT_MS);
    if (hret != HAL_OK) { Stat = STA_NOINIT; return RES_ERROR; }
    if (SD_WaitReady() != RES_OK) return RES_ERROR;
  }

  return RES_OK;
}

/**
  * @brief  I/O control operation.
  */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  if (pdrv != 0) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  HAL_SD_CardInfoTypeDef info;

  switch (cmd) {
  case CTRL_SYNC:
    return SD_WaitReady();

  case GET_SECTOR_COUNT:
    HAL_SD_GetCardInfo(&hsd1, &info);
    *(DWORD *)buff = info.LogBlockNbr;
    return RES_OK;

  case GET_SECTOR_SIZE:
    HAL_SD_GetCardInfo(&hsd1, &info);
    *(WORD *)buff = (WORD)info.LogBlockSize;
    return RES_OK;

  case GET_BLOCK_SIZE:
    HAL_SD_GetCardInfo(&hsd1, &info);
    *(DWORD *)buff = info.LogBlockSize / SD_BLOCK_SIZE;
    return RES_OK;

  case MMC_GET_TYPE:
    HAL_SD_GetCardInfo(&hsd1, &info);
    *(BYTE *)buff = (info.CardType == CARD_SDHC_SDXC) ? 2 : 1;
    return RES_OK;

  default:
    return RES_PARERR;
  }
}

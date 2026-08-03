#include "mcuboot_app.h"

#include <stdint.h>
#include <string.h>

#include "boot_layout.h"
#include "freertos/flash_operation_lock.h"
#include "stm32h7xx_hal.h"

#define MCUBOOT_MAGIC_SIZE       16U
#define MCUBOOT_IMAGE_OK_ADDRESS \
    (H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE - 64U)
#define MCUBOOT_MAGIC_ADDRESS \
    (H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE - MCUBOOT_MAGIC_SIZE)

static const uint8_t mcuboot_magic[MCUBOOT_MAGIC_SIZE] = {
    0x20U, 0x00U, 0x2dU, 0xe1U,
    0x5dU, 0x29U, 0x41U, 0x0bU,
    0x8dU, 0x77U, 0x67U, 0x9cU,
    0x11U, 0x0fU, 0x1fU, 0x8aU,
};

int mcuboot_confirm_running_image(void)
{
    int result = MCUBOOT_CONFIRM_FLASH_ERROR;
    if (!dima_flash_operation_lock()) {
        return result;
    }

    const uint8_t *magic = (const uint8_t *)(uintptr_t)MCUBOOT_MAGIC_ADDRESS;
    const uint8_t *image_ok = (const uint8_t *)(uintptr_t)MCUBOOT_IMAGE_OK_ADDRESS;

    if (memcmp(magic, mcuboot_magic, sizeof(mcuboot_magic)) != 0) {
        result = MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE;
        goto out;
    }
    if (*image_ok == 0x01U) {
        result = MCUBOOT_CONFIRM_ALREADY_CONFIRMED;
        goto out;
    }
    if (*image_ok != 0xFFU) {
        goto out;
    }

    uint32_t flash_word[H743_FLASH_WRITE_SIZE / sizeof(uint32_t)]
        __attribute__((aligned(H743_FLASH_WRITE_SIZE)));
    memset(flash_word, 0xFF, sizeof(flash_word));
    ((uint8_t *)flash_word)[0] = 0x01U;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        goto out;
    }
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1);

    const HAL_StatusTypeDef status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          MCUBOOT_IMAGE_OK_ADDRESS,
                          (uint32_t)(uintptr_t)flash_word);
    (void)HAL_FLASH_Lock();

    __DSB();
    __ISB();
    if (status == HAL_OK && *image_ok == 0x01U) {
        result = MCUBOOT_CONFIRM_OK;
    }

out:
    dima_flash_operation_unlock();
    return result;
}

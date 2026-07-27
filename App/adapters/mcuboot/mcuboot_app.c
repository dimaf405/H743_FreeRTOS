#include "App/adapters/mcuboot/mcuboot_app.h"

#include <stdint.h>
#include <string.h>

#include "Boards/H743/Inc/boot_layout.h"
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
    const uint8_t *magic = (const uint8_t *)(uintptr_t)MCUBOOT_MAGIC_ADDRESS;
    const uint8_t *image_ok = (const uint8_t *)(uintptr_t)MCUBOOT_IMAGE_OK_ADDRESS;

    if (memcmp(magic, mcuboot_magic, sizeof(mcuboot_magic)) != 0) {
        return MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE;
    }
    if (*image_ok == 0x01U) {
        return MCUBOOT_CONFIRM_ALREADY_CONFIRMED;
    }
    if (*image_ok != 0xFFU) {
        return MCUBOOT_CONFIRM_FLASH_ERROR;
    }

    uint32_t flash_word[H743_FLASH_WRITE_SIZE / sizeof(uint32_t)]
        __attribute__((aligned(H743_FLASH_WRITE_SIZE)));
    memset(flash_word, 0xFF, sizeof(flash_word));
    ((uint8_t *)flash_word)[0] = 0x01U;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return MCUBOOT_CONFIRM_FLASH_ERROR;
    }
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1);

    const HAL_StatusTypeDef status =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          MCUBOOT_IMAGE_OK_ADDRESS,
                          (uint32_t)(uintptr_t)flash_word);
    (void)HAL_FLASH_Lock();

    __DSB();
    __ISB();
    return (status == HAL_OK && *image_ok == 0x01U)
               ? MCUBOOT_CONFIRM_OK
               : MCUBOOT_CONFIRM_FLASH_ERROR;
}

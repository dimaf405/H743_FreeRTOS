#include "flash_map_backend/flash_map_backend.h"

#include <errno.h>
#include <string.h>

#include "boot_layout.h"
#include "stm32h7xx_hal.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const struct flash_area flash_areas[] = {
    {
        .fa_id = FLASH_AREA_BOOTLOADER,
        .fa_device_id = 0,
        .fa_off = H743_MCUBOOT_BASE,
        .fa_size = H743_MCUBOOT_SIZE,
    },
    {
        .fa_id = FLASH_AREA_IMAGE_PRIMARY_ID,
        .fa_device_id = 0,
        .fa_off = H743_PRIMARY_SLOT_BASE,
        .fa_size = H743_PRIMARY_SLOT_SIZE,
    },
    {
        .fa_id = FLASH_AREA_IMAGE_SECONDARY_ID,
        .fa_device_id = 0,
        .fa_off = H743_SECONDARY_SLOT_BASE,
        .fa_size = H743_SECONDARY_SLOT_SIZE,
    },
    {
        .fa_id = FLASH_AREA_IMAGE_SCRATCH,
        .fa_device_id = 0,
        .fa_off = H743_SCRATCH_BASE,
        .fa_size = H743_SCRATCH_SIZE,
    },
};

static int range_valid(const struct flash_area *fa, uint32_t off, uint32_t len)
{
    return fa != NULL && off <= fa->fa_size && len <= (fa->fa_size - off);
}

static uint32_t bank_for_address(uint32_t address)
{
    return address < H743_FLASH_BANK2_BASE ? FLASH_BANK_1 : FLASH_BANK_2;
}

static uint32_t sector_for_address(uint32_t address)
{
    const uint32_t bank_base = address < H743_FLASH_BANK2_BASE
                                   ? H743_FLASH_BASE
                                   : H743_FLASH_BANK2_BASE;
    return (address - bank_base) / H743_FLASH_SECTOR_SIZE;
}

static void clear_flash_errors(uint32_t bank)
{
    if (bank == FLASH_BANK_1) {
        __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1);
    } else {
        __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);
    }
}

int flash_area_open(uint8_t id, const struct flash_area **fa)
{
    if (fa == NULL) {
        return -EINVAL;
    }

    for (size_t index = 0; index < ARRAY_SIZE(flash_areas); ++index) {
        if (flash_areas[index].fa_id == id) {
            *fa = &flash_areas[index];
            return 0;
        }
    }

    *fa = NULL;
    return -ENOENT;
}

void flash_area_close(const struct flash_area *fa)
{
    (void)fa;
}

int flash_area_read(const struct flash_area *fa, uint32_t off, void *dst, uint32_t len)
{
    if (!range_valid(fa, off, len) || (dst == NULL && len != 0U)) {
        return -EINVAL;
    }

    memcpy(dst, (const void *)(uintptr_t)(fa->fa_off + off), len);
    return 0;
}

int flash_area_write(const struct flash_area *fa, uint32_t off, const void *src, uint32_t len)
{
    if (!range_valid(fa, off, len) || (src == NULL && len != 0U) ||
        ((fa->fa_off + off) % H743_FLASH_WRITE_SIZE) != 0U ||
        (len % H743_FLASH_WRITE_SIZE) != 0U) {
        return -EINVAL;
    }

    if (len == 0U) {
        return 0;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return -EIO;
    }

    int result = 0;
    const uint8_t *input = (const uint8_t *)src;
    uint32_t address = fa->fa_off + off;
    uint32_t flash_word[H743_FLASH_WRITE_SIZE / sizeof(uint32_t)]
        __attribute__((aligned(H743_FLASH_WRITE_SIZE)));

    while (len != 0U) {
        memcpy(flash_word, input, H743_FLASH_WRITE_SIZE);
        clear_flash_errors(bank_for_address(address));

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address,
                              (uint32_t)(uintptr_t)flash_word) != HAL_OK ||
            memcmp((const void *)(uintptr_t)address, flash_word,
                   H743_FLASH_WRITE_SIZE) != 0) {
            result = -EIO;
            break;
        }

        address += H743_FLASH_WRITE_SIZE;
        input += H743_FLASH_WRITE_SIZE;
        len -= H743_FLASH_WRITE_SIZE;
    }

    (void)HAL_FLASH_Lock();
    __DSB();
    __ISB();
    return result;
}

int flash_area_erase(const struct flash_area *fa, uint32_t off, uint32_t len)
{
    if (!range_valid(fa, off, len) || len == 0U ||
        ((fa->fa_off + off) % H743_FLASH_SECTOR_SIZE) != 0U ||
        (len % H743_FLASH_SECTOR_SIZE) != 0U) {
        return -EINVAL;
    }

    const uint32_t first_address = fa->fa_off + off;
    const uint32_t last_address = first_address + len - 1U;
    if (bank_for_address(first_address) != bank_for_address(last_address)) {
        return -EINVAL;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return -EIO;
    }

    const uint32_t bank = bank_for_address(first_address);
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = bank,
        .Sector = sector_for_address(first_address),
        .NbSectors = len / H743_FLASH_SECTOR_SIZE,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0xFFFFFFFFUL;

    clear_flash_errors(bank);
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();

    if (status != HAL_OK || sector_error != 0xFFFFFFFFUL) {
        return -EIO;
    }

    __DSB();
    __ISB();
    return 0;
}

uint32_t flash_area_align(const struct flash_area *fa)
{
    (void)fa;
    return H743_FLASH_WRITE_SIZE;
}

uint8_t flash_area_erased_val(const struct flash_area *fa)
{
    (void)fa;
    return 0xFFU;
}

bool flash_area_erase_required(const struct flash_area *fa)
{
    (void)fa;
    return true;
}

int flash_area_get_sector(const struct flash_area *fa, off_t off,
                          struct flash_sector *sector)
{
    if (fa == NULL || sector == NULL || off < 0 || (uint32_t)off >= fa->fa_size) {
        return -EINVAL;
    }

    sector->fs_off = ((uint32_t)off / H743_FLASH_SECTOR_SIZE) *
                     H743_FLASH_SECTOR_SIZE;
    sector->fs_size = H743_FLASH_SECTOR_SIZE;
    return 0;
}

int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors)
{
    const struct flash_area *fa;
    if (count == NULL || flash_area_open((uint8_t)fa_id, &fa) != 0) {
        return -EINVAL;
    }

    const uint32_t required = fa->fa_size / H743_FLASH_SECTOR_SIZE;
    if (sectors == NULL || *count < required) {
        *count = required;
        return -ENOMEM;
    }

    for (uint32_t index = 0; index < required; ++index) {
        sectors[index].fs_off = index * H743_FLASH_SECTOR_SIZE;
        sectors[index].fs_size = H743_FLASH_SECTOR_SIZE;
    }
    *count = required;
    return 0;
}

int flash_area_id_from_multi_image_slot(int image_index, int slot)
{
    if (image_index != 0) {
        return -EINVAL;
    }
    if (slot == 0) {
        return FLASH_AREA_IMAGE_PRIMARY_ID;
    }
    if (slot == 1) {
        return FLASH_AREA_IMAGE_SECONDARY_ID;
    }
    return -EINVAL;
}

int flash_area_id_to_multi_image_slot(int image_index, int area_id)
{
    if (image_index != 0) {
        return -EINVAL;
    }
    if (area_id == FLASH_AREA_IMAGE_PRIMARY_ID) {
        return 0;
    }
    if (area_id == FLASH_AREA_IMAGE_SECONDARY_ID) {
        return 1;
    }
    return -EINVAL;
}

int flash_area_id_from_image_slot(int slot)
{
    return flash_area_id_from_multi_image_slot(0, slot);
}

int flash_area_id_from_direct_image(int image_id)
{
    /* Image ID 0 (omitted by older mcumgr clients) is deliberately safe:
     * it writes the secondary slot, never the running primary image. */
    if (image_id == 0 || image_id == 2) {
        return FLASH_AREA_IMAGE_SECONDARY_ID;
    }
    return -EINVAL;
}

int flash_device_base(uint8_t fd_id, uintptr_t *ret)
{
    if (fd_id != 0U || ret == NULL) {
        return -EINVAL;
    }
    *ret = 0U;
    return 0;
}

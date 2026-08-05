#include "flash_bank1.h"

#include "boot_layout.h"
#include "cache.h"
#include "stm32h7xx_hal.h"

#define DIMA_FLASH_BUSY_MASK \
    (FLASH_FLAG_QW_BANK1 | FLASH_FLAG_BSY_BANK1 | FLASH_FLAG_WBNE_BANK1)
#define DIMA_FLASH_CLEAR_MASK \
    (FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1)
#define DIMA_FLASH_WORD_U32_COUNT \
    (H743_FLASH_WRITE_SIZE / sizeof(uint32_t))
#define DIMA_FLASH_PROGRAM_POLL_LIMIT UINT32_C(10000000)

__attribute__((noinline, section(".dima_ramfunc")))
static int program_from_dtcm(uint32_t address, const uint32_t *source,
                             size_t flash_word_count)
{
    if ((FLASH->SR1 & DIMA_FLASH_BUSY_MASK) != 0U) {
        return 0;
    }
    if ((FLASH->CR1 & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
        if ((FLASH->CR1 & FLASH_CR_LOCK) != 0U) {
            return 0;
        }
    }

    FLASH->CCR1 = DIMA_FLASH_CLEAR_MASK;
    int success = 1;
    for (size_t flash_word = 0U; flash_word < flash_word_count;
         ++flash_word) {
        if ((FLASH->SR1 & DIMA_FLASH_BUSY_MASK) != 0U) {
            success = 0;
            break;
        }

        FLASH->CR1 |= FLASH_CR_PG;
        __ISB();
        __DSB();
        volatile uint32_t *destination =
            (volatile uint32_t *)(uintptr_t)
                (address + flash_word * H743_FLASH_WRITE_SIZE);
        const uint32_t *input =
            source + flash_word * DIMA_FLASH_WORD_U32_COUNT;
        for (size_t word = 0U; word < DIMA_FLASH_WORD_U32_COUNT; ++word) {
            destination[word] = input[word];
        }
        __ISB();
        __DSB();

        uint32_t polls_remaining = DIMA_FLASH_PROGRAM_POLL_LIMIT;
        while ((FLASH->SR1 & DIMA_FLASH_BUSY_MASK) != 0U &&
               polls_remaining != 0U) {
            --polls_remaining;
        }
        const uint32_t status = FLASH->SR1;
        FLASH->CR1 &= ~FLASH_CR_PG;
        if (polls_remaining == 0U ||
            (status & FLASH_FLAG_ALL_ERRORS_BANK1) != 0U) {
            success = 0;
        }
        FLASH->CCR1 = status & DIMA_FLASH_CLEAR_MASK;
        if (!success) {
            break;
        }
    }

    FLASH->CR1 |= FLASH_CR_LOCK;
    __DSB();
    __ISB();
    return success;
}

bool dima_stm32_flash_bank1_program(uint32_t address, const void *source,
                                    size_t length)
{
    if (source == NULL || length == 0U ||
        (address % H743_FLASH_WRITE_SIZE) != 0U ||
        (length % H743_FLASH_WRITE_SIZE) != 0U ||
        address < H743_FLASH_BASE || address + length < address ||
        address + length > H743_FLASH_BANK2_BASE) {
        return false;
    }

    const uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    const int programmed = program_from_dtcm(
        address, (const uint32_t *)source,
        length / H743_FLASH_WRITE_SIZE);
    __DSB();
    __ISB();
    if (saved_primask == 0U) {
        __enable_irq();
    }
    if (!programmed) {
        return false;
    }

    dima_stm32_cache_invalidate_range((const void *)(uintptr_t)address,
                                      length);
    const uint8_t *written = (const uint8_t *)(uintptr_t)address;
    const uint8_t *expected = (const uint8_t *)source;
    for (size_t index = 0U; index < length; ++index) {
        if (written[index] != expected[index]) {
            return false;
        }
    }
    return true;
}

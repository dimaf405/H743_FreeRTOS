#include "boot_diagnostics_store.h"

#include <stddef.h>
#include <stdint.h>

#include "boot_diagnostics.h"
#include "boot_layout.h"
#include "stm32h7xx_hal.h"

#define RECORD_COUNT \
    (H743_BOOT_DIAGNOSTICS_SIZE / DIMA_BOOT_FLASH_RECORD_SIZE)
#define FLASH_PROGRAM_WORD_COUNT \
    (DIMA_BOOT_FLASH_RECORD_SIZE / H743_FLASH_WRITE_SIZE)
#define FLASH_WORD_U32_COUNT \
    (H743_FLASH_WRITE_SIZE / sizeof(uint32_t))

#define STORE_STATE_ENABLED UINT32_C(0x53544F52)
#define STORE_STATE_BUSY    UINT32_C(0x42555359)
#define FLASH_BUSY_MASK \
    (FLASH_FLAG_QW_BANK1 | FLASH_FLAG_BSY_BANK1 | FLASH_FLAG_WBNE_BANK1)
#define FLASH_CLEAR_MASK \
    (FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1)
#define FLASH_PROGRAM_POLL_LIMIT UINT32_C(10000000)

static dima_boot_flash_record_t staging_record
    __attribute__((aligned(H743_FLASH_WRITE_SIZE)));
static volatile uint32_t store_state;

_Static_assert(sizeof(dima_boot_diagnostics_t) == 192U,
               "Update the DFU parser when boot diagnostics changes");
_Static_assert(sizeof(dima_boot_flash_record_t) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE,
               "Boot diagnostics Flash record must occupy one slot");
_Static_assert(offsetof(dima_boot_flash_record_t, commit) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE - sizeof(uint32_t),
               "Commit marker must be in the final Flash word");
_Static_assert((H743_BOOT_DIAGNOSTICS_SIZE %
                DIMA_BOOT_FLASH_RECORD_SIZE) == 0U,
               "Boot diagnostics sector must contain whole records");
_Static_assert((DIMA_BOOT_FLASH_RECORD_SIZE %
                H743_FLASH_WRITE_SIZE) == 0U,
               "Boot diagnostics records must contain whole Flash words");

static const volatile dima_boot_diagnostics_t *capture_record(void)
{
    return (const volatile dima_boot_diagnostics_t *)(uintptr_t)
        DIMA_BOOT_DIAGNOSTICS_ADDRESS;
}

static uint32_t crc32_bytes(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    for (size_t index = 0U; index < length; ++index) {
        crc ^= bytes[index];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return ~crc;
}

static int capture_is_valid(const volatile dima_boot_diagnostics_t *capture)
{
    return capture->magic == DIMA_BOOT_DIAGNOSTICS_MAGIC &&
           capture->version == DIMA_BOOT_DIAGNOSTICS_VERSION &&
           capture->size == sizeof(dima_boot_diagnostics_t) &&
           capture->capture_valid == DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID &&
           capture->failure_kind != DIMA_BOOT_FAILURE_NONE;
}

static const dima_boot_flash_record_t *flash_record(uint32_t index)
{
    return (const dima_boot_flash_record_t *)(uintptr_t)
        (H743_BOOT_DIAGNOSTICS_BASE +
         index * DIMA_BOOT_FLASH_RECORD_SIZE);
}

static int record_is_erased(const dima_boot_flash_record_t *record)
{
    const uint32_t *words = (const uint32_t *)(const void *)record;
    for (size_t index = 0U;
         index < sizeof(*record) / sizeof(words[0]); ++index) {
        if (words[index] != UINT32_MAX) {
            return 0;
        }
    }
    return 1;
}

static int record_is_valid(const dima_boot_flash_record_t *record)
{
    if (record->magic != DIMA_BOOT_FLASH_RECORD_MAGIC ||
        record->version != DIMA_BOOT_FLASH_RECORD_VERSION ||
        record->size != sizeof(*record) ||
        record->commit != DIMA_BOOT_FLASH_RECORD_COMMIT ||
        !capture_is_valid(&record->diagnostics)) {
        return 0;
    }
    return record->crc32 ==
        crc32_bytes(record, offsetof(dima_boot_flash_record_t, crc32));
}

static void copy_capture(dima_boot_diagnostics_t *destination,
                         const volatile dima_boot_diagnostics_t *source)
{
    uint32_t *output = (uint32_t *)(void *)destination;
    const volatile uint32_t *input =
        (const volatile uint32_t *)(const volatile void *)source;
    for (size_t index = 0U;
         index < sizeof(*destination) / sizeof(output[0]); ++index) {
        output[index] = input[index];
    }
}

static int diagnostics_equal(const dima_boot_diagnostics_t *left,
                             const dima_boot_diagnostics_t *right)
{
    const uint32_t *left_words = (const uint32_t *)(const void *)left;
    const uint32_t *right_words = (const uint32_t *)(const void *)right;
    for (size_t index = 0U;
         index < sizeof(*left) / sizeof(left_words[0]); ++index) {
        if (left_words[index] != right_words[index]) {
            return 0;
        }
    }
    return 1;
}

static void fill_erased(void *destination, size_t length)
{
    uint32_t *words = (uint32_t *)destination;
    for (size_t index = 0U; index < length / sizeof(words[0]); ++index) {
        words[index] = UINT32_MAX;
    }
}

/* Both the application and the diagnostic sector execute from Flash Bank 1.
 * Keep the complete program operation in DTCM so no Bank 1 instruction fetch
 * is required while a Flash word is being committed. */
__attribute__((noinline, section(".dima_ramfunc")))
static int program_record_from_ram(
    uint32_t address, const uint32_t *source_words)
{
    if ((FLASH->SR1 & FLASH_BUSY_MASK) != 0U) {
        return 0;
    }

    if ((FLASH->CR1 & FLASH_CR_LOCK) != 0U) {
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
        if ((FLASH->CR1 & FLASH_CR_LOCK) != 0U) {
            return 0;
        }
    }

    FLASH->CCR1 = FLASH_CLEAR_MASK;
    int success = 1;
    for (uint32_t flash_word = 0U;
         flash_word < FLASH_PROGRAM_WORD_COUNT; ++flash_word) {
        if ((FLASH->SR1 & FLASH_BUSY_MASK) != 0U) {
            success = 0;
            break;
        }

        FLASH->CR1 |= FLASH_CR_PG;
        __ISB();
        __DSB();

        volatile uint32_t *destination =
            (volatile uint32_t *)(uintptr_t)
                (address + flash_word * H743_FLASH_WRITE_SIZE);
        const uint32_t *source =
            source_words + flash_word * FLASH_WORD_U32_COUNT;
        for (uint32_t word = 0U; word < FLASH_WORD_U32_COUNT; ++word) {
            destination[word] = source[word];
        }

        __ISB();
        __DSB();

        uint32_t polls_remaining = FLASH_PROGRAM_POLL_LIMIT;
        while ((FLASH->SR1 & FLASH_BUSY_MASK) != 0U &&
               polls_remaining != 0U) {
            --polls_remaining;
        }

        const uint32_t status = FLASH->SR1;
        FLASH->CR1 &= ~FLASH_CR_PG;
        if (polls_remaining == 0U ||
            (status & FLASH_FLAG_ALL_ERRORS_BANK1) != 0U) {
            success = 0;
        }
        FLASH->CCR1 = status & FLASH_CLEAR_MASK;
        if (!success) {
            break;
        }
    }

    FLASH->CR1 |= FLASH_CR_LOCK;
    __DSB();
    __ISB();
    return success;
}

static int program_record(uint32_t address,
                          const dima_boot_flash_record_t *record)
{
    if ((FLASH->SR1 & FLASH_BUSY_MASK) != 0U) {
        return 0;
    }

    const uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    const int programmed = program_record_from_ram(
        address, (const uint32_t *)(const void *)record);
    __DSB();
    __ISB();
    if (saved_primask == 0U) {
        __enable_irq();
    }
    if (!programmed) {
        return 0;
    }

    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)(uintptr_t)address, (int32_t)sizeof(*record));
    __DSB();
    __ISB();
    return record_is_valid(
        (const dima_boot_flash_record_t *)(uintptr_t)address) &&
        diagnostics_equal(
            &((const dima_boot_flash_record_t *)(uintptr_t)address)->diagnostics,
            &record->diagnostics);
}

static void clear_capture_if_requested(
    volatile dima_boot_diagnostics_t *capture, int clear_capture)
{
    if (!clear_capture) {
        return;
    }
    capture->capture_valid = 0U;
    SCB_CleanDCache_by_Addr(
        (uint32_t *)(void *)capture, (int32_t)sizeof(*capture));
    __DSB();
}

static dima_boot_diagnostics_store_result_t finish_store(
    dima_boot_diagnostics_store_result_t result)
{
    __DMB();
    store_state = STORE_STATE_ENABLED;
    __DMB();
    return result;
}

void dima_boot_diagnostics_store_enable(void)
{
    __DMB();
    store_state = STORE_STATE_ENABLED;
    __DMB();
}

int dima_boot_diagnostics_capture_pending(void)
{
    return capture_is_valid(capture_record());
}

dima_boot_diagnostics_store_result_t
dima_boot_diagnostics_store_pending(int clear_capture)
{
    volatile dima_boot_diagnostics_t *capture =
        (volatile dima_boot_diagnostics_t *)(uintptr_t)
            DIMA_BOOT_DIAGNOSTICS_ADDRESS;
    if (!capture_is_valid(capture)) {
        return DIMA_BOOT_DIAGNOSTICS_STORE_NONE;
    }
    if (store_state == STORE_STATE_BUSY) {
        return DIMA_BOOT_DIAGNOSTICS_STORE_BUSY;
    }
    if (store_state != STORE_STATE_ENABLED) {
        return DIMA_BOOT_DIAGNOSTICS_STORE_NOT_READY;
    }
    if ((FLASH->SR1 & FLASH_BUSY_MASK) != 0U) {
        return DIMA_BOOT_DIAGNOSTICS_STORE_BUSY;
    }
    store_state = STORE_STATE_BUSY;
    __DMB();

    fill_erased(&staging_record, sizeof(staging_record));
    copy_capture(&staging_record.diagnostics, capture);
    if (!capture_is_valid(&staging_record.diagnostics)) {
        return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_ERROR);
    }

    uint32_t next_sequence = 1U;
    uint32_t free_index = RECORD_COUNT;
    for (uint32_t index = 0U; index < RECORD_COUNT; ++index) {
        const dima_boot_flash_record_t *record = flash_record(index);
        if (record_is_valid(record)) {
            if (diagnostics_equal(&record->diagnostics,
                                  &staging_record.diagnostics)) {
                clear_capture_if_requested(capture, clear_capture);
                return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_EXISTING);
            }
            if (record->sequence >= next_sequence) {
                next_sequence = record->sequence + 1U;
                if (next_sequence == 0U) {
                    next_sequence = 1U;
                }
            }
        } else if (free_index == RECORD_COUNT && record_is_erased(record)) {
            free_index = index;
        }
    }

    if (free_index == RECORD_COUNT) {
        return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_FULL);
    }

    staging_record.magic = DIMA_BOOT_FLASH_RECORD_MAGIC;
    staging_record.version = DIMA_BOOT_FLASH_RECORD_VERSION;
    staging_record.size = sizeof(staging_record);
    staging_record.sequence = next_sequence;
    staging_record.crc32 = crc32_bytes(
        &staging_record, offsetof(dima_boot_flash_record_t, crc32));
    staging_record.commit = DIMA_BOOT_FLASH_RECORD_COMMIT;

    const uint32_t address = H743_BOOT_DIAGNOSTICS_BASE +
                             free_index * DIMA_BOOT_FLASH_RECORD_SIZE;
    if (!record_is_erased((const dima_boot_flash_record_t *)(uintptr_t)address) ||
        !program_record(address, &staging_record)) {
        return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_ERROR);
    }

    clear_capture_if_requested(capture, clear_capture);
    return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_OK);
}

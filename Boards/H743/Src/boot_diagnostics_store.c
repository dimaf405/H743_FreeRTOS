#include "boot_diagnostics_store.h"

#include <stddef.h>
#include <stdint.h>

#include "boot_diagnostics.h"
#include "boot_layout.h"
#include "platform/stm32h7/flash/flash_bank1.h"
#include "stm32h7xx_hal.h"

#define RECORD_COUNT \
    (H743_BOOT_DIAGNOSTICS_SIZE / DIMA_BOOT_FLASH_RECORD_SIZE)
#define STORE_STATE_ENABLED UINT32_C(0x53544F52)
#define STORE_STATE_BUSY    UINT32_C(0x42555359)
#define FLASH_BUSY_MASK \
    (FLASH_FLAG_QW_BANK1 | FLASH_FLAG_BSY_BANK1 | FLASH_FLAG_WBNE_BANK1)

typedef union {
    dima_boot_flash_record_v1_t v1;
    dima_boot_flash_record_t v2;
    uint32_t words[DIMA_BOOT_FLASH_RECORD_SIZE / sizeof(uint32_t)];
    uint8_t bytes[DIMA_BOOT_FLASH_RECORD_SIZE];
} dima_boot_flash_record_any_t;

static dima_boot_flash_record_any_t staging_record
    __attribute__((aligned(H743_FLASH_WRITE_SIZE)));
static volatile uint32_t store_state;

_Static_assert(sizeof(dima_boot_diagnostics_v1_t) == 192U,
               "Boot diagnostics v1 layout is a persistent ABI");
_Static_assert(sizeof(dima_boot_diagnostics_t) == 208U,
               "Boot diagnostics v2 layout is a persistent ABI");
_Static_assert(sizeof(dima_boot_flash_record_v1_t) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE,
               "Boot diagnostics v1 Flash record must occupy one slot");
_Static_assert(sizeof(dima_boot_flash_record_t) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE,
               "Boot diagnostics v2 Flash record must occupy one slot");
_Static_assert(sizeof(dima_boot_flash_record_any_t) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE,
               "Boot diagnostics union must occupy one slot");
_Static_assert(offsetof(dima_boot_flash_record_v1_t, crc32) == 208U,
               "Boot diagnostics v1 CRC offset is a persistent ABI");
_Static_assert(offsetof(dima_boot_flash_record_t, crc32) == 224U,
               "Boot diagnostics v2 CRC offset is a persistent ABI");
_Static_assert(offsetof(dima_boot_flash_record_v1_t, commit) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE - sizeof(uint32_t),
               "V1 commit marker must be in the final Flash word");
_Static_assert(offsetof(dima_boot_flash_record_t, commit) ==
                   DIMA_BOOT_FLASH_RECORD_SIZE - sizeof(uint32_t),
               "V2 commit marker must be in the final Flash word");
_Static_assert((H743_BOOT_DIAGNOSTICS_SIZE %
                DIMA_BOOT_FLASH_RECORD_SIZE) == 0U,
               "Boot diagnostics sector must contain whole records");
_Static_assert((DIMA_BOOT_FLASH_RECORD_SIZE %
                H743_FLASH_WRITE_SIZE) == 0U,
               "Boot diagnostics records must contain whole Flash words");

static const volatile void *capture_record(void)
{
    return (const volatile void *)(uintptr_t)DIMA_BOOT_DIAGNOSTICS_ADDRESS;
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

static uint32_t diagnostics_size_for_version(uint32_t version)
{
    if (version == DIMA_BOOT_DIAGNOSTICS_VERSION_V1) {
        return sizeof(dima_boot_diagnostics_v1_t);
    }
    if (version == DIMA_BOOT_DIAGNOSTICS_VERSION) {
        return sizeof(dima_boot_diagnostics_t);
    }
    return 0U;
}

static uint32_t record_crc_offset(uint32_t version)
{
    if (version == DIMA_BOOT_FLASH_RECORD_VERSION_V1) {
        return offsetof(dima_boot_flash_record_v1_t, crc32);
    }
    if (version == DIMA_BOOT_FLASH_RECORD_VERSION) {
        return offsetof(dima_boot_flash_record_t, crc32);
    }
    return 0U;
}

static int capture_is_valid(const volatile void *capture)
{
    const volatile uint32_t *words =
        (const volatile uint32_t *)capture;
    const uint32_t expected_size = diagnostics_size_for_version(words[1]);
    return words[0] == DIMA_BOOT_DIAGNOSTICS_MAGIC &&
           expected_size != 0U && words[2] == expected_size &&
           words[8] == DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID &&
           words[7] != DIMA_BOOT_FAILURE_NONE;
}

static const dima_boot_flash_record_any_t *flash_record(uint32_t index)
{
    return (const dima_boot_flash_record_any_t *)(uintptr_t)
        (H743_BOOT_DIAGNOSTICS_BASE +
         index * DIMA_BOOT_FLASH_RECORD_SIZE);
}

static int record_is_erased(const dima_boot_flash_record_any_t *record)
{
    for (size_t index = 0U;
         index < sizeof(record->words) / sizeof(record->words[0]); ++index) {
        if (record->words[index] != UINT32_MAX) {
            return 0;
        }
    }
    return 1;
}

static const volatile void *record_diagnostics(
    const dima_boot_flash_record_any_t *record)
{
    return (const volatile void *)&record->bytes[4U * sizeof(uint32_t)];
}

static int record_is_valid(const dima_boot_flash_record_any_t *record)
{
    const uint32_t version = record->words[1];
    const uint32_t crc_offset = record_crc_offset(version);
    if (record->words[0] != DIMA_BOOT_FLASH_RECORD_MAGIC ||
        crc_offset == 0U || record->words[2] != sizeof(*record) ||
        record->words[(DIMA_BOOT_FLASH_RECORD_SIZE / sizeof(uint32_t)) - 1U] !=
            DIMA_BOOT_FLASH_RECORD_COMMIT ||
        !capture_is_valid(record_diagnostics(record))) {
        return 0;
    }

    const volatile uint32_t *diagnostics =
        (const volatile uint32_t *)record_diagnostics(record);
    const uint32_t expected_diagnostics_version =
        version == DIMA_BOOT_FLASH_RECORD_VERSION_V1
            ? DIMA_BOOT_DIAGNOSTICS_VERSION_V1
            : DIMA_BOOT_DIAGNOSTICS_VERSION;
    return diagnostics[1] == expected_diagnostics_version &&
           record->words[crc_offset / sizeof(uint32_t)] ==
               crc32_bytes(record, crc_offset);
}

static int diagnostics_equal(const volatile void *left,
                             const volatile void *right)
{
    if (!capture_is_valid(left) || !capture_is_valid(right)) {
        return 0;
    }
    const volatile uint32_t *left_words =
        (const volatile uint32_t *)left;
    const volatile uint32_t *right_words =
        (const volatile uint32_t *)right;
    if (left_words[1] != right_words[1] ||
        left_words[2] != right_words[2]) {
        return 0;
    }
    for (size_t index = 0U;
         index < left_words[2] / sizeof(left_words[0]); ++index) {
        if (left_words[index] != right_words[index]) {
            return 0;
        }
    }
    return 1;
}

static void copy_capture(dima_boot_flash_record_any_t *destination,
                         const volatile void *source)
{
    const volatile uint32_t *input =
        (const volatile uint32_t *)source;
    uint32_t *output = &destination->words[4];
    const size_t count = input[2] / sizeof(input[0]);
    for (size_t index = 0U; index < count; ++index) {
        output[index] = input[index];
    }
}

static void fill_erased(void *destination, size_t length)
{
    uint32_t *words = (uint32_t *)destination;
    for (size_t index = 0U; index < length / sizeof(words[0]); ++index) {
        words[index] = UINT32_MAX;
    }
}

static int program_record(uint32_t address,
                          const dima_boot_flash_record_any_t *record)
{
    if (!dima_stm32_flash_bank1_program(address, record, sizeof(*record))) {
        return 0;
    }
    const dima_boot_flash_record_any_t *stored =
        (const dima_boot_flash_record_any_t *)(uintptr_t)address;
    return record_is_valid(stored) &&
           diagnostics_equal(record_diagnostics(stored),
                             record_diagnostics(record));
}

static void clear_capture_if_requested(volatile void *capture,
                                       int clear_capture)
{
    if (!clear_capture) {
        return;
    }
    volatile uint32_t *words = (volatile uint32_t *)capture;
    words[8] = 0U;
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

static void seed_application_bridge_record(
    volatile dima_boot_diagnostics_t *record)
{
    const uint32_t reset_flags = RCC->RSR;
    volatile uint32_t *words = (volatile uint32_t *)(void *)record;
    for (size_t index = 0U;
         index < sizeof(*record) / sizeof(words[0]); ++index) {
        words[index] = 0U;
    }

    /* MCUboot runs before the Application has initialized the NOLOAD D3
     * record after a cold boot or ROM-DFU recovery. Seed the persistent ABI
     * here so the reset-only handoff cannot depend on code that has not been
     * allowed to start yet. */
    record->version = DIMA_BOOT_DIAGNOSTICS_VERSION;
    record->size = sizeof(*record);
    record->reset_flags = reset_flags;
    record->stage = DIMA_BOOT_STAGE_SYSTEM_INIT;
    record->detail = DIMA_BOOT_DETAIL_APPLICATION_BRIDGE;
    record->failure_kind = DIMA_BOOT_FAILURE_NONE;
    record->capture_valid = 0U;
    record->abfsr = SCB->ABFSR;
    record->scb_ccr = SCB->CCR;
    record->mpu_ctrl = MPU->CTRL;
    __DMB();
    record->magic = DIMA_BOOT_DIAGNOSTICS_MAGIC;
    __DSB();
    __ISB();
}

int dima_boot_diagnostics_mark_application_bridge(void)
{
    volatile dima_boot_diagnostics_t *record =
        (volatile dima_boot_diagnostics_t *)(uintptr_t)
            DIMA_BOOT_DIAGNOSTICS_ADDRESS;
    const uint32_t expected_size =
        diagnostics_size_for_version(record->version);
    if (record->magic != DIMA_BOOT_DIAGNOSTICS_MAGIC ||
        expected_size == 0U || record->size != expected_size) {
        seed_application_bridge_record(record);
    } else {
        record->detail = DIMA_BOOT_DETAIL_APPLICATION_BRIDGE;
        __DMB();
        __DSB();
    }

    return record->magic == DIMA_BOOT_DIAGNOSTICS_MAGIC &&
           diagnostics_size_for_version(record->version) == record->size &&
           record->detail == DIMA_BOOT_DETAIL_APPLICATION_BRIDGE;
}

dima_boot_diagnostics_store_result_t
dima_boot_diagnostics_store_pending(int clear_capture)
{
    volatile void *capture =
        (volatile void *)(uintptr_t)DIMA_BOOT_DIAGNOSTICS_ADDRESS;
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
    copy_capture(&staging_record, capture);
    if (!capture_is_valid(record_diagnostics(&staging_record))) {
        return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_ERROR);
    }

    uint32_t next_sequence = 1U;
    uint32_t free_index = RECORD_COUNT;
    for (uint32_t index = 0U; index < RECORD_COUNT; ++index) {
        const dima_boot_flash_record_any_t *record = flash_record(index);
        if (record_is_valid(record)) {
            if (diagnostics_equal(record_diagnostics(record),
                                  record_diagnostics(&staging_record))) {
                clear_capture_if_requested(capture, clear_capture);
                return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_EXISTING);
            }
            if (record->words[3] >= next_sequence) {
                next_sequence = record->words[3] + 1U;
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

    const volatile uint32_t *diagnostics =
        (const volatile uint32_t *)record_diagnostics(&staging_record);
    const uint32_t record_version =
        diagnostics[1] == DIMA_BOOT_DIAGNOSTICS_VERSION_V1
            ? DIMA_BOOT_FLASH_RECORD_VERSION_V1
            : DIMA_BOOT_FLASH_RECORD_VERSION;
    const uint32_t crc_offset = record_crc_offset(record_version);
    staging_record.words[0] = DIMA_BOOT_FLASH_RECORD_MAGIC;
    staging_record.words[1] = record_version;
    staging_record.words[2] = sizeof(staging_record);
    staging_record.words[3] = next_sequence;
    staging_record.words[crc_offset / sizeof(uint32_t)] =
        crc32_bytes(&staging_record, crc_offset);
    staging_record.words[
        (DIMA_BOOT_FLASH_RECORD_SIZE / sizeof(uint32_t)) - 1U] =
        DIMA_BOOT_FLASH_RECORD_COMMIT;

    const uint32_t address = H743_BOOT_DIAGNOSTICS_BASE +
                             free_index * DIMA_BOOT_FLASH_RECORD_SIZE;
    if (!record_is_erased(
            (const dima_boot_flash_record_any_t *)(uintptr_t)address) ||
        !program_record(address, &staging_record)) {
        return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_ERROR);
    }

    clear_capture_if_requested(capture, clear_capture);
    return finish_store(DIMA_BOOT_DIAGNOSTICS_STORE_OK);
}

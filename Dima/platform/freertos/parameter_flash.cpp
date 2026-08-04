#include "parameter_flash.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include "boot_layout.h"
#include "safety/ArmingFlashInterlock.h"
#include "stm32h7xx_hal.h"

extern "C" {
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
}

namespace dima::platform {
namespace {

constexpr uint32_t kHeaderMagic = 0x444D5048U; // "DMPH"
constexpr uint32_t kCommitMagic = 0x444D5043U; // "DMPC"
constexpr uint32_t kFormatVersion = 1U;
constexpr size_t kFlashWordBytes = H743_FLASH_WRITE_SIZE;
constexpr size_t kStorageBytes = H743_STORAGE_SIZE;
constexpr uintptr_t kStorageBegin = H743_STORAGE_BASE;
constexpr uintptr_t kStorageEnd = H743_STORAGE_BASE + H743_STORAGE_SIZE;

struct alignas(kFlashWordBytes) RecordHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t payload_length;
    uint32_t payload_crc32;
    uint32_t record_length;
    uint32_t header_crc32;
    uint32_t reserved;
};

struct alignas(kFlashWordBytes) CommitMarker {
    uint32_t magic;
    uint32_t sequence;
    uint32_t payload_crc32;
    uint32_t record_length;
    uint32_t marker_crc32;
    uint32_t reserved[3];
};

static_assert(sizeof(RecordHeader) == kFlashWordBytes);
static_assert(sizeof(CommitMarker) == kFlashWordBytes);
static_assert((kStorageBegin % kFlashWordBytes) == 0U);
static_assert((kStorageBytes % kFlashWordBytes) == 0U);
constexpr size_t kMaxPayloadBytes = kStorageBytes - sizeof(RecordHeader)
                                    - sizeof(CommitMarker);

StaticSemaphore_t g_mutex_storage{};
SemaphoreHandle_t g_mutex{nullptr};
ParameterFlashStatus g_status{};
size_t g_append_offset{0U};
size_t g_latest_offset{0U};
bool g_has_snapshot{false};
bool g_initialized{false};
alignas(kFlashWordBytes) uint8_t g_flashword[kFlashWordBytes];
constexpr size_t kFaultBitmapWords = (kStorageBytes / kFlashWordBytes + 31U) / 32U;
uint32_t g_invalid_faults[kFaultBitmapWords]{};
uint32_t g_crc_faults[kFaultBitmapWords]{};
volatile bool g_safe_read_active{false};
volatile bool g_safe_read_faulted{false};
volatile uintptr_t g_safe_read_resume{0U};
volatile uintptr_t g_safe_read_address{0U};

class FlashLock {
public:
    FlashLock() noexcept
        : locked_(g_mutex != nullptr
                  && xSemaphoreTakeRecursive(g_mutex, portMAX_DELAY) == pdTRUE)
    {
    }

    ~FlashLock()
    {
        if (locked_) {
            (void)xSemaphoreGiveRecursive(g_mutex);
        }
    }

    explicit operator bool() const noexcept { return locked_; }

private:
    bool locked_;
};

class FlashWriteGuard {
public:
    FlashWriteGuard() noexcept
        : acquired_(dima_arming_flash_begin() == DIMA_FLASH_BEGIN_ACQUIRED)
    {
    }

    ~FlashWriteGuard()
    {
        if (acquired_) {
            dima_arming_flash_end();
        }
    }

    explicit operator bool() const noexcept { return acquired_; }

private:
    bool acquired_;
};

bool in_isr() noexcept
{
    return xPortIsInsideInterrupt() != pdFALSE;
}

constexpr size_t align_flashword(size_t value) noexcept
{
    return (value + kFlashWordBytes - 1U) & ~(kFlashWordBytes - 1U);
}

uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length) noexcept
{
    while (length-- > 0U) {
        crc ^= *data++;
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return crc;
}

uint32_t crc32(const void *data, size_t length) noexcept
{
    if (length == 0U) {
        return 0U;
    }
    return crc32_update(0xFFFFFFFFU, static_cast<const uint8_t *>(data), length)
           ^ 0xFFFFFFFFU;
}

__attribute__((noinline, optimize("O0")))
bool safe_read_u32(uintptr_t address, uint32_t &value) noexcept
{
    constexpr uint32_t kEccFlags = FLASH_FLAG_SNECCERR_BANK2
                                     | FLASH_FLAG_DBECCERR_BANK2;
    __HAL_FLASH_CLEAR_FLAG_BANK2(kEccFlags);
    g_safe_read_address = address;
    g_safe_read_faulted = false;
    g_safe_read_resume = reinterpret_cast<uintptr_t>(&&read_complete) & ~uintptr_t{1U};
    g_safe_read_active = true;
    __DSB();
    value = *reinterpret_cast<volatile const uint32_t *>(address);
read_complete:
    __DSB();
    g_safe_read_active = false;
    if (g_safe_read_faulted) {
        return false;
    }

    if (__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2)
        || __HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_DBECCERR_BANK2)) {
        __HAL_FLASH_CLEAR_FLAG_BANK2(kEccFlags);
        return false;
    }
    return true;
}

bool safe_read_bytes(uintptr_t address, void *destination, size_t length) noexcept
{
    auto *out = static_cast<uint8_t *>(destination);
    size_t offset = 0U;
    while (offset < length) {
        uint32_t word{};
        if (!safe_read_u32(address + offset, word)) {
            return false;
        }
        const size_t copy_length = std::min(sizeof(word), length - offset);
        std::memcpy(out + offset, &word, copy_length);
        offset += sizeof(word);
    }
    return true;
}

bool flashword_erased(uintptr_t address, bool &readable) noexcept
{
    alignas(4) uint8_t data[kFlashWordBytes]{};
    readable = safe_read_bytes(address, data, sizeof(data));
    if (!readable) {
        return false;
    }
    for (uint8_t byte : data) {
        if (byte != 0xFFU) {
            return false;
        }
    }
    return true;
}

bool flash_crc32(uintptr_t address, size_t length, uint32_t &result) noexcept
{
    uint32_t crc = 0xFFFFFFFFU;
    alignas(4) uint8_t data[kFlashWordBytes]{};
    size_t offset = 0U;
    while (offset < length) {
        const size_t chunk = std::min(sizeof(data), length - offset);
        if (!safe_read_bytes(address + offset, data, chunk)) {
            return false;
        }
        crc = crc32_update(crc, data, chunk);
        offset += chunk;
    }
    result = length == 0U ? 0U : crc ^ 0xFFFFFFFFU;
    return true;
}

void invalidate_flash(uintptr_t address, size_t length) noexcept
{
    // MCUboot disables D-cache before handing control to the application and
    // the current application does not enable it again.  Cache maintenance is
    // unnecessary in that state and repeatedly writing DCIMVAC for the whole
    // parameter sector has been observed to raise an imprecise BusFault.
    if (length == 0U || (SCB->CCR & SCB_CCR_DC_Msk) == 0U) {
        return;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<void *>(address),
                                static_cast<int32_t>(length));
    __DSB();
    __ISB();
}


bool mark_fault_once(uint32_t *bitmap, size_t offset) noexcept
{
    const size_t index = offset / kFlashWordBytes;
    const size_t word = index / 32U;
    const uint32_t mask = 1UL << (index % 32U);
    if ((bitmap[word] & mask) != 0U) {
        return false;
    }
    bitmap[word] |= mask;
    return true;
}

bool header_valid(const RecordHeader &header, size_t offset) noexcept
{
    if (header.magic != kHeaderMagic || header.version != kFormatVersion
        || header.reserved != 0xFFFFFFFFU) {
        return false;
    }

    if (header.payload_length > kMaxPayloadBytes) {
        return false;
    }

    const size_t padded_payload = align_flashword(header.payload_length);
    const size_t expected_length = sizeof(RecordHeader) + padded_payload
                                   + sizeof(CommitMarker);
    if (header.record_length != expected_length
        || expected_length > kStorageBytes - offset) {
        return false;
    }

    return header.header_crc32 == crc32(&header, offsetof(RecordHeader, header_crc32));
}

bool marker_valid(const RecordHeader &header, const CommitMarker &marker) noexcept
{
    return marker.magic == kCommitMagic
           && marker.sequence == header.sequence
           && marker.payload_crc32 == header.payload_crc32
           && marker.record_length == header.record_length
           && marker.marker_crc32
                  == crc32(&marker, offsetof(CommitMarker, marker_crc32));
}

int validate_record_locked(size_t offset, RecordHeader &header) noexcept
{
    if (offset > kStorageBytes - sizeof(RecordHeader)
        || !safe_read_bytes(kStorageBegin + offset, &header, sizeof(header))
        || !header_valid(header, offset)) {
        if (offset < kStorageBytes && mark_fault_once(g_invalid_faults, offset)) {
            ++g_status.invalid_records;
        }
        return -EIO;
    }

    const size_t marker_offset = offset + header.record_length
                                 - sizeof(CommitMarker);
    CommitMarker marker{};
    if (!safe_read_bytes(kStorageBegin + marker_offset, &marker, sizeof(marker))
        || !marker_valid(header, marker)) {
        if (mark_fault_once(g_invalid_faults, offset)) {
            ++g_status.invalid_records;
        }
        return -EIO;
    }

    uint32_t payload_crc{};
    if (!flash_crc32(kStorageBegin + offset + sizeof(RecordHeader),
                     header.payload_length, payload_crc)
        || payload_crc != header.payload_crc32) {
        if (mark_fault_once(g_crc_faults, offset)) {
            ++g_status.crc_failures;
        }
        return -EILSEQ;
    }
    return 0;
}

int scan_locked() noexcept
{
    invalidate_flash(kStorageBegin, kStorageBytes);

    g_status.valid_sequence = 0U;
    g_status.payload_bytes = 0U;
    g_status.used_bytes = 0U;
    g_status.free_bytes = kStorageBytes;
    g_append_offset = 0U;
    g_latest_offset = 0U;
    g_has_snapshot = false;

    size_t high_water = 0U;
    for (size_t offset = 0U; offset + kFlashWordBytes <= kStorageBytes;
         offset += kFlashWordBytes) {
        const uintptr_t word_address = kStorageBegin + offset;
        bool readable{};
        const bool erased = flashword_erased(word_address, readable);
        if (!readable) {
            // 掉电可能留下 ECC 不可读 Flashword；将其视为占用并继续寻找旧有效快照。
            high_water = offset + kFlashWordBytes;
            if (mark_fault_once(g_invalid_faults, offset)) {
                ++g_status.invalid_records;
            }
            continue;
        }
        if (!erased) {
            high_water = offset + kFlashWordBytes;
        }

        RecordHeader header{};
        if (!safe_read_bytes(word_address, &header, sizeof(header))) {
            if (mark_fault_once(g_invalid_faults, offset)) {
                ++g_status.invalid_records;
            }
            continue;
        }
        if (header.magic != kHeaderMagic) {
            continue;
        }
        if (!header_valid(header, offset)) {
            if (mark_fault_once(g_invalid_faults, offset)) {
                ++g_status.invalid_records;
            }
            continue;
        }

        high_water = std::max(high_water,
                              offset + static_cast<size_t>(header.record_length));
        const size_t marker_offset = offset + header.record_length
                                     - sizeof(CommitMarker);
        CommitMarker marker{};
        if (!safe_read_bytes(kStorageBegin + marker_offset, &marker, sizeof(marker))
            || !marker_valid(header, marker)) {
            if (mark_fault_once(g_invalid_faults, offset)) {
                ++g_status.invalid_records;
            }
            continue;
        }

        uint32_t payload_crc{};
        if (!flash_crc32(kStorageBegin + offset + sizeof(RecordHeader),
                         header.payload_length, payload_crc)
            || payload_crc != header.payload_crc32) {
            if (mark_fault_once(g_crc_faults, offset)) {
                ++g_status.crc_failures;
            }
            continue;
        }

        if (!g_has_snapshot || header.sequence > g_status.valid_sequence) {
            g_has_snapshot = true;
            g_latest_offset = offset;
            g_status.valid_sequence = header.sequence;
            g_status.payload_bytes = header.payload_length;
        }
    }

    g_append_offset = std::min(align_flashword(high_water), kStorageBytes);
    g_status.used_bytes = g_append_offset;
    g_status.free_bytes = kStorageBytes - g_append_offset;
    return g_has_snapshot ? 0 : -ENOENT;
}

bool program_flashword(uintptr_t address, const void *source) noexcept
{
    if (source != g_flashword) {
        std::memcpy(g_flashword, source, sizeof(g_flashword));
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address,
                          static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_flashword)))
        != HAL_OK) {
        return false;
    }
    invalidate_flash(address, kFlashWordBytes);
    alignas(4) uint8_t verify[kFlashWordBytes]{};
    return safe_read_bytes(address, verify, sizeof(verify))
           && std::memcmp(verify, g_flashword, kFlashWordBytes) == 0;
}

int append_locked(const void *payload, size_t payload_size,
                  uint32_t *sequence_out) noexcept
{
    if (payload == nullptr && payload_size != 0U) {
        return -EINVAL;
    }
    // 必须先限制原始长度，再执行向上对齐，避免 32 位 size_t 加法回绕。
    if (payload_size > kMaxPayloadBytes) {
        return -EFBIG;
    }

    const size_t padded_payload = align_flashword(payload_size);
    const size_t record_length = sizeof(RecordHeader) + padded_payload
                                 + sizeof(CommitMarker);
    if (record_length > kStorageBytes - g_append_offset) {
        ++g_status.enospc_failures;
        return -ENOSPC;
    }

    const uint32_t sequence = g_has_snapshot ? g_status.valid_sequence + 1U : 1U;
    RecordHeader header{};
    std::memset(&header, 0xFF, sizeof(header));
    header.magic = kHeaderMagic;
    header.version = kFormatVersion;
    header.sequence = sequence;
    header.payload_length = static_cast<uint32_t>(payload_size);
    header.payload_crc32 = crc32(payload, payload_size);
    header.record_length = static_cast<uint32_t>(record_length);
    header.header_crc32 = crc32(&header, offsetof(RecordHeader, header_crc32));

    CommitMarker marker{};
    std::memset(&marker, 0xFF, sizeof(marker));
    marker.magic = kCommitMagic;
    marker.sequence = sequence;
    marker.payload_crc32 = header.payload_crc32;
    marker.record_length = header.record_length;
    marker.marker_crc32 = crc32(&marker, offsetof(CommitMarker, marker_crc32));

    const uintptr_t record_address = kStorageBegin + g_append_offset;
    if (HAL_FLASH_Unlock() != HAL_OK) {
        ++g_status.write_failures;
        return -EIO;
    }
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);

    bool ok = program_flashword(record_address, &header);
    const auto *payload_bytes = static_cast<const uint8_t *>(payload);
    for (size_t offset = 0U; ok && offset < padded_payload;
         offset += kFlashWordBytes) {
        std::memset(g_flashword, 0xFF, sizeof(g_flashword));
        const size_t copy_length = std::min(kFlashWordBytes, payload_size - offset);
        if (copy_length > 0U) {
            std::memcpy(g_flashword, payload_bytes + offset, copy_length);
        }
        ok = program_flashword(record_address + sizeof(RecordHeader) + offset,
                               g_flashword);
    }

    // Commit Marker独占最后一个Flashword，必须在Header和Payload回读通过后写入。
    if (ok) {
        invalidate_flash(record_address, record_length - sizeof(CommitMarker));
        uint32_t stored_crc{};
        ok = flash_crc32(record_address + sizeof(RecordHeader), payload_size, stored_crc)
             && stored_crc == header.payload_crc32;
    }
    if (ok) {
        ok = program_flashword(record_address + record_length
                                   - sizeof(CommitMarker),
                               &marker);
    }

    (void)HAL_FLASH_Lock();
    if (!ok) {
        ++g_status.write_failures;
        (void)scan_locked();
        return -EIO;
    }

    g_has_snapshot = true;
    g_latest_offset = g_append_offset;
    g_append_offset += record_length;
    g_status.valid_sequence = sequence;
    g_status.payload_bytes = payload_size;
    g_status.used_bytes = g_append_offset;
    g_status.free_bytes = kStorageBytes - g_append_offset;
    if (sequence_out != nullptr) {
        *sequence_out = sequence;
    }
    return 0;
}

int erase_locked() noexcept
{
    if (HAL_FLASH_Unlock() != HAL_OK) {
        ++g_status.erase_failures;
        return -EIO;
    }
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0xFFFFFFFFU;
    const HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();
    invalidate_flash(kStorageBegin, kStorageBytes);

    bool erased = result == HAL_OK && sector_error == 0xFFFFFFFFU;
    for (uintptr_t address = kStorageBegin; erased && address < kStorageEnd;
         address += kFlashWordBytes) {
        bool readable{};
        erased = flashword_erased(address, readable) && readable;
    }
    if (!erased) {
        ++g_status.erase_failures;
        return -EIO;
    }

    std::memset(g_invalid_faults, 0, sizeof(g_invalid_faults));
    std::memset(g_crc_faults, 0, sizeof(g_crc_faults));
    (void)scan_locked();
    return 0;
}

} // namespace

extern "C" int dima_flash_operation_lock(void)
{
    if (in_isr() || !g_initialized || g_mutex == nullptr) {
        return 0;
    }
    return xSemaphoreTakeRecursive(g_mutex, 0U) == pdTRUE ? 1 : 0;
}

extern "C" void dima_flash_operation_unlock(void)
{
    if (!in_isr() && g_initialized && g_mutex != nullptr) {
        (void)xSemaphoreGiveRecursive(g_mutex);
    }
}

extern "C" bool dima_parameter_flash_recover_busfault(uint32_t *stacked_frame) noexcept
{
    if (!g_safe_read_active || stacked_frame == nullptr
        || g_safe_read_address < kStorageBegin || g_safe_read_address >= kStorageEnd
        || (FLASH->SR2 & FLASH_SR_DBECCERR) == 0U) {
        return false;
    }

    g_safe_read_active = false;
    g_safe_read_faulted = true;
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2
                                 | FLASH_FLAG_DBECCERR_BANK2);
    SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;
    stacked_frame[6] = static_cast<uint32_t>(g_safe_read_resume);
    __DSB();
    __ISB();
    return true;
}

bool parameter_flash_init() noexcept
{
    if (in_isr()) {
        return false;
    }
    if (!g_initialized) {
        g_mutex = xSemaphoreCreateRecursiveMutexStatic(&g_mutex_storage);
        g_initialized = g_mutex != nullptr;
    }
    if (!g_initialized) {
        return false;
    }

    // 参数区安全读取依赖可恢复的精确 BusFault，避免 ECC 双错升级为 HardFault。
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    __DSB();
    __ISB();

    FlashLock lock;
    if (!lock) {
        return false;
    }
    (void)scan_locked();
    return true;
}

int parameter_flash_rescan() noexcept
{
    if (in_isr() || !g_initialized) {
        return -EPERM;
    }
    FlashLock lock;
    return lock ? scan_locked() : -EDEADLK;
}

int parameter_flash_load(void *payload, size_t capacity, size_t *payload_size,
                         uint32_t *sequence) noexcept
{
    if (in_isr() || !g_initialized) {
        return -EPERM;
    }
    FlashLock lock;
    if (!lock) {
        return -EDEADLK;
    }
    bool rescanned = false;
    for (;;) {
        if (!g_has_snapshot) {
            return rescanned ? -EIO : -ENOENT;
        }

        invalidate_flash(kStorageBegin + g_latest_offset,
                         kStorageBytes - g_latest_offset);
        RecordHeader header{};
        int validation = validate_record_locked(g_latest_offset, header);
        if (validation == 0) {
            if (payload_size != nullptr) {
                *payload_size = header.payload_length;
            }
            if (sequence != nullptr) {
                *sequence = header.sequence;
            }
            if (header.payload_length > capacity
                || (payload == nullptr && header.payload_length != 0U)) {
                return -ENOBUFS;
            }

            if (header.payload_length == 0U) {
                return 0;
            }
            if (safe_read_bytes(kStorageBegin + g_latest_offset
                                    + sizeof(RecordHeader),
                                payload, header.payload_length)
                && crc32(payload, header.payload_length)
                       == header.payload_crc32) {
                return 0;
            }
            if (mark_fault_once(g_crc_faults, g_latest_offset)) {
                ++g_status.crc_failures;
            }
            validation = -EILSEQ;
        }

        if (rescanned) {
            return validation;
        }
        rescanned = true;
        if (scan_locked() != 0) {
            // A previously known snapshot became corrupt and no fallback exists.
            return -EIO;
        }
    }
}

int parameter_flash_append(const void *payload, size_t payload_size,
                           uint32_t *sequence) noexcept
{
    if (in_isr() || !g_initialized) {
        return -EPERM;
    }
    FlashLock lock;
    if (!lock) {
        return -EDEADLK;
    }
    FlashWriteGuard write_guard;
    return write_guard ? append_locked(payload, payload_size, sequence) : -EAGAIN;
}

int parameter_flash_erase() noexcept
{
    if (in_isr() || !g_initialized) {
        return -EPERM;
    }
    FlashLock lock;
    if (!lock) {
        return -EDEADLK;
    }
    FlashWriteGuard write_guard;
    return write_guard ? erase_locked() : -EAGAIN;
}

int parameter_flash_erase_and_append(const void *payload, size_t payload_size,
                                     uint32_t *sequence) noexcept
{
    if (in_isr() || !g_initialized) {
        return -EPERM;
    }
    FlashLock lock;
    if (!lock) {
        return -EDEADLK;
    }
    FlashWriteGuard write_guard;
    if (!write_guard) {
        return -EAGAIN;
    }
    const int erase_result = erase_locked();
    return erase_result == 0 ? append_locked(payload, payload_size, sequence)
                             : erase_result;
}

ParameterFlashStatus parameter_flash_status() noexcept
{
    if (in_isr() || !g_initialized) {
        return {};
    }
    FlashLock lock;
    return lock ? g_status : ParameterFlashStatus{};
}

} // namespace dima::platform

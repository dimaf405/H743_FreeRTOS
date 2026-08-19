#include "ParameterJournal.hpp"
#include "platform/api/Execution.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dima::parameters {
namespace {

constexpr std::uint32_t kHeaderMagic = UINT32_C(0x444D5048);
constexpr std::uint32_t kCommitMagic = UINT32_C(0x444D5043);
constexpr std::uint32_t kFormatVersion = 1U;

constexpr std::size_t align_flashword(std::size_t value) noexcept
{
    return (value + 31U) & ~std::size_t{31U};
}

std::uint32_t crc32_update(std::uint32_t crc, const std::uint8_t *data,
                           std::size_t length) noexcept
{
    while (length-- > 0U) {
        crc ^= *data++;
        for (std::uint32_t bit = 0U; bit < 8U; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return crc;
}

std::uint32_t crc32(const void *data, std::size_t length) noexcept
{
    if (length == 0U) {
        return 0U;
    }
    return crc32_update(UINT32_MAX,
                        static_cast<const std::uint8_t *>(data), length) ^
           UINT32_MAX;
}

} // namespace

ParameterJournal::ParameterJournal(
    platform::FlashPartition &partition,
    platform::FlashTransactionManager &transactions,
    platform::ArmedFlashCoordinator &armed_flash,
    platform::Synchronization &synchronization) noexcept
    : partition_(partition), transactions_(transactions),
      armed_flash_(armed_flash), synchronization_(synchronization)
{
}

bool ParameterJournal::initialize() noexcept
{
    if (platform::in_interrupt_context()) {
        return false;
    }
    if (!initialized_) {
        if (partition_.size() != kStorageBytes ||
            partition_.program_size() != kFlashWordBytes ||
            (partition_.base() % kFlashWordBytes) != 0U ||
            !operation_mutex_.initialize(synchronization_)) {
            return false;
        }
        initialized_ = true;
    }
    platform::MutexGuard lock{operation_mutex_};
    if (!lock) {
        return false;
    }
    (void)scan_locked();
    return true;
}

void ParameterJournal::clear_runtime_state() noexcept
{
    status_ = {};
    append_offset_ = 0U;
    latest_offset_ = 0U;
    has_snapshot_ = false;
    std::memset(flashword_, 0, sizeof(flashword_));
    std::memset(invalid_faults_, 0, sizeof(invalid_faults_));
    std::memset(crc_faults_, 0, sizeof(crc_faults_));
}

bool ParameterJournal::shutdown() noexcept
{
    if (platform::in_interrupt_context()) {
        return false;
    }
    if (initialized_) {
        {
            platform::MutexGuard lock{operation_mutex_};
            if (!lock) {
                return false;
            }
            initialized_ = false;
            clear_runtime_state();
        }
        operation_mutex_.reset();
    } else {
        clear_runtime_state();
        operation_mutex_.reset();
    }
    return true;
}

bool ParameterJournal::read(std::size_t offset, void *destination,
                            std::size_t length) noexcept
{
    return partition_.read(offset, destination, length);
}

bool ParameterJournal::flashword_erased(std::size_t offset,
                                        bool &readable) noexcept
{
    alignas(4) std::uint8_t data[kFlashWordBytes]{};
    readable = read(offset, data, sizeof(data));
    if (!readable) {
        return false;
    }
    for (std::uint8_t byte : data) {
        if (byte != UINT8_MAX) {
            return false;
        }
    }
    return true;
}

bool ParameterJournal::flash_crc32(std::size_t offset, std::size_t length,
                                   std::uint32_t &result) noexcept
{
    std::uint32_t crc = UINT32_MAX;
    alignas(4) std::uint8_t data[kFlashWordBytes]{};
    std::size_t consumed = 0U;
    while (consumed < length) {
        const std::size_t chunk =
            std::min(sizeof(data), length - consumed);
        if (!read(offset + consumed, data, chunk)) {
            return false;
        }
        crc = crc32_update(crc, data, chunk);
        consumed += chunk;
    }
    result = length == 0U ? 0U : crc ^ UINT32_MAX;
    return true;
}

bool ParameterJournal::mark_fault_once(std::uint32_t *bitmap,
                                       std::size_t offset) noexcept
{
    const std::size_t index = offset / kFlashWordBytes;
    const std::size_t word = index / 32U;
    const std::uint32_t mask = 1UL << (index % 32U);
    if ((bitmap[word] & mask) != 0U) {
        return false;
    }
    bitmap[word] |= mask;
    return true;
}

bool ParameterJournal::header_valid(const RecordHeader &header,
                                    std::size_t offset) const noexcept
{
    constexpr std::size_t kMaxPayloadBytes =
        kStorageBytes - sizeof(RecordHeader) - sizeof(CommitMarker);
    if (header.magic != kHeaderMagic || header.version != kFormatVersion ||
        header.reserved != UINT32_MAX ||
        header.payload_length > kMaxPayloadBytes) {
        return false;
    }
    const std::size_t padded_payload =
        align_flashword(header.payload_length);
    const std::size_t expected_length =
        sizeof(RecordHeader) + padded_payload + sizeof(CommitMarker);
    if (header.record_length != expected_length ||
        expected_length > kStorageBytes - offset) {
        return false;
    }
    return header.header_crc32 ==
           crc32(&header, offsetof(RecordHeader, header_crc32));
}

bool ParameterJournal::marker_valid(const RecordHeader &header,
                                    const CommitMarker &marker) noexcept
{
    return marker.magic == kCommitMagic &&
           marker.sequence == header.sequence &&
           marker.payload_crc32 == header.payload_crc32 &&
           marker.record_length == header.record_length &&
           marker.marker_crc32 ==
               crc32(&marker, offsetof(CommitMarker, marker_crc32));
}

int ParameterJournal::validate_record_locked(std::size_t offset,
                                             RecordHeader &header) noexcept
{
    if (offset > kStorageBytes - sizeof(RecordHeader) ||
        !read(offset, &header, sizeof(header)) ||
        !header_valid(header, offset)) {
        if (offset < kStorageBytes &&
            mark_fault_once(invalid_faults_, offset)) {
            ++status_.invalid_records;
        }
        return -EIO;
    }

    const std::size_t marker_offset =
        offset + header.record_length - sizeof(CommitMarker);
    CommitMarker marker{};
    if (!read(marker_offset, &marker, sizeof(marker)) ||
        !marker_valid(header, marker)) {
        if (mark_fault_once(invalid_faults_, offset)) {
            ++status_.invalid_records;
        }
        return -EIO;
    }

    std::uint32_t payload_crc = 0U;
    if (!flash_crc32(offset + sizeof(RecordHeader), header.payload_length,
                     payload_crc) ||
        payload_crc != header.payload_crc32) {
        if (mark_fault_once(crc_faults_, offset)) {
            ++status_.crc_failures;
        }
        return -EILSEQ;
    }
    return 0;
}

int ParameterJournal::scan_locked() noexcept
{
    status_.valid_sequence = 0U;
    status_.payload_bytes = 0U;
    status_.used_bytes = 0U;
    status_.free_bytes = kStorageBytes;
    append_offset_ = 0U;
    latest_offset_ = 0U;
    has_snapshot_ = false;

    /* 损坏或掉电中断的 flashword 也必须推进 high-water。Flash 不能把已写过的
     * 0 位恢复成 1，复用“洞”会把新记录写进不可擦除的旧数据。 */
    std::size_t high_water = 0U;
    for (std::size_t offset = 0U;
         offset + kFlashWordBytes <= kStorageBytes;
         offset += kFlashWordBytes) {
        bool readable = false;
        const bool erased = flashword_erased(offset, readable);
        if (!readable) {
            high_water = offset + kFlashWordBytes;
            if (mark_fault_once(invalid_faults_, offset)) {
                ++status_.invalid_records;
            }
            continue;
        }
        if (!erased) {
            high_water = offset + kFlashWordBytes;
        }

        RecordHeader header{};
        if (!read(offset, &header, sizeof(header))) {
            if (mark_fault_once(invalid_faults_, offset)) {
                ++status_.invalid_records;
            }
            continue;
        }
        if (header.magic != kHeaderMagic) {
            continue;
        }
        if (!header_valid(header, offset)) {
            if (mark_fault_once(invalid_faults_, offset)) {
                ++status_.invalid_records;
            }
            continue;
        }

        high_water = std::max(
            high_water,
            offset + static_cast<std::size_t>(header.record_length));
        const std::size_t marker_offset =
            offset + header.record_length - sizeof(CommitMarker);
        CommitMarker marker{};
        if (!read(marker_offset, &marker, sizeof(marker)) ||
            !marker_valid(header, marker)) {
            if (mark_fault_once(invalid_faults_, offset)) {
                ++status_.invalid_records;
            }
            continue;
        }

        std::uint32_t payload_crc = 0U;
        if (!flash_crc32(offset + sizeof(RecordHeader),
                         header.payload_length, payload_crc) ||
            payload_crc != header.payload_crc32) {
            if (mark_fault_once(crc_faults_, offset)) {
                ++status_.crc_failures;
            }
            continue;
        }

        if (!has_snapshot_ || header.sequence > status_.valid_sequence) {
            has_snapshot_ = true;
            latest_offset_ = offset;
            status_.valid_sequence = header.sequence;
            status_.payload_bytes = header.payload_length;
        }
    }

    append_offset_ =
        std::min(align_flashword(high_water), kStorageBytes);
    status_.used_bytes = append_offset_;
    status_.free_bytes = kStorageBytes - append_offset_;
    return has_snapshot_ ? 0 : -ENOENT;
}

bool ParameterJournal::program_flashword(std::size_t offset,
                                         const void *source) noexcept
{
    if (source != flashword_) {
        std::memcpy(flashword_, source, sizeof(flashword_));
    }
    return partition_.program(offset, flashword_, sizeof(flashword_));
}

int ParameterJournal::append_locked(const void *payload,
                                    std::size_t payload_size,
                                    std::uint32_t *sequence_out) noexcept
{
    constexpr std::size_t kMaxPayloadBytes =
        kStorageBytes - sizeof(RecordHeader) - sizeof(CommitMarker);
    if (payload == nullptr && payload_size != 0U) {
        return -EINVAL;
    }
    if (payload_size > kMaxPayloadBytes) {
        return -EFBIG;
    }

    const std::size_t padded_payload = align_flashword(payload_size);
    const std::size_t record_length =
        sizeof(RecordHeader) + padded_payload + sizeof(CommitMarker);
    if (record_length > kStorageBytes - append_offset_) {
        ++status_.enospc_failures;
        return -ENOSPC;
    }

    const std::uint32_t sequence =
        has_snapshot_ ? status_.valid_sequence + 1U : 1U;
    RecordHeader header{};
    std::memset(&header, UINT8_MAX, sizeof(header));
    header.magic = kHeaderMagic;
    header.version = kFormatVersion;
    header.sequence = sequence;
    header.payload_length = static_cast<std::uint32_t>(payload_size);
    header.payload_crc32 = crc32(payload, payload_size);
    header.record_length = static_cast<std::uint32_t>(record_length);
    header.header_crc32 =
        crc32(&header, offsetof(RecordHeader, header_crc32));

    CommitMarker marker{};
    std::memset(&marker, UINT8_MAX, sizeof(marker));
    marker.magic = kCommitMagic;
    marker.sequence = sequence;
    marker.payload_crc32 = header.payload_crc32;
    marker.record_length = header.record_length;
    marker.marker_crc32 =
        crc32(&marker, offsetof(CommitMarker, marker_crc32));

    /* 掉电原子性依赖固定顺序：Header -> Payload -> 回读 CRC -> Commit Marker。
     * Marker 最后写入，扫描器才不会把半写记录当作有效快照。 */
    const std::size_t record_offset = append_offset_;
    bool ok = program_flashword(record_offset, &header);
    const auto *payload_bytes = static_cast<const std::uint8_t *>(payload);
    for (std::size_t offset = 0U; ok && offset < padded_payload;
         offset += kFlashWordBytes) {
        std::memset(flashword_, UINT8_MAX, sizeof(flashword_));
        const std::size_t copy_length =
            offset < payload_size
                ? std::min(kFlashWordBytes, payload_size - offset)
                : 0U;
        if (copy_length > 0U) {
            std::memcpy(flashword_, payload_bytes + offset, copy_length);
        }
        ok = program_flashword(record_offset + sizeof(RecordHeader) + offset,
                               flashword_);
    }

    if (ok) {
        std::uint32_t stored_crc = 0U;
        ok = flash_crc32(record_offset + sizeof(RecordHeader), payload_size,
                         stored_crc) &&
             stored_crc == header.payload_crc32;
    }
    if (ok) {
        ok = program_flashword(
            record_offset + record_length - sizeof(CommitMarker), &marker);
    }
    if (!ok) {
        ++status_.write_failures;
        (void)scan_locked();
        return -EIO;
    }

    has_snapshot_ = true;
    latest_offset_ = append_offset_;
    append_offset_ += record_length;
    status_.valid_sequence = sequence;
    status_.payload_bytes = payload_size;
    status_.used_bytes = append_offset_;
    status_.free_bytes = kStorageBytes - append_offset_;
    if (sequence_out != nullptr) {
        *sequence_out = sequence;
    }
    return 0;
}

int ParameterJournal::erase_locked() noexcept
{
    if (!partition_.erase()) {
        ++status_.erase_failures;
        return -EIO;
    }
    std::memset(invalid_faults_, 0, sizeof(invalid_faults_));
    std::memset(crc_faults_, 0, sizeof(crc_faults_));
    (void)scan_locked();
    return 0;
}

int ParameterJournal::rescan() noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return -EPERM;
    }
    platform::MutexGuard lock{operation_mutex_};
    return lock ? scan_locked() : -EDEADLK;
}

int ParameterJournal::load(void *payload, std::size_t capacity,
                           std::size_t *payload_size,
                           std::uint32_t *sequence) noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return -EPERM;
    }
    platform::MutexGuard lock{operation_mutex_};
    if (!lock) {
        return -EDEADLK;
    }

    /* 缓存的 latest offset 每次 load 都重新验证。首次失败只允许全盘重扫一次，
     * 以便回退上一条完整快照，同时避免损坏 Flash 导致无限重试。 */
    bool rescanned = false;
    for (;;) {
        if (!has_snapshot_) {
            return rescanned ? -EIO : -ENOENT;
        }
        RecordHeader header{};
        int validation = validate_record_locked(latest_offset_, header);
        if (validation == 0) {
            if (payload_size != nullptr) {
                *payload_size = header.payload_length;
            }
            if (sequence != nullptr) {
                *sequence = header.sequence;
            }
            if (header.payload_length > capacity ||
                (payload == nullptr && header.payload_length != 0U)) {
                return -ENOBUFS;
            }
            if (header.payload_length == 0U) {
                return 0;
            }
            if (read(latest_offset_ + sizeof(RecordHeader), payload,
                     header.payload_length) &&
                crc32(payload, header.payload_length) ==
                    header.payload_crc32) {
                return 0;
            }
            if (mark_fault_once(crc_faults_, latest_offset_)) {
                ++status_.crc_failures;
            }
            validation = -EILSEQ;
        }

        if (rescanned) {
            return validation;
        }
        rescanned = true;
        if (scan_locked() != 0) {
            return -EIO;
        }
    }
}

int ParameterJournal::append(const void *payload, std::size_t payload_size,
                             std::uint32_t *sequence) noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return -EPERM;
    }
    platform::MutexGuard lock{operation_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    platform::FlashTransaction transaction{transactions_,
                                            platform::Timeout::forever()};
    if (!transaction) {
        return -EDEADLK;
    }
    platform::FlashWriteLease write{armed_flash_};
    return write ? append_locked(payload, payload_size, sequence) : -EAGAIN;
}

int ParameterJournal::erase() noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return -EPERM;
    }
    platform::MutexGuard lock{operation_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    platform::FlashTransaction transaction{transactions_,
                                            platform::Timeout::forever()};
    if (!transaction) {
        return -EDEADLK;
    }
    platform::FlashWriteLease write{armed_flash_};
    return write ? erase_locked() : -EAGAIN;
}

int ParameterJournal::erase_and_append(const void *payload,
                                       std::size_t payload_size,
                                       std::uint32_t *sequence) noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return -EPERM;
    }
    platform::MutexGuard lock{operation_mutex_};
    if (!lock) {
        return -EDEADLK;
    }
    platform::FlashTransaction transaction{transactions_,
                                            platform::Timeout::forever()};
    if (!transaction) {
        return -EDEADLK;
    }
    platform::FlashWriteLease write{armed_flash_};
    if (!write) {
        return -EAGAIN;
    }
    const int erase_result = erase_locked();
    return erase_result == 0 ? append_locked(payload, payload_size, sequence)
                             : erase_result;
}

ParameterJournalStatus ParameterJournal::status() noexcept
{
    if (platform::in_interrupt_context() || !initialized_) {
        return {};
    }
    platform::MutexGuard lock{operation_mutex_};
    return lock ? status_ : ParameterJournalStatus{};
}

} // namespace dima::parameters

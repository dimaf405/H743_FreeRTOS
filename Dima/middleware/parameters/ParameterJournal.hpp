#pragma once

#include "platform/api/Platform.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::parameters {

struct ParameterJournalStatus {
    std::uint32_t valid_sequence{0U};
    std::size_t payload_bytes{0U};
    std::size_t used_bytes{0U};
    std::size_t free_bytes{0U};
    std::uint32_t crc_failures{0U};
    std::uint32_t invalid_records{0U};
    std::uint32_t write_failures{0U};
    std::uint32_t enospc_failures{0U};
    std::uint32_t erase_failures{0U};
};

class ParameterJournal final {
public:
    ParameterJournal(platform::FlashPartition &partition,
                     platform::FlashTransactionManager &transactions,
                     platform::ArmedFlashCoordinator &armed_flash,
                     platform::Synchronization &synchronization) noexcept;

    bool initialize() noexcept;
    bool shutdown() noexcept;
    int rescan() noexcept;
    int load(void *payload, std::size_t capacity, std::size_t *payload_size,
             std::uint32_t *sequence) noexcept;
    int append(const void *payload, std::size_t payload_size,
               std::uint32_t *sequence) noexcept;
    int erase() noexcept;
    int erase_and_append(const void *payload, std::size_t payload_size,
                         std::uint32_t *sequence) noexcept;
    ParameterJournalStatus status() noexcept;

private:
    static constexpr std::size_t kFlashWordBytes = 32U;
    static constexpr std::size_t kStorageBytes = 128U * 1024U;
    static constexpr std::size_t kFaultBitmapWords =
        (kStorageBytes / kFlashWordBytes + 31U) / 32U;

    struct alignas(kFlashWordBytes) RecordHeader {
        std::uint32_t magic;
        std::uint32_t version;
        std::uint32_t sequence;
        std::uint32_t payload_length;
        std::uint32_t payload_crc32;
        std::uint32_t record_length;
        std::uint32_t header_crc32;
        std::uint32_t reserved;
    };

    struct alignas(kFlashWordBytes) CommitMarker {
        std::uint32_t magic;
        std::uint32_t sequence;
        std::uint32_t payload_crc32;
        std::uint32_t record_length;
        std::uint32_t marker_crc32;
        std::uint32_t reserved[3];
    };

    static_assert(sizeof(RecordHeader) == kFlashWordBytes);
    static_assert(sizeof(CommitMarker) == kFlashWordBytes);

    int scan_locked() noexcept;
    int validate_record_locked(std::size_t offset,
                               RecordHeader &header) noexcept;
    int append_locked(const void *payload, std::size_t payload_size,
                      std::uint32_t *sequence) noexcept;
    int erase_locked() noexcept;
    bool read(std::size_t offset, void *destination,
              std::size_t length) noexcept;
    bool flashword_erased(std::size_t offset, bool &readable) noexcept;
    bool flash_crc32(std::size_t offset, std::size_t length,
                     std::uint32_t &result) noexcept;
    bool program_flashword(std::size_t offset, const void *source) noexcept;
    bool header_valid(const RecordHeader &header,
                      std::size_t offset) const noexcept;
    static bool marker_valid(const RecordHeader &header,
                             const CommitMarker &marker) noexcept;
    bool mark_fault_once(std::uint32_t *bitmap,
                         std::size_t offset) noexcept;
    void clear_runtime_state() noexcept;

    platform::FlashPartition &partition_;
    platform::FlashTransactionManager &transactions_;
    platform::ArmedFlashCoordinator &armed_flash_;
    platform::Synchronization &synchronization_;
    platform::RecursiveMutex operation_mutex_{};
    ParameterJournalStatus status_{};
    std::size_t append_offset_{0U};
    std::size_t latest_offset_{0U};
    bool has_snapshot_{false};
    bool initialized_{false};
    alignas(kFlashWordBytes) std::uint8_t flashword_[kFlashWordBytes]{};
    std::uint32_t invalid_faults_[kFaultBitmapWords]{};
    std::uint32_t crc_faults_[kFaultBitmapWords]{};
};

} // namespace dima::parameters

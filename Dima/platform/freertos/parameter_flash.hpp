#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

struct ParameterFlashStatus {
    uint32_t valid_sequence;
    size_t payload_bytes;
    size_t used_bytes;
    size_t free_bytes;
    uint32_t crc_failures;
    uint32_t invalid_records;
    uint32_t write_failures;
    uint32_t enospc_failures;
    uint32_t erase_failures;
};

// 以下接口均为任务上下文接口，禁止从ISR调用。
bool parameter_flash_init() noexcept;
int parameter_flash_rescan() noexcept;
int parameter_flash_load(void *payload, size_t capacity, size_t *payload_size,
                         uint32_t *sequence) noexcept;
int parameter_flash_append(const void *payload, size_t payload_size,
                           uint32_t *sequence) noexcept;
int parameter_flash_erase() noexcept;
int parameter_flash_erase_and_append(const void *payload, size_t payload_size,
                                     uint32_t *sequence) noexcept;
ParameterFlashStatus parameter_flash_status() noexcept;

} // namespace dima::platform

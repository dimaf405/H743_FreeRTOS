#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace dima::platform {

enum class AllocationDomain : uint8_t {
    Startup,
    Service,
    RealtimeForbidden,
};

struct HeapStats {
    size_t total_bytes;
    size_t free_bytes;
    size_t minimum_ever_free_bytes;
    size_t largest_free_block;
    uint32_t allocation_failures;
};

bool heap_init() noexcept;
void *allocate(size_t size, AllocationDomain domain) noexcept;
void deallocate(void *ptr) noexcept;
HeapStats heap_stats() noexcept;
bool in_realtime_context() noexcept;
bool register_realtime_task(TaskHandle_t task) noexcept;
void record_allocation_failure() noexcept;

} // namespace dima::platform

extern "C" bool dima_platform_early_init(void);

#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

enum class AllocationDomain : std::uint8_t {
    Startup,
    Service,
    RealtimeForbidden,
};

struct HeapStats {
    std::size_t total_bytes{0U};
    std::size_t free_bytes{0U};
    std::size_t minimum_ever_free_bytes{0U};
    std::size_t largest_free_block{0U};
    std::uint32_t allocation_failures{0U};
};

class Heap {
public:
    virtual ~Heap() = default;
    virtual bool initialize() noexcept = 0;
    virtual void *allocate(std::size_t size,
                           AllocationDomain domain) noexcept = 0;
    virtual void deallocate(void *pointer) noexcept = 0;
    virtual HeapStats stats() const noexcept = 0;
    virtual std::size_t alignment() const noexcept = 0;
    virtual void record_failure() noexcept = 0;
};

enum class DmaDirection : std::uint8_t {
    PeripheralToMemory,
    MemoryToPeripheral,
    Bidirectional,
};

struct DmaBufferView {
    std::uint8_t *data{nullptr};
    std::size_t size{0U};
    std::uintptr_t token{0U};

    explicit operator bool() const noexcept
    {
        return data != nullptr && size != 0U && token != 0U;
    }
};

class DmaMemory {
public:
    virtual ~DmaMemory() = default;
    virtual DmaBufferView view(void *buffer, std::size_t length) noexcept = 0;
    virtual bool valid(const DmaBufferView &view) const noexcept = 0;
    virtual DmaBufferView acquire_bounce(const void *source,
                                         std::size_t length,
                                         DmaDirection direction) noexcept = 0;
    virtual void release_bounce(DmaBufferView &view, void *destination,
                                DmaDirection direction) noexcept = 0;
};

void *allocate(std::size_t size, AllocationDomain domain) noexcept;
void deallocate(void *pointer) noexcept;
HeapStats heap_stats() noexcept;

} // namespace dima::platform

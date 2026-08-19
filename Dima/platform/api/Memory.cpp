#include "Memory.hpp"

#include "Services.hpp"

namespace dima::platform {

void *allocate(std::size_t size, AllocationDomain domain) noexcept
{
    Services *const installed = try_services();
    return installed != nullptr ? installed->heap.allocate(size, domain)
                                : nullptr;
}

void deallocate(void *pointer) noexcept
{
    Services *const installed = try_services();
    if (installed != nullptr) {
        installed->heap.deallocate(pointer);
    }
}

HeapStats heap_stats() noexcept
{
    Services *const installed = try_services();
    return installed != nullptr ? installed->heap.stats() : HeapStats{};
}

} // namespace dima::platform

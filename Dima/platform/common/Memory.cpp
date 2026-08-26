#include "api/Memory.hpp"

#include "api/Services.hpp"

namespace dima::platform {

void *allocate(std::size_t size, AllocationDomain domain) noexcept
{
    /* 所有动态分配统一经过平台 Heap，以便实时上下文禁配和失败计数生效；服务
     * 尚未安装时直接失败，不回退到 libc heap。 */
    Services *const installed = try_services();
    return installed != nullptr ? installed->heap.allocate(size, domain)
                                : nullptr;
}

void deallocate(void *pointer) noexcept
{
    /* 分配与释放必须属于同一平台 heap；启动早期没有后端时不尝试猜测来源。 */
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

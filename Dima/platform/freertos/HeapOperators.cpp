#include "Backend.hpp"

#include <cstdint>
#include <new>

extern "C" void vApplicationMallocFailedHook(void)
{
    dima::platform::freertos::heap().record_failure();
}

namespace {

// 全局 new/delete 必须统一经过固定 D1 Heap，沿用实时禁配和失败计数语义。
void *allocate_aligned(std::size_t size, std::align_val_t requested) noexcept
{
    std::size_t alignment = static_cast<std::size_t>(requested);
    if (alignment < alignof(void *)) {
        alignment = alignof(void *);
    }
    if ((alignment & (alignment - 1U)) != 0U ||
        size > SIZE_MAX - (alignment - 1U) - sizeof(void *)) {
        dima::platform::freertos::heap().record_failure();
        return nullptr;
    }

    void *const raw = dima::platform::freertos::heap().allocate(
        size + alignment - 1U + sizeof(void *),
        dima::platform::AllocationDomain::Service);
    if (raw == nullptr) {
        return nullptr;
    }
    const std::uintptr_t first =
        reinterpret_cast<std::uintptr_t>(raw) + sizeof(void *);
    const std::uintptr_t aligned =
        (first + alignment - 1U) & ~(alignment - 1U);
    auto **const header = reinterpret_cast<void **>(aligned);
    header[-1] = raw;
    return reinterpret_cast<void *>(aligned);
}

void deallocate_aligned(void *pointer) noexcept
{
    if (pointer == nullptr) {
        return;
    }
    auto **const header = reinterpret_cast<void **>(pointer);
    dima::platform::freertos::heap().deallocate(header[-1]);
}

} // namespace

void *operator new(std::size_t size, const std::nothrow_t &) noexcept
{
    return dima::platform::freertos::heap().allocate(
        size, dima::platform::AllocationDomain::Service);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
    return dima::platform::freertos::heap().allocate(
        size, dima::platform::AllocationDomain::Service);
}

void *operator new(std::size_t size)
{
    return dima::platform::freertos::heap().allocate(
        size, dima::platform::AllocationDomain::Service);
}

void *operator new[](std::size_t size)
{
    return dima::platform::freertos::heap().allocate(
        size, dima::platform::AllocationDomain::Service);
}

void *operator new(std::size_t size, std::align_val_t alignment)
{
    return allocate_aligned(size, alignment);
}

void *operator new[](std::size_t size, std::align_val_t alignment)
{
    return allocate_aligned(size, alignment);
}

void *operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t &) noexcept
{
    return allocate_aligned(size, alignment);
}

void *operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t &) noexcept
{
    return allocate_aligned(size, alignment);
}

void operator delete(void *pointer) noexcept
{
    dima::platform::freertos::heap().deallocate(pointer);
}

void operator delete[](void *pointer) noexcept
{
    dima::platform::freertos::heap().deallocate(pointer);
}

void operator delete(void *pointer, std::size_t) noexcept
{
    dima::platform::freertos::heap().deallocate(pointer);
}

void operator delete[](void *pointer, std::size_t) noexcept
{
    dima::platform::freertos::heap().deallocate(pointer);
}

void operator delete(void *pointer, std::align_val_t) noexcept
{
    deallocate_aligned(pointer);
}

void operator delete[](void *pointer, std::align_val_t) noexcept
{
    deallocate_aligned(pointer);
}

void operator delete(void *pointer, std::size_t,
                     std::align_val_t) noexcept
{
    deallocate_aligned(pointer);
}

void operator delete[](void *pointer, std::size_t,
                       std::align_val_t) noexcept
{
    deallocate_aligned(pointer);
}

void operator delete(void *pointer, std::align_val_t,
                     const std::nothrow_t &) noexcept
{
    deallocate_aligned(pointer);
}

void operator delete[](void *pointer, std::align_val_t,
                       const std::nothrow_t &) noexcept
{
    deallocate_aligned(pointer);
}

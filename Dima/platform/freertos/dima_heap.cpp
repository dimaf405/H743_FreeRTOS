#include "Dima/platform/freertos/dima_platform.hpp"
#include "Dima/platform/freertos/hrt.hpp"

#include <atomic>
#include <new>

#include "stm32h7xx.h"

extern "C" {
#include "portable.h"
}

namespace dima::platform {
namespace {

extern "C" uint8_t __dima_heap_start__;
extern "C" uint8_t __dima_heap_end__;

constexpr size_t kHeapBytes = 256U * 1024U;
constexpr size_t kMaxRealtimeTasks = 12U;
std::atomic<uint32_t> g_allocation_failures{0U};
TaskHandle_t g_realtime_tasks[kMaxRealtimeTasks]{};
size_t g_realtime_task_count{0U};
bool g_heap_initialized{false};

bool is_in_isr() noexcept
{
    return (__get_IPSR() != 0U);
}

} // namespace

bool heap_init() noexcept
{
    if (g_heap_initialized) {
        return true;
    }

    auto *const begin = &__dima_heap_start__;
    auto *const end = &__dima_heap_end__;
    if ((reinterpret_cast<uintptr_t>(begin) & 31U) != 0U ||
        static_cast<size_t>(end - begin) != kHeapBytes) {
        return false;
    }

    const HeapRegion_t regions[] = {
        {begin, kHeapBytes},
        {nullptr, 0U},
    };
    vPortDefineHeapRegions(regions);
    g_heap_initialized = true;
    return true;
}

bool in_realtime_context() noexcept
{
    if (is_in_isr()) {
        return true;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return false;
    }
    const TaskHandle_t current = xTaskGetCurrentTaskHandle();
    for (size_t i = 0U; i < g_realtime_task_count; ++i) {
        if (g_realtime_tasks[i] == current) {
            return true;
        }
    }
    return false;
}

bool register_realtime_task(TaskHandle_t task) noexcept
{
    if (task == nullptr || g_realtime_task_count >= kMaxRealtimeTasks) {
        return false;
    }
    taskENTER_CRITICAL();
    for (size_t i = 0U; i < g_realtime_task_count; ++i) {
        if (g_realtime_tasks[i] == task) {
            taskEXIT_CRITICAL();
            return true;
        }
    }
    g_realtime_tasks[g_realtime_task_count++] = task;
    taskEXIT_CRITICAL();
    return true;
}

void *allocate(size_t size, AllocationDomain domain) noexcept
{
    if (!g_heap_initialized || size == 0U ||
        domain == AllocationDomain::RealtimeForbidden || in_realtime_context()) {
        record_allocation_failure();
        return nullptr;
    }
    void *const memory = pvPortMalloc(size);
    if (memory == nullptr) {
        record_allocation_failure();
    }
    return memory;
}

void deallocate(void *ptr) noexcept
{
    if (ptr != nullptr) {
        vPortFree(ptr);
    }
}

HeapStats heap_stats() noexcept
{
    const size_t free_bytes = g_heap_initialized ? xPortGetFreeHeapSize() : 0U;
    return HeapStats{
        kHeapBytes,
        free_bytes,
        g_heap_initialized ? xPortGetMinimumEverFreeHeapSize() : 0U,
        free_bytes, // 当前 FreeRTOS 版本未公开最大空闲块，先给出安全上界。
        g_allocation_failures.load(std::memory_order_relaxed),
    };
}

void record_allocation_failure() noexcept
{
    g_allocation_failures.fetch_add(1U, std::memory_order_relaxed);
}

} // namespace dima::platform

extern "C" bool dima_platform_early_init(void)
{
    return dima::platform::heap_init() && hrt_init();
}

extern "C" void vApplicationMallocFailedHook(void)
{
    dima::platform::record_allocation_failure();
}

void *operator new(std::size_t size, const std::nothrow_t &) noexcept
{
    return dima::platform::allocate(size, dima::platform::AllocationDomain::Service);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
    return dima::platform::allocate(size, dima::platform::AllocationDomain::Service);
}

void *operator new(std::size_t size)
{
    return dima::platform::allocate(size, dima::platform::AllocationDomain::Service);
}

void *operator new[](std::size_t size)
{
    return dima::platform::allocate(size, dima::platform::AllocationDomain::Service);
}

void operator delete(void *ptr) noexcept { dima::platform::deallocate(ptr); }
void operator delete[](void *ptr) noexcept { dima::platform::deallocate(ptr); }
void operator delete(void *ptr, std::size_t) noexcept { dima::platform::deallocate(ptr); }
void operator delete[](void *ptr, std::size_t) noexcept { dima::platform::deallocate(ptr); }

#include "Backend.hpp"
#include "BackendTimeout.hpp"

#include "platform/api/platform_config.h"

#include <atomic>
#include <cstring>

extern "C" {
#include "FreeRTOS.h"
#include "portable.h"
#include "semphr.h"
#include "task.h"
}

namespace dima::platform::freertos {
namespace {

constexpr std::size_t kHeapBytes = 256U * 1024U;
constexpr std::size_t kMutexCount = 12U;
constexpr std::size_t kSignalCount = 16U;
constexpr std::size_t kTaskCount = 16U;
constexpr std::size_t kTaskStackPoolBytes = 48U * 1024U;
constexpr std::size_t kTaskStackBlockBytes = 256U;
constexpr std::size_t kTaskStackBlocks =
    kTaskStackPoolBytes / kTaskStackBlockBytes;
constexpr std::size_t kTaskBitmapWords = (kTaskStackBlocks + 31U) / 32U;

extern "C" std::uint8_t __dima_heap_start__;
extern "C" std::uint8_t __dima_heap_end__;

alignas(32) std::uint8_t g_task_stack_pool[kTaskStackPoolBytes]
    __attribute__((section(".dima_task_pool")));

bool pointer_in_range(std::uintptr_t pointer, std::uintptr_t begin,
                      std::uintptr_t end, std::size_t stride) noexcept
{
    return pointer >= begin && pointer < end &&
           ((pointer - begin) % stride) == 0U;
}

struct MutexSlot {
    StaticSemaphore_t storage{};
    SemaphoreHandle_t native{nullptr};
    bool recursive{false};
    bool in_use{false};
};

struct SignalSlot {
    StaticSemaphore_t storage{};
    SemaphoreHandle_t native{nullptr};
    bool in_use{false};
};

struct TaskSlot {
    StaticTask_t storage{};
    TaskHandle_t native{nullptr};
    std::size_t first_block{0U};
    std::size_t block_count{0U};
    bool realtime{false};
    bool in_use{false};
};

struct BackendState {
    MutexSlot mutex_slots[kMutexCount]{};
    SignalSlot signal_slots[kSignalCount]{};
    TaskSlot task_slots[kTaskCount]{};
    std::uint32_t task_stack_bitmap[kTaskBitmapWords]{};
    std::atomic<std::uint32_t> allocation_failures{0U};
    MutexHandle flash_mutex{};
    bool heap_initialized{false};
    bool initialized{false};
};

BackendState g_backend_state{};

class Backend final : public ExecutionContext,
                      public CriticalSection,
                      public Synchronization,
                      public TaskRuntime,
                      public Heap,
                      public FlashTransactionManager {
public:
    explicit Backend(BackendState &state) noexcept : state_(state) {}

    bool initialize_backend() noexcept
    {
        if (state_.initialized) {
            return true;
        }
        if (!initialize()) {
            return false;
        }
        state_.flash_mutex = create_mutex(MutexKind::Recursive);
        state_.initialized = static_cast<bool>(state_.flash_mutex);
        return state_.initialized;
    }

    bool in_interrupt() const noexcept override
    {
        return xPortIsInsideInterrupt() != pdFALSE;
    }

    bool scheduler_running() const noexcept override
    {
        return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
    }

    bool in_realtime_task() const noexcept override
    {
        if (in_interrupt()) {
            return true;
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
            return false;
        }
        const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        auto &mutable_backend = const_cast<Backend &>(*this);
        const CriticalToken token = mutable_backend.enter();
        bool realtime = false;
        for (const auto &slot : state_.task_slots) {
            if (slot.in_use && slot.realtime && slot.native == current_task) {
                realtime = true;
                break;
            }
        }
        mutable_backend.leave(token);
        return realtime;
    }

    CriticalToken enter() noexcept override
    {
        CriticalToken token{};
        token.from_interrupt = in_interrupt();
        if (token.from_interrupt) {
            token.state = taskENTER_CRITICAL_FROM_ISR();
        } else {
            taskENTER_CRITICAL();
        }
        return token;
    }

    void leave(CriticalToken token) noexcept override
    {
        if (token.from_interrupt) {
            taskEXIT_CRITICAL_FROM_ISR(
                static_cast<UBaseType_t>(token.state));
        } else {
            taskEXIT_CRITICAL();
        }
    }

    MutexHandle create_mutex(MutexKind kind) noexcept override
    {
        if (in_interrupt()) {
            return {};
        }
        CriticalToken token = enter();
        for (auto &slot : state_.mutex_slots) {
            if (slot.in_use) {
                continue;
            }
            slot.in_use = true;
            slot.recursive = kind == MutexKind::Recursive;
            slot.native = slot.recursive
                              ? xSemaphoreCreateRecursiveMutexStatic(
                                    &slot.storage)
                              : xSemaphoreCreateMutexStatic(&slot.storage);
            if (slot.native == nullptr) {
                slot.in_use = false;
                slot.recursive = false;
                leave(token);
                return {};
            }
            leave(token);
            return MutexHandle{reinterpret_cast<std::uintptr_t>(&slot)};
        }
        leave(token);
        return {};
    }

    void destroy_mutex(MutexHandle handle) noexcept override
    {
        if (in_interrupt()) {
            return;
        }
        MutexSlot *const slot = mutex_slot(handle);
        if (slot == nullptr) {
            return;
        }
        CriticalToken token = enter();
        if (slot->in_use && slot->native != nullptr) {
            vSemaphoreDelete(slot->native);
            slot->native = nullptr;
            slot->recursive = false;
            slot->in_use = false;
            std::memset(&slot->storage, 0, sizeof(slot->storage));
        }
        leave(token);
    }

    bool lock(MutexHandle handle, Timeout timeout) noexcept override
    {
        if (in_interrupt()) {
            return false;
        }
        MutexSlot *const slot = mutex_slot(handle);
        if (slot == nullptr || !slot->in_use || slot->native == nullptr) {
            return false;
        }
        const TickType_t ticks = timeout_to_ticks(timeout);
        return slot->recursive
                   ? xSemaphoreTakeRecursive(slot->native, ticks) == pdTRUE
                   : xSemaphoreTake(slot->native, ticks) == pdTRUE;
    }

    void unlock(MutexHandle handle) noexcept override
    {
        if (in_interrupt()) {
            return;
        }
        MutexSlot *const slot = mutex_slot(handle);
        if (slot == nullptr || !slot->in_use || slot->native == nullptr) {
            return;
        }
        if (slot->recursive) {
            (void)xSemaphoreGiveRecursive(slot->native);
        } else {
            (void)xSemaphoreGive(slot->native);
        }
    }

    SignalHandle create_signal() noexcept override
    {
        if (in_interrupt()) {
            return {};
        }
        CriticalToken token = enter();
        for (auto &slot : state_.signal_slots) {
            if (slot.in_use) {
                continue;
            }
            slot.in_use = true;
            slot.native = xSemaphoreCreateBinaryStatic(&slot.storage);
            if (slot.native == nullptr) {
                slot.in_use = false;
                leave(token);
                return {};
            }
            leave(token);
            return SignalHandle{reinterpret_cast<std::uintptr_t>(&slot)};
        }
        leave(token);
        return {};
    }

    void destroy_signal(SignalHandle handle) noexcept override
    {
        if (in_interrupt()) {
            return;
        }
        SignalSlot *const slot = signal_slot(handle);
        if (slot == nullptr) {
            return;
        }
        CriticalToken token = enter();
        if (slot->in_use && slot->native != nullptr) {
            vSemaphoreDelete(slot->native);
            slot->native = nullptr;
            slot->in_use = false;
            std::memset(&slot->storage, 0, sizeof(slot->storage));
        }
        leave(token);
    }

    bool wait(SignalHandle handle, Timeout timeout) noexcept override
    {
        if (in_interrupt()) {
            return false;
        }
        SignalSlot *const slot = signal_slot(handle);
        return slot != nullptr && slot->in_use && slot->native != nullptr &&
               xSemaphoreTake(slot->native, timeout_to_ticks(timeout)) ==
                   pdTRUE;
    }

    void notify(SignalHandle handle) noexcept override
    {
        if (in_interrupt()) {
            notify_from_isr(handle);
            return;
        }
        SignalSlot *const slot = signal_slot(handle);
        if (slot != nullptr && slot->in_use && slot->native != nullptr) {
            (void)xSemaphoreGive(slot->native);
        }
    }

    void notify_from_isr(SignalHandle handle) noexcept override
    {
        SignalSlot *const slot = signal_slot(handle);
        if (slot == nullptr || !slot->in_use || slot->native == nullptr) {
            return;
        }
        BaseType_t higher_priority_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(slot->native,
                                    &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    TaskHandle create(const TaskConfig &config, TaskEntry entry,
                      void *argument) noexcept override
    {
        if (in_interrupt() || config.name == nullptr || entry == nullptr ||
            config.stack_bytes == 0U || config.priority >= configMAX_PRIORITIES ||
            ::strnlen(config.name, DIMA_TASK_NAME_CAPACITY) >=
                DIMA_TASK_NAME_CAPACITY) {
            return {};
        }

        const std::size_t requested_blocks =
            (config.stack_bytes + kTaskStackBlockBytes - 1U) /
            kTaskStackBlockBytes;
        if (requested_blocks == 0U || requested_blocks > kTaskStackBlocks) {
            return {};
        }

        CriticalToken token = enter();
        TaskSlot *slot = nullptr;
        for (auto &candidate : state_.task_slots) {
            if (!candidate.in_use) {
                slot = &candidate;
                break;
            }
        }
        const std::size_t first_block = find_free_blocks(requested_blocks);
        if (slot == nullptr || first_block == kTaskStackBlocks) {
            leave(token);
            return {};
        }
        mark_blocks(first_block, requested_blocks, true);
        slot->in_use = true;
        slot->realtime = config.realtime;
        slot->first_block = first_block;
        slot->block_count = requested_blocks;
        leave(token);

        auto *const stack = reinterpret_cast<StackType_t *>(
            &g_task_stack_pool[first_block * kTaskStackBlockBytes]);
        const std::size_t stack_bytes = requested_blocks * kTaskStackBlockBytes;
        std::memset(stack, 0, stack_bytes);

        /* Keep the scheduler excluded until the native handle is published.
         * xTaskCreateStatic() may make a higher-priority task ready immediately;
         * the outer critical section defers that context switch until the slot
         * is complete. */
        token = enter();
        slot->native = xTaskCreateStatic(
            entry, config.name,
            static_cast<std::uint32_t>(stack_bytes / sizeof(StackType_t)),
            argument, config.priority, stack, &slot->storage);
        if (slot->native == nullptr) {
            release_task_slot(*slot);
            leave(token);
            return {};
        }
        const TaskHandle handle{reinterpret_cast<std::uintptr_t>(slot)};
        leave(token);
        return handle;
    }

    bool destroy(TaskHandle handle) noexcept override
    {
        if (in_interrupt()) {
            return false;
        }
        TaskSlot *const slot = task_slot(handle);
        CriticalToken token = enter();
        if (slot == nullptr || !slot->in_use || slot->native == nullptr) {
            leave(token);
            return false;
        }
        const TaskHandle_t native = slot->native;
        if (native == xTaskGetCurrentTaskHandle()) {
            leave(token);
            return false;
        }
        vTaskDelete(native);
        release_task_slot(*slot);
        leave(token);
        return true;
    }

    TaskHandle current() const noexcept override
    {
        if (in_interrupt() ||
            xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
            return {};
        }
        const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        auto &mutable_backend = const_cast<Backend &>(*this);
        const CriticalToken token = mutable_backend.enter();
        TaskHandle handle{};
        for (const auto &slot : state_.task_slots) {
            if (slot.in_use && slot.native == current_task) {
                handle = TaskHandle{
                    reinterpret_cast<std::uintptr_t>(&slot)};
                break;
            }
        }
        mutable_backend.leave(token);
        return handle;
    }

    void suspend_current() noexcept override
    {
        if (!in_interrupt()) {
            vTaskSuspend(nullptr);
        }
    }

    void delay(Timeout duration) noexcept override
    {
        if (in_interrupt() || duration.infinite) {
            return;
        }
        vTaskDelay(timeout_to_ticks(duration));
    }

    bool initialize() noexcept override
    {
        if (state_.heap_initialized) {
            return true;
        }
        auto *const begin = &__dima_heap_start__;
        auto *const end = &__dima_heap_end__;
        if ((reinterpret_cast<std::uintptr_t>(begin) & 31U) != 0U ||
            static_cast<std::size_t>(end - begin) != kHeapBytes) {
            return false;
        }
        const HeapRegion_t regions[] = {
            {begin, kHeapBytes},
            {nullptr, 0U},
        };
        vPortDefineHeapRegions(regions);
        state_.heap_initialized = true;
        return true;
    }

    void *allocate(std::size_t size,
                   AllocationDomain domain) noexcept override
    {
        if (!state_.heap_initialized || size == 0U ||
            domain == AllocationDomain::RealtimeForbidden ||
            in_realtime_task()) {
            record_failure();
            return nullptr;
        }
        void *const memory = pvPortMalloc(size);
        if (memory == nullptr) {
            record_failure();
        }
        return memory;
    }

    void deallocate(void *pointer) noexcept override
    {
        if (pointer != nullptr) {
            vPortFree(pointer);
        }
    }

    HeapStats stats() const noexcept override
    {
        HeapStats_t native{};
        if (state_.heap_initialized) {
            vPortGetHeapStats(&native);
        }
        return HeapStats{
            kHeapBytes,
            native.xAvailableHeapSpaceInBytes,
            native.xMinimumEverFreeBytesRemaining,
            native.xSizeOfLargestFreeBlockInBytes,
            state_.allocation_failures.load(std::memory_order_relaxed),
        };
    }

    std::size_t alignment() const noexcept override
    {
        return portBYTE_ALIGNMENT;
    }

    void record_failure() noexcept override
    {
        state_.allocation_failures.fetch_add(1U, std::memory_order_relaxed);
    }

    bool acquire(Timeout timeout) noexcept override
    {
        return state_.flash_mutex && lock(state_.flash_mutex, timeout);
    }

    void release() noexcept override
    {
        if (state_.flash_mutex) {
            unlock(state_.flash_mutex);
        }
    }

private:
    MutexSlot *mutex_slot(MutexHandle handle) noexcept
    {
        const std::uintptr_t begin =
            reinterpret_cast<std::uintptr_t>(&state_.mutex_slots[0]);
        const std::uintptr_t end =
            reinterpret_cast<std::uintptr_t>(
                &state_.mutex_slots[kMutexCount]);
        return pointer_in_range(handle.value, begin, end, sizeof(MutexSlot))
                   ? reinterpret_cast<MutexSlot *>(handle.value)
                   : nullptr;
    }

    SignalSlot *signal_slot(SignalHandle handle) noexcept
    {
        const std::uintptr_t begin =
            reinterpret_cast<std::uintptr_t>(&state_.signal_slots[0]);
        const std::uintptr_t end =
            reinterpret_cast<std::uintptr_t>(
                &state_.signal_slots[kSignalCount]);
        return pointer_in_range(handle.value, begin, end, sizeof(SignalSlot))
                   ? reinterpret_cast<SignalSlot *>(handle.value)
                   : nullptr;
    }

    TaskSlot *task_slot(TaskHandle handle) noexcept
    {
        const std::uintptr_t begin =
            reinterpret_cast<std::uintptr_t>(&state_.task_slots[0]);
        const std::uintptr_t end =
            reinterpret_cast<std::uintptr_t>(&state_.task_slots[kTaskCount]);
        return pointer_in_range(handle.value, begin, end, sizeof(TaskSlot))
                   ? reinterpret_cast<TaskSlot *>(handle.value)
                   : nullptr;
    }

    bool block_used(std::size_t block) const noexcept
    {
        return (state_.task_stack_bitmap[block / 32U] &
                (1UL << (block % 32U))) != 0U;
    }

    std::size_t find_free_blocks(std::size_t count) const noexcept
    {
        std::size_t run = 0U;
        for (std::size_t block = 0U; block < kTaskStackBlocks; ++block) {
            run = block_used(block) ? 0U : run + 1U;
            if (run == count) {
                return block + 1U - count;
            }
        }
        return kTaskStackBlocks;
    }

    void mark_blocks(std::size_t first, std::size_t count,
                     bool used) noexcept
    {
        for (std::size_t block = first; block < first + count; ++block) {
            const std::uint32_t mask = 1UL << (block % 32U);
            if (used) {
                state_.task_stack_bitmap[block / 32U] |= mask;
            } else {
                state_.task_stack_bitmap[block / 32U] &= ~mask;
            }
        }
    }

    void release_task_slot(TaskSlot &slot) noexcept
    {
        mark_blocks(slot.first_block, slot.block_count, false);
        slot.native = nullptr;
        slot.first_block = 0U;
        slot.block_count = 0U;
        slot.realtime = false;
        slot.in_use = false;
        std::memset(&slot.storage, 0, sizeof(slot.storage));
    }

    BackendState &state_;
};

Backend &backend() noexcept
{
    static Backend instance{g_backend_state};
    return instance;
}

} // namespace

bool initialize() noexcept { return backend().initialize_backend(); }
ExecutionContext &execution_context() noexcept { return backend(); }
CriticalSection &critical_section() noexcept { return backend(); }
Synchronization &synchronization() noexcept { return backend(); }
TaskRuntime &task_runtime() noexcept { return backend(); }
Heap &heap() noexcept { return backend(); }
FlashTransactionManager &flash_transactions() noexcept { return backend(); }

} // namespace dima::platform::freertos

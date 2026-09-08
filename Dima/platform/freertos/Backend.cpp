#include "Backend.hpp"
#include "BackendTimeout.hpp"

#include "api/platform_config.h"
#include "api/Services.hpp"
#include "api/Time.hpp"

#include <atomic>
#include <cstring>

extern "C" {
#include "FreeRTOS.h"
#include "portable.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
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

/* 资源均来自固定容量池：互斥量/信号/任务槽不走 heap，任务栈按 256 字节块从
 * 专用链接段分配。bitmap 字数使用 ceil(blocks/32)，末字未用位永不被扫描。 */
extern "C" std::uint8_t __dima_heap_start__;
extern "C" std::uint8_t __dima_heap_end__;

alignas(32) std::uint8_t g_task_stack_pool[kTaskStackPoolBytes]
    __attribute__((section(".dima_task_pool")));

bool pointer_in_range(std::uintptr_t pointer, std::uintptr_t begin,
                      std::uintptr_t end, std::size_t stride) noexcept
{
    /* 对外句柄实际是池内槽地址；范围与 stride 同时校验可拒绝伪造的中间地址，
     * 但它是进程内能力校验，不是安全加密 token。 */
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
        /* Flash 上层调用会嵌套事务，因此使用递归 mutex；创建失败则整个后端
         * 保持未初始化，不能只发布部分 RTOS 服务。 */
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
        /* ISR 一律视为实时上下文；任务实时属性来自受临界区保护的静态槽，
         * 作为动态分配禁用条件，而不是由任务优先级临时推断。 */
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

        /* 栈容量向上取整到 256 字节块：ceil(stack_bytes / block_bytes)。返回的
         * 实际栈可能略大于请求值，但绝不小于请求值。 */
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

        /* xTaskCreateStatic 可能立即使更高优先级任务 ready；外层临界区延迟切换，
         * 直到 native handle 和槽元数据完整发布，避免新任务反查 current 时看到
         * 半初始化槽。 */
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
        /* 禁止任务经此接口删除自身：自身栈仍承载当前调用链，必须由明确的退出
         * 生命周期处理，不能先回收 bitmap 再继续执行。 */
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

    CpuUsage cpu_usage() noexcept override
    {
        // 控制/ISR 只允许读取已有快照；任务枚举最多由非实时上下文每秒执行一次。
        if (in_interrupt()) {
            return {};
        }
        if (in_realtime_task() || !scheduler_running()) {
            return cached_cpu_usage();
        }
        auto *services = try_services();
        if (services == nullptr || !services->clock.initialized()) {
            return cached_cpu_usage();
        }
        const std::uint64_t now = services->clock.now_us();
        CriticalToken token = enter();
        if (runtime_sampling_ ||
            (runtime_attempt_us_ != 0U && now >= runtime_attempt_us_ &&
             now - runtime_attempt_us_ < 1000000ULL)) {
            const CpuUsage result = runtime_usage_;
            leave(token);
            return result;
        }
        runtime_sampling_ = true;
        runtime_attempt_us_ = now;
        leave(token);

        std::uint32_t total{};
        std::size_t idle_index = kRuntimeTaskCapacity;
        // 名称指针属于 TCB，必须在调度暂停期间复制到自有固定表，避免任务删除/
        // 重建后解引用过期名称。这里不关中断，也不使用格式化或动态分配。
        vTaskSuspendAll();
        // 已知任务来自固定平台槽和内核 Idle/Timer；逐项查询时禁止扫描栈高水位，
        // 避免 uxTaskGetSystemState 在暂停调度期间遍历几十 KiB 的空闲栈。
        std::size_t count = 0U;
        for (const auto &slot : state_.task_slots) {
            if (slot.in_use && slot.native != nullptr) {
                vTaskGetInfo(slot.native, &runtime_native_[count++], pdFALSE, eInvalid);
            }
        }
        const TaskHandle_t idle = xTaskGetIdleTaskHandle();
        vTaskGetInfo(idle, &runtime_native_[count++], pdFALSE, eInvalid);
        vTaskGetInfo(xTimerGetTimerDaemonTaskHandle(), &runtime_native_[count++],
                     pdFALSE, eInvalid);
        total = dima_freertos_runtime_counter();
        if (count != uxTaskGetNumberOfTasks()) {
            count = 0U;
        }
        for (std::size_t i = 0U; i < count; ++i) {
            auto &row = runtime_pending_[i];
            row = {};
            row.id = runtime_native_[i].xTaskNumber;
            row.runtime_counter = runtime_native_[i].ulRunTimeCounter;
            if (runtime_native_[i].pcTaskName != nullptr) {
                std::strncpy(row.name, runtime_native_[i].pcTaskName,
                             sizeof(row.name) - 1U);
            }
            if (runtime_native_[i].xHandle == idle) {
                idle_index = i;
            }
        }
        (void)xTaskResumeAll();

        const std::uint64_t sampled_us = services->clock.now_us();
        const std::uint32_t window = total - runtime_previous_total_;
        bool valid = runtime_baseline_valid_ && count != 0U &&
                     count == runtime_previous_count_ &&
                     idle_index < count && total > runtime_previous_total_ &&
                     sampled_us > runtime_previous_us_ &&
                     sampled_us - runtime_previous_us_ <= 60000000ULL &&
                     window != 0U;
        std::uint32_t deltas[kRuntimeTaskCapacity]{};
        for (std::size_t i = 0U; i < count; ++i) {
            const RuntimePrevious *previous = nullptr;
            for (std::size_t j = 0U; j < runtime_previous_count_; ++j) {
                if (runtime_previous_[j].id == runtime_pending_[i].id) {
                    previous = &runtime_previous_[j];
                    break;
                }
            }
            if (previous == nullptr ||
                runtime_pending_[i].runtime_counter < previous->counter) {
                valid = false;
            } else {
                deltas[i] = runtime_pending_[i].runtime_counter -
                            previous->counter;
                if (deltas[i] > window) {
                    valid = false;
                }
            }
        }
        CpuUsage next{};
        next.timestamp_us = sampled_us;
        next.window_us = valid ? window : 0U;
        next.task_count = static_cast<std::uint16_t>(count);
        next.valid = valid;
        // 以 1 us 的运行时间计数求窗口差：load = 1000 * (1 - idle/window)。
        // 64-bit 中间量避免乘 1000 溢出；回绕、任务代次变化和长窗口重建基线。
        if (valid) {
            next.load_permille = static_cast<std::uint16_t>(
                1000ULL - (1000ULL * deltas[idle_index]) / window);
        }
        for (std::size_t i = 0U; i < count; ++i) {
            runtime_pending_[i].valid = valid;
            runtime_pending_[i].load_permille = valid
                ? static_cast<std::uint16_t>((1000ULL * deltas[i]) / window)
                : 0U;
            runtime_previous_[i] = RuntimePrevious{
                runtime_pending_[i].id, runtime_pending_[i].runtime_counter};
        }
        runtime_previous_count_ = count;
        runtime_previous_total_ = total;
        runtime_previous_us_ = sampled_us;
        runtime_baseline_valid_ = count != 0U && idle_index < count;
        // 只在发布有限快照时进入短临界区，读取者不会看到跨代的负载/任务表。
        token = enter();
        runtime_usage_ = next;
        std::memcpy(runtime_published_, runtime_pending_,
                    count * sizeof(TaskCpuUsage));
        runtime_sampling_ = false;
        leave(token);
        return next;
    }

    bool task_cpu_usage(std::size_t index, TaskCpuUsage &usage) noexcept override
    {
        if (in_interrupt()) {
            return false;
        }
        const CriticalToken token = enter();
        const bool available = index < runtime_usage_.task_count;
        if (available) {
            usage = runtime_published_[index];
        }
        leave(token);
        return available;
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
        /* 链接脚本必须提供精确 256 KiB、32 字节对齐区域；不满足时拒绝调用
         * heap_5，防止把相邻 NOLOAD/DMA 段误纳入动态内存。 */
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
        /* RealtimeForbidden 是调用点显式禁配；即便 domain 允许，ISR 或 realtime
         * 任务也统一拒绝，并把拒绝与耗尽都计入 allocation_failures。 */
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
        /* 首次适配连续区间；run 遇已用位归零，命中 count 时反推区间首块。固定池
         * 不做搬迁，碎片只会导致“总空闲足够但无连续区间”的可见失败。 */
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

    static constexpr std::size_t kRuntimeTaskCapacity = kTaskCount + 2U;
    struct RuntimePrevious {
        std::uint32_t id{0U};
        std::uint32_t counter{0U};
    };

    CpuUsage cached_cpu_usage() noexcept
    {
        const CriticalToken token = enter();
        const CpuUsage result = runtime_usage_;
        leave(token);
        return result;
    }

    TaskStatus_t runtime_native_[kRuntimeTaskCapacity]{};
    TaskCpuUsage runtime_pending_[kRuntimeTaskCapacity]{};
    TaskCpuUsage runtime_published_[kRuntimeTaskCapacity]{};
    RuntimePrevious runtime_previous_[kRuntimeTaskCapacity]{};
    CpuUsage runtime_usage_{};
    std::uint64_t runtime_attempt_us_{0U};
    std::uint64_t runtime_previous_us_{0U};
    std::uint32_t runtime_previous_total_{0U};
    std::size_t runtime_previous_count_{0U};
    bool runtime_baseline_valid_{false};
    bool runtime_sampling_{false};

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

extern "C" std::uint32_t dima_freertos_runtime_counter(void)
{
    // 时钟/Services 先于调度器就绪；防御启动早期查询，保留 32-bit 计数 ABI。
    const auto *services = dima::platform::try_services();
    return services != nullptr && services->clock.initialized()
               ? static_cast<std::uint32_t>(services->clock.now_us())
               : 0U;
}

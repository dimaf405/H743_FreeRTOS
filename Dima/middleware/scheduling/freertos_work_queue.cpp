#include "Dima/middleware/scheduling/freertos_work_queue.hpp"

#include "Dima/platform/freertos/platform_time.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include <stdint.h>

namespace dima::middleware::scheduling {
namespace {

constexpr uint32_t kStackBytes = 2048U;
constexpr uint8_t kEntryCount = WorkQueue::kMaxItems;
constexpr UBaseType_t kHighPriority = tskIDLE_PRIORITY + 3U;
constexpr UBaseType_t kLowPriority = tskIDLE_PRIORITY + 2U;

struct Entry {
    bool active;
    void *context;
    IWorkQueueBackend::ClaimCallback claim;
    IWorkQueueBackend::RunCallback run;
    uint32_t deadline_ms;
};

struct WorkerContext {
    Entry *entries;
    uint8_t next_scan_index;
};

Entry g_hp_entries[kEntryCount]{};
Entry g_lp_entries[kEntryCount]{};
StackType_t g_hp_stack[kStackBytes / sizeof(StackType_t)]{};
StackType_t g_lp_stack[kStackBytes / sizeof(StackType_t)]{};
StaticTask_t g_hp_task_buffer{};
StaticTask_t g_lp_task_buffer{};
TaskHandle_t g_hp_task{nullptr};
TaskHandle_t g_lp_task{nullptr};
WorkerContext g_hp_worker{g_hp_entries, 0U};
WorkerContext g_lp_worker{g_lp_entries, 0U};

enum class InitState : uint8_t {
    Uninitialized,
    Initializing,
    Ready,
};

InitState g_init_state{InitState::Uninitialized};

Entry *entries_for(QueueClass queue)
{
    return queue == QueueClass::HighPriority ? g_hp_entries : g_lp_entries;
}

TaskHandle_t task_for(QueueClass queue)
{
    return queue == QueueClass::HighPriority ? g_hp_task : g_lp_task;
}

TickType_t milliseconds_to_ticks(uint32_t milliseconds)
{
    const uint64_t scaled = static_cast<uint64_t>(milliseconds) *
                            static_cast<uint32_t>(configTICK_RATE_HZ);
    uint64_t ticks = (scaled + 999ULL) / 1000ULL;
    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks > static_cast<uint64_t>(portMAX_DELAY)) {
        ticks = static_cast<uint64_t>(portMAX_DELAY);
    }
    return static_cast<TickType_t>(ticks);
}

class FreeRtosClock final : public IClock {
public:
    constexpr FreeRtosClock() = default;

    uint32_t now_ms() const override
    {
        return static_cast<uint32_t>(dima::platform::platform_time_ms());
    }
};

class FreeRtosWorkQueueBackend final : public IWorkQueueBackend {
public:
    constexpr FreeRtosWorkQueueBackend() = default;

    bool schedule(QueueClass queue, void *context, ClaimCallback claim,
                  RunCallback run,
                  uint32_t deadline_ms) override
    {
        bool scheduled = false;
        TaskHandle_t task = nullptr;
        taskENTER_CRITICAL();
        Entry *const entries = entries_for(queue);
        for (uint8_t index = 0U; index < kEntryCount; ++index) {
            if (!entries[index].active) {
                entries[index] = {true, context, claim, run, deadline_ms};
                scheduled = true;
                break;
            }
        }
        task = task_for(queue);
        taskEXIT_CRITICAL();

        if (scheduled && task != nullptr) {
            xTaskNotifyGive(task);
        }
        return scheduled;
    }

    void clear(QueueClass queue, void *context) override
    {
        TaskHandle_t task = nullptr;
        taskENTER_CRITICAL();
        Entry *const entries = entries_for(queue);
        for (uint8_t index = 0U; index < kEntryCount; ++index) {
            if (entries[index].active && entries[index].context == context) {
                entries[index].active = false;
            }
        }
        task = task_for(queue);
        taskEXIT_CRITICAL();

        if (task != nullptr) {
            xTaskNotifyGive(task);
        }
    }
};

FreeRtosClock g_clock{};
FreeRtosWorkQueueBackend g_backend{};
WorkQueue g_hp_queue{QueueClass::HighPriority, g_clock, g_backend};
WorkQueue g_lp_queue{QueueClass::LowPriority, g_clock, g_backend};

bool run_worker_once(WorkerContext &worker)
{
    IWorkQueueBackend::RunCallback run = nullptr;
    void *callback_context = nullptr;
    TickType_t wait_ticks = portMAX_DELAY;

    taskENTER_CRITICAL();
    const uint32_t now_ms = static_cast<uint32_t>(
        dima::platform::platform_time_ms());
    bool have_deadline = false;
    uint32_t first_wait_ms = 0U;

    for (uint8_t offset = 0U; offset < kEntryCount; ++offset) {
        const uint8_t index = static_cast<uint8_t>(
            (worker.next_scan_index + offset) % kEntryCount);
        Entry &entry = worker.entries[index];
        if (!entry.active) {
            continue;
        }
        if (deadline_reached(now_ms, entry.deadline_ms)) {
            entry.active = false;
            if (entry.claim != nullptr && entry.claim(entry.context)) {
                run = entry.run;
                callback_context = entry.context;
                worker.next_scan_index = static_cast<uint8_t>(
                    (index + 1U) % kEntryCount);
                break;
            }
            continue;
        }
        const uint32_t entry_wait_ms = entry.deadline_ms - now_ms;
        if (!have_deadline || entry_wait_ms < first_wait_ms) {
            first_wait_ms = entry_wait_ms;
            have_deadline = true;
        }
    }

    if (run == nullptr && have_deadline) {
        wait_ticks = milliseconds_to_ticks(first_wait_ms);
    }
    taskEXIT_CRITICAL();

    if (run != nullptr) {
        run(callback_context);
        return true;
    }

    (void)ulTaskNotifyTake(pdTRUE, wait_ticks);
    return false;
}

void work_queue_task(void *argument)
{
    auto &worker = *static_cast<WorkerContext *>(argument);
    for (;;) {
        (void)run_worker_once(worker);
    }
}

} // namespace

bool init_default_work_queues()
{
    configASSERT(xPortIsInsideInterrupt() == pdFALSE);
    TaskHandle_t hp_task = nullptr;
    TaskHandle_t lp_task = nullptr;

    taskENTER_CRITICAL();
    if (g_init_state == InitState::Initializing) {
        taskEXIT_CRITICAL();
        return false;
    }
    if (g_init_state == InitState::Ready) {
        hp_task = g_hp_task;
        lp_task = g_lp_task;
        taskEXIT_CRITICAL();
    } else {
        g_init_state = InitState::Initializing;
        hp_task = g_hp_task;
        lp_task = g_lp_task;
        taskEXIT_CRITICAL();

        if (hp_task == nullptr) {
            hp_task = xTaskCreateStatic(work_queue_task, "wq:hp_default",
                                       kStackBytes / sizeof(StackType_t),
                                       &g_hp_worker, kHighPriority, g_hp_stack,
                                       &g_hp_task_buffer);
        }
        if (lp_task == nullptr) {
            lp_task = xTaskCreateStatic(work_queue_task, "wq:lp_default",
                                       kStackBytes / sizeof(StackType_t),
                                       &g_lp_worker, kLowPriority, g_lp_stack,
                                       &g_lp_task_buffer);
        }

        taskENTER_CRITICAL();
        if (g_hp_task == nullptr && hp_task != nullptr) {
            g_hp_task = hp_task;
        }
        if (g_lp_task == nullptr && lp_task != nullptr) {
            g_lp_task = lp_task;
        }
        hp_task = g_hp_task;
        lp_task = g_lp_task;
        const bool ready = hp_task != nullptr && lp_task != nullptr;
        g_init_state = ready ? InitState::Ready : InitState::Uninitialized;
        taskEXIT_CRITICAL();
    }

    if (hp_task != nullptr) {
        xTaskNotifyGive(hp_task);
    }
    if (lp_task != nullptr) {
        xTaskNotifyGive(lp_task);
    }
    return hp_task != nullptr && lp_task != nullptr;
}

WorkQueue &hp_default_work_queue()
{
    return g_hp_queue;
}

WorkQueue &lp_default_work_queue()
{
    return g_lp_queue;
}

#if defined(APP_FREERTOS_WORK_QUEUE_TEST_SEAM)
bool freertos_work_queue_test_run_one_step(QueueClass queue)
{
    return run_worker_once(queue == QueueClass::HighPriority ? g_hp_worker
                                                              : g_lp_worker);
}

void freertos_work_queue_test_reset()
{
    g_hp_task = nullptr;
    g_lp_task = nullptr;
    g_init_state = InitState::Uninitialized;
    g_hp_worker.next_scan_index = 0U;
    g_lp_worker.next_scan_index = 0U;
    for (uint8_t index = 0U; index < kEntryCount; ++index) {
        g_hp_entries[index] = {};
        g_lp_entries[index] = {};
    }
}
#endif

} // namespace dima::middleware::scheduling

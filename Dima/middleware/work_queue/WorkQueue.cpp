#include "Dima/middleware/work_queue/WorkQueue.hpp"

#include "Dima/platform/freertos/platform_time.hpp"
#include "Dima/platform/freertos/dima_platform.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "stm32h743xx.h"

namespace px4 {

class WorkQueueManager {
public:
    static bool schedule(WorkItem &item, hrt_abstime deadline,
                         hrt_abstime interval) noexcept;
    static bool schedule_from_isr(WorkItem &item) noexcept;
    static void worker(void *argument);
};

namespace {

constexpr size_t kMaximumItemsPerQueue = 32U;
constexpr size_t kQueueCount = 7U;
constexpr hrt_abstime kMicrosecondsPerSecond = 1000000ULL;

struct QueueRuntime {
    const wq_config_t *config;
    WorkItem *items[kMaximumItemsPerQueue];
    StaticTask_t task_buffer;
    StackType_t *stack;
    TaskHandle_t task;
    bool started;
};

// 栈空间全部静态保留；stack_size 使用字节表示并在编译期换算。
StackType_t g_estimator_stack[8192U / sizeof(StackType_t)]{};
StackType_t g_rate_ctrl_stack[4096U / sizeof(StackType_t)]{};
StackType_t g_sensors_stack[4096U / sizeof(StackType_t)]{};
StackType_t g_io_stack[4096U / sizeof(StackType_t)]{};
StackType_t g_nav_stack[4096U / sizeof(StackType_t)]{};
StackType_t g_hp_stack[2048U / sizeof(StackType_t)]{};
StackType_t g_lp_stack[2048U / sizeof(StackType_t)]{};

QueueRuntime g_queues[kQueueCount]{};
bool g_initialized{false};

bool in_isr() noexcept
{
    return __get_IPSR() != 0U;
}

TickType_t duration_to_ticks(hrt_abstime duration_us) noexcept
{
    if (duration_us == 0U) {
        return 0U;
    }

    uint64_t ticks = (duration_us * configTICK_RATE_HZ +
                      kMicrosecondsPerSecond - 1U) /
                     kMicrosecondsPerSecond;
    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks > static_cast<uint64_t>(portMAX_DELAY)) {
        ticks = static_cast<uint64_t>(portMAX_DELAY);
    }
    return static_cast<TickType_t>(ticks);
}

QueueRuntime *find_queue(const wq_config_t &config) noexcept
{
    for (auto &queue : g_queues) {
        if (queue.config == &config) {
            return &queue;
        }
    }
    return nullptr;
}

bool attach_item(QueueRuntime &queue, WorkItem &item) noexcept
{
    for (auto *entry : queue.items) {
        if (entry == &item) {
            return true;
        }
    }
    for (auto &entry : queue.items) {
        if (entry == nullptr) {
            entry = &item;
            return true;
        }
    }
    return false;
}

} // namespace

bool WorkQueueManager::schedule(WorkItem &item, hrt_abstime deadline,
              hrt_abstime interval) noexcept
{
    if (!g_initialized || in_isr()) {
        return false;
    }

    QueueRuntime *const queue = find_queue(*item.config_);
    if (queue == nullptr) {
        return false;
    }

    bool accepted = false;
    taskENTER_CRITICAL();
    if (attach_item(*queue, item)) {
        ++item.schedule_revision_;
        item.deadline_ = deadline;
        item.interval_ = interval;
        item.scheduled_ = true;
        accepted = true;
    }
    taskEXIT_CRITICAL();

    if (accepted && queue->task != nullptr) {
        xTaskNotifyGive(queue->task);
    }
    return accepted;
}

bool WorkQueueManager::schedule_from_isr(WorkItem &item) noexcept
{
    if (!g_initialized || !in_isr()) {
        return false;
    }
    QueueRuntime *const queue = find_queue(*item.config_);
    if (queue == nullptr || queue->task == nullptr) {
        return false;
    }
    BaseType_t higher_priority_task_woken = pdFALSE;
    const UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    const bool accepted = attach_item(*queue, item);
    if (accepted) {
        ++item.schedule_revision_;
        item.deadline_ = work_queue_time_us();
        item.interval_ = 0U;
        item.scheduled_ = true;
    }
    taskEXIT_CRITICAL_FROM_ISR(saved);
    if (accepted) {
        vTaskNotifyGiveFromISR(queue->task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
    return accepted;
}

void WorkQueueManager::worker(void *argument)
{
    auto &queue = *static_cast<QueueRuntime *>(argument);

    for (;;) {
        WorkItem *ready = nullptr;
        uint32_t revision = 0U;
        hrt_abstime deadline = 0U;
        hrt_abstime wait_us = UINT64_MAX;
        const hrt_abstime now = work_queue_time_us();

        taskENTER_CRITICAL();
        for (auto *item : queue.items) {
            if (item == nullptr || !item->scheduled_ || item->running_) {
                continue;
            }
            if (item->deadline_ <= now) {
                ready = item;
                revision = item->schedule_revision_;
                deadline = item->deadline_;
                item->scheduled_ = false;
                item->running_ = true;
                break;
            }
            const hrt_abstime remaining = item->deadline_ - now;
            if (remaining < wait_us) {
                wait_us = remaining;
            }
        }
        taskEXIT_CRITICAL();

        if (ready == nullptr) {
            const TickType_t ticks = wait_us == UINT64_MAX
                                         ? portMAX_DELAY
                                         : duration_to_ticks(wait_us);
            (void)ulTaskNotifyTake(pdTRUE, ticks);
            continue;
        }

        const hrt_abstime started = work_queue_time_us();
        if (started > deadline) {
            ++ready->statistics_.deadline_misses;
        }
        ready->Run();
        const hrt_abstime finished = work_queue_time_us();
        const hrt_abstime elapsed = finished - started;

        taskENTER_CRITICAL();
        ++ready->statistics_.executions;
        ready->statistics_.last_execution_time = elapsed;
        if (elapsed > ready->statistics_.maximum_execution_time) {
            ready->statistics_.maximum_execution_time = elapsed;
        }
        ready->running_ = false;
        // Run() 内若重新调度或清除，revision 会变化，此处不得覆盖新请求。
        if (ready->schedule_revision_ == revision && ready->interval_ != 0U) {
            ready->deadline_ = finished + ready->interval_;
            ready->scheduled_ = true;
        }
        taskEXIT_CRITICAL();
    }
}

namespace {

void configure_runtime() noexcept
{
    const wq_config_t *configs[kQueueCount] = {
        &wq_configurations::estimator, &wq_configurations::rate_ctrl,
        &wq_configurations::sensors,   &wq_configurations::io,
        &wq_configurations::nav,       &wq_configurations::hp_default,
        &wq_configurations::lp_default};
    StackType_t *stacks[kQueueCount] = {
        g_estimator_stack, g_rate_ctrl_stack, g_sensors_stack, g_io_stack,
        g_nav_stack, g_hp_stack, g_lp_stack};

    for (size_t index = 0U; index < kQueueCount; ++index) {
        g_queues[index].config = configs[index];
        g_queues[index].stack = stacks[index];
    }
}

} // namespace

namespace wq_configurations {
const wq_config_t estimator{"wq:estimator", 8U, 8192U, true};
const wq_config_t rate_ctrl{"wq:rate_ctrl", 7U, 4096U, true};
const wq_config_t sensors{"wq:sensors", 6U, 4096U, true};
const wq_config_t io{"wq:io", 5U, 4096U, false};
const wq_config_t nav{"wq:nav", 4U, 4096U, false};
const wq_config_t hp_default{"wq:hp_default", 3U, 2048U, false};
const wq_config_t lp_default{"wq:lp_default", 2U, 2048U, false};
} // namespace wq_configurations

// HRT 阶段可提供同名强符号覆盖该弱实现，调用方无需修改。
__attribute__((weak)) hrt_abstime work_queue_time_us() noexcept
{
    return dima::platform::platform_time_us();
}

WorkItem::WorkItem(const char *name, const wq_config_t &config) noexcept
    : name_(name), config_(&config)
{
}

bool WorkItem::ScheduleNow() noexcept
{
    return WorkQueueManager::schedule(*this, work_queue_time_us(), 0U);
}

bool WorkItem::ScheduleNowFromISR() noexcept
{
    return WorkQueueManager::schedule_from_isr(*this);
}

void WorkItem::ScheduleClear() noexcept
{
    if (in_isr()) {
        return;
    }
    taskENTER_CRITICAL();
    ++schedule_revision_;
    scheduled_ = false;
    interval_ = 0U;
    taskEXIT_CRITICAL();
}

bool ScheduledWorkItem::ScheduleDelayed(uint32_t delay_us) noexcept
{
    return WorkQueueManager::schedule(*this, work_queue_time_us() + delay_us, 0U);
}

bool ScheduledWorkItem::ScheduleOnInterval(uint32_t interval_us,
                                           uint32_t delay_us) noexcept
{
    if (interval_us == 0U) {
        return false;
    }
    const uint32_t first_delay = delay_us == 0U ? interval_us : delay_us;
    return WorkQueueManager::schedule(*this, work_queue_time_us() + first_delay, interval_us);
}

bool ScheduledWorkItem::ScheduleAt(hrt_abstime time_us) noexcept
{
    return WorkQueueManager::schedule(*this, time_us, 0U);
}

bool work_queue_init() noexcept
{
    if (g_initialized || in_isr()) {
        return g_initialized;
    }

    configure_runtime();
    for (auto &queue : g_queues) {
        const uint32_t stack_depth = queue.config->stack_size /
                                     sizeof(StackType_t);
        queue.task = xTaskCreateStatic(WorkQueueManager::worker, queue.config->name, stack_depth,
                                       &queue, queue.config->priority,
                                       queue.stack, &queue.task_buffer);
        if (queue.task == nullptr) {
            work_queue_shutdown();
            return false;
        }
        queue.started = true;
        if (queue.config->realtime &&
            !dima::platform::register_realtime_task(queue.task)) {
            work_queue_shutdown();
            return false;
        }
    }
    g_initialized = true;
    return true;
}

void work_queue_shutdown() noexcept
{
    if (in_isr()) {
        return;
    }
    for (auto &queue : g_queues) {
        if (queue.started && queue.task != nullptr) {
            if (queue.config != nullptr && queue.config->realtime) {
                (void)dima::platform::unregister_realtime_task(queue.task);
            }
            vTaskDelete(queue.task);
        }
        queue.task = nullptr;
        queue.started = false;
        for (auto &item : queue.items) {
            item = nullptr;
        }
    }
    g_initialized = false;
}

} // namespace px4





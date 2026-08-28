#include "WorkQueue.hpp"

#include "api/Execution.hpp"
#include "api/Services.hpp"
#include "api/Synchronization.hpp"
#include "api/TaskRuntime.hpp"

#include <cstddef>
#include <cstdint>

namespace px4 {

class WorkQueueManager {
public:
    static bool schedule(WorkItem &item, hrt_abstime deadline,
                         hrt_abstime interval) noexcept;
    static bool schedule_from_isr(WorkItem &item) noexcept;
    static void worker(void *argument);
};

namespace {

constexpr std::size_t kMaximumItemsPerQueue = 32U;
constexpr std::size_t kQueueCount = 8U;

struct QueueRuntime {
    const wq_config_t *config{nullptr};
    WorkItem *items[kMaximumItemsPerQueue]{};
    dima::platform::SignalHandle signal{};
    dima::platform::TaskHandle task{};
    bool started{false};
};

QueueRuntime g_queues[kQueueCount]{};

enum class ManagerState : std::uint8_t {
    Stopped,
    Starting,
    Running,
    Stopping,
};

ManagerState g_state{ManagerState::Stopped};
dima::platform::TaskHandle g_owner_task{};

/* 管理器只能由 owner 任务执行 init/shutdown：
 * Stopped -> Starting -> Running -> Stopping -> Stopped。
 * 这样部分创建失败可由同一调用栈按逆序回收，不与 worker 自销毁交错。 */

bool in_isr() noexcept
{
    return dima::platform::in_interrupt_context();
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

void detach_item(QueueRuntime &queue, WorkItem &item) noexcept
{
    for (auto &entry : queue.items) {
        if (entry == &item) {
            entry = nullptr;
            return;
        }
    }
}

void configure_runtime() noexcept
{
    const wq_config_t *configs[kQueueCount] = {
        &wq_configurations::estimator, &wq_configurations::rate_ctrl,
        &wq_configurations::sensors,   &wq_configurations::io,
        &wq_configurations::nav,        &wq_configurations::hp_default,
        &wq_configurations::lp_default, &wq_configurations::storage,
    };
    for (std::size_t index = 0U; index < kQueueCount; ++index) {
        g_queues[index].config = configs[index];
    }
}

} // namespace

bool WorkQueueManager::schedule(WorkItem &item, hrt_abstime deadline,
                                hrt_abstime interval) noexcept
{
    if (in_isr()) {
        return false;
    }

    dima::platform::CriticalGuard guard;
    if (g_state != ManagerState::Running) {
        return false;
    }
    QueueRuntime *const queue = find_queue(*item.config_);
    if (queue == nullptr || !queue->started || !queue->task ||
        !queue->signal || !item.accepting_schedules_ ||
        !attach_item(*queue, item)) {
        return false;
    }

    /* revision 标识本次调度所有权；Run 内重排/取消会推进 revision，使 worker
     * 收尾阶段不会用旧 interval 覆盖新请求。 */
    ++item.schedule_revision_;
    item.deadline_ = deadline;
    item.interval_ = interval;
    item.scheduled_ = true;
    dima::platform::services().synchronization.notify(queue->signal);
    return true;
}

bool WorkQueueManager::schedule_from_isr(WorkItem &item) noexcept
{
    if (!in_isr()) {
        return false;
    }

    dima::platform::CriticalGuard guard;
    if (g_state != ManagerState::Running) {
        return false;
    }
    QueueRuntime *const queue = find_queue(*item.config_);
    if (queue == nullptr || !queue->started || !queue->task ||
        !queue->signal || !item.accepting_schedules_ ||
        !attach_item(*queue, item)) {
        return false;
    }

    ++item.schedule_revision_;
    item.deadline_ = work_queue_time_us();
    item.interval_ = 0U;
    item.scheduled_ = true;
    dima::platform::services().synchronization.notify_from_isr(
        queue->signal);
    return true;
}

void WorkQueueManager::worker(void *argument)
{
    auto &queue = *static_cast<QueueRuntime *>(argument);

    for (;;) {
        WorkItem *ready = nullptr;
        std::uint32_t revision = 0U;
        hrt_abstime deadline = 0U;
        hrt_abstime wait_us = UINT64_MAX;
        const hrt_abstime now = work_queue_time_us();

        {
            dima::platform::CriticalGuard guard;
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
        }

        /* 无到期项时等待“最近 deadline-now”或队列 signal；新调度会 signal 提前
         * 唤醒并重新计算，forever 仅表示队列当前为空。 */
        if (ready == nullptr) {
            const auto timeout =
                wait_us == UINT64_MAX
                    ? dima::platform::Timeout::forever()
                    : dima::platform::Timeout::from_us(wait_us);
            (void)dima::platform::services().synchronization.wait(
                queue.signal, timeout);
            continue;
        }

        const hrt_abstime started = work_queue_time_us();
        if (started > deadline) {
            ++ready->statistics_.deadline_misses;
        }
        ready->Run();
        const hrt_abstime finished = work_queue_time_us();
        const hrt_abstime elapsed = finished - started;

        {
            dima::platform::CriticalGuard guard;
            ++ready->statistics_.executions;
            ready->statistics_.last_execution_time = elapsed;
            if (elapsed > ready->statistics_.maximum_execution_time) {
                ready->statistics_.maximum_execution_time = elapsed;
            }
            ready->running_ = false;
            /* Run 可能自行重排或清除；revision 未变且仍为周期项时，按原 deadline
             * 跳过已经错过的周期：elapsed=(finished-deadline)/interval，
             * next=deadline+(elapsed+1)*interval。锚定原节拍而非 finished，避免漂移。 */
            if (!ready->accepting_schedules_) {
                detach_item(queue, *ready);
            } else if (ready->schedule_revision_ == revision &&
                       ready->interval_ != 0U) {
                const hrt_abstime elapsed_periods =
                    (finished - deadline) / ready->interval_;
                const hrt_abstime maximum_periods =
                    (UINT64_MAX - deadline) / ready->interval_;
                if (elapsed_periods < maximum_periods) {
                    ready->deadline_ =
                        deadline + (elapsed_periods + 1U) * ready->interval_;
                    ready->scheduled_ = true;
                } else {
                    ready->interval_ = 0U;
                }
            }
        }
    }
}

namespace wq_configurations {
const wq_config_t estimator{"wq:estimator", 8U, 8192U, true};
const wq_config_t rate_ctrl{"wq:rate_ctrl", 7U, 4096U, true};
const wq_config_t sensors{"wq:sensors", 6U, 4096U, true};
const wq_config_t io{"wq:io", 5U, 4096U, false};
const wq_config_t nav{"wq:nav", 4U, 4096U, false};
const wq_config_t hp_default{"wq:hp_default", 3U, 2048U, false};
/* 结构化日志、校准和 MAVLink 共用此调用栈；实板 QGC 长连接曾测得
 * PSP 低于旧 2 KiB 栈底 56 bytes，因此固定保留 4 KiB。 */
const wq_config_t lp_default{"wq:lp_default", 2U, 4096U, false};
/* SD/FatFs/HAL 允许同步等待，只能位于比通信更低优先级的独立执行域；即使坏卡
 * 耗尽单次有限超时，也不能饿死 HEARTBEAT、传感器遥测或结构化日志。 */
const wq_config_t storage{"wq:storage", 1U, 4096U, false};
} // namespace wq_configurations

hrt_abstime work_queue_time_us() noexcept
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

bool WorkItem::ScheduleEnable() noexcept
{
    if (in_isr()) {
        return false;
    }
    bool enabled = false;
    {
        dima::platform::CriticalGuard guard;
        if (accepting_schedules_ || !running_) {
            /* 从 disabled 重新启用时推进 revision，并清截止/周期/统计，确保上一代
             * worker 收尾不能复活旧调度。 */
            if (!accepting_schedules_) {
                ++schedule_revision_;
                deadline_ = 0U;
                interval_ = 0U;
                scheduled_ = false;
                statistics_ = {};
            }
            accepting_schedules_ = true;
            enabled = true;
        }
    }
    return enabled;
}

void WorkItem::ScheduleClear() noexcept
{
    if (in_isr()) {
        return;
    }
    dima::platform::CriticalGuard guard;
    ++schedule_revision_;
    scheduled_ = false;
    interval_ = 0U;
}

void WorkItem::ScheduleCancelAndDrain() noexcept
{
    if (in_isr()) {
        return;
    }

    QueueRuntime *const queue = find_queue(*config_);
    const dima::platform::TaskHandle current =
        dima::platform::services().tasks.current();
    bool running = false;
    bool called_from_run = false;
    {
        dima::platform::CriticalGuard guard;
        ++schedule_revision_;
        scheduled_ = false;
        interval_ = 0U;
        accepting_schedules_ = false;
        running = running_;
        called_from_run = running && queue != nullptr &&
                          queue->task == current;
        if (!running && queue != nullptr) {
            detach_item(*queue, *this);
        }
    }

    /* 从自身 Run 调用若继续等待会死锁；此时仅标记不再接收调度，worker 收尾负责
     * detach。外部调用以 1 ms 轮询等待 running 清除。 */
    if (called_from_run) {
        return;
    }

    while (running) {
        dima::platform::services().tasks.delay(
            dima::platform::Timeout::from_ms(1U));
        dima::platform::CriticalGuard guard;
        running = running_;
        if (!running && queue != nullptr) {
            detach_item(*queue, *this);
        }
    }
}

bool ScheduledWorkItem::ScheduleDelayed(std::uint32_t delay_us) noexcept
{
    return WorkQueueManager::schedule(
        *this, work_queue_time_us() + delay_us, 0U);
}

bool ScheduledWorkItem::ScheduleOnInterval(std::uint32_t interval_us,
                                           std::uint32_t delay_us) noexcept
{
    if (interval_us == 0U) {
        return false;
    }
    const std::uint32_t first_delay =
        delay_us == 0U ? interval_us : delay_us;
    return WorkQueueManager::schedule(
        *this, work_queue_time_us() + first_delay, interval_us);
}

bool ScheduledWorkItem::ScheduleAt(hrt_abstime time_us) noexcept
{
    return WorkQueueManager::schedule(*this, time_us, 0U);
}

bool work_queue_init() noexcept
{
    if (in_isr() || !dima::platform::services_installed()) {
        return false;
    }

    auto &services = dima::platform::services();
    const dima::platform::TaskHandle current = services.tasks.current();
    if (!current) {
        return false;
    }
    {
        dima::platform::CriticalGuard guard;
        if (g_state == ManagerState::Running) {
            return g_owner_task == current;
        }
        if (g_state != ManagerState::Stopped) {
            return false;
        }
        g_owner_task = current;
        g_state = ManagerState::Starting;
        configure_runtime();
    }

    /* 每队列先建 signal 再建静态任务；任一失败统一进入 shutdown 回滚已创建项，
     * 只有八个队列全部就绪才把全局状态发布为 Running。 */
    for (auto &queue : g_queues) {
        queue.signal = services.synchronization.create_signal();
        if (!queue.signal) {
            (void)work_queue_shutdown();
            return false;
        }
        const dima::platform::TaskConfig config{
            queue.config->name,
            queue.config->priority,
            queue.config->stack_size,
            queue.config->realtime,
        };
        queue.task = services.tasks.create(
            config, &WorkQueueManager::worker, &queue);
        if (!queue.task) {
            (void)work_queue_shutdown();
            return false;
        }
        queue.started = true;
    }

    {
        dima::platform::CriticalGuard guard;
        g_state = ManagerState::Running;
    }
    return true;
}

bool work_queue_shutdown() noexcept
{
    if (in_isr() || !dima::platform::services_installed()) {
        return false;
    }

    auto &services = dima::platform::services();
    auto &tasks = services.tasks;
    const dima::platform::TaskHandle current = tasks.current();
    if (!current) {
        return false;
    }

    {
        dima::platform::CriticalGuard guard;
        if (g_state == ManagerState::Stopped) {
            g_owner_task = {};
            return true;
        }
        if (g_owner_task != current) {
            return false;
        }
        for (const auto &queue : g_queues) {
            if (queue.task && queue.task == current) {
                return false;
            }
        }
        g_state = ManagerState::Stopping;
    }

    /* 先逐项 cancel+drain，保证没有 Run 再访问模块对象；随后销毁 worker，再销毁
     * signal。若任一任务销毁失败，保持 Stopping 供调用方重试，不伪报 Stopped。 */
    for (auto &queue : g_queues) {
        for (;;) {
            WorkItem *item = nullptr;
            {
                dima::platform::CriticalGuard guard;
                for (auto *entry : queue.items) {
                    if (entry != nullptr) {
                        item = entry;
                        break;
                    }
                }
            }
            if (item == nullptr) {
                break;
            }
            item->ScheduleCancelAndDrain();
        }
    }

    bool destroyed = true;
    for (auto &queue : g_queues) {
        if (queue.started && queue.task) {
            if (!tasks.destroy(queue.task)) {
                destroyed = false;
                continue;
            }
        }
        queue.task = {};
        queue.started = false;
        if (queue.signal) {
            services.synchronization.destroy_signal(queue.signal);
            queue.signal = {};
        }
    }
    if (!destroyed) {
        return false;
    }

    {
        dima::platform::CriticalGuard guard;
        for (auto &queue : g_queues) {
            for (auto &item : queue.items) {
                item = nullptr;
            }
        }
        g_state = ManagerState::Stopped;
        g_owner_task = {};
    }
    return true;
}

} // namespace px4

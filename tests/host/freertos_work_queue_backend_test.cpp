#include "test_framework.hpp"

#include "App/runtime/time/platform_time.hpp"
#include "App/runtime/scheduling/freertos_work_queue.hpp"
#include "App/runtime/scheduling/scheduled_work_item.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace app::runtime::scheduling {
bool freertos_work_queue_test_run_one_step(QueueClass queue);
void freertos_work_queue_test_reset();
}

namespace {

using app::runtime::scheduling::QueueClass;
using app::runtime::scheduling::ScheduledWorkItem;
using app::runtime::scheduling::WorkQueue;

struct TaskRecord {
    TaskFunction_t function{nullptr};
    const char *name{nullptr};
    uint32_t stack_depth{0U};
    void *argument{nullptr};
    UBaseType_t priority{0U};
    StackType_t *stack{nullptr};
    StaticTask_t *task_buffer{nullptr};
    uint32_t notifications{0U};
};

std::array<TaskRecord, 4> g_tasks{};
unsigned g_create_attempts{0U};
unsigned g_successful_creates{0U};
unsigned g_fail_create_attempt{UINT32_MAX};
unsigned g_critical_depth{0U};
unsigned g_max_critical_depth{0U};
TaskRecord *g_current_task{nullptr};
TickType_t g_last_wait_ticks{0U};
unsigned g_notify_take_calls{0U};
bool g_inside_interrupt{false};
void (*g_before_wait_hook)(){nullptr};
void (*g_create_hook)(){nullptr};
bool g_create_hook_deferred{false};
bool g_reentrant_init_result{false};
uint64_t g_now_ms{0U};

class BackendItem final : public ScheduledWorkItem {
public:
    explicit BackendItem(WorkQueue &queue, bool reschedule_now = false)
        : ScheduledWorkItem(queue), reschedule_now_(reschedule_now)
    {
    }

    unsigned runs{0U};

private:
    void Run() override
    {
        ++runs;
        if (reschedule_now_) {
            CHECK(ScheduleNow());
        }
    }

    bool reschedule_now_;
};

BackendItem *g_wait_hook_item{nullptr};

void schedule_wait_hook_item()
{
    CHECK(g_wait_hook_item != nullptr);
    CHECK(g_wait_hook_item->ScheduleNow());
}

TaskRecord &record_for(QueueClass queue)
{
    return queue == QueueClass::HighPriority ? g_tasks[0] : g_tasks[1];
}

bool run_one_step(QueueClass queue)
{
    g_current_task = &record_for(queue);
    const bool ran =
        app::runtime::scheduling::freertos_work_queue_test_run_one_step(queue);
    g_current_task = nullptr;
    return ran;
}

void reset_backend_test_state()
{
    app::runtime::scheduling::freertos_work_queue_test_reset();
    g_tasks = {};
    g_create_attempts = 0U;
    g_successful_creates = 0U;
    g_fail_create_attempt = UINT32_MAX;
    g_critical_depth = 0U;
    g_max_critical_depth = 0U;
    g_current_task = nullptr;
    g_last_wait_ticks = 0U;
    g_notify_take_calls = 0U;
    g_inside_interrupt = false;
    g_before_wait_hook = nullptr;
    g_create_hook = nullptr;
    g_create_hook_deferred = false;
    g_reentrant_init_result = false;
    g_now_ms = 0U;
}

void reenter_default_queue_init()
{
    g_reentrant_init_result =
        app::runtime::scheduling::init_default_work_queues();
}

} // namespace

extern "C" TaskHandle_t xTaskCreateStatic(TaskFunction_t task_function,
                                            const char *task_name,
                                            uint32_t stack_depth,
                                            void *argument,
                                            UBaseType_t priority,
                                            StackType_t *stack_buffer,
                                            StaticTask_t *task_buffer)
{
    ++g_create_attempts;
    if (g_create_attempts == g_fail_create_attempt) {
        return nullptr;
    }
    if (g_successful_creates >= g_tasks.size()) {
        throw std::runtime_error("unexpected static task creation");
    }
    TaskRecord &record = g_tasks[g_successful_creates++];
    record.function = task_function;
    record.name = task_name;
    record.stack_depth = stack_depth;
    record.argument = argument;
    record.priority = priority;
    record.stack = stack_buffer;
    record.task_buffer = task_buffer;
    if (g_create_hook != nullptr) {
        void (*const hook)() = g_create_hook;
        g_create_hook = nullptr;
        if (g_critical_depth == 0U) {
            hook();
        } else {
            g_create_hook = hook;
            g_create_hook_deferred = true;
        }
    }
    return &record;
}

extern "C" void xTaskNotifyGive(TaskHandle_t task)
{
    auto *const record = static_cast<TaskRecord *>(task);
    if (record == nullptr) {
        throw std::runtime_error("notification sent to null task");
    }
    ++record->notifications;
}

extern "C" uint32_t ulTaskNotifyTake(BaseType_t clear_count_on_exit,
                                       TickType_t ticks_to_wait)
{
    ++g_notify_take_calls;
    g_last_wait_ticks = ticks_to_wait;
    if (g_before_wait_hook != nullptr) {
        void (*const hook)() = g_before_wait_hook;
        g_before_wait_hook = nullptr;
        hook();
    }
    if (g_current_task == nullptr || g_current_task->notifications == 0U) {
        return 0U;
    }
    const uint32_t value = g_current_task->notifications;
    if (clear_count_on_exit == pdTRUE) {
        g_current_task->notifications = 0U;
    } else {
        --g_current_task->notifications;
    }
    return value;
}

extern "C" void freertos_test_enter_critical(void)
{
    ++g_critical_depth;
    if (g_critical_depth > g_max_critical_depth) {
        g_max_critical_depth = g_critical_depth;
    }
}

extern "C" void freertos_test_exit_critical(void)
{
    if (g_critical_depth == 0U) {
        throw std::runtime_error("unbalanced critical section exit");
    }
    --g_critical_depth;
    if (g_critical_depth == 0U && g_create_hook_deferred) {
        g_create_hook_deferred = false;
        void (*const hook)() = g_create_hook;
        g_create_hook = nullptr;
        hook();
    }
}

extern "C" void freertos_test_assert(BaseType_t condition)
{
    if (condition == pdFALSE) {
        throw std::runtime_error("FreeRTOS configASSERT failed");
    }
}

extern "C" BaseType_t xPortIsInsideInterrupt(void)
{
    return g_inside_interrupt ? pdTRUE : pdFALSE;
}

namespace app::runtime::time {

uint64_t platform_time_us()
{
    return g_now_ms * 1000U;
}

uint64_t platform_time_ms()
{
    return g_now_ms;
}

} // namespace app::runtime::time

HOST_TEST(freertos_backend_partial_static_worker_creation_can_retry_safely)
{
    reset_backend_test_state();
    g_fail_create_attempt = 2U;

    CHECK(!app::runtime::scheduling::init_default_work_queues());
    CHECK_EQ(g_create_attempts, 2U);
    CHECK_EQ(g_successful_creates, 1U);

    CHECK(app::runtime::scheduling::init_default_work_queues());
    CHECK_EQ(g_create_attempts, 3U);
    CHECK_EQ(g_successful_creates, 2U);
    CHECK(std::strcmp(g_tasks[0].name, "wq:hp_default") == 0);
    CHECK(std::strcmp(g_tasks[1].name, "wq:lp_default") == 0);
    CHECK_NE(g_tasks[0].stack, g_tasks[1].stack);
}

HOST_TEST(freertos_backend_concurrent_init_publishes_each_static_worker_once)
{
    reset_backend_test_state();
    g_create_hook = &reenter_default_queue_init;

    CHECK(app::runtime::scheduling::init_default_work_queues());
    CHECK(!g_reentrant_init_result);
    CHECK_EQ(g_create_attempts, 2U);
    CHECK_EQ(g_successful_creates, 2U);
    CHECK(app::runtime::scheduling::init_default_work_queues());
    CHECK_EQ(g_create_attempts, 2U);

    CHECK(std::strcmp(g_tasks[0].name, "wq:hp_default") == 0);
    CHECK(std::strcmp(g_tasks[1].name, "wq:lp_default") == 0);
    CHECK_EQ(g_tasks[0].stack_depth * sizeof(StackType_t), 2048U);
    CHECK_EQ(g_tasks[1].stack_depth * sizeof(StackType_t), 2048U);
    CHECK_NE(g_tasks[0].stack, g_tasks[1].stack);
    CHECK_NE(g_tasks[0].task_buffer, g_tasks[1].task_buffer);
    CHECK(g_tasks[0].priority > g_tasks[1].priority);
    CHECK(g_tasks[0].function != nullptr);
    CHECK(g_tasks[1].function != nullptr);

    g_inside_interrupt = true;
    bool rejected_isr_call = false;
    try {
        (void)app::runtime::scheduling::init_default_work_queues();
    } catch (const std::runtime_error &) {
        rejected_isr_call = true;
    }
    g_inside_interrupt = false;
    CHECK(rejected_isr_call);
    CHECK_EQ(g_critical_depth, 0U);
}

HOST_TEST(freertos_backend_uses_production_nested_critical_and_rejects_schedule_from_isr)
{
    WorkQueue &queue = app::runtime::scheduling::hp_default_work_queue();
    BackendItem item{queue};
    g_max_critical_depth = 0U;

    CHECK(item.ScheduleNow());
    CHECK(g_max_critical_depth >= 2U);
    CHECK_EQ(g_critical_depth, 0U);
    item.ScheduleClear();
    CHECK_EQ(g_critical_depth, 0U);

    g_inside_interrupt = true;
    bool now_rejected = false;
    bool interval_rejected = false;
    bool clear_rejected = false;
    try {
        (void)item.ScheduleNow();
    } catch (const std::runtime_error &) {
        now_rejected = true;
    }
    try {
        (void)item.ScheduleOnInterval(1U);
    } catch (const std::runtime_error &) {
        interval_rejected = true;
    }
    try {
        item.ScheduleClear();
    } catch (const std::runtime_error &) {
        clear_rejected = true;
    }
    g_inside_interrupt = false;

    CHECK(now_rejected);
    CHECK(interval_rejected);
    CHECK(clear_rejected);
    CHECK_EQ(g_critical_depth, 0U);
}

HOST_TEST(freertos_backend_keeps_queue_storage_and_notifications_independent)
{
    WorkQueue &high = app::runtime::scheduling::hp_default_work_queue();
    WorkQueue &low = app::runtime::scheduling::lp_default_work_queue();
    BackendItem high_item{high};
    BackendItem low_item{low};
    const uint32_t high_notifications = g_tasks[0].notifications;
    const uint32_t low_notifications = g_tasks[1].notifications;

    CHECK(high_item.ScheduleNow());
    CHECK_EQ(g_tasks[0].notifications, high_notifications + 1U);
    CHECK_EQ(g_tasks[1].notifications, low_notifications);
    CHECK(low_item.ScheduleNow());
    CHECK_EQ(g_tasks[1].notifications, low_notifications + 1U);

    CHECK(run_one_step(QueueClass::HighPriority));
    CHECK_EQ(high_item.runs, 1U);
    CHECK_EQ(low_item.runs, 0U);
    CHECK(run_one_step(QueueClass::LowPriority));
    CHECK_EQ(low_item.runs, 1U);
}

HOST_TEST(freertos_backend_rotates_ready_scan_to_prevent_slot_starvation)
{
    WorkQueue &queue = app::runtime::scheduling::hp_default_work_queue();
    BackendItem continuously_due{queue, true};
    BackendItem peer{queue};

    CHECK(continuously_due.ScheduleNow());
    CHECK(peer.ScheduleNow());
    CHECK(run_one_step(QueueClass::HighPriority));
    CHECK(run_one_step(QueueClass::HighPriority));

    CHECK_EQ(continuously_due.runs, 1U);
    CHECK_EQ(peer.runs, 1U);
    continuously_due.ScheduleClear();
}

HOST_TEST(freertos_backend_notification_between_scan_and_wait_is_not_lost)
{
    WorkQueue &queue = app::runtime::scheduling::hp_default_work_queue();
    BackendItem item{queue};
    g_tasks[0].notifications = 0U;
    g_wait_hook_item = &item;
    g_before_wait_hook = &schedule_wait_hook_item;
    const unsigned wait_calls = g_notify_take_calls;

    CHECK(!run_one_step(QueueClass::HighPriority));
    CHECK_EQ(g_notify_take_calls, wait_calls + 1U);
    CHECK_EQ(g_tasks[0].notifications, 0U);
    CHECK_EQ(item.runs, 0U);
    CHECK(run_one_step(QueueClass::HighPriority));
    CHECK_EQ(item.runs, 1U);
    g_wait_hook_item = nullptr;
}

HOST_TEST(freertos_backend_selects_earliest_deadline_across_tick_wrap)
{
    WorkQueue &queue = app::runtime::scheduling::lp_default_work_queue();
    BackendItem later{queue};
    BackendItem earlier{queue};
    g_tasks[1].notifications = 0U;
    g_now_ms = 0xFFFFFFF0ULL;

    CHECK(later.ScheduleOnInterval(32U));
    CHECK(earlier.ScheduleOnInterval(16U));
    CHECK(!run_one_step(QueueClass::LowPriority));
    CHECK_EQ(g_last_wait_ticks, 16U);

    g_now_ms = 0x100000000ULL;
    CHECK(run_one_step(QueueClass::LowPriority));
    CHECK_EQ(earlier.runs, 1U);
    CHECK_EQ(later.runs, 0U);
    earlier.ScheduleClear();
    later.ScheduleClear();
}

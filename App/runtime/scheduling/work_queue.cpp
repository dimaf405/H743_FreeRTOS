#include "App/runtime/scheduling/work_queue.hpp"

#include "App/runtime/scheduling/scheduled_work_item.hpp"

#if !defined(APP_HOST_TEST)
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}
#endif

namespace app::runtime::scheduling {
namespace {

void state_enter_critical()
{
#if !defined(APP_HOST_TEST)
    taskENTER_CRITICAL();
#endif
}

void state_exit_critical()
{
#if !defined(APP_HOST_TEST)
    taskEXIT_CRITICAL();
#endif
}

void assert_task_context()
{
#if !defined(APP_HOST_TEST)
    configASSERT(xPortIsInsideInterrupt() == pdFALSE);
#endif
}

} // namespace

bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (now_ms - deadline_ms) < 0x80000000U;
}

bool WorkQueue::schedule_now(ScheduledWorkItem &item)
{
    assert_task_context();
    state_enter_critical();

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Pending &&
        !item.periodic_) {
        state_exit_critical();
        return true;
    }

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Claimed ||
        item.dispatch_state_ == ScheduledWorkItem::DispatchState::Running) {
        item.periodic_ = false;
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::Now;
        item.deferred_interval_ms_ = 0U;
        state_exit_critical();
        return true;
    }

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Pending) {
        backend_.clear(queue_class_, &item);
    } else {
        if (managed_count_ >= kMaxItems) {
            state_exit_critical();
            return false;
        }
        ++managed_count_;
    }

    item.periodic_ = false;
    item.interval_ms_ = 0U;
    item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
    item.deferred_interval_ms_ = 0U;
    item.cancel_current_ = false;
    const bool scheduled = enqueue_locked(item, clock_.now_ms());
    if (!scheduled) {
        release_locked(item);
    }

    state_exit_critical();
    return scheduled;
}

bool WorkQueue::schedule_interval(ScheduledWorkItem &item, uint32_t interval_ms)
{
    assert_task_context();
    if (interval_ms == 0U || interval_ms > 0x7FFFFFFFU) {
        return false;
    }

    state_enter_critical();

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Claimed ||
        item.dispatch_state_ == ScheduledWorkItem::DispatchState::Running) {
        item.periodic_ = false;
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::Interval;
        item.deferred_interval_ms_ = interval_ms;
        state_exit_critical();
        return true;
    }

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Pending) {
        backend_.clear(queue_class_, &item);
    } else {
        if (managed_count_ >= kMaxItems) {
            state_exit_critical();
            return false;
        }
        ++managed_count_;
    }

    item.periodic_ = true;
    item.interval_ms_ = interval_ms;
    item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
    item.deferred_interval_ms_ = 0U;
    item.cancel_current_ = false;
    const bool scheduled = enqueue_locked(item, clock_.now_ms() + interval_ms);
    if (!scheduled) {
        release_locked(item);
    }

    state_exit_critical();
    return scheduled;
}

bool WorkQueue::enqueue_locked(ScheduledWorkItem &item, uint32_t deadline_ms)
{
    item.dispatch_state_ = ScheduledWorkItem::DispatchState::Pending;
    item.deadline_ms_ = deadline_ms;
    item.cancel_current_ = false;
    return backend_.schedule(queue_class_, &item, &WorkQueue::claim_callback,
                             &WorkQueue::run_callback, deadline_ms);
}

void WorkQueue::release_locked(ScheduledWorkItem &item)
{
    item.dispatch_state_ = ScheduledWorkItem::DispatchState::Idle;
    item.interval_ms_ = 0U;
    item.deadline_ms_ = 0U;
    item.deferred_interval_ms_ = 0U;
    item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
    item.periodic_ = false;
    item.cancel_current_ = false;
    if (managed_count_ > 0U) {
        --managed_count_;
    }
}

void WorkQueue::finish_run_locked(ScheduledWorkItem &item)
{
    if (item.deferred_request_ == ScheduledWorkItem::DeferredRequest::Now) {
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
        item.deferred_interval_ms_ = 0U;
        item.periodic_ = false;
        if (!enqueue_locked(item, clock_.now_ms())) {
            release_locked(item);
        }
        return;
    }

    if (item.deferred_request_ == ScheduledWorkItem::DeferredRequest::Interval) {
        const uint32_t interval_ms = item.deferred_interval_ms_;
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
        item.deferred_interval_ms_ = 0U;
        item.periodic_ = true;
        item.interval_ms_ = interval_ms;
        if (!enqueue_locked(item, clock_.now_ms() + interval_ms)) {
            release_locked(item);
        }
        return;
    }

    if (!item.cancel_current_ && item.periodic_) {
        const uint32_t now_ms = clock_.now_ms();
        const uint32_t phase_elapsed_ms = now_ms - item.deadline_ms_;
        const uint32_t phase_remainder_ms =
            phase_elapsed_ms % item.interval_ms_;
        const uint32_t next_deadline_ms =
            now_ms + (item.interval_ms_ - phase_remainder_ms);
        if (!enqueue_locked(item, next_deadline_ms)) {
            release_locked(item);
        }
        return;
    }

    release_locked(item);
}

void WorkQueue::clear(ScheduledWorkItem &item)
{
    assert_task_context();
    state_enter_critical();

    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Pending) {
        backend_.clear(queue_class_, &item);
        release_locked(item);
    } else if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Claimed ||
               item.dispatch_state_ == ScheduledWorkItem::DispatchState::Running) {
        item.periodic_ = false;
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
        item.deferred_interval_ms_ = 0U;
        item.cancel_current_ = true;
    } else {
        item.periodic_ = false;
        item.interval_ms_ = 0U;
        item.deferred_request_ = ScheduledWorkItem::DeferredRequest::None;
        item.deferred_interval_ms_ = 0U;
    }

    state_exit_critical();
}

bool WorkQueue::claim(ScheduledWorkItem &item)
{
    state_enter_critical();
    const bool claimable =
        item.dispatch_state_ == ScheduledWorkItem::DispatchState::Pending;
    if (claimable) {
        item.dispatch_state_ = ScheduledWorkItem::DispatchState::Claimed;
        item.cancel_current_ = false;
    }
    state_exit_critical();
    return claimable;
}

void WorkQueue::run(ScheduledWorkItem &item)
{
    state_enter_critical();
    if (item.dispatch_state_ != ScheduledWorkItem::DispatchState::Claimed) {
        state_exit_critical();
        return;
    }

    const bool should_run = !item.cancel_current_;
    if (should_run) {
        item.dispatch_state_ = ScheduledWorkItem::DispatchState::Running;
    }
    state_exit_critical();

    if (should_run) {
        item.Run();
    }

    state_enter_critical();
    if (item.dispatch_state_ == ScheduledWorkItem::DispatchState::Claimed ||
        item.dispatch_state_ == ScheduledWorkItem::DispatchState::Running) {
        finish_run_locked(item);
    }
    state_exit_critical();
}

bool WorkQueue::claim_callback(void *context)
{
    auto &item = *static_cast<ScheduledWorkItem *>(context);
    return item.queue_.claim(item);
}

void WorkQueue::run_callback(void *context)
{
    auto &item = *static_cast<ScheduledWorkItem *>(context);
    item.queue_.run(item);
}

} // namespace app::runtime::scheduling

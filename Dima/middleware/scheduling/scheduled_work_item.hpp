#pragma once

#include "Dima/middleware/scheduling/work_queue.hpp"

#include <stdint.h>

namespace dima::middleware::scheduling {

class ScheduledWorkItem {
public:
    explicit constexpr ScheduledWorkItem(WorkQueue &queue) : queue_(queue) {}

    // Task context only. These calls are not ISR-safe. ScheduleClear updates
    // scheduling state but is not a quiescence barrier: a Run that already
    // crossed the Claimed-to-Running linearization point may still be active.
    bool ScheduleNow();
    bool ScheduleOnInterval(uint32_t interval_ms);
    void ScheduleClear();

protected:
    // Work items must outlive every queue callback. Production instances are
    // static, or their owner must establish a quiescent lifetime boundary.
    ~ScheduledWorkItem() = default;
    virtual void Run() = 0;

private:
    friend class WorkQueue;

    enum class DispatchState : uint8_t {
        Idle,
        Pending,
        Claimed,
        Running,
    };

    enum class DeferredRequest : uint8_t {
        None,
        Now,
        Interval,
    };

    WorkQueue &queue_;
    uint32_t interval_ms_{0U};
    uint32_t deadline_ms_{0U};
    uint32_t deferred_interval_ms_{0U};
    DispatchState dispatch_state_{DispatchState::Idle};
    DeferredRequest deferred_request_{DeferredRequest::None};
    bool periodic_{false};
    bool cancel_current_{false};
};

} // namespace dima::middleware::scheduling

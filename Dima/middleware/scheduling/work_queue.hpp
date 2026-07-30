#pragma once

#include <stdint.h>

namespace dima::middleware::scheduling {

class ScheduledWorkItem;

enum class QueueClass : uint8_t {
    HighPriority,
    LowPriority,
};

class IClock {
public:
    virtual uint32_t now_ms() const = 0;

protected:
    constexpr IClock() = default;
    ~IClock() = default;
};

class IWorkQueueBackend {
public:
    using ClaimCallback = bool (*)(void *context);
    using RunCallback = void (*)(void *context);

    virtual bool schedule(QueueClass queue, void *context, ClaimCallback claim,
                          RunCallback run,
                          uint32_t deadline_ms) = 0;
    virtual void clear(QueueClass queue, void *context) = 0;

protected:
    constexpr IWorkQueueBackend() = default;
    ~IWorkQueueBackend() = default;
};

bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms);

class WorkQueue {
public:
    static constexpr uint8_t kMaxItems = 16U;

    constexpr WorkQueue(QueueClass queue_class, IClock &clock,
                        IWorkQueueBackend &backend)
        : queue_class_(queue_class), clock_(clock), backend_(backend)
    {
    }

private:
    friend class ScheduledWorkItem;

    bool schedule_now(ScheduledWorkItem &item);
    bool schedule_interval(ScheduledWorkItem &item, uint32_t interval_ms);
    bool enqueue_locked(ScheduledWorkItem &item, uint32_t deadline_ms);
    void release_locked(ScheduledWorkItem &item);
    void finish_run_locked(ScheduledWorkItem &item);
    void clear(ScheduledWorkItem &item);
    bool claim(ScheduledWorkItem &item);
    void run(ScheduledWorkItem &item);
    static bool claim_callback(void *context);
    static void run_callback(void *context);

    QueueClass queue_class_;
    IClock &clock_;
    IWorkQueueBackend &backend_;
    uint8_t managed_count_{0U};
};

} // namespace dima::middleware::scheduling

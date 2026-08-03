#pragma once

#include <stddef.h>
#include <stdint.h>

namespace px4 {

using hrt_abstime = uint64_t;

struct wq_config_t {
    const char *name;
    uint8_t priority;
    uint16_t stack_size;
    bool realtime;
};

struct WorkQueueStats {
    uint32_t executions;
    hrt_abstime last_execution_time;
    hrt_abstime maximum_execution_time;
    uint32_t deadline_misses;
};

namespace wq_configurations {
extern const wq_config_t estimator;
extern const wq_config_t rate_ctrl;
extern const wq_config_t sensors;
extern const wq_config_t io;
extern const wq_config_t nav;
extern const wq_config_t hp_default;
extern const wq_config_t lp_default;
} // namespace wq_configurations

class WorkItem {
public:
    WorkItem(const char *name, const wq_config_t &config) noexcept;
    virtual ~WorkItem() = default;

    bool ScheduleNow() noexcept;
    bool ScheduleNowFromISR() noexcept;
    void ScheduleClear() noexcept;
    const char *Name() const noexcept { return name_; }
    const WorkQueueStats &statistics() const noexcept { return statistics_; }

protected:
    virtual void Run() = 0;

private:
    friend class WorkQueueManager;
    const char *name_;
    const wq_config_t *config_;
    volatile uint32_t schedule_revision_{0U};
    hrt_abstime deadline_{0U};
    hrt_abstime interval_{0U};
    bool scheduled_{false};
    bool running_{false};
    WorkQueueStats statistics_{};
};

class ScheduledWorkItem : public WorkItem {
public:
    using WorkItem::WorkItem;

    bool ScheduleDelayed(uint32_t delay_us) noexcept;
    bool ScheduleOnInterval(uint32_t interval_us,
                            uint32_t delay_us = 0U) noexcept;
    bool ScheduleAt(hrt_abstime time_us) noexcept;
};

bool work_queue_init() noexcept;
void work_queue_shutdown() noexcept;
hrt_abstime work_queue_time_us() noexcept;

} // namespace px4





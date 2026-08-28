#pragma once

#include <stddef.h>
#include <stdint.h>

#include "api/PlatformTypes.hpp"

namespace px4 {

using hrt_abstime = dima::platform::TimeUs;

struct wq_config_t {
    /* stack_size 单位为字节；realtime=true 会使平台拒绝该任务中的动态分配。 */
    const char *name;
    uint8_t priority;
    uint16_t stack_size;
    bool realtime;
};

struct WorkQueueStats {
    /* 执行时间和 deadline 均为单调微秒；deadline_misses 只统计实际开始晚于截止。 */
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
extern const wq_config_t storage;
} // namespace wq_configurations

class WorkItem {
public:
    WorkItem(const char *name, const wq_config_t &config) noexcept;
    virtual ~WorkItem() = default;

    bool ScheduleNow() noexcept;
    bool ScheduleNowFromISR() noexcept;
    bool ScheduleEnable() noexcept;
    void ScheduleClear() noexcept;
    /* 取消未来调度并等待在途 Run 退出；从自身 Run 调用时只取消、不自等待。 */
    void ScheduleCancelAndDrain() noexcept;
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
    bool accepting_schedules_{true};
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
bool work_queue_shutdown() noexcept;
hrt_abstime work_queue_time_us() noexcept;

} // namespace px4





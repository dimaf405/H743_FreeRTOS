#pragma once

#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"

namespace dima::product::rover {

class LogService final : public px4::ScheduledWorkItem {
public:
    LogService() noexcept;

    bool start() noexcept;
    void stop() noexcept;

protected:
    void Run() override;

private:
    static constexpr uint32_t kFlushIntervalUs = 20000U;
    bool started_{false};
};

} // namespace dima::product::rover

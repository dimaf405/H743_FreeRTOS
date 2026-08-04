#pragma once

#include "lifecycle/module_base.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

namespace dima::modules::logging {

class LogService final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    LogService() noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

protected:
    void Run() override;

private:
    static constexpr uint32_t kFlushIntervalUs = 20000U;
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::logging

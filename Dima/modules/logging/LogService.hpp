#pragma once

#include "lifecycle/module_base.hpp"
#include "platform/api/Platform.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

namespace dima::modules::logging {

class LogService final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    explicit LogService(dima::platform::Console &console) noexcept;

    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

protected:
    void Run() override;

private:
    dima::platform::Console &console_;
    static constexpr uint32_t kFlushIntervalUs = 20000U;
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
};

} // namespace dima::modules::logging

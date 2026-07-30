#pragma once

#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"

#include "Dima/middleware/uorb/Publication.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"

#include <stdint.h>

namespace dima::modules::hello_world {

class HelloWorld final : public dima::middleware::lifecycle::ModuleBase,
                         public px4::ScheduledWorkItem {
public:
    HelloWorld() noexcept;
    ~HelloWorld() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

protected:
    void Run() override;

private:
    uORB::Publication<app_heartbeat_s> heartbeat_publication_;
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    uint32_t sequence_{0U};
};

} // namespace dima::modules::hello_world

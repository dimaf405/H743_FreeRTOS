#pragma once

#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"

#include "Dima/middleware/uorb/SubscriptionData.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"

#include <stdint.h>

namespace dima::modules::boot_health {

class BootHealthService final : public dima::middleware::lifecycle::ModuleBase,
                                public px4::ScheduledWorkItem {
public:
    BootHealthService() noexcept;
    ~BootHealthService() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

protected:
    void Run() override;

private:
    using TimeMsFunction = uint64_t (*)(void *context);
    using ConfirmFunction = int (*)(void *context);

    static constexpr uint32_t kCheckIntervalMs = 100U;
    static constexpr uint64_t kStableWindowMs = 5000ULL;

    uORB::SubscriptionData<app_heartbeat_s> heartbeat_subscription_;
    void *dependency_context_{nullptr};
    TimeMsFunction time_ms_{nullptr};
    ConfirmFunction confirm_running_image_{nullptr};
    uint64_t stable_window_start_ms_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool heartbeat_observed_{false};
    bool confirmation_attempted_{false};
};

} // namespace dima::modules::boot_health

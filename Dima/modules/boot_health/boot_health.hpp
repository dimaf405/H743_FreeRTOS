#pragma once

#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/lifecycle/module_base.hpp"

#if defined(APP_HOST_TEST)
#include "Dima/middleware/messaging/topic.hpp"
#include "Dima/middleware/scheduling/scheduled_work_item.hpp"
#else
#include "Dima/middleware/uorb/SubscriptionData.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"
#endif

#include <stdint.h>

namespace dima::modules::boot_health {

#if defined(APP_HOST_TEST)
struct HostDependencies {
    void *context;
    uint64_t (*time_ms)(void *context);
    int (*confirm_running_image)(void *context);
};
#endif

class BootHealthService final : public dima::middleware::lifecycle::ModuleBase,
#if defined(APP_HOST_TEST)
                                public dima::middleware::scheduling::ScheduledWorkItem {
#else
                                public px4::ScheduledWorkItem {
#endif
public:
#if defined(APP_HOST_TEST)
    BootHealthService(
        dima::middleware::scheduling::WorkQueue &queue,
        dima::middleware::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
        HostDependencies dependencies);
#else
    BootHealthService() noexcept;
#endif
    ~BootHealthService() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

#if defined(APP_HOST_TEST)
    void RunForTest();
#endif

protected:
    void Run() override;

private:
    using TimeMsFunction = uint64_t (*)(void *context);
    using ConfirmFunction = int (*)(void *context);

    static constexpr uint32_t kCheckIntervalMs = 100U;
    static constexpr uint64_t kStableWindowMs = 5000ULL;

#if defined(APP_HOST_TEST)
    dima::middleware::messaging::Subscription<app_heartbeat_s> heartbeat_subscription_;
#else
    uORB::SubscriptionData<app_heartbeat_s> heartbeat_subscription_;
#endif
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

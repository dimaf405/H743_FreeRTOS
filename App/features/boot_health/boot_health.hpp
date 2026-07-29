#pragma once

#include "App/messages/app_heartbeat.hpp"
#include "App/runtime/lifecycle/module_base.hpp"

#if defined(APP_HOST_TEST)
#include "App/runtime/messaging/topic.hpp"
#include "App/runtime/scheduling/scheduled_work_item.hpp"
#else
#include "Dima/messages/app_heartbeat.hpp"
#include "Dima/middleware/uorb/SubscriptionData.hpp"
#include "Dima/middleware/work_queue/ScheduledWorkItem.hpp"
#endif

#include <stdint.h>

namespace app::features::boot_health {

#if defined(APP_HOST_TEST)
struct HostDependencies {
    void *context;
    uint64_t (*time_ms)(void *context);
    int (*confirm_running_image)(void *context);
};
#endif

class BootHealthService final : public runtime::lifecycle::ModuleBase,
#if defined(APP_HOST_TEST)
                                public runtime::scheduling::ScheduledWorkItem {
#else
                                public px4::ScheduledWorkItem {
#endif
public:
#if defined(APP_HOST_TEST)
    BootHealthService(
        runtime::scheduling::WorkQueue &queue,
        runtime::messaging::Topic<app_heartbeat_s> &heartbeat_topic,
        HostDependencies dependencies);
#else
    BootHealthService() noexcept;
#endif
    ~BootHealthService() = default;

    bool start() override;
    void stop() override;
    runtime::lifecycle::ModuleState state() const override;

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
    runtime::messaging::Subscription<app_heartbeat_s> heartbeat_subscription_;
#else
    uORB::SubscriptionData<app_heartbeat_s> heartbeat_subscription_;
#endif
    void *dependency_context_{nullptr};
    TimeMsFunction time_ms_{nullptr};
    ConfirmFunction confirm_running_image_{nullptr};
    uint64_t stable_window_start_ms_{0U};
    runtime::lifecycle::ModuleState state_{
        runtime::lifecycle::ModuleState::Stopped};
    bool heartbeat_observed_{false};
    bool confirmation_attempted_{false};
};

} // namespace app::features::boot_health

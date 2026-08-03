#pragma once

#include "actuator_armed.hpp"
#include "app_heartbeat.hpp"
#include "lifecycle/module_base.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"

#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

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
    void bind_commander(
        const dima::middleware::lifecycle::ModuleBase &commander) noexcept;

protected:
    void Run() override;

private:
    using TimeMsFunction = uint64_t (*)(void *context);
    using ConfirmFunction = int (*)(void *context);

    static constexpr uint32_t kCheckIntervalMs = 100U;
    static constexpr uint64_t kStableWindowMs = 5000ULL;
    static constexpr uint64_t kHeartbeatTimeoutMs = 1500ULL;
    static constexpr uint64_t kSafetyTopicTimeoutUs = 750000ULL;

    uORB::SubscriptionData<app_heartbeat_s> heartbeat_subscription_;
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        vehicle_control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    const dima::middleware::lifecycle::ModuleBase *commander_{nullptr};
    void *dependency_context_{nullptr};
    TimeMsFunction time_ms_{nullptr};
    ConfirmFunction confirm_running_image_{nullptr};
    uint64_t stable_window_start_ms_{0U};
    uint64_t last_heartbeat_progress_ms_{0U};
    uint64_t last_heartbeat_timestamp_us_{0U};
    uint32_t last_heartbeat_sequence_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool heartbeat_observed_{false};
    bool stable_window_active_{false};
    bool confirmation_attempted_{false};

    bool update_heartbeat_health(uint64_t now_ms, uint64_t now_us) noexcept;
    bool safety_topics_consistent(uint64_t now_us) const noexcept;
    void reset_stable_window(uint64_t now_ms) noexcept;
};

} // namespace dima::modules::boot_health

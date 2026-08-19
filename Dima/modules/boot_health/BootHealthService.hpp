#pragma once

#include "actuator_armed.hpp"
#include "actuator_output_status.hpp"
#include "lifecycle/module_base.hpp"
#include "platform/api/Platform.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"

#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::modules::boot_health {

class BootHealthService final : public dima::middleware::lifecycle::ModuleBase,
                                public px4::ScheduledWorkItem {
public:
    BootHealthService(dima::platform::BootControl &boot_control,
                      dima::platform::MonotonicClock &clock) noexcept;
    ~BootHealthService() = default;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;
    std::uint32_t health_generation() const noexcept;
    void bind_commander(
        const dima::middleware::lifecycle::ModuleBase &commander) noexcept;
    void bind_motor_output(
        const dima::middleware::lifecycle::ModuleBase &motor_output) noexcept;

protected:
    void Run() override;

private:
    static constexpr std::uint32_t kCheckIntervalMs = 100U;
    static constexpr std::uint64_t kStableWindowMs = 5000ULL;
    static constexpr std::uint64_t kSafetyTopicTimeoutUs = 750000ULL;
    static constexpr std::uint64_t kOutputStatusTimeoutUs = 250000ULL;
    static constexpr std::uint64_t kActuatorArmTransitionUs = 250000ULL;

    dima::platform::BootControl &boot_control_;
    dima::platform::MonotonicClock &clock_;
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        vehicle_control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::SubscriptionData<actuator_output_status_s>
        actuator_output_status_subscription_{ORB_ID(actuator_output_status)};
    const dima::middleware::lifecycle::ModuleBase *commander_{nullptr};
    const dima::middleware::lifecycle::ModuleBase *motor_output_{nullptr};
    std::uint64_t stable_window_start_ms_{0U};
    std::uint64_t last_safety_timestamp_us_{0U};
    std::uint32_t last_output_sequence_{0U};
    std::uint32_t health_generation_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool safety_snapshot_observed_{false};
    bool output_snapshot_observed_{false};
    bool stable_window_active_{false};
    bool confirmation_attempted_{false};

    bool update_safety_health(std::uint64_t now_us) noexcept;
    bool update_output_health(std::uint64_t now_us) noexcept;
    bool safety_topics_consistent(std::uint64_t now_us) const noexcept;
    bool output_status_runtime_healthy(std::uint64_t now_us) const noexcept;
    bool output_status_confirmation_safe() const noexcept;
    bool output_mapping_valid(const actuator_output_status_s &output) const noexcept;
    bool output_frame_valid(const actuator_output_status_s &output) const noexcept;
    bool confirmation_state_safe() const noexcept;
    void reset_stable_window(std::uint64_t now_ms) noexcept;
};

} // namespace dima::modules::boot_health

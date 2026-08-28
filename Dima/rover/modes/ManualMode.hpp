#pragma once

#include "manual_control_setpoint.hpp"
#include "parameter_update.hpp"
#include "rover_motion_request.hpp"
#include "vehicle_control_mode.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "uORB/Publication.hpp"
#include "uORB/uORB.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::rover::modes {

/** Owns the Rover Manual mode and publishes its two-axis motion request. */
class ManualMode final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    ManualMode() noexcept;
    ~ManualMode() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    static constexpr std::uint64_t kControlModeTimeoutUs = 750000ULL;

    struct Config {
        float yaw_stick_deadzone;
        float yaw_expo;
        float yaw_superexpo;
        float yaw_stick_gain;
    };

    void Run() override;
    bool bind_parameters() noexcept;
    void invalidate_parameter_bindings() noexcept;
    bool apply_parameter_snapshot() noexcept;
    bool apply_pending_parameters(std::uint64_t now_us) noexcept;
    bool fresh_disarmed_mode(std::uint64_t now_us) const noexcept;
    bool manual_mode_active(std::uint64_t now_us) const noexcept;
    bool manual_input_valid(std::uint64_t now_us) const noexcept;
    bool publish_current_request(std::uint64_t now_us) noexcept;
    void reset_runtime_state() noexcept;
    void enter_error(std::uint32_t event_id) noexcept;

    static bool finite(float value) noexcept;
    static float clamp(float value, float lower, float upper) noexcept;
    static float deadzone(float value, float width) noexcept;
    static float superexpo(float value, float expo, float superexpo) noexcept;
    static bool valid_config(const Config &config) noexcept;

    uORB::SubscriptionCallbackWorkItem manual_control_subscription_{
        ORB_ID(manual_control_setpoint), *this};
    uORB::SubscriptionCallbackWorkItem vehicle_control_mode_subscription_{
        ORB_ID(vehicle_control_mode), *this};
    uORB::SubscriptionCallbackWorkItem parameter_update_subscription_{
        ORB_ID(parameter_update), *this};
    uORB::Publication<rover_motion_request_s> motion_request_publication_{
        ORB_ID(rover_motion_request)};

    dima::ParamFloat<dima::params::RO_YAW_STICK_DZ> yaw_stick_deadzone_{};
    dima::ParamFloat<dima::params::RO_YAW_EXPO> yaw_expo_{};
    dima::ParamFloat<dima::params::RO_YAW_SUPEXPO> yaw_superexpo_{};
    dima::ParamFloat<dima::params::RD_YAW_STK_GAIN> yaw_stick_gain_{};

    Config config_{};
    manual_control_setpoint_s manual_control_{};
    vehicle_control_mode_s vehicle_control_mode_{};
    std::uint32_t sequence_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool have_manual_control_{false};
    bool have_control_mode_{false};
    bool parameters_valid_{false};
    bool parameter_update_pending_{false};
};

} // namespace dima::rover::modes

#pragma once

#include "actuator_armed.hpp"
#include "actuator_motors.hpp"
#include "actuator_output_status.hpp"
#include "parameter_update.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "platform/api/ActuatorPwm.hpp"
#include "uorb/Publication.hpp"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::motor {

/** Safety-gated mapping from two reversible motors to six ordinary PWM pins. */
class MotorOutput final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    explicit MotorOutput(dima::platform::ActuatorPwm *pwm) noexcept;
    ~MotorOutput() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

    bool safe_off_confirmed() const noexcept;

private:
    static constexpr std::uint32_t kRunIntervalUs = 10000U;
    static constexpr std::uint64_t kSafetyTopicTimeoutUs = 750000ULL;
    static constexpr std::size_t kChannelCount =
        dima::platform::kActuatorPwmChannelCount;
    static constexpr std::size_t kFieldsPerChannel = 5U;

    enum class ChannelFunction : std::int32_t {
        Disabled = 0,
        MotorRight = 101,
        MotorLeft = 102,
    };

    struct ChannelConfig {
        ChannelFunction function;
        std::uint16_t minimum_us;
        std::uint16_t center_us;
        std::uint16_t maximum_us;
        bool reversed;
    };

    struct ParameterSnapshot {
        float command_timeout_s;
        ChannelConfig channels[kChannelCount];
        std::uint8_t configured_mask;
        std::uint8_t right_mask;
        std::uint8_t left_mask;
        bool drive_available;
    };

    struct SafetySnapshot {
        actuator_armed_s actuator_armed;
        vehicle_control_mode_s control_mode;
        vehicle_status_s vehicle_status;
        bool valid;
    };

    void Run() override;
    bool bind_parameters() noexcept;
    void invalidate_parameter_bindings() noexcept;
    bool apply_parameter_snapshot() noexcept;
    bool fresh_disarmed_snapshot(std::uint64_t now_us) const noexcept;
    void refresh_safety_snapshot(std::uint64_t now_us) noexcept;
    bool observed_snapshot_complete(std::uint64_t now_us) const noexcept;
    bool active_snapshot_fresh(std::uint64_t now_us) const noexcept;
    bool safety_permits_output(std::uint64_t now_us) const noexcept;
    bool safety_permits_disarmed_neutral(
        std::uint64_t now_us) const noexcept;
    bool motor_command_valid(std::uint64_t now_us) const noexcept;
    bool build_frame(dima::platform::ActuatorPwmFrame &frame) const noexcept;
    bool build_neutral_frame(
        dima::platform::ActuatorPwmFrame &frame) const noexcept;
    dima::platform::ActuatorPwmResult apply_frame(
        const dima::platform::ActuatorPwmFrame &frame) noexcept;
    dima::platform::ActuatorPwmResult force_safe_off() noexcept;
    bool publish_status(std::uint64_t now_us, std::uint8_t output_state,
                        bool command_valid) noexcept;
    bool enter_parameter_safe_off() noexcept;
    bool drive_available() const noexcept;
    void reset_runtime_state() noexcept;
    void enter_error(std::uint32_t event_id) noexcept;

    static bool finite(float value) noexcept;
    static bool normalized(float value) noexcept;
    static std::uint16_t map_normalized(const ChannelConfig &channel,
                                        float value) noexcept;
    static bool safety_negative(const actuator_armed_s &armed) noexcept;
    static bool safety_negative(const vehicle_control_mode_s &control) noexcept;
    static bool safety_negative(const vehicle_status_s &status) noexcept;
    static bool hard_safe_negative(const actuator_armed_s &armed) noexcept;
    static bool hard_safe_negative(
        const vehicle_control_mode_s &control) noexcept;
    static bool hard_safe_negative(const vehicle_status_s &status) noexcept;

    dima::platform::ActuatorPwm *pwm_{nullptr};
    uORB::SubscriptionData<actuator_motors_s> actuator_motors_subscription_{
        ORB_ID(actuator_motors)};
    uORB::Subscription parameter_update_subscription_{ORB_ID(parameter_update)};
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        vehicle_control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::Publication<actuator_output_status_s> output_status_publication_{
        ORB_ID(actuator_output_status)};

    px4::ParamFloat<px4::params::COM_ACT_LOSS_T> command_timeout_{};
    param_t parameter_handles_[kChannelCount][kFieldsPerChannel]{};
    ParameterSnapshot parameters_{};
    actuator_motors_s actuator_motors_{};
    actuator_armed_s observed_actuator_armed_{};
    vehicle_control_mode_s observed_control_mode_{};
    vehicle_status_s observed_vehicle_status_{};
    SafetySnapshot safety_{};
    dima::platform::ActuatorPwmFrame applied_frame_{};
    std::uint32_t status_sequence_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool have_motor_command_{false};
    bool parameters_valid_{false};
    bool parameter_update_pending_{false};
    bool safety_inhibit_observed_{true};
    bool hard_safe_inhibit_observed_{true};
    bool backend_ready_{false};
    bool safe_off_{false};
};

} // namespace dima::modules::motor

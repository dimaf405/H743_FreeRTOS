#pragma once

#include "rover/DifferentialDrive.hpp"

#include "actuator_armed.hpp"
#include "actuator_motors.hpp"
#include "parameter_update.hpp"
#include "rover_motion_request.hpp"
#include "vehicle_control_mode.hpp"
#include "vehicle_status.hpp"
#include "lifecycle/module_base.hpp"
#include "parameters/param.h"
#include "uorb/Publication.hpp"
#include "uorb/SubscriptionData.hpp"
#include "work_queue/ScheduledWorkItem.hpp"

#include <cstdint>

namespace dima::rover::control {

/** 100 Hz Rover two-axis command validator and differential-drive producer. */
class RoverDifferential final
    : public dima::middleware::lifecycle::ModuleBase,
      public px4::ScheduledWorkItem {
public:
    RoverDifferential() noexcept;
    ~RoverDifferential() override;

    bool start() override;
    void stop() override;
    dima::middleware::lifecycle::ModuleState state() const override;

private:
    static constexpr std::uint32_t kRunIntervalUs = 10000U;
    static constexpr std::uint64_t kSafetyTopicTimeoutUs = 750000ULL;
    static constexpr std::uint8_t kMotionRequestQueueDepth = 8U;

    struct ParameterSnapshot {
        float command_timeout_s;
        dima::lib::rover::DifferentialDriveConfig drive;
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
    bool apply_pending_parameters(std::uint64_t now_us) noexcept;
    void refresh_safety_snapshot(std::uint64_t now_us) noexcept;
    bool observed_snapshot_complete(std::uint64_t now_us) const noexcept;
    bool active_snapshot_fresh(std::uint64_t now_us) const noexcept;
    bool fresh_disarmed_snapshot(std::uint64_t now_us) const noexcept;
    bool safety_permits_output(std::uint64_t now_us) const noexcept;
    bool request_valid(std::uint64_t now_us) const noexcept;
    bool publish_output(std::uint64_t now_us, float dt_s) noexcept;
    bool publish_invalid(std::uint64_t now_us,
                         std::uint64_t sample_time_us) noexcept;
    void reset_runtime_state() noexcept;
    void enter_error(std::uint32_t event_id) noexcept;

    static bool finite(float value) noexcept;
    static bool normalized(float value) noexcept;
    static bool valid_parameter_snapshot(
        const ParameterSnapshot &snapshot) noexcept;
    static bool safety_negative(const actuator_armed_s &armed) noexcept;
    static bool safety_negative(const vehicle_control_mode_s &control) noexcept;
    static bool safety_negative(const vehicle_status_s &status) noexcept;

    uORB::Subscription motion_request_subscription_{
        ORB_ID(rover_motion_request)};
    uORB::Subscription parameter_update_subscription_{ORB_ID(parameter_update)};
    uORB::SubscriptionData<actuator_armed_s> actuator_armed_subscription_{
        ORB_ID(actuator_armed)};
    uORB::SubscriptionData<vehicle_control_mode_s>
        vehicle_control_mode_subscription_{ORB_ID(vehicle_control_mode)};
    uORB::SubscriptionData<vehicle_status_s> vehicle_status_subscription_{
        ORB_ID(vehicle_status)};
    uORB::Publication<actuator_motors_s> actuator_motors_publication_{
        ORB_ID(actuator_motors)};

    px4::ParamFloat<px4::params::RO_CMD_TIMEOUT> command_timeout_{};
    px4::ParamInt<px4::params::RD_REV_STEER> reverse_steering_{};
    px4::ParamFloat<px4::params::RD_STR_THR_MIX> steering_throttle_mix_{};
    px4::ParamFloat<px4::params::MOT_THR_MIN> throttle_min_{};
    px4::ParamFloat<px4::params::MOT_THR_MAX> throttle_max_{};
    px4::ParamFloat<px4::params::MOT_SLEW_RATE> throttle_slew_rate_{};
    px4::ParamFloat<px4::params::MOT_REV_DELAY> reversal_delay_{};
    px4::ParamFloat<px4::params::MOT_THR_EXPO> throttle_expo_{};
    px4::ParamFloat<px4::params::MOT_THR_ASYM> thrust_asymmetry_{};
    px4::ParamFloat<px4::params::MOT_ARM_RAMP> arm_ramp_{};

    dima::lib::rover::DifferentialDrive drive_{};
    ParameterSnapshot parameters_{};
    rover_motion_request_s motion_request_{};
    actuator_armed_s observed_actuator_armed_{};
    vehicle_control_mode_s observed_control_mode_{};
    vehicle_status_s observed_vehicle_status_{};
    SafetySnapshot safety_{};
    std::uint64_t last_run_time_us_{0U};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool have_motion_request_{false};
    bool parameters_valid_{false};
    bool parameter_update_pending_{false};
    bool safety_inhibit_observed_{true};
};

} // namespace dima::rover::control

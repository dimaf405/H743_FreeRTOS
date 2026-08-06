#include "MotorOutput.hpp"

#include "events/events.hpp"
#include "platform/api/Time.hpp"

#include <cmath>

namespace dima::modules::motor {
namespace {

constexpr std::uint32_t kEventParameterInvalid = 0x524D4F01U;
constexpr std::uint32_t kEventBackendFault = 0x524D4F02U;
constexpr std::uint32_t kEventPublishFailure = 0x524D4F03U;
constexpr std::uint32_t kEventScheduleFailure = 0x524D4F04U;
constexpr std::uint16_t kRequiredReversibleMask = 0x0003U;

enum ParameterField : std::size_t {
    Function = 0U,
    Minimum,
    Center,
    Maximum,
    Reversed,
};

constexpr const char *kParameterNames[6][5] = {
    {"PWM_S1_FUNC", "PWM_S1_MIN", "PWM_S1_CENT", "PWM_S1_MAX",
     "PWM_S1_REV"},
    {"PWM_S2_FUNC", "PWM_S2_MIN", "PWM_S2_CENT", "PWM_S2_MAX",
     "PWM_S2_REV"},
    {"PWM_S3_FUNC", "PWM_S3_MIN", "PWM_S3_CENT", "PWM_S3_MAX",
     "PWM_S3_REV"},
    {"PWM_S4_FUNC", "PWM_S4_MIN", "PWM_S4_CENT", "PWM_S4_MAX",
     "PWM_S4_REV"},
    {"PWM_S5_FUNC", "PWM_S5_MIN", "PWM_S5_CENT", "PWM_S5_MAX",
     "PWM_S5_REV"},
    {"PWM_S6_FUNC", "PWM_S6_MIN", "PWM_S6_CENT", "PWM_S6_MAX",
     "PWM_S6_REV"},
};

static_assert(dima::platform::kActuatorPwmChannelCount ==
              actuator_output_status_s::NUM_OUTPUTS);

} // namespace

MotorOutput::MotorOutput(dima::platform::ActuatorPwm *pwm) noexcept
    : px4::ScheduledWorkItem("motor_output", px4::wq_configurations::io),
      pwm_(pwm)
{
    invalidate_parameter_bindings();
}

MotorOutput::~MotorOutput()
{
    stop();
}

bool MotorOutput::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    reset_runtime_state();
    if (pwm_ == nullptr) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    const dima::platform::ActuatorPwmResult stopped = pwm_->stop();
    if (stopped != dima::platform::ActuatorPwmResult::Applied) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    backend_ready_ = true;
    safe_off_ = true;
    if (!ScheduleEnable()) {
        enter_error(kEventScheduleFailure);
        return false;
    }

    if (!bind_parameters()) {
        enter_error(kEventParameterInvalid);
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    const std::uint64_t now = hrt_absolute_time();
    if (!publish_status(now, actuator_output_status_s::STATE_SAFE_OFF, false)) {
        enter_error(kEventPublishFailure);
        return false;
    }
    if (!ScheduleOnInterval(kRunIntervalUs, 1U)) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    return true;
}

void MotorOutput::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();

    const dima::platform::ActuatorPwmResult result =
        pwm_ != nullptr ? pwm_->stop()
                        : dima::platform::ActuatorPwmResult::Fault;
    backend_ready_ = result == dima::platform::ActuatorPwmResult::Applied;
    safe_off_ = backend_ready_;
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    invalidate_parameter_bindings();
    if (!backend_ready_) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
    }
    (void)publish_status(hrt_absolute_time(),
                         backend_ready_
                             ? actuator_output_status_s::STATE_STOPPED
                             : actuator_output_status_s::STATE_FAULT,
                         false);
}

dima::middleware::lifecycle::ModuleState MotorOutput::state() const
{
    return state_;
}

bool MotorOutput::safe_off_confirmed() const noexcept
{
    return backend_ready_ && safe_off_ &&
           (pwm_ == nullptr || !pwm_->started());
}

bool MotorOutput::backend_ready() const noexcept { return backend_ready_; }

bool MotorOutput::drive_available() const noexcept
{
    return parameters_valid_ && parameters_.drive_available;
}

void MotorOutput::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s update{};
    if (parameter_update_subscription_.copy(&update)) {
        parameter_update_pending_ = true;
    }
    if (actuator_motors_subscription_.update()) {
        actuator_motors_ = actuator_motors_subscription_.get();
        have_motor_command_ = true;
    }

    const std::uint64_t now = hrt_absolute_time();
    refresh_safety_snapshot(now);
    if (parameter_update_pending_ && fresh_disarmed_snapshot(now)) {
        parameter_update_pending_ = false;
        if (!apply_parameter_snapshot()) {
            enter_error(kEventParameterInvalid);
            return;
        }
    }

    const bool command_valid = motor_command_valid(now);
    if (!parameters_valid_ || !parameters_.drive_available ||
        !safety_permits_output(now) || !command_valid) {
        const dima::platform::ActuatorPwmResult result = force_safe_off();
        if (result == dima::platform::ActuatorPwmResult::Fault) {
            enter_error(kEventBackendFault);
            return;
        }
        const std::uint8_t output_state =
            result == dima::platform::ActuatorPwmResult::Applied
                ? actuator_output_status_s::STATE_SAFE_OFF
                : actuator_output_status_s::STATE_RETRY;
        if (!publish_status(now, output_state, command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    dima::platform::ActuatorPwmFrame frame{};
    if (!build_frame(frame)) {
        enter_error(kEventParameterInvalid);
        return;
    }

    if (!pwm_->started()) {
        const dima::platform::ActuatorPwmResult start_result = pwm_->start();
        if (start_result == dima::platform::ActuatorPwmResult::Fault) {
            enter_error(kEventBackendFault);
            return;
        }
        if (start_result == dima::platform::ActuatorPwmResult::Retry) {
            const dima::platform::ActuatorPwmResult stopped = force_safe_off();
            if (stopped == dima::platform::ActuatorPwmResult::Fault) {
                enter_error(kEventBackendFault);
                return;
            }
            if (!publish_status(now, actuator_output_status_s::STATE_RETRY,
                                command_valid)) {
                enter_error(kEventPublishFailure);
            }
            return;
        }
    }

    const dima::platform::ActuatorPwmResult write_result = pwm_->write(frame);
    if (write_result == dima::platform::ActuatorPwmResult::Applied) {
        applied_frame_ = frame;
        backend_ready_ = true;
        safe_off_ = false;
        if (!publish_status(now, actuator_output_status_s::STATE_ACTIVE,
                            command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }
    if (write_result == dima::platform::ActuatorPwmResult::Retry) {
        const dima::platform::ActuatorPwmResult stopped = force_safe_off();
        if (stopped == dima::platform::ActuatorPwmResult::Fault) {
            enter_error(kEventBackendFault);
            return;
        }
        if (!publish_status(now, actuator_output_status_s::STATE_RETRY,
                            command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }
    enter_error(kEventBackendFault);
}

bool MotorOutput::bind_parameters() noexcept
{
    invalidate_parameter_bindings();
    if (!command_timeout_.bind()) {
        return false;
    }
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        for (std::size_t field = 0U; field < kFieldsPerChannel; ++field) {
            const param_t handle =
                param_find_no_notification(kParameterNames[channel][field]);
            if (handle == PARAM_INVALID) {
                invalidate_parameter_bindings();
                return false;
            }
            param_set_used(handle);
            parameter_handles_[channel][field] = handle;
        }
    }
    return apply_parameter_snapshot();
}

void MotorOutput::invalidate_parameter_bindings() noexcept
{
    command_timeout_.invalidate();
    for (auto &channel : parameter_handles_) {
        for (param_t &handle : channel) {
            handle = PARAM_INVALID;
        }
    }
}

bool MotorOutput::apply_parameter_snapshot() noexcept
{
    ParameterSnapshot candidate{};
    std::int32_t raw[kChannelCount][kFieldsPerChannel]{};
    bool loaded = true;
    {
        px4::AtomicTransaction transaction;
        loaded = command_timeout_.bound() &&
                 param_get(command_timeout_.handle(),
                           &candidate.command_timeout_s) == 0;
        for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
            for (std::size_t field = 0U; field < kFieldsPerChannel; ++field) {
                const param_t handle = parameter_handles_[channel][field];
                if (handle == PARAM_INVALID ||
                    param_get(handle, &raw[channel][field]) != 0) {
                    loaded = false;
                }
            }
        }
    }
    if (!loaded) {
        parameters_valid_ = false;
        return false;
    }
    if (!finite(candidate.command_timeout_s) ||
        candidate.command_timeout_s < 0.02F ||
        candidate.command_timeout_s > 1.0F) {
        parameters_valid_ = false;
        return false;
    }

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        if (raw[channel][Minimum] < 800 || raw[channel][Minimum] > 2200 ||
            raw[channel][Center] < 800 || raw[channel][Center] > 2200 ||
            raw[channel][Maximum] < 800 || raw[channel][Maximum] > 2200) {
            parameters_valid_ = false;
            return false;
        }
        ChannelConfig config{};
        config.function =
            static_cast<ChannelFunction>(raw[channel][Function]);
        config.minimum_us = static_cast<std::uint16_t>(raw[channel][Minimum]);
        config.center_us = static_cast<std::uint16_t>(raw[channel][Center]);
        config.maximum_us = static_cast<std::uint16_t>(raw[channel][Maximum]);
        config.reversed = raw[channel][Reversed] != 0;
        if (!valid_channel(config) ||
            (raw[channel][Reversed] != 0 && raw[channel][Reversed] != 1)) {
            parameters_valid_ = false;
            return false;
        }

        const std::uint8_t bit = static_cast<std::uint8_t>(1U << channel);
        candidate.channels[channel] = config;
        if (config.function != ChannelFunction::Disabled) {
            candidate.configured_mask |= bit;
        }
        if (config.function == ChannelFunction::MotorRight) {
            candidate.right_mask |= bit;
        } else if (config.function == ChannelFunction::MotorLeft) {
            candidate.left_mask |= bit;
        }
    }
    candidate.drive_available =
        candidate.right_mask != 0U && candidate.left_mask != 0U;
    command_timeout_.set(candidate.command_timeout_s);
    parameters_ = candidate;
    parameters_valid_ = true;
    return true;
}

bool MotorOutput::fresh_disarmed_snapshot(std::uint64_t now_us) const noexcept
{
    return active_snapshot_fresh(now_us) &&
           safety_.vehicle_status.arming_state ==
               vehicle_status_s::ARMING_STATE_DISARMED &&
           !safety_.actuator_armed.armed && !safety_.control_mode.flag_armed;
}

void MotorOutput::refresh_safety_snapshot(std::uint64_t now_us) noexcept
{
    if (actuator_armed_subscription_.update()) {
        observed_actuator_armed_ = actuator_armed_subscription_.get();
        if (safety_negative(observed_actuator_armed_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_control_mode_subscription_.update()) {
        observed_control_mode_ = vehicle_control_mode_subscription_.get();
        if (safety_negative(observed_control_mode_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_status_subscription_.update()) {
        observed_vehicle_status_ = vehicle_status_subscription_.get();
        if (safety_negative(observed_vehicle_status_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (!observed_snapshot_complete(now_us)) {
        return;
    }

    safety_.actuator_armed = observed_actuator_armed_;
    safety_.control_mode = observed_control_mode_;
    safety_.vehicle_status = observed_vehicle_status_;
    safety_.valid = true;
    safety_inhibit_observed_ = safety_negative(safety_.actuator_armed) ||
                               safety_negative(safety_.control_mode) ||
                               safety_negative(safety_.vehicle_status);
}

bool MotorOutput::observed_snapshot_complete(
    std::uint64_t now_us) const noexcept
{
    const std::uint64_t timestamp = observed_actuator_armed_.timestamp;
    return timestamp != 0U && timestamp == observed_control_mode_.timestamp &&
           timestamp == observed_vehicle_status_.timestamp &&
           timestamp <= now_us &&
           (!safety_.valid ||
            timestamp > safety_.actuator_armed.timestamp);
}

bool MotorOutput::active_snapshot_fresh(std::uint64_t now_us) const noexcept
{
    const std::uint64_t timestamp = safety_.actuator_armed.timestamp;
    return safety_.valid && timestamp != 0U && timestamp <= now_us &&
           now_us - timestamp <= kSafetyTopicTimeoutUs;
}

bool MotorOutput::safety_permits_output(std::uint64_t now_us) const noexcept
{
    if (safety_inhibit_observed_ || !active_snapshot_fresh(now_us)) {
        return false;
    }
    const actuator_armed_s &armed = safety_.actuator_armed;
    const vehicle_control_mode_s &control = safety_.control_mode;
    const vehicle_status_s &status = safety_.vehicle_status;
    const std::uint32_t manual_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const std::uint32_t termination_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;
    return armed.armed && armed.ready_to_arm && !armed.prearmed &&
           !armed.lockdown && !armed.kill && !armed.termination &&
           !armed.in_esc_calibration_mode && control.flag_armed &&
           !control.flag_multicopter_position_control_enabled &&
           control.flag_control_manual_enabled &&
           !control.flag_control_termination_enabled &&
           !control.flag_control_auto_enabled &&
           !control.flag_control_offboard_enabled &&
           !control.flag_control_position_enabled &&
           !control.flag_control_velocity_enabled &&
           !control.flag_control_altitude_enabled &&
           !control.flag_control_climb_rate_enabled &&
           !control.flag_control_acceleration_enabled &&
           !control.flag_control_attitude_enabled &&
           !control.flag_control_rates_enabled &&
           !control.flag_control_allocation_enabled &&
           control.source_id == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
           status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
           status.failure_detector_status == vehicle_status_s::FAILURE_NONE &&
           status.hil_state == vehicle_status_s::HIL_STATE_OFF &&
           status.valid_nav_states_mask == (manual_mask | termination_mask) &&
           status.can_set_nav_states_mask == manual_mask && !status.failsafe;
}

bool MotorOutput::motor_command_valid(std::uint64_t now_us) const noexcept
{
    if (!parameters_valid_ || !have_motor_command_ ||
        actuator_motors_.timestamp == 0U ||
        actuator_motors_.timestamp_sample == 0U ||
        actuator_motors_.timestamp_sample > actuator_motors_.timestamp ||
        actuator_motors_.timestamp > now_us ||
        actuator_motors_.timestamp_sample > now_us ||
        (actuator_motors_.reversible_flags & kRequiredReversibleMask) !=
            kRequiredReversibleMask ||
        !normalized(actuator_motors_.control[0]) ||
        !normalized(actuator_motors_.control[1])) {
        return false;
    }
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        parameters_.command_timeout_s * 1000000.0F);
    return timeout_us > 0U &&
           now_us - actuator_motors_.timestamp <= timeout_us &&
           now_us - actuator_motors_.timestamp_sample <= timeout_us;
}

bool MotorOutput::build_frame(
    dima::platform::ActuatorPwmFrame &frame) const noexcept
{
    if (!parameters_valid_ || !parameters_.drive_available) {
        return false;
    }
    frame = dima::platform::ActuatorPwmFrame{};
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        const ChannelConfig &config = parameters_.channels[channel];
        if (config.function == ChannelFunction::Disabled) {
            continue;
        }
        float command = config.function == ChannelFunction::MotorRight
                            ? actuator_motors_.control[0]
                            : actuator_motors_.control[1];
        if (config.reversed) {
            command = -command;
        }
        if (!normalized(command)) {
            return false;
        }
        frame.pulse_us[channel] = map_normalized(config, command);
        frame.enabled_mask |= static_cast<std::uint8_t>(1U << channel);
    }
    return frame.enabled_mask == parameters_.configured_mask;
}

dima::platform::ActuatorPwmResult MotorOutput::force_safe_off() noexcept
{
    if (pwm_ == nullptr) {
        backend_ready_ = false;
        safe_off_ = false;
        return dima::platform::ActuatorPwmResult::Fault;
    }
    if (safe_off_ && !pwm_->started()) {
        return dima::platform::ActuatorPwmResult::Applied;
    }

    const dima::platform::ActuatorPwmResult result = pwm_->stop();
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    backend_ready_ = result == dima::platform::ActuatorPwmResult::Applied;
    safe_off_ = backend_ready_;
    return result;
}

bool MotorOutput::publish_status(std::uint64_t now_us,
                                 std::uint8_t output_state,
                                 bool command_valid) noexcept
{
    actuator_output_status_s status{};
    status.timestamp = now_us;
    status.timestamp_sample = actuator_motors_.timestamp_sample;
    ++status_sequence_;
    if (status_sequence_ == 0U) {
        ++status_sequence_;
    }
    status.sequence = status_sequence_;
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        status.pwm_us[channel] = applied_frame_.pulse_us[channel];
    }
    status.active_output_mask = applied_frame_.enabled_mask;
    status.state = output_state;
    status.backend_ready = backend_ready_;
    status.drive_available = drive_available();
    status.safe_off = safe_off_;
    status.command_valid = command_valid;
    status.parameter_update_pending = parameter_update_pending_;
    return output_status_publication_.publish(status);
}

void MotorOutput::reset_runtime_state() noexcept
{
    parameters_ = ParameterSnapshot{};
    actuator_motors_ = actuator_motors_s{};
    observed_actuator_armed_ = actuator_armed_s{};
    observed_control_mode_ = vehicle_control_mode_s{};
    observed_vehicle_status_ = vehicle_status_s{};
    safety_ = SafetySnapshot{};
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    status_sequence_ = 0U;
    have_motor_command_ = false;
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    safety_inhibit_observed_ = true;
    backend_ready_ = false;
    safe_off_ = false;
    invalidate_parameter_bindings();
}

void MotorOutput::enter_error(std::uint32_t event_id) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    ScheduleCancelAndDrain();
    const dima::platform::ActuatorPwmResult stopped = force_safe_off();
    if (stopped != dima::platform::ActuatorPwmResult::Applied) {
        backend_ready_ = false;
        safe_off_ = false;
    }
    (void)publish_status(hrt_absolute_time(),
                         actuator_output_status_s::STATE_FAULT, false);
    (void)dima::events::report(event_id, dima::events::Severity::Error);
}

bool MotorOutput::finite(float value) noexcept { return std::isfinite(value); }

bool MotorOutput::normalized(float value) noexcept
{
    return finite(value) && value >= -1.0F && value <= 1.0F;
}

bool MotorOutput::valid_channel(const ChannelConfig &channel) noexcept
{
    const bool function_valid =
        channel.function == ChannelFunction::Disabled ||
        channel.function == ChannelFunction::MotorRight ||
        channel.function == ChannelFunction::MotorLeft;
    return function_valid && channel.minimum_us >= 800U &&
           channel.maximum_us <= 2200U &&
           channel.minimum_us < channel.center_us &&
           channel.center_us < channel.maximum_us;
}

std::uint16_t MotorOutput::map_normalized(const ChannelConfig &channel,
                                          float value) noexcept
{
    const float center = static_cast<float>(channel.center_us);
    const float pulse = value >= 0.0F
                            ? center + value *
                                           static_cast<float>(
                                               channel.maximum_us -
                                               channel.center_us)
                            : center + value *
                                           static_cast<float>(
                                               channel.center_us -
                                               channel.minimum_us);
    return static_cast<std::uint16_t>(pulse + 0.5F);
}

bool MotorOutput::safety_negative(const actuator_armed_s &armed) noexcept
{
    return armed.timestamp != 0U &&
           (!armed.armed || !armed.ready_to_arm || armed.prearmed ||
            armed.kill || armed.termination || armed.lockdown ||
            armed.in_esc_calibration_mode);
}

bool MotorOutput::safety_negative(
    const vehicle_control_mode_s &control) noexcept
{
    return control.timestamp != 0U &&
           (!control.flag_armed ||
            control.flag_multicopter_position_control_enabled ||
            !control.flag_control_manual_enabled ||
            control.flag_control_auto_enabled ||
            control.flag_control_offboard_enabled ||
            control.flag_control_position_enabled ||
            control.flag_control_velocity_enabled ||
            control.flag_control_altitude_enabled ||
            control.flag_control_climb_rate_enabled ||
            control.flag_control_acceleration_enabled ||
            control.flag_control_attitude_enabled ||
            control.flag_control_rates_enabled ||
            control.flag_control_allocation_enabled ||
            control.flag_control_termination_enabled ||
            control.source_id != vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

bool MotorOutput::safety_negative(const vehicle_status_s &status) noexcept
{
    return status.timestamp != 0U &&
           (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
            status.nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL ||
            status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER ||
            status.failure_detector_status != vehicle_status_s::FAILURE_NONE ||
            status.hil_state != vehicle_status_s::HIL_STATE_OFF ||
            status.failsafe);
}

} // namespace dima::modules::motor

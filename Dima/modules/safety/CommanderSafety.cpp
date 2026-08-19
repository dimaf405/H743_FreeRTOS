/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "logging/logging.hpp"

#include <cmath>

namespace dima::modules::safety {
namespace {

constexpr std::uint8_t kMavTypeGroundRover = 10U;
constexpr std::uint8_t kMavAutopilotSystemId = 1U;
constexpr std::uint8_t kMavAutopilotComponentId = 1U;
constexpr std::int32_t kRcLossActionDisarm = 6;
constexpr std::int32_t kDataLinkLossActionDisabled = 0;
constexpr std::uint32_t kManualModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
constexpr std::uint32_t kTerminationModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;

bool normalized_axis(float value) noexcept
{
    return std::isfinite(value) && value >= -1.0F && value <= 1.0F;
}

} // namespace

bool Commander::initialize_parameter_handles() noexcept
{
    rc_loss_timeout_handle_ = param_find("COM_RC_LOSS_T");
    arm_stick_deadzone_handle_ = param_find("COM_ARM_STICK_DZ");
    rc_loss_action_handle_ = param_find("NAV_RCL_ACT");
    data_link_loss_action_handle_ = param_find("NAV_DLL_ACT");
    return rc_loss_timeout_handle_ != PARAM_INVALID &&
           arm_stick_deadzone_handle_ != PARAM_INVALID &&
           rc_loss_action_handle_ != PARAM_INVALID &&
           data_link_loss_action_handle_ != PARAM_INVALID;
}

bool Commander::refresh_parameters() noexcept
{
    float loss_timeout = rc_loss_timeout_s_;
    float stick_deadzone = arm_stick_deadzone_;
    std::int32_t rc_loss_action = rc_loss_action_;
    std::int32_t data_link_loss_action = data_link_loss_action_;
    const bool core_ready = param_is_ready();
    const bool loaded = core_ready && parameter_handles_ready_ &&
                        param_get(rc_loss_timeout_handle_, &loss_timeout) == 0 &&
                        param_get(arm_stick_deadzone_handle_, &stick_deadzone) == 0 &&
                        param_get(rc_loss_action_handle_, &rc_loss_action) == 0 &&
                        param_get(data_link_loss_action_handle_,
                                  &data_link_loss_action) == 0;
    const bool valid = loaded && std::isfinite(loss_timeout) &&
                       loss_timeout >= 0.1F && loss_timeout <= 35.0F &&
                       std::isfinite(stick_deadzone) &&
                       stick_deadzone >= 0.0F && stick_deadzone <= 0.5F &&
                       rc_loss_action == kRcLossActionDisarm &&
                       data_link_loss_action == kDataLinkLossActionDisabled;
    const bool changed = valid != parameters_valid_ ||
                         (valid && (loss_timeout != rc_loss_timeout_s_ ||
                                    stick_deadzone != arm_stick_deadzone_ ||
                                    rc_loss_action != rc_loss_action_ ||
                                    data_link_loss_action !=
                                        data_link_loss_action_));

    parameters_valid_ = valid;
    if (valid) {
        rc_loss_timeout_s_ = loss_timeout;
        arm_stick_deadzone_ = stick_deadzone;
        rc_loss_action_ = rc_loss_action;
        data_link_loss_action_ = data_link_loss_action;
    }
    return changed;
}

bool Commander::refresh_manual_control() noexcept
{
    manual_control_setpoint_s setpoint{};
    bool copied = false;
    while (manual_control_subscription_.copy(&setpoint)) {
        copied = true;
        manual_control_setpoint_ = setpoint;
        have_manual_control_ = true;
    }
    return copied;
}

bool Commander::refresh_actuator_output_status() noexcept
{
    if (!actuator_output_status_subscription_.update()) {
        return false;
    }
    const actuator_output_status_s &candidate =
        actuator_output_status_subscription_.get();
    const std::uint32_t sequence_delta =
        candidate.sequence - last_actuator_output_sequence_;
    const bool sequence_valid = candidate.sequence != 0U &&
        (last_actuator_output_sequence_ == 0U ||
         (sequence_delta != 0U && sequence_delta < 0x80000000U));
    if (!sequence_valid || candidate.timestamp == 0U) {
        actuator_output_status_valid_ = false;
        return true;
    }
    actuator_output_status_ = candidate;
    last_actuator_output_sequence_ = candidate.sequence;
    actuator_output_status_valid_ = true;
    return true;
}

bool Commander::evaluate_safety(std::uint64_t now) noexcept
{
    const bool rc_valid = rc_input_valid(now);
    std::uint8_t causes = recoverable_failsafe_causes_;

    if (rc_valid) {
        causes &= static_cast<std::uint8_t>(~FailsafeRcLoss);
    }
    if (parameters_valid_) {
        causes &= static_cast<std::uint8_t>(~FailsafeParameters);
    }
    if (!actuator_armed_.armed &&
        (actuator_output_ready_for_arming(now) ||
         actuator_output_recovered_disarmed(now))) {
        causes &= static_cast<std::uint8_t>(~FailsafeActuatorOutput);
    }

    bool state_changed = false;
    if (actuator_armed_.armed) {
        if (!rc_valid) {
            causes |= FailsafeRcLoss;
        }
        if (!parameters_valid_) {
            causes |= FailsafeParameters;
        }
        const bool actuator_failed = actuator_output_fault_while_armed(now);
        if (actuator_failed) {
            causes |= FailsafeActuatorOutput;
        }
        const bool rc_loss_requires_disarm =
            !rc_valid && rc_loss_action_ == kRcLossActionDisarm;
        if (rc_loss_requires_disarm || !parameters_valid_ ||
            actuator_failed) {
            state_changed = disarm(
                vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR) ==
                TransitionResult::Changed || state_changed;
            PX4_WARN("Commander forced disarm: RC, parameter or actuator failure");
        }
    }

    const bool causes_changed = causes != recoverable_failsafe_causes_;
    recoverable_failsafe_causes_ = causes;
    return state_changed || causes_changed;
}

bool Commander::update_public_projection(std::uint64_t now) noexcept
{
    const bool checks_pass = preflight_checks_pass(now);
    const bool ready_to_arm = checks_pass || actuator_armed_.armed;
    const bool failsafe = termination_latched_ ||
                          recoverable_failsafe_causes_ != FailsafeNone;
    bool changed = vehicle_status_.pre_flight_checks_pass != checks_pass ||
                   actuator_armed_.ready_to_arm != ready_to_arm ||
                   vehicle_status_.failsafe != failsafe ||
                   actuator_armed_.termination != termination_latched_;

    vehicle_status_.pre_flight_checks_pass = checks_pass;
    vehicle_status_.failsafe = failsafe;
    actuator_armed_.ready_to_arm = ready_to_arm;
    actuator_armed_.termination = termination_latched_;
    actuator_armed_.prearmed = false;
    actuator_armed_.lockdown = false;
    actuator_armed_.in_esc_calibration_mode = false;

    const bool manual_mode = vehicle_status_.nav_state ==
                             vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const bool termination_mode = vehicle_status_.nav_state ==
                                  vehicle_status_s::NAVIGATION_STATE_TERMINATION;
    const bool mode_changed =
        vehicle_control_mode_.flag_control_manual_enabled != manual_mode ||
        vehicle_control_mode_.flag_control_termination_enabled != termination_mode ||
        vehicle_control_mode_.flag_armed != actuator_armed_.armed ||
        vehicle_control_mode_.source_id != vehicle_status_.nav_state;
    changed = mode_changed || changed;

    vehicle_control_mode_ = vehicle_control_mode_s{};
    vehicle_control_mode_.flag_armed = actuator_armed_.armed;
    vehicle_control_mode_.flag_control_manual_enabled = manual_mode;
    vehicle_control_mode_.flag_control_termination_enabled = termination_mode;
    vehicle_control_mode_.source_id = vehicle_status_.nav_state;
    return changed;
}

Commander::TransitionResult Commander::arm(
    std::uint8_t reason, std::uint64_t now) noexcept
{
    if (actuator_armed_.armed) {
        return TransitionResult::NotChanged;
    }
    if (!preflight_checks_pass(now)) {
        PX4_WARN("Arming denied: safety checks failed");
        return TransitionResult::Denied;
    }
    if (!armed_flash_.try_arm()) {
        PX4_WARN("Arming denied: Flash operation in progress");
        return TransitionResult::Denied;
    }

    actuator_armed_.armed = true;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
    vehicle_status_.latest_arming_reason = reason;
    vehicle_status_.armed_time = now;
    PX4_INFO("Rover armed");
    return TransitionResult::Changed;
}

Commander::TransitionResult Commander::disarm(std::uint8_t reason) noexcept
{
    if (!actuator_armed_.armed) {
        return TransitionResult::NotChanged;
    }

    actuator_armed_.armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    vehicle_status_.latest_disarming_reason = reason;
    vehicle_status_.armed_time = 0U;
    vehicle_status_.takeoff_time = 0U;
    armed_flash_.disarm();
    PX4_INFO("Rover disarmed");
    return TransitionResult::Changed;
}

bool Commander::rc_input_valid(std::uint64_t now) const noexcept
{
    if (!have_manual_control_ || !manual_control_setpoint_.valid ||
        manual_control_setpoint_.data_source !=
            manual_control_setpoint_s::SOURCE_RC ||
        !normalized_axis(manual_control_setpoint_.throttle) ||
        !normalized_axis(manual_control_setpoint_.yaw) ||
        manual_control_setpoint_.timestamp == 0U ||
        manual_control_setpoint_.timestamp_sample == 0U ||
        manual_control_setpoint_.timestamp_sample >
            manual_control_setpoint_.timestamp ||
        manual_control_setpoint_.timestamp > now ||
        manual_control_setpoint_.timestamp_sample > now) {
        return false;
    }

    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        rc_loss_timeout_s_ * 1000000.0F);
    return timeout_us > 0U &&
           now - manual_control_setpoint_.timestamp_sample <= timeout_us;
}

bool Commander::sticks_centered() const noexcept
{
    return normalized_axis(manual_control_setpoint_.throttle) &&
           normalized_axis(manual_control_setpoint_.yaw) &&
           std::fabs(manual_control_setpoint_.throttle) <=
               arm_stick_deadzone_ &&
               std::fabs(manual_control_setpoint_.yaw) <= arm_stick_deadzone_;
}

bool Commander::actuator_output_status_fresh(std::uint64_t now) const noexcept
{
    return actuator_output_status_valid_ &&
           actuator_output_status_.timestamp != 0U &&
           actuator_output_status_.timestamp <= now &&
           now - actuator_output_status_.timestamp <=
               kActuatorStatusTimeoutUs;
}

bool Commander::actuator_output_mapping_valid() const noexcept
{
    const std::uint8_t configured =
        actuator_output_status_.configured_output_mask;
    const std::uint8_t right = actuator_output_status_.right_output_mask;
    const std::uint8_t left = actuator_output_status_.left_output_mask;
    const std::uint8_t supported =
        static_cast<std::uint8_t>((1U << actuator_output_status_s::NUM_OUTPUTS) -
                                  1U);
    return configured != 0U && right != 0U && left != 0U &&
           (configured & static_cast<std::uint8_t>(~supported)) == 0U &&
           (right & left) == 0U &&
           static_cast<std::uint8_t>(right | left) == configured;
}

bool Commander::actuator_output_ready_for_arming(
    std::uint64_t now) const noexcept
{
    if (!actuator_output_status_fresh(now) ||
        !actuator_output_mapping_valid() ||
        actuator_output_status_.state !=
            actuator_output_status_s::STATE_DISARMED_NEUTRAL ||
        !actuator_output_status_.backend_ready ||
        !actuator_output_status_.drive_available ||
        actuator_output_status_.safe_off ||
        actuator_output_status_.parameter_update_pending ||
        actuator_output_status_.active_output_mask !=
            actuator_output_status_.configured_output_mask) {
        return false;
    }
    for (std::uint8_t index = 0U;
         index < actuator_output_status_s::NUM_OUTPUTS; ++index) {
        const bool configured =
            (actuator_output_status_.configured_output_mask &
             static_cast<std::uint8_t>(1U << index)) != 0U;
        const std::uint16_t pulse = actuator_output_status_.pwm_us[index];
        if ((configured && (pulse < 800U || pulse > 2200U)) ||
            (!configured && pulse != 0U)) {
            return false;
        }
    }
    return true;
}

bool Commander::actuator_output_recovered_disarmed(
    std::uint64_t now) const noexcept
{
    if (!actuator_output_status_fresh(now) ||
        !actuator_output_mapping_valid() ||
        actuator_output_status_.state !=
            actuator_output_status_s::STATE_HARD_SAFE_OFF ||
        !actuator_output_status_.backend_ready ||
        !actuator_output_status_.drive_available ||
        !actuator_output_status_.safe_off ||
        actuator_output_status_.parameter_update_pending ||
        actuator_output_status_.active_output_mask != 0U) {
        return false;
    }
    for (const std::uint16_t pulse : actuator_output_status_.pwm_us) {
        if (pulse != 0U) {
            return false;
        }
    }
    return true;
}

bool Commander::actuator_output_fault_while_armed(
    std::uint64_t now) const noexcept
{
    if (!actuator_armed_.armed) {
        return false;
    }
    const bool in_transition = vehicle_status_.armed_time != 0U &&
        vehicle_status_.armed_time <= now &&
        now - vehicle_status_.armed_time <= kActuatorArmTransitionUs;
    if (!actuator_output_status_fresh(now)) {
        return !in_transition;
    }
    if (actuator_output_status_.state ==
        actuator_output_status_s::STATE_FAULT) {
        return true;
    }
    if (in_transition &&
        actuator_output_status_.timestamp < vehicle_status_.armed_time) {
        return false;
    }
    if (in_transition &&
        actuator_output_status_.state != actuator_output_status_s::STATE_ACTIVE) {
        /* A healthy neutral/hard-off frame is expected while the three Arm
         * topics converge. Backend retry or a broken mapping is not. */
        return actuator_output_status_.state ==
                   actuator_output_status_s::STATE_RETRY ||
               !actuator_output_mapping_valid() ||
               !actuator_output_status_.backend_ready ||
               !actuator_output_status_.drive_available;
    }
    return !actuator_output_mapping_valid() ||
           actuator_output_status_.state !=
               actuator_output_status_s::STATE_ACTIVE ||
           !actuator_output_status_.backend_ready ||
           !actuator_output_status_.drive_available ||
           actuator_output_status_.safe_off ||
           !actuator_output_status_.command_valid ||
           actuator_output_status_.active_output_mask !=
               actuator_output_status_.configured_output_mask;
}

bool Commander::preflight_checks_pass(std::uint64_t now) const noexcept
{
    return parameters_valid_ &&
           vehicle_status_.nav_state ==
               vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           rc_input_valid(now) && sticks_centered() &&
           actuator_output_ready_for_arming(now) &&
           !actuator_armed_.kill && !termination_latched_ &&
           !vehicle_status_.rc_calibration_in_progress;
}

bool Commander::action_request_fresh(const action_request_s &request,
                                     std::uint64_t now) const noexcept
{
    return request.timestamp != 0U && request.timestamp <= now &&
           now - request.timestamp <= kActionRequestMaxAgeUs;
}

bool Commander::publish_state(std::uint64_t now) noexcept
{
    actuator_armed_.timestamp = now;
    const bool armed_published = actuator_armed_publication_.publish(
        actuator_armed_);

    vehicle_control_mode_.timestamp = now;
    const bool control_mode_published =
        vehicle_control_mode_publication_.publish(vehicle_control_mode_);

    vehicle_status_.timestamp = now;
    const bool status_published = vehicle_status_publication_.publish(
        vehicle_status_);

    if (armed_published && control_mode_published && status_published) {
        last_publish_time_ = now;
        return true;
    }
    return false;
}

void Commander::reset_runtime_state() noexcept
{
    actuator_armed_ = actuator_armed_s{};
    vehicle_control_mode_ = vehicle_control_mode_s{};
    vehicle_status_ = vehicle_status_s{};
    manual_control_setpoint_ = manual_control_setpoint_s{};
    actuator_output_status_ = actuator_output_status_s{};
    rc_loss_timeout_handle_ = PARAM_INVALID;
    arm_stick_deadzone_handle_ = PARAM_INVALID;
    rc_loss_action_handle_ = PARAM_INVALID;
    data_link_loss_action_handle_ = PARAM_INVALID;
    rc_loss_timeout_s_ = 0.5F;
    arm_stick_deadzone_ = 0.1F;
    rc_loss_action_ = kRcLossActionDisarm;
    data_link_loss_action_ = kDataLinkLossActionDisabled;
    last_publish_time_ = 0U;
    last_actuator_output_sequence_ = 0U;
    recoverable_failsafe_causes_ = FailsafeNone;
    parameter_handles_ready_ = false;
    parameters_valid_ = false;
    have_manual_control_ = false;
    actuator_output_status_valid_ = false;
    // Termination 是唯一跨 Application Runtime 保留的安全锁存；
    // 此处故意不清 termination_latched_，只能由 MCU 复位解除。
}

void Commander::initialize_public_state(std::uint64_t now) noexcept
{
    actuator_armed_ = actuator_armed_s{};
    vehicle_control_mode_ = vehicle_control_mode_s{};
    vehicle_status_ = vehicle_status_s{};

    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    vehicle_status_.latest_arming_reason =
        vehicle_status_s::ARM_DISARM_REASON_TRANSITION_TO_STANDBY;
    vehicle_status_.latest_disarming_reason =
        vehicle_status_s::ARM_DISARM_REASON_TRANSITION_TO_STANDBY;
    vehicle_status_.nav_state_timestamp = now;
    vehicle_status_.nav_state_user_intention =
        vehicle_status_s::NAVIGATION_STATE_MANUAL;
    vehicle_status_.nav_state = termination_latched_
                                    ? vehicle_status_s::NAVIGATION_STATE_TERMINATION
                                    : vehicle_status_s::NAVIGATION_STATE_MANUAL;
    vehicle_status_.valid_nav_states_mask =
        kManualModeMask | kTerminationModeMask;
    vehicle_status_.can_set_nav_states_mask = kManualModeMask;
    vehicle_status_.failure_detector_status = vehicle_status_s::FAILURE_NONE;
    vehicle_status_.hil_state = vehicle_status_s::HIL_STATE_OFF;
    vehicle_status_.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
    vehicle_status_.failsafe_defer_state =
        vehicle_status_s::FAILSAFE_DEFER_STATE_DISABLED;
    vehicle_status_.system_type = kMavTypeGroundRover;
    vehicle_status_.system_id = kMavAutopilotSystemId;
    vehicle_status_.component_id = kMavAutopilotComponentId;

    (void)update_public_projection(now);
    last_publish_time_ = 0U;
}

void Commander::initialize_disarmed_snapshot(std::uint64_t now) noexcept
{
    const bool kill_latched = actuator_armed_.kill;
    armed_flash_.disarm();

    initialize_public_state(now);
    actuator_armed_.armed = false;
    actuator_armed_.ready_to_arm = false;
    actuator_armed_.kill = kill_latched;
    vehicle_control_mode_.flag_armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    vehicle_status_.latest_disarming_reason =
        vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR;
    vehicle_status_.armed_time = 0U;
    vehicle_status_.takeoff_time = 0U;
    vehicle_status_.pre_flight_checks_pass = false;
}

} // namespace dima::modules::safety

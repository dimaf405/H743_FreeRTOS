#include "MotorOutput.hpp"

namespace dima::modules::motor {
namespace {

constexpr std::uint16_t kRequiredReversibleMask = 0x0003U;

} // namespace

bool MotorOutput::fresh_disarmed_snapshot(std::uint64_t now_us) const noexcept
{
    return active_snapshot_fresh(now_us) &&
           safety_.vehicle_status.arming_state ==
               vehicle_status_s::ARMING_STATE_DISARMED &&
           !safety_.actuator_armed.armed && !safety_.control_mode.flag_armed;
}

void MotorOutput::refresh_safety_snapshot(std::uint64_t now_us) noexcept
{
    // 三个安全 Topic 异步到达：任一负向值必须立即锁存，不能等待完整快照，
    // 否则混合代际消息可能在窗口内短暂放行输出。
    if (actuator_armed_subscription_.update()) {
        observed_actuator_armed_ = actuator_armed_subscription_.get();
        if (safety_negative(observed_actuator_armed_)) {
            safety_inhibit_observed_ = true;
        }
        if (hard_safe_negative(observed_actuator_armed_)) {
            hard_safe_inhibit_observed_ = true;
        }
    }
    if (vehicle_control_mode_subscription_.update()) {
        observed_control_mode_ = vehicle_control_mode_subscription_.get();
        if (safety_negative(observed_control_mode_)) {
            safety_inhibit_observed_ = true;
        }
        if (hard_safe_negative(observed_control_mode_)) {
            hard_safe_inhibit_observed_ = true;
        }
    }
    if (vehicle_status_subscription_.update()) {
        observed_vehicle_status_ = vehicle_status_subscription_.get();
        if (safety_negative(observed_vehicle_status_)) {
            safety_inhibit_observed_ = true;
        }
        if (hard_safe_negative(observed_vehicle_status_)) {
            hard_safe_inhibit_observed_ = true;
        }
    }
    if (!observed_snapshot_complete(now_us)) {
        return;
    }

    safety_.actuator_armed = observed_actuator_armed_;
    safety_.control_mode = observed_control_mode_;
    safety_.vehicle_status = observed_vehicle_status_;
    safety_.valid = true;
    // ACTIVE inhibit 包含普通 Disarm；hard-safe inhibit 只锁存 Kill、Termination、
    // Failsafe 等必须物理断脉冲的条件，健康 Disarmed 才可在完整快照后输出中立帧。
    safety_inhibit_observed_ = safety_negative(safety_.actuator_armed) ||
                               safety_negative(safety_.control_mode) ||
                               safety_negative(safety_.vehicle_status);
    hard_safe_inhibit_observed_ =
        hard_safe_negative(safety_.actuator_armed) ||
        hard_safe_negative(safety_.control_mode) ||
        hard_safe_negative(safety_.vehicle_status);
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

bool MotorOutput::safety_permits_disarmed_neutral(
    std::uint64_t now_us) const noexcept
{
    if (hard_safe_inhibit_observed_ || !parameters_valid_ ||
        !parameters_.drive_available ||
        parameter_update_pending_ || !active_snapshot_fresh(now_us)) {
        return false;
    }
    const actuator_armed_s &armed = safety_.actuator_armed;
    const vehicle_control_mode_s &control = safety_.control_mode;
    const vehicle_status_s &status = safety_.vehicle_status;
    const std::uint32_t manual_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const std::uint32_t termination_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;
    return !armed.armed && !armed.prearmed && !armed.lockdown && !armed.kill &&
           !armed.termination && !armed.in_esc_calibration_mode &&
           !control.flag_armed && control.flag_control_manual_enabled &&
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
           status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED &&
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

bool MotorOutput::hard_safe_negative(
    const actuator_armed_s &armed) noexcept
{
    return armed.timestamp != 0U &&
           (armed.prearmed || armed.kill || armed.termination ||
            armed.lockdown || armed.in_esc_calibration_mode);
}

bool MotorOutput::hard_safe_negative(
    const vehicle_control_mode_s &control) noexcept
{
    return control.timestamp != 0U &&
           (control.flag_multicopter_position_control_enabled ||
            !control.flag_control_manual_enabled ||
            control.flag_control_termination_enabled ||
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
            control.source_id != vehicle_status_s::NAVIGATION_STATE_MANUAL);
}

bool MotorOutput::hard_safe_negative(
    const vehicle_status_s &status) noexcept
{
    return status.timestamp != 0U &&
           (status.nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL ||
            status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER ||
            status.failure_detector_status != vehicle_status_s::FAILURE_NONE ||
            status.hil_state != vehicle_status_s::HIL_STATE_OFF ||
            status.failsafe);
}

} // namespace dima::modules::motor

#include "MotorOutput.hpp"

#include <cmath>

namespace dima::modules::motor {
namespace {

constexpr std::uint16_t kRequiredReversibleMask = 0x0003U;
constexpr std::uint32_t kManualModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
constexpr std::uint32_t kAutoMissionModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
constexpr std::uint32_t kAutoLoiterModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
constexpr std::uint32_t kTerminationModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;
constexpr std::uint32_t kImplementedModeMask =
    kManualModeMask | kAutoMissionModeMask | kAutoLoiterModeMask |
    kTerminationModeMask;

bool manual_projection(const vehicle_control_mode_s &control) noexcept
{
    return control.source_id == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
        control.flag_control_manual_enabled &&
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
        !control.flag_control_termination_enabled &&
        !control.flag_multicopter_position_control_enabled;
}

bool auto_projection(const vehicle_control_mode_s &control) noexcept
{
    const bool supported =
        control.source_id ==
            vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        control.source_id == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return supported && !control.flag_control_manual_enabled &&
        control.flag_control_auto_enabled &&
        !control.flag_control_offboard_enabled &&
        control.flag_control_position_enabled &&
        control.flag_control_velocity_enabled &&
        !control.flag_control_altitude_enabled &&
        !control.flag_control_climb_rate_enabled &&
        !control.flag_control_acceleration_enabled &&
        control.flag_control_attitude_enabled &&
        control.flag_control_rates_enabled &&
        !control.flag_control_allocation_enabled &&
        !control.flag_control_termination_enabled &&
        !control.flag_multicopter_position_control_enabled;
}

bool termination_projection(const vehicle_control_mode_s &control) noexcept
{
    return control.source_id ==
               vehicle_status_s::NAVIGATION_STATE_TERMINATION &&
        !control.flag_control_manual_enabled &&
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
        control.flag_control_termination_enabled &&
        !control.flag_multicopter_position_control_enabled;
}

bool status_contract(const vehicle_status_s &status) noexcept
{
    return status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
        status.failure_detector_status == vehicle_status_s::FAILURE_NONE &&
        status.hil_state == vehicle_status_s::HIL_STATE_OFF &&
        status.valid_nav_states_mask == kImplementedModeMask &&
        status.can_set_nav_states_mask == kManualModeMask;
}

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
    // 三个安全 Topic 异步到达：任一负向值或更新代际必须立即锁存 ACTIVE inhibit。
    // 更新代际覆盖 Manual -> AUTO 等“每个单项都合法”但尚未同拍的正向切换，
    // 禁止 MotorOutput 在窗口内继续使用上一代 PWM；完整同拍后再统一解除。
    if (actuator_armed_subscription_.update()) {
        observed_actuator_armed_ = actuator_armed_subscription_.get();
        const bool newer_generation = observed_actuator_armed_.timestamp != 0U &&
            (!safety_.valid || observed_actuator_armed_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_actuator_armed_)) {
            safety_inhibit_observed_ = true;
        }
        if (hard_safe_negative(observed_actuator_armed_)) {
            hard_safe_inhibit_observed_ = true;
        }
    }
    if (vehicle_control_mode_subscription_.update()) {
        observed_control_mode_ = vehicle_control_mode_subscription_.get();
        const bool newer_generation = observed_control_mode_.timestamp != 0U &&
            (!safety_.valid || observed_control_mode_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_control_mode_)) {
            safety_inhibit_observed_ = true;
        }
        if (hard_safe_negative(observed_control_mode_)) {
            hard_safe_inhibit_observed_ = true;
        }
    }
    if (vehicle_status_subscription_.update()) {
        observed_vehicle_status_ = vehicle_status_subscription_.get();
        const bool newer_generation = observed_vehicle_status_.timestamp != 0U &&
            (!safety_.valid || observed_vehicle_status_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_vehicle_status_)) {
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
    const bool manual = manual_projection(control) &&
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const bool automatic = auto_projection(control) &&
        control.source_id == status.nav_state &&
        (status.nav_state ==
             vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
         status.nav_state ==
             vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
    return armed.armed && armed.ready_to_arm && !armed.prearmed &&
           !armed.lockdown && !armed.kill && !armed.termination &&
           !armed.in_esc_calibration_mode && control.flag_armed &&
           status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
           status_contract(status) && !status.failsafe &&
           (manual || automatic);
}

bool MotorOutput::safety_permits_disarmed_neutral(
    std::uint64_t now_us) const noexcept
{
    if (hard_safe_inhibit_observed_ || !parameters_valid_ ||
        parameters_.configured_mask == 0U ||
        parameter_update_pending_ || !active_snapshot_fresh(now_us)) {
        return false;
    }
    const actuator_armed_s &armed = safety_.actuator_armed;
    const vehicle_control_mode_s &control = safety_.control_mode;
    const vehicle_status_s &status = safety_.vehicle_status;
    return !armed.armed && !armed.prearmed && !armed.lockdown && !armed.kill &&
           !armed.termination && !armed.in_esc_calibration_mode &&
           !control.flag_armed && manual_projection(control) &&
           status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED &&
           status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status_contract(status) && !status.failsafe;
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

bool MotorOutput::motor_control_inhibit_valid(
    std::uint64_t now_us) const noexcept
{
    // 控制器主动失效帧与生产者超时必须可区分：这里只接受当前 Commander
    // 代际之后新鲜到达、12 路全部为 NaN 的精确失效帧。timestamp_sample
    // 可以为 0 或已经过期，因为 EKF 无效/失鲜正是请求停波的原因；但发布时刻
    // 仍必须在 COM_ACT_LOSS_T 内，避免把 RoverDifferential 停止运行伪装成
    // “有意抑制”。任何有限值、Inf、旧代帧或结构错误都退回普通 Hard Safe Off。
    if (!parameters_valid_ || !have_motor_command_ || !safety_.valid ||
        actuator_motors_.timestamp == 0U ||
        actuator_motors_.timestamp > now_us ||
        safety_.vehicle_status.nav_state_timestamp == 0U ||
        actuator_motors_.timestamp <=
            safety_.vehicle_status.nav_state_timestamp ||
        (actuator_motors_.timestamp_sample != 0U &&
         (actuator_motors_.timestamp_sample > actuator_motors_.timestamp ||
          actuator_motors_.timestamp_sample > now_us)) ||
        (actuator_motors_.reversible_flags & kRequiredReversibleMask) !=
            kRequiredReversibleMask) {
        return false;
    }

    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        parameters_.command_timeout_s * 1000000.0F);
    if (timeout_us == 0U ||
        now_us - actuator_motors_.timestamp > timeout_us) {
        return false;
    }

    for (const float control : actuator_motors_.control) {
        if (!std::isnan(control)) {
            return false;
        }
    }
    return true;
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
            (!manual_projection(control) && !auto_projection(control)));
}

bool MotorOutput::safety_negative(const vehicle_status_s &status) noexcept
{
    const bool active_mode =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return status.timestamp != 0U &&
           (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
            !active_mode || !status_contract(status) || status.failsafe);
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
    if (control.timestamp == 0U) {
        return false;
    }
    // Termination 的精确投影和任何未识别组合都要求物理停波；只有精确 Manual
    // 或精确 AUTO 可以保留 ACTIVE/之后恢复 Disarmed Neutral 的资格。
    return termination_projection(control) ||
           (!manual_projection(control) && !auto_projection(control));
}

bool MotorOutput::hard_safe_negative(
    const vehicle_status_s &status) noexcept
{
    const bool recoverable_mode =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return status.timestamp != 0U &&
           (!recoverable_mode || !status_contract(status) ||
            status.failsafe);
}

} // namespace dima::modules::motor

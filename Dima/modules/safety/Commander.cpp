/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to Dima FreeRTOS.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "ArmingFlashInterlock.h"
#include "logging/logging.hpp"
#include "freertos/hrt.hpp"

#include <cmath>

namespace dima::modules::safety {
namespace {

constexpr std::uint8_t kMavTypeGroundRover = 10U;
constexpr std::uint8_t kMavAutopilotSystemId = 1U;
constexpr std::uint8_t kMavAutopilotComponentId = 1U;
constexpr std::uint32_t kManualModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
constexpr std::uint32_t kTerminationModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;

bool normalized_axis(float value) noexcept
{
    return std::isfinite(value) && value >= -1.0F && value <= 1.0F;
}

} // namespace

Commander::Commander() noexcept
    : px4::ScheduledWorkItem("commander", px4::wq_configurations::hp_default)
{
}

Commander::~Commander()
{
    stop();
}

bool Commander::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("Commander WorkQueue unavailable");
        return false;
    }

    parameter_handles_ready_ = initialize_parameter_handles();
    if (!parameter_handles_ready_) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("Commander parameter handles unavailable");
        return false;
    }

    (void)refresh_parameters();
    have_manual_control_ = false;
    manual_control_setpoint_ = manual_control_setpoint_s{};
    recoverable_failsafe_causes_ = FailsafeNone;
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();

    if (!action_request_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("Commander action callback registration failed");
        return false;
    }
    if (!manual_control_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        action_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("Commander manual callback registration failed");
        return false;
    }
    if (!parameter_update_subscription_.registerCallback()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        manual_control_subscription_.unregisterCallback();
        action_request_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        PX4_ERR("Commander parameter callback registration failed");
        return false;
    }

    const std::uint64_t now = hrt_absolute_time();
    initialize_public_state(now);
    state_ = dima::middleware::lifecycle::ModuleState::Running;

    if (!publish_state(now)) {
        return handle_publication_failure(now);
    }
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
        return false;
    }

    return true;
}

void Commander::stop()
{
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();
    if (state_ != dima::middleware::lifecycle::ModuleState::Error) {
        state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    }
    parameter_update_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    action_request_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    actuator_armed_.armed = false;
    vehicle_control_mode_.flag_armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();
    have_manual_control_ = false;
    recoverable_failsafe_causes_ = FailsafeNone;
}

dima::middleware::lifecycle::ModuleState Commander::state() const
{
    return state_;
}

void Commander::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    while (parameter_update_subscription_.copy(&parameter_update)) {
        // 深度为 1；循环形式仍保证未来扩大队列后不会只消费最后一条。
    }

    bool state_changed = refresh_parameters();
    (void)refresh_manual_control();

    std::uint64_t now = hrt_absolute_time();
    state_changed = evaluate_safety(now) || state_changed;
    state_changed = update_public_projection(now) || state_changed;
    if (state_changed && !publish_state(now)) {
        (void)handle_publication_failure(now);
        return;
    }

    action_request_s request{};
    while (action_request_subscription_.copy(&request)) {
        now = hrt_absolute_time();
        bool action_changed = execute_action(request, now);
        action_changed = update_public_projection(now) || action_changed;

        // 每一个状态转换都完成一次固定顺序发布，保持 Arm/Kill 队列顺序。
        if (action_changed && !publish_state(now)) {
            (void)handle_publication_failure(now);
            return;
        }
    }

    now = hrt_absolute_time();
    state_changed = evaluate_safety(now);
    state_changed = update_public_projection(now) || state_changed;
    const bool heartbeat_due = now - last_publish_time_ >= kPublishIntervalUs;
    if ((state_changed || heartbeat_due) && !publish_state(now)) {
        (void)handle_publication_failure(now);
        return;
    }

    // uORB 回调的 ScheduleNow 会替换周期调度，每次运行后恢复 20 ms 检查。
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
    }
}

bool Commander::initialize_parameter_handles() noexcept
{
    rc_loss_timeout_handle_ = param_find("COM_RC_LOSS_T");
    arm_stick_deadzone_handle_ = param_find("COM_ARM_STICK_DZ");
    return rc_loss_timeout_handle_ != PARAM_INVALID &&
           arm_stick_deadzone_handle_ != PARAM_INVALID;
}

bool Commander::refresh_parameters() noexcept
{
    float loss_timeout = rc_loss_timeout_s_;
    float stick_deadzone = arm_stick_deadzone_;
    const bool core_ready = param_is_ready();
    const bool loaded = core_ready && parameter_handles_ready_ &&
                        param_get(rc_loss_timeout_handle_, &loss_timeout) == 0 &&
                        param_get(arm_stick_deadzone_handle_, &stick_deadzone) == 0;
    const bool valid = loaded && std::isfinite(loss_timeout) &&
                       loss_timeout >= 0.1F && loss_timeout <= 35.0F &&
                       std::isfinite(stick_deadzone) &&
                       stick_deadzone >= 0.0F && stick_deadzone <= 0.5F;
    const bool changed = valid != parameters_valid_ ||
                         (valid && (loss_timeout != rc_loss_timeout_s_ ||
                                    stick_deadzone != arm_stick_deadzone_));

    parameters_valid_ = valid;
    if (valid) {
        rc_loss_timeout_s_ = loss_timeout;
        arm_stick_deadzone_ = stick_deadzone;
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

    bool changed = causes != recoverable_failsafe_causes_;
    recoverable_failsafe_causes_ = causes;

    if (actuator_armed_.armed) {
        if (!rc_valid) {
            recoverable_failsafe_causes_ |= FailsafeRcLoss;
        }
        if (!parameters_valid_) {
            recoverable_failsafe_causes_ |= FailsafeParameters;
        }
        if (!rc_valid || !parameters_valid_) {
            changed = disarm(vehicle_status_s::ARM_DISARM_REASON_FAILURE_DETECTOR) || changed;
            PX4_WARN("Commander forced disarm: RC or parameter failure");
        }
    }

    return changed || causes != recoverable_failsafe_causes_;
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

bool Commander::execute_action(const action_request_s &request,
                               std::uint64_t now) noexcept
{
    const std::uint8_t reason = reason_from_source(request.source);

    switch (request.action) {
    case action_request_s::ACTION_DISARM:
        return disarm(reason);

    case action_request_s::ACTION_ARM:
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Arm request");
            return false;
        }
        return arm(reason, now);

    case action_request_s::ACTION_TOGGLE_ARMING:
        if (actuator_armed_.armed) {
            return disarm(reason);
        }
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Toggle-Arm request");
            return false;
        }
        return arm(reason, now);

    case action_request_s::ACTION_UNKILL:
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Unkill request");
            return false;
        }
        if (actuator_armed_.kill) {
            actuator_armed_.kill = false;
            PX4_INFO("Kill disengaged");
            return true;
        }
        return false;

    case action_request_s::ACTION_KILL:
        if (!actuator_armed_.kill) {
            actuator_armed_.kill = true;
            PX4_WARN("Kill engaged");
            return true;
        }
        return false;

    case action_request_s::ACTION_TERMINATION:
        if (!termination_latched_) {
            termination_latched_ = true;
            vehicle_status_.nav_state =
                vehicle_status_s::NAVIGATION_STATE_TERMINATION;
            vehicle_status_.nav_state_timestamp = now;
            PX4_ERR("Termination engaged");
            return true;
        }
        return false;

    case action_request_s::ACTION_SWITCH_MODE:
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale mode request");
            return false;
        }
        return switch_mode(request.mode, now);

    default:
        PX4_WARN("Commander rejected unsupported action %u", request.action);
        return false;
    }
}

bool Commander::arm(std::uint8_t reason, std::uint64_t now) noexcept
{
    if (actuator_armed_.armed) {
        return false;
    }
    if (!preflight_checks_pass(now)) {
        PX4_WARN("Arming denied: safety checks failed");
        return false;
    }
    if (!dima_arming_flash_try_arm()) {
        PX4_WARN("Arming denied: Flash operation in progress");
        return false;
    }

    actuator_armed_.armed = true;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
    vehicle_status_.latest_arming_reason = reason;
    vehicle_status_.armed_time = now;
    armed_snapshot_.store(true);
    PX4_INFO("Rover armed");
    return true;
}

bool Commander::disarm(std::uint8_t reason) noexcept
{
    if (!actuator_armed_.armed) {
        return false;
    }

    actuator_armed_.armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    vehicle_status_.latest_disarming_reason = reason;
    vehicle_status_.armed_time = 0U;
    vehicle_status_.takeoff_time = 0U;
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();
    PX4_INFO("Rover disarmed");
    return true;
}

bool Commander::switch_mode(std::uint8_t mode, std::uint64_t now) noexcept
{
    if (termination_latched_ ||
        mode != vehicle_status_s::NAVIGATION_STATE_MANUAL) {
        PX4_WARN("Commander rejected unsupported mode %u", mode);
        return false;
    }
    if (vehicle_status_.nav_state == mode) {
        return false;
    }

    vehicle_status_.nav_state = mode;
    vehicle_status_.nav_state_user_intention = mode;
    vehicle_status_.nav_state_timestamp = now;
    return true;
}

bool Commander::rc_input_valid(std::uint64_t now) const noexcept
{
    if (!have_manual_control_ || !manual_control_setpoint_.valid ||
        manual_control_setpoint_.data_source !=
            manual_control_setpoint_s::SOURCE_RC ||
        !normalized_axis(manual_control_setpoint_.throttle) ||
        !normalized_axis(manual_control_setpoint_.yaw) ||
        manual_control_setpoint_.timestamp_sample == 0U ||
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

bool Commander::preflight_checks_pass(std::uint64_t now) const noexcept
{
    return parameters_valid_ &&
           vehicle_status_.nav_state ==
               vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           rc_input_valid(now) && sticks_centered() &&
           !actuator_armed_.kill && !termination_latched_;
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
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();

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

bool Commander::handle_publication_failure(std::uint64_t now) noexcept
{
    initialize_disarmed_snapshot(now);
    if (!publish_state(now)) {
        enter_error("Commander DISARMED state publication failed");
        return false;
    }

    PX4_WARN("Commander publication recovered with DISARMED snapshot");
    if (!ScheduleOnInterval(kCheckIntervalUs)) {
        handle_scheduling_failure(now);
        return false;
    }
    return true;
}

void Commander::handle_scheduling_failure(std::uint64_t now) noexcept
{
    initialize_disarmed_snapshot(now);
    (void)publish_state(now);
    enter_error("Commander scheduling failed");
}

void Commander::enter_error(const char *reason) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();
    parameter_update_subscription_.unregisterCallback();
    manual_control_subscription_.unregisterCallback();
    action_request_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    actuator_armed_.armed = false;
    vehicle_control_mode_.flag_armed = false;
    vehicle_status_.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    armed_snapshot_.store(false);
    dima_arming_flash_disarm();
    PX4_ERR("%s", reason);
}

std::uint8_t Commander::reason_from_source(std::uint8_t source) noexcept
{
    switch (source) {
    case action_request_s::SOURCE_STICK_GESTURE:
        return vehicle_status_s::ARM_DISARM_REASON_STICK_GESTURE;
    case action_request_s::SOURCE_RC_SWITCH:
    case action_request_s::SOURCE_RC_BUTTON:
    case action_request_s::SOURCE_RC_MODE_SLOT:
        return vehicle_status_s::ARM_DISARM_REASON_RC_SWITCH;
    default:
        return vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL;
    }
}

} // namespace dima::modules::safety

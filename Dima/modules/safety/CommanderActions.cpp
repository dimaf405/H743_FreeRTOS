/****************************************************************************
 * PX4-Autopilot v1.17.0 Commander Rover subset adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "commander"
#include "Commander.hpp"

#include "logging/logging.hpp"

namespace dima::modules::safety {
namespace {

bool rc_action_source(std::uint8_t source) noexcept
{
    return source == action_request_s::SOURCE_STICK_GESTURE ||
           source == action_request_s::SOURCE_RC_SWITCH ||
           source == action_request_s::SOURCE_RC_BUTTON;
}

} // namespace

bool Commander::execute_action(const action_request_s &request,
                               std::uint64_t now) noexcept
{
    const std::uint8_t reason = reason_from_source(request.source);

    // 校准期间禁止来自 RC 的正向状态动作，但 Disarm/Kill/Termination 等负向安全动作
    // 永远保留，避免校准会话反而阻断紧急停机。
    if ((vehicle_status_.rc_calibration_in_progress ||
         vehicle_status_.calibration_enabled) &&
        rc_action_source(request.source) &&
        request.action != action_request_s::ACTION_DISARM &&
        request.action != action_request_s::ACTION_KILL &&
        request.action != action_request_s::ACTION_TERMINATION) {
        return false;
    }

    switch (request.action) {
    case action_request_s::ACTION_DISARM:
        return disarm(reason, now) == TransitionResult::Changed;

    case action_request_s::ACTION_ARM:
        // 正向动作必须满足队列新鲜度；安全负向动作即使延迟也仍允许执行。
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Arm request");
            return false;
        }
        return arm(reason, now) == TransitionResult::Changed;

    case action_request_s::ACTION_TOGGLE_ARMING:
        if (actuator_armed_.armed) {
            return disarm(reason, now) == TransitionResult::Changed;
        }
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Toggle-Arm request");
            return false;
        }
        return arm(reason, now) == TransitionResult::Changed;

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
    {
        // Kill 同时锁存 kill 位并解除 Armed；解除 Kill 不会自动重新 Arm。
        bool changed = false;
        if (!actuator_armed_.kill) {
            actuator_armed_.kill = true;
            changed = true;
        }
        if (actuator_armed_.armed) {
            changed = disarm(reason, now) == TransitionResult::Changed ||
                      changed;
        }
        if (changed) {
            PX4_WARN("Kill engaged; Rover requires a new Arm action");
        }
        return changed;
    }

    case action_request_s::ACTION_SWITCH_MODE:
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale mode switch");
            return false;
        }
        if (request.mode == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
            if (rc_action_source(request.source) && !rc_input_valid(now)) {
                PX4_WARN("Commander rejected Manual switch without RC");
                return false;
            }
            return change_navigation_state(
                vehicle_status_s::NAVIGATION_STATE_MANUAL, now);
        }
        if (request.mode ==
                vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
            request.source == action_request_s::SOURCE_RC_MODE_SLOT) {
            // QGC PX4 插件以 SET_MODE(AUTO_MISSION) 作为启动入口。
            // 这里不直接改 nav_state，而是调用与 MAV_CMD_MISSION_START
            // 完全相同的事务：必须已 Armed，且任务、参数、AutoMode
            // 和 EKF 全部就绪。Disarmed 请求会被拒绝，不会隐式 Arm。
            bool state_changed = false;
            const std::uint8_t result = start_mission(now, state_changed);
            if (result !=
                    vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
                PX4_WARN("Commander rejected AUTO mission mode request (%u)",
                         result);
            }
            return state_changed;
        }
        {
            PX4_WARN("Commander rejected unsupported or stale mode switch");
            return false;
        }

    case action_request_s::ACTION_TERMINATION:
        // Termination 只允许从 false 锁存到 true，Runtime stop/start 不会清除，
        // 必须通过 MCU 复位才能恢复。
        if (!termination_latched_) {
            termination_latched_ = true;
            // 统一经过导航状态事务，确保从 AUTO 进入 Termination 时同时锁存
            // Mission suspend；否则电机虽已 hard-off，QGC 仍会把任务误报为 Active。
            (void)change_navigation_state(
                vehicle_status_s::NAVIGATION_STATE_TERMINATION, now);
            PX4_ERR("Termination engaged");
            return true;
        }
        return false;

    default:
        PX4_WARN("Commander rejected unsupported action %u", request.action);
        return false;
    }
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

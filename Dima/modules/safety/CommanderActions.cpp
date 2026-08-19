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

    if (vehicle_status_.rc_calibration_in_progress &&
        rc_action_source(request.source) &&
        request.action != action_request_s::ACTION_DISARM &&
        request.action != action_request_s::ACTION_KILL &&
        request.action != action_request_s::ACTION_TERMINATION) {
        return false;
    }

    switch (request.action) {
    case action_request_s::ACTION_DISARM:
        return disarm(reason) == TransitionResult::Changed;

    case action_request_s::ACTION_ARM:
        if (!action_request_fresh(request, now)) {
            PX4_WARN("Commander rejected stale Arm request");
            return false;
        }
        return arm(reason, now) == TransitionResult::Changed;

    case action_request_s::ACTION_TOGGLE_ARMING:
        if (actuator_armed_.armed) {
            return disarm(reason) == TransitionResult::Changed;
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
        bool changed = false;
        if (!actuator_armed_.kill) {
            actuator_armed_.kill = true;
            changed = true;
        }
        if (actuator_armed_.armed) {
            changed = disarm(reason) == TransitionResult::Changed || changed;
        }
        if (changed) {
            PX4_WARN("Kill engaged; Rover requires a new Arm action");
        }
        return changed;
    }

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
        return vehicle_status_s::ARM_DISARM_REASON_RC_SWITCH;
    default:
        return vehicle_status_s::ARM_DISARM_REASON_COMMAND_INTERNAL;
    }
}

} // namespace dima::modules::safety

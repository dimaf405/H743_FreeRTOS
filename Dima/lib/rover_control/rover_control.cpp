/*
 * Rover controller core adapted from PX4-Autopilot v1.17.0 rover-control
 * algorithms. See PX4_NOTICE.md for source and BSD-3-Clause attribution.
 */
#include "Dima/lib/rover_control/rover_control.hpp"

namespace dima::lib::rover_control {
namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
}

bool RoverControllerChain::finite(float value) noexcept
{
    return __builtin_isfinite(value);
}

float RoverControllerChain::clamp(float value, float lower, float upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

float RoverControllerChain::wrap_pi(float angle) noexcept
{
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

float RoverControllerChain::slew(float current, float target, float accel,
                                 float decel, float dt_s) noexcept
{
    const bool same_direction = (current == 0.0F) || (target == 0.0F) ||
                                ((current > 0.0F) == (target > 0.0F));
    const bool increasing = same_direction &&
                            (__builtin_fabsf(target) > __builtin_fabsf(current));
    const float max_delta = (increasing ? accel : decel) * dt_s;
    return current + clamp(target - current, -max_delta, max_delta);
}

bool RoverControllerChain::valid_pid(const PidParams &params) noexcept
{
    return finite(params.proportional) && params.proportional >= 0.0F &&
           finite(params.integral) && params.integral >= 0.0F &&
           finite(params.derivative) && params.derivative >= 0.0F &&
           finite(params.feedforward) &&
           finite(params.integral_limit) && params.integral_limit >= 0.0F &&
           finite(params.output_limit) && params.output_limit > 0.0F;
}

RoverControlOutput RoverControllerChain::zero_output(
    RoverControlStatus status) noexcept
{
    RoverControlOutput output{};
    output.status = status;
    return output;
}

float RoverControllerChain::update_pid(const PidParams &params,
                                       PidState &state,
                                       float setpoint,
                                       float measurement,
                                       float dt_s,
                                       bool freeze_integrator,
                                       bool &limited) noexcept
{
    const float error = setpoint - measurement;
    const float derivative = state.initialized
                                 ? (error - state.previous_error) / dt_s
                                 : 0.0F;
    if (!freeze_integrator) {
        state.integral = clamp(state.integral + params.integral * error * dt_s,
                               -params.integral_limit,
                               params.integral_limit);
    }
    state.previous_error = error;
    state.initialized = true;

    const float raw = params.proportional * error + state.integral +
                      params.derivative * derivative +
                      params.feedforward * setpoint;
    const float bounded = clamp(raw, -params.output_limit, params.output_limit);
    limited = raw != bounded;
    return bounded;
}

bool RoverControllerChain::configure(const RoverControlParams &params) noexcept
{
    const bool valid = valid_pid(params.yaw_rate_pid) &&
                       valid_pid(params.speed_pid) &&
                       finite(params.heading_p) && params.heading_p > 0.0F &&
                       finite(params.max_yaw_rate_rad_s) && params.max_yaw_rate_rad_s > 0.0F &&
                       finite(params.max_yaw_accel_rad_s2) && params.max_yaw_accel_rad_s2 > 0.0F &&
                       finite(params.max_yaw_decel_rad_s2) && params.max_yaw_decel_rad_s2 > 0.0F &&
                       finite(params.max_speed_m_s) && params.max_speed_m_s > 0.0F &&
                       finite(params.max_accel_m_s2) && params.max_accel_m_s2 > 0.0F &&
                       finite(params.max_decel_m_s2) && params.max_decel_m_s2 > 0.0F &&
                       finite(params.spot_turn_enter_rad) &&
                       finite(params.spot_turn_exit_rad) &&
                       params.spot_turn_enter_rad > params.spot_turn_exit_rad &&
                       params.spot_turn_exit_rad > 0.0F &&
                       params.spot_turn_enter_rad <= kPi &&
                       finite(params.state_timeout_s) && params.state_timeout_s > 0.0F;
    configured_ = valid;
    if (valid) {
        params_ = params;
    }
    reset_state();
    return valid;
}

void RoverControllerChain::reset_state() noexcept
{
    yaw_rate_state_ = {};
    speed_state_ = {};
    limited_yaw_rate_ = 0.0F;
    limited_speed_ = 0.0F;
    spot_turn_active_ = false;
    steering_limited_previous_ = false;
    throttle_limited_previous_ = false;
}

void RoverControllerChain::reset() noexcept
{
    reset_state();
}

RoverControlOutput RoverControllerChain::update(
    const RoverControlCommand &command,
    const RoverVehicleState &state,
    uint64_t now_us,
    float dt_s) noexcept
{
    if (!configured_) {
        return zero_output(RoverControlStatus::InvalidParams);
    }
    if (command.mode == RoverControlMode::Stop) {
        reset_state();
        return zero_output(RoverControlStatus::Stopped);
    }
    if (!finite(dt_s) || dt_s <= 0.0F || dt_s > params_.state_timeout_s) {
        reset_state();
        return zero_output(RoverControlStatus::InvalidDt);
    }
    if (!finite(command.speed_m_s) ||
        (command.mode == RoverControlMode::SpeedAndYawRate &&
         !finite(command.yaw_rate_rad_s)) ||
        (command.mode == RoverControlMode::SpeedAndHeading &&
         !finite(command.heading_rad))) {
        reset_state();
        return zero_output(RoverControlStatus::InvalidCommand);
    }
    if (now_us < state.timestamp_us ||
        (now_us - state.timestamp_us) >
            static_cast<uint64_t>(params_.state_timeout_s * 1000000.0F) ||
        !state.yaw_rate_valid || !state.speed_valid ||
        !finite(state.yaw_rate_rad_s) || !finite(state.body_speed_m_s) ||
        (command.mode == RoverControlMode::SpeedAndHeading &&
         (!state.yaw_valid || !finite(state.yaw_rad)))) {
        reset_state();
        return zero_output(RoverControlStatus::InvalidState);
    }

    float target_speed = clamp(command.speed_m_s,
                               -params_.max_speed_m_s,
                               params_.max_speed_m_s);
    float target_yaw_rate = 0.0F;

    if (command.mode == RoverControlMode::SpeedAndHeading) {
        const float yaw_error = wrap_pi(command.heading_rad - state.yaw_rad);
        const float yaw_error_abs = __builtin_fabsf(yaw_error);
        if (!spot_turn_active_ && yaw_error_abs >= params_.spot_turn_enter_rad) {
            spot_turn_active_ = true;
        } else if (spot_turn_active_ && yaw_error_abs <= params_.spot_turn_exit_rad) {
            spot_turn_active_ = false;
        }
        if (spot_turn_active_) {
            target_speed = 0.0F;
        }
        target_yaw_rate = clamp(params_.heading_p * yaw_error,
                                -params_.max_yaw_rate_rad_s,
                                params_.max_yaw_rate_rad_s);
    } else if (command.mode == RoverControlMode::SpeedAndYawRate) {
        spot_turn_active_ = false;
        target_yaw_rate = clamp(command.yaw_rate_rad_s,
                                -params_.max_yaw_rate_rad_s,
                                params_.max_yaw_rate_rad_s);
    } else {
        reset_state();
        return zero_output(RoverControlStatus::InvalidCommand);
    }

    limited_speed_ = slew(limited_speed_, target_speed,
                          params_.max_accel_m_s2,
                          params_.max_decel_m_s2, dt_s);
    limited_yaw_rate_ = slew(limited_yaw_rate_, target_yaw_rate,
                             params_.max_yaw_accel_rad_s2,
                             params_.max_yaw_decel_rad_s2, dt_s);

    bool steering_pid_limited = false;
    bool throttle_pid_limited = false;
    float steering = update_pid(params_.yaw_rate_pid, yaw_rate_state_,
                                limited_yaw_rate_, state.yaw_rate_rad_s, dt_s,
                                steering_limited_previous_, steering_pid_limited);
    float throttle = update_pid(params_.speed_pid, speed_state_,
                                limited_speed_, state.body_speed_m_s, dt_s,
                                throttle_limited_previous_, throttle_pid_limited);
    steering = clamp(steering, -1.0F, 1.0F);
    throttle = clamp(throttle, -1.0F, 1.0F);

    bool throttle_limited = throttle_pid_limited;
    bool steering_limited = steering_pid_limited;
    const float steering_abs = __builtin_fabsf(steering);
    const float throttle_allowance = 1.0F - steering_abs;
    const float bounded_throttle = clamp(throttle, -throttle_allowance,
                                         throttle_allowance);
    if (bounded_throttle != throttle) {
        throttle_limited = true;
        throttle = bounded_throttle;
    }

    const float left = clamp(throttle + steering, -1.0F, 1.0F);
    const float right = clamp(throttle - steering, -1.0F, 1.0F);
    if (left != throttle + steering || right != throttle - steering) {
        steering_limited = true;
    }

    steering_limited_previous_ = steering_limited;
    throttle_limited_previous_ = throttle_limited;

    RoverControlOutput output{};
    output.requested_speed_m_s = limited_speed_;
    output.requested_yaw_rate_rad_s = limited_yaw_rate_;
    output.normalized_throttle = throttle;
    output.normalized_steering = steering;
    output.left_normalized = left;
    output.right_normalized = right;
    output.spot_turn_active = spot_turn_active_;
    output.throttle_limited = throttle_limited;
    output.steering_limited = steering_limited;
    output.valid = true;
    output.status = RoverControlStatus::Ok;
    return output;
}

} // namespace dima::lib::rover_control

#include "DifferentialDrive.hpp"

#include <cmath>

namespace dima::lib::rover {
namespace {

constexpr float kZeroThreshold = 1.0e-6F;

} // namespace

bool DifferentialDrive::finite(float value) noexcept
{
    return std::isfinite(value);
}

float DifferentialDrive::clamp(float value, float lower, float upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

float DifferentialDrive::interpolate(float from, float to,
                                     float ratio) noexcept
{
    return from + (to - from) * clamp(ratio, 0.0F, 1.0F);
}

float DifferentialDrive::signed_unit(float value) noexcept
{
    return value < 0.0F ? -1.0F : 1.0F;
}

float DifferentialDrive::expo_curve(float magnitude, float expo) noexcept
{
    const float input = clamp(magnitude, 0.0F, 1.0F);
    if (std::fabs(expo) <= kZeroThreshold) {
        return input;
    }

    const float one_minus_expo = 1.0F - expo;
    const float radicand = one_minus_expo * one_minus_expo +
                           4.0F * expo * input;
    if (radicand < 0.0F) {
        return input;
    }
    const float shaped =
        ((expo - 1.0F) + std::sqrt(radicand)) / (2.0F * expo);
    return clamp(shaped, 0.0F, 1.0F);
}

bool DifferentialDrive::valid_config(
    const DifferentialDriveConfig &config) noexcept
{
    return finite(config.steering_throttle_mix) &&
           config.steering_throttle_mix >= 0.0F &&
           config.steering_throttle_mix <= 1.0F &&
           finite(config.throttle_min) && config.throttle_min >= 0.0F &&
           finite(config.throttle_max) && config.throttle_max > 0.0F &&
           config.throttle_max <= 1.0F &&
           config.throttle_min <= config.throttle_max &&
           finite(config.throttle_slew_rate) &&
           config.throttle_slew_rate >= 0.0F &&
           config.throttle_slew_rate <= 10.0F &&
           finite(config.reversal_delay_s) &&
           config.reversal_delay_s >= 0.0F &&
           config.reversal_delay_s <= 1.0F &&
           finite(config.throttle_expo) &&
           config.throttle_expo >= -1.0F &&
           config.throttle_expo <= 1.0F &&
           finite(config.thrust_asymmetry) &&
           config.thrust_asymmetry >= 1.0F &&
           config.thrust_asymmetry <= 10.0F &&
           finite(config.arm_ramp_s) && config.arm_ramp_s >= 0.0F &&
           config.arm_ramp_s <= 5.0F;
}

void DifferentialDrive::prioritize_axes(float &longitudinal, float &steering,
                                        float priority,
                                        float lower_motor_limit) noexcept
{
    const float lower = clamp(lower_motor_limit, -1.0F, -0.1F);
    longitudinal = clamp(longitudinal, lower, 1.0F);
    steering = clamp(steering, -1.0F, 1.0F);
    const float steering_magnitude = std::fabs(steering);
    const float upper_output = longitudinal + steering_magnitude;
    const float lower_output = longitudinal - steering_magnitude;
    const float saturation = std::fmax(upper_output, lower_output / lower);
    if (saturation <= 1.0F) {
        return;
    }

    const float fair_scale = 1.0F / saturation;
    const float fair_longitudinal = longitudinal * fair_scale;
    const float fair_steering = steering * fair_scale;

    const float throttle_priority_longitudinal = longitudinal;
    const float throttle_priority_range = std::fmax(
        0.0F,
        std::fmin(1.0F - longitudinal, longitudinal - lower));
    const float throttle_priority_steering = clamp(
        steering, -throttle_priority_range, throttle_priority_range);

    const float maximum_steering = 0.5F * (1.0F - lower);
    const float steering_priority_steering = clamp(
        steering, -maximum_steering, maximum_steering);
    const float steering_priority_magnitude =
        std::fabs(steering_priority_steering);
    const float steering_priority_longitudinal = clamp(
        longitudinal, lower + steering_priority_magnitude,
        1.0F - steering_priority_magnitude);

    const float bounded_priority = clamp(priority, 0.0F, 1.0F);
    if (bounded_priority <= 0.5F) {
        const float blend = bounded_priority * 2.0F;
        longitudinal = interpolate(throttle_priority_longitudinal,
                                   fair_longitudinal, blend);
        steering = interpolate(throttle_priority_steering, fair_steering,
                               blend);
    } else {
        const float blend = (bounded_priority - 0.5F) * 2.0F;
        longitudinal = interpolate(fair_longitudinal,
                                   steering_priority_longitudinal, blend);
        steering = interpolate(fair_steering, steering_priority_steering,
                               blend);
    }
}

bool DifferentialDrive::configure(
    const DifferentialDriveConfig &config) noexcept
{
    if (!valid_config(config)) {
        configured_ = false;
        reset();
        return false;
    }
    config_ = config;
    configured_ = true;
    reset();
    return true;
}

float DifferentialDrive::shape_motor(float command) const noexcept
{
    float bounded = clamp(command, -1.0F, 1.0F);
    if (std::fabs(bounded) <= kZeroThreshold) {
        return 0.0F;
    }

    if (bounded < 0.0F) {
        bounded = clamp(bounded * config_.thrust_asymmetry, -1.0F, 0.0F);
    }

    const float shaped = expo_curve(std::fabs(bounded), config_.throttle_expo);
    const float magnitude = config_.throttle_min +
                            shaped * (config_.throttle_max -
                                      config_.throttle_min);
    return signed_unit(bounded) * clamp(magnitude, config_.throttle_min,
                                        config_.throttle_max);
}

float DifferentialDrive::apply_reversal_delay(
    float command, ReversalState &state, std::uint64_t now_us) const noexcept
{
    if (std::fabs(command) <= kZeroThreshold) {
        return 0.0F;
    }
    if (now_us < state.last_output_time_us) {
        state = {};
        return 0.0F;
    }

    const std::uint64_t delay_us = static_cast<std::uint64_t>(
        config_.reversal_delay_s * 1000000.0F);
    const bool changes_direction = state.last_nonzero * command < 0.0F;
    if (changes_direction && state.have_output &&
        now_us - state.last_output_time_us < delay_us) {
        return 0.0F;
    }

    state.last_nonzero = command;
    state.last_output_time_us = now_us;
    state.have_output = true;
    return command;
}

DifferentialDriveOutput DifferentialDrive::update(
    float longitudinal, float steering, bool manual_source, bool armed,
    std::uint64_t now_us, float dt_s) noexcept
{
    if (!configured_ || !armed || !finite(longitudinal) ||
        !finite(steering) || !finite(dt_s) || dt_s <= 0.0F || dt_s > 0.05F) {
        reset();
        return {};
    }

    if (!armed_) {
        armed_ = true;
        armed_since_us_ = now_us;
        limited_longitudinal_ = 0.0F;
        right_reversal_ = {};
        left_reversal_ = {};
    }
    if (now_us < armed_since_us_) {
        reset();
        return {};
    }

    const float target = clamp(longitudinal, -1.0F, 1.0F);
    if (!manual_source && std::fabs(target) <= kZeroThreshold) {
        // Navigation 的零命令来自停车确认、SpotTurning 或 Hold，是安全状态而非
        // 普通调速阶跃；必须立即清掉上一周期 slew，禁止残余纵向量混入原地转向。
        limited_longitudinal_ = 0.0F;
    } else if (config_.throttle_slew_rate > 0.0F) {
        const float maximum_change = config_.throttle_slew_rate * dt_s;
        limited_longitudinal_ += clamp(target - limited_longitudinal_,
                                       -maximum_change, maximum_change);
    } else {
        limited_longitudinal_ = target;
    }

    float adjusted_steering = clamp(steering, -1.0F, 1.0F);
    if (manual_source && config_.reverse_steering_in_manual &&
        limited_longitudinal_ < 0.0F) {
        adjusted_steering = -adjusted_steering;
    }

    if (!manual_source) {
        // Speed PI 在进入本层前已按 1-|steering| 限制输出；MOT_SLEW_RATE 的历史
        // 状态也必须重新投影到本周期可行域，否则减速转弯时会再次饱和并挤占转向。
        const float longitudinal_limit =
            std::fmax(0.0F, 1.0F - std::fabs(adjusted_steering));
        limited_longitudinal_ = clamp(limited_longitudinal_,
                                      -longitudinal_limit,
                                      longitudinal_limit);
    }

    float mixed_longitudinal = limited_longitudinal_;
    const float lower_motor_limit = -1.0F / config_.thrust_asymmetry;
    // Manual 继续使用 RD_STR_THR_MIX；Navigation 固定 steering priority=1，
    // 因为 Heading/YawRate 闭环的抗扰稳定性不能被人工油门优先参数削弱。
    const float axis_priority = manual_source
                                    ? config_.steering_throttle_mix
                                    : 1.0F;
    prioritize_axes(mixed_longitudinal, adjusted_steering,
                     axis_priority, lower_motor_limit);

    float right = shape_motor(mixed_longitudinal - adjusted_steering);
    float left = shape_motor(mixed_longitudinal + adjusted_steering);

    float arm_scale = 1.0F;
    if (config_.arm_ramp_s > 0.0F) {
        const float elapsed_s = static_cast<float>(now_us - armed_since_us_) *
                                1.0e-6F;
        arm_scale = clamp(elapsed_s / config_.arm_ramp_s, 0.0F, 1.0F);
    }
    right *= arm_scale;
    left *= arm_scale;

    right = apply_reversal_delay(right, right_reversal_, now_us);
    left = apply_reversal_delay(left, left_reversal_, now_us);

    return DifferentialDriveOutput{right, left, true};
}

void DifferentialDrive::reset() noexcept
{
    right_reversal_ = {};
    left_reversal_ = {};
    limited_longitudinal_ = 0.0F;
    armed_since_us_ = 0U;
    armed_ = false;
}

} // namespace dima::lib::rover

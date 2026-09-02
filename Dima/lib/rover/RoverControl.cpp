/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RoverControl.hpp"

#include <cfloat>
#include <cmath>

#include <mathlib/TrajMath.hpp>

namespace dima::lib::rover {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kMaximumControlDtS = 0.05F;
constexpr float kIntegralLimit = 1.0F;

bool finite(float value) noexcept
{
    return std::isfinite(value);
}

float clamp(float value, float lower, float upper) noexcept
{
    return value < lower ? lower : (value > upper ? upper : value);
}

float wrap_pi(float angle) noexcept
{
    float wrapped = std::fmod(angle + kPi, 2.0F * kPi);
    if (wrapped < 0.0F) {
        wrapped += 2.0F * kPi;
    }
    return wrapped - kPi;
}

bool valid_dt(float dt_s) noexcept
{
    return finite(dt_s) && dt_s > 0.0F && dt_s <= kMaximumControlDtS;
}

float slew_linear(float state, float target, float increasing_rate,
                  float decreasing_rate, float measured,
                  float dt_s) noexcept
{
    const bool increasing = std::fabs(target) >= std::fabs(measured);
    const float rate = increasing ? increasing_rate : decreasing_rate;
    if (rate <= FLT_EPSILON) {
        return target;
    }
    const float maximum_change = rate * dt_s;
    const float candidate = state +
        clamp(target - state, -maximum_change, maximum_change);

    // 测量值已经更接近目标时直接跟随目标，避免限速状态落在真实车辆之后，
    // 否则控制器可能因“追赶过期 setpoint”产生不必要的反向输出。
    return std::fabs(candidate - measured) > std::fabs(target - measured)
               ? target
               : candidate;
}

float conditional_pi(float error, float feedforward, float proportional_gain,
                     float integral_gain, float output_limit, float dt_s,
                     float &integral) noexcept
{
    const float bounded_limit = clamp(output_limit, 0.0F, 1.0F);
    const float candidate_integral = clamp(
        integral + integral_gain * error * dt_s,
        -kIntegralLimit, kIntegralLimit);
    const float candidate = feedforward + proportional_gain * error +
                            candidate_integral;

    // 条件积分反饱和：若候选输出已越过同方向边界，就冻结积分；若误差方向能
    // 把输出拉回可行域则继续积分。这样 FF 已饱和时也能自然消除积累误差。
    const bool drives_further_positive =
        candidate > bounded_limit && error > 0.0F;
    const bool drives_further_negative =
        candidate < -bounded_limit && error < 0.0F;
    if (!drives_further_positive && !drives_further_negative) {
        integral = candidate_integral;
    }
    return clamp(feedforward + proportional_gain * error + integral,
                 -bounded_limit, bounded_limit);
}

bool valid_speed_config(const SpeedControlConfig &config) noexcept
{
    return finite(config.proportional_gain) &&
           config.proportional_gain >= 0.0F &&
           finite(config.integral_gain) && config.integral_gain >= 0.0F &&
           finite(config.speed_at_full_throttle_m_s) &&
           config.speed_at_full_throttle_m_s > FLT_EPSILON &&
           finite(config.acceleration_limit_m_s2) &&
           config.acceleration_limit_m_s2 >= -1.0F &&
           finite(config.deceleration_limit_m_s2) &&
           config.deceleration_limit_m_s2 >= -1.0F &&
           finite(config.measurement_threshold_m_s) &&
           config.measurement_threshold_m_s >= 0.0F;
}

bool valid_yaw_rate_config(const YawRateControlConfig &config) noexcept
{
    return finite(config.proportional_gain) &&
           config.proportional_gain >= 0.0F &&
           finite(config.integral_gain) && config.integral_gain >= 0.0F &&
           finite(config.yaw_rate_correction) &&
           config.yaw_rate_correction > 0.0F &&
           finite(config.wheel_track_m) && config.wheel_track_m > 0.0F &&
           finite(config.speed_at_full_throttle_m_s) &&
           config.speed_at_full_throttle_m_s > FLT_EPSILON &&
           finite(config.yaw_rate_limit_rad_s) &&
           config.yaw_rate_limit_rad_s > 0.0F &&
           finite(config.yaw_acceleration_limit_rad_s2) &&
           config.yaw_acceleration_limit_rad_s2 >= -1.0F &&
           finite(config.yaw_deceleration_limit_rad_s2) &&
           config.yaw_deceleration_limit_rad_s2 >= -1.0F &&
           finite(config.measurement_threshold_rad_s) &&
           config.measurement_threshold_rad_s >= 0.0F;
}

} // namespace

bool SpeedController::configure(const SpeedControlConfig &config) noexcept
{
    configured_ = valid_speed_config(config);
    config_ = configured_ ? config : SpeedControlConfig{};
    reset();
    return configured_;
}

PiControlOutput SpeedController::update(float speed_setpoint_m_s,
                                        float measured_speed_m_s,
                                        float output_limit,
                                        float dt_s) noexcept
{
    if (!configured_ || !finite(speed_setpoint_m_s) ||
        !finite(measured_speed_m_s) || !finite(output_limit) ||
        output_limit < 0.0F || !valid_dt(dt_s)) {
        reset();
        return {};
    }

    const float measured =
        std::fabs(measured_speed_m_s) > config_.measurement_threshold_m_s
            ? measured_speed_m_s
            : 0.0F;
    if (std::fabs(speed_setpoint_m_s) <= FLT_EPSILON) {
        // RO_SPEED_TH 只定义“实测速度是否可信为非零”，不能拿来截断正在从
        // 0 上升的 slew setpoint；否则 100 Hz 下每步小于阈值时控制器永远无法
        // 起步。原始设定明确为零时才同时清积分和 slew，停车/原地转向不会
        // 继承上一周期的 longitudinal。
        reset();
        return {0.0F, 0.0F, 0.0F, true};
    }
    if (!initialized_) {
        // 首帧从实测速度建立 slew 状态，避免模式切换时从固定零值产生阶跃。
        adjusted_setpoint_ = measured;
        initialized_ = true;
    }
    adjusted_setpoint_ = slew_linear(
        adjusted_setpoint_, speed_setpoint_m_s,
        config_.acceleration_limit_m_s2,
        config_.deceleration_limit_m_s2, measured, dt_s);

    // 速度前馈把 m/s 线性映射为归一化油门，再由 PI 修正载荷、坡度与电压扰动。
    const float feedforward = adjusted_setpoint_ /
                              config_.speed_at_full_throttle_m_s;
    const float error = adjusted_setpoint_ - measured;
    const float output = conditional_pi(
        error, feedforward, config_.proportional_gain,
        config_.integral_gain, output_limit, dt_s, integral_);
    return {output, adjusted_setpoint_, integral_, true};
}

void SpeedController::reset() noexcept
{
    adjusted_setpoint_ = 0.0F;
    integral_ = 0.0F;
    initialized_ = false;
}

bool HeadingController::configure(const HeadingControlConfig &config) noexcept
{
    configured_ = finite(config.proportional_gain) &&
                  config.proportional_gain > 0.0F &&
                  finite(config.yaw_rate_limit_rad_s) &&
                  config.yaw_rate_limit_rad_s > 0.0F;
    config_ = configured_ ? config : HeadingControlConfig{};
    reset();
    return configured_;
}

HeadingControlOutput HeadingController::update(float yaw_setpoint_rad,
                                               float measured_yaw_rad,
                                               float dt_s) noexcept
{
    if (!configured_ || !finite(yaw_setpoint_rad) ||
        !finite(measured_yaw_rad) || !valid_dt(dt_s)) {
        reset();
        return {};
    }

    const float target = wrap_pi(yaw_setpoint_rad);
    const float measured = wrap_pi(measured_yaw_rad);
    if (!initialized_) {
        adjusted_yaw_setpoint_ = measured;
        initialized_ = true;
    }

    // 航向 setpoint 以最大 yaw-rate 沿最短角距离移动，避免跨 +/-pi 或
    // Pure Pursuit 交点跳变时给角速度内环一个瞬时大阶跃。
    const float maximum_step = config_.yaw_rate_limit_rad_s * dt_s;
    const float target_delta = wrap_pi(target - adjusted_yaw_setpoint_);
    adjusted_yaw_setpoint_ = wrap_pi(
        adjusted_yaw_setpoint_ +
        clamp(target_delta, -maximum_step, maximum_step));
    if (std::fabs(wrap_pi(adjusted_yaw_setpoint_ - measured)) >
        std::fabs(wrap_pi(target - measured))) {
        adjusted_yaw_setpoint_ = target;
    }

    const float error = wrap_pi(adjusted_yaw_setpoint_ - measured);
    const float yaw_rate_setpoint = clamp(
        config_.proportional_gain * error,
        -config_.yaw_rate_limit_rad_s,
        config_.yaw_rate_limit_rad_s);
    return {yaw_rate_setpoint, adjusted_yaw_setpoint_, error, true};
}

void HeadingController::reset() noexcept
{
    adjusted_yaw_setpoint_ = 0.0F;
    initialized_ = false;
}

bool YawRateController::configure(
    const YawRateControlConfig &config) noexcept
{
    configured_ = valid_yaw_rate_config(config);
    config_ = configured_ ? config : YawRateControlConfig{};
    reset();
    return configured_;
}

PiControlOutput YawRateController::update(
    float yaw_rate_setpoint_rad_s, float measured_yaw_rate_rad_s,
    float output_limit, float dt_s) noexcept
{
    if (!configured_ || !finite(yaw_rate_setpoint_rad_s) ||
        !finite(measured_yaw_rate_rad_s) || !finite(output_limit) ||
        output_limit < 0.0F || !valid_dt(dt_s)) {
        reset();
        return {};
    }

    const float measured =
        std::fabs(measured_yaw_rate_rad_s) >
                config_.measurement_threshold_rad_s
            ? measured_yaw_rate_rad_s
            : 0.0F;
    const float bounded_target = clamp(
        yaw_rate_setpoint_rad_s, -config_.yaw_rate_limit_rad_s,
        config_.yaw_rate_limit_rad_s);
    // 与 PX4 DifferentialRateControl 一致，RO_YAW_RATE_TH 同时把极小目标和
    // 极小测量解释为零，避免接近目标航向时由 FF/PI 维持轮端抖动。
    const float target =
        std::fabs(bounded_target) > config_.measurement_threshold_rad_s
            ? bounded_target
            : 0.0F;
    if (std::fabs(target) <= FLT_EPSILON) {
        // yaw-rate 测量死区同样不能吞掉小步 slew；只有目标明确为零才复位，
        // 使 StoppingForTurn 的 steering 为零，并让 SpotTurning 从干净状态启动。
        reset();
        return {0.0F, 0.0F, 0.0F, true};
    }
    if (!initialized_) {
        adjusted_setpoint_ = measured;
        initialized_ = true;
    }
    adjusted_setpoint_ = slew_linear(
        adjusted_setpoint_, target,
        config_.yaw_acceleration_limit_rad_s2,
        config_.yaw_deceleration_limit_rad_s2, measured, dt_s);

    // 差速运动学：左右轮半速度差 delta_v = yaw_rate*track/2；再除以
    // 满油门速度得到归一化 steering，CORR 补偿滑移与轮胎摩擦。
    const float feedforward =
        adjusted_setpoint_ * config_.wheel_track_m *
        config_.yaw_rate_correction /
        (2.0F * config_.speed_at_full_throttle_m_s);
    const float error = adjusted_setpoint_ - measured;
    const float output = conditional_pi(
        error, feedforward, config_.proportional_gain,
        config_.integral_gain, output_limit, dt_s, integral_);
    return {output, adjusted_setpoint_, integral_, true};
}

void YawRateController::reset() noexcept
{
    adjusted_setpoint_ = 0.0F;
    integral_ = 0.0F;
    initialized_ = false;
}

bool DrivingStateMachine::configure(
    const DrivingStateConfig &config) noexcept
{
    configured_ = finite(config.turn_to_drive_yaw_error_rad) &&
        config.turn_to_drive_yaw_error_rad > 0.0F &&
        finite(config.drive_to_turn_yaw_error_rad) &&
        config.drive_to_turn_yaw_error_rad >
            config.turn_to_drive_yaw_error_rad &&
        config.drive_to_turn_yaw_error_rad <= kPi &&
        finite(config.stopped_speed_threshold_m_s) &&
        config.stopped_speed_threshold_m_s >= 0.0F;
    config_ = configured_ ? config : DrivingStateConfig{};
    reset();
    return configured_;
}

DrivingStateOutput DrivingStateMachine::update(
    float yaw_error_rad, float adjusted_speed_setpoint_m_s,
    float measured_speed_m_s) noexcept
{
    if (!configured_ || !finite(yaw_error_rad) ||
        !finite(adjusted_speed_setpoint_m_s) ||
        !finite(measured_speed_m_s)) {
        reset();
        return {};
    }

    const float absolute_error = std::fabs(wrap_pi(yaw_error_rad));
    switch (state_) {
    case DrivingState::Driving:
        if (absolute_error > config_.drive_to_turn_yaw_error_rad) {
            // 大航向误差先进入只停车状态；本周期禁止 yaw-rate/steering，不能让
            // 尚在前进的车立即出现左右轮反转。
            state_ = DrivingState::StoppingForTurn;
        }
        break;

    case DrivingState::StoppingForTurn:
        if (absolute_error < config_.turn_to_drive_yaw_error_rad) {
            // 路径更新已消除大误差时取消原地转向，避免无意义地等到静止。
            state_ = DrivingState::Driving;
        } else if (std::fabs(adjusted_speed_setpoint_m_s) <=
                       config_.stopped_speed_threshold_m_s &&
                   std::fabs(measured_speed_m_s) <=
                       config_.stopped_speed_threshold_m_s) {
            // 设定和实测同时收敛到零才允许反向打轮；单看命令为零不足以证明
            // 车辆已经停住，单看 GNSS 低速也不足以证明 slew 已完成。
            state_ = DrivingState::SpotTurning;
        }
        break;

    case DrivingState::SpotTurning:
        if (absolute_error < config_.turn_to_drive_yaw_error_rad) {
            state_ = DrivingState::Driving;
        }
        break;
    }

    const bool translation_enabled = state_ == DrivingState::Driving;
    const bool heading_enabled = state_ != DrivingState::StoppingForTurn;
    return {state_, translation_enabled, heading_enabled, true};
}

void DrivingStateMachine::reset() noexcept
{
    state_ = DrivingState::Driving;
}

BodySpeedMeasurement measure_body_speed(float velocity_north_m_s,
                                        float velocity_east_m_s,
                                        float yaw_rad,
                                        float threshold_m_s) noexcept
{
    if (!finite(velocity_north_m_s) || !finite(velocity_east_m_s) ||
        !finite(yaw_rad) || !finite(threshold_m_s) || threshold_m_s < 0.0F) {
        return {};
    }

    // NED -> Body-FR：前向轴在 NED 中为 [cos(yaw), sin(yaw)]，右向轴为
    // [-sin(yaw), cos(yaw)]。差速车用前向分量符号和二维模长保留倒车语义。
    const float cosine = std::cos(yaw_rad);
    const float sine = std::sin(yaw_rad);
    const float forward = cosine * velocity_north_m_s +
                          sine * velocity_east_m_s;
    const float lateral = -sine * velocity_north_m_s +
                          cosine * velocity_east_m_s;
    const float magnitude = std::hypot(forward, lateral);
    // 严格使用 sign(v_body_x)*hypot(v_body_x,v_body_y)：纯横向运动时前向
    // 符号为 0，不能伪装成正向车速并永久阻塞停车确认/SpotTurning。
    const float direction = forward > 0.0F
                                ? 1.0F
                                : (forward < 0.0F ? -1.0F : 0.0F);
    const float speed = magnitude > threshold_m_s
                            ? direction * magnitude
                            : 0.0F;
    return {speed, forward, lateral, true};
}

WaypointSpeedPlan plan_waypoint_speed(
    float distance_to_waypoint_m, float acceptance_radius_m,
    float cruise_speed_m_s, float arrival_speed_m_s, float speed_limit_m_s,
    float jerk_limit_m_s3, float deceleration_limit_m_s2) noexcept
{
    if (!finite(distance_to_waypoint_m) || distance_to_waypoint_m < 0.0F ||
        !finite(acceptance_radius_m) || acceptance_radius_m <= 0.0F ||
        !finite(cruise_speed_m_s) || cruise_speed_m_s < 0.0F ||
        !finite(arrival_speed_m_s) || arrival_speed_m_s < 0.0F ||
        !finite(speed_limit_m_s) || speed_limit_m_s <= 0.0F ||
        !finite(jerk_limit_m_s3) || jerk_limit_m_s3 <= FLT_EPSILON ||
        !finite(deceleration_limit_m_s2) ||
        deceleration_limit_m_s2 <= FLT_EPSILON) {
        return {};
    }

    const bool inside = distance_to_waypoint_m <= acceptance_radius_m;
    const float braking_distance = std::fmax(
        distance_to_waypoint_m - acceptance_radius_m, 0.0F);
    if (inside) {
        return {arrival_speed_m_s, braking_distance, true, true};
    }

    // 直接复用现有 TrajMath 的保守 jerk 延迟模型：
    // vf^2 = vi^2 - 2*a*(x - vi*2*a/j)。输入已经在函数入口验证为正且有限，
    // 不在 Rover 控制核再维护第二份公式实现。
    const float braking_limited =
        math::trajectory::computeMaxSpeedFromDistance(
            jerk_limit_m_s3, deceleration_limit_m_s2,
            braking_distance, arrival_speed_m_s);
    const float requested = std::fmin(cruise_speed_m_s, speed_limit_m_s);
    return {std::fmin(requested, braking_limited), braking_distance,
            false, true};
}

float reduce_speed_for_heading_error(float speed_setpoint_m_s,
                                     float heading_error_rad,
                                     float speed_at_full_throttle_m_s,
                                     float reduction_gain) noexcept
{
    if (!finite(speed_setpoint_m_s) || !finite(heading_error_rad) ||
        !finite(speed_at_full_throttle_m_s) ||
        speed_at_full_throttle_m_s <= 0.0F || !finite(reduction_gain)) {
        return 0.0F;
    }
    if (reduction_gain < 0.0F) {
        return speed_setpoint_m_s;
    }

    // course_error 从 [0,pi] 归一化到 [0,1]，再按 RO_SPEED_RED 缩小
    // 最大可用速度；只改幅值并保留前后方向符号。
    const float normalized_error = clamp(
        std::fabs(wrap_pi(heading_error_rad)) / kPi, 0.0F, 1.0F);
    const float reduction = clamp(
        reduction_gain * normalized_error, 0.0F, 1.0F);
    const float maximum_speed = speed_at_full_throttle_m_s *
                                (1.0F - reduction);
    return clamp(speed_setpoint_m_s, -maximum_speed, maximum_speed);
}

} // namespace dima::lib::rover

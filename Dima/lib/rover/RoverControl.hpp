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

#pragma once

namespace dima::lib::rover {

struct SpeedControlConfig {
    float proportional_gain;
    float integral_gain;
    float speed_at_full_throttle_m_s;
    float acceleration_limit_m_s2;
    float deceleration_limit_m_s2;
    float measurement_threshold_m_s;
};

struct YawRateControlConfig {
    float proportional_gain;
    float integral_gain;
    float yaw_rate_correction;
    float wheel_track_m;
    float speed_at_full_throttle_m_s;
    float yaw_rate_limit_rad_s;
    float yaw_acceleration_limit_rad_s2;
    float yaw_deceleration_limit_rad_s2;
    float measurement_threshold_rad_s;
};

struct HeadingControlConfig {
    float proportional_gain;
    float yaw_rate_limit_rad_s;
};

struct PiControlOutput {
    float output;
    float adjusted_setpoint;
    float integral;
    bool valid;
};

struct HeadingControlOutput {
    float yaw_rate_setpoint_rad_s;
    float adjusted_yaw_setpoint_rad;
    float yaw_error_rad;
    bool valid;
};

struct BodySpeedMeasurement {
    float speed_m_s;
    float forward_m_s;
    float lateral_m_s;
    bool valid;
};

struct WaypointSpeedPlan {
    float speed_setpoint_m_s;
    float braking_distance_m;
    bool waypoint_inside_acceptance;
    bool valid;
};

enum class DrivingState {
    Driving,
    StoppingForTurn,
    SpotTurning,
};

struct DrivingStateConfig {
    float turn_to_drive_yaw_error_rad;
    float drive_to_turn_yaw_error_rad;
    float stopped_speed_threshold_m_s;
};

struct DrivingStateOutput {
    DrivingState state;
    bool translation_enabled;
    bool heading_control_enabled;
    bool valid;
};

/** 带条件积分反饱和的速度 PI，输出为归一化 longitudinal。 */
class SpeedController {
public:
    bool configure(const SpeedControlConfig &config) noexcept;
    PiControlOutput update(float speed_setpoint_m_s,
                           float measured_speed_m_s,
                           float output_limit,
                           float dt_s) noexcept;
    void reset() noexcept;

private:
    SpeedControlConfig config_{};
    float adjusted_setpoint_{0.0F};
    float integral_{0.0F};
    bool configured_{false};
    bool initialized_{false};
};

/** 航向角 P 外环，输出为受限的 yaw-rate setpoint。 */
class HeadingController {
public:
    bool configure(const HeadingControlConfig &config) noexcept;
    HeadingControlOutput update(float yaw_setpoint_rad,
                                float measured_yaw_rad,
                                float dt_s) noexcept;
    void reset() noexcept;

private:
    HeadingControlConfig config_{};
    float adjusted_yaw_setpoint_{0.0F};
    bool configured_{false};
    bool initialized_{false};
};

/** 带差速运动学前馈和条件积分反饱和的 yaw-rate PI。 */
class YawRateController {
public:
    bool configure(const YawRateControlConfig &config) noexcept;
    PiControlOutput update(float yaw_rate_setpoint_rad_s,
                           float measured_yaw_rate_rad_s,
                           float output_limit,
                           float dt_s) noexcept;
    void reset() noexcept;

private:
    YawRateControlConfig config_{};
    float adjusted_setpoint_{0.0F};
    float integral_{0.0F};
    bool configured_{false};
    bool initialized_{false};
};

/** Driving/停车确认/原地转向滞回状态机。 */
class DrivingStateMachine {
public:
    bool configure(const DrivingStateConfig &config) noexcept;
    DrivingStateOutput update(float yaw_error_rad,
                              float adjusted_speed_setpoint_m_s,
                              float measured_speed_m_s) noexcept;
    void reset() noexcept;
    DrivingState state() const noexcept { return state_; }

private:
    DrivingStateConfig config_{};
    DrivingState state_{DrivingState::Driving};
    bool configured_{false};
};

BodySpeedMeasurement measure_body_speed(float velocity_north_m_s,
                                        float velocity_east_m_s,
                                        float yaw_rad,
                                        float threshold_m_s) noexcept;

WaypointSpeedPlan plan_waypoint_speed(float distance_to_waypoint_m,
                                      float acceptance_radius_m,
                                      float cruise_speed_m_s,
                                      float arrival_speed_m_s,
                                      float speed_limit_m_s,
                                      float jerk_limit_m_s3,
                                      float deceleration_limit_m_s2) noexcept;

float reduce_speed_for_heading_error(float speed_setpoint_m_s,
                                     float heading_error_rad,
                                     float speed_at_full_throttle_m_s,
                                     float reduction_gain) noexcept;

} // namespace dima::lib::rover

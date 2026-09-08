// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "PurePursuit.hpp"
#include "RoverControl.hpp"

namespace dima::lib::rover {

// 单航段的无 I/O 输入，位置/速度为 NED 米与米每秒，yaw 为弧度，dt_s 为秒。
// maximum_speed 是全油门速度模型；本段规划上限仍是 cruise_speed。
struct SegmentGuidanceInput {
    Position2f start{}, target{}, position{};
    float velocity_north{}, velocity_east{}, yaw{}, dt_s{};
    float acceptance_radius{}, cruise_speed{}, arrival_speed{};
    float jerk{}, deceleration{}, maximum_speed{}, speed_reduction{}, speed_threshold{};
};

struct SegmentGuidanceOutput {
    PurePursuitOutput pursuit{};
    HeadingControlOutput heading{};
    WaypointSpeedPlan speed_plan{};
    DrivingStateOutput driving{};
    BodySpeedMeasurement measured_speed{};
    float heading_error{}, speed_setpoint{}, yaw_rate_setpoint{};
    bool valid{false};
};

// 复用调用者已配置的控制器及其连续状态，不复制控制核，也不推进 Mission。
// 只有 valid 时才能消费物理设定；失效后的安全停机/复位由各模式入口负责。
SegmentGuidanceOutput update_segment(PurePursuit &pursuit,
                                      HeadingController &heading,
                                      DrivingStateMachine &driving,
                                      const SegmentGuidanceInput &input) noexcept;

} // namespace dima::lib::rover

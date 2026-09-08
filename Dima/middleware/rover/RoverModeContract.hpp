#pragma once

#include "vehicle_status.hpp"
#include "vehicle_control_mode.hpp"

namespace dima::middleware::rover::mode_contract {

// 控制层与执行器层共用逐字段精确模式投影；Armed、快照一致性和失鲜门控
// 仍由各消费者独立检查，不因共享谓词而合并安全状态。
inline bool manual_projection(const vehicle_control_mode_s &control) noexcept
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

inline bool navigation_projection(const vehicle_control_mode_s &control) noexcept
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

} // namespace dima::middleware::rover::mode_contract

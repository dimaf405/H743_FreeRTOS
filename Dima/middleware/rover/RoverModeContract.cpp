#include "RoverModeContract.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::middleware::rover::mode_contract {

bool supported(std::uint8_t state) noexcept
{
    return state < 32U && (kImplementedMask & (1UL << state)) != 0U;
}

bool rc_slot_supported(std::int32_t state) noexcept
{
    return state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
           state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
           state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
}

bool auto_calibration(std::uint8_t state) noexcept
{
    return state == vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
}

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

bool navigation_projection(const vehicle_control_mode_s &control) noexcept
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

bool calibration_projection_common(const vehicle_control_mode_s &control) noexcept
{
    return auto_calibration(control.source_id) && control.flag_control_auto_enabled &&
        !control.flag_control_manual_enabled && !control.flag_control_offboard_enabled &&
        !control.flag_control_position_enabled &&
        !control.flag_control_altitude_enabled && !control.flag_control_climb_rate_enabled &&
        !control.flag_control_acceleration_enabled && !control.flag_control_attitude_enabled &&
        !control.flag_control_allocation_enabled &&
        !control.flag_control_termination_enabled && !control.flag_multicopter_position_control_enabled;
}

bool calibration_open_loop_projection(const vehicle_control_mode_s &control) noexcept
{
    // 开环校准只拥有 normalized axes；任何 Mission 闭环标志都必须保持关闭。
    return calibration_projection_common(control) &&
        !control.flag_control_velocity_enabled && !control.flag_control_rates_enabled;
}

bool calibration_closed_loop_projection(const vehicle_control_mode_s &control) noexcept
{
    // 闭环校准只借用真实 Speed/YawRate 内环。Position/Heading/高度等外环仍关闭，
    // 因而不能用这份投影冒充 AUTO Mission。
    return calibration_projection_common(control) &&
        control.flag_control_velocity_enabled && control.flag_control_rates_enabled;
}

bool calibration_projection(const vehicle_control_mode_s &control) noexcept
{
    // Commander、差速层和 PWM 后端只接受上述两种逐字段精确投影；一开一关或
    // 夹带其他控制标志的过渡帧均 fail-closed。
    return calibration_projection_common(control) &&
           control.flag_control_velocity_enabled == control.flag_control_rates_enabled;
}

} // namespace dima::middleware::rover::mode_contract

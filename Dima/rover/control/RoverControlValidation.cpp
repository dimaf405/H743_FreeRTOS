#include "RoverControlValidation.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::rover::control {

bool speed_control_parameters_valid(
    const dima::lib::rover::SpeedControlConfig &speed) noexcept
{
    return std::isfinite(speed.proportional_gain) && speed.proportional_gain >= 0.0F &&
        std::isfinite(speed.integral_gain) && speed.integral_gain >= 0.0F &&
        speed.proportional_gain + speed.integral_gain > 0.0F &&
        std::isfinite(speed.speed_at_full_throttle_m_s) &&
        speed.speed_at_full_throttle_m_s > 0.0F &&
        std::isfinite(speed.acceleration_limit_m_s2) &&
        speed.acceleration_limit_m_s2 > 0.0F &&
        std::isfinite(speed.deceleration_limit_m_s2) &&
        speed.deceleration_limit_m_s2 > 0.0F &&
        std::isfinite(speed.measurement_threshold_m_s) &&
        speed.measurement_threshold_m_s >= 0.0F;
}

bool yaw_rate_control_parameters_valid(
    const dima::lib::rover::YawRateControlConfig &yaw_rate) noexcept
{
    return std::isfinite(yaw_rate.proportional_gain) &&
        yaw_rate.proportional_gain >= 0.0F &&
        std::isfinite(yaw_rate.integral_gain) && yaw_rate.integral_gain >= 0.0F &&
        yaw_rate.proportional_gain + yaw_rate.integral_gain > 0.0F &&
        std::isfinite(yaw_rate.yaw_rate_correction) &&
        yaw_rate.yaw_rate_correction > 0.0F &&
        std::isfinite(yaw_rate.wheel_track_m) && yaw_rate.wheel_track_m > 0.0F &&
        std::isfinite(yaw_rate.speed_at_full_throttle_m_s) &&
        yaw_rate.speed_at_full_throttle_m_s > 0.0F &&
        std::isfinite(yaw_rate.yaw_rate_limit_rad_s) &&
        yaw_rate.yaw_rate_limit_rad_s > 0.0F &&
        std::isfinite(yaw_rate.yaw_acceleration_limit_rad_s2) &&
        yaw_rate.yaw_acceleration_limit_rad_s2 > 0.0F &&
        std::isfinite(yaw_rate.yaw_deceleration_limit_rad_s2) &&
        yaw_rate.yaw_deceleration_limit_rad_s2 > 0.0F &&
        std::isfinite(yaw_rate.measurement_threshold_rad_s) &&
        yaw_rate.measurement_threshold_rad_s >= 0.0F;
}

} // namespace dima::rover::control

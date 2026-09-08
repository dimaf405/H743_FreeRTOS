#include "CalibrationFence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::lib::rover::calibration {

CalibrationSessionLimits session_limits(float cruise, float fallback, float motor) noexcept
{
    CalibrationSessionLimits limits{};
    // 只有精确 0/历史 -1 表示未配置；NaN 或任意小负数不能意外授权满输出。
    const bool unset = cruise == 0.0F || cruise == -1.0F;
    if (!std::isfinite(cruise) || (!unset && (cruise <= 0.0F || cruise > 100.0F)) ||
        !std::isfinite(fallback) || fallback < 0.1F || fallback > 100.0F ||
        !std::isfinite(motor) || motor < 0.05F || motor > 1.0F) return limits;
    limits.speed_m_s = unset ? fallback : cruise;
    limits.motor_output = motor;
    limits.full_output_probe = unset;
    limits.valid = true;
    return limits;
}

CircleFenceResult evaluate_circle(const CircleFence &fence, double latitude,
    double longitude, float error, float age) noexcept
{
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    CircleFenceResult result{unavailable, unavailable, unavailable, false, false, false};
    if (!std::isfinite(fence.latitude_deg) || std::fabs(fence.latitude_deg) >= 85.0 ||
        !std::isfinite(fence.longitude_deg) || std::fabs(fence.longitude_deg) > 180.0 ||
        !std::isfinite(latitude) || std::fabs(latitude) >= 85.0 ||
        !std::isfinite(longitude) || std::fabs(longitude) > 180.0 ||
        !std::isfinite(error) || error <= 0.0F || error > 0.15F ||
        !std::isfinite(fence.origin_error_m) || fence.origin_error_m <= 0.0F || fence.origin_error_m > 0.15F ||
        !std::isfinite(fence.radius_m) || fence.radius_m < 1.0F || fence.radius_m > 100.0F ||
        !std::isfinite(fence.speed_limit_m_s) || fence.speed_limit_m_s <= 0.0F || fence.speed_limit_m_s > 100.0F ||
        !std::isfinite(age) || age < 0.0F || age > 0.3F) return result;

    // WGS84 子午圈/卯酉圈曲率半径；经纬度差保持 double。100 m 局部圆内
    // 用椭球尺度，避免球半径近似误差吞掉厘米级定位及安全余量。
    constexpr double radians = 0.01745329251994329577;
    constexpr double semi_major = 6378137.0;
    constexpr double eccentricity_squared = 0.0066943799901413165;
    const double phi = fence.latitude_deg * radians;
    const double denominator = 1.0 - eccentricity_squared * std::sin(phi) * std::sin(phi);
    const double root = std::sqrt(denominator);
    const double prime_vertical = semi_major / root;
    const double meridian = semi_major * (1.0 - eccentricity_squared) / (denominator * root);
    const double north = (latitude - fence.latitude_deg) * radians * meridian;
    const double east = std::remainder(longitude - fence.longitude_deg, 360.0) * radians * prime_vertical * std::cos(phi);
    result.distance_m = static_cast<float>(std::hypot(north, east));
    result.north_m = static_cast<float>(north);
    result.east_m = static_cast<float>(east);
    result.position_valid = std::isfinite(result.distance_m);
    result.inside = result.position_valid && result.distance_m < fence.radius_m;
    if (!std::isfinite(fence.stop_distance_m) || fence.stop_distance_m <= 0.0F || fence.stop_distance_m > 100.0F) return result;

    // 保守使用入场冻结的速度上限，而非某拍较小速度。300 ms 留给接收机
    // 解算/串行链路，100 ms 请求 TTL + 两个 10 ms 控制/输出周期另计；样本
    // 当前年龄再加入。停车参数覆盖请求发出后的执行器响应及惯性，不能拿
    // RO_DECEL_LIM 命令限值替代实测上界。实板须证明链路/停车满足这些预算。
    result.margin_m = 0.5F + 3.0F * (fence.origin_error_m + error) +
        fence.speed_limit_m_s * (0.30F + 0.10F + 0.01F + 0.01F + age) + fence.stop_distance_m;
    result.working_radius_m = std::max(0.0F, fence.radius_m - result.margin_m);
    result.can_stop = result.inside && result.distance_m < result.working_radius_m;
    return result;
}

} // namespace dima::lib::rover::calibration

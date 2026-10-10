#include "CalibrationFence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::lib::rover::calibration {

CalibrationSessionLimits session_limits(float cruise, float motor) noexcept
{
    CalibrationSessionLimits limits{};
    // 巡航速度直接承担会话速度上限（默认 1 m/s）；只有精确 0/历史 -1 表示
    // 未配置，此时没有任何可冻结的安全上限，判无效并拒绝自动校准运动。
    const bool unset = cruise == 0.0F || cruise == -1.0F;
    if (unset || !std::isfinite(cruise) || cruise <= 0.0F || cruise > 100.0F ||
        !std::isfinite(motor) || motor < 0.05F || motor > 1.0F) return limits;
    limits.speed_m_s = cruise;
    limits.motor_output = motor;
    limits.valid = true;
    return limits;
}

namespace {

CircleFenceResult evaluate_position(const CircleFence &fence, double latitude,
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
    // margin=定位误差本身；停车距离由调用方按实际速度的实测制动模型附加
    // （2026-09-30 用户确认：删除 0.5 固定容差与 speed_limit×0.44s 链路预留带——
    //  在 1.0 m/s 限速下两者合计约 0.96m，属过保护；围栏语义回到"当前速度的
    //  实测停车模型能否在边界前停住"的连续判定，样本年龄只做新鲜度门）。
    result.margin_m = fence.origin_error_m + error;
    return result;
}

} // namespace

CircleFenceResult evaluate_circle(const CircleFence &fence, double latitude,
    double longitude, float error, float age, float stop) noexcept
{
    auto result = evaluate_position(fence, latitude, longitude, error, age);
    if (!result.position_valid || !std::isfinite(stop) || stop < 0.0F) return result;
    result.margin_m += stop;
    result.working_radius_m = std::max(0.0F, fence.radius_m - result.margin_m);
    result.can_stop = result.inside && result.distance_m < result.working_radius_m;
    return result;
}

CircleFenceResult evaluate_braking_probe(const CircleFence &reference, float length,
    double latitude, double longitude, float error, float age) noexcept
{
    // 首次全输出试验尚无停车观测，只核验冻结的几何范围与有效定位。
    // 不保留已经退役的低速探测目标、3σ速度地板或0.3减速度假设。
    auto result = evaluate_position(reference, latitude, longitude, error, age);
    if (!result.position_valid || !std::isfinite(length) || length < 1.0F || length > 100.0F) return {};
    result.working_radius_m = length;
    result.inside = result.distance_m < length;
    result.can_stop = result.inside; // 在FENCE_BRAKING_PROBE中只表示试验范围，非停车能力证明。
    return result;
}

CircleFenceResult evaluate_straight(const CircleFence &reference, float length,
    double latitude, double longitude, float error, float age, float stop) noexcept
{
    auto result = evaluate_circle(reference, latitude, longitude, error, age, stop);
    if (!result.position_valid || !std::isfinite(result.margin_m) ||
        !std::isfinite(stop) || stop < 0.0F ||
        !std::isfinite(length) || length < 1.0F || length > 100.0F) return {};
    // 直线独立限制距入场点的位移，不用 RO_CAL_RADIUS 裁短航段。端点外的
    // 0.5 m 用于到点/掉头容差，误差、链路与理论制动余量仍用于越界判定。
    result.working_radius_m = length + 0.5F;
    result.inside = result.distance_m < result.working_radius_m + result.margin_m;
    result.can_stop = result.distance_m < result.working_radius_m;
    return result;
}

} // namespace dima::lib::rover::calibration

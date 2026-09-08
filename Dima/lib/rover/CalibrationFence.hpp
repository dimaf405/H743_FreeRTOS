#pragma once

namespace dima::lib::rover::calibration {

// 固定全球中心与定位点的圆形约束；不代表车身外廓，也不依赖 EKF 本地原点。
struct CircleFence {
    double latitude_deg{}, longitude_deg{};
    float origin_error_m{}, radius_m{}, stop_distance_m{};
    float speed_limit_m_s{};
};

struct CalibrationSessionLimits {
    float speed_m_s{}, motor_output{};
    bool full_output_probe{}, valid{};
};

// 只解释入场快照，不读实时参数；参数整定不能回头修改本次运动授权的边界。
CalibrationSessionLimits session_limits(float entry_cruise_m_s,
    float fallback_m_s, float motor_maximum) noexcept;

struct CircleFenceResult {
    float distance_m{}, margin_m{}, working_radius_m{};
    bool position_valid{}, inside{}, can_stop{};
    float north_m{}, east_m{};
};

CircleFenceResult evaluate_circle(const CircleFence &fence, double latitude_deg,
    double longitude_deg, float position_error_m, float position_age_s) noexcept;

} // namespace dima::lib::rover::calibration

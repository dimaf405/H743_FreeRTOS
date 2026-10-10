#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::modes {

// —— Fence helper（围栏与停车距离，去状态化）——————————————————————————
// 围栏几何只依赖阶段语义：直线族（STRAIGHT 的 straight_active 域）使用走廊
// 界，其余运动面使用圆形界；制动观测的轮次界由 Braking 子状态表达，不再
// 读顶层 status_.state。停车距离随实测制动候选更新。

bool AutoCalibrationMode::motion_configuration_valid() const noexcept
{
    // 包络=入场冻结的 MOT_THR_MAX（0.05–1.0，与 session_limits 同域校验）；
    // 纵/转向均无静态输出上限，其余申报项维持原有范围。
    return config_valid_ && std::isfinite(config_.track) && config_.track > 0.0F && config_.track <= 5.0F &&
        std::isfinite(config_.radius) && config_.radius >= 1.0F && config_.radius <= 100.0F &&
        std::isfinite(config_.straight_distance) && config_.straight_distance >= 1.0F && config_.straight_distance <= 100.0F &&
        std::isfinite(config_.deceleration) && config_.deceleration >= -1.0F && config_.deceleration <= 100.0F &&
        std::isfinite(config_.motor_reversal_delay_s) && config_.motor_reversal_delay_s >= 0.0F && config_.motor_reversal_delay_s <= 1.0F &&
        std::isfinite(config_.motor_slew_rate) && config_.motor_slew_rate >= 0.0F && config_.motor_slew_rate <= 10.0F &&
        dima::lib::rover::calibration::session_limits(fence_.speed_limit_m_s, config_.motor_maximum).valid &&
        dima::lib::rover::calibration::session_limits(fence_.speed_limit_m_s, status_.session_motor_limit).valid &&
        config_.gps_control >= 0 && config_.gps_control <= 15;
}

void AutoCalibrationMode::capture_fence(std::uint64_t now) noexcept
{
    const auto &gps = gps_sub_.get();
    const auto limits = dima::lib::rover::calibration::session_limits(
        config_.entry_cruise, config_.motor_maximum);
    fence_ = {gps.latitude_deg, gps.longitude_deg, gps.eph, config_.radius, config_.deceleration, limits.speed_m_s};
    status_.session_speed_limit_m_s = limits.speed_m_s;
    status_.session_motor_limit = limits.motor_output;
    status_.entry_cruise_speed_m_s = config_.entry_cruise;
    status_.fence_radius_m = config_.radius;
    status_.straight_distance_m = config_.straight_distance;
    status_.entry_deceleration_m_s2 = config_.deceleration;
    status_.straight_active = true;
    status_.fence_stop_distance_m = NAN; // 新会话尚无停车观测，旧配置不是实测距离。
    status_.fence_center_valid = fresh(gps.timestamp_sample, now, 300000ULL) && gps.device_id != 0U &&
        gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED && std::isfinite(gps.latitude_deg) &&
        std::fabs(gps.latitude_deg) < 85.0 && std::isfinite(gps.longitude_deg) && std::fabs(gps.longitude_deg) <= 180.0 &&
        std::isfinite(gps.eph) && gps.eph > 0.0F && gps.eph <= 0.15F;
    if (!status_.fence_center_valid) return;
    status_.fence_device_id = gps.device_id;
    status_.fence_center_timestamp = gps.timestamp_sample;
    status_.fence_latitude_deg = gps.latitude_deg;
    status_.fence_longitude_deg = gps.longitude_deg;
    status_.fence_origin_error_m = gps.eph;
    update_fence(now);
}

dima::lib::rover::calibration::CircleFenceResult AutoCalibrationMode::fence_result(std::uint64_t now) const noexcept
{
    const auto &gps = gps_sub_.get();
    if (!status_.fence_center_valid || !fresh(gps.timestamp_sample, now, 300000ULL) ||
        gps.device_id != status_.fence_device_id || gps.fix_type != sensor_gps_s::FIX_TYPE_RTK_FIXED) return {};
    auto reference = fence_;
    const float age = 1.0e-6F * static_cast<float>(now - gps.timestamp_sample);
    const bool probe = status_.braking_model_generation == 0U || status_.braking_full_output ||
        session_.substate == PhaseSubstate::Braking;
    // 停车项按实际车速的实测制动模型计算（2026-09-30 用户确认）：围栏是
    // "从当前速度能否在边界前停住"的逐拍连续判定，不用冻结限速预扣余量。
    // 旋转时 GNSS 天线速度本身已含杆臂摆动，无需按姿态分支取限速。
    const float stop = braking_distance(ground_speed());
    auto result = probe
        ? dima::lib::rover::calibration::evaluate_braking_probe(reference, config_.straight_distance,
            gps.latitude_deg, gps.longitude_deg, gps.eph, age)
        : status_.straight_active
        ? dima::lib::rover::calibration::evaluate_straight(reference, config_.straight_distance,
            gps.latitude_deg, gps.longitude_deg, gps.eph, age, stop)
        : dima::lib::rover::calibration::evaluate_circle(reference, gps.latitude_deg, gps.longitude_deg, gps.eph, age, stop);
    const bool returning = session_.substate == PhaseSubstate::Return ||
        (status_.state == Status::STATE_STRAIGHT && !session_.straight_outward);
    if (returning && result.position_valid) {
        // s=p·e：e在return_prepare冻结为起点→返程出发点。s<0才表示越过起点，
        // 径向距离无法区分前后；此边界同时覆盖返程中的正式制动与事务等待。
        const float along = result.north_m * std::cos(return_axis_rad_) +
            result.east_m * std::sin(return_axis_rad_);
        const float tolerance = fence_.origin_error_m + gps.eph;
        if (!std::isfinite(along) ||
            (along < -tolerance && result.distance_m > 0.5F + tolerance)) {
            result.inside = result.can_stop = false;
        }
    }
    return result;
}

void AutoCalibrationMode::update_fence(std::uint64_t now) noexcept
{
    const auto result = fence_result(now);
    // 显示巡航/当前速度对应的名义停车距离；实测距离仍在 braking_stop_distance_m。
    status_.fence_stop_distance_m = braking_distance(std::max(fence_.speed_limit_m_s, ground_speed()));
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    status_.fence_distance_m = result.position_valid ? result.distance_m : unavailable;
    status_.fence_margin_m = result.position_valid ? result.margin_m : unavailable;
    status_.fence_working_radius_m = result.position_valid ? result.working_radius_m : unavailable;
    // 去状态化：FENCE_BRAKING_PROBE 语义 = 尚无停车模型，或正处于内联制动
    // 观测子状态（仅有界探测范围，不代表已有停车模型）。
    status_.fence_state = !status_.fence_center_valid ? Status::FENCE_UNAVAILABLE
        : !result.position_valid ? Status::FENCE_POSITION_LOST
        : !result.inside ? Status::FENCE_OUTSIDE
        : status_.braking_model_generation == 0U || status_.braking_full_output || session_.substate == PhaseSubstate::Braking ? Status::FENCE_BRAKING_PROBE
        : !std::isfinite(status_.fence_stop_distance_m) ? Status::FENCE_DECELERATION_INVALID
        : result.can_stop ? Status::FENCE_SAFE : Status::FENCE_STOP_MARGIN;
}

bool AutoCalibrationMode::prepare_straight(std::uint64_t now) noexcept
{
    status_.straight_active = true;
    const auto result = fence_result(now);
    // 独立参数决定直线目标长度；不扣圆形围栏和停车距离，也不设固定五米
    // 门槛。航向/速度拟合仍各自验证有效样本，空间短不等于校准成功。
    leg_distance_ = config_.straight_distance;
    return result.can_stop && std::isfinite(leg_distance_) && leg_distance_ >= 1.0F;
}

float AutoCalibrationMode::sensor_lever_arm() const noexcept
{
    // 2026-09-30 用户确认：优先用本会话原地旋转实测的天线杆臂（旋转圆半径，
    // 两方向取大）。参数和（GPS+IMU 偏置模长之和）是保守上界代理——实测
    // 0.30m vs 参数和 0.82m，高估 2.7 倍直接吃掉方形尺寸。未测得（静止
    // 会话/旋转未满整圈）时回退参数和，语义不变。
    if (measured_lever_valid_) return measured_lever_m_;
    float gx{}, gy{}, gz{}, ix{}, iy{}, iz{};
    px4::AtomicTransaction atomic;
    if (param_get(param_handle(dima::params::EKF2_GPS_POS_X), &gx) != 0 ||
        param_get(param_handle(dima::params::EKF2_GPS_POS_Y), &gy) != 0 ||
        param_get(param_handle(dima::params::EKF2_GPS_POS_Z), &gz) != 0 ||
        param_get(param_handle(dima::params::EKF2_IMU_POS_X), &ix) != 0 ||
        param_get(param_handle(dima::params::EKF2_IMU_POS_Y), &iy) != 0 ||
        param_get(param_handle(dima::params::EKF2_IMU_POS_Z), &iz) != 0) return NAN;
    return std::hypot(std::hypot(gx, gy), gz) + std::hypot(std::hypot(ix, iy), iz);
}

} // namespace dima::rover::modes

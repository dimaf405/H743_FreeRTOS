#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"
#include <uORB/topics/auto_calibration_status_labels.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::modes {

bool AutoCalibrationMode::motion_configuration_valid() const noexcept
{
    return config_valid_ && std::isfinite(config_.throttle) && config_.throttle >= 0.10F && config_.throttle <= 0.40F &&
        std::isfinite(config_.steering) && config_.steering >= 0.10F && config_.steering <= 0.35F &&
        std::isfinite(config_.distance) && config_.distance >= 5.0F && config_.distance <= 30.0F &&
        std::isfinite(config_.track) && config_.track > 0.0F && config_.track <= 5.0F &&
        std::isfinite(config_.radius) && config_.radius >= 1.0F && config_.radius <= 100.0F &&
        std::isfinite(config_.stop_distance) && config_.stop_distance > 0.0F && config_.stop_distance <= 100.0F &&
        std::isfinite(fence_.speed_limit_m_s) && fence_.speed_limit_m_s > 0.0F &&
        std::isfinite(status_.session_motor_limit) && status_.session_motor_limit >= 0.05F && status_.session_motor_limit <= 1.0F &&
        config_.gps_control >= 0 && config_.gps_control <= 15;
}

void AutoCalibrationMode::capture_fence(std::uint64_t now) noexcept
{
    const auto &gps = gps_sub_.get();
    const auto limits = dima::lib::rover::calibration::session_limits(
        config_.entry_cruise, config_.fallback_speed, config_.motor_maximum);
    fence_ = {gps.latitude_deg, gps.longitude_deg, gps.eph, config_.radius, config_.stop_distance, limits.speed_m_s};
    status_.session_speed_limit_m_s = limits.speed_m_s;
    status_.session_motor_limit = limits.motor_output;
    status_.entry_cruise_speed_m_s = config_.entry_cruise;
    status_.full_output_requested = limits.valid && limits.full_output_probe;
    status_.fence_radius_m = config_.radius;
    status_.fence_stop_distance_m = config_.stop_distance;
    status_.fence_center_valid = fresh(gps.timestamp_sample, now, 300000ULL) && gps.device_id != 0U &&
        gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED && std::isfinite(gps.latitude_deg) &&
        std::fabs(gps.latitude_deg) < 85.0 && std::isfinite(gps.longitude_deg) && std::fabs(gps.longitude_deg) <= 180.0 &&
        std::isfinite(gps.eph) && gps.eph > 0.0F && gps.eph <= 0.15F;
    if (!status_.fence_center_valid) return;
    gps_device_id_ = status_.fence_device_id = gps.device_id;
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
    return dima::lib::rover::calibration::evaluate_circle(fence_, gps.latitude_deg, gps.longitude_deg,
        gps.eph, 1.0e-6F * static_cast<float>(now - gps.timestamp_sample));
}

void AutoCalibrationMode::update_fence(std::uint64_t now) noexcept
{
    const auto result = fence_result(now);
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    status_.fence_distance_m = result.position_valid ? result.distance_m : unavailable;
    status_.fence_margin_m = result.position_valid ? result.margin_m : unavailable;
    status_.fence_working_radius_m = result.position_valid ? result.working_radius_m : unavailable;
    status_.fence_state = !status_.fence_center_valid ? Status::FENCE_UNAVAILABLE
        : !result.position_valid ? Status::FENCE_POSITION_LOST
        : !result.inside ? Status::FENCE_OUTSIDE
        : !(fence_.stop_distance_m > 0.0F) ? Status::FENCE_STOP_UNKNOWN
        : result.can_stop ? Status::FENCE_SAFE : Status::FENCE_STOP_MARGIN;
}

bool AutoCalibrationMode::prepare_straight(std::uint64_t now) noexcept
{
    const auto result = fence_result(now);
    // 初始 yaw 安装偏置未知，按任意方向最坏长度分配空间，不拿阵列 heading
    // 冒充车头方向做射线预测。整段再留 0.5 m 跟踪/掉头余量；不足五米拒绝。
    leg_distance_ = std::min(config_.distance, result.working_radius_m - result.distance_m - 0.5F);
    return result.can_stop && std::isfinite(leg_distance_) && leg_distance_ >= 5.0F;
}

void AutoCalibrationMode::report_status(std::uint64_t now) noexcept
{
    if (status_.session_id == 0U || (last_report_ != 0U && now >= last_report_ && now - last_report_ < 5000000ULL)) return;
    last_report_ = now;
    using namespace dima::generated::uorb_labels;
    // 周期快照走非实时 RAW 路径，不受普通日志级别过滤。USB/QGC 重连不依赖
    // 过期的一次性提示；不使用 [cal] 假装 Sensors 拥有整个组合会话。
    PX4_INFO_RAW("[autocal] %s; %u%%; saved=0x%lx unavailable=0x%lx\n",
        auto_calibration_status_state_name(status_.state), static_cast<unsigned>(status_.progress),
        static_cast<unsigned long>(status_.completed_stages), static_cast<unsigned long>(status_.unavailable_stages));
    PX4_INFO_RAW("[autocal] fence %s %.2f/%.2fm margin %.2fm stop bound %.2fm\n",
        auto_calibration_status_fence_name(status_.fence_state), static_cast<double>(status_.fence_distance_m),
        static_cast<double>(status_.fence_radius_m), static_cast<double>(status_.fence_margin_m),
        static_cast<double>(status_.fence_stop_distance_m));
    PX4_INFO_RAW("[autocal] speed cap %.2fm/s motor cap %.2f; full probe %s\n",
        static_cast<double>(status_.session_speed_limit_m_s), static_cast<double>(status_.session_motor_limit),
        status_.full_output_requested ? "requested (not RPM)" : "off");
    if (status_.full_output_requested && (!status_.active || status_.gains_provisional))
        PX4_INFO_RAW(status_.full_output_reached
            ? "[autocal] allowed command endpoint reached; RPM not measured\n"
            : "[autocal] output coverage partial; RPM not measured\n");
    if (status_.awaiting_arm) {
        PX4_INFO_RAW(status_.session_authorized
            ? "[autocal] automatic continuation; Manual/Disarm cancels\n"
            : "[autocal] Arm ONCE for session; Manual/Disarm cancels\n");
        if (!armed_sub_.get().ready_to_arm) PX4_INFO_RAW("[autocal] motion interlock not ready: check RC/outputs\n");
    }
    if (status_.mag_bootstrap_active || status_.gains_provisional)
        px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] provisional RAM values; NOT saved\n");
    if (status_.provisional_validated_stages != 0U)
        PX4_INFO_RAW("[autocal] validated RAM=0x%lx; awaiting related checks\n",
            static_cast<unsigned long>(status_.provisional_validated_stages));
    if (!status_.active || pending_termination_ || status_.result != Status::RESULT_RUNNING) {
        PX4_INFO_RAW("[autocal] %s: %s\n", auto_calibration_status_result_name(status_.result),
            auto_calibration_status_failure_name(status_.failure_reason));
        if (status_.active) px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] Arm inhibited; stop/rollback pending\n");
        if (transaction_.phase() == CalibrationParameters::Phase::Fault)
            px4_log_raw(_PX4_LOG_LEVEL_ERROR, "[autocal] rollback unconfirmed; interlock latched; reset required\n");
    }
}

void AutoCalibrationMode::service_motion_authorization(std::uint64_t now) noexcept
{
    if (!status_.active || pending_termination_ || !selected_) return;
    // 只镜像真实 Commander Armed 边沿，不把选择模式或等待状态当作首次授权。
    // Commander 内部仍独立持有 session grant，并可随时否决后续请求。
    if (armed_.armed() && safety_fresh(now)) status_.session_authorized = true;
    if (status_.session_authorized && status_.awaiting_arm && !armed_.armed() &&
        (last_resume_request_ == 0U || now - last_resume_request_ >= 200000ULL)) {
        last_resume_request_ = now;
        request(auto_calibration_request_s::REQUEST_STAGE_ARM, now);
    }
}

} // namespace dima::rover::modes

#include "RoverDifferential.hpp"

#include "rover/RoverModeContract.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::rover::control {
namespace {

constexpr float kUnavailable = std::numeric_limits<float>::quiet_NaN();

bool fresh(std::uint64_t timestamp, std::uint64_t now,
           std::uint64_t limit) noexcept
{
    return timestamp != 0U && timestamp <= now && now - timestamp <= limit;
}

} // namespace

bool RoverDifferential::calibration_gains_applied(
    std::uint32_t instance, const float (&gains)[4]) const noexcept
{
    px4::AtomicTransaction transaction;
    // 回滚到四个零增益时 Navigation 会按设计变为不可用；这里只确认同一代
    // 参数和值已被真实控制器消费，不能把闭环 readiness 误作回滚成功条件。
    return applied_parameter_valid_ && applied_parameter_instance_ == instance &&
        parameters_.speed.proportional_gain == gains[0] &&
        parameters_.speed.integral_gain == gains[1] &&
        parameters_.yaw_rate.proportional_gain == gains[2] &&
        parameters_.yaw_rate.integral_gain == gains[3];
}

bool RoverDifferential::calibration_control_ready(
    std::uint32_t instance) const noexcept
{
    px4::AtomicTransaction transaction;
    return applied_parameter_valid_ && applied_parameter_instance_ == instance &&
           navigation_parameters_valid_;
}

bool RoverDifferential::calibration_generation_applied(std::uint32_t instance) const noexcept
{
    px4::AtomicTransaction transaction;
    // 全快照成功读取/应用（包括安全抑制）才推进此代次。协调器同时核对
    // 事务内每个实际参数及 set_count；旧零 PI 的回滚不强迫导航 ready。
    return applied_parameter_valid_ && applied_parameter_instance_ == instance;
}

void RoverDifferential::refresh_calibration_fence(
    std::uint64_t now_us) noexcept
{
    const auto &status = calibration_sub_.get();
    if (status.session_id == 0U) return;

    if (!calibration_fence_latched_ ||
        status.session_id != calibration_fence_session_id_) {
        // 新会话只能在 Commander 的完整 Disarmed 同拍快照下锁定一次。阶段
        // Disarm 仍是同一 session，绝不能把车辆当前位置重新定义为圆心。
        if (!status.active || !fresh_disarmed_snapshot(now_us)) return;
        calibration_fence_latched_ = true;
        calibration_fence_session_id_ = status.session_id;
        calibration_fence_center_timestamp_ = status.fence_center_timestamp;
        calibration_fence_device_id_ = status.fence_device_id;
        // ceiling 与圆心在同一个新 session 中只锁存一次；后续阶段 Disarm
        // 只允许应用同值参数代，不能扩大已经由操作者授权的归一化轴范围。
        calibration_session_throttle_ceiling_ =
            parameters_.calibration_throttle_ceiling;
        calibration_session_steering_ceiling_ =
            parameters_.calibration_steering_ceiling;
        const auto limits = dima::lib::rover::calibration::session_limits(parameters_.calibration_entry_cruise,
            parameters_.calibration_fallback_speed, parameters_.drive.throttle_max);
        calibration_entry_cruise_ = parameters_.calibration_entry_cruise;
        calibration_fallback_speed_ = parameters_.calibration_fallback_speed;
        calibration_session_motor_limit_ = limits.motor_output;
        calibration_full_probe_enabled_ = limits.full_output_probe;
        calibration_fence_ = {status.fence_latitude_deg,
                              status.fence_longitude_deg,
                              status.fence_origin_error_m,
                              status.fence_radius_m,
                              status.fence_stop_distance_m, limits.speed_m_s};

        const auto origin = dima::lib::rover::calibration::evaluate_circle(
            calibration_fence_, calibration_fence_.latitude_deg,
            calibration_fence_.longitude_deg,
            calibration_fence_.origin_error_m, 0.0F);
        calibration_fence_valid_ = limits.valid && status.fence_center_valid &&
            status.session_speed_limit_m_s == limits.speed_m_s &&
            status.session_motor_limit == limits.motor_output &&
            status.entry_cruise_speed_m_s == calibration_entry_cruise_ &&
            status.full_output_requested == limits.full_output_probe &&
            fresh(status.timestamp, now_us, 100000ULL) &&
            calibration_fence_center_timestamp_ != 0U &&
            calibration_fence_center_timestamp_ <= status.timestamp &&
            calibration_fence_device_id_ != 0U &&
            valid_calibration_ceiling_snapshot(parameters_) &&
            calibration_fence_.radius_m == parameters_.calibration_radius_m &&
            calibration_fence_.stop_distance_m ==
                parameters_.calibration_stop_distance_m && origin.can_stop;
        return;
    }

    // 同一 session 的任一锁定字段或对应参数只要变化一次，本会话永久失效；
    // 即使稍后改回旧值也不能恢复，必须由新的显式校准会话重新取得圆心。
    if (!calibration_fence_status_unchanged())
        calibration_fence_valid_ = false;
}

bool RoverDifferential::calibration_fence_status_unchanged() const noexcept
{
    const auto &status = calibration_sub_.get();
    return calibration_fence_latched_ && status.fence_center_valid &&
        status.session_id == calibration_fence_session_id_ &&
        status.fence_center_timestamp == calibration_fence_center_timestamp_ &&
        status.fence_device_id == calibration_fence_device_id_ &&
        status.fence_latitude_deg == calibration_fence_.latitude_deg &&
        status.fence_longitude_deg == calibration_fence_.longitude_deg &&
        status.fence_origin_error_m == calibration_fence_.origin_error_m &&
        status.fence_radius_m == calibration_fence_.radius_m &&
        status.fence_stop_distance_m == calibration_fence_.stop_distance_m &&
        status.session_speed_limit_m_s == calibration_fence_.speed_limit_m_s &&
        status.session_motor_limit == calibration_session_motor_limit_ &&
        status.entry_cruise_speed_m_s == calibration_entry_cruise_ &&
        status.full_output_requested == calibration_full_probe_enabled_ &&
        parameters_.drive.throttle_max == calibration_session_motor_limit_ &&
        parameters_.calibration_fallback_speed == calibration_fallback_speed_ &&
        parameters_.calibration_radius_m == calibration_fence_.radius_m &&
        parameters_.calibration_stop_distance_m ==
            calibration_fence_.stop_distance_m &&
        parameters_.calibration_throttle_ceiling ==
            calibration_session_throttle_ceiling_ &&
        parameters_.calibration_steering_ceiling ==
            calibration_session_steering_ceiling_;
}

bool RoverDifferential::calibration_full_output() const noexcept
{
    const auto &status = calibration_sub_.get();
    return calibration_full_probe_enabled_ && status.full_output_requested && !status.closed_loop &&
        status.state == auto_calibration_status_s::STATE_PROFILE_FULL &&
        (status.completed_stages & auto_calibration_status_s::STAGE_RTK) != 0U;
}

bool RoverDifferential::calibration_reverse() const noexcept
{
    return calibration_sub_.get().state == auto_calibration_status_s::STATE_PROFILE_REVERSE ||
        calibration_sub_.get().state == auto_calibration_status_s::STATE_VALIDATE_REVERSE;
}

float RoverDifferential::calibration_motor_limit() const noexcept
{
    return calibration_full_output() ? calibration_session_motor_limit_ : std::min(0.40F, calibration_session_motor_limit_);
}

bool RoverDifferential::calibration_fence_allows_output(
    std::uint64_t now_us) const noexcept
{
    if (!calibration_fence_valid_ ||
        !calibration_fence_status_unchanged()) return false;
    const auto &gps = sensor_gps_;
    if (!fresh(gps.timestamp, now_us, 300000ULL) ||
        !fresh(gps.timestamp_sample, now_us, 300000ULL) ||
        gps.timestamp_sample > gps.timestamp ||
        gps.device_id != calibration_fence_device_id_ ||
        gps.fix_type != sensor_gps_s::FIX_TYPE_RTK_FIXED) return false;
    const auto result = dima::lib::rover::calibration::evaluate_circle(
        calibration_fence_, gps.latitude_deg, gps.longitude_deg, gps.eph,
        static_cast<float>(now_us - gps.timestamp_sample) * 1.0e-6F);
    return result.position_valid && result.inside && result.can_stop;
}

bool RoverDifferential::calibration_input_valid(
    std::uint64_t now_us) const noexcept
{
    const auto &status = calibration_sub_.get();
    const auto &rtk = calibration_rtk_sub_.get();
    const auto &imu = calibration_imu_sub_.get();
    const auto &control = safety_.control_mode;
    const bool projection = status.closed_loop
        ? dima::middleware::rover::mode_contract::calibration_closed_loop_projection(control)
        : dima::middleware::rover::mode_contract::calibration_open_loop_projection(control);
    // 每一条有效校准请求都重新核对 session fence、原始 RTK、IMU 和 Commander
    // 精确投影。协调器的一次 true 不能替代 100 Hz 物理输出边界的独立证明。
    return status.active && status.motion_allowed && status.session_id != 0U &&
        status.session_id == calibration_fence_session_id_ && projection &&
        fresh(status.timestamp, now_us, 100000ULL) &&
        calibration_fence_allows_output(now_us) &&
        fresh(rtk.timestamp, now_us, 300000ULL) &&
        fresh(rtk.timestamp_sample, now_us, 300000ULL) &&
        rtk.timestamp_sample <= rtk.timestamp &&
        rtk.device_id == calibration_fence_device_id_ && rtk.solution_computed &&
        rtk.integer_fixed && rtk.velocity_aligned &&
        std::isfinite(rtk.velocity_north_m_s) &&
        std::isfinite(rtk.velocity_east_m_s) &&
        std::hypot(rtk.velocity_north_m_s, rtk.velocity_east_m_s) <=
            (calibration_reverse() ? std::min(0.3F, calibration_fence_.speed_limit_m_s) : calibration_fence_.speed_limit_m_s) &&
        fresh(imu.timestamp, now_us, 100000ULL) &&
        fresh(imu.timestamp_sample, now_us, 100000ULL) &&
        imu.timestamp_sample <= imu.timestamp &&
        imu.accel_device_id != 0U && imu.gyro_device_id != 0U &&
        imu.delta_angle_dt > 0U && imu.delta_angle_clipping == 0U &&
        imu.delta_velocity_clipping == 0U &&
        std::isfinite(imu.delta_angle[2]) &&
        std::fabs(imu.delta_angle[2] /
                  (1.0e-6F * imu.delta_angle_dt)) <= 0.6F;
}

bool RoverDifferential::control_measurement(
    std::uint64_t now_us, float &speed_m_s, float &yaw_rate_rad_s,
    std::uint64_t &timestamp_sample) const noexcept
{
    if (!navigation_estimator_valid(now_us)) return false;
    const auto speed = dima::lib::rover::measure_body_speed(
        vehicle_local_position_.vx, vehicle_local_position_.vy,
        vehicle_local_position_.heading,
        parameters_.speed.measurement_threshold_m_s);
    const float rate = vehicle_odometry_.angular_velocity[2];
    if (!speed.valid || !std::isfinite(speed.speed_m_s) ||
        !std::isfinite(rate)) return false;
    speed_m_s = speed.speed_m_s;
    yaw_rate_rad_s = rate;
    timestamp_sample = std::min(vehicle_local_position_.timestamp_sample,
                                vehicle_odometry_.timestamp_sample);
    return timestamp_sample != 0U;
}

bool RoverDifferential::publish_control_status(
    std::uint64_t now_us, const rover_motion_request_s *request,
    const ControlCycleFeedback &feedback) noexcept
{
    rover_control_status_s status{};
    status.timestamp = now_us;
    status.timestamp_sample = feedback.measurement_valid
        ? feedback.timestamp_sample : 0U;
    status.parameter_update_instance = applied_parameter_valid_
        ? applied_parameter_instance_ : 0U;
    status.source = request != nullptr ? request->source
                                       : rover_motion_request_s::SOURCE_MANUAL;
    status.session_id = request != nullptr &&
        request->source == rover_motion_request_s::SOURCE_CALIBRATION
        ? calibration_sub_.get().session_id : 0U;
    status.request_sequence = request != nullptr ? request->sequence : 0U;
    status.closed_loop = feedback.closed_loop;
    status.valid = feedback.output_valid && feedback.measurement_valid;
    status.saturated = feedback.output_valid && feedback.saturated;
    status.input_limited = feedback.output_valid && feedback.input_limited;
    status.motor_slew_active = feedback.output_valid && feedback.motor_slew_active;
    status.mixing_limited = feedback.output_valid && feedback.mixing_limited;
    status.shaping_active = feedback.output_valid && feedback.shaping_active;
    status.arm_ramp_active = feedback.output_valid && feedback.arm_ramp_active;
    status.reversal_held = feedback.output_valid && feedback.reversal_held;
    status.safety_output_limited = feedback.output_valid && feedback.safety_output_limited;
    status.safety_slew_active = feedback.output_valid && feedback.safety_slew_active;
    status.speed_setpoint_m_s = feedback.closed_loop
        ? feedback.speed_setpoint_m_s : kUnavailable;
    status.yaw_rate_setpoint_rad_s = feedback.closed_loop
        ? feedback.yaw_rate_setpoint_rad_s : kUnavailable;
    status.speed_m_s = feedback.measurement_valid ? feedback.speed_m_s
                                                   : kUnavailable;
    const auto raw = dima::lib::rover::measure_body_speed(vehicle_local_position_.vx,
        vehicle_local_position_.vy, vehicle_local_position_.heading, 0.0F);
    status.speed_raw_m_s = feedback.measurement_valid && raw.valid ? raw.speed_m_s : kUnavailable;
    status.forward_raw_m_s = feedback.measurement_valid && raw.valid ? raw.forward_m_s : kUnavailable;
    status.lateral_raw_m_s = feedback.measurement_valid && raw.valid ? raw.lateral_m_s : kUnavailable;
    status.yaw_rate_rad_s = feedback.measurement_valid
        ? feedback.yaw_rate_rad_s : kUnavailable;
    status.longitudinal = feedback.output_valid ? feedback.longitudinal
                                                 : kUnavailable;
    status.steering = feedback.output_valid ? feedback.steering : kUnavailable;
    // applied_* 是由最终左右执行器归一化命令反算的轴量，供辨识核对整形映射；
    // 它们不是物理轮速、车速或角速度，不能替代 estimator measurement。
    status.applied_longitudinal = feedback.output_valid
        ? feedback.applied_longitudinal : kUnavailable;
    status.applied_steering = feedback.output_valid
        ? feedback.applied_steering : kUnavailable;
    status.speed_integral = feedback.closed_loop && feedback.output_valid
        ? feedback.speed_integral : kUnavailable;
    status.yaw_rate_integral = feedback.closed_loop && feedback.output_valid
        ? feedback.yaw_rate_integral : kUnavailable;
    return control_status_publication_.publish(status);
}

} // namespace dima::rover::control

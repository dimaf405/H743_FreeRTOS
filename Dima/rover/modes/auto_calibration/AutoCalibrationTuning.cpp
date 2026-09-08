#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "auto/AutoMode.hpp"
#include "control/RoverDifferential.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

bool AutoCalibrationMode::read_tuning_config() noexcept
{
    px4::AtomicTransaction atomic;
    const auto read = [](dima::params parameter, float &value) {
        const auto handle = param_handle(parameter);
        param_set_used(handle);
        return param_get(handle, &value) == 0;
    };
    TuningConfig next{};
    const bool loaded = read(dima::params::RO_SPEED_P, next.inner[0]) && read(dima::params::RO_SPEED_I, next.inner[1]) &&
        read(dima::params::RO_YAW_RATE_P, next.inner[2]) && read(dima::params::RO_YAW_RATE_I, next.inner[3]) &&
        read(dima::params::RO_SPEED_LIM, next.speed_limit) && read(dima::params::RO_SPEED_TH, next.speed_threshold) &&
        read(dima::params::RO_YAW_RATE_LIM, next.rate_limit) && read(dima::params::RO_YAW_RATE_TH, next.rate_threshold) &&
        read(dima::params::RO_ACCEL_LIM, next.acceleration) && read(dima::params::RO_DECEL_LIM, next.deceleration) &&
        read(dima::params::RO_YAW_ACCEL_LIM, next.rate_acceleration) && read(dima::params::RO_YAW_DECEL_LIM, next.rate_deceleration) &&
        read(dima::params::RO_JERK_LIM, next.jerk) && read(dima::params::RO_SPEED_RED, next.speed_reduction) &&
        read(dima::params::RO_YAW_P, next.heading_p) && read(dima::params::NAV_ACC_RAD, next.acceptance) &&
        read(dima::params::PP_LOOKAHD_GAIN, next.pursuit.lookahead_gain) &&
        read(dima::params::PP_LOOKAHD_MIN, next.pursuit.lookahead_min_m) &&
        read(dima::params::PP_LOOKAHD_MAX, next.pursuit.lookahead_max_m) &&
        read(dima::params::RD_TRANS_TRN_DRV, next.driving.turn_to_drive_yaw_error_rad) &&
        read(dima::params::RD_TRANS_DRV_TRN, next.driving.drive_to_turn_yaw_error_rad);
    if (!loaded) return false;
    // RO_YAW_* 的限制沿用 PX4 deg/s、deg/s^2；PI 本身使用 rad/s 误差，
    // 因此 P/I 不作角度换算。RD_TRANS_* 已是 rad，不能再转换一次。
    next.rate_limit *= kRadians;
    next.rate_threshold *= kRadians;
    next.rate_acceleration *= kRadians;
    next.rate_deceleration *= kRadians;
    next.driving.stopped_speed_threshold_m_s = next.speed_threshold;
    tuning_config_ = next;
    return true;
}

float AutoCalibrationMode::body_yaw() const noexcept
{
    const auto &q = attitude_sub_.get().q;
    return std::atan2(2.0F * (q[0] * q[3] + q[1] * q[2]),
        1.0F - 2.0F * (q[2] * q[2] + q[3] * q[3]));
}

void AutoCalibrationMode::start_tuning(std::uint64_t now) noexcept
{
    if (pending_termination_ || tuning_started_) { finish(now); return; }
    tuning_started_ = true;
    if ((status_.completed_stages & Status::STAGE_RTK) == 0U || !read_tuning_config()) {
        fail_tuning(Status::FAILURE_MOTION_UNAVAILABLE, now);
        return;
    }
    // 正常 MIN/EXPO/ASYM 可能使旧线性 FF 不可观；RTK 成功后仍能独立采集
    // 真实响应，不能以“FF 已保存”为后置整形/运行参数辨识的循环依赖。
    if (begin_imu_bias(now)) return;
    if (transaction_.active()) { terminate(Status::FAILURE_PARAMETER, false, now); return; }
    status_.unavailable_stages |= Status::STAGE_IMU_BIAS;
    start_response_profile(now);
}

void AutoCalibrationMode::begin_identification(std::uint64_t now) noexcept
{
    if (!rtk_yaw_fused(now) || !std::isfinite(rtk_sub_.get().speed_accuracy_m_s) ||
        rtk_sub_.get().speed_accuracy_m_s <= 0.0F) {
        fail_tuning(Status::FAILURE_SENSOR_STALE, now);
        return;
    }
    const auto &c = tuning_config_;
    const bool limits_valid = std::isfinite(c.speed_limit) && c.speed_limit > 0.0F && c.speed_limit <= fence_.speed_limit_m_s &&
        std::isfinite(c.rate_limit) && c.rate_limit > 0.0F && c.rate_limit <= 0.6F &&
        std::isfinite(c.speed_threshold) && c.speed_threshold >= 0.0F && c.speed_threshold < 0.4F * c.speed_limit &&
        std::isfinite(c.rate_threshold) && c.rate_threshold >= 0.0F && c.rate_threshold < 0.4F * c.rate_limit &&
        std::isfinite(c.acceleration) && c.acceleration > 0.0F && std::isfinite(c.deceleration) && c.deceleration > 0.0F &&
        std::isfinite(c.rate_acceleration) && c.rate_acceleration > 0.0F && std::isfinite(c.rate_deceleration) && c.rate_deceleration > 0.0F;
    // 待验证运行值来自本会话响应候选，不是入场冻结上限。前馈不能吃完
    // 普通阶段的输出余量；满输出探测的端点不能冒充 PI 调节余量。
    const float speed_ff = c.speed_limit / status_.maximum_speed_m_s;
    const float rate_ff = c.rate_limit * config_.track * status_.yaw_rate_correction / (2.0F * status_.maximum_speed_m_s);
    if (!limits_valid || !std::isfinite(speed_ff) || !std::isfinite(rate_ff) ||
        speed_ff >= std::min(0.35F, config_.throttle) || rate_ff >= std::min(0.30F, config_.steering) ||
        !prepare_straight(now) || now - session_started_ > 450000000ULL) {
        fail_tuning(Status::FAILURE_MOTION_UNAVAILABLE, now);
        return;
    }
    status_.closed_loop = false;
    status_.gain_group = Status::GAIN_INNER;
    const auto &imu_status = imu_status_sub_.get();
    float gyro_variance = 0.0F;
    for (float variance : imu_status.var_gyro) {
        if (!std::isfinite(variance) || variance < 0.0F) { fail_tuning(Status::FAILURE_SENSOR_STALE, now); return; }
        gyro_variance = std::max(gyro_variance, variance);
    }
    if (!fresh(imu_status.timestamp, now, 1500000ULL) || imu_status.gyro_device_id != imu_sub_.get().gyro_device_id) {
        fail_tuning(Status::FAILURE_SENSOR_STALE, now); return;
    }
    tuning_noise_[0] = std::max(response_noise_[0], rtk_sub_.get().speed_accuracy_m_s);
    tuning_noise_[1] = std::max(response_noise_[1], std::sqrt(gyro_variance));
    if (!reset_identification()) { fail_tuning(Status::FAILURE_IDENTIFICATION, now); return; }
    exercise_ = 0U;
    transition(Status::STATE_WAIT_ARM_IDENTIFICATION, now);
}

bool AutoCalibrationMode::reset_identification() noexcept
{
    math::IdentificationConfig configuration{};
    configuration.maximum_absolute_input = 0.40F;
    configuration.maximum_absolute_output = fence_.speed_limit_m_s;
    configuration.maximum_normalized_rms_residual = 0.20F;
    configuration.sample_period_tolerance_s = 0.04F;
    const float velocity_input = tuning_config_.speed_limit / status_.maximum_speed_m_s;
    configuration.minimum_input_variance = std::max(1.0e-8F, 0.0004F * velocity_input * velocity_input);
    configuration.output_noise_variance = tuning_noise_[0] * tuning_noise_[0];
    for (unsigned i = 0U; i < 2U; ++i) if (!identifiers_[i].configure(configuration)) return false;
    configuration.sample_period_s = 0.02F;
    configuration.sample_period_tolerance_s = 0.012F;
    configuration.maximum_absolute_output = 0.6F;
    configuration.minimum_samples = 150U;
    const float rate_input = tuning_config_.rate_limit * config_.track * status_.yaw_rate_correction / (2.0F * status_.maximum_speed_m_s);
    configuration.minimum_input_variance = std::max(1.0e-8F, 0.0004F * rate_input * rate_input);
    configuration.output_noise_variance = tuning_noise_[1] * tuning_noise_[1];
    for (unsigned i = 2U; i < 4U; ++i) if (!identifiers_[i].configure(configuration)) return false;
    for (unsigned i = 0U; i < 4U; ++i) shaping_uu_[i] = shaping_up_[i] = shaping_pp_[i] = 0.0;
    tuning_sample_ = exercise_started_ = 0U;
    exercise_running_ = false;
    return true;
}

bool AutoCalibrationMode::tuning_estimator_valid(std::uint64_t now) const noexcept
{
    const auto &position = position_sub_.get();
    const auto &odometry = odometry_sub_.get();
    // GNSS yaw 融合不等于水平位置/速度融合。静态等待即检查真实闭环反馈的
    // 依赖，避免先允许 Arm 再发现只有航向而没有可用 EKF 速度。
    return fresh(position.timestamp, now, 200000ULL) && fresh(position.timestamp_sample, now, 200000ULL) &&
        position.timestamp_sample <= position.timestamp && fresh(odometry.timestamp, now, 200000ULL) &&
        fresh(odometry.timestamp_sample, now, 200000ULL) && odometry.timestamp_sample <= odometry.timestamp &&
        position.xy_valid && position.v_xy_valid && position.xy_global && position.heading_good_for_control &&
        !position.dead_reckoning && position.ref_timestamp != 0U && position.ref_timestamp <= position.timestamp &&
        (tuning_reference_ == 0U || (position.ref_timestamp == tuning_reference_ &&
            position.xy_reset_counter == tuning_xy_reset_ && position.vxy_reset_counter == tuning_vxy_reset_ &&
            position.heading_reset_counter == tuning_yaw_reset_ && odometry.reset_counter == tuning_odom_reset_)) &&
        std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.vx) && std::isfinite(position.vy) &&
        std::isfinite(position.heading) && std::isfinite(position.ref_lat) && std::fabs(position.ref_lat) < 85.0 &&
        std::isfinite(position.ref_lon) && std::fabs(position.ref_lon) <= 180.0 && std::isfinite(odometry.angular_velocity[2]);
}

bool AutoCalibrationMode::tuning_feedback(std::uint64_t now) const noexcept
{
    const auto &feedback = control_feedback_sub_.get();
    return feedback.valid && feedback.source == rover_motion_request_s::SOURCE_CALIBRATION &&
        feedback.session_id == status_.session_id && feedback.closed_loop == status_.closed_loop &&
        fresh(feedback.timestamp, now, 100000ULL) && fresh(feedback.timestamp_sample, now, 100000ULL) &&
        feedback.timestamp_sample >= arm_started_ && rtk_yaw_fused(now) && tuning_estimator_valid(now) &&
        std::isfinite(feedback.speed_m_s) && std::isfinite(feedback.speed_raw_m_s) &&
        std::isfinite(feedback.forward_raw_m_s) && std::isfinite(feedback.lateral_raw_m_s) && std::isfinite(feedback.yaw_rate_rad_s);
}

bool AutoCalibrationMode::take_tuning_sample(std::uint64_t now, bool rate) noexcept
{
    if (!tuning_feedback(now)) return false;
    const auto timestamp = control_feedback_sub_.get().timestamp_sample;
    const std::uint64_t interval = rate ? 20000U : 100000U;
    // 容差只判断实际采样抖动，不能作为提前采样许可；否则 20 ms 调度会把
    // 标称 100 ms 的速度模型稳定地采成 80 ms，导致时间常数与 PI 单位错误。
    if (timestamp <= tuning_sample_ || (tuning_sample_ != 0U && timestamp - tuning_sample_ < interval)) return false;
    tuning_sample_ = timestamp;
    return true;
}

void AutoCalibrationMode::fail_tuning(std::uint8_t reason, std::uint64_t now) noexcept
{
    const auto missing = (Status::STAGE_INNER_GAINS | Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN |
        Status::STAGE_MOTOR_PROFILE | Status::STAGE_RUNTIME | Status::STAGE_NAV_STRATEGY) & ~status_.completed_stages;
    status_.unavailable_stages |= missing;
    if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = reason;
    // 所有失败都先撤销正向许可；Disarmed 的回滚/保存交接窗口也不能继承
    // WAIT_ARM 标志。统一退出请求使 Commander 清 session grant，再集中回滚。
    status_.awaiting_arm = status_.motion_allowed = status_.session_authorized = false;
    if (!armed_.armed() && transaction_.active()) transition(Status::STATE_RESTORE_GAINS, now);
    terminate(reason, false, now);
}

bool AutoCalibrationMode::step_tuning(std::uint64_t now) noexcept
{
    if (step_imu_bias(now)) return true;
    if (step_response_profile(now)) return true;
    switch (status_.state) {
    case Status::STATE_WAIT_ARM_IDENTIFICATION:
    case Status::STATE_WAIT_ARM_VALIDATION: {
        physical_speed_ = physical_rate_ = longitudinal_ = steering_ = 0.0F;
        // Arm 等待不得吃掉接下来完整运动段及停车/保存的预算；尤其不能在
        // 会话只剩几十秒时启动一个最长 75 s 的路径试验。
        const std::uint64_t reserve = status_.state == Status::STATE_WAIT_ARM_IDENTIFICATION && exercise_ < 2U
            ? 200000000ULL : 110000000ULL;
        if (now - state_started_ > 120000000ULL || now - session_started_ + reserve > 600000000ULL) {
            fail_tuning(Status::FAILURE_TIMEOUT, now); break;
        }
        status_.awaiting_arm = rtk_yaw_fused(now) && imu_quality(now) && stopped() && fence_result(now).can_stop &&
            tuning_estimator_valid(now) && (status_.state != Status::STATE_WAIT_ARM_VALIDATION || gain_frontend_confirmed());
        if (!armed_.armed()) break;
        if (!status_.awaiting_arm) { terminate(Status::FAILURE_PREFLIGHT, false, now); break; }
        status_.awaiting_arm = false;
        arm_started_ = motion_started_ = now;
        exercise_started_ = tuning_sample_ = 0U;
        exercise_running_ = false;
        leg_lat_ = gps_sub_.get().latitude_deg;
        leg_lon_ = gps_sub_.get().longitude_deg;
        leg_heading_ = rtk_sub_.get().array_heading_rad;
        exercise_heading_ = body_yaw();
        if (status_.state == Status::STATE_WAIT_ARM_IDENTIFICATION)
            transition(exercise_ < 2U ? Status::STATE_IDENTIFY_SPEED : Status::STATE_IDENTIFY_RATE, now);
        else start_validation(now);
        break;
    }
    case Status::STATE_IDENTIFY_SPEED: identify_speed(now); break;
    case Status::STATE_IDENTIFY_RATE: identify_rate(now); break;
    case Status::STATE_STOP_IDENTIFICATION:
        longitudinal_ = steering_ = 0.0F;
        if (now - state_started_ > 15000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (armed_.armed()) { if (stopped()) request(auto_calibration_request_s::REQUEST_STAGE_DISARM, now); break; }
        if (exercise_ == 2U) transition(Status::STATE_WAIT_ARM_IDENTIFICATION, now);
        else if (!calculate_inner_gains() || !begin_gain_transaction(now, Status::GAIN_INNER)) fail_tuning(Status::FAILURE_IDENTIFICATION, now);
        break;
    case Status::STATE_APPLY_GAINS: case Status::STATE_SAVE_GAINS: case Status::STATE_RESTORE_GAINS:
        poll_gain_transaction(now); break;
    case Status::STATE_VALIDATE_SPEED: validate_inner(now, false); break;
    case Status::STATE_VALIDATE_REVERSE: validate_inner(now, false); break;
    case Status::STATE_VALIDATE_RATE: validate_inner(now, true); break;
    case Status::STATE_VALIDATE_HEADING: validate_heading(now); break;
    case Status::STATE_VALIDATE_DRIVING: validate_driving(now); break;
    case Status::STATE_VALIDATE_PATH: validate_path(now); break;
    case Status::STATE_STOP_VALIDATION:
        physical_speed_ = physical_rate_ = 0.0F;
        if (now - state_started_ > 15000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (armed_.armed()) { if (stopped()) request(auto_calibration_request_s::REQUEST_STAGE_DISARM, now); break; }
        if (status_.gain_group == Status::GAIN_INNER && (exercise_ == 3U || exercise_ == 9U)) {
            transition(Status::STATE_WAIT_ARM_VALIDATION, now);
        } else if (status_.gain_group == Status::GAIN_PATH) {
            // 多候选选择在参数文件处理，始终保留同组最初快照。
            advance_path_trial(now);
        } else if (validation_passed_) advance_cohort_validation(now);
        else fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
        break;
    default: return false;
    }
    return true;
}

} // namespace dima::rover::modes

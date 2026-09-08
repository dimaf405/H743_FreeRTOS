#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

void AutoCalibrationMode::identify_speed(std::uint64_t now) noexcept
{
    const auto &feedback = control_feedback_sub_.get();
    if (exercise_ > 1U || now - state_started_ > 90000000ULL) { fail_tuning(Status::FAILURE_TIMEOUT, now); return; }
    if (!exercise_running_) {
        longitudinal_ = steering_ = 0.0F;
        if (!stopped() || !tuning_feedback(now) || !feedback_unmasked() ||
            now - arm_started_ < static_cast<std::uint64_t>((tuning_motor_.arm_ramp_s + 0.25F) * 1000000.0F)) return;
        exercise_running_ = true;
        exercise_started_ = now;
        tuning_sample_ = 0U;
        identification_input_origin_ = feedback.longitudinal;
        identification_applied_origin_ = feedback.applied_longitudinal;
        identification_output_origin_ = feedback.speed_raw_m_s;
    }
    const unsigned phase = static_cast<unsigned>((now - exercise_started_) / 4000000ULL) % 4U;
    const float fraction = phase == 1U ? 1.0F : phase == 3U ? 0.75F : 0.5F;
    const float desired = fraction * tuning_config_.speed_limit / status_.maximum_speed_m_s;
    // 已验证前馈确定安全激励量；周期变化与稳态 FF/磁样本窗口隔离。保持原
    // 0.15/s 请求斜率，模型输入是最终执行器命令，另存整形前后的比例供
    // PI 坐标转换；输出是 EKF 车速，不拿执行器命令冒充物理响应。
    const float step = 0.02F * (tuning_motor_.throttle_slew_rate > 0.0F
        ? std::min(0.05F, 0.5F * tuning_motor_.throttle_slew_rate) : 0.05F);
    longitudinal_ += std::clamp(desired - longitudinal_, -step, step);
    steering_ = std::clamp(0.5F * math::wrap_pi(leg_heading_ - rtk_sub_.get().array_heading_rad), -0.10F, 0.10F);
    if (now - exercise_started_ > 8000000ULL && ground_speed() < 0.08F) {
        fail_tuning(Status::FAILURE_MOTION_UNAVAILABLE, now); return;
    }
    if (take_tuning_sample(now, false) && feedback.timestamp_sample >= exercise_started_) {
        const float input = feedback.longitudinal - identification_input_origin_;
        const float applied = feedback.applied_longitudinal - identification_applied_origin_;
        if (!feedback_unmasked() || feedback.motor_slew_active || !std::isfinite(input) || !std::isfinite(applied) ||
            !identifiers_[exercise_].add_sample(feedback.timestamp_sample,
                applied, feedback.speed_raw_m_s - identification_output_origin_)) {
            fail_tuning(Status::FAILURE_IDENTIFICATION, now); return;
        }
        shaping_uu_[exercise_] += input * input;
        shaping_up_[exercise_] += input * applied;
        shaping_pp_[exercise_] += applied * applied;
    }
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg, leg_lat_, leg_lon_, north, east);
    if (std::hypot(north, east) < leg_distance_) return;
    longitudinal_ = steering_ = 0.0F;
    exercise_running_ = false;
    tuning_sample_ = 0U;
    if (exercise_ == 0U) {
        exercise_ = 1U;
        turn_heading_ = math::wrap_pi(leg_heading_ + kPi);
        turn_resume_state_ = Status::STATE_IDENTIFY_SPEED;
        transition(Status::STATE_TURN_AROUND, now);
    } else {
        exercise_ = 2U;
        transition(Status::STATE_STOP_IDENTIFICATION, now);
    }
}

void AutoCalibrationMode::identify_rate(std::uint64_t now) noexcept
{
    longitudinal_ = 0.0F;
    const auto &feedback = control_feedback_sub_.get();
    if (exercise_ < 2U || exercise_ > 3U || now - state_started_ > 75000000ULL) {
        fail_tuning(Status::FAILURE_TIMEOUT, now); return;
    }
    if (!exercise_running_) {
        steering_ = 0.0F;
        if (!stopped() || !tuning_feedback(now) || !feedback_unmasked() ||
            now - arm_started_ < static_cast<std::uint64_t>((tuning_motor_.arm_ramp_s + 0.25F) * 1000000.0F)) return;
        exercise_running_ = true;
        exercise_started_ = now;
        tuning_sample_ = 0U;
        identification_input_origin_ = feedback.steering;
        identification_applied_origin_ = feedback.applied_steering;
        identification_output_origin_ = feedback.yaw_rate_rad_s;
    }
    const float direction = exercise_ == 2U ? 1.0F : -1.0F;
    const unsigned phase = static_cast<unsigned>((now - exercise_started_) / 4000000ULL) % 4U;
    const float fraction = phase == 1U ? 1.0F : phase == 3U ? 0.75F : 0.5F;
    const float desired = direction * fraction * tuning_config_.rate_limit * config_.track * status_.yaw_rate_correction /
        (2.0F * status_.maximum_speed_m_s);
    steering_ += std::clamp(desired - steering_, -0.001F, 0.001F);
    if (now - exercise_started_ > 8000000ULL && std::fabs(yaw_rate()) < 0.03F) {
        fail_tuning(Status::FAILURE_MOTION_UNAVAILABLE, now); return;
    }
    if (take_tuning_sample(now, true) && feedback.timestamp_sample >= exercise_started_) {
        // 方向归一化只改变坐标，不把上一方向尚未停稳的样本归入新模型。
        const float input = direction * (feedback.steering - identification_input_origin_);
        const float applied = direction * (feedback.applied_steering - identification_applied_origin_);
        if (!feedback_unmasked() || feedback.motor_slew_active || !std::isfinite(input) || !std::isfinite(applied) ||
            !identifiers_[exercise_].add_sample(feedback.timestamp_sample,
                applied,
                direction * (feedback.yaw_rate_rad_s - identification_output_origin_))) {
            fail_tuning(Status::FAILURE_IDENTIFICATION, now); return;
        }
        shaping_uu_[exercise_] += input * input;
        shaping_up_[exercise_] += input * applied;
        shaping_pp_[exercise_] += applied * applied;
    }
    if (now - exercise_started_ < 16000000ULL) return;
    steering_ = 0.0F;
    exercise_running_ = false;
    ++exercise_;
    if (exercise_ == 3U) transition(Status::STATE_IDENTIFY_RATE, now);
    else transition(Status::STATE_STOP_IDENTIFICATION, now);
}

bool AutoCalibrationMode::calculate_inner_gains() noexcept
{
    math::IdentificationResult fits[4]{};
    for (unsigned i = 0U; i < 4U; ++i) {
        const bool rate = i >= 2U;
        const float range = rate ? tuning_config_.rate_limit : tuning_config_.speed_limit;
        const float feedforward = rate ? range * config_.track * status_.yaw_rate_correction /
            (2.0F * status_.maximum_speed_m_s) : range / status_.maximum_speed_m_s;
        // RLS 输入是最终左右电机命令的均值/差分，不用布尔近似代替。另拟合
        // delta_applied=s*delta_command，把得到的 PI 再换回整形前控制器坐标。
        // 非线性/迟滞/斜率限制不能用单一正斜率解释时拒绝，不偷偷混用输入单位。
        if (shaping_uu_[i] <= 1.0e-8 || shaping_pp_[i] <= 1.0e-8) return false;
        const float shaping = static_cast<float>(shaping_up_[i] / shaping_uu_[i]);
        const double residual = std::max(0.0, shaping_pp_[i] - shaping_up_[i] * shaping_up_[i] / shaping_uu_[i]);
        if (!std::isfinite(shaping) || shaping < 0.1F || shaping > 10.0F ||
            residual > 0.0025 * shaping_pp_[i]) return false;
        math::PiDesignLimits limits{};
        // 元数据的 0..100 不是自动搜索范围；按本次激励的实际 FF 余量预留
        // 一半给模型误差，PI 最坏项还须在该余量内，永不放宽硬输出上限。
        const float ceiling = rate ? std::min(0.35F, config_.steering) : std::min(0.40F, config_.throttle);
        limits.output_headroom = 0.5F * (ceiling - feedforward) * shaping;
        limits.maximum_proportional_gain = limits.maximum_integral_gain = 100.0F * shaping;
        limits.maximum_error = range;
        limits.integration_horizon_s = 6.0F;
        fits[i] = identifiers_[i].fit(limits);
        if (!fits[i].valid()) return false;
        fits[i].pi.proportional_gain /= shaping;
        fits[i].pi.integral_gain /= shaping;
        fits[i].model.dc_gain *= shaping;
        identified_models_[i] = fits[i].model;
    }
    for (unsigned axis = 0U; axis < 2U; ++axis) {
        const unsigned index = axis * 2U;
        const float a = fits[index].model.dc_gain, b = fits[index + 1U].model.dc_gain;
        if (std::fabs(a - b) > 0.20F * 0.5F * (a + b)) return false;
        gains_[index] = 0.5F * (fits[index].pi.proportional_gain + fits[index + 1U].pi.proportional_gain);
        gains_[index + 1U] = 0.5F * (fits[index].pi.integral_gain + fits[index + 1U].pi.integral_gain);
        loop_time_[axis] = std::max(fits[index].pi.closed_loop_time_s, fits[index + 1U].pi.closed_loop_time_s);
    }
    // 候选运行速度必须能在本圆内走完真实 ramp+稳态验证，不能等 Arm 后才
    // 发现注定超时。收紧的是待验证运行值，不是入场冻结的围栏速度上限。
    const float fixed_time = std::max(3.0F, 3.0F * loop_time_[0]) + 1.0F;
    const float acceleration = std::min(tuning_config_.acceleration, tuning_config_.deceleration);
    const float space = leg_distance_ - 2.0F * config_.stop_distance;
    if (fixed_time >= 15.0F || space <= 0.0F || acceleration <= 0.0F) return false;
    const float at = acceleration * fixed_time;
    const float space_speed = 0.5F * (-at + std::sqrt(at * at + 4.0F * acceleration * space / 1.5F));
    tuning_config_.speed_limit = std::min({tuning_config_.speed_limit, 0.9F * space_speed,
        0.9F * acceleration * (15.0F - fixed_time)});
    const float fixed_rate_time = std::max(3.0F, 3.0F * loop_time_[1]) + 1.0F;
    if (fixed_rate_time >= 15.0F) return false;
    tuning_config_.rate_limit = std::min(tuning_config_.rate_limit,
        0.9F * std::min(tuning_config_.rate_acceleration, tuning_config_.rate_deceleration) * (15.0F - fixed_rate_time));
    if (tuning_config_.speed_threshold >= 0.1F * tuning_config_.speed_limit ||
        tuning_config_.rate_threshold >= 0.1F * tuning_config_.rate_limit) return false;
    return true;
}

} // namespace dima::rover::modes

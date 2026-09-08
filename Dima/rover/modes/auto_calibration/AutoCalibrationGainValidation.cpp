#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

void AutoCalibrationMode::start_validation(std::uint64_t now) noexcept
{
    status_.closed_loop = true;
    exercise_running_ = false;
    physical_speed_ = physical_rate_ = 0.0F;
    validation_passed_ = false;
    if (status_.gain_group == Status::GAIN_INNER) {
        if ((exercise_ < 3U || exercise_ >= 9U) && !prepare_straight(now)) { fail_tuning(Status::FAILURE_FENCE_SPACE, now); return; }
        transition(exercise_ < 3U ? Status::STATE_VALIDATE_SPEED : exercise_ < 9U ? Status::STATE_VALIDATE_RATE
            : Status::STATE_VALIDATE_REVERSE, now);
    } else if (status_.gain_group == Status::GAIN_HEADING) {
        if (!tuning_heading_.configure({status_.heading_p, tuning_config_.rate_limit})) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
        exercise_ = 0U;
        transition(Status::STATE_VALIDATE_HEADING, now);
    } else if (status_.gain_group == Status::GAIN_NAVIGATION) {
        if (!tuning_heading_.configure({tuning_config_.heading_p, tuning_config_.rate_limit}) ||
            !tuning_driving_.configure(tuning_config_.driving)) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
        exercise_ = navigation_phase_ = navigation_directions_ = 0U;
        navigation_phase_started_ = now;
        exercise_heading_ = body_yaw();
        transition(Status::STATE_VALIDATE_DRIVING, now);
    } else {
        if (!tuning_pursuit_.configure({status_.lookahead_gain, tuning_config_.pursuit.lookahead_min_m,
                tuning_config_.pursuit.lookahead_max_m}) ||
            !tuning_heading_.configure({tuning_config_.heading_p, tuning_config_.rate_limit}) ||
            !tuning_driving_.configure(tuning_config_.driving)) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
        path_index_ = 0U;
        path_arrived_ = false;
        path_entry_captured_ = false;
        path_errors_.reset();
        path_observable_samples_ = path_crossings_ = 0U;
        path_sample_ = 0U;
        path_leg_started_ = now;
        path_yaw_rate_previous_ = 0.0F;
        path_jerk_samples_ = path_reduction_samples_ = 0U;
        path_observation_sample_ = path_measured_started_ = 0U;
        transition(Status::STATE_VALIDATE_PATH, now);
    }
}

void AutoCalibrationMode::end_validation_motion(std::uint64_t now) noexcept
{
    physical_speed_ = physical_rate_ = 0.0F;
    exercise_running_ = false;
    transition(Status::STATE_STOP_VALIDATION, now);
}

void AutoCalibrationMode::validate_inner(std::uint64_t now, bool rate) noexcept
{
    const auto &feedback = control_feedback_sub_.get();
    if (now - state_started_ > 75000000ULL || !transaction_.generation_valid() ||
        (tuning_feedback(now) && feedback.parameter_update_instance != transaction_.generation())) {
        fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
    }
    const bool reverse = status_.state == Status::STATE_VALIDATE_REVERSE;
    const unsigned phase = rate ? (exercise_ - 3U) % 3U : reverse ? exercise_ - 9U : exercise_;
    const bool falling = phase == 2U;
    const float direction = reverse || (rate && exercise_ >= 6U) ? -1.0F : 1.0F;
    const float fraction = phase == 1U ? 1.0F : 0.5F;
    const float range = rate ? tuning_config_.rate_limit : reverse
        ? std::min({0.24F, tuning_config_.speed_limit, 0.8F * status_.observed_reverse_speed_m_s}) : tuning_config_.speed_limit;
    const float target = fraction * range * direction;
    if (!exercise_running_) {
        if (!falling) physical_speed_ = physical_rate_ = 0.0F;
        if ((!falling && !stopped()) || !tuning_feedback(now)) return;
        math::StepValidationConfig config{};
        config.sample_period_s = rate ? 0.02F : 0.1F;
        config.sample_period_tolerance_s = rate ? 0.012F : 0.04F;
        config.initial_output = rate ? feedback.yaw_rate_rad_s : feedback.speed_m_s;
        config.target_output = target;
        config.noise = tuning_noise_[rate ? 1U : 0U];
        config.minimum_samples = rate ? 150U : 30U;
        config.steady_window_samples = rate ? 51U : 11U;
        if (!validator_.reset(config)) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
        // 验证包含实际 setpoint slew 与足够稳态时间；空间/截止预算不能满足
        // 声明速度时拒绝本组，不偷偷改小 RO_SPEED_LIM 来制造“验证通过”。
        const float limit = rate ? (falling ? tuning_config_.rate_deceleration : tuning_config_.rate_acceleration)
                                 : (falling ? tuning_config_.deceleration : tuning_config_.acceleration);
        const float rise = std::fabs(target - config.initial_output) / limit;
        exercise_duration_ = rise + std::max(3.0F, 3.0F * loop_time_[rate ? 1U : 0U]) + 1.0F;
        if (exercise_duration_ > 15.0F || (!rate &&
            1.5F * range * exercise_duration_ + 2.0F * config_.stop_distance > leg_distance_)) {
            fail_tuning(Status::FAILURE_FENCE_SPACE, now); return;
        }
        exercise_started_ = now;
        tuning_sample_ = 0U;
        steady_since_ = 0U;
        validation_ramp_samples_ = 0U;
        validation_setpoint_time_ = 0U;
        validation_previous_unmasked_ = false;
        exercise_running_ = true;
    }
    physical_speed_ = rate ? 0.0F : target;
    // 速度验证的航向保持使用既有有界角差目标，不依赖尚未整定的 Heading P。
    physical_rate_ = rate ? target : std::clamp(0.5F * math::wrap_pi(exercise_heading_ - body_yaw()), -0.15F, 0.15F);
    if (!feedback_unmasked() || feedback.motor_slew_active) steady_since_ = 0U;
    else if (steady_since_ == 0U) steady_since_ = now;
    if (take_tuning_sample(now, rate) && feedback.timestamp_sample >= exercise_started_) {
        const float actual_setpoint = rate ? feedback.yaw_rate_setpoint_rad_s : feedback.speed_setpoint_m_s;
        // 非零下降阶跃必须看到真实生产控制器的限斜率设定经过中间值；
        // 精确零命令的 reset/自由停车不能替代 RO_*_DECEL_LIM 的生效证据。
        const bool unmasked = feedback_unmasked() && !feedback.motor_slew_active;
        const float magnitude = std::fabs(actual_setpoint);
        const bool interior = falling ? magnitude > 0.55F * range && magnitude < 0.95F * range
            : magnitude > 0.05F * std::fabs(target) && magnitude < 0.95F * std::fabs(target);
        if (interior && unmasked && validation_previous_unmasked_ && validation_setpoint_time_ != 0U &&
            feedback.timestamp > validation_setpoint_time_) {
            const float dt = 1.0e-6F * static_cast<float>(feedback.timestamp - validation_setpoint_time_);
            const float expected_rate = (falling ? -1.0F : 1.0F) * (rate
                ? (falling ? tuning_config_.rate_deceleration : tuning_config_.rate_acceleration)
                : (falling ? tuning_config_.deceleration : tuning_config_.acceleration));
            const float observed_rate = (magnitude - std::fabs(validation_previous_setpoint_)) / dt;
            if (dt <= 0.15F && std::isfinite(observed_rate) &&
                std::fabs(observed_rate - expected_rate) <= std::max(1.0e-4F, 0.10F * std::fabs(expected_rate)))
                ++validation_ramp_samples_;
        }
        validation_setpoint_time_ = feedback.timestamp;
        validation_previous_setpoint_ = actual_setpoint;
        validation_previous_unmasked_ = unmasked;
        if (!validator_.add_sample(feedback.timestamp_sample,
            rate ? feedback.yaw_rate_rad_s : feedback.speed_m_s, feedback.saturated)) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
    }
    if (now - exercise_started_ < static_cast<std::uint64_t>(exercise_duration_ * 1000000.0F)) return;
    const auto result = validator_.result();
    status_.validation_error = result.steady_state_error;
    const unsigned rate_bit = (rate ? 2U : 0U) + (falling ? 1U : 0U);
    if (!result.valid() || ((runtime_observed_mask_ & (1U << rate_bit)) != 0U && validation_ramp_samples_ < 2U) ||
        steady_since_ == 0U || now - steady_since_ < 1000000ULL) {
        fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
    }
    // 完整幅值通过后保持当前非零命令，下一拍直接进入半幅下降验证。
    // 其余段先请求停车，避免把换向滑行混到另一方向的响应里。
    if (phase != 1U) physical_speed_ = physical_rate_ = 0.0F;
    exercise_running_ = false;
    ++exercise_;
    if ((!rate && !reverse && exercise_ == 3U) || (rate && exercise_ == 9U) || (reverse && exercise_ == 12U)) {
        validation_passed_ = reverse;
        end_validation_motion(now);
    }
}

void AutoCalibrationMode::validate_heading(std::uint64_t now) noexcept
{
    const auto &feedback = control_feedback_sub_.get();
    if (now - state_started_ > 75000000ULL) { fail_tuning(Status::FAILURE_TIMEOUT, now); return; }
    if (!exercise_running_) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (!stopped() || !tuning_feedback(now)) return;
        exercise_start_yaw_ = body_yaw();
        const float angle = std::max(30.0F * kRadians, 2.0F * tuning_config_.driving.turn_to_drive_yaw_error_rad);
        const float target = exercise_ == 0U ? angle : -angle;
        exercise_heading_ = math::wrap_pi(exercise_start_yaw_ + target);
        math::StepValidationConfig config{};
        config.sample_period_tolerance_s = 0.04F;
        config.steady_window_samples = 11U;
        config.target_output = target;
        config.noise = std::max(0.002F, rtk_sub_.get().heading_accuracy_rad);
        // 外环收敛要求来自原 Driving 退出角，不伪造噪声，也不修改用户死区。
        config.absolute_steady_tolerance = 0.8F * tuning_config_.driving.turn_to_drive_yaw_error_rad;
        if (!validator_.reset(config)) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
        tuning_heading_.reset();
        exercise_started_ = now;
        tuning_sample_ = 0U;
        stable_since_ = 0U;
        exercise_running_ = true;
    }
    const auto heading = tuning_heading_.update(exercise_heading_, body_yaw(), 0.02F);
    if (!heading.valid) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    physical_speed_ = 0.0F;
    physical_rate_ = heading.yaw_rate_setpoint_rad_s;
    const float angle = math::wrap_pi(body_yaw() - exercise_start_yaw_);
    const float error = math::wrap_pi(exercise_heading_ - body_yaw());
    if (take_tuning_sample(now, false) && feedback.timestamp_sample >= exercise_started_) {
        if (feedback.parameter_update_instance != transaction_.generation() ||
            !validator_.add_sample(feedback.timestamp_sample, angle, feedback.saturated)) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
    }
    const float target_size = std::fabs(math::wrap_pi(exercise_heading_ - exercise_start_yaw_));
    const bool converged = std::fabs(error) < tuning_config_.driving.turn_to_drive_yaw_error_rad &&
        std::fabs(angle) >= 0.5F * target_size && std::fabs(yaw_rate()) < 0.05F && feedback_unmasked();
    if (converged) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
    if (now - exercise_started_ > 30000000ULL) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    if (stable_since_ == 0U || now - stable_since_ < 1500000ULL || !validator_.result().valid()) return;
    status_.validation_error = std::fabs(error);
    physical_rate_ = 0.0F;
    exercise_running_ = false;
    ++exercise_;
    if (exercise_ == 2U) { validation_passed_ = true; end_validation_motion(now); }
}

} // namespace dima::rover::modes

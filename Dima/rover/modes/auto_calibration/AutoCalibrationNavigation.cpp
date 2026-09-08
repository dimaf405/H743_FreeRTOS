#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

bool AutoCalibrationMode::prepare_navigation_candidate() noexcept
{
    const float noise = rtk_sub_.get().heading_accuracy_rad;
    const float old_exit = transaction_.original_float(dima::params::RD_TRANS_TRN_DRV);
    const float old_entry = transaction_.original_float(dima::params::RD_TRANS_DRV_TRN);
    if (!std::isfinite(noise) || noise <= 0.0F || !std::isfinite(old_exit) || old_exit <= 0.0F ||
        !std::isfinite(old_entry) || old_entry <= old_exit || status_.heading_p <= 0.0F) return false;
    // 退出角不能小于真实航向噪声，也不能让 yaw-rate 死区先清掉转向；
    // 进入角保留可观察滞回，不靠提高门限掩盖无法退出的控制器。
    const float exit = std::max({0.8F * old_exit, 3.0F * noise,
        1.25F * tuning_config_.rate_threshold / status_.heading_p});
    const float delay = std::max(identified_models_[2].delay_s, identified_models_[3].delay_s);
    const float entry = std::max(0.8F * old_entry, exit + 3.0F * noise + delay * tuning_config_.rate_threshold);
    if (!std::isfinite(exit) || exit > 0.5F || !std::isfinite(entry) || entry <= exit || entry > 0.75F) return false;
    tuning_config_.driving.turn_to_drive_yaw_error_rad = exit;
    tuning_config_.driving.drive_to_turn_yaw_error_rad = entry;
    return true;
}

void AutoCalibrationMode::validate_driving(std::uint64_t now) noexcept
{
    const auto &f = control_feedback_sub_.get();
    if (now - state_started_ > 60000000ULL || !tuning_feedback(now) ||
        f.parameter_update_instance != transaction_.generation()) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (now - arm_started_ > 250000ULL) fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
        return;
    }
    // 转驱验收也服从既有闭环持续饱和门槛；不能只看状态机最终能退出，
    // 就让持续顶住输出上限的一段获得导航参数保存资格。
    if (f.saturated) {
        if (steady_since_ == 0U) steady_since_ = now;
        if (now - steady_since_ > 1000000ULL) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    } else steady_since_ = 0U;
    const float speed = 0.5F * tuning_config_.speed_limit;
    const float error = math::wrap_pi(exercise_heading_ - body_yaw());
    if (navigation_phase_ == 2U) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (!stopped()) return;
        if (++exercise_ == 2U) {
            validation_passed_ = navigation_directions_ == 3U;
            end_validation_motion(now);
            return;
        }
        navigation_phase_ = 0U;
        navigation_phase_started_ = now;
        exercise_heading_ = body_yaw();
        tuning_driving_.reset(); tuning_heading_.reset();
        stable_since_ = 0U;
        return;
    }
    const float state_speed = tuning_driving_.state() == dima::lib::rover::DrivingState::Driving ? speed : 0.0F;
    const auto before = tuning_driving_.state();
    const auto driving = tuning_driving_.update(error, state_speed, f.speed_m_s);
    const auto heading = tuning_heading_.update(exercise_heading_, body_yaw(), 0.02F);
    if (!driving.valid || !heading.valid) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    physical_speed_ = driving.translation_enabled ? speed : 0.0F;
    physical_rate_ = driving.heading_control_enabled ? heading.yaw_rate_setpoint_rad_s : 0.0F;
    if (navigation_phase_ == 0U) {
        const bool steady = f.speed_raw_m_s >= 0.8F * speed && std::fabs(error) < 0.5F *
            tuning_config_.driving.turn_to_drive_yaw_error_rad && feedback_unmasked();
        if (steady) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
        if (stable_since_ != 0U && now - stable_since_ >= 1000000ULL) {
            // 在真实前进中跨过进入阈值；不是在停车航点上假装验证 Driving→Stop。
            const float sign = exercise_ == 0U ? 1.0F : -1.0F;
            exercise_heading_ = math::wrap_pi(body_yaw() + sign * 1.2F * tuning_config_.driving.drive_to_turn_yaw_error_rad);
            navigation_phase_ = 1U;
            navigation_phase_started_ = now;
        }
    } else {
        if (before == dima::lib::rover::DrivingState::Driving &&
            driving.state == dima::lib::rover::DrivingState::StoppingForTurn) {
            if (f.speed_raw_m_s < 0.5F * speed) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
            navigation_directions_ |= static_cast<std::uint8_t>(1U << exercise_);
        }
        if (before == dima::lib::rover::DrivingState::StoppingForTurn &&
            driving.state == dima::lib::rover::DrivingState::SpotTurning &&
            std::hypot(f.forward_raw_m_s, f.lateral_raw_m_s) > tuning_config_.speed_threshold) {
            fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
        }
        if (before == dima::lib::rover::DrivingState::SpotTurning && driving.state == dima::lib::rover::DrivingState::Driving) {
            navigation_phase_ = 2U;
            physical_speed_ = physical_rate_ = 0.0F;
        }
        if (now - navigation_phase_started_ > 20000000ULL) fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
    }
}

void AutoCalibrationMode::start_path_trials(std::uint64_t now) noexcept
{
    if (!read_tuning_config() || !prepare_path(now)) { fail_tuning(Status::FAILURE_PATH_UNOBSERVABLE, now); return; }
    const float original_gain = tuning_config_.pursuit.lookahead_gain;
    best_path_error_ = INFINITY; best_path_trial_ = 0U;
    path_trial_ = 0U;
    for (unsigned i = 0U; i < 3U; ++i) { path_scores_[i] = INFINITY; path_duration_s_[i] = INFINITY; path_observed_fields_[i] = 0U; }
    path_candidates_[0] = {original_gain, tuning_config_.jerk, tuning_config_.speed_reduction};
    const float error = 0.8F * tuning_config_.driving.turn_to_drive_yaw_error_rad;
    const float reduction_seed = kPi / error * (1.0F - 0.6F * tuning_config_.speed_limit / status_.maximum_speed_m_s);
    for (unsigned i = 1U; i < 3U; ++i) {
        const float ratio = i == 1U ? 0.8F : 1.25F;
        auto &candidate = path_candidates_[i];
        candidate = {original_gain * ratio, tuning_config_.jerk * ratio,
            tuning_config_.speed_reduction >= 0.0F ? tuning_config_.speed_reduction / ratio : reduction_seed / ratio};
        if (!path_gain_sensitive_ || candidate.gain < 0.1F || candidate.gain > 100.0F) candidate.gain = original_gain;
        if (!std::isfinite(candidate.jerk) || candidate.jerk <= 0.0F || candidate.jerk > 100.0F) candidate.jerk = tuning_config_.jerk;
        if (!std::isfinite(candidate.reduction) || candidate.reduction < 0.0F || candidate.reduction > 100.0F)
            candidate.reduction = tuning_config_.speed_reduction;
    }
    status_.lookahead_gain = original_gain;
    if (!begin_gain_transaction(now, Status::GAIN_PATH)) fail_tuning(Status::FAILURE_PARAMETER, now);
}

bool AutoCalibrationMode::apply_path_candidate(std::uint64_t now, unsigned trial) noexcept
{
    if (trial >= 3U) return false;
    const auto &candidate = path_candidates_[trial];
    status_.lookahead_gain = candidate.gain;
    tuning_config_.jerk = candidate.jerk;
    tuning_config_.speed_reduction = candidate.reduction;
    return revise_runtime_candidates() && transaction_.apply_revisions(now, expected_set_count_);
}

} // namespace dima::rover::modes

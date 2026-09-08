#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "auto/AutoMode.hpp"
#include "control/RoverDifferential.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {

bool AutoCalibrationMode::begin_gain_transaction(std::uint64_t now, std::uint8_t group) noexcept
{
    if (armed_.armed() || !runtime_cohort_ || transaction_.phase() != CalibrationParameters::Phase::Provisional) return false;
    status_.gain_group = group;
    status_.closed_loop = false;
    validation_passed_ = false;
    transaction_stage_ = Status::STATE_APPLY_GAINS;
    if (group == Status::GAIN_INNER) {
        // 四个内环增益共享一次快照、参数通知与回滚，不能只使速度环有效而
        // 留下不完整的 yaw-rate 控制配置。参数标识全部来自生成的 dima::params。
        status_.speed_p = gains_[0]; status_.speed_i = gains_[1];
        status_.yaw_rate_p = gains_[2]; status_.yaw_rate_i = gains_[3];
    }
    if (!revise_runtime_candidates() || !transaction_.apply_revisions(now, expected_set_count_)) return false;
    transition(Status::STATE_APPLY_GAINS, now);
    return true;
}

bool AutoCalibrationMode::gain_frontend_confirmed() const noexcept
{
    if (!transaction_.generation_valid() || !runtime_frontend_confirmed()) return false;
    {
        const float expected[4]{transaction_.expected_float(dima::params::RO_SPEED_P),
            transaction_.expected_float(dima::params::RO_SPEED_I), transaction_.expected_float(dima::params::RO_YAW_RATE_P),
            transaction_.expected_float(dima::params::RO_YAW_RATE_I)};
        return drive_.calibration_gains_applied(transaction_.generation(), expected);
    }
}

void AutoCalibrationMode::poll_gain_transaction(std::uint64_t now) noexcept
{
    const bool applied = gain_frontend_confirmed();
    const bool ready = transaction_.rolling_back() || (status_.gain_group == Status::GAIN_INNER
        ? drive_.calibration_control_ready(transaction_.generation())
        : navigation_.calibration_configuration_ready(transaction_.generation()));
    const bool valid = applied && ready && (transaction_.rolling_back() ||
        (rtk_yaw_fused(now) && stopped() && fence_result(now).can_stop &&
         (status_.state != Status::STATE_SAVE_GAINS || validation_passed_)));
    if (valid) { if (stable_since_ == 0U) stable_since_ = now; } else stable_since_ = 0U;
    transaction_.poll(applied, valid && stable_since_ != 0U && now - stable_since_ >= 1000000ULL, now);

    if (transaction_.phase() == CalibrationParameters::Phase::Provisional && status_.state == Status::STATE_APPLY_GAINS) {
        expected_set_count_ = transaction_.set_count_snapshot();
        status_.gains_provisional = true;
        status_.closed_loop = true;
        exercise_ = 0U;
        transition(Status::STATE_WAIT_ARM_VALIDATION, now);
    } else if (transaction_.phase() == CalibrationParameters::Phase::Done) {
        expected_set_count_ = transaction_.set_count_snapshot();
        status_.gains_provisional = false;
        after_gain_saved(now);
    } else if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
        expected_set_count_ = transaction_.set_count_snapshot();
        status_.gains_provisional = false;
        status_.unavailable_stages |= (Status::STAGE_INNER_GAINS | Status::STAGE_HEADING_GAIN | Status::STAGE_PATH_GAIN) & ~status_.completed_stages;
        if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_GAIN_VALIDATION;
        finish(now);
    } else if (transaction_.phase() == CalibrationParameters::Phase::Fault) {
        terminate(Status::FAILURE_PARAMETER, false, now);
    }
}

void AutoCalibrationMode::after_gain_saved(std::uint64_t now) noexcept
{
    if (!read_tuning_config()) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
    status_.completed_stages |= status_.provisional_validated_stages;
    status_.provisional_validated_stages = 0U;
    if (runtime_fully_observed_) status_.completed_stages |= Status::STAGE_RUNTIME;
    else status_.unavailable_stages |= Status::STAGE_RUNTIME;
    // 整形与 slew 分别有可观性门禁，不能仅凭其中一个 changed 位宣称整组
    // 电机标定完成。选回原值但经过公平实跑比较也算验证，不要求为改而改。
    if (motor_profile_verified_ && motor_slew_verified_) {
        status_.completed_stages |= Status::STAGE_MOTOR_PROFILE;
        status_.unavailable_stages &= ~Status::STAGE_MOTOR_PROFILE;
    } else status_.unavailable_stages |= Status::STAGE_MOTOR_PROFILE;
    PX4_INFO_RAW("[autocal] MIN/EXPO/ASYM %s; MOT_SLEW_RATE %s\n",
        motor_profile_verified_ ? (motor_candidate_changed_ ? "validated/saved" : "validated/retained") : "unobservable/retained",
        motor_slew_verified_ ? (motor_slew_changed_ ? "validated/saved" : "validated/retained") : "unobservable/retained");
    if (status_.failure_reason == Status::FAILURE_DYNAMICS_UNOBSERVABLE ||
        status_.failure_reason == Status::FAILURE_MECHANICAL_ASYMMETRY) status_.failure_reason = Status::FAILURE_NONE;
    runtime_cohort_ = false;
    finish(now);
}

void AutoCalibrationMode::advance_cohort_validation(std::uint64_t now) noexcept
{
    // 内环通过只记录 RAM 验证证据，绝不提前开放 autosave。新整形/FF/运行
    // 参数必须与后续航向和真实路径一起验证，失败恢复这一整组最初旧值。
    if (!runtime_cohort_ || !validation_passed_) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    if (status_.gain_group == Status::GAIN_INNER) {
        status_.provisional_validated_stages |= Status::STAGE_INNER_GAINS | Status::STAGE_SPEED | Status::STAGE_YAW;
        status_.progress = 85U;
        const float exit_angle = tuning_config_.driving.turn_to_drive_yaw_error_rad;
        // 外环至少慢于已验证内环，且在转向退出边界仍能产生超过测量死区的
        // 角速度。两项不兼容就拒绝，不能调小死区来伪造可用的 Heading P。
        const float tau = std::max({identified_models_[2].time_constant_s, identified_models_[3].time_constant_s, 0.1F});
        const float test_angle = std::max(30.0F * kRadians, 2.0F * exit_angle);
        status_.heading_p = std::min(1.0F / (3.0F * tau), tuning_config_.rate_limit / test_angle);
        if (!std::isfinite(exit_angle) || exit_angle <= 0.0F || exit_angle > 0.5F ||
            !std::isfinite(status_.heading_p) || status_.heading_p <= 0.0F || status_.heading_p > 100.0F ||
            !prepare_navigation_candidate() ||
            !begin_gain_transaction(now, Status::GAIN_HEADING)) fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
    } else if (status_.gain_group == Status::GAIN_HEADING) {
        status_.provisional_validated_stages |= Status::STAGE_HEADING_GAIN;
        status_.progress = 92U;
        if (!read_tuning_config()) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
        if (!begin_gain_transaction(now, Status::GAIN_NAVIGATION)) fail_tuning(Status::FAILURE_PARAMETER, now);
    } else if (status_.gain_group == Status::GAIN_NAVIGATION) {
        status_.navigation_observed_fields |= Status::NAV_FIELD_TRANSITIONS;
        status_.provisional_validated_stages |= Status::STAGE_NAV_STRATEGY;
        start_path_trials(now);
    } else advance_path_trial(now);
}

void AutoCalibrationMode::advance_path_trial(std::uint64_t now) noexcept
{
    if (!validation_passed_) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    const auto changed_fields = [&](unsigned trial) {
        const auto &a = path_candidates_[trial], &base = path_candidates_[0];
        return (a.gain != base.gain ? Status::NAV_FIELD_LOOKAHEAD : 0U) |
            (a.jerk != base.jerk ? Status::NAV_FIELD_JERK : 0U) |
            (a.reduction != base.reduction ? Status::NAV_FIELD_SPEED_REDUCTION : 0U);
    };
    if (path_trial_ < 3U) {
        const unsigned trial = path_trial_;
        const float score = path_scores_[trial];
        if (!std::isfinite(score)) { fail_tuning(Status::FAILURE_PATH_UNOBSERVABLE, now); return; }
        const auto changed = changed_fields(trial);
        const bool eligible = (path_observed_fields_[trial] & changed) == changed &&
            (trial == 0U || (path_duration_s_[trial] <= 1.25F * path_duration_s_[0] &&
                path_sample_counts_[trial] >= 0.8F * path_sample_counts_[0]));
        if (eligible && score < best_path_error_) { best_path_error_ = score; best_path_trial_ = trial; }
        if (trial == 0U) {
            // 基线没有敏感证据的字段保持原值，不把与其他参数同时改变产生的
            // 改善错误归功于一个从未生效的字段，更不修改 MIN/MAX 或到点容差。
            for (unsigned i = 1U; i < 3U; ++i) {
                if ((path_observed_fields_[0] & Status::NAV_FIELD_LOOKAHEAD) == 0U) path_candidates_[i].gain = path_candidates_[0].gain;
                if ((path_observed_fields_[0] & Status::NAV_FIELD_JERK) == 0U) path_candidates_[i].jerk = path_candidates_[0].jerk;
                if ((path_observed_fields_[0] & Status::NAV_FIELD_SPEED_REDUCTION) == 0U) path_candidates_[i].reduction = path_candidates_[0].reduction;
            }
        }
        ++path_trial_;
        while (path_trial_ < 3U) {
            // 新候选预算同时预留选回旧候选的确认圈，不能等最后才发现无法恢复验收。
            if (now - session_started_ > 420000000ULL) { path_trial_ = 3U; break; }
            if (changed_fields(path_trial_) == 0U) { ++path_trial_; continue; }
            validation_passed_ = false;
            if (!apply_path_candidate(now, path_trial_)) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
            transition(Status::STATE_APPLY_GAINS, now);
            return;
        }
    }
    const float improvement = path_scores_[0] - best_path_error_;
    if (!(improvement > std::max(0.10F * path_scores_[0], 3.0F * gps_sub_.get().eph))) best_path_trial_ = 0U;
    const auto &chosen = path_candidates_[best_path_trial_];
    if (status_.lookahead_gain != chosen.gain || tuning_config_.jerk != chosen.jerk ||
        tuning_config_.speed_reduction != chosen.reduction) {
        if (now - session_started_ > 510000000ULL) { fail_tuning(Status::FAILURE_TIMEOUT, now); return; }
        // 最终选中的整组配置必须重新应用并实跑；不拿另一组参数代的结果保存。
        validation_passed_ = false;
        path_trial_ = 3U;
        if (!apply_path_candidate(now, best_path_trial_)) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
        transition(Status::STATE_APPLY_GAINS, now);
        return;
    }
    // 比较候选使用 RAM baseline；保存却必须相对最初持久化快照逐字段核对。
    // 例如旧 jerk=-1 的初始化若始终未生效，就不能只打 PARTIAL 仍保存正值。
    const auto required = (chosen.gain != transaction_.original_float(dima::params::PP_LOOKAHD_GAIN) ? Status::NAV_FIELD_LOOKAHEAD : 0U) |
        (chosen.jerk != transaction_.original_float(dima::params::RO_JERK_LIM) ? Status::NAV_FIELD_JERK : 0U) |
        (chosen.reduction != transaction_.original_float(dima::params::RO_SPEED_RED) ? Status::NAV_FIELD_SPEED_REDUCTION : 0U);
    if ((path_observed_fields_[best_path_trial_] & required) != required) {
        fail_tuning(Status::FAILURE_PATH_UNOBSERVABLE, now);
        return;
    }
    status_.navigation_observed_fields |= path_observed_fields_[best_path_trial_];
    if ((path_observed_fields_[best_path_trial_] & Status::NAV_FIELD_LOOKAHEAD) != 0U)
        status_.provisional_validated_stages |= Status::STAGE_PATH_GAIN;
    else status_.unavailable_stages |= Status::STAGE_PATH_GAIN;
    if ((status_.navigation_observed_fields & (Status::NAV_FIELD_JERK | Status::NAV_FIELD_SPEED_REDUCTION)) !=
        (Status::NAV_FIELD_JERK | Status::NAV_FIELD_SPEED_REDUCTION)) status_.unavailable_stages |= Status::STAGE_NAV_STRATEGY;
    if (!transaction_.finalize_provisional(now, expected_set_count_)) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
    transition(Status::STATE_SAVE_GAINS, now);
}

} // namespace dima::rover::modes

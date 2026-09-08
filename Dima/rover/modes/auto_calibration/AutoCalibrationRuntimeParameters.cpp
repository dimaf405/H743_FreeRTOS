#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "auto/AutoMode.hpp"
#include "control/RoverDifferential.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;
namespace {

bool origin_gain(const math::MotorResponseProfile &profile, unsigned direction, float noise, float &gain,
    const dima::lib::rover::DifferentialDriveConfig *mapping = nullptr) noexcept
{
    if (profile.failure() != math::ResponseFailure::None || !std::isfinite(noise) || noise <= 0.0F) return false;
    double uu = 0.0, uv = 0.0, vv = 0.0, sum = 0.0, weight = 0.0;
    unsigned levels = 0U;
    for (unsigned i = 0U; i < math::MotorResponseProfile::kLevels; ++i) {
        const auto *p = profile.plateau(direction, i);
        if (p == nullptr || p->count() < 10U || p->raw_speed.duration_s() < 1.0F) continue;
        double u = p->pre_input.mean();
        const double v = p->raw_speed.mean();
        // 与纯响应库的真实平台判据一致：不能把仍缓升的命令均值或一次
        // 中断前后的拼接样本作为静态 FF。零样本方差允许存在，不伪造噪声底。
        if (p->pre_input.count() != p->count() || p->applied_input.count() != p->count() ||
            p->pre_input.failure() != math::ResponseFailure::None ||
            p->applied_input.failure() != math::ResponseFailure::None || p->raw_speed.failure() != math::ResponseFailure::None ||
            !std::isfinite(p->raw_speed.standard_deviation()) ||
            p->pre_input.standard_deviation() > std::max(0.002, 0.02 * u) ||
            p->applied_input.standard_deviation() > std::max(0.002, 0.02 * p->applied_input.mean()) ||
            p->raw_speed.standard_deviation() > std::max(3.0 * noise, 0.10 * std::fabs(v))) continue;
        if (mapping != nullptr) {
            const double scale = mapping->throttle_max - mapping->throttle_min;
            if (scale <= 0.0) return false;
            const double r = (p->applied_input.mean() - mapping->throttle_min) / scale;
            if (r <= 0.0 || r > 1.0) continue;
            // 从已测轮端量反解新整形对应的 pre 坐标，仅作重采样前的激励
            // 先验。新整形应用后仍必须再次实采 FF，不能直接据此最终保存。
            u = ((1.0 - mapping->throttle_expo) * r + mapping->throttle_expo * r * r) /
                (direction == 0U ? 1.0 : mapping->thrust_asymmetry);
        }
        if (!std::isfinite(u) || !std::isfinite(v) || u <= 0.0 || v <= 5.0F * noise) continue;
        const double w = p->count();
        uu += w * u * u; uv += w * u * v; vv += w * v * v; sum += w * v; weight += w;
        ++levels;
    }
    if (levels < 3U || uu <= 1.0e-10 || weight <= 0.0 || sum <= 0.0) return false;
    gain = static_cast<float>(uv / uu);
    // FF 是过原点的前馈，不是局部微分增益；非零整形导致显著截距/弯曲时
    // 不能仅靠一个正比值冒充线性映射。闭环还会重新验证这份候选。
    const double rms = std::sqrt(std::max(0.0, vv - uv * uv / uu) / weight);
    return std::isfinite(gain) && gain > 0.0F && gain <= 1000.0F && rms <= 0.10 * sum / weight;
}

} // namespace

bool AutoCalibrationMode::calculate_runtime_candidates(std::uint64_t now) noexcept
{
    if (!runtime_cohort_) {
        const auto motor = math::design_motor_response(response_speed_, {status_.session_motor_limit, response_noise_[0],
            tuning_motor_.throttle_min, tuning_motor_.throttle_expo, tuning_motor_.thrust_asymmetry});
        motor_candidate_changed_ = motor.complete() && (motor.min_output != tuning_motor_.throttle_min ||
            motor.expo != tuning_motor_.throttle_expo || motor.asymmetry != tuning_motor_.thrust_asymmetry);
        if (motor_candidate_changed_ && now - session_started_ + 420000000ULL > 600000000ULL) {
            // 全部重新profile、两次消费者确认以及后续关联验证必须有预算。
            // 当前场地/倒车界使多数车辆不能满足此条件，不为追求“全自动”强行应用。
            motor_candidate_changed_ = false;
            status_.unavailable_stages |= Status::STAGE_MOTOR_PROFILE;
            PX4_INFO_RAW("[autocal] MIN/EXPO/ASYM retained: no complete reprofile/validation budget\n");
        } else if (motor.complete()) {
            motor_profile_reference_ = motor;
            tuning_motor_.throttle_min = motor.min_output;
            tuning_motor_.throttle_expo = motor.expo;
            tuning_motor_.thrust_asymmetry = motor.asymmetry;
            motor_profile_verified_ = !motor_candidate_changed_;
        } else {
            status_.unavailable_stages |= Status::STAGE_MOTOR_PROFILE;
            PX4_INFO_RAW("[autocal] MIN/EXPO/ASYM retained: insufficient forward/reverse coverage or start brackets\n");
        }
    } else if (motor_candidate_changed_ && motor_reprofiled_) {
        const auto repeated = math::design_motor_response(response_speed_, {status_.session_motor_limit, response_noise_[0],
            tuning_motor_.throttle_min, tuning_motor_.throttle_expo, tuning_motor_.thrust_asymmetry});
        // 重复起步/停车/反向起步必须重新给出相容括界，不只重算一个正 FF。
        // 新 MIN 掩住原先“不动”区间时，本硬件/动作不能重复证明，宁可整组回滚。
        if (!repeated.complete() || tuning_motor_.throttle_min < repeated.minimum_lower_bound ||
            tuning_motor_.throttle_min > repeated.minimum_upper_bound ||
            std::max(repeated.minimum_lower_bound, motor_profile_reference_.minimum_lower_bound) >=
                std::min(repeated.minimum_upper_bound, motor_profile_reference_.minimum_upper_bound) ||
            std::fabs(repeated.expo - tuning_motor_.throttle_expo) > 0.10F ||
            std::fabs(repeated.asymmetry - tuning_motor_.thrust_asymmetry) > 0.10F * tuning_motor_.thrust_asymmetry) {
            PX4_INFO_RAW("[autocal] shaping reprofile rejected: coverage/start brackets or curves changed\n");
            return false;
        }
        motor_profile_verified_ = true;
    }
    float forward{}, reverse{}, clockwise{}, counterclockwise{};
    const auto *mapping = motor_candidate_changed_ && !motor_reprofiled_ ? &tuning_motor_ : nullptr;
    bool rate_fit = origin_gain(response_rate_, 0U, response_noise_[1], clockwise) &&
        origin_gain(response_rate_, 1U, response_noise_[1], counterclockwise);
    if (!rate_fit && mapping != nullptr) {
        const auto cw = response_rate_.gain_lower_bound(0U, response_noise_[1]);
        const auto ccw = response_rate_.gain_lower_bound(1U, response_noise_[1]);
        rate_fit = cw.valid() && ccw.valid();
        clockwise = cw.value; counterclockwise = ccw.value;
    }
    if (!origin_gain(response_speed_, 0U, response_noise_[0], forward, mapping) ||
        !origin_gain(response_speed_, 1U, response_noise_[0], reverse, mapping) || !rate_fit ||
        std::fabs(clockwise - counterclockwise) > 0.20F * 0.5F * (clockwise + counterclockwise)) return false;
    const auto forward_speed = response_speed_.speed_lower_bound(0U, response_noise_[0]);
    const auto rate_cw = response_rate_.speed_lower_bound(0U, response_noise_[1]);
    const auto rate_ccw = response_rate_.speed_lower_bound(1U, response_noise_[1]);
    if (!forward_speed.valid() || !rate_cw.valid() || !rate_ccw.valid()) return false;
    runtime_ff_speed_ = forward;
    runtime_ff_yaw_ = 2.0F * forward / (config_.track * 0.5F * (clockwise + counterclockwise));
    if (!std::isfinite(runtime_ff_yaw_) || runtime_ff_yaw_ < 0.01F || runtime_ff_yaw_ > 100.0F || forward > 100.0F) return false;
    auto &c = tuning_config_;
    // 前进巡航与低速倒车是不同声明范围，不能因倒车只测到 0.3 就假装已
    // 验证同样高速倒车；前进取实测下界与普通校准输出余量的较小者。
    c.speed_limit = std::min({0.8F * fence_.speed_limit_m_s, 0.8F * forward_speed.value,
                             0.75F * forward * config_.throttle});
    c.rate_limit = std::min({0.48F, 0.8F * rate_cw.value, 0.8F * rate_ccw.value,
                            0.75F * 0.5F * (clockwise + counterclockwise) * config_.steering});
    c.speed_threshold = 3.0F * response_noise_[0]; c.rate_threshold = 3.0F * response_noise_[1];
    if (!std::isfinite(c.speed_limit) || c.speed_limit <= 0.0F || !std::isfinite(c.rate_limit) || c.rate_limit <= 0.0F ||
        c.speed_threshold >= std::min(0.08F, 0.1F * c.speed_limit) ||
        c.rate_threshold >= std::min(0.05F, 0.1F * c.rate_limit)) return false;
    float *limits[4]{&c.acceleration, &c.deceleration, &c.rate_acceleration, &c.rate_deceleration};
    runtime_fully_observed_ = true;
    runtime_observed_mask_ = 0U;
    for (unsigned i = 0U; i < 4U; ++i) {
        if (std::isfinite(response_rate_lower_[i]) && response_rate_lower_[i] > 0.0F) {
            *limits[i] = std::min(1.5F, 0.5F * response_rate_lower_[i]);
            runtime_observed_mask_ |= static_cast<std::uint8_t>(1U << i);
        } else {
            runtime_fully_observed_ = false;
            // 保留本来合法的配置，不给缺少观测的 -1/0 填任意正数开门。
            if (!std::isfinite(*limits[i]) || *limits[i] <= 0.0F) return false;
        }
    }
    if (!std::isfinite(c.jerk) || c.jerk <= 0.0F) {
        if (!std::isfinite(response_settle_s_) || response_settle_s_ <= 0.0F) return false;
        c.jerk = 2.0F * c.deceleration / std::max(0.5F, response_settle_s_);
    }
    if (!std::isfinite(c.jerk) || c.jerk <= 0.0F || c.jerk > 100.0F) return false;
    c.driving.stopped_speed_threshold_m_s = c.speed_threshold;
    // 三个全局整形量不能用局部正反比独立提交。当前倒车/空间无法覆盖时
    // 明确保留 Manual 共用的原值，不新加 Manual 限制来制造成功。
    // SLEW 若没有全局动态覆盖和独立比较证据则保持原值；不能从静态曲线
    // 生成一个更快的值。本会话的完整效果由随后真实 FF/PI/路径复验约束。
    status_.maximum_speed_m_s = runtime_ff_speed_;
    status_.yaw_rate_correction = runtime_ff_yaw_;
    return true;
}

bool AutoCalibrationMode::slew_evidence_valid() const noexcept
{
    if (!slew_probe_eligible_ || slew_evidence_.failed_mask != 0U || slew_evidence_.active_directions != 0x0FU ||
        response_speed_.global_coverage(status_.session_motor_limit, response_noise_[0]) != math::ResponseFailure::None) return false;
    unsigned windows{};
    for (unsigned i = 0U; i < 21U; ++i) if ((slew_evidence_.observed_mask & (1U << i)) != 0U) ++windows;
    for (unsigned direction = 0U; direction < 2U; ++direction) {
        if (!std::isfinite(slew_evidence_.active_min[direction]) ||
            slew_evidence_.active_min[direction] > 0.25F * status_.session_motor_limit ||
            slew_evidence_.active_max[direction] < 0.75F * status_.session_motor_limit) return false;
    }
    return windows >= 6U && std::isfinite(slew_evidence_.noise_ratio) && slew_evidence_.noise_ratio > 0.0F;
}

bool AutoCalibrationMode::select_slew_candidate(std::uint64_t now) noexcept
{
    if (slew_trial_ != SlewTrial::Baseline || (motor_candidate_changed_ && !motor_profile_verified_)) return false;
    if (!slew_evidence_valid() || now - session_started_ + 360000000ULL > 600000000ULL) {
        slew_trial_ = SlewTrial::Complete;
        PX4_INFO_RAW("[autocal] MOT_SLEW_RATE retained: incomplete bidirectional coverage, masked slew, or no trial budget\n");
        return false;
    }
    const float denominator = 1.0F - std::fabs(tuning_motor_.throttle_expo);
    const float derivative = denominator > 0.0F ? (tuning_motor_.throttle_max - tuning_motor_.throttle_min) *
        tuning_motor_.thrust_asymmetry / denominator : INFINITY;
    // 只实跑一个邻近候选：已有明显超调先考察0.8，否则先考察1.25；这只是
    // 试验顺序，必须实测改善才接纳。另一项仅为非法/不可观候选的有界备选。
    for (unsigned i = 0U; i < 2U; ++i) {
        const bool slower = (slew_evidence_.peak_overshoot > 0.10F) != (i != 0U);
        const float candidate = slew_original_ * (slower ? 0.8F : 1.25F);
        if (!std::isfinite(candidate) || candidate <= 0.0F || candidate >= 0.15F || candidate > 10.0F ||
            !std::isfinite(derivative) || candidate * derivative > 0.135F) continue;
        slew_baseline_ = slew_evidence_;
        slew_trial_ = SlewTrial::Candidate;
        tuning_motor_.throttle_slew_rate = candidate;
        slew_reprofile_pending_ = true;
        PX4_INFO_RAW("[autocal] MOT_SLEW_RATE RAM comparison %.3f -> %.3f; full affected profile required\n",
            static_cast<double>(slew_original_), static_cast<double>(candidate));
        return true;
    }
    slew_trial_ = SlewTrial::Complete;
    PX4_INFO_RAW("[autocal] MOT_SLEW_RATE retained: neighboring candidates hidden by safety slew\n");
    return false;
}

bool AutoCalibrationMode::finish_slew_trial(std::uint64_t now) noexcept
{
    if (slew_trial_ == SlewTrial::Baseline || slew_trial_ == SlewTrial::Complete) return false;
    bool comparable = slew_evidence_valid() && slew_evidence_.observed_mask == slew_baseline_.observed_mask;
    double current{}, baseline{};
    unsigned windows{};
    for (unsigned i = 0U; i < 21U; ++i) {
        if ((slew_baseline_.observed_mask & (1U << i)) == 0U) continue;
        comparable = comparable && std::fabs(slew_evidence_.input[i] - slew_baseline_.input[i]) <=
            std::max(0.002F, 0.02F * slew_baseline_.input[i]) &&
            std::fabs(slew_evidence_.target[i] - slew_baseline_.target[i]) <=
            std::max(3.0F * std::max(slew_evidence_.noise, slew_baseline_.noise), 0.10F * slew_baseline_.target[i]);
        current += slew_evidence_.error[i]; baseline += slew_baseline_.error[i]; ++windows;
    }
    if (windows != 0U) { current /= windows; baseline /= windows; }
    const bool improved = comparable && windows != 0U && baseline - current >
        std::max(0.10 * baseline, 3.0 * std::max(slew_evidence_.noise_ratio, slew_baseline_.noise_ratio)) &&
        slew_evidence_.peak_overshoot <= std::max(0.05F, slew_baseline_.peak_overshoot);
    if (slew_trial_ == SlewTrial::Restore) {
        if (!comparable) { fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true; }
        slew_trial_ = SlewTrial::Complete; motor_slew_verified_ = true;
        PX4_INFO_RAW("[autocal] MOT_SLEW_RATE original restored and reprofiled; no significant improvement\n");
        return false;
    }
    if (improved) {
        slew_trial_ = SlewTrial::Complete;
        motor_slew_verified_ = motor_slew_changed_ = true;
        PX4_INFO_RAW("[autocal] MOT_SLEW_RATE response improved; RAM-only until FF/PI/navigation pass\n");
        return false;
    }
    // 未胜出时不拿原参数旧代的统计为恢复代保存。预算允许就恢复并重新采集，
    // 否则回滚整个当前关联组，保留先前独立成功阶段，不留下未验证候选。
    if (now - session_started_ + 360000000ULL > 600000000ULL) {
        PX4_INFO_RAW("[autocal] MOT_SLEW_RATE trial inconclusive; no restore-profile budget, cohort rollback\n");
        fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true;
    }
    tuning_motor_.throttle_slew_rate = slew_original_;
    slew_trial_ = SlewTrial::Restore; slew_reprofile_pending_ = true;
    if (!begin_runtime_transaction(now)) fail_tuning(Status::FAILURE_PARAMETER, now);
    return true;
}

bool AutoCalibrationMode::begin_runtime_transaction(std::uint64_t now) noexcept
{
    const bool replacing = runtime_cohort_;
    if (armed_.armed() || (replacing ? transaction_.phase() != CalibrationParameters::Phase::Provisional : !transaction_.prepare())) return false;
    const auto set = [&](dima::params parameter, float value) {
        return replacing ? transaction_.revise_float(parameter, value) : transaction_.add_float(parameter, value);
    };
    const auto &c = tuning_config_;
    // 组成员只引用权威生成的参数标识；32 槽覆盖 24 项关联参数，最初旧值
    // 在后续 PI、Heading、路径候选更新时一直保留，直到整体保存或回滚。
    const bool added =
        set(dima::params::MOT_THR_MIN, tuning_motor_.throttle_min) &&
        set(dima::params::MOT_THR_EXPO, tuning_motor_.throttle_expo) &&
        set(dima::params::MOT_THR_ASYM, tuning_motor_.thrust_asymmetry) &&
        set(dima::params::MOT_SLEW_RATE, tuning_motor_.throttle_slew_rate) &&
        set(dima::params::RO_MAX_THR_SPEED, runtime_ff_speed_) &&
        set(dima::params::RO_YAW_RATE_CORR, runtime_ff_yaw_) &&
        set(dima::params::RO_SPEED_P, c.inner[0]) && set(dima::params::RO_SPEED_I, c.inner[1]) &&
        set(dima::params::RO_YAW_RATE_P, c.inner[2]) && set(dima::params::RO_YAW_RATE_I, c.inner[3]) &&
        set(dima::params::RO_SPEED_LIM, c.speed_limit) && set(dima::params::RO_YAW_RATE_LIM, c.rate_limit / kRadians) &&
        set(dima::params::RO_ACCEL_LIM, c.acceleration) && set(dima::params::RO_DECEL_LIM, c.deceleration) &&
        set(dima::params::RO_YAW_ACCEL_LIM, c.rate_acceleration / kRadians) &&
        set(dima::params::RO_YAW_DECEL_LIM, c.rate_deceleration / kRadians) &&
        set(dima::params::RO_SPEED_TH, c.speed_threshold) &&
        set(dima::params::RO_YAW_RATE_TH, c.rate_threshold / kRadians) &&
        set(dima::params::RO_JERK_LIM, c.jerk) && set(dima::params::RO_SPEED_RED, c.speed_reduction) &&
        set(dima::params::RO_YAW_P, c.heading_p) &&
        set(dima::params::RD_TRANS_TRN_DRV, c.driving.turn_to_drive_yaw_error_rad) &&
        set(dima::params::RD_TRANS_DRV_TRN, c.driving.drive_to_turn_yaw_error_rad) &&
        set(dima::params::PP_LOOKAHD_GAIN, c.pursuit.lookahead_gain);
    runtime_cohort_ = true;
    transaction_stage_ = Status::STATE_APPLY_RUNTIME;
    if (!added || !(replacing ? transaction_.apply_revisions(now, expected_set_count_)
                             : transaction_.apply(now, expected_set_count_, true))) return false;
    status_.closed_loop = false;
    transition(Status::STATE_APPLY_RUNTIME, now);
    return true;
}

bool AutoCalibrationMode::runtime_frontend_confirmed() const noexcept
{
    return transaction_.generation_valid() && drive_.calibration_generation_applied(transaction_.generation()) &&
        drive_.calibration_parameters_applied(transaction_.generation(), transaction_.expected_float(dima::params::RO_MAX_THR_SPEED),
            transaction_.expected_float(dima::params::RO_YAW_RATE_CORR)) &&
        navigation_.calibration_parameters_applied(transaction_.generation(), transaction_.expected_float(dima::params::RO_YAW_P),
            transaction_.expected_float(dima::params::PP_LOOKAHD_GAIN));
}

bool AutoCalibrationMode::revise_runtime_candidates() noexcept
{
    if (status_.gain_group == Status::GAIN_INNER) {
        return transaction_.revise_float(dima::params::RO_SPEED_P, gains_[0]) &&
            transaction_.revise_float(dima::params::RO_SPEED_I, gains_[1]) &&
            transaction_.revise_float(dima::params::RO_YAW_RATE_P, gains_[2]) &&
            transaction_.revise_float(dima::params::RO_YAW_RATE_I, gains_[3]) &&
            transaction_.revise_float(dima::params::RO_SPEED_LIM, tuning_config_.speed_limit) &&
            transaction_.revise_float(dima::params::RO_YAW_RATE_LIM, tuning_config_.rate_limit / kRadians) &&
            transaction_.revise_float(dima::params::RO_MAX_THR_SPEED, status_.maximum_speed_m_s) &&
            transaction_.revise_float(dima::params::RO_YAW_RATE_CORR, status_.yaw_rate_correction);
    }
    if (status_.gain_group == Status::GAIN_HEADING)
        return transaction_.revise_float(dima::params::RO_YAW_P, status_.heading_p) &&
            transaction_.revise_float(dima::params::RD_TRANS_TRN_DRV, tuning_config_.driving.turn_to_drive_yaw_error_rad) &&
            transaction_.revise_float(dima::params::RD_TRANS_DRV_TRN, tuning_config_.driving.drive_to_turn_yaw_error_rad);
    if (status_.gain_group == Status::GAIN_NAVIGATION)
        return transaction_.revise_float(dima::params::RD_TRANS_TRN_DRV, tuning_config_.driving.turn_to_drive_yaw_error_rad) &&
            transaction_.revise_float(dima::params::RD_TRANS_DRV_TRN, tuning_config_.driving.drive_to_turn_yaw_error_rad);
    return transaction_.revise_float(dima::params::PP_LOOKAHD_GAIN, status_.lookahead_gain) &&
        transaction_.revise_float(dima::params::RO_JERK_LIM, tuning_config_.jerk) &&
        transaction_.revise_float(dima::params::RO_SPEED_RED, tuning_config_.speed_reduction);
}

} // namespace dima::rover::modes

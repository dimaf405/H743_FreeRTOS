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

bool origin_gain(const math::MotorResponseProfile &profile, unsigned direction, float noise, float &gain) noexcept
{
    if (profile.failure() != math::ResponseFailure::None || !std::isfinite(noise) || noise <= 0.0F) return false;
    double uu = 0.0, uv = 0.0, vv = 0.0, sum = 0.0, weight = 0.0;
    unsigned levels = 0U;
    for (unsigned i = 0U; i < math::MotorResponseProfile::kLevels; ++i) {
        const auto *p = profile.plateau(direction, i);
        if (p == nullptr || p->count() < 10U || p->raw_speed.duration_s() < 1.0F) continue;
        const double u = p->pre_input.mean();
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
    (void)now;
    // 前进能力可辨识 FF/运行值；全局电机整形需要正反域，本模式明确跳过。
    float forward{}, clockwise{}, counterclockwise{};
    bool rate_fit = origin_gain(response_rate_, 0U, response_noise_[1], clockwise) &&
        origin_gain(response_rate_, 1U, response_noise_[1], counterclockwise);
    if (!origin_gain(response_speed_, 0U, response_noise_[0], forward) ||
        !rate_fit ||
        std::fabs(clockwise - counterclockwise) > 0.20F * 0.5F * (clockwise + counterclockwise)) return false;
    const auto forward_speed = response_speed_.speed_lower_bound(0U, response_noise_[0]);
    const auto rate_cw = response_rate_.speed_lower_bound(0U, response_noise_[1]);
    const auto rate_ccw = response_rate_.speed_lower_bound(1U, response_noise_[1]);
    if (!forward_speed.valid() || !rate_cw.valid() || !rate_ccw.valid()) return false;
    runtime_ff_speed_ = forward;
    runtime_ff_yaw_ = 2.0F * forward / (config_.track * 0.5F * (clockwise + counterclockwise));
    if (!std::isfinite(runtime_ff_yaw_) || runtime_ff_yaw_ < 0.01F || runtime_ff_yaw_ > 100.0F || forward > 100.0F) return false;
    auto &c = tuning_config_;
    // 纵向只声明本次实际采到的前进范围；转向仍要求 CW/CCW 一致。
    // 运行候选留出控制余量，不将它当作另一层固定驱动输出上限。
    c.speed_limit = std::min({0.8F * fence_.speed_limit_m_s, 0.8F * forward_speed.value,
                             0.75F * forward * config_.motor_maximum});
    c.rate_limit = std::min({0.48F, 0.8F * rate_cw.value, 0.8F * rate_ccw.value,
                            0.75F * 0.5F * (clockwise + counterclockwise)});
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
    // 当前模式保留 Manual 共用电机参数，只提交随后可被真实 FF/PI/路径验证的运行值。
    status_.maximum_speed_m_s = runtime_ff_speed_;
    status_.yaw_rate_correction = runtime_ff_yaw_;
    return true;
}

bool AutoCalibrationMode::begin_runtime_transaction(std::uint64_t now) noexcept
{
    if (armed_.armed() || runtime_cohort_ || !transaction_.prepare()) return false;
    const auto set = [&](dima::params parameter, float value) {
        return transaction_.add_float(parameter, value);
    };
    const auto &c = tuning_config_;
    // 组成员只引用权威生成的参数标识；32 槽保存关联运行/增益参数，最初旧值
    // 在后续 PI、Heading、路径候选更新时一直保留，直到整体保存或回滚。
    const bool added =
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
    if (!added || !transaction_.apply(now, expected_set_count_, true)) return false;
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

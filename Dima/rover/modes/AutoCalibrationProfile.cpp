#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

bool AutoCalibrationMode::feedback_unmasked() const noexcept
{
    const auto &f = control_feedback_sub_.get();
    // 普通非线性整形不属于保护介入；真正的限幅、等待和末端 slew 才使
    // 当前物理能力不可观。MOT 自身 slew 的活跃性另供策略比较使用。
    return !f.mixing_limited && !f.arm_ramp_active && !f.reversal_held &&
        !f.safety_output_limited && !f.safety_slew_active && !f.saturated;
}

void AutoCalibrationMode::start_response_profile(std::uint64_t now) noexcept
{
    if (!read_tuning_config() || !rtk_yaw_fused(now) || !motion_configuration_valid() ||
        !std::isfinite(rtk_sub_.get().speed_accuracy_m_s) || rtk_sub_.get().speed_accuracy_m_s <= 0.0F ||
        (runtime_cohort_ && tuning_reference_ != 0U && !tuning_estimator_valid(now)) ||
        now - session_started_ > (runtime_cohort_ ? 390000000ULL : 300000000ULL)) {
        fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return;
    }
    px4::AtomicTransaction atomic;
    const auto read = [](dima::params id, auto &value) { return param_get(param_handle(id), &value) == 0; };
    std::int32_t reverse{};
    if (!read(dima::params::MOT_THR_MIN, tuning_motor_.throttle_min) ||
        !read(dima::params::MOT_THR_MAX, tuning_motor_.throttle_max) ||
        !read(dima::params::MOT_THR_EXPO, tuning_motor_.throttle_expo) ||
        !read(dima::params::MOT_THR_ASYM, tuning_motor_.thrust_asymmetry) ||
        !read(dima::params::MOT_SLEW_RATE, tuning_motor_.throttle_slew_rate) ||
        !read(dima::params::MOT_REV_DELAY, tuning_motor_.reversal_delay_s) ||
        !read(dima::params::MOT_ARM_RAMP, tuning_motor_.arm_ramp_s) ||
        !read(dima::params::RD_STR_THR_MIX, tuning_motor_.steering_throttle_mix) ||
        !read(dima::params::RD_REV_STEER, reverse)) { fail_tuning(Status::FAILURE_PARAMETER, now); return; }
    tuning_motor_.reverse_steering_in_manual = reverse != 0;
    if (!runtime_cohort_) {
        slew_trial_ = SlewTrial::Baseline; slew_reprofile_pending_ = false;
        slew_baseline_ = {}; slew_original_ = tuning_motor_.throttle_slew_rate;
        slew_input_ceiling_[0] = slew_input_ceiling_[1] = 0.0F;
        motor_profile_reference_ = {}; motor_profile_verified_ = false;
        motor_slew_verified_ = motor_slew_changed_ = false;
    }
    profile_speed_only_ = slew_trial_ == SlewTrial::Candidate || slew_trial_ == SlewTrial::Restore;
    response_speed_.reset(); response_full_.reset(); noise_speed_.reset(); noise_rate_.reset();
    // 只改纵向 MOT slew 不改变 longitudinal=0 的原地转向映射；保留其已测
    // 物理曲线，正反直线和受影响 FF 必须在新参数代重新采集。
    if (!profile_speed_only_) response_rate_.reset();
    for (unsigned i = 0U; i < (profile_speed_only_ ? 2U : 4U); ++i) response_rate_lower_[i] = INFINITY;
    slew_evidence_ = {};
    slew_evidence_.active_min[0] = slew_evidence_.active_min[1] = INFINITY;
    const float range = tuning_motor_.throttle_max - tuning_motor_.throttle_min;
    const float r = range > 0.0F ? (0.95F * tuning_motor_.throttle_max - tuning_motor_.throttle_min) / range : INFINITY;
    const float reverse_required = ((1.0F - tuning_motor_.throttle_expo) * r + tuning_motor_.throttle_expo * r * r) /
        tuning_motor_.thrust_asymmetry;
    const float derivative_denominator = 1.0F - std::fabs(tuning_motor_.throttle_expo);
    const float derivative = derivative_denominator > 0.0F
        ? range * tuning_motor_.thrust_asymmetry / derivative_denominator : INFINITY;
    // 先排除正常反向 ceiling 结构上到不了端点，以及原 slew 已被末端0.15/s
    // 遮蔽的配置。只有可观机会存在时使用既有0.15/s请求斜率，不放宽任何输出界。
    slew_probe_eligible_ = status_.full_output_requested && r > 0.0F && r <= 1.0F &&
        reverse_required <= config_.throttle && tuning_motor_.throttle_slew_rate > 0.0F &&
        tuning_motor_.throttle_slew_rate < 0.15F && std::isfinite(derivative) &&
        tuning_motor_.throttle_slew_rate * derivative <= 0.135F;
    profile_motion_ = profile_level_ = 0U;
    profile_noise_ready_ = profile_started_ = profile_braking_ = false;
    status_.full_output_reached = false;
    response_last_sample_ = response_phase_started_ = response_stop_started_ = 0U;
    response_settle_s_ = 0.0F;
    response_tail_timestamp_ = 0U;
    response_profile_deadline_ = runtime_cohort_ ? now + (profile_speed_only_ ? 120000000ULL : 180000000ULL) : 0U;
    physical_speed_ = physical_rate_ = longitudinal_ = steering_ = 0.0F;
    status_.closed_loop = false;
    status_.gain_group = Status::GAIN_INNER;
    const auto &p = position_sub_.get();
    tuning_reference_ = p.ref_timestamp;
    tuning_xy_reset_ = p.xy_reset_counter; tuning_vxy_reset_ = p.vxy_reset_counter;
    tuning_yaw_reset_ = p.heading_reset_counter; tuning_odom_reset_ = odometry_sub_.get().reset_counter;
    transition(Status::STATE_WAIT_ARM_PROFILE, now);
}

void AutoCalibrationMode::reset_response_tail() noexcept
{
    response_tail_.reset(); response_tail_timestamp_ = 0U;
    if (profile_level_ >= math::MotorResponseProfile::kLevels) return;
    const bool full = profile_motion_ == 4U, rate = profile_motion_ == 2U || profile_motion_ == 3U;
    auto &profile = full ? response_full_ : rate ? response_rate_ : response_speed_;
    (void)profile.reset_plateau(profile_motion_ == 1U || profile_motion_ == 3U ? 1U : 0U, profile_level_);
}

void AutoCalibrationMode::finish_slew_window(bool rate) noexcept
{
    if (rate || !slew_probe_eligible_ || profile_level_ == 0U) return;
    const unsigned direction = profile_motion_ == 1U ? 1U : 0U;
    const unsigned slot = (profile_motion_ == 4U ? 14U : 7U * direction) + profile_level_;
    if (slot >= 21U || response_tail_.count() < 10U || response_tail_.duration_s() < 1.0F) return;
    const float target = static_cast<float>(response_tail_.mean()), change = target - response_initial_;
    const float noise = response_noise_[0];
    if (!std::isfinite(change) || target <= 5.0F * noise || response_initial_ <= 5.0F * noise ||
        std::fabs(change) <= 10.0F * noise) return;
    double squared{};
    unsigned active{}, count{}, crossings{};
    int previous_sign{};
    std::uint64_t previous{};
    for (std::size_t i = 0U; i < response_sample_count_; ++i) {
        const auto &sample = response_samples_[i];
        if (!sample.usable || (previous != 0U && sample.timestamp - previous > 150000ULL)) {
            slew_evidence_.failed_mask |= 1U << slot; return;
        }
        previous = sample.timestamp;
        const float error = (sample.value - target) / change;
        const int sign = std::fabs(sample.value - target) <= 3.0F * noise ? 0 : (error > 0.0F ? 1 : -1);
        if (sign != 0 && previous_sign != 0 && sign != previous_sign) ++crossings;
        if (sign != 0) previous_sign = sign;
        slew_evidence_.peak_overshoot = std::max(slew_evidence_.peak_overshoot, error);
        squared += error * error; ++count;
        if (sample.motor_slew) ++active;
    }
    if (count < 10U || crossings > 4U || slew_evidence_.peak_overshoot > 0.20F) {
        slew_evidence_.failed_mask |= 1U << slot; return;
    }
    // 用同一平台终值归一化动态误差，随后还逐窗核对候选的输入与终值，
    // 防止“实际跑慢了/激励变小了”赢得评分。降低 slew 从不被默认视作改善。
    slew_evidence_.target[slot] = target;
    slew_evidence_.input[slot] = std::fabs(control_feedback_sub_.get().longitudinal);
    slew_evidence_.error[slot] = static_cast<float>(std::sqrt(squared / count));
    slew_evidence_.noise_ratio = std::max(slew_evidence_.noise_ratio, noise / std::fabs(change));
    slew_evidence_.noise = std::max(slew_evidence_.noise, noise);
    slew_evidence_.observed_mask |= 1U << slot;
    if (active >= 2U) slew_evidence_.active_directions |= static_cast<std::uint8_t>(1U << (2U * direction + (change < 0.0F ? 1U : 0U)));
}

void AutoCalibrationMode::finish_response_window(bool rate) noexcept
{
    if (response_tail_.count() < 10U || response_tail_.duration_s() < 1.0F) {
        response_initial_ = NAN; return;
    }
    const float final = static_cast<float>(response_tail_.mean());
    const float change = final - response_initial_;
    const float noise = response_noise_[rate ? 1U : 0U];
    // 用完整平台终值回看 20%..80% 区间；不以预估目标或含稳态平段的平均
    // 斜率冒充真实过渡。下降段仍为非零输入，不用最后停车替代减速辨识。
    if (profile_level_ != 0U && final > 5.0F * noise && response_initial_ > 5.0F * noise &&
        std::fabs(change) > 5.0F * noise) {
        math::TransientSlope fit;
        for (std::size_t i = 0U; i < response_sample_count_; ++i) {
            const auto &sample = response_samples_[i];
            const float progress = (sample.value - response_initial_) / change;
            if (sample.usable && progress >= 0.2F && progress <= 0.8F)
                (void)fit.add(sample.timestamp, sample.value);
        }
        const auto estimate = fit.lower_confidence_rate(noise);
        if (estimate.valid() && fit.signed_rate() * change > 0.0F) {
            const unsigned index = (rate ? 2U : 0U) + (change < 0.0F ? 1U : 0U);
            response_rate_lower_[index] = std::min(response_rate_lower_[index], estimate.value);
        }
    }
    response_initial_ = final;
}

bool AutoCalibrationMode::step_response_profile(std::uint64_t now) noexcept
{
    if (status_.state == Status::STATE_APPLY_RUNTIME) {
        const bool applied = runtime_frontend_confirmed();
        transaction_.poll(applied, applied && stopped() && rtk_yaw_fused(now), now);
        if (transaction_.phase() == CalibrationParameters::Phase::Provisional) {
            expected_set_count_ = transaction_.set_count_snapshot();
            status_.gains_provisional = true;
            if (motor_candidate_changed_ && !motor_reprofiled_) {
                // 新整形会改变 pre-command 到实际动力的映射，必须重新采集 FF；
                // 初始 FF 只用于有界激励，不能让 PI 积分补偿后冒充重标定。
                motor_reprofiled_ = true;
                start_response_profile(now);
            } else if (slew_reprofile_pending_) {
                slew_reprofile_pending_ = false;
                start_response_profile(now);
            } else begin_identification(now);
        } else if (transaction_.phase() == CalibrationParameters::Phase::Failed) {
            expected_set_count_ = transaction_.set_count_snapshot();
            status_.gains_provisional = false;
            fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
        } else if (transaction_.phase() == CalibrationParameters::Phase::Fault)
            terminate(Status::FAILURE_PARAMETER, false, now);
        return true;
    }
    if (status_.state == Status::STATE_WAIT_ARM_PROFILE) {
        longitudinal_ = steering_ = physical_speed_ = physical_rate_ = 0.0F;
        if (now - state_started_ > 30000000ULL || now - session_started_ > 390000000ULL ||
            (response_profile_deadline_ != 0U && now > response_profile_deadline_)) {
            fail_tuning(Status::FAILURE_TIMEOUT, now); return true;
        }
        if (!rtk_yaw_fused(now) || !tuning_estimator_valid(now) || !stopped() || !fence_result(now).can_stop) {
            if (!profile_noise_ready_) { noise_speed_.reset(); noise_rate_.reset(); response_last_sample_ = 0U; }
            return true;
        }
        const auto &feedback = control_feedback_sub_.get();
        if (!profile_noise_ready_) {
            if (tuning_reference_ == 0U) {
                // 入场尚无本地参考时，首个可用噪声样本之前锁定非零EKF代次；
                // 否则 reference==0 会让整个后续 profile 永久绕过 reset 比较。
                const auto &position = position_sub_.get();
                tuning_reference_ = position.ref_timestamp;
                tuning_xy_reset_ = position.xy_reset_counter; tuning_vxy_reset_ = position.vxy_reset_counter;
                tuning_yaw_reset_ = position.heading_reset_counter; tuning_odom_reset_ = odometry_sub_.get().reset_counter;
            }
            // Disarmed 时输出 valid=false，但新鲜 EKF 测量仍由差速层提供；
            // 使用未零区化速度，不能把 RO_SPEED_TH 已清零的值当作无噪声。
            if (!fresh(feedback.timestamp_sample, now, 200000ULL) || !std::isfinite(feedback.speed_raw_m_s) ||
                !std::isfinite(feedback.yaw_rate_rad_s)) {
                noise_speed_.reset(); noise_rate_.reset(); response_last_sample_ = 0U;
                return true;
            }
            if (feedback.timestamp_sample <= response_last_sample_ ||
                (response_last_sample_ != 0U && feedback.timestamp_sample - response_last_sample_ < 100000ULL)) return true;
            if (response_last_sample_ != 0U && feedback.timestamp_sample - response_last_sample_ > 150000ULL) {
                noise_speed_.reset(); noise_rate_.reset();
            }
            response_last_sample_ = feedback.timestamp_sample;
            if (!noise_speed_.add(response_last_sample_, feedback.speed_raw_m_s) ||
                !noise_rate_.add(response_last_sample_, feedback.yaw_rate_rad_s)) {
                fail_tuning(Status::FAILURE_SENSOR_STALE, now); return true;
            }
            if (noise_speed_.duration_s() < 5.0F || noise_speed_.count() < 50U) return true;
            if (!fresh(imu_status_sub_.get().timestamp, now, 1500000ULL) ||
                imu_status_sub_.get().gyro_device_id != imu_sub_.get().gyro_device_id) {
                noise_speed_.reset(); noise_rate_.reset(); response_last_sample_ = 0U;
                return true;
            }
            float gyro_variance = 0.0F;
            for (float variance : imu_status_sub_.get().var_gyro) {
                if (!std::isfinite(variance) || variance < 0.0F) { fail_tuning(Status::FAILURE_SENSOR_STALE, now); return true; }
                gyro_variance = std::max(gyro_variance, variance);
            }
            response_noise_[0] = std::max(noise_speed_.standard_deviation(), rtk_sub_.get().speed_accuracy_m_s);
            response_noise_[1] = std::max(noise_rate_.standard_deviation(), std::sqrt(gyro_variance));
            if (!std::isfinite(response_noise_[0]) || response_noise_[0] <= 0.0F ||
                !std::isfinite(response_noise_[1]) || response_noise_[1] <= 0.0F ||
                std::fabs(noise_speed_.mean()) > 3.0F * response_noise_[0] ||
                std::fabs(noise_rate_.mean()) > 0.03F) { fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true; }
            profile_noise_ready_ = true;
        }
        if (!prepare_straight(now)) { fail_tuning(Status::FAILURE_FENCE_SPACE, now); return true; }
        status_.awaiting_arm = true;
        if (!armed_.armed()) return true;
        status_.awaiting_arm = false;
        arm_started_ = motion_started_ = now;
        profile_started_ = profile_braking_ = false;
        response_stop_started_ = 0U;
        profile_level_ = 0U;
        leg_lat_ = gps_sub_.get().latitude_deg; leg_lon_ = gps_sub_.get().longitude_deg;
        exercise_heading_ = body_yaw();
        const std::uint8_t next = profile_motion_ == 0U ? Status::STATE_PROFILE_FORWARD
            : profile_motion_ == 1U ? Status::STATE_PROFILE_REVERSE : profile_motion_ == 2U ? Status::STATE_PROFILE_RATE_CW
            : profile_motion_ == 3U ? Status::STATE_PROFILE_RATE_CCW : Status::STATE_PROFILE_FULL;
        transition(next, now);
        return true;
    }
    if (status_.state == Status::STATE_STOP_PROFILE) {
        longitudinal_ = steering_ = 0.0F;
        if (now - state_started_ > 15000000ULL || (response_profile_deadline_ != 0U && now > response_profile_deadline_)) {
            terminate(Status::FAILURE_TIMEOUT, false, now); return true;
        }
        if (armed_.armed()) { if (stopped()) request(auto_calibration_request_s::REQUEST_STAGE_DISARM, now); return true; }
        ++profile_motion_;
        if (profile_speed_only_ && profile_motion_ == 2U) profile_motion_ = 4U;
        if (profile_motion_ < 4U || (profile_motion_ == 4U && status_.full_output_requested && now - session_started_ < 350000000ULL)) {
            if (profile_motion_ == 1U) {
                const auto gain = response_speed_.gain_lower_bound(0U, response_noise_[0]);
                if (!gain.valid()) { fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true; }
                profile_forward_gain_ = gain.value;
            }
            transition(Status::STATE_WAIT_ARM_PROFILE, now);
        } else {
            if (status_.full_output_reached && !response_speed_.replace_direction(0U, response_full_, 0U)) {
                fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true;
            }
            response_profile_deadline_ = 0U;
            if (finish_slew_trial(now)) return true;
            if (!calculate_runtime_candidates(now)) {
                fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
                return true;
            }
            (void)select_slew_candidate(now);
            if (!begin_runtime_transaction(now)) fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
        }
        return true;
    }
    if (status_.state == Status::STATE_PROFILE_FORWARD || status_.state == Status::STATE_PROFILE_REVERSE ||
        status_.state == Status::STATE_PROFILE_RATE_CW || status_.state == Status::STATE_PROFILE_RATE_CCW ||
        status_.state == Status::STATE_PROFILE_FULL) {
        run_response_profile(now); return true;
    }
    return false;
}

void AutoCalibrationMode::run_response_profile(std::uint64_t now) noexcept
{
    const bool full = profile_motion_ == 4U;
    const bool rate = profile_motion_ == 2U || profile_motion_ == 3U;
    const bool reverse = profile_motion_ == 1U;
    const float direction = reverse || profile_motion_ == 3U ? -1.0F : 1.0F;
    const auto &f = control_feedback_sub_.get();
    if (response_profile_deadline_ != 0U && now > response_profile_deadline_) {
        fail_tuning(Status::FAILURE_TIMEOUT, now); return;
    }
    if (!tuning_feedback(now)) {
        longitudinal_ = steering_ = 0.0F;
        if (now - arm_started_ > 250000ULL) terminate(Status::FAILURE_SENSOR_STALE, false, now);
        return;
    }
    const auto fence = fence_result(now);
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg, leg_lat_, leg_lon_, north, east);
    const float speed_guard = reverse ? std::min(0.3F, fence_.speed_limit_m_s) : fence_.speed_limit_m_s;
    if (now - state_started_ > (full ? 60000000ULL : 40000000ULL) ||
        (!rate && std::hypot(north, east) >= leg_distance_) ||
        fence.working_radius_m - fence.distance_m < std::max(0.5F, ground_speed()) ||
        (!rate && ground_speed() >= 0.90F * speed_guard) ||
        (!rate && std::fabs(math::wrap_pi(body_yaw() - exercise_heading_)) > (full ? 3.0F : 10.0F) * kRadians))
        profile_braking_ = true;
    if (profile_braking_) {
        // FULL 减速仍留在 FULL 状态，直到实际发布轮端为零才收回高输出许可；
        // 无论车体是否因阻力提前停下，不能把 0.8 的历史 slew 带进普通阶段。
        longitudinal_ = steering_ = 0.0F;
        if (response_stop_started_ == 0U) response_stop_started_ = now;
        if (stopped() && std::fabs(f.applied_longitudinal) < 1.0e-4F && std::fabs(f.applied_steering) < 1.0e-4F)
            transition(Status::STATE_STOP_PROFILE, now);
        else if (now - response_stop_started_ > 15000000ULL) terminate(Status::FAILURE_TIMEOUT, false, now);
        return;
    }
    if (!profile_started_) {
        profile_started_ = true;
        response_phase_started_ = now; response_sample_count_ = 0U; response_tail_.reset();
        response_tail_timestamp_ = 0U;
        response_initial_ = direction * (rate ? f.yaw_rate_rad_s : f.speed_raw_m_s);
        tuning_sample_ = 0U;
        profile_input_ceiling_ = reverse ? std::min(config_.throttle, 0.75F * speed_guard / profile_forward_gain_)
            : status_.maximum_speed_m_s > 0.0F ? std::min(config_.throttle, 0.75F * speed_guard / status_.maximum_speed_m_s) : config_.throttle;
        if (profile_speed_only_ && !rate && !full) profile_input_ceiling_ = slew_input_ceiling_[reverse ? 1U : 0U];
        else if (!rate && !full) slew_input_ceiling_[reverse ? 1U : 0U] = profile_input_ceiling_;
        profile_rate_gain_ = status_.maximum_speed_m_s > 0.0F && status_.yaw_rate_correction > 0.0F
            ? 2.0F * status_.maximum_speed_m_s / (config_.track * status_.yaw_rate_correction) : 0.0F;
    }
    constexpr float fractions[6]{0.05F, 0.15F, 0.35F, 0.55F, 0.75F, 1.0F};
    const float fraction = profile_level_ < 6U ? fractions[profile_level_] : 0.5F;
    if (rate) {
        longitudinal_ = 0.0F;
        const float target = direction * fraction * 0.35F;
        const float desired = profile_rate_gain_ > 0.0F ? target / profile_rate_gain_
            : steering_ + 0.06F * (target - f.yaw_rate_rad_s) * 0.02F;
        steering_ += std::clamp(std::clamp(desired, -config_.steering, config_.steering) - steering_, -0.0008F, 0.0008F);
    } else {
        const float full_fraction = profile_level_ >= 6U ? 0.5F : profile_level_ == 0U ? 0.05F : profile_level_ == 1U ? 0.15F
            : profile_level_ == 5U ? 1.0F : 0.15F + 0.20F * (profile_level_ - 1U);
        const float desired = full ? full_fraction : direction * fraction * profile_input_ceiling_;
        const float request_step = slew_probe_eligible_ ? 0.003F : 0.001F;
        longitudinal_ += std::clamp(desired - longitudinal_, -request_step, request_step);
        const float turn_limit = std::min(0.05F, 0.2F * std::fabs(longitudinal_));
        steering_ = full ? 0.0F : std::clamp(0.2F * math::wrap_pi(exercise_heading_ - body_yaw()), -turn_limit, turn_limit);
    }
    const std::uint64_t duration = full ? 6000000ULL : 4000000ULL;
    const bool usable = feedback_unmasked() && (rate || (std::fabs(f.yaw_rate_rad_s) < 0.05F &&
        std::fabs(f.lateral_raw_m_s) < std::max(0.05F, 0.15F * std::fabs(f.forward_raw_m_s))));
    const bool settled = rate ? std::fabs(filtered_angular_acceleration_) < 0.10F
        : std::hypot(filtered_acceleration_[0], filtered_acceleration_[1]) < 0.10F;
    // 50Hz安全原因变化也必须立即断开10Hz速度尾窗，不能漏掉两次采样间
    // 短暂的保护介入。间隔按样本时间差计算，不将正常估计输出延迟重复计入。
    if (!usable || !settled || f.motor_slew_active || (response_tail_timestamp_ != 0U &&
        f.timestamp_sample <= response_tail_timestamp_ && now - response_tail_timestamp_ > 150000ULL)) reset_response_tail();
    if (!rate && slew_probe_eligible_ && profile_level_ != 0U && !usable) {
        const unsigned slot = (full ? 14U : reverse ? 7U : 0U) + profile_level_;
        if (slot < 21U) slew_evidence_.failed_mask |= 1U << slot;
    }
    if (take_tuning_sample(now, rate)) {
        const float q = direction * (rate ? f.yaw_rate_rad_s : f.speed_raw_m_s);
        if (response_tail_timestamp_ != 0U && f.timestamp_sample - response_tail_timestamp_ >
            (rate ? 40000ULL : 150000ULL)) reset_response_tail();
        if (f.timestamp_sample >= response_phase_started_ && response_sample_count_ < 256U)
            response_samples_[response_sample_count_++] = {f.timestamp_sample, q, usable, f.motor_slew_active};
        if (!rate && slew_probe_eligible_ && profile_level_ != 0U && usable && f.motor_slew_active) {
            const unsigned side = reverse ? 1U : 0U;
            slew_evidence_.active_min[side] = std::min(slew_evidence_.active_min[side], std::fabs(f.applied_longitudinal));
            slew_evidence_.active_max[side] = std::max(slew_evidence_.active_max[side], std::fabs(f.applied_longitudinal));
        }
        if (usable && settled && !f.motor_slew_active && now - response_phase_started_ >= duration - 1200000ULL) {
            if (response_tail_.count() == 0U && !full)
                response_settle_s_ = std::max(response_settle_s_, 1.0e-6F * static_cast<float>(now - response_phase_started_));
            (void)response_tail_.add(f.timestamp_sample, q);
            response_tail_timestamp_ = f.timestamp_sample;
            if (full && profile_level_ < 6U) {
                if (!response_full_.add(0U, profile_level_, f.timestamp_sample, f.longitudinal, f.applied_longitudinal, f.speed_raw_m_s)) {
                    fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return;
                }
            } else if (profile_level_ < 6U) {
                auto &profile = rate ? response_rate_ : response_speed_;
                if (!profile.add(direction > 0.0F ? 0U : 1U, profile_level_, f.timestamp_sample,
                    rate ? f.steering : f.longitudinal, rate ? f.applied_steering : f.applied_longitudinal,
                    rate ? f.yaw_rate_rad_s : f.speed_raw_m_s)) {
                    fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return;
                }
            }
        }
        if (!rate) {
            if (reverse) status_.observed_reverse_speed_m_s = std::max(status_.observed_reverse_speed_m_s, q);
            else status_.observed_forward_speed_m_s = std::max(status_.observed_forward_speed_m_s, q);
            status_.observed_output_max = std::max(status_.observed_output_max, std::fabs(f.applied_longitudinal));
        }
        if (full && usable && f.applied_longitudinal >= 0.995F * status_.session_motor_limit &&
            std::fabs(f.applied_steering) < 0.005F && response_tail_.count() >= 10U && response_tail_.duration_s() >= 1.0F)
            status_.full_output_reached = true; // 发布命令端点，不是 RPM。
    }
    if (profile_level_ >= 3U && now - state_started_ > 8000000ULL && (rate ? std::fabs(f.yaw_rate_rad_s) : std::fabs(f.speed_raw_m_s)) <
        3.0F * response_noise_[rate ? 1U : 0U]) profile_braking_ = true;
    if (now - response_phase_started_ < duration) return;
    finish_slew_window(rate);
    if (!full) finish_response_window(rate);
    else response_initial_ = response_tail_.count() >= 10U ? static_cast<float>(response_tail_.mean()) : NAN;
    if (!profile_speed_only_ && !full && !rate && !reverse && profile_level_ == 0U && response_tail_.count() >= 10U &&
        response_tail_.mean() > 5.0F * response_noise_[0] && f.longitudinal > 1.0e-5F) {
        // 首个可靠低档响应即可收紧后续激励，避免未知高速度系数使第二档
        // 必然撞上地速上限。只是试验幅值选择，不据此保存前馈或扩大边界。
        const float observed_gain = static_cast<float>(response_tail_.mean()) / f.longitudinal;
        profile_input_ceiling_ = std::min(profile_input_ceiling_, 0.75F * fence_.speed_limit_m_s / observed_gain);
        slew_input_ceiling_[0] = profile_input_ceiling_;
    }
    if (rate && response_tail_.count() >= 10U && response_tail_.mean() > 5.0F * response_noise_[1] && std::fabs(f.steering) > 1.0e-4F)
        profile_rate_gain_ = static_cast<float>(response_tail_.mean()) / std::fabs(f.steering);
    response_tail_.reset(); response_sample_count_ = 0U;
    response_tail_timestamp_ = 0U;
    response_phase_started_ = now;
    if (++profile_level_ >= (full && !slew_probe_eligible_ ? 6U : 7U)) profile_braking_ = true;
}

} // namespace dima::rover::modes

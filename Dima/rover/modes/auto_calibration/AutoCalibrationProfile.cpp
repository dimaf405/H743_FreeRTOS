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
        now - session_started_ > 300000000ULL) {
        fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return;
    }
    px4::AtomicTransaction atomic;
    if (param_get(param_handle(dima::params::MOT_SLEW_RATE), &tuning_motor_slew_) != 0 ||
        param_get(param_handle(dima::params::MOT_ARM_RAMP), &tuning_arm_ramp_) != 0 ||
        !std::isfinite(tuning_motor_slew_) || tuning_motor_slew_ < 0.0F ||
        !std::isfinite(tuning_arm_ramp_) || tuning_arm_ramp_ < 0.0F) {
        fail_tuning(Status::FAILURE_PARAMETER, now); return;
    }
    // 前进模式只辨识运行范围；全局 MIN/EXPO/ASYM/SLEW 不试改、不伪造反向证据。
    response_speed_.reset(); response_rate_.reset(); noise_speed_.reset(); noise_rate_.reset();
    for (float &value : response_rate_lower_) value = INFINITY;
    profile_motion_ = profile_level_ = 0U;
    profile_noise_ready_ = profile_started_ = profile_braking_ = false;
    response_last_sample_ = response_phase_started_ = response_stop_started_ = 0U;
    response_settle_s_ = 0.0F;
    response_tail_timestamp_ = profile_motion_deadline_ = profile_phase_deadline_ = profile_stable_since_ = 0U;
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
    status_.excitation_samples = 0U;
    if (profile_level_ >= math::MotorResponseProfile::kLevels) return;
    const bool rate = profile_motion_ == 1U || profile_motion_ == 2U;
    auto &profile = rate ? response_rate_ : response_speed_;
    (void)profile.reset_plateau(profile_motion_ == 2U ? 1U : 0U, profile_level_);
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
            begin_identification(now);
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
        if (now - state_started_ > 30000000ULL || now - session_started_ > 390000000ULL) {
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
        const bool space = profile_motion_ == 0U ? prepare_straight(now) : prepare_profile_turn(now);
        if (!space) { fail_tuning(Status::FAILURE_FENCE_SPACE, now); return true; }
        status_.awaiting_arm = true;
        if (!armed_.armed()) return true;
        status_.awaiting_arm = false;
        arm_started_ = motion_started_ = now;
        profile_started_ = profile_braking_ = false;
        response_stop_started_ = 0U;
        profile_level_ = 0U;
        leg_lat_ = gps_sub_.get().latitude_deg; leg_lon_ = gps_sub_.get().longitude_deg;
        exercise_heading_ = body_yaw();
        // 序列重编号（无倒退）：0=前进、1=CW、2=CCW；3U 为收尾哨兵。
        const std::uint8_t next = profile_motion_ == 0U ? Status::STATE_PROFILE_FORWARD
            : profile_motion_ == 1U ? Status::STATE_PROFILE_RATE_CW : Status::STATE_PROFILE_RATE_CCW;
        transition(next, now);
        return true;
    }
    if (status_.state == Status::STATE_STOP_PROFILE) {
        longitudinal_ = steering_ = 0.0F;
        if (now - state_started_ > 15000000ULL) {
            terminate(Status::FAILURE_TIMEOUT, false, now); return true;
        }
        if (armed_.armed()) { if (stopped()) request(auto_calibration_request_s::REQUEST_STAGE_DISARM, now); return true; }
        ++profile_motion_;
        if (profile_motion_ < 3U) {
            if (profile_motion_ == 1U) {
                const auto gain = response_speed_.gain_lower_bound(0U, response_noise_[0]);
                if (!gain.valid()) { fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return true; }
            }
            transition(Status::STATE_WAIT_ARM_PROFILE, now);
        } else {
            profile_motion_deadline_ = 0U;
            if (!calculate_runtime_candidates(now)) {
                fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
                return true;
            }
            if (!begin_runtime_transaction(now)) fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now);
        }
        return true;
    }
    if (status_.state == Status::STATE_PROFILE_FORWARD ||
        status_.state == Status::STATE_PROFILE_RATE_CW || status_.state == Status::STATE_PROFILE_RATE_CCW) {
        run_response_profile(now); return true;
    }
    return false;
}

} // namespace dima::rover::modes

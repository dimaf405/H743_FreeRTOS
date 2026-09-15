#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

void AutoCalibrationMode::begin_profile_window(std::uint64_t now, bool rate) noexcept
{
    const auto &feedback = control_feedback_sub_.get();
    const float direction = profile_motion_ == 2U ? -1.0F : 1.0F;
    // 六档覆盖“已证实能动的下界→当前允许上界”，另加一档非零下降响应。
    // 弱动力起步之前不消耗平台档位，因此不会只剩两个可用的高档样本。
    const float fraction = profile_level_ < 6U ? static_cast<float>(profile_level_) / 5.0F : 0.5F;
    status_.excitation_target = profile_input_floor_ + fraction * (profile_input_ceiling_ - profile_input_floor_);
    status_.excitation_phase = Status::EXCITATION_RAMP;
    status_.excitation_level = profile_level_;
    status_.excitation_samples = 0U;
    response_phase_started_ = now;
    profile_stable_since_ = 0U;
    response_sample_count_ = 0U;
    response_sample_interval_ = rate ? 20000ULL : 100000ULL;
    response_initial_ = direction * (rate ? feedback.yaw_rate_rad_s : feedback.speed_raw_m_s);
    reset_response_tail();
    const float current = std::fabs(rate ? steering_ : longitudinal_);
    const float delta = std::fabs(status_.excitation_target - current);
    float ramp_seconds = delta / (rate ? 0.04F : 0.05F);
    if (!rate && tuning_motor_slew_ > 0.0F)
        ramp_seconds = std::max(ramp_seconds, delta / (config_.motor_maximum * tuning_motor_slew_));
    // 预算同时考虑请求 slew、现有电机 slew 和最终 0.15/s；额外八秒用于
    // 连续稳定与有效采样。所有子窗口仍受本次运动和全会话硬截止约束。
    ramp_seconds = std::max(ramp_seconds, config_.motor_maximum / 0.15F);
    const float budget = std::min(90.0F, ramp_seconds + 8.0F);
    profile_phase_deadline_ = std::min(profile_motion_deadline_, now + static_cast<std::uint64_t>(budget * 1000000.0F));
}

void AutoCalibrationMode::record_response_sample(std::uint64_t timestamp, float value, bool usable) noexcept
{
    if (response_sample_count_ != 0U &&
        (timestamp <= response_samples_[response_sample_count_ - 1U].timestamp ||
         timestamp - response_samples_[response_sample_count_ - 1U].timestamp < response_sample_interval_)) return;
    constexpr std::size_t capacity = sizeof(response_samples_) / sizeof(response_samples_[0]);
    if (response_sample_count_ == capacity) {
        // 慢平台的过渡可能长于 256 个采样点。原位二倍抽稀并保留真实时间戳，
        // 不截掉起始段、不扩大 RAM，也不把抽稀后的数据冒充固定 20 ms。
        for (std::size_t i = 0U; i < capacity / 2U; ++i) response_samples_[i] = response_samples_[2U * i];
        response_sample_count_ = capacity / 2U;
        response_sample_interval_ *= 2U;
    }
    response_samples_[response_sample_count_++] = {timestamp, value, usable};
}

void AutoCalibrationMode::run_response_profile(std::uint64_t now) noexcept
{
    const bool rate = profile_motion_ != 0U;
    const float direction = profile_motion_ == 2U ? -1.0F : 1.0F;
    const float maximum = rate ? 1.0F : config_.motor_maximum;
    const auto &f = control_feedback_sub_.get();
    if (!tuning_feedback(now)) {
        longitudinal_ = steering_ = 0.0F;
        if (now - arm_started_ > 250000ULL) terminate(Status::FAILURE_SENSOR_STALE, false, now);
        return;
    }
    if (!profile_started_) {
        profile_started_ = true;
        profile_motion_deadline_ = std::min(now + 90000000ULL, session_started_ + 585000000ULL);
        profile_stable_since_ = drive_envelope_since_ = 0U;
        response_stop_started_ = tuning_sample_ = 0U;
        profile_level_ = 0U;
        status_.excitation_phase = Status::EXCITATION_PROBE;
        status_.excitation_level = 0U;
        status_.excitation_target = maximum;
        status_.excitation_samples = 0U;
        PX4_INFO("[autocal] profile startup probe; frozen motor envelope %.3f", static_cast<double>(config_.motor_maximum));
    }
    status_.excitation_remaining_s = now < profile_motion_deadline_
        ? static_cast<float>(profile_motion_deadline_ - now) * 1.0e-6F : 0.0F;
    const auto fence = fence_result(now);
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg, leg_lat_, leg_lon_, north, east);
    if (now >= profile_motion_deadline_ || (!rate && std::hypot(north, east) >= leg_distance_) ||
        fence.working_radius_m - fence.distance_m < std::max(0.5F, ground_speed()) ||
        (!rate && ground_speed() >= 0.90F * fence_.speed_limit_m_s) ||
        (!rate && std::fabs(math::wrap_pi(body_yaw() - exercise_heading_)) > 10.0F * kRadians))
        profile_braking_ = true;
    if (profile_braking_) {
        status_.excitation_phase = Status::EXCITATION_BRAKE;
        status_.excitation_target = 0.0F;
        longitudinal_ = steering_ = 0.0F;
        if (response_stop_started_ == 0U) response_stop_started_ = now;
        // 先排空真实发布轮端的末端 slew，再请求内部 Disarm；车体不动不是零输出证明。
        if (stopped() && std::fabs(f.applied_longitudinal) < 1.0e-4F && std::fabs(f.applied_steering) < 1.0e-4F)
            transition(Status::STATE_STOP_PROFILE, now);
        else if (now - response_stop_started_ > 15000000ULL) terminate(Status::FAILURE_TIMEOUT, false, now);
        return;
    }
    const float dt = last_run_ != 0U && now >= last_run_ ? std::min(0.1F, static_cast<float>(now - last_run_) * 1.0e-6F) : 0.02F;
    const float step = (rate ? 0.04F : 0.05F) * dt;
    const float measured = direction * (rate ? f.yaw_rate_rad_s : f.speed_raw_m_s);
    const float noise = response_noise_[rate ? 1U : 0U];
    const bool usable = feedback_unmasked() && (rate || (std::fabs(f.yaw_rate_rad_s) < 0.05F &&
        std::fabs(f.lateral_raw_m_s) < std::max(0.05F, 0.15F * std::fabs(f.forward_raw_m_s))));
    const bool settled = rate ? std::fabs(filtered_angular_acceleration_) < 0.10F
        : std::hypot(filtered_acceleration_[0], filtered_acceleration_[1]) < 0.10F;
    if (status_.excitation_phase == Status::EXCITATION_PROBE) {
        // Arm ramp 尚未结束时保持零请求，避免缓升掩盖起步阈值后继续叠加大输入。
        if (f.arm_ramp_active || now - arm_started_ < static_cast<std::uint64_t>((tuning_arm_ramp_ + 0.25F) * 1000000.0F)) {
            longitudinal_ = steering_ = 0.0F;
            return;
        }
        const bool moving = measured > std::max(rate ? 0.03F : 0.08F, 5.0F * noise);
        const float current = std::fabs(rate ? steering_ : longitudinal_);
        if (!moving && current >= 0.995F * maximum) {
            if (drive_envelope_since_ == 0U) drive_envelope_since_ = now;
            if (now - drive_envelope_since_ >= 8000000ULL) {
                PX4_WARN("[autocal] profile drive envelope exhausted; check MOT_THR_MIN/mechanics/battery");
                fail_tuning(Status::FAILURE_DRIVE_ENVELOPE, now);
                return;
            }
        } else drive_envelope_since_ = 0U;
        if (moving) {
            // 首次运动后先保持当前请求，连续 0.5 s 稳定才锁起步下界，防止
            // 高增益车辆在确认窗口内仍不断加油。未运动则继续探测到真实包络。
            if (usable && settled && !f.motor_slew_active) {
                if (profile_stable_since_ == 0U) profile_stable_since_ = now;
                if (now - profile_stable_since_ >= 500000ULL) {
                    profile_input_floor_ = current;
                    const float pre = std::fabs(rate ? f.steering : f.longitudinal);
                    const float gain = pre > 1.0e-6F ? measured / pre : 0.0F;
                    profile_input_ceiling_ = gain > 0.0F
                        ? maximum * std::min(1.0F, (rate ? 0.35F : 0.75F * fence_.speed_limit_m_s) / gain) : maximum;
                    if (!std::isfinite(profile_input_ceiling_) || profile_input_ceiling_ - current < 0.02F * maximum) {
                        PX4_WARN("[autocal] profile has insufficient observable span above startup");
                        profile_braking_ = true;
                    } else begin_profile_window(now, rate);
                }
            } else profile_stable_since_ = 0U;
        } else {
            profile_stable_since_ = 0U;
            if (rate) steering_ = direction * std::min(maximum, current + step);
            else longitudinal_ = std::min(maximum, current + step);
        }
    } else {
        const float target = direction * status_.excitation_target;
        if (rate) steering_ += std::clamp(target - steering_, -step, step);
        else longitudinal_ += std::clamp(target - longitudinal_, -step, step);
        const float request = rate ? steering_ : longitudinal_ / config_.motor_maximum;
        const float expected = rate ? target : target / config_.motor_maximum;
        const bool at_target = std::fabs(request - expected) <= 1.0e-3F &&
            std::fabs((rate ? f.steering : f.longitudinal) - expected) <= 1.0e-3F;
        const bool stable = at_target && usable && settled && !f.motor_slew_active && measured > 5.0F * noise;
        if (now >= profile_phase_deadline_) {
            // 本档无法在有界时间内取得平台，保留此前合格档并停车；后续 FF
            // 至少三档的原门禁决定是否可继续，不用无效样本填充覆盖度。
            PX4_WARN("[autocal] profile window incomplete; retaining earlier plateaus");
            profile_braking_ = true;
            return;
        }
        if (!stable) {
            profile_stable_since_ = 0U;
            if (status_.excitation_phase == Status::EXCITATION_COLLECT) reset_response_tail();
            status_.excitation_phase = at_target ? Status::EXCITATION_SETTLE : Status::EXCITATION_RAMP;
        } else if (status_.excitation_phase != Status::EXCITATION_COLLECT) {
            status_.excitation_phase = Status::EXCITATION_SETTLE;
            if (profile_stable_since_ == 0U) profile_stable_since_ = now;
            if (now - profile_stable_since_ >= 1000000ULL) status_.excitation_phase = Status::EXCITATION_COLLECT;
        }
        if (take_tuning_sample(now, rate)) {
            record_response_sample(f.timestamp_sample, measured, usable);
            if (response_tail_timestamp_ != 0U && f.timestamp_sample - response_tail_timestamp_ >
                (rate ? 40000ULL : 150000ULL)) {
                reset_response_tail();
                profile_stable_since_ = 0U;
                status_.excitation_phase = Status::EXCITATION_SETTLE;
            }
            if (status_.excitation_phase == Status::EXCITATION_COLLECT) {
                (void)response_tail_.add(f.timestamp_sample, measured);
                response_tail_timestamp_ = f.timestamp_sample;
                status_.excitation_samples = response_tail_.count();
                if (profile_level_ < 6U) {
                    auto &profile = rate ? response_rate_ : response_speed_;
                    if (!profile.add(direction > 0.0F ? 0U : 1U, profile_level_, f.timestamp_sample,
                        rate ? f.steering : f.longitudinal, rate ? f.applied_steering : f.applied_longitudinal,
                        rate ? f.yaw_rate_rad_s : f.speed_raw_m_s)) {
                        fail_tuning(Status::FAILURE_PROFILE_UNOBSERVABLE, now); return;
                    }
                }
                if (!rate) {
                    status_.observed_forward_speed_m_s = std::max(status_.observed_forward_speed_m_s, measured);
                    float actual{};
                    if (motor_history_.forward_output(f.timestamp_sample, actual)) {
                        status_.observed_output_max = std::max(status_.observed_output_max, actual);
                        if (profile_level_ == 5U && !endpoint_reported_ && actual >= 0.995F * config_.motor_maximum) {
                            endpoint_reported_ = true;
                            PX4_INFO("[autocal] command endpoint reached: %.3f of envelope %.3f",
                                static_cast<double>(actual), static_cast<double>(config_.motor_maximum));
                        }
                    }
                }
            }
        }
        if (response_tail_.count() >= 10U && response_tail_.duration_s() >= 1.2F) {
            response_settle_s_ = std::max(response_settle_s_, static_cast<float>(profile_stable_since_ - response_phase_started_) * 1.0e-6F);
            finish_response_window(rate);
            if (++profile_level_ >= 7U) profile_braking_ = true;
            else begin_profile_window(now, rate);
        }
    }
    if (rate) longitudinal_ = 0.0F;
    else {
        const float error = math::wrap_pi(exercise_heading_ - body_yaw());
        const float turn_limit = std::min(0.05F, 0.2F * longitudinal_);
        // 顶档附近在一度航向误差内保持零转向，避免微小修正始终触发混控
        // 限制而使满输出平台无法成立；十度/角速度/围栏硬守卫仍独立有效。
        steering_ = longitudinal_ >= 0.9F * config_.motor_maximum && std::fabs(error) < kRadians
            ? 0.0F : std::clamp(0.2F * error, -turn_limit, turn_limit);
    }
}

} // namespace dima::rover::modes

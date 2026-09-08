#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

void AutoCalibrationMode::collect_baseline(std::uint64_t now) noexcept
{
    if (now - state_started_ > 60000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); return; }
    if (!rtk_quality(now) || !stopped()) { baseline_count_ = 0U; return; }
    if (!new_heading_epoch()) return;
    baseline_samples_[baseline_count_++] = rtk_sub_.get().baseline_m;
    status_.progress = static_cast<std::uint8_t>(10U + baseline_count_ / 10U);
    if (baseline_count_ < 100U) return;
    if (!math::baseline(baseline_samples_, baseline_count_, status_.rtk_baseline_m)) {
        terminate(Status::FAILURE_RTK_INCONSISTENT, false, now); return;
    }
    if (!prepare_straight(now)) { terminate(Status::FAILURE_FENCE_SPACE, false, now); return; }
    transition(Status::STATE_WAIT_ARM_FIRST, now);
    PX4_INFO("[autocal] RTK baseline %.3f m; arm for reciprocal straight runs", static_cast<double>(status_.rtk_baseline_m));
}

void AutoCalibrationMode::run_straight(std::uint64_t now, bool returning) noexcept
{
    const auto &rtk = rtk_sub_.get();
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg, leg_lat_, leg_lon_, north, east);
    const float distance = std::hypot(north, east);
    if (now - state_started_ > 90000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); return; }
    // 后端 Ready 不等于实体电机已接通。保留最长 Arm ramp 后仍无位置响应，
    // 提前结束，不能在卡死/未接电机时持续等待整个九十秒航段。
    if (now - state_started_ > 8000000ULL && ground_speed() < 0.08F) {
        terminate(Status::FAILURE_MOTION_UNAVAILABLE, false, now); return;
    }
    const unsigned level = std::min(2U, static_cast<unsigned>(std::fmax(0.0F, 3.0F * distance / leg_distance_)));
    const float unrestricted = config_.throttle * (0.5F + 0.25F * level);
    const float predicted_speed = ground_speed() + 0.5F * std::hypot(filtered_acceleration_[0], filtered_acceleration_[1]);
    const bool speed_limited = predicted_speed > 0.65F * fence_.speed_limit_m_s && longitudinal_ > 0.0F;
    const float desired = speed_limited ? std::min(unrestricted,
        longitudinal_ * (0.60F * fence_.speed_limit_m_s / predicted_speed)) : unrestricted;
    // 请求每个 20 ms 周期最多增加 0.003（0.15/s）；起步不依赖尚未辨识的
    // maximum_speed，实际车速/转速仍由独立包络限制。
    longitudinal_ += std::clamp(desired - longitudinal_, -0.003F, 0.003F);
    steering_ = std::clamp(0.5F * math::wrap_pi(leg_heading_ - rtk.array_heading_rad), -0.10F, 0.10F);
    const auto &motors = motors_sub_.get();
    const float applied = 0.5F * (motors.control[0] + motors.control[1]);
    const bool settled = !speed_limited && fresh(motors.timestamp, now, 100000ULL) && std::isfinite(applied) &&
        std::fabs(applied - longitudinal_) < 0.015F && std::fabs(longitudinal_ - desired) < 0.001F &&
        std::hypot(filtered_acceleration_[0], filtered_acceleration_[1]) < 0.10F && std::fabs(yaw_rate()) < 0.05F;
    // 危险包络使用未滤波的 3 m/s^2；拟合另需低通加速度 <0.1 且连续稳定
    // 1 s。命令到位不等于车速到稳态，每次换档都重新累计稳态窗口。
    if (!settled || steady_level_ != level) steady_since_ = 0U;
    steady_level_ = level;
    if (settled && steady_since_ == 0U) steady_since_ = now;
    if (new_heading_epoch()) {
        const auto velocity_epoch = static_cast<std::uint64_t>(rtk.velocity_gps_week) * 604800000ULL + rtk.velocity_gps_milliseconds;
        const float speed = ground_speed();
        const bool observable = velocity_epoch > last_fit_velocity_epoch_ && speed >= 0.08F && rtk.speed_accuracy_m_s / speed <= 0.05F &&
            std::fabs(yaw_rate()) <= 0.05F &&
            std::fabs(math::wrap_pi(leg_heading_ - rtk.array_heading_rad)) < 5.0F * kRadians;
        if (observable) {
            // 相邻 Heading 可携带同一 AGRICA；速度与航向各自去重，一份速度
            // 不能给圆均值/速度拟合贡献两次置信度。
            last_fit_velocity_epoch_ = velocity_epoch;
            const float course = std::atan2(rtk.velocity_east_m_s, rtk.velocity_north_m_s);
            const float variance = rtk.heading_accuracy_rad * rtk.heading_accuracy_rad +
                (rtk.speed_accuracy_m_s * rtk.speed_accuracy_m_s) / (speed * speed);
            heading_mean_[returning ? 1U : 0U].add(math::wrap_pi(rtk.array_heading_rad - course), 1.0F / variance);
            // v=k*u 的 u 属于控制器前馈输入；仅当整形后实际输入与它一致且
            // 已稳定时采样。非线性整形/换向/Arm ramp 数据不冒充线性辨识结果。
            if (settled && steady_since_ != 0U && now - steady_since_ >= 1000000ULL)
                speed_fit_.add(longitudinal_, speed, level);
        }
    }
    if (distance < leg_distance_) return;
    longitudinal_ = steering_ = 0.0F;
    if (!returning) {
        turn_heading_ = math::wrap_pi(leg_heading_ + kPi);
        turn_resume_state_ = Status::STATE_STRAIGHT_BACK;
        transition(Status::STATE_TURN_AROUND, now);
    } else {
        if (!finish_rtk()) { terminate(Status::FAILURE_RTK_INCONSISTENT, false, now); return; }
        transition(Status::STATE_STOP_DISARM_FIRST, now);
    }
}

bool AutoCalibrationMode::finish_rtk() noexcept
{
    float outward{}, inward{};
    if (!heading_mean_[0].result(outward) || !heading_mean_[1].result(inward) ||
        std::fabs(math::wrap_pi(outward - inward)) > 3.0F * kRadians) return false;
    // 先分别验证两段，再按方向等权合并，避免慢速一段样本更多就掩盖另一方向。
    float offset = std::atan2(std::sin(outward) + std::sin(inward), std::cos(outward) + std::cos(inward));
    if (offset < 0.0F) offset += 2.0F * kPi;
    status_.rtk_yaw_offset_deg = offset / kRadians;
    status_.maximum_speed_m_s = 0.0F;
    if (!speed_fit_.result(status_.maximum_speed_m_s))
        status_.failure_reason = Status::FAILURE_DYNAMICS_UNOBSERVABLE;
    return true;
}

void AutoCalibrationMode::run_turn(std::uint64_t now, int direction) noexcept
{
    longitudinal_ = 0.0F;
    const unsigned side = direction > 0 ? 0U : 1U;
    if (now - state_started_ > 75000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); return; }
    // 原地转向时 GNSS 天线绕车心运动，地速不为零；只在起转前等待车体停止，
    // 转起来后由总位移/速度包络门控，避免把 lever arm 速度当成直线滑行。
    if (!turn_started_) {
        if (!stopped()) { steering_ = 0.0F; return; }
        // 起转确认只锁定一次，不以 steering 大小推断；高 yaw 增益车辆可能
        // 只需很小输入。换向后的滑行角不计入新方向，也不冒充航向跳变。
        turn_started_ = true;
        turn_integral_ = 0.0F;
        last_turn_heading_ = rtk_sub_.get().array_heading_rad;
    }
    // 尚未知道 yaw 前馈系数，不能把 steering ceiling 当成恒定激励直接输出。
    // 用小积分增益跟踪 0.3 rad/s，d(steering)=0.06*(rate_sp-rate)*dt；
    // ceiling/slew/实际速度和加速度硬门禁仍独立生效。
    const float previous = steering_;
    steering_ = std::clamp(steering_ + std::clamp(0.06F * (direction * 0.3F - yaw_rate()) * 0.02F,
        -0.003F, 0.003F), -config_.steering, config_.steering);
    const auto &motors = motors_sub_.get();
    const float applied = 0.5F * (motors.control[1] - motors.control[0]);
    // 真实电机差分和车体转速须同向；不设固定 0.08 的电机输入下限，
    // 否则高 yaw 增益车辆在安全的 0.3 rad/s 下永远无法进入采样窗口。
    const bool steady_rate = std::fabs(steering_ - previous) <= 0.0002F &&
        yaw_rate() * direction > 0.08F && std::fabs(filtered_angular_acceleration_) < 0.10F &&
        fresh(motors.timestamp, now, 100000ULL) && std::isfinite(applied) &&
        applied * direction > 0.0F && steering_ * direction > 0.0F;
    if (!steady_rate) steady_since_ = 0U;
    else if (steady_since_ == 0U) steady_since_ = now;
    if (new_heading_epoch()) {
        const float heading = rtk_sub_.get().array_heading_rad;
        const float delta = math::wrap_pi(heading - last_turn_heading_);
        last_turn_heading_ = heading;
        // 有符号积分拒绝来回摆动伪造整圈；单次跳变超过 10 度立即失败。
        if (std::fabs(delta) > 10.0F * kRadians) { terminate(Status::FAILURE_RTK_INCONSISTENT, false, now); return; }
        turn_integral_ += delta;
        const float rate = yaw_rate();
        if (rate * direction > 0.08F && steady_rate &&
            steady_since_ != 0U && now - steady_since_ >= 1000000ULL &&
            fresh(motors.timestamp, now, 100000ULL) && std::isfinite(applied) &&
            std::fabs(applied - steering_) <= 0.10F * std::fabs(steering_) && status_.maximum_speed_m_s > 0.0F) {
            // 从 steering=rate*track*correction/(2*maximum_speed) 反解；
            // 整形偏差限于输入的 10%，避免小输入被固定绝对容差掩盖。
            // 左右方向分别统计，最终必须一致，不能用单一系数掩盖机械不对称。
            const float correction = 2.0F * status_.maximum_speed_m_s * steering_ / (config_.track * rate);
            if (std::isfinite(correction) && correction >= 0.01F && correction <= 100.0F)
                yaw_fit_[side].add(correction, 0.0, 0.0);
        }
    }
    // 换向期间实际电机/车体可能仍按旧方向运动；只有真实方向一致且稳态
    // 窗口成立，才给该方向的磁拟合归属样本。
    if (steady_since_ != 0U && now - steady_since_ >= 1000000ULL) collect_mag(now, side);
    else mag_stable_since_ = 0U;
    if (direction * turn_integral_ < 2.0F * kPi) return;
    if (first_circle_at_ == 0U) first_circle_at_ = now;
    // 一圈只是覆盖下限，不是学习时间充分的证明。允许继续有界转动，让每个
    // 方向在连续 10 s 学习窗之后至少取得 5 份 bias；首圈后最多再等 45 s，
    // 且方向状态的 75 s 硬截止优先，不允许慢圈带来无上限运动。
    const bool bias_expected = bootstrap_applied_ || (config_.mag_id > 0 && flags_sub_.get().cs_mag && !flags_sub_.get().cs_mag_field_disturbed);
    if (mag_path_ready(now) && bias_fit_[side].count() < 5U &&
        (bias_expected || bootstrap_fit_[side].count() < 60U) && now - first_circle_at_ < 45000000ULL) return;
    steering_ = 0.0F;
    if (direction > 0) {
        turn_integral_ = 0.0F;
        last_turn_heading_ = rtk_sub_.get().array_heading_rad;
        transition(Status::STATE_TURN_CCW, now);
    } else transition(Status::STATE_STOP_DISARM_SECOND, now);
}

bool AutoCalibrationMode::finish_dynamics() noexcept
{
    if (status_.maximum_speed_m_s <= 0.0F || yaw_fit_[0].count() < 30U || yaw_fit_[1].count() < 30U) return false;
    const double a = yaw_fit_[0].mean().x, b = yaw_fit_[1].mean().x;
    const double mean = 0.5 * (a + b);
    if (mean <= 0.0 || std::fabs(a - b) > 0.20 * mean ||
        std::sqrt(yaw_fit_[0].variance().x) > 0.15 * a || std::sqrt(yaw_fit_[1].variance().x) > 0.15 * b) return false;
    status_.yaw_rate_correction = static_cast<float>(mean);
    return true;
}

void AutoCalibrationMode::step(std::uint64_t now) noexcept
{
    status_.awaiting_arm = false;
    if (step_tuning(now)) return;
    switch (status_.state) {
    case Status::STATE_PREFLIGHT_CHECK:
        if (armed_.armed()) { terminate(Status::FAILURE_ARMED_AT_ENTRY, false, now); break; }
        if (now - state_started_ > 30000000ULL) { terminate(Status::FAILURE_PREFLIGHT, false, now); break; }
        if (!imu_quality(now) || level_sub_.get().active) break;
        imu_device_id_ = imu_sub_.get().accel_device_id;
        if (mag_device_id_ == 0U && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL)) mag_device_id_ = raw_mag_sub_.get().device_id;
        status_.magnetometer_present = status_.magnetometer_present || config_.mag_id != 0 ||
            (mag_device_id_ != 0U && fresh(raw_mag_sub_.get().timestamp, now, 1000000ULL));
        level_request_time_ = now;
        transition(Status::STATE_LEVEL_HOLD, now);
        request(auto_calibration_request_s::REQUEST_LEVEL, level_request_time_);
        break;
    case Status::STATE_LEVEL_HOLD: {
        const auto &level = level_sub_.get();
        if (now - state_started_ > 45000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (level.request_timestamp != level_request_time_) break;
        if (!level.active && level.result == sensor_calibration_status_s::RESULT_SUCCESS) {
            if (level.parameter_start_count != expected_set_count_ || level.parameter_owned_changes > 2U ||
                level.parameter_set_count != level.parameter_start_count + level.parameter_owned_changes) {
                terminate(Status::FAILURE_PARAMETER, false, now); break;
            }
            expected_set_count_ = level.parameter_set_count;
            if (param_set_count() != expected_set_count_) terminate(Status::FAILURE_PARAMETER, false, now);
            else transition(Status::STATE_COMMIT_LEVEL, now);
        }
        else if (!level.active && level.result != sensor_calibration_status_s::RESULT_RUNNING)
            terminate(Status::FAILURE_NOT_STATIONARY, false, now);
        break;
    }
    case Status::STATE_COMMIT_LEVEL:
        if (param_save_default(false) == 0) {
            px4::AtomicTransaction atomic;
            if (param_set_count() != expected_set_count_) { terminate(Status::FAILURE_PARAMETER, false, now); break; }
            if (!read_config()) { terminate(Status::FAILURE_PARAMETER, false, now); break; }
            status_.level_roll_offset_deg = config_.board_offset[0];
            status_.level_pitch_offset_deg = config_.board_offset[1];
            status_.completed_stages |= Status::STAGE_LEVEL;
            if (!status_.fence_center_valid || !motion_configuration_valid()) {
                status_.unavailable_stages |= Status::STAGE_RTK | Status::STAGE_SPEED | Status::STAGE_YAW;
                status_.failure_reason = !status_.fence_center_valid ? Status::FAILURE_ENTRY_POSITION
                    : config_.stop_distance <= 0.0F ? Status::FAILURE_STOP_DISTANCE : Status::FAILURE_MOTION_UNAVAILABLE;
                finish(now);
                break;
            }
            transition(Status::STATE_RTK_BASELINE_COLLECT, now);
        } else if (now - state_started_ > 20000000ULL) terminate(Status::FAILURE_PARAMETER, false, now);
        break;
    case Status::STATE_RTK_BASELINE_COLLECT: collect_baseline(now); break;
    case Status::STATE_WAIT_ARM_FIRST:
    case Status::STATE_WAIT_ARM_SECOND:
        status_.awaiting_arm = imu_quality(now) && rtk_quality(now) && stopped() && fence_result(now).can_stop &&
            (status_.state == Status::STATE_WAIT_ARM_FIRST || (rtk_yaw_fused(now) && (dynamics_pending() || mag_path_ready(now))));
        if (now - state_started_ > 120000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (!armed_.armed()) break;
        if (!status_.awaiting_arm) { terminate(Status::FAILURE_PREFLIGHT, false, now); break; }
        arm_started_ = motion_started_ = now;
        status_.awaiting_arm = false;
        longitudinal_ = steering_ = 0.0F;
        if (status_.state == Status::STATE_WAIT_ARM_FIRST) {
            leg_lat_ = gps_sub_.get().latitude_deg; leg_lon_ = gps_sub_.get().longitude_deg;
            leg_heading_ = rtk_sub_.get().array_heading_rad;
            transition(Status::STATE_STRAIGHT_OUT, now);
        } else {
            turn_integral_ = 0.0F;
            last_turn_heading_ = rtk_sub_.get().array_heading_rad;
            mag_stable_since_ = last_bias_sample_ = last_mag_sample_ = 0U;
            for (unsigned i = 0U; i < 2U; ++i) { bias_fit_[i].reset(); bootstrap_fit_[i].reset(); coverage_[i] = 0U; }
            transition(Status::STATE_TURN_CW, now);
        }
        break;
    case Status::STATE_STRAIGHT_OUT: run_straight(now, false); break;
    case Status::STATE_STRAIGHT_BACK: run_straight(now, true); break;
    case Status::STATE_TURN_AROUND: {
        longitudinal_ = 0.0F;
        if (now - state_started_ > 60000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        const float heading = rtk_sub_.get().array_heading_rad;
        if (!turn_started_) {
            if (!stopped()) { steering_ = 0.0F; break; }
            turn_started_ = true;
            last_turn_heading_ = heading;
            turn_remaining_ = math::wrap_pi(turn_heading_ - heading);
            if (turn_remaining_ < 0.0F) turn_remaining_ += 2.0F * kPi;
        }
        if (new_heading_epoch()) {
            const float delta = math::wrap_pi(heading - last_turn_heading_);
            if (std::fabs(delta) > 10.0F * kRadians) { terminate(Status::FAILURE_RTK_INCONSISTENT, false, now); break; }
            if (turn_remaining_ * (turn_remaining_ - delta) <= 0.0F) turn_braking_ = true;
            turn_remaining_ -= delta;
            last_turn_heading_ = heading;
        }
        // 掉头起转后不再拿天线 lever arm 地速判断滑行。固定先顺时针，
        // 用逐历元增量维护连续剩余角，避免半圈目标在 +/-pi 处因噪声反复换向。
        const float error = turn_remaining_;
        if (std::fabs(error) < 3.0F * kRadians) turn_braking_ = true;
        if (turn_braking_) {
            // 近目标或跨过目标后先请求零输入，由下游保留电机 slew/硬限幅。
            // 纯积分内环在低 yaw authority 下阻尼不足，不能带着残余积分
            // 反复穿越目标。先等真实停车，再以零积分做必要的小角度修正；
            // 所有修正共用原 60 s 截止，不把慢响应变成无界自动重试。
            steering_ = 0.0F;
            if (!stopped()) break;
            if (std::fabs(error) >= 3.0F * kRadians) { turn_braking_ = false; break; }
            leg_lat_ = gps_sub_.get().latitude_deg; leg_lon_ = gps_sub_.get().longitude_deg;
            leg_heading_ = turn_heading_;
            transition(turn_resume_state_, now);
            break;
        }
        const float target_rate = std::clamp(0.5F * error, -0.3F, 0.3F);
        steering_ = std::clamp(steering_ + std::clamp(0.06F * (target_rate - yaw_rate()) * 0.02F,
            -0.003F, 0.003F), -config_.steering, config_.steering);
        break;
    }
    case Status::STATE_STOP_DISARM_FIRST:
    case Status::STATE_STOP_DISARM_SECOND:
        longitudinal_ = steering_ = 0.0F;
        if (now - state_started_ > 15000000ULL) { terminate(Status::FAILURE_TIMEOUT, false, now); break; }
        if (armed_.armed()) { if (stopped()) request(auto_calibration_request_s::REQUEST_STAGE_DISARM, now); break; }
        if (status_.state == Status::STATE_STOP_DISARM_FIRST) {
            if (begin_rtk_transaction(now)) transition(Status::STATE_COMMIT_RTK, now);
            else terminate(Status::FAILURE_PARAMETER, false, now);
        } else if (!bootstrap_applied_ && (status_.completed_stages & Status::STAGE_SPEED) == 0U && finish_dynamics()) {
            if (begin_dynamics_transaction(now)) transition(Status::STATE_COMMIT_DYNAMICS, now);
            else terminate(Status::FAILURE_PARAMETER, false, now);
        } else {
            if ((status_.completed_stages & Status::STAGE_SPEED) == 0U && status_.maximum_speed_m_s > 0.0F)
                status_.failure_reason = Status::FAILURE_MECHANICAL_ASYMMETRY;
            finish_movement(now);
        }
        break;
    case Status::STATE_WAIT_RTK_RELOCK:
        if (rtk_yaw_fused(now)) {
            if (stable_since_ == 0U) stable_since_ = now;
            if (now - stable_since_ > 3000000ULL) {
                // RTK 是磁学习的依赖，动力学不是：速度不可观时无磁设备就部分
                // 结束；确有磁设备则明确以“仅磁校准”目的继续，禁用 yaw 辨识。
                if (status_.maximum_speed_m_s <= 0.0F && !status_.magnetometer_present) {
                    status_.completed_stages |= Status::STAGE_MAG_SKIPPED;
                    start_tuning(now);
                    break;
                }
                if (!dynamics_pending() && !mag_path_ready(now)) {
                    if (bootstrap_applied_) terminate(Status::FAILURE_SENSOR_STALE, false, now);
                    else {
                        if (status_.failure_reason == Status::FAILURE_NONE) status_.failure_reason = Status::FAILURE_SENSOR_STALE;
                        start_tuning(now);
                    }
                    break;
                }
                transition(Status::STATE_WAIT_ARM_SECOND, now);
                if (status_.maximum_speed_m_s <= 0.0F) PX4_WARN("[autocal] speed unobservable; arm for magnetic calibration only");
                else PX4_INFO("[autocal] GNSS yaw fused; arm for clockwise/counter-clockwise turns");
            }
        } else stable_since_ = 0U;
        if (now - state_started_ > 30000000ULL) terminate(Status::FAILURE_ESTIMATOR, false, now);
        break;
    case Status::STATE_TURN_CW: run_turn(now, 1); break;
    case Status::STATE_TURN_CCW: run_turn(now, -1); break;
    case Status::STATE_COMMIT_RTK: case Status::STATE_COMMIT_DYNAMICS:
    case Status::STATE_COMMIT_MAG: case Status::STATE_APPLY_MAG_BOOTSTRAP: case Status::STATE_RESTORE_MAG:
        poll_transaction(now); break;
    default: terminate(Status::FAILURE_PREFLIGHT, false, now); break;
    }
}

} // namespace dima::rover::modes

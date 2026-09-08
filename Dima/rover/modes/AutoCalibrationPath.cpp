#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "geo/geo.h"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {

bool AutoCalibrationMode::prepare_path(std::uint64_t now) noexcept
{
    const auto &c = tuning_config_;
    const auto fence = fence_result(now);
    if (!fence.can_stop || !tuning_estimator_valid(now) || !std::isfinite(c.jerk) || c.jerk <= 0.0F ||
        !std::isfinite(c.acceptance) || c.acceptance < 0.1F ||
        !std::isfinite(c.pursuit.lookahead_gain) || c.pursuit.lookahead_gain < 0.1F || c.pursuit.lookahead_gain > 100.0F ||
        !std::isfinite(c.pursuit.lookahead_min_m) || c.pursuit.lookahead_min_m <= 0.0F ||
        !std::isfinite(c.pursuit.lookahead_max_m) || c.pursuit.lookahead_max_m < c.pursuit.lookahead_min_m ||
        now - session_started_ > 510000000ULL) return false;
    // 在声明运行速度的有效采样区间内，至少一个合法候选必须不被 Lmin/Lmax
    // 完全钳住。三条相同轨迹不能伪造“增益已辨识”。不改前视上下限或半径。
    bool sensitive = false;
    for (unsigned i = 0U; i < 3U; ++i) {
        const float gain = c.pursuit.lookahead_gain * (i == 0U ? 1.0F : i == 1U ? 0.8F : 1.25F);
        sensitive = sensitive || (gain >= 0.1F && gain <= 100.0F &&
            gain * c.speed_limit > c.pursuit.lookahead_min_m * 1.02F &&
            gain * 0.5F * c.speed_limit < c.pursuit.lookahead_max_m * 0.98F);
    }
    path_gain_sensitive_ = sensitive;
    float gps_x{}, gps_y{}, gps_z{}, imu_x{}, imu_y{}, imu_z{};
    {
        px4::AtomicTransaction atomic;
        if (param_get(param_handle(dima::params::EKF2_GPS_POS_X), &gps_x) != 0 ||
            param_get(param_handle(dima::params::EKF2_GPS_POS_Y), &gps_y) != 0 ||
            param_get(param_handle(dima::params::EKF2_GPS_POS_Z), &gps_z) != 0 ||
            param_get(param_handle(dima::params::EKF2_IMU_POS_X), &imu_x) != 0 ||
            param_get(param_handle(dima::params::EKF2_IMU_POS_Y), &imu_y) != 0 ||
            param_get(param_handle(dima::params::EKF2_IMU_POS_Z), &imu_z) != 0) return false;
    }
    const float lever_bound = std::hypot(std::hypot(gps_x, gps_y), gps_z) + std::hypot(std::hypot(imu_x, imu_y), imu_z);
    const float dynamic_distance = c.speed_limit * c.speed_limit * (0.5F / c.acceleration + 0.5F / c.deceleration) +
        2.0F * c.speed_limit * c.deceleration / c.jerk + 2.0F * c.speed_limit;
    const float length = std::max({3.0F, dynamic_distance,
        sensitive ? 4.0F * c.pursuit.lookahead_min_m + 2.0F * c.acceptance : 0.0F});
    if (!std::isfinite(length) || length > config_.distance || !std::isfinite(lever_bound)) return false;
    const auto &position = position_sub_.get();
    MapProjection projection(position.ref_lat, position.ref_lon, position.ref_timestamp);
    float center_x{}, center_y{};
    projection.project(fence_.latitude_deg, fence_.longitude_deg, center_x, center_y);
    const dima::lib::rover::Position2f anchor{position.x - center_x, position.y - center_y};
    // 闭合瘦三角替代四边大圈：返回同一初始位置供候选公平比较，又避免
    // 恰好 180 度的航向符号歧义。长度包含实际加减速及有效速度观察区间。
    // 只在原固定圆内选择有限朝向，不把验证路径的起点当作新围栏圆心。
    for (unsigned orientation = 0U; orientation < 4U; ++orientation) {
        const float offset = orientation == 0U ? 0.0F : orientation == 1U ? 0.5F * kPi
            : orientation == 2U ? -0.5F * kPi : 0.95F * kPi;
        const float angle = body_yaw() + offset;
        const float x = std::cos(angle), y = std::sin(angle);
        for (int side : {-1, 1}) {
            path_points_[0] = anchor;
            path_points_[1] = {anchor.north_m + length * x, anchor.east_m + length * y};
            path_points_[2] = {anchor.north_m + 0.5F * length * x - side * length * y / 6.0F,
                               anchor.east_m + 0.5F * length * y + side * length * x / 6.0F};
            path_points_[3] = anchor;
            path_count_ = 4U;
            bool inside = true;
            for (unsigned i = 0U; i < path_count_; ++i)
                inside = inside && std::hypot(path_points_[i].north_m, path_points_[i].east_m) + lever_bound + 0.5F < fence.working_radius_m;
            if (inside) return true;
        }
    }
    return false;
}

void AutoCalibrationMode::validate_path(std::uint64_t now) noexcept
{
    if (now - state_started_ > 75000000ULL || path_index_ >= path_count_) {
        fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
    }
    if (!tuning_feedback(now)) {
        // 首次闭环要让差速层清旧积分并建立估计器代际；在既有 Arm 投影窗口
        // 内只请求零输入，不能把这一帧正常初始化误判为路径验证失败。
        physical_speed_ = physical_rate_ = 0.0F;
        if (now - arm_started_ > 250000ULL) fail_tuning(Status::FAILURE_SENSOR_STALE, now);
        return;
    }
    const auto &feedback = control_feedback_sub_.get();
    const float previous_speed_request = physical_speed_;
    const float previous_rate_request = physical_rate_;
    const auto fence = fence_result(now);
    const auto &position = position_sub_.get();
    if (!fence.can_stop || feedback.parameter_update_instance != transaction_.generation() ||
        !fresh(position.timestamp_sample, now, 200000ULL) || !position.xy_valid || !position.v_xy_valid) {
        fail_tuning(Status::FAILURE_FENCE_BOUNDARY, now); return;
    }
    // 路径反馈使用与生产 AutoMode 相同的 EKF 位置参考点；围栏仍独立检查
    // 原始 GNSS 定位点。不能把天线转动位移当成可直接迁移到导航的路径误差。
    MapProjection projection(position.ref_lat, position.ref_lon, position.ref_timestamp);
    float center_north{}, center_east{};
    projection.project(fence_.latitude_deg, fence_.longitude_deg, center_north, center_east);
    const dima::lib::rover::Position2f current{position.x - center_north, position.y - center_east};
    if (!path_entry_captured_) { path_entry_ = current; path_entry_captured_ = true; }
    if (path_arrived_) {
        physical_speed_ = physical_rate_ = 0.0F;
        if (path_index_ == 0U) {
            // 所有候选先在不计分的入口阶段对齐同一首边。闭合三角的末边
            // 与首边相差约 162 度，不能只让基线省掉这次转向后比较用时。
            if (std::hypot(feedback.forward_raw_m_s, feedback.lateral_raw_m_s) > tuning_config_.speed_threshold) {
                steady_since_ = 0U;
                return;
            }
            const float heading = std::atan2(path_points_[1].east_m - path_points_[0].east_m,
                path_points_[1].north_m - path_points_[0].north_m);
            const float error = std::fabs(dima::lib::rover::calibration::wrap_pi(heading - body_yaw()));
            const float tolerance = std::max(3.0F * rtk_sub_.get().heading_accuracy_rad,
                1.25F * tuning_config_.rate_threshold / status_.heading_p);
            const auto control = tuning_heading_.update(heading, body_yaw(), 0.02F);
            if (!control.valid) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
            if (error > tolerance) {
                physical_rate_ = control.yaw_rate_setpoint_rad_s;
                steady_since_ = 0U;
                return;
            }
            if (!stopped() || !feedback_unmasked()) { steady_since_ = 0U; return; }
            if (steady_since_ == 0U) steady_since_ = now;
            if (now - steady_since_ < 1000000ULL) return;
        }
        if (!stopped()) return;
        ++path_index_;
        if (path_index_ == 1U) path_measured_started_ = now;
        path_arrived_ = false;
        tuning_heading_.reset();
        tuning_driving_.reset();
        path_leg_started_ = now;
        path_yaw_rate_previous_ = 0.0F;
        if (path_index_ == path_count_) {
            if (path_errors_.count() < 40U) { fail_tuning(Status::FAILURE_PATH_UNOBSERVABLE, now); return; }
            const float score = std::sqrt(static_cast<float>(path_errors_.mean().x));
            if (!std::isfinite(score)) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
            if (path_trial_ < 3U) {
                path_scores_[path_trial_] = score;
                path_duration_s_[path_trial_] = 1.0e-6F * static_cast<float>(now - path_measured_started_);
                path_sample_counts_[path_trial_] = path_errors_.count();
                path_observed_fields_[path_trial_] = (path_observable_samples_ >= 30U ? Status::NAV_FIELD_LOOKAHEAD : 0U) |
                    (path_jerk_samples_ >= 30U ? Status::NAV_FIELD_JERK : 0U) |
                    (path_reduction_samples_ >= 30U ? Status::NAV_FIELD_SPEED_REDUCTION : 0U);
            } else {
                const auto &chosen = path_candidates_[best_path_trial_];
                // 初始化的正 jerk 也属于候选，不能以 RAM baseline 为原值而
                // 漏掉验证。确认圈对事务最初快照的全部变化重新证明生效。
                const auto required = (chosen.gain != transaction_.original_float(dima::params::PP_LOOKAHD_GAIN) ? Status::NAV_FIELD_LOOKAHEAD : 0U) |
                    (chosen.jerk != transaction_.original_float(dima::params::RO_JERK_LIM) ? Status::NAV_FIELD_JERK : 0U) |
                    (chosen.reduction != transaction_.original_float(dima::params::RO_SPEED_RED) ? Status::NAV_FIELD_SPEED_REDUCTION : 0U);
                const auto observed = (path_observable_samples_ >= 30U ? Status::NAV_FIELD_LOOKAHEAD : 0U) |
                    (path_jerk_samples_ >= 30U ? Status::NAV_FIELD_JERK : 0U) |
                    (path_reduction_samples_ >= 30U ? Status::NAV_FIELD_SPEED_REDUCTION : 0U);
                const float duration = 1.0e-6F * static_cast<float>(now - path_measured_started_);
                // 确认圈要求同一组字段重新出现敏感证据，并保持改善与速度覆盖。
                if ((observed & required) != required || duration > 1.25F * path_duration_s_[0] ||
                    path_errors_.count() < 0.8F * path_sample_counts_[0] || (best_path_trial_ != 0U &&
                    path_scores_[0] - score <= std::max(0.10F * path_scores_[0], 3.0F * gps_sub_.get().eph))) {
                    fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return;
                }
                path_observed_fields_[best_path_trial_] = observed;
            }
            status_.validation_error = score;
            validation_passed_ = true;
            end_validation_motion(now);
        }
        return;
    }
    dima::lib::rover::SegmentGuidanceInput input{};
    // 第零段仅移动到固定入口，不计分；所有候选从相同固定路径开始。
    if (path_index_ == 0U) {
        input.start = path_entry_;
    } else input.start = path_points_[path_index_ - 1U];
    input.target = path_points_[path_index_];
    input.position = current;
    input.velocity_north = position.vx;
    input.velocity_east = position.vy;
    input.yaw = body_yaw();
    input.dt_s = 0.02F;
    input.acceptance_radius = path_index_ == 0U ? std::min(tuning_config_.acceptance, 0.05F + 3.0F * gps_sub_.get().eph)
                                              : tuning_config_.acceptance;
    input.cruise_speed = tuning_config_.speed_limit;
    input.arrival_speed = 0.0F;
    input.jerk = tuning_config_.jerk;
    input.deceleration = tuning_config_.deceleration;
    input.maximum_speed = status_.maximum_speed_m_s;
    input.speed_reduction = tuning_config_.speed_reduction;
    input.speed_threshold = tuning_config_.speed_threshold;
    // 固定大小值副本保存更新前 Heading 状态，供同一拍 PP alternate 比较；
    // 不能用更新后的 slew 状态重复推进一步，也不创建第二个运行导航控制器。
    auto alternate_heading = tuning_heading_;
    const auto guidance = dima::lib::rover::update_segment(tuning_pursuit_, tuning_heading_, tuning_driving_, input);
    if (!guidance.valid) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    physical_speed_ = guidance.speed_setpoint;
    physical_rate_ = guidance.yaw_rate_setpoint;
    if (guidance.speed_plan.waypoint_inside_acceptance) {
        path_arrived_ = true;
        physical_speed_ = physical_rate_ = 0.0F;
    }
    if (path_index_ == 0U) return;
    const float error = guidance.pursuit.crosstrack_error_m;
    if (std::fabs(error) > 1.0F) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    if (feedback.saturated) {
        if (stable_since_ == 0U) stable_since_ = now;
        if (now - stable_since_ > 1000000ULL) { fail_tuning(Status::FAILURE_GAIN_VALIDATION, now); return; }
    } else stable_since_ = 0U;
    const auto observation_sample = gps_sub_.get().timestamp_sample;
    // 只用已消费上一拍请求、且真实 PI 设定未再被加减速器覆盖的样本；
    // 新旧请求错拍或下游 limiter 尚未跟上都不能证明导航字段已生效。
    const bool speed_request_applied = feedback.request_sequence == sequence_ &&
        std::fabs(feedback.speed_setpoint_m_s - previous_speed_request) <= 1.0e-4F;
    if (observation_sample > path_observation_sample_ && feedback_unmasked() && !feedback.motor_slew_active && speed_request_applied &&
        guidance.driving.state == dima::lib::rover::DrivingState::Driving) {
        path_observation_sample_ = observation_sample;
        const auto alternate = dima::lib::rover::plan_waypoint_speed(guidance.pursuit.distance_to_waypoint_m,
            input.acceptance_radius, input.cruise_speed, 0.0F, input.cruise_speed, 0.8F * input.jerk, input.deceleration);
        const float alternate_final_speed = alternate.valid ? dima::lib::rover::reduce_speed_for_heading_error(
            alternate.speed_setpoint_m_s, guidance.heading_error, input.maximum_speed, input.speed_reduction) : NAN;
        // heading 减速可能把两个不同的 jerk 规划值压成同一请求。必须比较
        // 完整生产组合后的速度，不能让另一个策略的改善授权一个被遮蔽的 jerk。
        if (alternate.valid && guidance.speed_plan.speed_setpoint_m_s < input.cruise_speed &&
            std::fabs(alternate_final_speed - guidance.speed_setpoint) > 3.0F * response_noise_[0])
            ++path_jerk_samples_;
        const float alternate_reduction = input.speed_reduction >= 0.0F ? 1.25F * input.speed_reduction : path_candidates_[1].reduction;
        const float speed_a = dima::lib::rover::reduce_speed_for_heading_error(guidance.speed_plan.speed_setpoint_m_s,
            guidance.heading_error, input.maximum_speed, input.speed_reduction);
        const float speed_b = dima::lib::rover::reduce_speed_for_heading_error(guidance.speed_plan.speed_setpoint_m_s,
            guidance.heading_error, input.maximum_speed, alternate_reduction);
        if (std::fabs(speed_a - speed_b) > 3.0F * response_noise_[0]) ++path_reduction_samples_;
    }
    if (guidance.driving.state != dima::lib::rover::DrivingState::Driving || !feedback_unmasked() ||
        now - path_leg_started_ < 2000000ULL || feedback.speed_m_s < 0.5F * tuning_config_.speed_limit) return;
    const auto sample = gps_sub_.get().timestamp_sample;
    if (sample <= path_sample_) return;
    path_sample_ = sample;
    path_errors_.add(error * error, 0.0, 0.0);
    const float lookahead = status_.lookahead_gain * std::hypot(position.vx, position.vy);
    const bool rate_request_applied = feedback.request_sequence == sequence_ &&
        std::fabs(feedback.yaw_rate_setpoint_rad_s - previous_rate_request) <= 1.0e-4F;
    if (rate_request_applied && !feedback.motor_slew_active && lookahead > tuning_config_.pursuit.lookahead_min_m * 1.02F &&
        lookahead < tuning_config_.pursuit.lookahead_max_m * 0.98F) {
        dima::lib::rover::PurePursuit alternate;
        if (alternate.configure({std::max(0.1F, 0.8F * status_.lookahead_gain),
                tuning_config_.pursuit.lookahead_min_m, tuning_config_.pursuit.lookahead_max_m})) {
            const auto other = alternate.update(input.start, input.target, current, std::hypot(position.vx, position.vy));
            if (other.valid) {
                const auto other_heading = alternate_heading.update(other.target_bearing_rad, input.yaw, input.dt_s);
                // 不同 PP 方位也可能被 Heading slew/clamp 压成同一 rate；
                // 只有实际生产组合后的请求差异超过噪声，且 rate PI 没有遮蔽，才计数。
                if (other_heading.valid && std::fabs(dima::lib::rover::calibration::wrap_pi(
                        other.target_bearing_rad - guidance.pursuit.target_bearing_rad)) > 3.0F * rtk_sub_.get().heading_accuracy_rad &&
                    std::fabs(other_heading.yaw_rate_setpoint_rad_s - guidance.yaw_rate_setpoint) > 3.0F * response_noise_[1])
                    ++path_observable_samples_;
            }
        }
    }
    // 转弯/到点时正常换向不算振荡；只检查直线稳态段的显著重复纠偏。
    if (std::fabs(physical_rate_) > 0.08F && std::fabs(path_yaw_rate_previous_) > 0.08F &&
        physical_rate_ * path_yaw_rate_previous_ < 0.0F) ++path_crossings_;
    if (std::fabs(physical_rate_) > 0.08F) path_yaw_rate_previous_ = physical_rate_;
    if (path_crossings_ > 12U) fail_tuning(Status::FAILURE_GAIN_VALIDATION, now);
}

} // namespace dima::rover::modes

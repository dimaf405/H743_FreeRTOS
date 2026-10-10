#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "geo/geo.h"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cfloat>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

bool AutoCalibrationMode::prepare_path(std::uint64_t now) noexcept
{
    status_.straight_active = false;
    const auto fence = fence_result(now);
    if (!fence.can_stop || !tuning_estimator_valid(now)) return false;
    if (nav_.side == 0.0F) {
        const float lever = sensor_lever_arm();
        // 沿用原场地/杆臂留距；正方形外接圆决定边长，不扩大围栏。
        // 停车项按试验速度折算（2026-09-30 用户确认）：本函数在车辆静止时
        // 首次计算，live fence 的停车项≈0，而试验以巡航速度运行时安全带随
        // 速度增大约 v²/2a+v·delay；按静止余量布置会把角落的结构余量
        // （杆臂+0.5）吃掉近半。显式按将要行驶的速度取实测制动模型的
        // 停车距离，静止/运动失配消除，满速角落余量恢复为杆臂+0.5。
        const float position_margin = fence_.origin_error_m + gps_sub_.get().eph;
        const float trial_stop = braking_distance(nav_.nominal_speed);
        const float radius = fence_.radius_m - position_margin -
            (std::isfinite(trial_stop) ? trial_stop : 0.0F) - lever - 0.5F;
        nav_.side = std::min(config_.straight_distance, std::sqrt(2.0F) * radius);
        if (!std::isfinite(nav_.side) || nav_.side <= 2.0F * std::min(tuning_config_.acceptance, config_.path_error)) return false;
        const auto &p = position_sub_.get();
        MapProjection projection(p.ref_lat, p.ref_lon, p.ref_timestamp);
        projection.project(fence_.latitude_deg, fence_.longitude_deg, nav_.center.north_m, nav_.center.east_m);
        nav_.heading = body_yaw();
    }
    const float half = 0.5F * nav_.side;
    const bool square = nav_.item == Status::NAV_TUNE_JERK ||
        (nav_.pass == NavigationPass::Final && nav_.direction == 0U);
    if (square) {
        path_points_[0] = {-half, -half}; path_points_[1] = {half, -half};
        path_points_[2] = {half, half}; path_points_[3] = {-half, half}; path_points_[4] = path_points_[0];
        path_count_ = 5U;
    } else if (nav_.item == Status::NAV_TUNE_LOOKAHEAD) {
        path_points_[0] = {-half, 0.0F}; path_points_[1] = {half, 0.0F}; path_count_ = 2U;
    } else {
        const float angle = nav_.pass == NavigationPass::Final && nav_.direction >= 3U
            ? tuning_config_.driving.drive_to_turn_yaw_error_rad : nav_.turn_angle;
        const bool right = nav_.pass == NavigationPass::Final ? nav_.direction % 2U == 0U : nav_.direction != 0U;
        // 局部x向前、y向右（NED正偏航）；航点几何的正转角对应右转。
        const float y = (right ? 1.0F : -1.0F) * half * std::sin(0.5F * angle);
        const float x = half * std::cos(0.5F * angle);
        path_points_[0] = {-x, y}; path_points_[1] = {0.0F, 0.0F}; path_points_[2] = {0.5F * x, 0.5F * y}; path_points_[3] = {x, y}; path_count_ = 4U;
        // 出弯观测点带速通过，停车交给下一条同向航段，避免终点制动伪装成转弯减速。
        // 两个到达区域必须能区分；否则本场地不能提供独立过弯观测。
        if (0.5F * half <= 2.0F * std::min(tuning_config_.acceptance, config_.path_error)) return false;
    }
    const float cosine = std::cos(nav_.heading), sine = std::sin(nav_.heading);
    for (unsigned i = 0U; i < path_count_; ++i) {
        const auto p = path_points_[i];
        path_points_[i] = {nav_.center.north_m + cosine * p.north_m - sine * p.east_m,
            nav_.center.east_m + sine * p.north_m + cosine * p.east_m};
    }
    return true;
}

bool AutoCalibrationMode::reset_navigation_trial(std::uint64_t now) noexcept
{
    if (!prepare_path(now) ||
        !tuning_pursuit_.configure({status_.lookahead_gain, tuning_config_.pursuit.lookahead_min_m,
            tuning_config_.pursuit.lookahead_max_m}) ||
        !tuning_heading_.configure({tuning_config_.heading_p, tuning_config_.rate_limit}) ||
        !tuning_driving_.configure(tuning_config_.driving)) return false;
    tuning_heading_.reset(); tuning_driving_.reset();
    path_index_ = 0U; path_arrived_ = path_entry_captured_ = false;
    nav_.started = now; nav_.sample = nav_.acceleration_epoch = 0U;
    nav_.leg_timeout_s = 0.0F; nav_.previous_setpoint = 0.0F;
    nav_.duration = nav_.error_squared = nav_.speed_squared = nav_.rate_squared = 0.0;
    nav_.jerk_squared = nav_.jerk_duration = nav_.yaw_accel_squared = 0.0;
    nav_.samples = nav_.brake_samples = nav_.braking_segments = 0U; nav_.peak_error = nav_.peak_speed_target = nav_.peak_rate_target = 0.0F;
    nav_.failed = nav_.observable = nav_.unmasked = nav_.incomplete = nav_.arrival_observed = nav_.braking_window = false;
    nav_.turn_window = nav_.turn_completed = nav_.turn_failed = nav_.turn_stop_allowed = nav_.cruise_observed = false;
    nav_.leg_sequence = sequence_ + 2U; // WaitArm本拍仍发布零请求，下拍才执行本腿。
    nav_.turn_duration = nav_.turn_speed_squared = 0.0;
    nav_.measured_arrival = 0.0F; nav_.result = {};
    status_.navigation_xtrack_max_m = 0.0F;
    status_.navigation_speed_rms_m_s = status_.navigation_jerk_rms_m_s3 = NAN;
    return true;
}

void AutoCalibrationMode::observe_navigation(const dima::lib::rover::SegmentGuidanceOutput &g,
    std::uint64_t now) noexcept
{
    (void)now;
    if (path_index_ == 0U) return;
    const auto &p = position_sub_.get();
    const auto &f = control_feedback_sub_.get();
    // 位置误差独立观察，不能因控制反馈重复历元而漏掉新位置上的超差。
    nav_.peak_error = std::max(nav_.peak_error, std::fabs(g.pursuit.crosstrack_error_m));
    status_.navigation_xtrack_max_m = nav_.peak_error;
    if (nav_.peak_error > config_.path_error) {
        nav_.failed = true;
        if (nav_.turn_window) nav_.turn_failed = true;
    }
    const bool driving = g.driving.state == dima::lib::rover::DrivingState::Driving;
    // 不把上一航段停车/转向的反馈归给本腿。使用已有请求序号（含uint32回绕），
    // 不新增时间容差或影子控制器；位置性能仍按当前正式航段检查。
    const bool current_request = static_cast<std::uint32_t>(f.request_sequence - nav_.leg_sequence) < 0x80000000U;
    const bool unmasked = current_request && feedback_unmasked(true) && !f.input_limited && !f.motor_slew_active;
    if (driving && unmasked) nav_.unmasked = true;
    // 实验的独立入口目标是巡航速度，而不是已被当前候选压低的轨迹。
    // 保留内环10%跟踪口径；必须出现真实巡航规划段，不能靠慢行降低jerk代价。
    if (driving && unmasked && g.speed_plan.speed_setpoint_m_s == nav_.trial_speed &&
        std::fabs(g.measured_speed.forward_m_s - nav_.trial_speed) <= 0.10F * nav_.trial_speed)
        nav_.cruise_observed = true;
    // 停车轨迹的上限随剩余距离减小。jerk候选尚未建立巡航就开始压低上限，
    // 本腿不可能再达到独立试验目标，应结束候选，不能长时间慢行后凭低jerk获胜。
    if (nav_.item == Status::NAV_TUNE_JERK && !nav_.cruise_observed &&
        g.speed_plan.speed_setpoint_m_s < nav_.trial_speed)
        nav_.failed = true;
    const bool final_square = nav_.pass == NavigationPass::Final && nav_.direction == 0U;
    const bool new_feedback = current_request && f.timestamp_sample > nav_.sample;
    if (new_feedback && (nav_.cruise_observed || final_square) && nav_.arrival_speed == 0.0F &&
        driving && f.speed_setpoint_m_s < nav_.previous_setpoint)
        nav_.braking_window = true;

    // EKF已提供同一车辆参考点的NED加速度和航向，禁止再对天线速度二次差分。
    // 每个历元先投影a_forward，再求导，保留旋转基向量变化项；不增加私有滤波器。
    if (g.driving.state == dima::lib::rover::DrivingState::SpotTurning) {
        nav_.acceleration_epoch = 0U;
    } else if (p.timestamp_sample > nav_.acceleration_epoch) {
        const float acceleration = p.ax * std::cos(p.heading) + p.ay * std::sin(p.heading);
        if (!std::isfinite(acceleration)) {
            if (nav_.braking_window) nav_.incomplete = true;
            nav_.acceleration_epoch = 0U;
        } else {
            if (nav_.braking_window && nav_.acceleration_epoch != 0U) {
                const auto gap = p.timestamp_sample - nav_.acceleration_epoch;
                // 与已有位置反馈200ms新鲜度合同一致；缺失的一段不能用远端差分补造。
                if (gap > 200000ULL) nav_.incomplete = true;
                else {
                    const float dt = 1.0e-6F * static_cast<float>(gap);
                    const float jerk = (acceleration - nav_.previous_acceleration) / dt;
                    if (std::isfinite(jerk)) {
                        nav_.jerk_squared += jerk * jerk * dt; nav_.jerk_duration += dt;
                        ++nav_.brake_samples;
                    } else nav_.incomplete = true;
                }
            }
            nav_.previous_acceleration = acceleration;
            nav_.acceleration_epoch = p.timestamp_sample;
        }
    }
    if (nav_.turn_window) {
        // 中心到出弯观测点必须持续平移。停车/原地转向不能冒充该速度下的过弯能力。
        if (!nav_.turn_stop_allowed && (!driving ||
            g.speed_setpoint <= tuning_config_.speed_threshold ||
            g.measured_speed.speed_m_s <= tuning_config_.speed_threshold)) {
            nav_.failed = nav_.turn_failed = true;
        }
        if (nav_.speed_probe && g.speed_plan.speed_setpoint_m_s < nav_.trial_speed) nav_.incomplete = true;
    }
    if (!new_feedback) return;
    const auto previous = nav_.sample;
    nav_.sample = f.timestamp_sample;
    nav_.previous_setpoint = f.speed_setpoint_m_s;
    // 近零平移采用生产控制器既有死区/停车语义，不能要求“10%×接近零”精度。
    // 位置上限和完整制动观测仍继续累计；这里只排除刻意停止/旋转时的速度评分。
    const bool translating = std::fabs(f.speed_setpoint_m_s) > tuning_config_.speed_threshold ||
        std::fabs(f.speed_raw_m_s) > tuning_config_.speed_threshold;
    if (!driving || !translating) { nav_.previous_rate = f.yaw_rate_rad_s; return; }
    if (previous != 0U) {
        const float dt = 1.0e-6F * static_cast<float>(f.timestamp_sample - previous);
        const float ev = f.speed_raw_m_s - f.speed_setpoint_m_s;
        const float ew = f.yaw_rate_rad_s - f.yaw_rate_setpoint_rad_s;
        nav_.duration += dt;
        nav_.error_squared += g.pursuit.crosstrack_error_m * g.pursuit.crosstrack_error_m * dt;
        nav_.speed_squared += ev * ev * dt; nav_.rate_squared += ew * ew * dt;
        const float yaw_acceleration = (f.yaw_rate_rad_s - nav_.previous_rate) / dt;
        nav_.yaw_accel_squared += yaw_acceleration * yaw_acceleration * dt;
        if (nav_.turn_window) {
            nav_.turn_duration += dt;
            nav_.turn_speed_squared += ev * ev * dt;
        }
        ++nav_.samples;
    }
    nav_.previous_rate = f.yaw_rate_rad_s;
    nav_.peak_speed_target = std::max(nav_.peak_speed_target, std::fabs(f.speed_setpoint_m_s));
    nav_.peak_rate_target = std::max(nav_.peak_rate_target, std::fabs(f.yaw_rate_setpoint_rad_s));
    // 保护/限斜率短暂介入只使本拍不能证明参数生效，不否决已有完整响应数据。
    if (!unmasked) return;
    if (nav_.item == Status::NAV_TUNE_LOOKAHEAD &&
        g.pursuit.lookahead_distance_m > tuning_config_.pursuit.lookahead_min_m &&
        g.pursuit.lookahead_distance_m < tuning_config_.pursuit.lookahead_max_m && g.pursuit.lookahead_active)
        nav_.observable = true;
    if ((nav_.item == Status::NAV_TUNE_JERK || final_square) && nav_.braking_window &&
        g.speed_setpoint > tuning_config_.speed_threshold)
        nav_.observable = true;
    if (nav_.item == Status::NAV_TUNE_SPEED_REDUCTION && !nav_.speed_probe &&
        nav_.turn_window && g.speed_setpoint < g.speed_plan.speed_setpoint_m_s)
        nav_.observable = true;
}

StepResult AutoCalibrationMode::validation_path(std::uint64_t now) noexcept
{
    const auto finish_trial = [&]() {
        physical_speed_ = physical_rate_ = 0.0F;
        const bool data = nav_.duration > 0.0 && nav_.samples != 0U;
        if (nav_.speed_probe && !nav_.arrival_observed) nav_.incomplete = true;
        if (!nav_.failed && (!data || !nav_.unmasked)) nav_.incomplete = true;
        const float xte = data ? std::sqrt(nav_.error_squared / nav_.duration) : INFINITY;
        const float speed = data ? std::sqrt(nav_.speed_squared / nav_.duration) : INFINITY;
        const float rate = data ? std::sqrt(nav_.rate_squared / nav_.duration) : INFINITY;
        const float jerk = nav_.jerk_duration > 0.0 ? std::sqrt(nav_.jerk_squared / nav_.jerk_duration) : INFINITY;
        status_.navigation_speed_rms_m_s = speed;
        status_.navigation_jerk_rms_m_s3 = std::isfinite(jerk) ? jerk : NAN;
        status_.validation_error = xte; // 与既有路径验证一致，单位始终m。
        // 继续使用内环10%误差口径；近零轴采用该轴既有测量死区，不造新噪声倍数。
        const bool tracking = speed <= 0.10F * nav_.peak_speed_target &&
            rate <= std::max(tuning_config_.rate_threshold, 0.10F * nav_.peak_rate_target);
        const bool final_square = nav_.pass == NavigationPass::Final && nav_.direction == 0U;
        const bool turn = nav_.item == Status::NAV_TUNE_SPEED_REDUCTION && !final_square;
        const bool turn_tracking = !turn || (nav_.turn_completed && nav_.turn_duration > 0.0 &&
            std::sqrt(nav_.turn_speed_squared / nav_.turn_duration) <= 0.10F * nav_.trial_speed);
        if (data && !tracking) nav_.failed = true;
        if (turn && nav_.turn_completed && !turn_tracking) nav_.failed = nav_.turn_failed = true;
        // 入场/出弯停车失败不代表转弯速度过高，不能用于构造速度失败上界。
        if (nav_.speed_probe && nav_.failed && !nav_.turn_failed) nav_.incomplete = true;
        const bool observed = final_square ? nav_.observable && nav_.braking_segments == path_count_ - 1U :
            nav_.speed_probe ? nav_.arrival_observed && nav_.turn_completed :
            nav_.item == Status::NAV_TUNE_JERK ? nav_.observable && nav_.braking_segments == path_count_ - 1U :
            nav_.item == Status::NAV_TUNE_SPEED_REDUCTION ?
                (nav_.observable || nav_.turn_stop_allowed) && nav_.turn_completed :
            nav_.pass == NavigationPass::LowSpeed || nav_.observable;
        nav_.result.valid = data && tracking && turn_tracking && observed && nav_.unmasked && !nav_.failed && !nav_.incomplete;
        // 准入已经确认跟踪合格，jerk项目此后以实际平顺性择优。
        nav_.result.primary = nav_.item == Status::NAV_TUNE_JERK ? jerk : xte;
        nav_.result.secondary = nav_.item == Status::NAV_TUNE_JERK ? speed :
            data ? static_cast<float>(std::sqrt(nav_.yaw_accel_squared / nav_.duration)) : INFINITY;
        nav_.result.valid = nav_.result.valid && std::isfinite(nav_.result.primary) &&
            std::isfinite(nav_.result.secondary);
        validation_passed_ = nav_.result.valid;
        return StepResult::WantStop;
    };
    const auto &f = control_feedback_sub_.get();
    const auto &p = position_sub_.get();
    if (f.parameter_update_instance != transaction_.generation()) return fail_step(Status::FAILURE_PARAMETER);
    if (!fence_result(now).can_stop) return abort_step(Status::FAILURE_FENCE_BOUNDARY);
    if (path_index_ >= path_count_) return finish_trial();
    const dima::lib::rover::Position2f position{p.x, p.y};
    if (!path_entry_captured_) { path_entry_ = position; path_entry_captured_ = true; }
    const bool square = nav_.item == Status::NAV_TUNE_JERK ||
        (nav_.pass == NavigationPass::Final && nav_.direction == 0U);
    const bool last = path_index_ + 1U == path_count_;
    dima::lib::rover::SegmentGuidanceInput input{};
    input.start = path_index_ == 0U ? path_entry_ : path_points_[path_index_ - 1U];
    input.target = path_points_[path_index_]; input.position = position;
    input.velocity_north = p.vx; input.velocity_east = p.vy;
    input.yaw = body_yaw();
    input.dt_s = control_interval_s(now);
    // 停车点到达区域不能比用户路径误差目标更宽，否则切换90度航段就必超差。
    // 仅使用已有航点输入容差，不改写NAV_ACC_RAD或生产控制逻辑。
    input.acceptance_radius = std::min(tuning_config_.acceptance, config_.path_error);
    input.cruise_speed = nav_.trial_speed;
    input.arrival_speed = dima::lib::rover::segment_arrival_speed(input.start, input.target,
        last ? input.target : path_points_[path_index_ + 1U], last || square || path_index_ == 0U,
        nav_.trial_speed, status_.maximum_speed_m_s,
        tuning_config_.driving.drive_to_turn_yaw_error_rad, tuning_config_.speed_reduction);
    nav_.arrival_speed = input.arrival_speed;
    if (nav_.leg_timeout_s == 0.0F) {
        // 原75秒只作基础截止；合法低速/长段至少容纳行驶、建速、转向和jerk过渡。
        // 超时是实验未完成，不能用它给转弯速度建立物理失败上界。
        const float length = std::hypot(input.target.north_m - position.north_m, input.target.east_m - position.east_m);
        nav_.leg_timeout_s = std::max(75.0F, length / nav_.trial_speed + nav_.trial_speed / tuning_config_.acceleration +
            nav_.trial_speed / tuning_config_.deceleration + 2.0F * tuning_config_.deceleration / tuning_config_.jerk +
            kPi / tuning_config_.rate_limit + std::log(kPi / tuning_config_.driving.turn_to_drive_yaw_error_rad) / tuning_config_.heading_p);
    }
    if (!std::isfinite(nav_.leg_timeout_s) || 1.0e-6 * static_cast<double>(now - nav_.started) > nav_.leg_timeout_s) {
        nav_.incomplete = true; return finish_trial();
    }
    input.jerk = tuning_config_.jerk; input.deceleration = tuning_config_.deceleration;
    input.maximum_speed = status_.maximum_speed_m_s;
    input.speed_reduction = tuning_config_.speed_reduction; input.speed_threshold = tuning_config_.speed_threshold;
    const auto guidance = dima::lib::rover::update_segment(tuning_pursuit_, tuning_heading_, tuning_driving_, input);
    if (!guidance.valid) return fail_step(Status::FAILURE_GAIN_VALIDATION);
    physical_speed_ = guidance.speed_setpoint; physical_rate_ = guidance.yaw_rate_setpoint;
    observe_navigation(guidance, now);
    if (nav_.failed || nav_.incomplete) return finish_trial();
    const auto progress = dima::lib::rover::update_waypoint_progress(
        guidance.speed_plan.waypoint_inside_acceptance, last, input.arrival_speed,
        guidance.measured_speed.speed_m_s, tuning_config_.speed_threshold, path_arrived_);
    if (progress.hold) physical_speed_ = physical_rate_ = 0.0F;
    if (!progress.ready) return StepResult::Busy;
    if (square && path_index_ != 0U) {
        // 独立jerk整定必须达到名义巡航；最终组合允许短航段按生产规划降速，
        // 但每条边仍必须实际减速并取得加速度观测，不能仅凭任意一拍未限幅通过。
        if (nav_.item == Status::NAV_TUNE_JERK && !nav_.cruise_observed) { nav_.failed = true; return finish_trial(); }
        if (nav_.braking_window && nav_.brake_samples != 0U) ++nav_.braking_segments;
        else nav_.incomplete = true;
    }
    if (path_index_ == 1U && !square && nav_.item == Status::NAV_TUNE_SPEED_REDUCTION) {
        nav_.measured_arrival = guidance.measured_speed.forward_m_s;
        nav_.arrival_observed = input.arrival_speed > tuning_config_.speed_threshold &&
            nav_.measured_arrival > tuning_config_.speed_threshold && feedback_unmasked(true) && !f.input_limited && !f.motor_slew_active &&
            guidance.driving.state == dima::lib::rover::DrivingState::Driving;
        // 独立测速必须先达到候选巡航，短入场加速不能用来反解减速系数。
        if (nav_.speed_probe && !nav_.cruise_observed) nav_.incomplete = true;
        // 大角度最终复核允许公共导航根据实际航向误差停车/原转。
        // 几何转角恰在阈值上时，位置偏差会改变PurePursuit误差，不能要求两者
        // 作相同的浮点分类；中角度能力试验仍必须连续通过。
        nav_.turn_stop_allowed = nav_.pass == NavigationPass::Final && nav_.direction >= 3U;
        if (!nav_.arrival_observed && !nav_.turn_stop_allowed) nav_.incomplete = true;
        nav_.turn_window = nav_.arrival_observed || nav_.turn_stop_allowed;
    }
    if (path_index_ == 2U && nav_.turn_window) {
        nav_.turn_completed = true; nav_.turn_window = false;
    }
    ++path_index_; path_arrived_ = false; nav_.started = now; nav_.leg_timeout_s = 0.0F;
    nav_.braking_window = nav_.cruise_observed = false; nav_.acceleration_epoch = 0U; nav_.brake_samples = 0U;
    nav_.previous_setpoint = 0.0F; nav_.sample = 0U;
    nav_.leg_sequence = sequence_ + 2U; // 本拍仍发送完成点的旧请求，下一拍才是新腿。
    if (nav_.incomplete) return finish_trial();
    if (progress.hold) { tuning_heading_.reset(); tuning_driving_.reset(); }
    // 带速通过时保留控制器连续状态，与生产AutoMode一致，不在转角清积分。
    return path_index_ == path_count_ ? finish_trial() : StepResult::Busy;
}

} // namespace dima::rover::modes

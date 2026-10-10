#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

// —— Return 阶段 helper ————————————————————————————————————————————
// 返程按入场 GNSS 点（fence 圆心）返回；STRAIGHT的有效直行段保留标定采样。阶段文件只
// 返回 StepResult：调度器经 WantReturn/Return 子状态进入，Advance 后按
// session_.resume_substate 恢复原流程；超时/传感器失败经abort_step上抛。

float AutoCalibrationMode::distance_to_start() const noexcept
{
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg,
        fence_.latitude_deg, fence_.longitude_deg, north, east);
    return std::hypot(north, east);
}

float AutoCalibrationMode::heading_to_start() const noexcept
{
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg,
        fence_.latitude_deg, fence_.longitude_deg, north, east);
    // RTK 提交前使用去程圆均值的安装偏置，提交后使用接收机已确认的偏置。
    // 因此目标始终是车头朝向入场点，不能把天线阵列 heading 当作车头 heading。
    const float offset = (status_.provisional_validated_stages & Status::STAGE_RTK) != 0U
        ? rtk_sub_.get().configured_yaw_offset_rad : return_yaw_offset_;
    return math::wrap_pi(std::atan2(-east, -north) + offset);
}

float AutoCalibrationMode::straight_speed_limit(float remaining) const noexcept
{
    // 按本轮实测减速度与剩余距离反解名义速度 v=sqrt(2as)，与停车模型一致。
    // 无观测不再用旧参数或0.3虚构物理模型，调用者必须明确退出。
    const float a = braking_deceleration();
    if (!std::isfinite(a) || a <= 0.0F || !std::isfinite(remaining)) return NAN;
    return std::min(fence_.speed_limit_m_s,
        std::sqrt(2.0F * a * std::max(0.0F, remaining)));
}

void AutoCalibrationMode::return_prepare(std::uint64_t now) noexcept
{
    // 由统一WantReturn出口执行：保存当前闭环标志并清零全部
    // 运动请求；只在新返场入口冻结方向和截止时间，制动恢复不重新初始化。故障/取消不在传感器
    // 或安全状态失效后强行返航。
    return_closed_loop_ = status_.closed_loop;
    // 基础返程明确使用冻结的满输出 E，不再依赖去程的起步输入缓存。
    // PROFILE 初探返场保留自己的可行驶输入；停车时机仍由剩余距离决定。
    return_open_loop_input_ = std::clamp(status_.state == Status::STATE_PROFILE &&
        profile_motion_ == 0U && profile_input_floor_ > 0.0F
            ? profile_input_floor_ : config_.motor_maximum, 0.0F, config_.motor_maximum);
    return_motion_ = ReturnMotion::Align;
    return_started_ = now;
    float north{}, east{};
    math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg,
        fence_.latitude_deg, fence_.longitude_deg, north, east);
    // 固定起点→返程出发点的方向，用于识别越过起点；停车/重对准不得旋转此边界。
    return_axis_rad_ = std::atan2(east, north);
    stop_turn_rate_request();
    status_.closed_loop = false;
    longitudinal_ = steering_ = physical_speed_ = physical_rate_ = 0.0F;
}

StepResult AutoCalibrationMode::return_step(std::uint64_t now) noexcept
{
    // 普通返程、方向纠正和到点停车共用本入口；第二次制动/事务暂停后继续
    // 原截止时间，不能靠重对准或BrakingResume无限延长返程。
    if (return_started_ == 0U || now < return_started_ || now - return_started_ > 90000000ULL)
        return abort_step(Status::FAILURE_TIMEOUT);
    physical_speed_ = physical_rate_ = 0.0F;
    status_.braking_full_output = false;
    const float distance = distance_to_start();
    if (!std::isfinite(distance)) return abort_step(Status::FAILURE_SENSOR_STALE);
    if (distance <= 0.5F) return_motion_ = ReturnMotion::Arrived;
    if (return_motion_ == ReturnMotion::Brake || return_motion_ == ReturnMotion::Arrived) {
        // 一旦决定停车就保持有效零请求，公共执行层负责制动力。距离/速度更新
        // 不能提前解除停车；命中过到达区后又滑出，必须失败而不是再次加油追赶。
        longitudinal_ = steering_ = 0.0F;
        stop_turn_rate_request();
        steady_since_ = 0U;
        if (!stopped() || !braking_output_zero(now)) return StepResult::Busy;
        if (return_motion_ == ReturnMotion::Arrived) {
            // 命中内圈后保持停车；停稳复核复用围栏的位置误差预算，毫米级
            // GNSS抖动不等同于再次驶出。真正超出容差仍失败，不能重新加油追赶。
            if (distance > 0.5F + fence_.origin_error_m + gps_sub_.get().eph) {
                capture_motion_failure("arrival", now);
                return abort_step(Status::FAILURE_MOTION_ENVELOPE);
            }
            status_.closed_loop = return_closed_loop_;
            exercise_heading_ = body_yaw();
            leg_heading_ = rtk_sub_.get().array_heading_rad;
            PX4_INFO("[autocal] returned to entry point: %.2f m", static_cast<double>(distance));
            return StepResult::Advance;
        }
        return_motion_ = ReturnMotion::Align;
        turn_started_ = false;
        return StepResult::Busy;
    }
    const float error = math::wrap_pi(heading_to_start() - rtk_sub_.get().array_heading_rad);
    if (!std::isfinite(error)) return abort_step(Status::FAILURE_SENSOR_STALE);
    if (return_motion_ == ReturnMotion::Align) {
        longitudinal_ = 0.0F;
        steady_since_ = 0U;
        // 起转前等待实际停车；旋转后地速可包含天线杆臂运动，不反复用它切断转向。
        if (!turn_started_ && !stopped()) { steering_ = 0.0F; return StepResult::Busy; }
        if (!turn_started_) stop_turn_rate_request();
        turn_started_ = true;
        if (std::fabs(error) < 3.0F * kRadians) {
            steering_ = 0.0F;
            stop_turn_rate_request();
            if (stopped() && braking_output_zero(now)) {
                return_motion_ = ReturnMotion::Drive;
                leg_heading_ = rtk_sub_.get().array_heading_rad;
            }
            return StepResult::Busy;
        }
        return request_turn_rate(now, std::clamp(0.5F * error, -0.3F, 0.3F));
    }
    if (std::fabs(error) > 20.0F * kRadians) {
        // 偏离后先完成停车再重新对准，禁止带着余速转弯或继续直推。
        return_motion_ = ReturnMotion::Brake;
        longitudinal_ = steering_ = 0.0F;
        steady_since_ = 0U;
        return StepResult::Busy;
    }
    steering_ = 0.0F;
    status_.closed_loop = return_closed_loop_;
    const float speed = ground_speed();
    const float stop = braking_distance(speed);
    const auto &gps = gps_sub_.get();
    const float age = fresh(gps.timestamp_sample, now, 300000ULL)
        ? 1.0e-6F * static_cast<float>(now - gps.timestamp_sample) : NAN;
    // 复用围栏的位置误差与0.42秒链路/请求预算，但不重复加入0.5米到达半径。
    // s_stop=v²/(2a)是本轮模型的名义距离；这部分延迟距离不混入制动观测。
    const float margin = fence_.origin_error_m + gps.eph + speed * (0.42F + age);
    const float remaining = distance - 0.5F;
    if (!std::isfinite(stop) || !std::isfinite(margin))
        return abort_step(Status::FAILURE_MOTION_UNAVAILABLE);
    // 只有RTK提交前的基础返程保留第二次制动及航向采样。独立速度实验的
    // 返场只负责回到起点，既不触发制动实验，也不向速度平台补入运输段样本。
    const bool calibration_leg = status_.state == Status::STATE_STRAIGHT &&
        (status_.provisional_validated_stages & Status::STAGE_RTK) == 0U;
    const bool full_trial = calibration_leg &&
        (status_.unavailable_stages & Status::STAGE_DECELERATION) == 0U && status_.braking_observations == 1U;
    // 第二次正式观测也服从目标侧停车空间，不能为凑足三秒满输出越过起点。
    if (full_trial && remaining <= stop + margin)
        return abort_step(Status::FAILURE_FENCE_SPACE);
    if (speed >= kStoppedSpeedMps && remaining <= stop + margin) {
        return_motion_ = ReturnMotion::Brake;
        longitudinal_ = 0.0F;
        steady_since_ = 0U;
        return StepResult::Busy;
    }
    if (return_closed_loop_) {
        longitudinal_ = 0.0F;
        physical_speed_ = straight_speed_limit(remaining);
        if (!std::isfinite(physical_speed_)) return abort_step(Status::FAILURE_MOTION_UNAVAILABLE);
        return StepResult::Busy;
    }
    if (!std::isfinite(return_open_loop_input_) || return_open_loop_input_ <= 0.0F)
        return abort_step(Status::FAILURE_MOTION_UNAVAILABLE);
    status_.braking_full_output = full_trial;
    const float desired = full_trial ? config_.motor_maximum : return_open_loop_input_;
    // 第二次满输出沿用直线实验0.15/s请求斜率；基础普通返程使用冻结满输出E。
    if (full_trial) {
        const float dt = std::min(0.1F, control_interval_s(now));
        longitudinal_ += std::clamp(desired - longitudinal_, -0.15F * dt, 0.15F * dt);
    } else if (desired < config_.motor_maximum) {
        // PROFILE 初探返场的地板输入只承担弱动力起步确认；行驶建立后升档到
        // 满输出。恒定地板速度（本车0.12m/s）走不完15m腿，90s返程截止内必然
        // TIMEOUT（2026-09-30 实车：爬行80s后离起点7m处超时）。停车时机仍由
        // 上面的剩余距离/制动模型判定，升档不改变到达语义。
        const float dt = std::min(0.1F, control_interval_s(now));
        const float target = speed >= kStoppedSpeedMps ? config_.motor_maximum : desired;
        longitudinal_ += std::clamp(target - longitudinal_, -0.15F * dt, 0.15F * dt);
    } else longitudinal_ = desired;
    if (!calibration_leg) return StepResult::Busy;
    return observe_straight(now, full_trial);
}

} // namespace dima::rover::modes

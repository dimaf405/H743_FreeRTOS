#define MODULE_NAME "auto_cal"
#include "AutoCalibrationMode.hpp"
#include "control/RoverDifferential.hpp"
#include "rover/RoverControl.hpp"
#include "logging/logging.hpp"

#include <algorithm>
#include <cmath>

namespace dima::rover::modes {
namespace math = dima::lib::rover::calibration;

// —— BASELINE/STRAIGHT/TURN 阶段 helper ——————————————————————————————
// 本文件只向调度器返回结果（StepResult），不调用 transition()/terminate()，
// 不用顶层阶段判断事务身份；失败经fail_step/abort_step登记原因并返回。
// 直线族的实际输出包络与无运动检测由调度器统一执行，这里产出实验目标与采样。

StepResult AutoCalibrationMode::baseline_collect(std::uint64_t now) noexcept
{
    // RTK 基线静态采样 + 直线腿空间准备；Advance 后由调度器进入统一 Arm gate。
    if (now - session_.substate_started > 60000000ULL) {
        return abort_step(Status::FAILURE_TIMEOUT);
    }
    if (!rtk_quality(now) || !stopped()) { baseline_count_ = 0U; return StepResult::Busy; }
    if (!new_heading_epoch()) return StepResult::Busy;
    baseline_samples_[baseline_count_++] = rtk_sub_.get().baseline_m;
    status_.progress = static_cast<std::uint8_t>(10U + baseline_count_ / 10U);
    if (baseline_count_ < 100U) return StepResult::Busy;
    if (!math::baseline(baseline_samples_, baseline_count_, status_.rtk_baseline_m)) {
        return abort_step(Status::FAILURE_RTK_INCONSISTENT);
    }
    if (!prepare_straight(now)) {
        return abort_step(Status::FAILURE_FENCE_SPACE);
    }
    PX4_INFO("[autocal] RTK baseline %.3f m; inline braking observations from straight platforms",
        static_cast<double>(status_.rtk_baseline_m));
    return StepResult::Advance;
}

StepResult AutoCalibrationMode::straight_leg(std::uint64_t now) noexcept
{
    // 本函数只产生去程实验输入；返程运动统一由return_step管理，采样共用observe_straight。
    const float distance = distance_to_start();
    const float remaining = leg_distance_ - distance;
    if (now - session_.substate_started > 90000000ULL) {
        return abort_step(Status::FAILURE_TIMEOUT);
    }
    // 去程/返程各从确认的满输出开始制动，初速使用切换时的实测值。
    // 满输出持续时间不等于车速已经稳态；观测结果为平均减速度，不是速度上限。
    const bool full_output_trial = (status_.unavailable_stages & Status::STAGE_DECELERATION) == 0U &&
        status_.braking_observations == 0U;
    status_.braking_full_output = full_output_trial;
    // 未观测时只保留实际定位/链路几何余量，不假设0.3 m/s²或另加3米。
    // 第二轮复用上一轮测得的减速度估算本次速度下的停车留距。
    const float trial_stop = status_.braking_model_generation != 0U ? braking_distance(ground_speed()) : 0.0F;
    const float trial_margin = fence_result(now).margin_m;
    if (full_output_trial && (!std::isfinite(trial_margin) || remaining < trial_margin + trial_stop)) {
        px4_log_raw(_PX4_LOG_LEVEL_WARN,
            "[autocal] full-output braking space insufficient: remaining=%.2fm speed=%.2fm/s\n",
            static_cast<double>(remaining), static_cast<double>(ground_speed()));
        return abort_step(Status::FAILURE_FENCE_SPACE);
    }
    const bool moving = ground_speed() >= kStoppedSpeedMps;
    // 基础去程只服务制动和RTK，不再用里程伪造速度平台。普通行驶复用本轮
    // 已观测起步输入，巡航速度上限仍由公共执行层管理。
    const float distance_target = 0.5F * config_.motor_maximum;
    if (moving && steady_since_ == 0U)
        straight_start_floor_ = std::max(straight_start_floor_, longitudinal_);
    const float dt = std::min(0.1F, control_interval_s(now));
    const float stage_target = moving
        ? std::max(distance_target, straight_start_floor_)
        : std::min(config_.motor_maximum, std::max(distance_target, longitudinal_ + 0.003F));
    // 此阶段仅给既有开环激励，巡航减速交公共执行层，不私自预测车速或调油门。
    const float desired = full_output_trial ? config_.motor_maximum : stage_target;
    longitudinal_ += std::clamp(desired - longitudinal_, -0.15F * dt, 0.15F * dt);
    steering_ = 0.0F;
    const StepResult observed = observe_straight(now, full_output_trial);
    if (observed != StepResult::Busy) return observed;
    if (distance < leg_distance_ - 0.3F) return StepResult::Busy;
    longitudinal_ = steering_ = 0.0F;
    // 去程均值确定RTK安装偏置，掉头目标沿用既有执行层航向机动。
    if (!heading_mean_[0].result(return_yaw_offset_))
        return abort_step(Status::FAILURE_RTK_INCONSISTENT);
    turn_heading_ = heading_to_start();
    return StepResult::Advance;
}

StepResult AutoCalibrationMode::observe_straight(std::uint64_t now, bool full_output_trial) noexcept
{
    // 基础去/返程只采RTK、磁-油门与正式制动证据；速度拟合独立运行。
    // 调用者已完成方向/空间/停车判定，转向和停车窗口不进入本采样入口。
    const auto &rtk = rtk_sub_.get();
    // 高输出直线段同步采集磁-油门回归样本（内部自带门槛与去重）。
    sample_mag_throttle(now);
    steering_ = 0.0F;
    // 模式消费执行层发布的纵向轴，不自行解释电机槽位或左右轮混控。
    const auto &feedback = control_feedback_sub_.get();
    // 满输出以电机后端确认的左右轮端为准，沿用既有0.5%输出到顶判据。
    // timestamp_output必须持续新鲜，不能用未执行的请求或旧输出累计3秒。
    const auto &output = output_sub_.get();
    const bool full_output = fresh(output.timestamp_output, now, 100000ULL) && output.backend_ready &&
        !output.parameter_update_pending && output.state == actuator_output_status_s::STATE_ACTIVE &&
        output.applied_right >= 0.995F * config_.motor_maximum &&
        output.applied_left >= 0.995F * config_.motor_maximum;
    // 制动只按后端满输出持续时间触发，与后续独立速度平台的采样窗口无关。
    if (!full_output_trial || !full_output) steady_since_ = 0U;
    else if (steady_since_ == 0U) steady_since_ = now;
    if (full_output_trial && steady_since_ != 0U && now - steady_since_ > 3000000ULL)
        return begin_braking_observation(now) ? StepResult::WantBrake : abort_step(Status::FAILURE_DECELERATION);
    if (new_heading_epoch()) {
        const auto velocity_epoch = velocity_epoch_us();
        const float speed = ground_speed();
        // 重对准后的首拍可能仍带着转向反馈；只有本会话已执行双轮前进、
        // 转向轴归零后才接纳行驶方向，避免天线杆臂速度混入返程安装偏置。
        // 此处使用原始RTK，不能要求feedback.valid中尚未标定的EKF测速条件。
        const bool observable = rtk_quality(now) &&
            velocity_epoch > last_fit_velocity_epoch_ && speed >= kStoppedSpeedMps &&
            feedback.source == rover_motion_request_s::SOURCE_CALIBRATION &&
            feedback.session_id == status_.session_id && fresh(feedback.timestamp, now, 100000ULL) &&
            std::isfinite(feedback.steering) && std::fabs(feedback.steering) < 1.0e-4F &&
            fresh(output.timestamp_output, now, 100000ULL) && output.state == actuator_output_status_s::STATE_ACTIVE &&
            output.applied_right > 0.0F && output.applied_left > 0.0F;
        if (observable) {
            // 相邻 Heading 可携带同一 AGRICA；速度与航向各自去重，一份速度
            // 不能给圆均值/速度拟合贡献两次置信度。
            last_fit_velocity_epoch_ = velocity_epoch;
            if (!session_.straight_outward) {
                // 去程已确定阵列相对车头的偏置；返程把实测速度投到车头方向。
                // 制动切换后轮端正输出不代表此速度历元已前行；反向航迹会使偏置多出pi。
                const float body_heading = rtk.array_heading_rad - return_yaw_offset_;
                const float forward = rtk.velocity_north_m_s * std::cos(body_heading) +
                    rtk.velocity_east_m_s * std::sin(body_heading);
                if (!(forward > 0.0F)) return StepResult::Busy;
            }
            const float course = std::atan2(rtk.velocity_east_m_s, rtk.velocity_north_m_s);
            // 最小化Σ||R(offset)*v-|v|*[cos(heading),sin(heading)]||²，等价于圆均值权重|v|²。
            // 在速度向量域拟合，避免atan2把低速误差放大后仍与高速样本等权；不新增速度/噪声门限。
            heading_mean_[session_.straight_outward ? 0U : 1U].add(math::wrap_pi(rtk.array_heading_rad - course), speed * speed);
        }
    }
    return StepResult::Busy;
}

StepResult AutoCalibrationMode::speed_model_leg(std::uint64_t now) noexcept
{
    // 独立实验只辨识一个可运行工作点，删除固定三档、静态SpeedFit和跨返场拼样本。
    // 既有一阶动态辨识器处理加速过程；保持输入不再被当作速度已稳态。
    const auto fence = fence_result(now);
    if (!std::isfinite(fence.margin_m) || leg_distance_ - distance_to_start() <= fence.margin_m)
        return abort_step(Status::FAILURE_FENCE_SPACE);
    status_.closed_loop = status_.braking_full_output = false;
    steering_ = physical_speed_ = physical_rate_ = 0.0F;
    const auto &f = control_feedback_sub_.get();
    if (!fresh(f.timestamp, now, 100000ULL) || f.source != rover_motion_request_s::SOURCE_CALIBRATION ||
        f.session_id != status_.session_id || !std::isfinite(f.longitudinal) || !std::isfinite(f.applied_longitudinal)) {
        longitudinal_ = 0.0F;
        return now - arm_started_ <= 250000ULL ? StepResult::Busy : abort_step(Status::FAILURE_SENSOR_STALE);
    }
    // 限速/换向/安全限制破坏本次开环输入—响应关系，立即停止，不在空档继续凑计数。
    // Arm ramp和motor slew已体现在对齐的轮端输入中，属于被辨识输入，不能当成故障。
    if (f.braking_speed_limited || f.mixing_limited || f.reversal_held || f.safety_output_limited ||
        f.safety_slew_active || f.saturated || std::fabs(f.steering) > 1.0e-4F)
        return abort_step(Status::FAILURE_DYNAMICS_UNOBSERVABLE);
    const auto &rtk = rtk_sub_.get();
    const auto epoch = velocity_epoch_us();
    const auto heading_epoch = (static_cast<std::uint64_t>(rtk.gps_week) * 604800000ULL + rtk.gps_milliseconds) * 1000ULL;
    const auto sample_time = static_cast<std::int64_t>(rtk.timestamp_sample) +
        static_cast<std::int64_t>(epoch) - static_cast<std::int64_t>(heading_epoch);
    float applied{};
    if (!rtk_quality(now) || sample_time <= 0) return abort_step(Status::FAILURE_SENSOR_STALE);
    // AGRICA可能先于同历元Heading到达；等待时间映射落入已发生的输出历史，
    // 不把正常的串行先后顺序当作故障，也不拿未来时间查询执行证据。
    if (sample_time > static_cast<std::int64_t>(now)) return StepResult::Busy;
    if (now - static_cast<std::uint64_t>(sample_time) > 300000ULL ||
        !motor_history_.forward_output(static_cast<std::uint64_t>(sample_time), applied))
        return abort_step(Status::FAILURE_SENSOR_STALE);
    // 与速度控制器共用带前向符号的二维速度，后溜不能被当作正向起步/响应。
    const auto measured = dima::lib::rover::measure_body_speed(rtk.velocity_north_m_s,
        rtk.velocity_east_m_s, rtk.array_heading_rad - rtk.configured_yaw_offset_rad, 0.0F);
    if (!measured.valid) return abort_step(Status::FAILURE_SENSOR_STALE);
    const float speed = measured.speed_m_s;
    // 入口停稳门（2026-09-30 实车）：前一对准旋转残余的天线杆臂摆速不是
    // 平移响应；未衰减完就把它记为输出原点，一阶拟合会在"输入不变、输出
    // 回落"段选非正增益，candidate<=0 整场弃测。原地等摆速落到停止边界，
    // 10 秒仍不止说明该工作点无法建立静息基准，不再带污染采样。
    if (!exercise_running_ && speed >= kStoppedSpeedMps)
        return now - arm_started_ <= 10000000ULL ? StepResult::Busy
            : abort_step(Status::FAILURE_DYNAMICS_UNOBSERVABLE);
    math::IdentificationConfig configuration{};
    configuration.sample_period_tolerance_s = 0.04F;
    if (!exercise_running_) {
        // 复用后续IDENTIFICATION的工作区；只在本次独立实验入口配置一次。
        // 输入使用同速度历元对齐的已执行轮端轴，MIN/EXPO不再冒充车体增益。
        if (!identifier_.configure(configuration)) return abort_step(Status::FAILURE_DYNAMICS_UNOBSERVABLE);
        identification_input_origin_ = applied;
        identification_output_origin_ = speed;
        last_fit_velocity_epoch_ = 0U;
        exercise_running_ = true;
        status_.excitation_phase = Status::EXCITATION_PROBE;
    }
    if (status_.excitation_phase == Status::EXCITATION_PROBE) {
        // 制动初速仅给出工作点提示，不是稳态最高速度证明。实际未起步则继续缓升，
        // 到顶仍不动由原8秒无运动出口处置；取消50%低档原地带载等180秒的路径。
        const float hint = config_.motor_maximum * std::min(1.0F,
            0.5F * fence_.speed_limit_m_s / status_.braking_full_speed_m_s);
        const bool moving = speed >= kStoppedSpeedMps;
        if ((moving && (longitudinal_ >= hint || speed >= 0.5F * fence_.speed_limit_m_s)) ||
            longitudinal_ >= config_.motor_maximum) {
            status_.excitation_phase = Status::EXCITATION_COLLECT;
            steady_since_ = now; // 仅作冻结输入的历元边界，不作为速度已稳态的计时器。
        } else longitudinal_ = std::min(config_.motor_maximum,
            longitudinal_ + 0.15F * std::min(0.1F, control_interval_s(now)));
    }
    status_.excitation_target = longitudinal_;
    status_.excitation_remaining_s = std::max(0.0F, 180.0F - 1.0e-6F * static_cast<float>(now - exercise_started_));
    if (epoch == last_fit_velocity_epoch_) return StepResult::Busy;
    if (epoch < last_fit_velocity_epoch_) return abort_step(Status::FAILURE_SENSOR_STALE);
    last_fit_velocity_epoch_ = epoch;
    if (!identifier_.add_sample(epoch, applied - identification_input_origin_, speed - identification_output_origin_))
        return abort_step(Status::FAILURE_DYNAMICS_UNOBSERVABLE);
    status_.excitation_samples = identifier_.sample_count();
    if (status_.excitation_phase != Status::EXCITATION_COLLECT || f.motor_slew_active || f.arm_ramp_active ||
        static_cast<std::uint64_t>(sample_time) < steady_since_ || speed < kStoppedSpeedMps ||
        f.longitudinal <= 0.0F || applied <= 0.0F ||
        std::fabs(f.longitudinal - longitudinal_ / config_.motor_maximum) > 1.0e-3F) return StepResult::Busy;
    // 复用辨识器既有最小数据跨度覆盖冻结后的响应；起步前的零速/爬升样本
    // 不能独自凑足模型证据。保留完整动态数据，不把这段时间宣称为已达到稳态。
    if (1.0e-6F * static_cast<float>(sample_time - static_cast<std::int64_t>(steady_since_)) <
        static_cast<float>(configuration.minimum_samples) * configuration.sample_period_s) return StepResult::Busy;
    const auto result = identifier_.fit({});
    if (!result.model.valid) return StepResult::Busy;
    // 此处只消费动态模型，不消费fit附带的PI（PiRejected不否定已有效的模型）。
    // K轮端=Δv/Δq，再按冻结工作点q/u换回控制器整形前坐标的等效前馈系数。
    // 不直接把轮端增益写给整形前控制器，也不声称测得了未覆盖的物理最高车速。
    const float predicted = identification_output_origin_ + result.model.dc_gain *
        (applied - identification_input_origin_);
    const float candidate = predicted / f.longitudinal;
    if (!std::isfinite(candidate) || candidate <= 0.0F || candidate > 100.0F)
        return abort_step(Status::FAILURE_DYNAMICS_UNOBSERVABLE);
    status_.maximum_speed_m_s = candidate;
    longitudinal_ = 0.0F;
    PX4_INFO_RAW("[autocal] speed FF candidate %.3f tau=%.2fs samples=%lu; returning\n",
        static_cast<double>(candidate), static_cast<double>(result.model.time_constant_s),
        static_cast<unsigned long>(identifier_.sample_count()));
    return StepResult::WantReturn;
}

void AutoCalibrationMode::stop_turn_rate_request() noexcept
{
    // 模式只撤销物理目标；已有执行层在闭环/开环切换时复位唯一的 PI 实例。
    if (turn_rate_request_active_) {
        status_.closed_loop = false;
        physical_speed_ = physical_rate_ = 0.0F;
    }
    turn_rate_request_active_ = false;
    turn_rate_closed_loop_ = turn_excitation_locked_ = false;
    turn_excitation_input_ = 0.0F;
    turn_excitation_observed_since_ = 0U;
    turn_rate_target_ = 0.0F;
    turn_rate_opposed_since_ = 0U;
    turn_rate_opposed_heading_ = 0.0F;
}

StepResult AutoCalibrationMode::request_turn_rate(std::uint64_t now, float target_rate) noexcept
{
    if (!std::isfinite(target_rate) || !std::isfinite(yaw_rate())) {
        stop_turn_rate_request();
        longitudinal_ = steering_ = 0.0F;
        return abort_step(Status::FAILURE_SENSOR_STALE);
    }
    if (std::fabs(target_rate) < 1.0e-6F) {
        stop_turn_rate_request();
        longitudinal_ = steering_ = 0.0F;
        return StepResult::Busy;
    }
    if (!turn_rate_request_active_) {
        // 初始辨识固定使用已有开环实验通路，不依赖也不继承旧 PID 效果。
        // 本轮内环已经验证后才复用原控制器；选择在机动内冻结，禁止带载切换。
        turn_rate_closed_loop_ = (status_.provisional_validated_stages & Status::STAGE_INNER_GAINS) != 0U &&
            drive_.calibration_yaw_control_ready(std::min(std::fabs(target_rate), 0.5F * 3.0F * kRadians));
        turn_excitation_input_ = 0.0F;
        turn_excitation_locked_ = false;
        turn_excitation_observed_since_ = 0U;
    }
    if (target_rate * turn_rate_target_ < 0.0F) {
        // 本层只维护实验观察窗；积分、设定整形和换向等待由现有执行链处理。
        turn_rate_opposed_since_ = 0U;
        turn_excitation_input_ = 0.0F;
        turn_excitation_locked_ = false;
        turn_excitation_observed_since_ = 0U;
    }
    turn_rate_target_ = target_rate;
    turn_rate_request_active_ = true;
    status_.closed_loop = turn_rate_closed_loop_;
    physical_speed_ = 0.0F;
    physical_rate_ = turn_rate_closed_loop_ ? target_rate : 0.0F;
    longitudinal_ = 0.0F;
    // 闭环只发物理目标，steering_ 缓存真实控制轴供拟合；开环则表示实验激励。
    const auto &feedback = control_feedback_sub_.get();
    const bool feedback_current = feedback.closed_loop == turn_rate_closed_loop_ &&
        feedback.source == rover_motion_request_s::SOURCE_CALIBRATION &&
        feedback.session_id == status_.session_id && fresh(feedback.timestamp, now, 100000ULL);
    if (turn_rate_closed_loop_) {
        steering_ = feedback_current && feedback.valid && std::isfinite(feedback.steering)
            ? feedback.steering : 0.0F;
    } else {
        // 起转探测沿用已有 0.04/s 激励与 0.5 s 观测确认，确认后建立辨识基线。
        // 运动识别沿用PROFILE已有0.03判据，删除另加的0.08起转要求；动作完成由调用方
        // 的目标航向/累计转角决定，真实响应只用于采样和辨识。
        if (feedback_current && std::isfinite(feedback.applied_steering) &&
            std::fabs(feedback.applied_steering) > 1.0e-4F && std::fabs(yaw_rate()) > 0.03F) {
            if (turn_excitation_observed_since_ == 0U) turn_excitation_observed_since_ = now;
            if (now - turn_excitation_observed_since_ >= 500000ULL) turn_excitation_locked_ = true;
        } else turn_excitation_observed_since_ = 0U;
        const float dt = std::min(0.05F, control_interval_s(now));
        // 两级激励：探测级过可辨识地板并确认后建立基线（采样在输入保持段
        // 进行）；覆盖级不冻结——速率低于 0.7×目标带下沿时继续有界爬升
        // （0.08/s，快于探测级），进入目标带保持。锁定即冻结会让车以地板
        // 速率（0.03 rad/s≈2°/s）跑完 2π——那是探测判据不是工作点。
        // 关键：观测确认窗只在锁定【前】封锁爬升；锁定后速率只要 ≥0.03，
        // observed_since 就永不为零，若继续用它封锁，覆盖级永远不会执行
        // （实车 3-5°/s 慢转根因）。物理达不到目标带的车由方向窗口
        // 自适应（停滞滑窗+绝对上限）兜底，不在这里硬顶满舵。
        const bool confirm_window = !turn_excitation_locked_ && turn_excitation_observed_since_ != 0U;
        const bool below_band = std::fabs(yaw_rate()) < 0.7F * std::fabs(target_rate);
        const float ramp_rate = turn_excitation_locked_ && below_band ? 0.08F : 0.04F;
        if (!(confirm_window || turn_rate_opposed_since_ != 0U) && (!turn_excitation_locked_ || below_band))
            turn_excitation_input_ = std::min(1.0F, turn_excitation_input_ + ramp_rate * dt);
        steering_ = target_rate > 0.0F ? turn_excitation_input_ : -turn_excitation_input_;
    }
    const float rate = turn_rate_closed_loop_ ? feedback.yaw_rate_rad_s : yaw_rate();
    const float executed_target = turn_rate_closed_loop_ ? feedback.yaw_rate_setpoint_rad_s : target_rate;
    // 闭环比较实际设定，开环只检查实验指定的旋转方向；都只使用物理反馈，
    // 不从 PWM 或 REV 推断方向。闭环目标整形尚在旧方向时不作反向判定。
    const bool opposed = feedback_current &&
        (!turn_rate_closed_loop_ || (feedback.valid && fresh(feedback.timestamp_sample, now, 200000ULL))) &&
        std::isfinite(executed_target) && std::isfinite(rate) &&
        executed_target * target_rate > 0.0F && rate * executed_target < 0.0F &&
        std::fabs(rate) >= 0.03F &&
        imu_quality(now) && rtk_quality(now);
    if (opposed) {
        if (turn_rate_opposed_since_ == 0U) {
            turn_rate_opposed_since_ = now;
            turn_rate_opposed_heading_ = rtk_sub_.get().array_heading_rad;
        }
        const float heading_delta = math::wrap_pi(
            rtk_sub_.get().array_heading_rad - turn_rate_opposed_heading_);
        // 陀螺仪连续反向 300 ms，且独立 RTK 航向净反转至少 1 度才确认。
        // 这里仅停止错误响应，不识别/保存方向，不改符号、RC 或 PWM 参数。
        if (now - turn_rate_opposed_since_ >= 300000ULL &&
            heading_delta * target_rate < 0.0F && std::fabs(heading_delta) >= kRadians) {
            capture_motion_failure("direction", now);
            px4_log_raw(_PX4_LOG_LEVEL_WARN, "[autocal] turn direction mismatch: target=%.3f rate=%.3f\n",
                static_cast<double>(executed_target), static_cast<double>(rate));
            steering_ = 0.0F;
            stop_turn_rate_request();
            return abort_step(Status::FAILURE_MOTION_ENVELOPE);
        }
    } else {
        turn_rate_opposed_since_ = 0U;
    }
    return StepResult::Busy;
}

StepResult AutoCalibrationMode::straight_turnaround(std::uint64_t now) noexcept
{
    // 模式只发送turn_heading_目标（见publish），不生成转速或转向输入，也不按固定用时要求完成。
    status_.closed_loop = false;
    longitudinal_ = steering_ = physical_speed_ = physical_rate_ = 0.0F;
    const auto &feedback = control_feedback_sub_.get();
    if (!fresh(feedback.timestamp, now, 100000ULL) || feedback.source != rover_motion_request_s::SOURCE_CALIBRATION ||
        feedback.session_id != status_.session_id || feedback.heading_request_timestamp != session_.substate_started)
        return StepResult::Busy;
    if (feedback.heading_result == rover_control_status_s::HEADING_FAILED) {
        PX4_WARN("[autocal] heading maneuver failed: target=%.3f remaining=%.3f",
            static_cast<double>(turn_heading_), static_cast<double>(feedback.heading_error_rad));
        return abort_step(Status::FAILURE_MOTION_ENVELOPE);
    }
    if (!feedback.valid || feedback.heading_result != rover_control_status_s::HEADING_COMPLETE) return StepResult::Busy;
    leg_heading_ = turn_heading_;
    return StepResult::Advance;
}

StepResult AutoCalibrationMode::turn_spin(std::uint64_t now) noexcept
{
    // 原地 CW/CCW 旋转：角速度辨识采样与磁覆盖采集。方向由
    // session_.turn_direction 表达，CW/CCW 切换在本 helper 内部完成（写意图
    // 字段并复位方向局部量），双向结束后才 Advance 交调度器停车评估。
    const int direction = session_.turn_direction;
    longitudinal_ = 0.0F;
    const unsigned side = direction > 0 ? 0U : 1U;
    // 每方向窗口自适应：75s 是"无有效旋转"的停滞窗（自最后一次有效旋转
    // 滑动起算）；在指令方向以可辨识速率（≥0.03 rad/s，与起转探测同一
    // 地板）旋转时窗口随旋转延长——转向速率是被测量，固定 75s 会把慢车
    // 系统性排除在 yaw/磁覆盖之外。绝对上限=可辨识地板转满 2π 加 bias
    // 学习窗与余量（270s）：更慢的旋转本来就不可辨识，多等无样本。
    // 到期走既有换向/评估出口（组级继续），不改失败语义。
    if (state_started_ < session_.substate_started) state_started_ = now;
    if (yaw_rate() * direction >= 0.03F) turn_productive_at_ = now;
    const std::uint64_t stall_anchor = turn_productive_at_ != 0U ? turn_productive_at_ : state_started_;
    // 正常采集结束与方向超时共用下方清零/换向出口；超时不再复制一套复位动作。
    if (now - stall_anchor <= 75000000ULL && now - state_started_ <= 270000000ULL) {
        // 原地转向时 GNSS 天线绕车心运动，地速不为零；只在起转前等待车体停止，
        // 转起来后由总位移/速度包络门控，避免把 lever arm 速度当成直线滑行。
        if (!turn_started_) {
            if (!stopped()) { steering_ = 0.0F; return StepResult::Busy; }
            stop_turn_rate_request();
            // 起转确认只锁定一次，不以 steering 大小推断；高 yaw 增益车辆可能
            // 只需很小输入。换向后的滑行角不计入新方向，也不冒充航向跳变。
            turn_started_ = true;
            turn_integral_ = 0.0F;
            last_turn_heading_ = rtk_sub_.get().array_heading_rad;
        }
        // 尚未知道 yaw 前馈系数，不能把 steering ceiling 当成恒定激励直接输出。
        // 与掉头/返程使用同一实验通路选择；参数可用时交原控制器，否则开环
        // 起步并保持激励采样，旋转圈数与有效样本决定完成；本阶段不另建 PI。
        const float previous = steering_;
        const StepResult control = request_turn_rate(now, direction * 0.3F);
        if (control != StepResult::Busy) return control;
        const auto &feedback = control_feedback_sub_.get();
        const float applied = feedback.applied_steering;
        // 辨识使用执行层的统一转向轴与物理转速；模式不自行从左右电机反算。
        // 不设固定电机输入下限；高增益车辆的小输入也可形成有效辨识样本。
        const bool input_held_rate = std::fabs(steering_ - previous) <= 0.0002F &&
            yaw_rate() * direction > 0.0F &&
            feedback.source == rover_motion_request_s::SOURCE_CALIBRATION &&
            feedback.session_id == status_.session_id && fresh(feedback.timestamp, now, 100000ULL) && std::isfinite(applied) &&
            feedback_unmasked() && !feedback.motor_slew_active && std::isfinite(feedback.steering) &&
            std::fabs(feedback.steering - steering_) < 0.015F &&
            applied * direction > 0.0F && steering_ * direction > 0.0F;
        if (!input_held_rate) steady_since_ = 0U;
        else if (steady_since_ == 0U) steady_since_ = now;
        if (new_heading_epoch()) {
            const float heading = rtk_sub_.get().array_heading_rad;
            const float delta = math::wrap_pi(heading - last_turn_heading_);
            last_turn_heading_ = heading;
            // 只累计新鲜有效RTK历元的有符号转角；固定每帧10度会把正常转速
            // 与接收机频率混为故障，删除该隐含转速上限，仍保留独立质量/方向检查。
            turn_integral_ += delta;
            const float rate = yaw_rate();
            if (input_held_rate &&
                steady_since_ != 0U && now - steady_since_ >= 1000000ULL &&
                status_.maximum_speed_m_s > 0.0F) {
                // 从 steering=rate*track*correction/(2*maximum_speed) 反解；
                // steering 是整形前控制轴；正常 MIN/EXPO 由统一执行链实现，
                // 不能拿它与整形后 applied 的差值作为拒绝辨识的条件。
                // 左右方向分别统计，最终必须一致，不能用单一系数掩盖机械不对称。
                const float correction = 2.0F * status_.maximum_speed_m_s * steering_ / (config_.track * rate);
                if (std::isfinite(correction) && correction >= 0.01F && correction <= 100.0F)
                    yaw_fit_[side].add(correction, 0.0, 0.0);
            }
        }
        // 换向期间仍可能沿旧方向运动；转向候选只统计方向一致、输入保持的窗口。
        // 磁采集不要求固定转速；只要有新鲜 RTK/EKF 姿态和
        // 有界开环旋转，就可以积累方向覆盖。TURN 的 rate 统计仍只使用
        // input_held_rate 样本，不以固定转速或角加速度要求阻断观察。
        // collect_mag 自身检查融合质量并在失败时清稳定窗，调用点不重复检查。
        collect_mag(now, side);
        // 天线杆臂自动测量（2026-09-30 用户确认）：原地旋转时天线绕旋转中心
        // 画圆，圆半径=天线到旋转中心的实际杆臂。流式累积位置二阶矩，方向
        // 完成（≥2π）时结算；参数和只是保守代理（实测 0.30m vs 参数和 0.82m）。
        if (turn_started_) {
            float north{}, east{};
            math::displacement(gps_sub_.get().latitude_deg, gps_sub_.get().longitude_deg,
                rotation_lever_lat_, rotation_lever_lon_, north, east);
            rotation_lever_n_sum_ += north;
            rotation_lever_e_sum_ += east;
            rotation_lever_nn_ += north * north;
            rotation_lever_ee_ += east * east;
            ++rotation_lever_count_;
        }
        if (direction * turn_integral_ < 2.0F * kPi) return StepResult::Busy;
        if (first_circle_at_ == 0U) first_circle_at_ = now;
        // 一圈只是覆盖下限，不是学习时间充分的证明。允许继续有界转动，让每个
        // 方向在连续 10 s 学习窗之后至少取得 5 份 bias；首圈后最多再等 45 s，
        // 且方向窗口的 75 s 硬截止优先，不允许慢圈带来无上限运动。
        const bool bias_expected = bootstrap_applied_ || (config_.mag_id > 0 && flags_sub_.get().cs_mag && !flags_sub_.get().cs_mag_field_disturbed);
        if (mag_path_ready(now) && bias_fit_[side].count() < 5U &&
            (bias_expected || bootstrap_fit_[side].count() < 60U) && now - first_circle_at_ < 45000000ULL)
            return StepResult::Busy;
    }
    // 本方向已完成或时间用尽，都先撤销请求；另一方向启动仍须确认真实停车。
    steering_ = 0.0F;
    stop_turn_rate_request();
    finalize_rotation_lever(direction);
    if (direction > 0) {
        // CW 完成：切换 CCW。方向局部量复位等价旧 TURN_CCW 阶段入场复位，
        // 起转前重新等待真实停车。
        session_.turn_direction = -1;
        turn_integral_ = 0.0F;
        turn_started_ = false;
        last_turn_heading_ = rtk_sub_.get().array_heading_rad;
        mag_stable_since_ = steady_since_ = first_circle_at_ = 0U;
        state_started_ = now;
        turn_productive_at_ = 0U;
        return StepResult::Busy;
    }
    return StepResult::Advance;
}

void AutoCalibrationMode::reset_rotation_lever_window() noexcept
{
    rotation_lever_n_sum_ = rotation_lever_e_sum_ = 0.0;
    rotation_lever_nn_ = rotation_lever_ee_ = 0.0;
    rotation_lever_count_ = 0U;
    const auto &gps = gps_sub_.get();
    rotation_lever_lat_ = gps.latitude_deg;
    rotation_lever_lon_ = gps.longitude_deg;
}

void AutoCalibrationMode::finalize_rotation_lever(int direction) noexcept
{
    // 方向结算：完整一圈（integral≥2π；停滞/超时出口不满一圈不结算）且样本充足时，
    // var(N)+var(E) = r² + 2σ²（圆均匀采样+各向同性噪声），噪声修正可忽略
    // （σ≈eph≈0.014）。两方向取大（保守侧），供 PATH/PROFILE 几何使用。
    if (direction * turn_integral_ < 2.0F * kPi || rotation_lever_count_ < 600U) {
        reset_rotation_lever_window();
        return;
    }
    const double n = static_cast<double>(rotation_lever_count_);
    const double var_sum = (rotation_lever_nn_ / n - (rotation_lever_n_sum_ / n) * (rotation_lever_n_sum_ / n)) +
        (rotation_lever_ee_ / n - (rotation_lever_e_sum_ / n) * (rotation_lever_e_sum_ / n));
    const double radius = std::sqrt(std::max(var_sum, 0.0));
    if (std::isfinite(radius) && radius > 0.05F && radius < 5.0F) {
        measured_lever_m_ = measured_lever_valid_ ? std::max(measured_lever_m_, static_cast<float>(radius))
                                                 : static_cast<float>(radius);
        measured_lever_valid_ = true;
        PX4_INFO("[autocal] measured antenna lever %.2fm (dir %+d, n=%lu)",
            static_cast<double>(measured_lever_m_), direction,
            static_cast<unsigned long>(rotation_lever_count_));
    }
    reset_rotation_lever_window();
    (void)direction;
}

bool AutoCalibrationMode::finish_rtk() noexcept
{
    // 基础去/返程仅合并RTK安装偏置。制动试验占用的里程不再被当作速度档位；
    // 速度候选在RTK应用确认后单独实验，不能在这里提前判不可观。
    for (unsigned side = 0U; side < 2U; ++side) {
        const auto &fit = heading_mean_[side];
        const double concentration = fit.weight > 0.0 ? std::hypot(fit.sine, fit.cosine) / fit.weight : 0.0;
        const double offset = fit.weight > 0.0 ? std::atan2(fit.sine, fit.cosine) / kRadians : NAN;
        PX4_INFO_RAW("[autocal] RTK %s n=%lu offset=%.2fdeg R=%.6f\n", side == 0U ? "out" : "back",
            static_cast<unsigned long>(fit.count), offset, concentration);
    }
    float outward{}, inward{};
    if (!heading_mean_[0].result(outward) || !heading_mean_[1].result(inward)) {
        PX4_WARN("[autocal] RTK offset rejected: sample count/concentration");
        return false;
    }
    const float difference = std::fabs(math::wrap_pi(outward - inward));
    if (difference > 3.0F * kRadians) {
        PX4_WARN("[autocal] RTK offset rejected: leg difference %.2fdeg", static_cast<double>(difference / kRadians));
        return false;
    }
    // 先分别验证两段，再按方向等权合并，避免慢速一段样本更多就掩盖另一方向。
    float offset = std::atan2(std::sin(outward) + std::sin(inward), std::cos(outward) + std::cos(inward));
    if (offset < 0.0F) offset += 2.0F * kPi;
    status_.rtk_yaw_offset_deg = offset / kRadians;
    return true;
}

bool AutoCalibrationMode::finish_dynamics() noexcept
{
    // 纯算法：CW/CCW 角速度修正合并。
    if (status_.maximum_speed_m_s <= 0.0F || yaw_fit_[0].count() < 30U || yaw_fit_[1].count() < 30U) return false;
    const double a = yaw_fit_[0].mean().x, b = yaw_fit_[1].mean().x;
    const double mean = 0.5 * (a + b);
    // 地面和左右载荷允许两向响应不同；分别有效后合并，不把20%对称性当作健康门。
    if (!std::isfinite(mean) || mean <= 0.0) return false;
    status_.yaw_rate_correction = static_cast<float>(mean);
    return true;
}

} // namespace dima::rover::modes

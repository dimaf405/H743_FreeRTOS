#include "RoverDifferential.hpp"

#include "events/events.hpp"
#include "api/Time.hpp"

#include <cmath>
#include <limits>

namespace dima::rover::control {
namespace {

constexpr std::uint32_t kEventParameterInvalid = 0x52444601U;
constexpr std::uint32_t kEventPublishFailure = 0x52444602U;
constexpr std::uint32_t kEventScheduleFailure = 0x52444603U;
constexpr std::uint32_t kEventClockRegression = 0x52444604U;
// 仅 actuator_motors.control[0..1] 是可逆左右电机；其余通道必须保持 NaN 不可用语义。
constexpr std::uint16_t kReversibleMotorMask = 0x0003U;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegreesToRadians = kPi / 180.0F;
constexpr float kUnavailable = std::numeric_limits<float>::quiet_NaN();
constexpr std::uint32_t kManualModeMask =
    1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
constexpr std::uint32_t kImplementedModeMask =
    kManualModeMask |
    (1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) |
    (1UL << vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) |
    (1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION);

float angular_parameter_to_radians(float value) noexcept
{
    // 参数层沿用 PX4 的 deg/s、deg/s^2 单位，控制核使用 SI rad/s、rad/s^2；
    // -1 禁用哨兵保持原值，避免被换算成一个看似有效的小负数。
    return value < 0.0F ? value : value * kDegreesToRadians;
}

bool exact_manual_control_projection(
    const vehicle_control_mode_s &control) noexcept
{
    return control.source_id == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
        control.flag_control_manual_enabled &&
        !control.flag_control_auto_enabled &&
        !control.flag_control_offboard_enabled &&
        !control.flag_control_position_enabled &&
        !control.flag_control_velocity_enabled &&
        !control.flag_control_altitude_enabled &&
        !control.flag_control_climb_rate_enabled &&
        !control.flag_control_acceleration_enabled &&
        !control.flag_control_attitude_enabled &&
        !control.flag_control_rates_enabled &&
        !control.flag_control_allocation_enabled &&
        !control.flag_control_termination_enabled &&
        !control.flag_multicopter_position_control_enabled;
}

bool exact_navigation_control_projection(
    const vehicle_control_mode_s &control) noexcept
{
    const bool supported =
        control.source_id ==
            vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        control.source_id == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return supported && !control.flag_control_manual_enabled &&
        control.flag_control_auto_enabled &&
        !control.flag_control_offboard_enabled &&
        control.flag_control_position_enabled &&
        control.flag_control_velocity_enabled &&
        !control.flag_control_altitude_enabled &&
        !control.flag_control_climb_rate_enabled &&
        !control.flag_control_acceleration_enabled &&
        control.flag_control_attitude_enabled &&
        control.flag_control_rates_enabled &&
        !control.flag_control_allocation_enabled &&
        !control.flag_control_termination_enabled &&
        !control.flag_multicopter_position_control_enabled;
}

} // namespace

RoverDifferential::RoverDifferential() noexcept
    : px4::ScheduledWorkItem("rover_differential",
                             px4::wq_configurations::rate_ctrl)
{
}

RoverDifferential::~RoverDifferential()
{
    stop();
}

bool RoverDifferential::start()
{
    // 启动必须先建立 work-item 执行权，再绑定并验证整组参数；首次对外发布 NaN
    // 无效帧，确保下游在收到第一条有效运动请求前不会沿用旧电机命令。
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    reset_runtime_state();
    if (!bind_parameters()) {
        enter_error(kEventParameterInvalid);
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    const std::uint64_t now = hrt_absolute_time();
    if (!publish_invalid(now, 0U)) {
        enter_error(kEventPublishFailure);
        return false;
    }
    if (!ScheduleOnInterval(kRunIntervalUs, 1U)) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    return true;
}

void RoverDifferential::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    drive_.reset();
    reset_navigation_control();
    (void)publish_invalid(hrt_absolute_time(), 0U);
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState RoverDifferential::state() const
{
    return state_;
}

void RoverDifferential::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s parameter_update{};
    if (parameter_update_subscription_.copy(&parameter_update)) {
        parameter_update_pending_ = true;
    }

    // motion_request 是共享有界队列，但 Manual 与 Navigation 分别保存最新帧。
    // 因此非活动模式发布的无效帧只会清自己的来源，不能以“队列最后一帧获胜”
    // 覆盖 Commander 当前安全投影所选择的活动来源。
    rover_motion_request_s request{};
    for (std::uint8_t count = 0U;
         count < kMotionRequestQueueDepth &&
         motion_request_subscription_.copy(&request);
         ++count) {
        if (request.source == rover_motion_request_s::SOURCE_MANUAL) {
            manual_request_ = request;
            have_manual_request_ = true;
        } else if (request.source ==
                   rover_motion_request_s::SOURCE_NAVIGATION) {
            navigation_request_ = request;
            have_navigation_request_ = true;
        }
    }

    if (vehicle_local_position_subscription_.update()) {
        vehicle_local_position_ =
            vehicle_local_position_subscription_.get();
        have_local_position_ = true;
    }
    if (vehicle_odometry_subscription_.update()) {
        vehicle_odometry_ = vehicle_odometry_subscription_.get();
        have_odometry_ = true;
    }

    const std::uint64_t now = hrt_absolute_time();
    refresh_safety_snapshot(now);
    (void)apply_pending_parameters(now);

    // 斜率限制、换向延时和解锁渐入均依赖真实运行间隔；时钟回退时 dt 无定义，
    // 因而必须清空控制器并 fail-closed，不能退回名义周期继续输出。
    if (last_run_time_us_ != 0U && now < last_run_time_us_) {
        drive_.reset();
        reset_navigation_control();
        const rover_motion_request_s *active = active_request();
        (void)publish_invalid(now,
                              active != nullptr ? active->timestamp_sample
                                                : 0U);
        enter_error(kEventClockRegression);
        return;
    }

    float dt_s = static_cast<float>(kRunIntervalUs) * 1.0e-6F;
    if (last_run_time_us_ != 0U) {
        dt_s = static_cast<float>(now - last_run_time_us_) * 1.0e-6F;
    }
    last_run_time_us_ = now;

    if (!publish_output(now, dt_s)) {
        enter_error(kEventPublishFailure);
    }
}

bool RoverDifferential::bind_parameters() noexcept
{
    const bool bound = command_timeout_.bind() && reverse_steering_.bind() &&
                       steering_throttle_mix_.bind() && throttle_min_.bind() &&
                       throttle_max_.bind() && throttle_slew_rate_.bind() &&
                       reversal_delay_.bind() && throttle_expo_.bind() &&
                       thrust_asymmetry_.bind() && arm_ramp_.bind() &&
                       maximum_speed_.bind() && speed_p_.bind() &&
                       speed_i_.bind() && acceleration_limit_.bind() &&
                       deceleration_limit_.bind() && speed_threshold_.bind() &&
                       yaw_rate_p_.bind() && yaw_rate_i_.bind() &&
                       yaw_rate_limit_.bind() &&
                       yaw_rate_correction_.bind() &&
                       yaw_acceleration_.bind() && yaw_deceleration_.bind() &&
                       yaw_rate_threshold_.bind() && wheel_track_.bind();
    if (!bound || !apply_parameter_snapshot()) {
        invalidate_parameter_bindings();
        parameters_valid_ = false;
        return false;
    }
    return true;
}

void RoverDifferential::invalidate_parameter_bindings() noexcept
{
    command_timeout_.invalidate();
    reverse_steering_.invalidate();
    steering_throttle_mix_.invalidate();
    throttle_min_.invalidate();
    throttle_max_.invalidate();
    throttle_slew_rate_.invalidate();
    reversal_delay_.invalidate();
    throttle_expo_.invalidate();
    thrust_asymmetry_.invalidate();
    arm_ramp_.invalidate();
    maximum_speed_.invalidate();
    speed_p_.invalidate();
    speed_i_.invalidate();
    acceleration_limit_.invalidate();
    deceleration_limit_.invalidate();
    speed_threshold_.invalidate();
    yaw_rate_p_.invalidate();
    yaw_rate_i_.invalidate();
    yaw_rate_limit_.invalidate();
    yaw_rate_correction_.invalidate();
    yaw_acceleration_.invalidate();
    yaw_deceleration_.invalidate();
    yaw_rate_threshold_.invalidate();
    wheel_track_.invalidate();
}

bool RoverDifferential::apply_parameter_snapshot() noexcept
{
    if (!command_timeout_.bound() || !reverse_steering_.bound() ||
        !steering_throttle_mix_.bound() || !throttle_min_.bound() ||
        !throttle_max_.bound() || !throttle_slew_rate_.bound() ||
        !reversal_delay_.bound() || !throttle_expo_.bound() ||
        !thrust_asymmetry_.bound() || !arm_ramp_.bound() ||
        !maximum_speed_.bound() || !speed_p_.bound() || !speed_i_.bound() ||
        !acceleration_limit_.bound() || !deceleration_limit_.bound() ||
        !speed_threshold_.bound() || !yaw_rate_p_.bound() ||
        !yaw_rate_i_.bound() || !yaw_rate_limit_.bound() ||
        !yaw_rate_correction_.bound() || !yaw_acceleration_.bound() ||
        !yaw_deceleration_.bound() || !yaw_rate_threshold_.bound() ||
        !wheel_track_.bound()) {
        parameters_valid_ = false;
        return false;
    }

    // 原子读取整组候选快照；只有全部读取、交叉约束和控制器配置均成功，才一次
    // 性替换当前有效参数，避免一次 PARAM_SET 让控制器看到跨代混合配置。
    ParameterSnapshot candidate{};
    std::int32_t reverse_steering = 0;
    float yaw_rate_limit_deg_s{};
    float yaw_acceleration_deg_s2{};
    float yaw_deceleration_deg_s2{};
    float yaw_rate_threshold_deg_s{};
    bool loaded = false;
    {
        px4::AtomicTransaction transaction;
        loaded = param_get(command_timeout_.handle(),
                           &candidate.command_timeout_s) == 0 &&
                 param_get(reverse_steering_.handle(), &reverse_steering) == 0 &&
                 param_get(steering_throttle_mix_.handle(),
                           &candidate.drive.steering_throttle_mix) == 0 &&
                 param_get(throttle_min_.handle(),
                           &candidate.drive.throttle_min) == 0 &&
                 param_get(throttle_max_.handle(),
                           &candidate.drive.throttle_max) == 0 &&
                 param_get(throttle_slew_rate_.handle(),
                           &candidate.drive.throttle_slew_rate) == 0 &&
                 param_get(reversal_delay_.handle(),
                           &candidate.drive.reversal_delay_s) == 0 &&
                 param_get(throttle_expo_.handle(),
                           &candidate.drive.throttle_expo) == 0 &&
                 param_get(thrust_asymmetry_.handle(),
                           &candidate.drive.thrust_asymmetry) == 0 &&
                 param_get(arm_ramp_.handle(),
                           &candidate.drive.arm_ramp_s) == 0 &&
                 param_get(maximum_speed_.handle(),
                           &candidate.speed.speed_at_full_throttle_m_s) == 0 &&
                 param_get(speed_p_.handle(),
                           &candidate.speed.proportional_gain) == 0 &&
                 param_get(speed_i_.handle(),
                           &candidate.speed.integral_gain) == 0 &&
                 param_get(acceleration_limit_.handle(),
                           &candidate.speed.acceleration_limit_m_s2) == 0 &&
                 param_get(deceleration_limit_.handle(),
                           &candidate.speed.deceleration_limit_m_s2) == 0 &&
                 param_get(speed_threshold_.handle(),
                           &candidate.speed.measurement_threshold_m_s) == 0 &&
                 param_get(yaw_rate_p_.handle(),
                           &candidate.yaw_rate.proportional_gain) == 0 &&
                 param_get(yaw_rate_i_.handle(),
                           &candidate.yaw_rate.integral_gain) == 0 &&
                 param_get(yaw_rate_limit_.handle(),
                           &yaw_rate_limit_deg_s) == 0 &&
                 param_get(yaw_rate_correction_.handle(),
                           &candidate.yaw_rate.yaw_rate_correction) == 0 &&
                 param_get(yaw_acceleration_.handle(),
                           &yaw_acceleration_deg_s2) == 0 &&
                 param_get(yaw_deceleration_.handle(),
                           &yaw_deceleration_deg_s2) == 0 &&
                 param_get(yaw_rate_threshold_.handle(),
                           &yaw_rate_threshold_deg_s) == 0 &&
                 param_get(wheel_track_.handle(),
                           &candidate.yaw_rate.wheel_track_m) == 0;
    }
    if (!loaded) {
        parameters_valid_ = false;
        return false;
    }
    candidate.drive.reverse_steering_in_manual = reverse_steering != 0;
    candidate.yaw_rate.speed_at_full_throttle_m_s =
        candidate.speed.speed_at_full_throttle_m_s;
    candidate.yaw_rate.yaw_rate_limit_rad_s =
        angular_parameter_to_radians(yaw_rate_limit_deg_s);
    candidate.yaw_rate.yaw_acceleration_limit_rad_s2 =
        angular_parameter_to_radians(yaw_acceleration_deg_s2);
    candidate.yaw_rate.yaw_deceleration_limit_rad_s2 =
        angular_parameter_to_radians(yaw_deceleration_deg_s2);
    candidate.yaw_rate.measurement_threshold_rad_s =
        angular_parameter_to_radians(yaw_rate_threshold_deg_s);

    if ((reverse_steering != 0 && reverse_steering != 1) ||
        !valid_parameter_snapshot(candidate) ||
        !drive_.configure(candidate.drive)) {
        parameters_valid_ = false;
        drive_.reset();
        return false;
    }

    // 车辆专属增益默认未标定时，Manual 混控仍保持可用；AUTO readiness 与
    // Navigation 请求单独由 navigation_parameters_valid_ 锁闭。只有整组四环
    // 内环参数有效时才同时配置两个 PI，不能只启用其中一个半闭环。
    navigation_parameters_valid_ =
        valid_navigation_parameter_snapshot(candidate) &&
        speed_controller_.configure(candidate.speed) &&
        yaw_rate_controller_.configure(candidate.yaw_rate);
    if (!navigation_parameters_valid_) {
        reset_navigation_control();
    }

    command_timeout_.set(candidate.command_timeout_s);
    reverse_steering_.set(reverse_steering);
    steering_throttle_mix_.set(candidate.drive.steering_throttle_mix);
    throttle_min_.set(candidate.drive.throttle_min);
    throttle_max_.set(candidate.drive.throttle_max);
    throttle_slew_rate_.set(candidate.drive.throttle_slew_rate);
    reversal_delay_.set(candidate.drive.reversal_delay_s);
    throttle_expo_.set(candidate.drive.throttle_expo);
    thrust_asymmetry_.set(candidate.drive.thrust_asymmetry);
    arm_ramp_.set(candidate.drive.arm_ramp_s);
    maximum_speed_.set(candidate.speed.speed_at_full_throttle_m_s);
    speed_p_.set(candidate.speed.proportional_gain);
    speed_i_.set(candidate.speed.integral_gain);
    acceleration_limit_.set(candidate.speed.acceleration_limit_m_s2);
    deceleration_limit_.set(candidate.speed.deceleration_limit_m_s2);
    speed_threshold_.set(candidate.speed.measurement_threshold_m_s);
    yaw_rate_p_.set(candidate.yaw_rate.proportional_gain);
    yaw_rate_i_.set(candidate.yaw_rate.integral_gain);
    yaw_rate_limit_.set(yaw_rate_limit_deg_s);
    yaw_rate_correction_.set(candidate.yaw_rate.yaw_rate_correction);
    yaw_acceleration_.set(yaw_acceleration_deg_s2);
    yaw_deceleration_.set(yaw_deceleration_deg_s2);
    yaw_rate_threshold_.set(yaw_rate_threshold_deg_s);
    wheel_track_.set(candidate.yaw_rate.wheel_track_m);
    parameters_ = candidate;
    parameters_valid_ = true;
    return true;
}

bool RoverDifferential::apply_pending_parameters(
    std::uint64_t now_us) noexcept
{
    // 运行期参数只允许在 Commander 发布的新鲜 Disarmed 一致快照下整体切换。
    if (!parameter_update_pending_ || !fresh_disarmed_snapshot(now_us)) {
        return false;
    }

    parameter_update_pending_ = false;
    if (apply_parameter_snapshot()) {
        return true;
    }
    (void)dima::events::report(kEventParameterInvalid,
                               dima::events::Severity::Error);
    return false;
}

void RoverDifferential::refresh_safety_snapshot(
    std::uint64_t now_us) noexcept
{
    // 任一 Topic 先观察到否定安全状态或更新代际就立即锁存抑制；后者覆盖
    // Manual -> AUTO 这类三个单项都“正向合法”、但尚未同拍的模式交接窗口。
    // 只有三项 Topic 以同一时间戳完整收敛后，才重新选择该代际的活动请求。
    if (actuator_armed_subscription_.update()) {
        observed_actuator_armed_ = actuator_armed_subscription_.get();
        const bool newer_generation = observed_actuator_armed_.timestamp != 0U &&
            (!safety_.valid || observed_actuator_armed_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_actuator_armed_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_control_mode_subscription_.update()) {
        observed_control_mode_ = vehicle_control_mode_subscription_.get();
        const bool newer_generation = observed_control_mode_.timestamp != 0U &&
            (!safety_.valid || observed_control_mode_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_control_mode_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_status_subscription_.update()) {
        observed_vehicle_status_ = vehicle_status_subscription_.get();
        const bool newer_generation = observed_vehicle_status_.timestamp != 0U &&
            (!safety_.valid || observed_vehicle_status_.timestamp >
                                   safety_.actuator_armed.timestamp);
        if (newer_generation || safety_negative(observed_vehicle_status_)) {
            safety_inhibit_observed_ = true;
        }
    }

    if (!observed_snapshot_complete(now_us)) {
        return;
    }

    safety_.actuator_armed = observed_actuator_armed_;
    safety_.control_mode = observed_control_mode_;
    safety_.vehicle_status = observed_vehicle_status_;
    safety_.valid = true;
    safety_inhibit_observed_ = safety_negative(safety_.actuator_armed) ||
                               safety_negative(safety_.control_mode) ||
                               safety_negative(safety_.vehicle_status);
}

bool RoverDifferential::observed_snapshot_complete(
    std::uint64_t now_us) const noexcept
{
    const std::uint64_t timestamp = observed_actuator_armed_.timestamp;
    return timestamp != 0U && timestamp == observed_control_mode_.timestamp &&
           timestamp == observed_vehicle_status_.timestamp &&
           timestamp <= now_us &&
           (!safety_.valid ||
            timestamp > safety_.actuator_armed.timestamp);
}

bool RoverDifferential::active_snapshot_fresh(
    std::uint64_t now_us) const noexcept
{
    const std::uint64_t timestamp = safety_.actuator_armed.timestamp;
    return safety_.valid && timestamp != 0U && timestamp <= now_us &&
           now_us - timestamp <= kSafetyTopicTimeoutUs;
}

bool RoverDifferential::fresh_disarmed_snapshot(
    std::uint64_t now_us) const noexcept
{
    if (!active_snapshot_fresh(now_us)) {
        return false;
    }
    const bool status_disarmed =
        safety_.vehicle_status.arming_state ==
        vehicle_status_s::ARMING_STATE_DISARMED;
    return status_disarmed && !safety_.actuator_armed.armed &&
           !safety_.control_mode.flag_armed;
}

bool RoverDifferential::safety_permits_output(
    std::uint64_t now_us) const noexcept
{
    // 任一新 Topic 出现负向安全证据就立即锁止；恢复必须等待三份同拍、鲜活且
    // 字段互相一致的安全快照。Manual 与两种 AUTO 投影逐字段精确匹配，不能用
    // “任意 auto flag”扩大允许范围。
    if (safety_inhibit_observed_ || !active_snapshot_fresh(now_us)) {
        return false;
    }

    const actuator_armed_s &armed = safety_.actuator_armed;
    const vehicle_control_mode_s &control = safety_.control_mode;
    const vehicle_status_s &status = safety_.vehicle_status;
    return armed.armed && armed.ready_to_arm && !armed.prearmed &&
           !armed.lockdown && !armed.kill && !armed.termination &&
           !armed.in_esc_calibration_mode && control.flag_armed &&
           status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
           status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
           status.valid_nav_states_mask == kImplementedModeMask &&
           status.can_set_nav_states_mask == kManualModeMask &&
           !status.failsafe &&
           (manual_projection(control, status) ||
            navigation_projection(control, status));
}

const rover_motion_request_s *RoverDifferential::active_request() const noexcept
{
    if (!safety_.valid) {
        return nullptr;
    }
    const std::uint8_t nav_state = safety_.vehicle_status.nav_state;
    if (nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
        return have_manual_request_ ? &manual_request_ : nullptr;
    }
    if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
        return have_navigation_request_ ? &navigation_request_ : nullptr;
    }
    return nullptr;
}

bool RoverDifferential::request_valid(
    const rover_motion_request_s &request, std::uint64_t now_us,
    bool navigation_source) const noexcept
{
    if (!request.valid || request.timestamp == 0U ||
        request.timestamp_sample == 0U ||
        request.timestamp_sample > request.timestamp ||
        request.timestamp > now_us || request.timestamp_sample > now_us ||
        safety_.vehicle_status.nav_state_timestamp == 0U ||
        request.timestamp <= safety_.vehicle_status.nav_state_timestamp) {
        return false;
    }

    if (navigation_source) {
        // Navigation one-of：只允许 SI 物理量，两个归一化字段必须明确为 NaN。
        // 首版禁止倒车航段，速度/yaw-rate 都必须落在已标定物理极限内，不能
        // 把超范围设定交给 PI 饱和后继续伪装成有效闭环。
        if (!navigation_parameters_valid_ ||
            request.source != rover_motion_request_s::SOURCE_NAVIGATION ||
            request.mode != rover_motion_request_s::MODE_SPEED_YAW_RATE ||
            !std::isnan(request.normalized_longitudinal) ||
            !std::isnan(request.normalized_steering) ||
            !finite(request.speed_m_s) || request.speed_m_s < 0.0F ||
            request.speed_m_s >
                parameters_.speed.speed_at_full_throttle_m_s ||
            !finite(request.yaw_rate_rad_s) ||
            std::fabs(request.yaw_rate_rad_s) >
                parameters_.yaw_rate.yaw_rate_limit_rad_s) {
            return false;
        }
    } else {
        // Manual one-of：只允许归一化双轴，物理量字段必须为 NaN，避免不同
        // 消费者对同一帧选择不同语义。
        if (request.source != rover_motion_request_s::SOURCE_MANUAL ||
            request.mode != rover_motion_request_s::MODE_NORMALIZED_AXES ||
            !normalized(request.normalized_longitudinal) ||
            !normalized(request.normalized_steering) ||
            !std::isnan(request.speed_m_s) ||
            !std::isnan(request.yaw_rate_rad_s)) {
            return false;
        }
    }

    // timeout_us = command_timeout_s * 1e6。timestamp 约束模块链路鲜度，
    // timestamp_sample 约束原始 RC 样本鲜度；两者都在窗口内才能避免刚转发的
    // 旧摇杆样本重新激活电机命令。
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        parameters_.command_timeout_s * 1000000.0F);
    return timeout_us > 0U &&
           now_us - request.timestamp <= timeout_us &&
           now_us - request.timestamp_sample <= timeout_us;
}

bool RoverDifferential::navigation_estimator_valid(
    std::uint64_t now_us) const noexcept
{
    if (!have_local_position_ || !have_odometry_ ||
        vehicle_local_position_.timestamp == 0U ||
        vehicle_local_position_.timestamp_sample == 0U ||
        vehicle_local_position_.timestamp_sample >
            vehicle_local_position_.timestamp ||
        vehicle_local_position_.timestamp > now_us ||
        now_us - vehicle_local_position_.timestamp > kEstimatorTimeoutUs ||
        now_us - vehicle_local_position_.timestamp_sample >
            kEstimatorTimeoutUs ||
        vehicle_odometry_.timestamp == 0U ||
        vehicle_odometry_.timestamp_sample == 0U ||
        vehicle_odometry_.timestamp_sample > vehicle_odometry_.timestamp ||
        vehicle_odometry_.timestamp > now_us ||
        now_us - vehicle_odometry_.timestamp > kEstimatorTimeoutUs ||
        now_us - vehicle_odometry_.timestamp_sample > kEstimatorTimeoutUs ||
        !vehicle_local_position_.xy_valid ||
        !vehicle_local_position_.v_xy_valid ||
        !vehicle_local_position_.heading_good_for_control ||
        !vehicle_local_position_.xy_global ||
        vehicle_local_position_.dead_reckoning ||
        vehicle_local_position_.ref_timestamp == 0U ||
        vehicle_local_position_.ref_timestamp >
            vehicle_local_position_.timestamp) {
        return false;
    }
    // 内环虽然只用速度、航向和 yaw-rate，也必须复核外环所依赖的完整全球
    // NED 闭包；这样 xy_global/ref 或位置非有限在 100 Hz 当周期就失效，而不是
    // 等待下一条 50 Hz Navigation 请求才停止沿用旧控制量。
    return finite(vehicle_local_position_.x) &&
        finite(vehicle_local_position_.y) &&
        finite(vehicle_local_position_.vx) &&
        finite(vehicle_local_position_.vy) &&
        finite(vehicle_local_position_.heading) &&
        std::isfinite(vehicle_local_position_.ref_lat) &&
        std::isfinite(vehicle_local_position_.ref_lon) &&
        vehicle_local_position_.ref_lat >= -90.0 &&
        vehicle_local_position_.ref_lat <= 90.0 &&
        vehicle_local_position_.ref_lon >= -180.0 &&
        vehicle_local_position_.ref_lon <= 180.0 &&
        finite(vehicle_odometry_.angular_velocity[2]);
}

bool RoverDifferential::estimator_reset_detected() noexcept
{
    const bool reset = !have_estimator_baseline_ ||
        vehicle_local_position_.timestamp <
            last_local_position_timestamp_us_ ||
        vehicle_local_position_.timestamp_sample <
            last_local_position_sample_us_ ||
        vehicle_odometry_.timestamp < last_odometry_timestamp_us_ ||
        vehicle_odometry_.timestamp_sample < last_odometry_sample_us_ ||
        vehicle_local_position_.ref_timestamp !=
            local_reference_timestamp_us_ ||
        vehicle_local_position_.xy_reset_counter !=
            local_position_reset_counter_ ||
        vehicle_local_position_.vxy_reset_counter !=
            local_velocity_reset_counter_ ||
        vehicle_local_position_.heading_reset_counter !=
            local_heading_reset_counter_ ||
        vehicle_odometry_.reset_counter != odometry_reset_counter_;

    last_local_position_timestamp_us_ = vehicle_local_position_.timestamp;
    last_local_position_sample_us_ =
        vehicle_local_position_.timestamp_sample;
    last_odometry_timestamp_us_ = vehicle_odometry_.timestamp;
    last_odometry_sample_us_ = vehicle_odometry_.timestamp_sample;
    local_reference_timestamp_us_ = vehicle_local_position_.ref_timestamp;
    local_position_reset_counter_ =
        vehicle_local_position_.xy_reset_counter;
    local_velocity_reset_counter_ =
        vehicle_local_position_.vxy_reset_counter;
    local_heading_reset_counter_ =
        vehicle_local_position_.heading_reset_counter;
    odometry_reset_counter_ = vehicle_odometry_.reset_counter;
    have_estimator_baseline_ = true;
    return reset;
}

bool RoverDifferential::publish_output(std::uint64_t now_us,
                                       float dt_s) noexcept
{
    // 参数、安全或输入任一合同失败都重置 slew/reversal/ramp 内部状态并发布
    // 全 NaN；不能仅把输出夹到零，否则下游会把零当作仍然有效的控制命令。
    const rover_motion_request_s *request = active_request();
    const bool navigation_source = safety_.valid &&
        (safety_.vehicle_status.nav_state ==
             vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
         safety_.vehicle_status.nav_state ==
             vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
    const std::uint64_t request_sample =
        request != nullptr ? request->timestamp_sample : 0U;
    if (!parameters_valid_ || !safety_permits_output(now_us) ||
        request == nullptr ||
        !request_valid(*request, now_us, navigation_source)) {
        drive_.reset();
        reset_navigation_control();
        return publish_invalid(now_us, request_sample);
    }

    float longitudinal = request->normalized_longitudinal;
    float steering = request->normalized_steering;
    std::uint64_t sample_time_us = request_sample;
    if (navigation_source) {
        if (!navigation_estimator_valid(now_us)) {
            drive_.reset();
            reset_navigation_control();
            return publish_invalid(now_us, request_sample);
        }
        if (estimator_reset_detected()) {
            // 位置、速度、航向或 odometry reset 后固定空一帧；两只 PI 的积分与
            // slew 同时清零，禁止新旧估计代际混算。
            drive_.reset();
            speed_controller_.reset();
            yaw_rate_controller_.reset();
            return publish_invalid(now_us, request_sample);
        }

        const auto measured_speed = dima::lib::rover::measure_body_speed(
            vehicle_local_position_.vx, vehicle_local_position_.vy,
            vehicle_local_position_.heading,
            parameters_.speed.measurement_threshold_m_s);
        if (!measured_speed.valid) {
            drive_.reset();
            reset_navigation_control();
            return publish_invalid(now_us, request_sample);
        }

        // 转向优先：先由 YawRate PI（含差速运动学 FF）算 steering，再把速度
        // PI 的可用归一化范围限制为 1-|steering|。这样左右轮和在进入下游
        // DifferentialDrive 前已可行，避免二次饱和破坏航向内环。
        const auto yaw_rate = yaw_rate_controller_.update(
            request->yaw_rate_rad_s,
            vehicle_odometry_.angular_velocity[2], 1.0F, dt_s);
        if (!yaw_rate.valid || !normalized(yaw_rate.output)) {
            drive_.reset();
            reset_navigation_control();
            return publish_invalid(now_us, request_sample);
        }
        steering = yaw_rate.output;
        const float longitudinal_limit =
            std::fmax(0.0F, 1.0F - std::fabs(steering));
        const auto speed = speed_controller_.update(
            request->speed_m_s, measured_speed.speed_m_s,
            longitudinal_limit, dt_s);
        if (!speed.valid || !normalized(speed.output)) {
            drive_.reset();
            reset_navigation_control();
            return publish_invalid(now_us, request_sample);
        }
        longitudinal = speed.output;
        sample_time_us = request_sample;
        if (vehicle_local_position_.timestamp_sample < sample_time_us) {
            sample_time_us = vehicle_local_position_.timestamp_sample;
        }
        if (vehicle_odometry_.timestamp_sample < sample_time_us) {
            sample_time_us = vehicle_odometry_.timestamp_sample;
        }
    } else {
        // Manual 保持原来的归一化双轴与倒车转向修正；切离 AUTO 时立即清空
        // 两只 PI，下一次 Mission Start 不得继承旧积分或 slew 状态。
        reset_navigation_control();
    }

    const dima::lib::rover::DifferentialDriveOutput output = drive_.update(
        longitudinal, steering, !navigation_source, true, now_us, dt_s);
    if (!output.valid || !normalized(output.right) || !normalized(output.left)) {
        drive_.reset();
        reset_navigation_control();
        return publish_invalid(now_us, sample_time_us);
    }

    actuator_motors_s motors{};
    motors.timestamp = now_us;
    motors.timestamp_sample = sample_time_us;
    motors.reversible_flags = kReversibleMotorMask;
    // 未实现的电机槽用 NaN 明确表示不可用，不能填 0 伪装成有效零油门命令。
    for (float &control : motors.control) {
        control = kUnavailable;
    }
    motors.control[0] = output.right;
    motors.control[1] = output.left;
    return actuator_motors_publication_.publish(motors);
}

bool RoverDifferential::publish_invalid(std::uint64_t now_us,
                                        std::uint64_t sample_time_us) noexcept
{
    actuator_motors_s motors{};
    motors.timestamp = now_us;
    motors.timestamp_sample = sample_time_us;
    motors.reversible_flags = kReversibleMotorMask;
    for (float &control : motors.control) {
        control = kUnavailable;
    }
    return actuator_motors_publication_.publish(motors);
}

void RoverDifferential::reset_runtime_state() noexcept
{
    drive_.reset();
    reset_navigation_control();
    parameters_ = ParameterSnapshot{};
    manual_request_ = rover_motion_request_s{};
    navigation_request_ = rover_motion_request_s{};
    vehicle_local_position_ = vehicle_local_position_s{};
    vehicle_odometry_ = vehicle_odometry_s{};
    observed_actuator_armed_ = actuator_armed_s{};
    observed_control_mode_ = vehicle_control_mode_s{};
    observed_vehicle_status_ = vehicle_status_s{};
    safety_ = SafetySnapshot{};
    last_run_time_us_ = 0U;
    have_manual_request_ = false;
    have_navigation_request_ = false;
    have_local_position_ = false;
    have_odometry_ = false;
    parameters_valid_ = false;
    navigation_parameters_valid_ = false;
    parameter_update_pending_ = false;
    safety_inhibit_observed_ = true;
    invalidate_parameter_bindings();
}

void RoverDifferential::reset_navigation_control() noexcept
{
    speed_controller_.reset();
    yaw_rate_controller_.reset();
    last_local_position_timestamp_us_ = 0U;
    last_local_position_sample_us_ = 0U;
    last_odometry_timestamp_us_ = 0U;
    last_odometry_sample_us_ = 0U;
    local_reference_timestamp_us_ = 0U;
    local_position_reset_counter_ = 0U;
    local_velocity_reset_counter_ = 0U;
    local_heading_reset_counter_ = 0U;
    odometry_reset_counter_ = 0U;
    have_estimator_baseline_ = false;
}

void RoverDifferential::enter_error(std::uint32_t event_id) noexcept
{
    drive_.reset();
    reset_navigation_control();
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    ScheduleCancelAndDrain();
    (void)dima::events::report(event_id, dima::events::Severity::Error);
}

bool RoverDifferential::finite(float value) noexcept
{
    return std::isfinite(value);
}

bool RoverDifferential::normalized(float value) noexcept
{
    return finite(value) && value >= -1.0F && value <= 1.0F;
}

bool RoverDifferential::valid_parameter_snapshot(
    const ParameterSnapshot &snapshot) noexcept
{
    // 此处复核参数元数据之外的运行时合同：timeout 单位 s，slew 单位 1/s，
    // reversal_delay/arm_ramp 单位 s，并保证 throttle_min <= throttle_max。
    const dima::lib::rover::DifferentialDriveConfig &drive = snapshot.drive;
    return finite(snapshot.command_timeout_s) &&
           snapshot.command_timeout_s >= 0.02F &&
           snapshot.command_timeout_s <= 1.0F &&
           finite(drive.steering_throttle_mix) &&
           drive.steering_throttle_mix >= 0.0F &&
           drive.steering_throttle_mix <= 1.0F && finite(drive.throttle_min) &&
           drive.throttle_min >= 0.0F && finite(drive.throttle_max) &&
           drive.throttle_max >= 0.05F && drive.throttle_max <= 1.0F &&
           drive.throttle_min <= drive.throttle_max &&
           finite(drive.throttle_slew_rate) &&
           drive.throttle_slew_rate >= 0.0F &&
           drive.throttle_slew_rate <= 10.0F &&
           finite(drive.reversal_delay_s) &&
           drive.reversal_delay_s >= 0.0F &&
           drive.reversal_delay_s <= 1.0F && finite(drive.throttle_expo) &&
           drive.throttle_expo >= -1.0F && drive.throttle_expo <= 1.0F &&
           finite(drive.thrust_asymmetry) &&
           drive.thrust_asymmetry >= 1.0F &&
           drive.thrust_asymmetry <= 10.0F && finite(drive.arm_ramp_s) &&
           drive.arm_ramp_s >= 0.0F && drive.arm_ramp_s <= 5.0F;
}

bool RoverDifferential::valid_navigation_parameter_snapshot(
    const ParameterSnapshot &snapshot) noexcept
{
    const auto &speed = snapshot.speed;
    const auto &yaw_rate = snapshot.yaw_rate;
    // AUTO 使用的车辆专属量没有凭空默认值：最大速度、轮距、两组闭环增益与
    // 线/角加减速度必须完成实车标定后才允许 Navigation 请求进入混控；速度和
    // yaw-rate 测量死区还必须严格小于对应物理上限，防止整个闭环范围被判成零。
    return finite(speed.proportional_gain) && speed.proportional_gain >= 0.0F &&
        finite(speed.integral_gain) && speed.integral_gain >= 0.0F &&
        speed.proportional_gain + speed.integral_gain > 0.0F &&
        finite(speed.speed_at_full_throttle_m_s) &&
        speed.speed_at_full_throttle_m_s > 0.0F &&
        finite(speed.acceleration_limit_m_s2) &&
        speed.acceleration_limit_m_s2 > 0.0F &&
        finite(speed.deceleration_limit_m_s2) &&
        speed.deceleration_limit_m_s2 > 0.0F &&
        finite(speed.measurement_threshold_m_s) &&
        speed.measurement_threshold_m_s >= 0.0F &&
        speed.measurement_threshold_m_s <
            speed.speed_at_full_throttle_m_s &&
        finite(yaw_rate.proportional_gain) &&
        yaw_rate.proportional_gain >= 0.0F &&
        finite(yaw_rate.integral_gain) && yaw_rate.integral_gain >= 0.0F &&
        yaw_rate.proportional_gain + yaw_rate.integral_gain > 0.0F &&
        finite(yaw_rate.yaw_rate_correction) &&
        yaw_rate.yaw_rate_correction > 0.0F &&
        finite(yaw_rate.wheel_track_m) && yaw_rate.wheel_track_m > 0.0F &&
        finite(yaw_rate.speed_at_full_throttle_m_s) &&
        yaw_rate.speed_at_full_throttle_m_s > 0.0F &&
        finite(yaw_rate.yaw_rate_limit_rad_s) &&
        yaw_rate.yaw_rate_limit_rad_s > 0.0F &&
        finite(yaw_rate.yaw_acceleration_limit_rad_s2) &&
        yaw_rate.yaw_acceleration_limit_rad_s2 > 0.0F &&
        finite(yaw_rate.yaw_deceleration_limit_rad_s2) &&
        yaw_rate.yaw_deceleration_limit_rad_s2 > 0.0F &&
        finite(yaw_rate.measurement_threshold_rad_s) &&
        yaw_rate.measurement_threshold_rad_s >= 0.0F &&
        yaw_rate.measurement_threshold_rad_s <
            yaw_rate.yaw_rate_limit_rad_s;
}

bool RoverDifferential::manual_projection(
    const vehicle_control_mode_s &control,
    const vehicle_status_s &status) noexcept
{
    return status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
        exact_manual_control_projection(control);
}

bool RoverDifferential::navigation_projection(
    const vehicle_control_mode_s &control,
    const vehicle_status_s &status) noexcept
{
    const bool supported =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return supported && control.source_id == status.nav_state &&
        exact_navigation_control_projection(control);
}

bool RoverDifferential::safety_negative(
    const actuator_armed_s &armed) noexcept
{
    return armed.timestamp != 0U &&
           (!armed.armed || armed.kill || armed.termination || armed.lockdown);
}

bool RoverDifferential::safety_negative(
    const vehicle_control_mode_s &control) noexcept
{
    if (control.timestamp == 0U) {
        return false;
    }
    // 任一新 control Topic 即使尚未与另外两项组成同拍快照，只要不是精确
    // Manual/AUTO 投影就立即锁止本层输出；不能依赖 MotorOutput 再兜底异常 flag。
    const bool manual = exact_manual_control_projection(control);
    const bool navigation = exact_navigation_control_projection(control);
    return !control.flag_armed || (!manual && !navigation);
}

bool RoverDifferential::safety_negative(const vehicle_status_s &status) noexcept
{
    if (status.timestamp == 0U) {
        return false;
    }
    const bool supported =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
    return status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
           !supported ||
           status.valid_nav_states_mask != kImplementedModeMask ||
           status.can_set_nav_states_mask != kManualModeMask ||
           status.failsafe;
}

} // namespace dima::rover::control

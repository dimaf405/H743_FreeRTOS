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
constexpr float kUnavailable = std::numeric_limits<float>::quiet_NaN();

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
    (void)publish_invalid(hrt_absolute_time(), motion_request_.timestamp_sample);
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

    // motion_request 是有界队列；单轮最多排空队列深度并只保留最新请求，既避免
    // 旧操纵量累积执行，也保证本 work item 不会因异常积压而无界占用 CPU。
    rover_motion_request_s request{};
    for (std::uint8_t count = 0U;
         count < kMotionRequestQueueDepth &&
         motion_request_subscription_.copy(&request);
         ++count) {
        motion_request_ = request;
        have_motion_request_ = true;
    }

    const std::uint64_t now = hrt_absolute_time();
    refresh_safety_snapshot(now);
    (void)apply_pending_parameters(now);

    // 斜率限制、换向延时和解锁渐入均依赖真实运行间隔；时钟回退时 dt 无定义，
    // 因而必须清空控制器并 fail-closed，不能退回名义周期继续输出。
    if (last_run_time_us_ != 0U && now < last_run_time_us_) {
        drive_.reset();
        (void)publish_invalid(now, motion_request_.timestamp_sample);
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
                       thrust_asymmetry_.bind() && arm_ramp_.bind();
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
}

bool RoverDifferential::apply_parameter_snapshot() noexcept
{
    if (!command_timeout_.bound() || !reverse_steering_.bound() ||
        !steering_throttle_mix_.bound() || !throttle_min_.bound() ||
        !throttle_max_.bound() || !throttle_slew_rate_.bound() ||
        !reversal_delay_.bound() || !throttle_expo_.bound() ||
        !thrust_asymmetry_.bound() || !arm_ramp_.bound()) {
        parameters_valid_ = false;
        return false;
    }

    // 原子读取完整候选快照，只有全量校验和控制器配置均成功后才替换生效参数。
    // 原子读取整组候选快照；只有全部读取、交叉约束和控制器配置均成功，才一次
    // 性替换当前有效参数，避免一次 PARAM_SET 让控制器看到跨代混合配置。
    ParameterSnapshot candidate{};
    std::int32_t reverse_steering = 0;
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
                           &candidate.drive.arm_ramp_s) == 0;
    }
    if (!loaded) {
        parameters_valid_ = false;
        return false;
    }
    candidate.drive.reverse_steering_in_manual = reverse_steering != 0;

    if ((reverse_steering != 0 && reverse_steering != 1) ||
        !valid_parameter_snapshot(candidate) ||
        !drive_.configure(candidate.drive)) {
        parameters_valid_ = false;
        drive_.reset();
        return false;
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
    // 任一 Topic 先观察到否定安全状态就立即锁存抑制；只有三项 Topic 以同一时间戳
    // 完整收敛后，才用这一批 Commander 安全快照重新计算是否允许驱动。
    if (actuator_armed_subscription_.update()) {
        observed_actuator_armed_ = actuator_armed_subscription_.get();
        if (safety_negative(observed_actuator_armed_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_control_mode_subscription_.update()) {
        observed_control_mode_ = vehicle_control_mode_subscription_.get();
        if (safety_negative(observed_control_mode_)) {
            safety_inhibit_observed_ = true;
        }
    }
    if (vehicle_status_subscription_.update()) {
        observed_vehicle_status_ = vehicle_status_subscription_.get();
        if (safety_negative(observed_vehicle_status_)) {
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
           timestamp <= now_us;
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
    // 任一新 Topic 出现负向安全证据就立即锁止；恢复则必须等待三份同拍、鲜活且
    // 字段互相一致的 Manual+Armed 快照，不能靠较旧的正向快照抵消新故障。
    if (safety_inhibit_observed_ || !active_snapshot_fresh(now_us)) {
        return false;
    }

    const actuator_armed_s &armed = safety_.actuator_armed;
    const vehicle_control_mode_s &control = safety_.control_mode;
    const vehicle_status_s &status = safety_.vehicle_status;
    return armed.armed && armed.ready_to_arm && !armed.prearmed &&
           !armed.lockdown && !armed.kill && !armed.termination &&
           !armed.in_esc_calibration_mode && control.flag_armed &&
           control.flag_control_manual_enabled &&
           !control.flag_control_termination_enabled &&
           !control.flag_control_auto_enabled &&
           !control.flag_control_offboard_enabled &&
           !control.flag_control_position_enabled &&
           !control.flag_control_velocity_enabled &&
           !control.flag_control_attitude_enabled &&
           !control.flag_control_rates_enabled &&
           !control.flag_control_allocation_enabled &&
           control.source_id == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
           status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
           !status.failsafe;
}

bool RoverDifferential::request_valid(std::uint64_t now_us) const noexcept
{
    if (!have_motion_request_ || !motion_request_.valid ||
        motion_request_.source != rover_motion_request_s::SOURCE_MANUAL ||
        motion_request_.mode !=
            rover_motion_request_s::MODE_NORMALIZED_AXES ||
        motion_request_.timestamp == 0U ||
        motion_request_.timestamp_sample == 0U ||
        motion_request_.timestamp_sample > motion_request_.timestamp ||
        motion_request_.timestamp > now_us ||
        motion_request_.timestamp_sample > now_us ||
        !normalized(motion_request_.normalized_longitudinal) ||
        !normalized(motion_request_.normalized_steering)) {
        return false;
    }

    // timestamp 约束处理链路新鲜度，timestamp_sample 约束原始 RC 样本新鲜度；
    // 两者都在超时内才能避免“刚转发的旧摇杆样本”被误当作新命令。
    // timeout_us = command_timeout_s * 1e6。timestamp 约束模块链路鲜度，
    // timestamp_sample 约束原始 RC 样本鲜度；两者都在窗口内才能避免刚转发的
    // 旧摇杆样本重新激活电机命令。
    const std::uint64_t timeout_us = static_cast<std::uint64_t>(
        parameters_.command_timeout_s * 1000000.0F);
    return timeout_us > 0U &&
           now_us - motion_request_.timestamp <= timeout_us &&
           now_us - motion_request_.timestamp_sample <= timeout_us;
}

bool RoverDifferential::publish_output(std::uint64_t now_us,
                                       float dt_s) noexcept
{
    // 参数、安全或输入任一合同失败都重置 slew/reversal/ramp 内部状态并发布
    // 全 NaN；不能仅把输出夹到零，否则下游会把零当作仍然有效的控制命令。
    if (!parameters_valid_ || !safety_permits_output(now_us) ||
        !request_valid(now_us)) {
        drive_.reset();
        return publish_invalid(now_us, motion_request_.timestamp_sample);
    }

    const dima::lib::rover::DifferentialDriveOutput output = drive_.update(
        motion_request_.normalized_longitudinal,
        motion_request_.normalized_steering, true, true, now_us, dt_s);
    if (!output.valid || !normalized(output.right) || !normalized(output.left)) {
        drive_.reset();
        return publish_invalid(now_us, motion_request_.timestamp_sample);
    }

    actuator_motors_s motors{};
    motors.timestamp = now_us;
    motors.timestamp_sample = motion_request_.timestamp_sample;
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
    parameters_ = ParameterSnapshot{};
    motion_request_ = rover_motion_request_s{};
    observed_actuator_armed_ = actuator_armed_s{};
    observed_control_mode_ = vehicle_control_mode_s{};
    observed_vehicle_status_ = vehicle_status_s{};
    safety_ = SafetySnapshot{};
    last_run_time_us_ = 0U;
    have_motion_request_ = false;
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    safety_inhibit_observed_ = true;
    invalidate_parameter_bindings();
}

void RoverDifferential::enter_error(std::uint32_t event_id) noexcept
{
    drive_.reset();
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

bool RoverDifferential::safety_negative(
    const actuator_armed_s &armed) noexcept
{
    return armed.timestamp != 0U &&
           (!armed.armed || armed.kill || armed.termination || armed.lockdown);
}

bool RoverDifferential::safety_negative(
    const vehicle_control_mode_s &control) noexcept
{
    return control.timestamp != 0U &&
           (!control.flag_armed || !control.flag_control_manual_enabled ||
            control.flag_control_termination_enabled);
}

bool RoverDifferential::safety_negative(const vehicle_status_s &status) noexcept
{
    return status.timestamp != 0U &&
           (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
            status.nav_state != vehicle_status_s::NAVIGATION_STATE_MANUAL ||
            status.failsafe);
}

} // namespace dima::rover::control

#include "MotorOutput.hpp"

#include "events/events.hpp"
#include "api/Time.hpp"

namespace dima::modules::motor {
namespace {

constexpr std::uint32_t kEventParameterInvalid = 0x524D4F01U;
constexpr std::uint32_t kEventBackendFault = 0x524D4F02U;
constexpr std::uint32_t kEventPublishFailure = 0x524D4F03U;
constexpr std::uint32_t kEventScheduleFailure = 0x524D4F04U;

static_assert(dima::platform::kActuatorPwmChannelCount ==
              actuator_output_status_s::NUM_OUTPUTS);

} // namespace

MotorOutput::MotorOutput(dima::platform::ActuatorPwm *pwm) noexcept
    : px4::ScheduledWorkItem("motor_output", px4::wq_configurations::io),
      pwm_(pwm)
{
    invalidate_parameter_bindings();
}

MotorOutput::~MotorOutput()
{
    stop();
}

bool MotorOutput::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    reset_runtime_state();
    if (pwm_ == nullptr) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    // 启动调度器和读取参数以前先让后端进入物理 Safe Off；若无法确认停波，
    // 模块不得以 Running 状态对外提供任何输出能力。
    const dima::platform::ActuatorPwmResult stopped = pwm_->stop();
    if (stopped != dima::platform::ActuatorPwmResult::Applied) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    backend_ready_ = true;
    safe_off_ = true;
    if (!ScheduleEnable()) {
        enter_error(kEventScheduleFailure);
        return false;
    }

    if (!bind_parameters()) {
        enter_error(kEventParameterInvalid);
        return false;
    }

    const bool parameter_snapshot_valid = apply_parameter_snapshot();
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    const std::uint64_t now = hrt_absolute_time();
    if (!parameter_snapshot_valid) {
        if (!enter_parameter_safe_off()) {
            return false;
        }
    } else if (!publish_status(
                   now, actuator_output_status_s::STATE_SAFE_OFF, false)) {
        enter_error(kEventPublishFailure);
        return false;
    }
    if (!ScheduleOnInterval(kRunIntervalUs, 1U)) {
        enter_error(kEventScheduleFailure);
        return false;
    }
    return true;
}

void MotorOutput::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();

    const dima::platform::ActuatorPwmResult result =
        pwm_ != nullptr ? pwm_->stop()
                        : dima::platform::ActuatorPwmResult::Fault;
    backend_ready_ = result == dima::platform::ActuatorPwmResult::Applied;
    safe_off_ = backend_ready_;
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    invalidate_parameter_bindings();
    if (!backend_ready_) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
    }
    (void)publish_status(hrt_absolute_time(),
                         backend_ready_
                             ? actuator_output_status_s::STATE_STOPPED
                             : actuator_output_status_s::STATE_FAULT,
                         false);
}

dima::middleware::lifecycle::ModuleState MotorOutput::state() const
{
    return state_;
}

bool MotorOutput::safe_off_confirmed() const noexcept
{
    // BootHealth 使用这条独立证据判断电机链是否真正停波，不能只依赖本模块状态枚举。
    return backend_ready_ && safe_off_ &&
           (pwm_ == nullptr || !pwm_->started());
}

bool MotorOutput::drive_available() const noexcept
{
    return parameters_valid_ && parameters_.drive_available;
}

void MotorOutput::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    parameter_update_s update{};
    if (parameter_update_subscription_.copy(&update)) {
        parameter_update_pending_ = true;
    }
    if (actuator_motors_subscription_.update()) {
        actuator_motors_ = actuator_motors_subscription_.get();
        have_motor_command_ = true;
    }

    const std::uint64_t now = hrt_absolute_time();
    refresh_safety_snapshot(now);
    // PWM 映射与范围只能在同时间戳、完整且新鲜的 Disarmed 安全快照后整体切换，
    // 防止 Armed 或安全 Topic 尚未收敛时应用半套新参数。
    if (parameter_update_pending_ && fresh_disarmed_snapshot(now)) {
        parameter_update_pending_ = false;
        if (!apply_parameter_snapshot()) {
            (void)enter_parameter_safe_off();
            return;
        }
    }

    const bool output_permitted = parameters_valid_ &&
        parameters_.drive_available && safety_permits_output(now);
    const bool command_valid = motor_command_valid(now);
    const bool control_inhibited = output_permitted && !command_valid &&
        motor_control_inhibit_valid(now);
    const bool active_output = output_permitted && command_valid;
    const bool disarmed_neutral = !active_output &&
        safety_permits_disarmed_neutral(now);

    // 输出策略：Active 写入受控波形；Disarmed Neutral 保持可配置的中位；
    // 新鲜的精确失效帧停波后标记 Control Inhibited；其余场景执行 Hard Safe Off。
    // 后端暂不可用时只允许进入 Retry，绝不沿用旧帧。
    if (!active_output && !disarmed_neutral) {
        const dima::platform::ActuatorPwmResult result = force_safe_off();
        if (result == dima::platform::ActuatorPwmResult::Fault) {
            enter_error(kEventBackendFault);
            return;
        }
        const std::uint8_t output_state =
            result == dima::platform::ActuatorPwmResult::Applied
                ? (control_inhibited
                       ? actuator_output_status_s::STATE_CONTROL_INHIBITED
                       : actuator_output_status_s::STATE_HARD_SAFE_OFF)
                : actuator_output_status_s::STATE_RETRY;
        if (!publish_status(now, output_state, command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }

    dima::platform::ActuatorPwmFrame frame{};
    if (!(active_output ? build_frame(frame) : build_neutral_frame(frame))) {
        enter_error(kEventParameterInvalid);
        return;
    }
    const dima::platform::ActuatorPwmResult write_result = apply_frame(frame);
    if (write_result == dima::platform::ActuatorPwmResult::Applied) {
        const std::uint8_t output_state = active_output
            ? actuator_output_status_s::STATE_ACTIVE
            : actuator_output_status_s::STATE_DISARMED_NEUTRAL;
        if (!publish_status(now, output_state,
                            command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }
    if (write_result == dima::platform::ActuatorPwmResult::Retry) {
        // 写波形未成功时立即尝试停波；只有后端明确返回 Applied 才能重新确认 Safe Off。
        const dima::platform::ActuatorPwmResult stopped = force_safe_off();
        if (stopped == dima::platform::ActuatorPwmResult::Fault) {
            enter_error(kEventBackendFault);
            return;
        }
        if (!publish_status(now, actuator_output_status_s::STATE_RETRY,
                            command_valid)) {
            enter_error(kEventPublishFailure);
        }
        return;
    }
    enter_error(kEventBackendFault);
}

void MotorOutput::reset_runtime_state() noexcept
{
    parameters_ = ParameterSnapshot{};
    actuator_motors_ = actuator_motors_s{};
    observed_actuator_armed_ = actuator_armed_s{};
    observed_control_mode_ = vehicle_control_mode_s{};
    observed_vehicle_status_ = vehicle_status_s{};
    safety_ = SafetySnapshot{};
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    status_sequence_ = 0U;
    have_motor_command_ = false;
    parameters_valid_ = false;
    parameter_update_pending_ = false;
    safety_inhibit_observed_ = true;
    hard_safe_inhibit_observed_ = true;
    backend_ready_ = false;
    safe_off_ = false;
    invalidate_parameter_bindings();
}

bool MotorOutput::enter_parameter_safe_off() noexcept
{
    parameters_valid_ = false;
    const dima::platform::ActuatorPwmResult stopped = force_safe_off();
    if (stopped == dima::platform::ActuatorPwmResult::Fault) {
        enter_error(kEventBackendFault);
        return false;
    }

    const std::uint8_t output_state =
        stopped == dima::platform::ActuatorPwmResult::Applied
            ? actuator_output_status_s::STATE_HARD_SAFE_OFF
            : actuator_output_status_s::STATE_RETRY;
    if (!publish_status(hrt_absolute_time(), output_state, false)) {
        enter_error(kEventPublishFailure);
        return false;
    }

    (void)dima::events::report(kEventParameterInvalid,
                               dima::events::Severity::Error);
    return true;
}

void MotorOutput::enter_error(std::uint32_t event_id) noexcept
{
    // 故障路径先取消后续调度，再强制停波并发布 FAULT，避免错误状态仍残留最后一帧 PWM。
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    ScheduleCancelAndDrain();
    const dima::platform::ActuatorPwmResult stopped = force_safe_off();
    if (stopped != dima::platform::ActuatorPwmResult::Applied) {
        backend_ready_ = false;
        safe_off_ = false;
    }
    (void)publish_status(hrt_absolute_time(),
                         actuator_output_status_s::STATE_FAULT, false);
    (void)dima::events::report(event_id, dima::events::Severity::Error);
}

} // namespace dima::modules::motor

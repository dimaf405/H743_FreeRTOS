#include "BootHealthService.hpp"

#include "api/ActuatorPwm.hpp"

#include "parameters/param.h"

namespace dima::modules::boot_health {

BootHealthService::BootHealthService(
    dima::platform::BootControl &boot_control,
    dima::platform::MonotonicClock &clock,
    dima::middleware::maintenance::
        RuntimeMaintenanceCoordinator &maintenance) noexcept
    : px4::ScheduledWorkItem("boot_health", px4::wq_configurations::hp_default),
      boot_control_(boot_control), clock_(clock), maintenance_(maintenance)
{
}

bool BootHealthService::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    // 每次启动都丢弃上一次 Runtime 的确认窗口、序号和活性证据。
    stable_window_start_ms_ = clock_.now_ms();
    safety_snapshot_observed_ = false;
    output_snapshot_observed_ = false;
    stable_window_active_ = false;
    last_safety_timestamp_us_ = 0U;
    last_output_sequence_ = 0U;
    // 丢弃 Commander 启动时的静态快照；必须观察下一组严格前进的
    // 三 Topic 快照后，连续健康窗口才允许开始。
    (void)actuator_armed_subscription_.update();
    (void)vehicle_control_mode_subscription_.update();
    (void)vehicle_status_subscription_.update();
    (void)actuator_output_status_subscription_.update();
    const bool scheduled = ScheduleOnInterval(kCheckIntervalMs * 1000U);
    if (!scheduled) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void BootHealthService::stop()
{
    ScheduleCancelAndDrain();
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    stable_window_active_ = false;
    stable_window_start_ms_ = 0U;
    safety_snapshot_observed_ = false;
    output_snapshot_observed_ = false;
    last_safety_timestamp_us_ = 0U;
    last_output_sequence_ = 0U;
}

dima::middleware::lifecycle::ModuleState BootHealthService::state() const
{
    return state_;
}

std::uint32_t BootHealthService::health_generation() const noexcept
{
    return __atomic_load_n(&health_generation_, __ATOMIC_ACQUIRE);
}

void BootHealthService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    const std::uint64_t now_us = clock_.now_us();
    const std::uint64_t now_ms = clock_.now_ms();

    // watchdog 健康与 MCUboot 确认分层：前者允许 Armed/Active，只要输出与
    // 安全状态一致；后者必须 Disarmed 且输出处于确认安全态。
    const bool safety_healthy = update_safety_health(now_us);
    const bool output_healthy = update_output_health(now_us);
    const bool runtime_healthy =
        param_is_ready() && safety_healthy && output_healthy;
    const bool maintenance_safe =
        runtime_healthy && confirmation_state_safe() &&
        output_status_confirmation_safe();
    // 维护协调器在此统一仲裁 Flash/参数/驱动重配置；返回 false 时本轮既不推进
    // generation，也不累计镜像确认窗口。
    if (!maintenance_.boot_health_update(
            now_us, runtime_healthy, maintenance_safe)) {
        reset_stable_window(now_ms);
        return;
    }

    // 每次完整 Runtime 健康检查只发行一代票据。uint32 回绕到 0 时再前进一步，
    // 因为 0 保留为“尚无健康证明”，appMain 不会把它当作新代。
    std::uint32_t generation = __atomic_add_fetch(
        &health_generation_, 1U, __ATOMIC_RELEASE);
    if (generation == 0U) {
        (void)__atomic_add_fetch(&health_generation_, 1U, __ATOMIC_RELEASE);
    }

    if (confirmation_attempted_ || maintenance_.in_progress()) {
        reset_stable_window(now_ms);
        return;
    }

    if (!confirmation_state_safe() ||
        !output_status_confirmation_safe()) {
        reset_stable_window(now_ms);
        return;
    }

    // 确认窗口从第一次满足“运行健康 + Disarmed + 输出安全”开始，任一轮不满足
    // 都在上方 reset；累计满 5 s 后才写 MCUboot image_ok。
    if (!stable_window_active_) {
        stable_window_active_ = true;
        stable_window_start_ms_ = now_ms;
        return;
    }

    if (now_ms - stable_window_start_ms_ < kStableWindowMs) {
        return;
    }

    const dima::platform::BootConfirmResult result =
        boot_control_.confirm_running_image();
    switch (result) {
    case dima::platform::BootConfirmResult::Ok:
    case dima::platform::BootConfirmResult::AlreadyConfirmed:
    case dima::platform::BootConfirmResult::NotTestImage:
        confirmation_attempted_ = true;
        break;
    case dima::platform::BootConfirmResult::Deferred:
        break;
    case dima::platform::BootConfirmResult::FlashError:
    default:
        confirmation_attempted_ = true;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        break;
    }
}

bool BootHealthService::update_safety_health(std::uint64_t now_us) noexcept
{
    const bool armed_updated = actuator_armed_subscription_.update();
    const bool control_updated = vehicle_control_mode_subscription_.update();
    const bool status_updated = vehicle_status_subscription_.update();
    // Commander 用同一 timestamp 连续发布三 Topic。只更新其中一项表示读到了
    // 撕裂批次，不能把新旧状态拼成一份安全证明。
    const bool any_updated = armed_updated || control_updated || status_updated;
    const bool all_updated = armed_updated && control_updated && status_updated;

    if (!safety_topics_consistent(now_us)) {
        safety_snapshot_observed_ = false;
        return false;
    }

    const std::uint64_t timestamp =
        actuator_armed_subscription_.get().timestamp;
    if (any_updated) {
        if (!all_updated ||
            (last_safety_timestamp_us_ != 0U &&
             timestamp <= last_safety_timestamp_us_)) {
            safety_snapshot_observed_ = false;
            return false;
        }
        last_safety_timestamp_us_ = timestamp;
        safety_snapshot_observed_ = true;
    }

    return safety_snapshot_observed_ &&
           last_safety_timestamp_us_ <= now_us &&
           now_us - last_safety_timestamp_us_ <= kSafetyTopicTimeoutUs;
}

bool BootHealthService::safety_topics_consistent(
    std::uint64_t now_us) const noexcept
{
    const actuator_armed_s &armed = actuator_armed_subscription_.get();
    const vehicle_control_mode_s &control =
        vehicle_control_mode_subscription_.get();
    const vehicle_status_s &status = vehicle_status_subscription_.get();

    // timestamp 相等先证明同批；随后逐字段交叉验证 armed/control/status 是
    // 同一状态机投影，而不是三个各自“看起来合理”的快照。
    if (armed.timestamp == 0U || armed.timestamp != control.timestamp ||
        armed.timestamp != status.timestamp || armed.timestamp > now_us ||
        now_us - armed.timestamp > kSafetyTopicTimeoutUs) {
        return false;
    }

    const bool status_armed =
        status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    const bool status_disarmed =
        status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED;
    const bool manual =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const bool termination =
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION;
    // 本 Rover 运行时只承认 Manual 与 Termination；can_set 只允许主动进入 Manual，
    // Termination 必须由安全故障路径触发。
    const std::uint32_t manual_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_MANUAL;
    const std::uint32_t termination_mask =
        1UL << vehicle_status_s::NAVIGATION_STATE_TERMINATION;

    return (status_armed || status_disarmed) &&
           armed.armed == status_armed && control.flag_armed == armed.armed &&
           (!armed.kill || status_disarmed) &&
           armed.ready_to_arm ==
               (status.pre_flight_checks_pass || armed.armed) &&
           !armed.prearmed && !armed.lockdown &&
           !armed.in_esc_calibration_mode &&
           armed.termination == termination &&
           (!termination || status.failsafe) && (manual || termination) &&
           control.flag_control_manual_enabled == manual &&
           control.flag_control_termination_enabled == termination &&
           control.source_id == status.nav_state &&
           !control.flag_multicopter_position_control_enabled &&
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
           status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER &&
           status.valid_nav_states_mask == (manual_mask | termination_mask) &&
           status.can_set_nav_states_mask == manual_mask;
}

bool BootHealthService::update_output_health(std::uint64_t now_us) noexcept
{
    if (actuator_output_status_subscription_.update()) {
        const actuator_output_status_s &status =
            actuator_output_status_subscription_.get();
        // sequence 按 uint32 模空间前进：delta==0 是重复，delta>=2^31 视为倒退；
        // 正常从 UINT32_MAX 回绕到 1 仍得到小正 delta，0 永远保留为无效。
        const std::uint32_t sequence_delta =
            status.sequence - last_output_sequence_;
        if (status.sequence == 0U ||
            (last_output_sequence_ != 0U &&
             (sequence_delta == 0U || sequence_delta >= 0x80000000U))) {
            output_snapshot_observed_ = false;
            return false;
        }
        last_output_sequence_ = status.sequence;
        output_snapshot_observed_ = true;
    }
    return output_snapshot_observed_ && output_status_runtime_healthy(now_us);
}

bool BootHealthService::output_mapping_consistent(
    const actuator_output_status_s &output) const noexcept
{
    // supported=(1<<NUM_OUTPUTS)-1；所有 mapping 位必须落在物理输出范围内，
    // 左右集合互斥且并集恰好等于 configured mask。
    const std::uint8_t supported = static_cast<std::uint8_t>(
        (1U << actuator_output_status_s::NUM_OUTPUTS) - 1U);
    const bool complete_drive_mapping =
        output.right_output_mask != 0U && output.left_output_mask != 0U;
    return output.configured_output_mask != 0U &&
           (output.configured_output_mask &
            static_cast<std::uint8_t>(~supported)) == 0U &&
           (output.right_output_mask & output.left_output_mask) == 0U &&
           static_cast<std::uint8_t>(output.right_output_mask |
                                     output.left_output_mask) ==
                output.configured_output_mask &&
           output.drive_available == complete_drive_mapping;
}

bool BootHealthService::output_mapping_valid(
    const actuator_output_status_s &output) const noexcept
{
    return output_mapping_consistent(output) &&
           output.right_output_mask != 0U && output.left_output_mask != 0U;
}

bool BootHealthService::output_frame_valid(
    const actuator_output_status_s &output) const noexcept
{
    // 非 Hard-Safe 帧要求每个 configured 输出都有合法 PWM，未激活槽必须严格为 0；
    // 这样 Topic 不能掩盖“mask 未置位但数组残留脉宽”的后台错误。
    if (output.active_output_mask != output.configured_output_mask) {
        return false;
    }
    for (std::uint8_t index = 0U;
         index < actuator_output_status_s::NUM_OUTPUTS; ++index) {
        const bool active = (output.active_output_mask &
            static_cast<std::uint8_t>(1U << index)) != 0U;
        const std::uint16_t pulse = output.pwm_us[index];
        if ((active &&
             (pulse < dima::platform::kActuatorPwmMinimumPulseUs ||
              pulse > dima::platform::kActuatorPwmMaximumPulseUs)) ||
            (!active && pulse != 0U)) {
            return false;
        }
    }
    return true;
}

bool BootHealthService::output_status_runtime_healthy(
    std::uint64_t now_us) const noexcept
{
    const actuator_output_status_s &output =
        actuator_output_status_subscription_.get();
    if (output.timestamp == 0U || output.timestamp > now_us ||
        now_us - output.timestamp > kOutputStatusTimeoutUs ||
        !output.backend_ready) {
        return false;
    }

    const actuator_armed_s &armed = actuator_armed_subscription_.get();
    const vehicle_status_s &status = vehicle_status_subscription_.get();
    // Hard-Safe 必须同时满足状态、safe_off、空 active mask 和全零脉宽，
    // 不能只相信一个布尔位。
    const bool hard_safe =
        output.state == actuator_output_status_s::STATE_HARD_SAFE_OFF &&
        output.safe_off && output.active_output_mask == 0U;
    if (hard_safe) {
        for (const std::uint16_t pulse : output.pwm_us) {
            if (pulse != 0U) {
                return false;
            }
        }
    }

    const bool forced_safe_state = armed.kill || armed.termination ||
                                   armed.lockdown || status.failsafe;
    if (forced_safe_state) {
        return hard_safe;
    }

    if (armed.armed) {
        // 解锁沿后的短窗口允许 PWM 尚处 Hard-Safe；窗口结束后必须进入 Active，
        // 且命令、左右映射和整帧范围全部有效。
        const bool in_transition = status.armed_time != 0U &&
            status.armed_time <= now_us &&
            now_us - status.armed_time <= kActuatorArmTransitionUs;
        if (in_transition && hard_safe) {
            return true;
        }
        return output.state == actuator_output_status_s::STATE_ACTIVE &&
               !output.safe_off && output.drive_available &&
               output.command_valid && output_mapping_valid(output) &&
               output_frame_valid(output);
    }

    if (output.drive_available) {
        // Disarmed 且具备完整左右驱动时应持续输出 Neutral；映射不完整的降级态
        // 可保持一致的 Neutral 或退回物理 Hard-Safe，但绝不允许 Active。
        return output.state ==
                   actuator_output_status_s::STATE_DISARMED_NEUTRAL &&
               !output.safe_off && !output.parameter_update_pending &&
               output_mapping_valid(output) &&
               output_frame_valid(output);
    }
    const bool degraded_neutral =
        output.state == actuator_output_status_s::STATE_DISARMED_NEUTRAL &&
        !output.safe_off && !output.parameter_update_pending &&
        output_mapping_consistent(output) && output_frame_valid(output);
    return degraded_neutral || hard_safe;
}

bool BootHealthService::output_status_confirmation_safe() const noexcept
{
    const actuator_output_status_s &output =
        actuator_output_status_subscription_.get();
    // image_ok 写 Flash 前不能存在尚未应用的 PWM 参数；只接受不产生动力的两态。
    return !output.parameter_update_pending &&
           (output.state == actuator_output_status_s::STATE_HARD_SAFE_OFF ||
            output.state ==
                actuator_output_status_s::STATE_DISARMED_NEUTRAL);
}

bool BootHealthService::confirmation_state_safe() const noexcept
{
    const actuator_armed_s &armed = actuator_armed_subscription_.get();
    const vehicle_status_s &status = vehicle_status_subscription_.get();
    // MCUboot 确认比 watchdog 运行健康更严格：必须人工安全态、无故障/失控，
    // 避免在 Armed 或 failsafe 期间把可能回滚的 test image 永久确认。
    return !armed.armed && !armed.kill && !armed.termination &&
           !armed.lockdown &&
           status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED &&
           status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
           status.failure_detector_status == vehicle_status_s::FAILURE_NONE &&
           !status.failsafe;
}

void BootHealthService::reset_stable_window(std::uint64_t now_ms) noexcept
{
    // 记录 now 只便于下一次激活重新起算；active=false 保证中断窗口不可续接。
    stable_window_active_ = false;
    stable_window_start_ms_ = now_ms;
}

} // namespace dima::modules::boot_health

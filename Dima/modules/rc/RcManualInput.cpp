/****************************************************************************
 * PX4-Autopilot v1.17.0 ManualControl RC subset adapted to the Dima platform.
 ****************************************************************************/
#include "RcManualInput.hpp"
#include "rover/RoverModeContract.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"
#include "vehicle_status.hpp"

#include <cmath>
#include <limits>

namespace dima::modules::rc {
namespace {

constexpr float kUnavailableControl = std::numeric_limits<float>::quiet_NaN();

} // namespace

RcManualInput::RcManualInput() noexcept
    : px4::ScheduledWorkItem("rc_manual_input",
                             px4::wq_configurations::hp_default)
{
}

RcManualInput::~RcManualInput()
{
    stop();
}

bool RcManualInput::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }

    rc_signal_available_ = false;
    lost_invalid_published_ = false;
    reset_switch_baseline();
    reset_switch_parameter_state();
    if (!initialize_switch_parameter_handles() ||
        !refresh_switch_configuration()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("RcManualInput switch parameters unavailable");
        return false;
    }

    if (!rc_channels_subscription_.registerCallback(*this)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    if (!switches_subscription_.registerCallback(*this)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        rc_channels_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        return false;
    }

    if (!parameter_update_subscription_.registerCallback(*this)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        switches_subscription_.unregisterCallback();
        rc_channels_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        return false;
    }

    state_ = dima::middleware::lifecycle::ModuleState::Running;

    // 启动时主动消费已存在的最新样本，避免等待下一次 RC 发布。
    if (!ScheduleNow()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        parameter_update_subscription_.unregisterCallback();
        switches_subscription_.unregisterCallback();
        rc_channels_subscription_.unregisterCallback();
        ScheduleCancelAndDrain();
        return false;
    }

    return true;
}

void RcManualInput::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    parameter_update_subscription_.unregisterCallback();
    switches_subscription_.unregisterCallback();
    rc_channels_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    rc_signal_available_ = false;
    lost_invalid_published_ = false;
    reset_switch_baseline();
    reset_switch_parameter_state();
}

dima::middleware::lifecycle::ModuleState RcManualInput::state() const
{
    return state_;
}

void RcManualInput::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }

    if (parameter_update_subscription_.update() &&
        !refresh_switch_configuration()) {
        reset_switch_baseline();
        PX4_ERR("RcManualInput switch parameter refresh failed");
    }

    if (rc_channels_subscription_.update()) {
        process_rc_channels(rc_channels_subscription_.get());
    }

    if (switches_subscription_.update()) {
        process_switches(switches_subscription_.get());
    }
}

bool RcManualInput::mapped_channel(const rc_channels_s &channels,
                                   std::uint8_t function,
                                   float &value) noexcept
{
    // 功能未映射、索引越界或通道为非有限值时统一返回 NaN，不制造可用的零输入。
    const std::int8_t channel = channels.function[function];

    if (channel < 0 ||
        static_cast<std::uint8_t>(channel) >= channels.channel_count ||
        static_cast<std::size_t>(channel) >=
            sizeof(channels.channels) / sizeof(channels.channels[0])) {
        value = kUnavailableControl;
        return false;
    }

    value = channels.channels[static_cast<std::uint8_t>(channel)];
    if (!std::isfinite(value)) {
        value = kUnavailableControl;
        return false;
    }
    return true;
}

void RcManualInput::process_rc_channels(const rc_channels_s &channels) noexcept
{
    const std::uint64_t now = hrt_absolute_time();

    if (channels.signal_lost) {
        rc_signal_available_ = false;
        reset_switch_baseline();

        // 失联沿只发布一次无效值，持续失联不重复冲刷 Topic。
        if (!lost_invalid_published_) {
            manual_control_setpoint_s setpoint{};
            setpoint.timestamp = now;
            setpoint.timestamp_sample = channels.timestamp_last_valid;
            setpoint.valid = false;
            setpoint.data_source = manual_control_setpoint_s::SOURCE_RC;
            setpoint.roll = kUnavailableControl;
            setpoint.pitch = kUnavailableControl;
            setpoint.yaw = kUnavailableControl;
            setpoint.throttle = kUnavailableControl;
            (void)setpoint_publication_.publish(setpoint);
            lost_invalid_published_ = true;
        }
        return;
    }

    rc_signal_available_ = true;
    lost_invalid_published_ = false;

    manual_control_setpoint_s setpoint{};
    setpoint.timestamp = now;
    setpoint.timestamp_sample = channels.timestamp_last_valid;
    setpoint.data_source = manual_control_setpoint_s::SOURCE_RC;

    bool throttle_mapped = mapped_channel(
        channels, rc_channels_s::FUNCTION_THROTTLE, setpoint.throttle);
    bool yaw_mapped = mapped_channel(
        channels, rc_channels_s::FUNCTION_YAW, setpoint.yaw);
    // RC_MAP_ROLL/PITCH 只作为 stock QGC 四轴完成门的固定非零标记；
    // 差速 Rover 运行链明确不消费这两个伪映射，避免第三路模式开关或
    // 两个真实控制轴被误解释成不存在的横滚/俯仰控制。
    setpoint.roll = kUnavailableControl;
    setpoint.pitch = kUnavailableControl;

    setpoint.valid = throttle_mapped && yaw_mapped;
    setpoint.sticks_moving = false;
    setpoint.buttons = 0U;
    (void)setpoint_publication_.publish(setpoint);
}

void RcManualInput::process_switches(
    const manual_control_switches_s &switches) noexcept
{
    if (!rc_signal_available_) {
        reset_switch_baseline();
        return;
    }

    const std::uint64_t sample_time = switches.timestamp_sample;
    if (sample_time == 0U ||
        (last_switch_sample_us_ != 0U &&
         sample_time < last_switch_sample_us_)) {
        reset_switch_baseline();
        return;
    }

    // 去抖同时要求状态一致、样本数足够且基于原始 sample_time 的持续时间足够；
    // 调度器重复处理同一 Topic 不会被累计成多个稳定样本。
    const bool candidate_matches = candidate_sample_count_ != 0U &&
        switches.mode_slot == candidate_switches_.mode_slot &&
        switches.arm_switch == candidate_switches_.arm_switch &&
        switches.kill_switch == candidate_switches_.kill_switch;
    if (!candidate_matches) {
        candidate_switches_ = switches;
        candidate_since_us_ = sample_time;
        last_switch_sample_us_ = sample_time;
        candidate_sample_count_ = 1U;
        return;
    }

    if (sample_time > last_switch_sample_us_) {
        last_switch_sample_us_ = sample_time;
        if (candidate_sample_count_ < kRequiredStableSamples) {
            ++candidate_sample_count_;
        }
    }
    if (candidate_sample_count_ < kRequiredStableSamples ||
        sample_time - candidate_since_us_ < kSwitchDebounceUs) {
        return;
    }

    if (!switches_initialized_) {
        // 首个稳定离散样本只建立基线，禁止上电状态触发模式或 Arm/Kill。
        previous_switches_ = candidate_switches_;
        switches_initialized_ = true;
        return;
    }

    const bool kill_changed =
        candidate_switches_.kill_switch != previous_switches_.kill_switch;
    const bool kill_engaged =
        kill_changed &&
        candidate_switches_.kill_switch ==
            manual_control_switches_s::SWITCH_POS_ON;
    const bool arm_changed =
        candidate_switches_.arm_switch != previous_switches_.arm_switch;
    const bool mode_changed =
        candidate_switches_.mode_slot != previous_switches_.mode_slot;

    // Kill 与 Arm 同帧变化时必须先发布 Kill，防止中间态 ARMED 快照逃逸到 MotorOutput。
    if (kill_engaged) {
        publish_action(action_request_s::ACTION_KILL);
    }

    if (arm_changed) {
        if (candidate_switches_.arm_switch ==
            manual_control_switches_s::SWITCH_POS_ON) {
            publish_action(action_request_s::ACTION_ARM);
        } else if (candidate_switches_.arm_switch ==
                   manual_control_switches_s::SWITCH_POS_OFF) {
            publish_action(action_request_s::ACTION_DISARM);
        }
    }

    // Unkill 始终最后发布：同帧 Arm 边沿仍在 Kill 锁存期间求值，只有后续新的
    // Arm 边沿才可能重新解锁，避免一个复合开关动作直接恢复动力。
    if (kill_changed && !kill_engaged &&
        candidate_switches_.kill_switch ==
            manual_control_switches_s::SWITCH_POS_OFF) {
        publish_action(action_request_s::ACTION_UNKILL);
    }

    // Arm/Kill 与模式同帧变化时只执行安全动作并消费该模式边沿；禁止先切
    // AUTO 再解锁，也禁止解除 Kill 的同一复合动作恢复任务。操作者必须在
    // 安全状态稳定后重新拨动模式开关，才会产生新的模式请求。
    if (!kill_changed && !arm_changed && mode_changed) {
        evaluate_mode_slot(candidate_switches_.mode_slot);
    }

    previous_switches_ = candidate_switches_;
}

void RcManualInput::evaluate_mode_slot(std::uint8_t mode_slot) noexcept
{
    namespace contract = dima::generated::parameters;
    if (mode_slot == manual_control_switches_s::MODE_SLOT_NONE) {
        return;
    }
    const std::size_t index = static_cast<std::size_t>(mode_slot - 1U);
    if (index >= mode_slot_values_.size()) {
        PX4_WARN("RC mode slot overflow: %u", mode_slot);
        return;
    }

    const std::int32_t selection = mode_slot_values_[index];
    if (!contract::flight_mode_slot_value_allowed(selection) ||
        selection < 0) {
        // 负值是权威 YAML 定义的 Calibration Reserved/Unassigned：当前只
        // 占位，不发布动作，因此从 Mission 拨入预留槽也保持现有模式。
        return;
    }
    if (!dima::middleware::rover::mode_contract::rc_slot_supported(selection)) {
        // 即使未来 YAML 被扩展，未实现的导航态也必须在实际消费者处闭锁，
        // 不能仅因 QGC 元数据出现一个枚举值就宣称板端支持该模式。
        PX4_WARN("RC mode slot selected unsupported mode: %ld",
                 static_cast<long>(selection));
        return;
    }
    publish_mode_action(static_cast<std::uint8_t>(selection));
}

void RcManualInput::publish_action(std::uint8_t action) noexcept
{
    action_request_s request{};
    request.timestamp = hrt_absolute_time();
    request.action = action;
    request.source = action_request_s::SOURCE_RC_SWITCH;
    request.mode = 0U;
    (void)action_request_publication_.publish(request);
}

void RcManualInput::publish_mode_action(std::uint8_t mode) noexcept
{
    action_request_s request{};
    request.timestamp = hrt_absolute_time();
    request.action = action_request_s::ACTION_SWITCH_MODE;
    request.source = action_request_s::SOURCE_RC_MODE_SLOT;
    request.mode = mode;
    (void)action_request_publication_.publish(request);
}

void RcManualInput::reset_switch_baseline() noexcept
{
    previous_switches_ = manual_control_switches_s{};
    candidate_switches_ = manual_control_switches_s{};
    candidate_since_us_ = 0U;
    last_switch_sample_us_ = 0U;
    candidate_sample_count_ = 0U;
    switches_initialized_ = false;
}

void RcManualInput::reset_switch_parameter_state() noexcept
{
    namespace contract = dima::generated::parameters;
    arm_mapping_handle_ = PARAM_INVALID;
    kill_mapping_handle_ = PARAM_INVALID;
    mode_mapping_handle_ = PARAM_INVALID;
    arm_threshold_handle_ = PARAM_INVALID;
    kill_threshold_handle_ = PARAM_INVALID;
    mode_slot_handles_.fill(PARAM_INVALID);
    for (std::size_t index = 0U; index < mode_slot_values_.size(); ++index) {
        mode_slot_values_[index] =
            contract::kFlightModeSlotParameters[index].default_value;
    }
    mode_slot_invalid_reported_.fill(false);
    arm_mapping_ = 0;
    kill_mapping_ = 0;
    mode_mapping_ = 0;
    arm_threshold_ = 0.75F;
    kill_threshold_ = 0.75F;
    switch_configuration_initialized_ = false;
}

bool RcManualInput::initialize_switch_parameter_handles() noexcept
{
    namespace contract = dima::generated::parameters;
    // 直接消费生成参数枚举；模式/开关业务只声明所需角色，不复制字符串名称表。
    arm_mapping_handle_ = param_handle(dima::params::RC_MAP_ARM_SW);
    kill_mapping_handle_ = param_handle(dima::params::RC_MAP_KILL_SW);
    mode_mapping_handle_ = param_handle(dima::params::RC_MAP_FLTMODE);
    arm_threshold_handle_ = param_handle(dima::params::RC_ARMSWITCH_TH);
    kill_threshold_handle_ = param_handle(dima::params::RC_KILLSWITCH_TH);
    bool valid = arm_mapping_handle_ != PARAM_INVALID &&
                 kill_mapping_handle_ != PARAM_INVALID &&
                 mode_mapping_handle_ != PARAM_INVALID &&
                 arm_threshold_handle_ != PARAM_INVALID &&
                 kill_threshold_handle_ != PARAM_INVALID;
    for (std::size_t index = 0U; index < mode_slot_handles_.size(); ++index) {
        mode_slot_handles_[index] = param_handle(
            contract::kFlightModeSlotParameters[index].parameter);
        valid = valid && mode_slot_handles_[index] != PARAM_INVALID;
    }
    return valid;
}

bool RcManualInput::refresh_switch_configuration() noexcept
{
    namespace contract = dima::generated::parameters;
    std::int32_t arm_mapping = 0;
    std::int32_t kill_mapping = 0;
    std::int32_t mode_mapping = 0;
    float arm_threshold = 0.0F;
    float kill_threshold = 0.0F;
    std::array<std::int32_t, kFlightModeSlotCount> mode_slot_values{};
    if (param_get(arm_mapping_handle_, &arm_mapping) != 0 ||
        param_get(kill_mapping_handle_, &kill_mapping) != 0 ||
        param_get(mode_mapping_handle_, &mode_mapping) != 0 ||
        param_get(arm_threshold_handle_, &arm_threshold) != 0 ||
        param_get(kill_threshold_handle_, &kill_threshold) != 0) {
        return false;
    }
    for (std::size_t index = 0U; index < mode_slot_values.size(); ++index) {
        if (param_get(mode_slot_handles_[index], &mode_slot_values[index]) != 0) {
            return false;
        }
        if (!contract::flight_mode_slot_value_allowed(
                mode_slot_values[index])) {
            if (!mode_slot_invalid_reported_[index]) {
                PX4_WARN("invalid RC mode slot %u value %ld; using default",
                         static_cast<unsigned>(index + 1U),
                         static_cast<long>(mode_slot_values[index]));
                mode_slot_invalid_reported_[index] = true;
            }
            // 非法值只降级当前槽，不使 Throttle/Yaw 整条手动链失效；默认值
            // 同样来自生成合同，因此旧快照或内部误写不会绕过 YAML 权威定义。
            mode_slot_values[index] =
                contract::kFlightModeSlotParameters[index].default_value;
        } else {
            mode_slot_invalid_reported_[index] = false;
        }
    }

    const bool changed = switch_configuration_initialized_ &&
        (arm_mapping != arm_mapping_ || kill_mapping != kill_mapping_ ||
         mode_mapping != mode_mapping_ ||
         mode_slot_values != mode_slot_values_ ||
         arm_threshold != arm_threshold_ || kill_threshold != kill_threshold_);
    arm_mapping_ = arm_mapping;
    kill_mapping_ = kill_mapping;
    mode_mapping_ = mode_mapping;
    mode_slot_values_ = mode_slot_values;
    arm_threshold_ = arm_threshold;
    kill_threshold_ = kill_threshold;
    switch_configuration_initialized_ = true;
    if (changed) {
        // 映射或阈值变化会改变边沿含义，必须丢弃旧基线，避免参数更新本身触发动作。
        reset_switch_baseline();
    }
    return true;
}

} // namespace dima::modules::rc

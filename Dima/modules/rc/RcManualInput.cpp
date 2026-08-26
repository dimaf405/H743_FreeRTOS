/****************************************************************************
 * PX4-Autopilot v1.17.0 ManualControl RC subset adapted to the Dima platform.
 ****************************************************************************/
#include "RcManualInput.hpp"

#include "logging/logging.hpp"
#include "api/Time.hpp"

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
        static_cast<std::uint8_t>(channel) >= 18U) {
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
            setpoint.flaps = kUnavailableControl;
            setpoint.aux1 = kUnavailableControl;
            setpoint.aux2 = kUnavailableControl;
            setpoint.aux3 = kUnavailableControl;
            setpoint.aux4 = kUnavailableControl;
            setpoint.aux5 = kUnavailableControl;
            setpoint.aux6 = kUnavailableControl;
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
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_ROLL, setpoint.roll);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_PITCH, setpoint.pitch);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_FLAPS, setpoint.flaps);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_1, setpoint.aux1);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_2, setpoint.aux2);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_3, setpoint.aux3);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_4, setpoint.aux4);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_5, setpoint.aux5);
    (void)mapped_channel(channels, rc_channels_s::FUNCTION_AUX_6, setpoint.aux6);

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
        // 首个稳定离散样本只建立基线，禁止上电状态触发 Arm/Kill。
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

    // Kill 与 Arm 同帧变化时必须先发布 Kill，防止中间态 ARMED 快照逃逸到 MotorOutput。
    if (kill_engaged) {
        publish_action(action_request_s::ACTION_KILL);
    }

    if (candidate_switches_.arm_switch != previous_switches_.arm_switch) {
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

    previous_switches_ = candidate_switches_;
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
    arm_mapping_handle_ = PARAM_INVALID;
    kill_mapping_handle_ = PARAM_INVALID;
    arm_threshold_handle_ = PARAM_INVALID;
    kill_threshold_handle_ = PARAM_INVALID;
    arm_mapping_ = 0;
    kill_mapping_ = 0;
    arm_threshold_ = 0.75F;
    kill_threshold_ = 0.75F;
    switch_configuration_initialized_ = false;
}

bool RcManualInput::initialize_switch_parameter_handles() noexcept
{
    // 直接消费生成参数枚举；开关业务只声明所需角色，不复制字符串名称表。
    arm_mapping_handle_ = param_handle(px4::params::RC_MAP_ARM_SW);
    kill_mapping_handle_ = param_handle(px4::params::RC_MAP_KILL_SW);
    arm_threshold_handle_ = param_handle(px4::params::RC_ARMSWITCH_TH);
    kill_threshold_handle_ = param_handle(px4::params::RC_KILLSWITCH_TH);
    return arm_mapping_handle_ != PARAM_INVALID &&
           kill_mapping_handle_ != PARAM_INVALID &&
           arm_threshold_handle_ != PARAM_INVALID &&
           kill_threshold_handle_ != PARAM_INVALID;
}

bool RcManualInput::refresh_switch_configuration() noexcept
{
    std::int32_t arm_mapping = 0;
    std::int32_t kill_mapping = 0;
    float arm_threshold = 0.0F;
    float kill_threshold = 0.0F;
    if (param_get(arm_mapping_handle_, &arm_mapping) != 0 ||
        param_get(kill_mapping_handle_, &kill_mapping) != 0 ||
        param_get(arm_threshold_handle_, &arm_threshold) != 0 ||
        param_get(kill_threshold_handle_, &kill_threshold) != 0) {
        return false;
    }

    const bool changed = switch_configuration_initialized_ &&
        (arm_mapping != arm_mapping_ || kill_mapping != kill_mapping_ ||
         arm_threshold != arm_threshold_ || kill_threshold != kill_threshold_);
    arm_mapping_ = arm_mapping;
    kill_mapping_ = kill_mapping;
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

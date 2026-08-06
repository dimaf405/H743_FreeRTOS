/****************************************************************************
 * PX4-Autopilot v1.17.0 ManualControl RC subset adapted to the Dima platform.
 ****************************************************************************/
#include "RcManualInput.hpp"

#include "platform/api/Time.hpp"

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

    state_ = dima::middleware::lifecycle::ModuleState::Running;

    // 启动时主动消费已存在的最新样本，避免等待下一次 RC 发布。
    if (!ScheduleNow()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
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
    switches_subscription_.unregisterCallback();
    rc_channels_subscription_.unregisterCallback();
    ScheduleCancelAndDrain();
    rc_signal_available_ = false;
    lost_invalid_published_ = false;
    reset_switch_baseline();
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

    if (!switches_initialized_) {
        // 首个稳定离散样本只建立基线，禁止上电状态触发 Arm/Kill。
        previous_switches_ = switches;
        switches_initialized_ = true;
        return;
    }

    if (switches.arm_switch != previous_switches_.arm_switch) {
        if (switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON) {
            publish_action(action_request_s::ACTION_ARM);
        } else if (switches.arm_switch == manual_control_switches_s::SWITCH_POS_OFF) {
            publish_action(action_request_s::ACTION_DISARM);
        }
    }

    // Kill 后发布，使同一帧 Arm/Kill 同时变化时安全动作最终排在队尾。
    if (switches.kill_switch != previous_switches_.kill_switch) {
        if (switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
            publish_action(action_request_s::ACTION_KILL);
        } else if (switches.kill_switch == manual_control_switches_s::SWITCH_POS_OFF) {
            publish_action(action_request_s::ACTION_UNKILL);
        }
    }

    previous_switches_ = switches;
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
    switches_initialized_ = false;
}

} // namespace dima::modules::rc

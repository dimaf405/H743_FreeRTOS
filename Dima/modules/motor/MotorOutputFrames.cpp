#include "MotorOutput.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dima::modules::motor {

bool MotorOutput::build_frame(
    dima::platform::ActuatorPwmFrame &frame) const noexcept
{
    if (!parameters_valid_ || !parameters_.drive_available) {
        return false;
    }
    frame = dima::platform::ActuatorPwmFrame{};
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        const ChannelConfig &config = parameters_.channels[channel];
        if (config.function == ChannelFunction::Disabled) {
            continue;
        }
        float command = config.function == ChannelFunction::MotorRight
                            ? actuator_motors_.control[0]
                            : actuator_motors_.control[1];
        if (config.reversed) {
            command = -command;
        }
        if (!normalized(command)) {
            return false;
        }
        frame.pulse_us[channel] = map_normalized(config, command);
        frame.enabled_mask |= static_cast<std::uint8_t>(1U << channel);
    }
    return frame.enabled_mask == parameters_.configured_mask;
}

bool MotorOutput::build_neutral_frame(
    dima::platform::ActuatorPwmFrame &frame) const noexcept
{
    if (!parameters_valid_ || parameters_.configured_mask == 0U) {
        return false;
    }
    frame = dima::platform::ActuatorPwmFrame{};
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        const ChannelConfig &config = parameters_.channels[channel];
        if (config.function == ChannelFunction::Disabled) {
            continue;
        }
        frame.pulse_us[channel] = config.center_us;
        frame.enabled_mask |= static_cast<std::uint8_t>(1U << channel);
    }
    return frame.enabled_mask == parameters_.configured_mask;
}

dima::platform::ActuatorPwmResult MotorOutput::apply_frame(
    const dima::platform::ActuatorPwmFrame &frame) noexcept
{
    if (pwm_ == nullptr) {
        return dima::platform::ActuatorPwmResult::Fault;
    }
    if (!pwm_->started()) {
        const dima::platform::ActuatorPwmResult started = pwm_->start();
        if (started != dima::platform::ActuatorPwmResult::Applied) {
            return started;
        }
    }
    const dima::platform::ActuatorPwmResult result = pwm_->write(frame);
    if (result == dima::platform::ActuatorPwmResult::Applied) {
        applied_frame_ = frame;
        // 以 PWM 后端成功为证据，从已应用脉宽反解逻辑归一化输出，撤销各通道
        // REV。多路映射到同一侧时取均值；量化误差保留，不伪装成电流/RPM。
        float right = 0.0F, left = 0.0F;
        unsigned right_count = 0U, left_count = 0U;
        for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
            if ((frame.enabled_mask & (1U << channel)) == 0U) continue;
            const auto &config = parameters_.channels[channel];
            const float delta = static_cast<float>(frame.pulse_us[channel]) - config.center_us;
            const float span = delta >= 0.0F ? config.maximum_us - config.center_us
                                            : config.center_us - config.minimum_us;
            const float command = span > 0.0F ? std::clamp(delta / span, -1.0F, 1.0F) : 0.0F;
            const float logical = config.reversed ? -command : command;
            if (config.function == ChannelFunction::MotorRight) { right += logical; ++right_count; }
            if (config.function == ChannelFunction::MotorLeft) { left += logical; ++left_count; }
        }
        applied_right_ = right_count != 0U ? right / right_count : 0.0F;
        applied_left_ = left_count != 0U ? left / left_count : 0.0F;
        applied_output_timestamp_ = hrt_absolute_time();
        backend_ready_ = true;
        safe_off_ = false;
    }
    return result;
}

dima::platform::ActuatorPwmResult MotorOutput::force_safe_off() noexcept
{
    if (pwm_ == nullptr) {
        backend_ready_ = false;
        safe_off_ = false;
        return dima::platform::ActuatorPwmResult::Fault;
    }
    if (safe_off_ && !pwm_->started()) {
        applied_right_ = applied_left_ = 0.0F;
        applied_output_timestamp_ = hrt_absolute_time();
        return dima::platform::ActuatorPwmResult::Applied;
    }

    const dima::platform::ActuatorPwmResult result = pwm_->stop();
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    backend_ready_ = result == dima::platform::ActuatorPwmResult::Applied;
    safe_off_ = backend_ready_;
    if (safe_off_) {
        applied_right_ = applied_left_ = 0.0F;
        applied_output_timestamp_ = hrt_absolute_time();
    }
    return result;
}

bool MotorOutput::publish_status(std::uint64_t now_us,
                                 std::uint8_t output_state,
                                 bool command_valid) noexcept
{
    actuator_output_status_s status{};
    status.timestamp = std::max(now_us, hrt_absolute_time());
    status.timestamp_sample = actuator_motors_.timestamp_sample;
    const bool output_confirmed = backend_ready_ &&
        (output_state == actuator_output_status_s::STATE_ACTIVE ||
         output_state == actuator_output_status_s::STATE_DISARMED_NEUTRAL || safe_off_);
    status.timestamp_output = output_confirmed ? applied_output_timestamp_ : 0U;
    status.applied_right = output_confirmed ? applied_right_ : std::numeric_limits<float>::quiet_NaN();
    status.applied_left = output_confirmed ? applied_left_ : std::numeric_limits<float>::quiet_NaN();
    ++status_sequence_;
    if (status_sequence_ == 0U) {
        ++status_sequence_;
    }
    status.sequence = status_sequence_;
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        status.pwm_us[channel] = applied_frame_.pulse_us[channel];
    }
    status.active_output_mask = applied_frame_.enabled_mask;
    status.configured_output_mask = parameters_valid_
        ? parameters_.configured_mask : 0U;
    status.right_output_mask = parameters_valid_ ? parameters_.right_mask : 0U;
    status.left_output_mask = parameters_valid_ ? parameters_.left_mask : 0U;
    status.state = output_state;
    status.backend_ready = backend_ready_;
    status.drive_available = drive_available();
    status.safe_off = safe_off_;
    status.command_valid = command_valid;
    status.parameter_update_pending = parameter_update_pending_;
    return output_status_publication_.publish(status);
}

bool MotorOutput::finite(float value) noexcept { return std::isfinite(value); }

bool MotorOutput::normalized(float value) noexcept
{
    return finite(value) && value >= -1.0F && value <= 1.0F;
}

std::uint16_t MotorOutput::map_normalized(const ChannelConfig &channel,
                                          float value) noexcept
{
    // 对照 APM SRV_Channel::pwm_from_angle：先对偏离中位的幅值截断，
    // 再按符号加/减。小于 1 us 的命令保持 CENT，正负量化对称；不能对
    // 最终脉宽四舍五入，让边界附近的小命令被额外推离停止中位。
    const float span = value >= 0.0F ? channel.maximum_us - channel.center_us
                                    : channel.center_us - channel.minimum_us;
    const auto delta = static_cast<std::uint16_t>(std::fabs(value) * span);
    return value >= 0.0F ? static_cast<std::uint16_t>(channel.center_us + delta)
                        : static_cast<std::uint16_t>(channel.center_us - delta);
}

} // namespace dima::modules::motor

#include "MotorOutput.hpp"

#include <cmath>

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
        return dima::platform::ActuatorPwmResult::Applied;
    }

    const dima::platform::ActuatorPwmResult result = pwm_->stop();
    applied_frame_ = dima::platform::ActuatorPwmFrame{};
    backend_ready_ = result == dima::platform::ActuatorPwmResult::Applied;
    safe_off_ = backend_ready_;
    return result;
}

bool MotorOutput::publish_status(std::uint64_t now_us,
                                 std::uint8_t output_state,
                                 bool command_valid) noexcept
{
    actuator_output_status_s status{};
    status.timestamp = now_us;
    status.timestamp_sample = actuator_motors_.timestamp_sample;
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
    const float center = static_cast<float>(channel.center_us);
    const float pulse = value >= 0.0F
                            ? center + value *
                                           static_cast<float>(
                                               channel.maximum_us -
                                               channel.center_us)
                            : center + value *
                                           static_cast<float>(
                                               channel.center_us -
                                               channel.minimum_us);
    return static_cast<std::uint16_t>(pulse + 0.5F);
}

} // namespace dima::modules::motor

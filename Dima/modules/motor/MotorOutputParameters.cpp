#include "MotorOutput.hpp"

namespace dima::modules::motor {
namespace {

enum ParameterField : std::size_t {
    Function = 0U,
    Minimum,
    Center,
    Maximum,
    Reversed,
};

constexpr const char *kParameterNames[6][5] = {
    {"PWM_S1_FUNC", "PWM_S1_MIN", "PWM_S1_CENT", "PWM_S1_MAX",
     "PWM_S1_REV"},
    {"PWM_S2_FUNC", "PWM_S2_MIN", "PWM_S2_CENT", "PWM_S2_MAX",
     "PWM_S2_REV"},
    {"PWM_S3_FUNC", "PWM_S3_MIN", "PWM_S3_CENT", "PWM_S3_MAX",
     "PWM_S3_REV"},
    {"PWM_S4_FUNC", "PWM_S4_MIN", "PWM_S4_CENT", "PWM_S4_MAX",
     "PWM_S4_REV"},
    {"PWM_S5_FUNC", "PWM_S5_MIN", "PWM_S5_CENT", "PWM_S5_MAX",
     "PWM_S5_REV"},
    {"PWM_S6_FUNC", "PWM_S6_MIN", "PWM_S6_CENT", "PWM_S6_MAX",
     "PWM_S6_REV"},
};

} // namespace

bool MotorOutput::bind_parameters() noexcept
{
    invalidate_parameter_bindings();
    if (!command_timeout_.bind()) {
        return false;
    }
    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        for (std::size_t field = 0U; field < kFieldsPerChannel; ++field) {
            const param_t handle =
                param_find_no_notification(kParameterNames[channel][field]);
            if (handle == PARAM_INVALID) {
                invalidate_parameter_bindings();
                return false;
            }
            param_set_used(handle);
            parameter_handles_[channel][field] = handle;
        }
    }
    return apply_parameter_snapshot();
}

void MotorOutput::invalidate_parameter_bindings() noexcept
{
    command_timeout_.invalidate();
    for (auto &channel : parameter_handles_) {
        for (param_t &handle : channel) {
            handle = PARAM_INVALID;
        }
    }
}

bool MotorOutput::apply_parameter_snapshot() noexcept
{
    ParameterSnapshot candidate{};
    std::int32_t raw[kChannelCount][kFieldsPerChannel]{};
    bool loaded = true;
    {
        px4::AtomicTransaction transaction;
        loaded = command_timeout_.bound() &&
                 param_get(command_timeout_.handle(),
                           &candidate.command_timeout_s) == 0;
        for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
            for (std::size_t field = 0U; field < kFieldsPerChannel; ++field) {
                const param_t handle = parameter_handles_[channel][field];
                if (handle == PARAM_INVALID ||
                    param_get(handle, &raw[channel][field]) != 0) {
                    loaded = false;
                }
            }
        }
    }
    if (!loaded) {
        parameters_valid_ = false;
        return false;
    }
    if (!finite(candidate.command_timeout_s) ||
        candidate.command_timeout_s < 0.02F ||
        candidate.command_timeout_s > 1.0F) {
        parameters_valid_ = false;
        return false;
    }

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        if (raw[channel][Minimum] < 800 || raw[channel][Minimum] > 2200 ||
            raw[channel][Center] < 800 || raw[channel][Center] > 2200 ||
            raw[channel][Maximum] < 800 || raw[channel][Maximum] > 2200) {
            parameters_valid_ = false;
            return false;
        }
        ChannelConfig config{};
        config.function =
            static_cast<ChannelFunction>(raw[channel][Function]);
        config.minimum_us = static_cast<std::uint16_t>(raw[channel][Minimum]);
        config.center_us = static_cast<std::uint16_t>(raw[channel][Center]);
        config.maximum_us = static_cast<std::uint16_t>(raw[channel][Maximum]);
        config.reversed = raw[channel][Reversed] != 0;
        if (!valid_channel(config) ||
            (raw[channel][Reversed] != 0 && raw[channel][Reversed] != 1)) {
            parameters_valid_ = false;
            return false;
        }

        const std::uint8_t bit = static_cast<std::uint8_t>(1U << channel);
        candidate.channels[channel] = config;
        if (config.function != ChannelFunction::Disabled) {
            candidate.configured_mask |= bit;
        }
        if (config.function == ChannelFunction::MotorRight) {
            candidate.right_mask |= bit;
        } else if (config.function == ChannelFunction::MotorLeft) {
            candidate.left_mask |= bit;
        }
    }
    candidate.drive_available =
        candidate.right_mask != 0U && candidate.left_mask != 0U;
    command_timeout_.set(candidate.command_timeout_s);
    parameters_ = candidate;
    parameters_valid_ = true;
    return true;
}

} // namespace dima::modules::motor

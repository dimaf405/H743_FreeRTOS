#include "MotorOutput.hpp"

#include "events/events.hpp"

#include <cmath>
#include <cstring>

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

constexpr std::uint32_t kEventParameterInvalid = 0x524D4F01U;
constexpr float kDefaultCommandTimeoutS = 0.10F;
constexpr std::int32_t kMinimumPulseUs = static_cast<std::int32_t>(
    dima::platform::kActuatorPwmMinimumPulseUs);
constexpr std::int32_t kMaximumPulseUs = static_cast<std::int32_t>(
    dima::platform::kActuatorPwmMaximumPulseUs);

enum class ParameterIssue : std::uint32_t {
    CommandTimeoutFallback = 1U,
    UnsupportedFunction = 2U,
    MinimumOutOfRange = 3U,
    CenterOutOfRange = 4U,
    MaximumOutOfRange = 5U,
    EndpointsSwapped = 6U,
    CenterBelowMinimum = 7U,
    CenterAboveMaximum = 8U,
    InvalidReversed = 9U,
};

bool command_timeout_valid(float value) noexcept
{
    return std::isfinite(value) && value >= 0.02F && value <= 1.0F;
}

bool pulse_in_output_envelope(std::int32_t value) noexcept
{
    return value >= kMinimumPulseUs && value <= kMaximumPulseUs;
}

std::uint32_t float_bits(float value) noexcept
{
    static_assert(sizeof(value) == sizeof(std::uint32_t));
    std::uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void report_parameter_issue(ParameterIssue issue,
                            std::uint32_t channel,
                            std::uint32_t first_value,
                            std::uint32_t second_value) noexcept
{
    const std::uint32_t arguments[4]{static_cast<std::uint32_t>(issue),
                                     channel, first_value, second_value};
    (void)dima::events::report(kEventParameterInvalid,
                               dima::events::Severity::Warning, arguments,
                               sizeof(arguments) / sizeof(arguments[0]));
}

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
    return true;
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
    if (!command_timeout_valid(candidate.command_timeout_s)) {
        const float requested_timeout = candidate.command_timeout_s;
        candidate.command_timeout_s =
            command_timeout_valid(parameters_.command_timeout_s)
                ? parameters_.command_timeout_s
                : kDefaultCommandTimeoutS;
        report_parameter_issue(ParameterIssue::CommandTimeoutFallback, 0U,
                               float_bits(requested_timeout),
                               float_bits(candidate.command_timeout_s));
    }

    for (std::size_t channel = 0U; channel < kChannelCount; ++channel) {
        const std::uint32_t channel_number =
            static_cast<std::uint32_t>(channel + 1U);
        const std::int32_t raw_function = raw[channel][Function];
        if (raw_function == static_cast<std::int32_t>(
                                ChannelFunction::Disabled)) {
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }
        if (raw_function != static_cast<std::int32_t>(
                                ChannelFunction::MotorRight) &&
            raw_function != static_cast<std::int32_t>(
                                ChannelFunction::MotorLeft)) {
            report_parameter_issue(ParameterIssue::UnsupportedFunction,
                                   channel_number,
                                   static_cast<std::uint32_t>(raw_function),
                                   0U);
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }
        if (raw[channel][Reversed] != 0 &&
            raw[channel][Reversed] != 1) {
            report_parameter_issue(ParameterIssue::InvalidReversed,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Reversed]),
                                   0U);
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }

        if (!pulse_in_output_envelope(raw[channel][Minimum])) {
            report_parameter_issue(ParameterIssue::MinimumOutOfRange,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Minimum]),
                                   0U);
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }
        if (!pulse_in_output_envelope(raw[channel][Center])) {
            report_parameter_issue(ParameterIssue::CenterOutOfRange,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Center]),
                                   0U);
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }
        if (!pulse_in_output_envelope(raw[channel][Maximum])) {
            report_parameter_issue(ParameterIssue::MaximumOutOfRange,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Maximum]),
                                   0U);
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }

        std::int32_t effective_minimum = raw[channel][Minimum];
        std::int32_t effective_maximum = raw[channel][Maximum];
        if (effective_minimum > effective_maximum) {
            const std::int32_t temporary = effective_minimum;
            effective_minimum = effective_maximum;
            effective_maximum = temporary;
            report_parameter_issue(ParameterIssue::EndpointsSwapped,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Minimum]),
                                   static_cast<std::uint32_t>(
                                       raw[channel][Maximum]));
        }
        if (raw[channel][Center] < effective_minimum) {
            report_parameter_issue(ParameterIssue::CenterBelowMinimum,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Center]),
                                   static_cast<std::uint32_t>(
                                       effective_minimum));
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }
        if (raw[channel][Center] > effective_maximum) {
            report_parameter_issue(ParameterIssue::CenterAboveMaximum,
                                   channel_number,
                                   static_cast<std::uint32_t>(
                                       raw[channel][Center]),
                                   static_cast<std::uint32_t>(
                                       effective_maximum));
            candidate.channels[channel] = ChannelConfig{};
            continue;
        }

        ChannelConfig config{};
        config.function = static_cast<ChannelFunction>(raw_function);
        config.minimum_us = static_cast<std::uint16_t>(effective_minimum);
        config.center_us = static_cast<std::uint16_t>(raw[channel][Center]);
        config.maximum_us = static_cast<std::uint16_t>(effective_maximum);
        config.reversed = raw[channel][Reversed] == 1;

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

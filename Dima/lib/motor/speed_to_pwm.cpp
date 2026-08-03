#include "speed_to_pwm.hpp"

namespace dima::lib::motor {
namespace {

static_assert(kMotorCount <= 8U,
              "MotorSpeedFrame::valid_mask can represent at most eight motors");

bool calibration_is_valid(const SpeedPwmCalibration &calibration) noexcept
{
    return calibration.min_us < calibration.neutral_us &&
           calibration.neutral_us < calibration.max_us;
}

bool result_has_valid_pwm(ConvertStatus status) noexcept
{
    return status == ConvertStatus::Neutral || status == ConvertStatus::Ok ||
           status == ConvertStatus::SaturatedForward ||
           status == ConvertStatus::SaturatedReverse;
}

uint16_t map_forward(float speed,
                     const SpeedPwmCalibration &calibration) noexcept
{
    const uint32_t span =
        static_cast<uint32_t>(calibration.max_us - calibration.neutral_us);
    const uint32_t offset =
        static_cast<uint32_t>((speed * static_cast<float>(span)) + 0.5F);
    return static_cast<uint16_t>(static_cast<uint32_t>(calibration.neutral_us) +
                                 offset);
}

uint16_t map_reverse(float speed,
                     const SpeedPwmCalibration &calibration) noexcept
{
    const uint32_t span =
        static_cast<uint32_t>(calibration.neutral_us - calibration.min_us);
    const uint32_t offset =
        static_cast<uint32_t>(((-speed) * static_cast<float>(span)) + 0.5F);
    return static_cast<uint16_t>(static_cast<uint32_t>(calibration.neutral_us) -
                                 offset);
}

} // namespace

SpeedToPwmResult speed_to_pwm(
    float normalized_speed,
    const SpeedPwmCalibration &calibration) noexcept
{
    if (!calibration_is_valid(calibration)) {
        return {0U, ConvertStatus::InvalidConfig};
    }

    if (!__builtin_isfinite(normalized_speed)) {
        return {0U, ConvertStatus::InvalidInput};
    }

    ConvertStatus status = ConvertStatus::Ok;
    float bounded_speed = calibration.reversed ? -normalized_speed
                                               : normalized_speed;

    if (bounded_speed > 1.0F) {
        bounded_speed = 1.0F;
        status = ConvertStatus::SaturatedForward;
    } else if (bounded_speed < -1.0F) {
        bounded_speed = -1.0F;
        status = ConvertStatus::SaturatedReverse;
    }

    if (bounded_speed == 0.0F) {
        return {calibration.neutral_us, ConvertStatus::Neutral};
    }

    const uint16_t pulse_us = bounded_speed > 0.0F
                                  ? map_forward(bounded_speed, calibration)
                                  : map_reverse(bounded_speed, calibration);
    return {pulse_us, status};
}

void speed_frame_to_pwm(
    const MotorSpeedFrame &input,
    const SpeedPwmCalibration (&calibration)[kMotorCount],
    MotorPwmFrame &output) noexcept
{
    MotorPwmFrame converted{};

    for (uint8_t index = 0U; index < kMotorCount; ++index) {
        converted.status[index] = ConvertStatus::NotCommanded;
        const uint8_t channel_mask = static_cast<uint8_t>(1U << index);

        if ((input.valid_mask & channel_mask) == 0U) {
            continue;
        }

        const SpeedToPwmResult result =
            speed_to_pwm(input.normalized_speed[index], calibration[index]);
        converted.pulse_us[index] = result.pulse_us;
        converted.status[index] = result.status;

        if (result_has_valid_pwm(result.status)) {
            converted.valid_mask =
                static_cast<uint8_t>(converted.valid_mask | channel_mask);
        }
    }

    output = converted;
}

} // namespace dima::lib::motor

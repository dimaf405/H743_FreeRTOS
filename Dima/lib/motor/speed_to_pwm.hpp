#pragma once

#include <stdint.h>

namespace dima::lib::motor {

inline constexpr uint8_t kMotorCount = 6U;

enum class ConvertStatus : uint8_t {
    NotCommanded,
    Neutral,
    Ok,
    // Saturation direction is reported after applying channel reversal.
    SaturatedForward,
    SaturatedReverse,
    InvalidInput,
    InvalidConfig,
};

struct SpeedPwmCalibration {
    uint16_t min_us;
    uint16_t neutral_us;
    uint16_t max_us;
    bool reversed;
};

inline constexpr SpeedPwmCalibration kDefaultSpeedPwmCalibration{
    1000U,
    1500U,
    2000U,
    false,
};

struct SpeedToPwmResult {
    uint16_t pulse_us;
    ConvertStatus status;
};

struct MotorSpeedFrame {
    float normalized_speed[kMotorCount];
    uint8_t valid_mask;
};

struct MotorPwmFrame {
    uint16_t pulse_us[kMotorCount];
    ConvertStatus status[kMotorCount];
    uint8_t valid_mask;
};

SpeedToPwmResult speed_to_pwm(
    float normalized_speed,
    const SpeedPwmCalibration &calibration) noexcept;

void speed_frame_to_pwm(
    const MotorSpeedFrame &input,
    const SpeedPwmCalibration (&calibration)[kMotorCount],
    MotorPwmFrame &output) noexcept;

} // namespace dima::lib::motor

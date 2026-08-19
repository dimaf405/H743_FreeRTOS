#pragma once

#include "uorb/uORB.hpp"

#include <cstdint>

/** 六路普通 PWM 后端的实际状态快照；零脉宽表示物理 hard-safe-off。 */
struct actuator_output_status_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 0U;
    static constexpr std::uint8_t NUM_OUTPUTS = 6U;

    static constexpr std::uint8_t STATE_STOPPED = 0U;
    static constexpr std::uint8_t STATE_HARD_SAFE_OFF = 1U;
    static constexpr std::uint8_t STATE_SAFE_OFF = STATE_HARD_SAFE_OFF;
    static constexpr std::uint8_t STATE_ACTIVE = 2U;
    static constexpr std::uint8_t STATE_RETRY = 3U;
    static constexpr std::uint8_t STATE_FAULT = 4U;
    static constexpr std::uint8_t STATE_DISARMED_NEUTRAL = 5U;

    std::uint64_t timestamp;
    std::uint64_t timestamp_sample;
    std::uint32_t sequence;
    std::uint16_t pwm_us[NUM_OUTPUTS];
    std::uint8_t active_output_mask;
    std::uint8_t configured_output_mask;
    std::uint8_t right_output_mask;
    std::uint8_t left_output_mask;
    std::uint8_t state;
    bool backend_ready;
    bool drive_available;
    bool safe_off;
    bool command_valid;
    bool parameter_update_pending;
};

ORB_DECLARE(actuator_output_status);

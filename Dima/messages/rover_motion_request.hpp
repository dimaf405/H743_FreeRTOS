#pragma once

#include "uorb/uORB.hpp"

#include <cstdint>

/**
 * Rover 产品域的两轴运动请求。
 *
 * 阶段 5 只接受 MANUAL + NORMALIZED_AXES。导航层未来只能发布本消息，
 * 不得直接访问差速混控、actuator_motors 或板级执行器。
 */
struct rover_motion_request_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 0U;

    static constexpr std::uint8_t SOURCE_MANUAL = 0U;
    static constexpr std::uint8_t SOURCE_NAVIGATION = 1U;

    static constexpr std::uint8_t MODE_NORMALIZED_AXES = 0U;
    static constexpr std::uint8_t MODE_SPEED_YAW_RATE = 1U;

    std::uint64_t timestamp;
    std::uint64_t timestamp_sample;
    std::uint32_t sequence;
    bool valid;
    std::uint8_t source;
    std::uint8_t mode;
    float normalized_longitudinal;
    float normalized_steering;
    float speed_m_s;
    float yaw_rate_rad_s;
};

ORB_DECLARE(rover_motion_request);

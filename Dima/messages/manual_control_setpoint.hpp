/****************************************************************************
 *
 *   Copyright (c) 2012-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "uorb/uORB.hpp"

#include <cstdint>

/** PX4 v1.17.0 versioned/ManualControlSetpoint.msg 的 Dima uORB 契约。 */
struct manual_control_setpoint_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 0U;
    static constexpr std::uint8_t SOURCE_UNKNOWN = 0U;
    static constexpr std::uint8_t SOURCE_RC = 1U;
    static constexpr std::uint8_t SOURCE_MAVLINK_0 = 2U;
    static constexpr std::uint8_t SOURCE_MAVLINK_1 = 3U;
    static constexpr std::uint8_t SOURCE_MAVLINK_2 = 4U;
    static constexpr std::uint8_t SOURCE_MAVLINK_3 = 5U;
    static constexpr std::uint8_t SOURCE_MAVLINK_4 = 6U;
    static constexpr std::uint8_t SOURCE_MAVLINK_5 = 7U;

    std::uint64_t timestamp;
    std::uint64_t timestamp_sample;
    bool valid;
    std::uint8_t data_source;
    float roll;                        // 摇杆量通常为 -1..1，缺失通道使用 NaN。
    float pitch;
    float yaw;
    float throttle;
    float flaps;
    float aux1;
    float aux2;
    float aux3;
    float aux4;
    float aux5;
    float aux6;
    bool sticks_moving;
    std::uint16_t buttons;
};

ORB_DECLARE(manual_control_setpoint);
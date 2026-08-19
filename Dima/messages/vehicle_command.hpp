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

/** PX4 v1.17.0 VehicleCommand.msg 的 Dima uORB 契约。
 *
 *  外部 MAVLink COMMAND_LONG 经解析后写入此 topic，
 *  Commander 订阅并裁决。深度 8。
 */
struct vehicle_command_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 1U;

    /* NAV_CMD / MAV_CMD 值直接复用 uint16 编码。 */
    static constexpr std::uint16_t NAV_CMD_DO_SET_MODE = 176U;
    static constexpr std::uint16_t NAV_CMD_COMPONENT_ARM_DISARM = 400U;
    static constexpr std::uint16_t NAV_CMD_PREFLIGHT_CALIBRATION = 241U;
    static constexpr std::uint16_t NAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246U;
    static constexpr std::uint16_t NAV_CMD_REQUEST_MESSAGE = 512U;

    /* Source origins — 标识命令来源。 */
    static constexpr std::uint8_t SOURCE_MAVLINK = 0U;
    static constexpr std::uint8_t SOURCE_INTERNAL = 1U;

    std::uint64_t timestamp;
    std::uint32_t param1_raw;     /* IEEE-754 bit pattern (float as uint32) */
    std::uint32_t param2_raw;
    std::uint32_t param3_raw;
    std::uint32_t param4_raw;
    std::uint32_t param5_raw;
    std::uint32_t param6_raw;
    std::uint32_t param7_raw;
    std::uint16_t command;
    std::uint8_t  target_system;
    std::uint8_t  target_component;
    std::uint8_t  source_system;
    std::uint8_t  source_component;
    std::uint8_t  confirmation;
    bool          from_external;
};

ORB_DECLARE(vehicle_command);

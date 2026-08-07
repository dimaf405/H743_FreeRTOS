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

/** PX4 v1.17.0 VehicleCommandAck.msg 的 Dima uORB 契约。
 *
 *  Commander 处理 vehicle_command 后发布 ACK，
 *  MAVLink 桥接层订阅并发回 MAVLink COMMAND_ACK。深度 4。
 */
struct vehicle_command_ack_s {
    static constexpr std::uint32_t MESSAGE_VERSION = 1U;

    /* MAV_RESULT 枚举值 */
    static constexpr std::uint8_t RESULT_ACCEPTED             = 0U;
    static constexpr std::uint8_t RESULT_TEMPORARILY_REJECTED = 1U;
    static constexpr std::uint8_t RESULT_DENIED               = 2U;
    static constexpr std::uint8_t RESULT_UNSUPPORTED          = 3U;
    static constexpr std::uint8_t RESULT_FAILED               = 4U;
    static constexpr std::uint8_t RESULT_IN_PROGRESS          = 5U;
    static constexpr std::uint8_t RESULT_CANCELLED            = 6U;

    std::uint64_t timestamp;
    std::uint32_t result_param2;
    std::uint16_t command;
    std::uint8_t  result;
    bool          from_external;
    std::uint8_t  target_system;
    std::uint8_t  target_component;
};

ORB_DECLARE(vehicle_command_ack);

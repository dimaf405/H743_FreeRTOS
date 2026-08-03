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

/** PX4 v1.17.0 RcChannels.msg 的 Dima uORB 契约。 */
struct rc_channels_s {
    static constexpr std::uint8_t FUNCTION_THROTTLE = 0U;
    static constexpr std::uint8_t FUNCTION_ROLL = 1U;
    static constexpr std::uint8_t FUNCTION_PITCH = 2U;
    static constexpr std::uint8_t FUNCTION_YAW = 3U;
    static constexpr std::uint8_t FUNCTION_RETURN = 4U;
    static constexpr std::uint8_t FUNCTION_LOITER = 5U;
    static constexpr std::uint8_t FUNCTION_OFFBOARD = 6U;
    static constexpr std::uint8_t FUNCTION_FLAPS = 7U;
    static constexpr std::uint8_t FUNCTION_AUX_1 = 8U;
    static constexpr std::uint8_t FUNCTION_AUX_2 = 9U;
    static constexpr std::uint8_t FUNCTION_AUX_3 = 10U;
    static constexpr std::uint8_t FUNCTION_AUX_4 = 11U;
    static constexpr std::uint8_t FUNCTION_AUX_5 = 12U;
    static constexpr std::uint8_t FUNCTION_AUX_6 = 13U;
    static constexpr std::uint8_t FUNCTION_PARAM_1 = 14U;
    static constexpr std::uint8_t FUNCTION_PARAM_2 = 15U;
    static constexpr std::uint8_t FUNCTION_PARAM_3_5 = 16U;
    static constexpr std::uint8_t FUNCTION_KILLSWITCH = 17U;
    static constexpr std::uint8_t FUNCTION_TRANSITION = 18U;
    static constexpr std::uint8_t FUNCTION_GEAR = 19U;
    static constexpr std::uint8_t FUNCTION_ARMSWITCH = 20U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_1 = 21U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_2 = 22U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_3 = 23U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_4 = 24U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_5 = 25U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_6 = 26U;
    static constexpr std::uint8_t FUNCTION_ENGAGE_MAIN_MOTOR = 27U;
    static constexpr std::uint8_t FUNCTION_PAYLOAD_POWER = 28U;
    static constexpr std::uint8_t FUNCTION_TERMINATION = 29U;
    static constexpr std::uint8_t FUNCTION_FLTBTN_SLOT_COUNT = 6U;

    std::uint64_t timestamp;
    std::uint64_t timestamp_last_valid;
    float channels[18];     // 所有物理通道统一标准化为 -1..1。
    std::uint8_t channel_count;
    std::int8_t function[30];
    std::uint8_t rssi;
    bool signal_lost;
    std::uint32_t frame_drop_count;
};

ORB_DECLARE(rc_channels);
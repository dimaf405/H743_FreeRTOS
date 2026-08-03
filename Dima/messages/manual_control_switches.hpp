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

/** PX4 v1.17.0 ManualControlSwitches.msg 的 Dima uORB 契约。 */
struct manual_control_switches_s {
    static constexpr std::uint8_t SWITCH_POS_NONE = 0U;
    static constexpr std::uint8_t SWITCH_POS_ON = 1U;
    static constexpr std::uint8_t SWITCH_POS_MIDDLE = 2U;
    static constexpr std::uint8_t SWITCH_POS_OFF = 3U;
    static constexpr std::uint8_t MODE_SLOT_NONE = 0U;
    static constexpr std::uint8_t MODE_SLOT_1 = 1U;
    static constexpr std::uint8_t MODE_SLOT_2 = 2U;
    static constexpr std::uint8_t MODE_SLOT_3 = 3U;
    static constexpr std::uint8_t MODE_SLOT_4 = 4U;
    static constexpr std::uint8_t MODE_SLOT_5 = 5U;
    static constexpr std::uint8_t MODE_SLOT_6 = 6U;
    static constexpr std::uint8_t MODE_SLOT_NUM = 6U;

    std::uint64_t timestamp;
    std::uint64_t timestamp_sample;
    std::uint8_t mode_slot;
    std::uint8_t arm_switch;
    std::uint8_t return_switch;
    std::uint8_t loiter_switch;
    std::uint8_t offboard_switch;
    std::uint8_t kill_switch;
    std::uint8_t termination_switch;
    std::uint8_t gear_switch;
    std::uint8_t transition_switch;
    std::uint8_t photo_switch;
    std::uint8_t video_switch;
    std::uint8_t engage_main_motor_switch;
    std::uint8_t payload_power_switch;
    std::uint32_t switch_changes;      // 任一离散开关状态变化时递增。
};

ORB_DECLARE(manual_control_switches);
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

/** PX4 v1.17.0 ActionRequest.msg 的 Dima uORB 契约。 */
struct action_request_s {
    static constexpr std::uint8_t ACTION_DISARM = 0U;
    static constexpr std::uint8_t ACTION_ARM = 1U;
    static constexpr std::uint8_t ACTION_TOGGLE_ARMING = 2U;
    static constexpr std::uint8_t ACTION_UNKILL = 3U;
    static constexpr std::uint8_t ACTION_KILL = 4U;
    static constexpr std::uint8_t ACTION_SWITCH_MODE = 5U;
    static constexpr std::uint8_t ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6U;
    static constexpr std::uint8_t ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7U;
    static constexpr std::uint8_t ACTION_TERMINATION = 8U;
    static constexpr std::uint8_t SOURCE_STICK_GESTURE = 0U;
    static constexpr std::uint8_t SOURCE_RC_SWITCH = 1U;
    static constexpr std::uint8_t SOURCE_RC_BUTTON = 2U;
    static constexpr std::uint8_t SOURCE_RC_MODE_SLOT = 3U;

    std::uint64_t timestamp;
    std::uint8_t action;
    std::uint8_t source;
    std::uint8_t mode;                 // 仅 ACTION_SWITCH_MODE 使用该字段。
};

ORB_DECLARE(action_request);
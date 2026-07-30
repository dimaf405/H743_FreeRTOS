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

#include "Dima/middleware/uorb/uORB.hpp"

#include <cstdint>

/** PX4 v1.17.0 InputRc.msg 的 Dima uORB 契约。 */
struct input_rc_s {
    static constexpr std::uint8_t RC_INPUT_SOURCE_UNKNOWN = 0U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_PPM = 1U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4IO_PPM = 2U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4IO_SBUS = 4U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4IO_ST24 = 5U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_MAVLINK = 6U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_QURT = 7U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_SBUS = 9U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_ST24 = 10U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_SUMD = 11U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_DSM = 12U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4IO_SUMD = 13U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_CRSF = 14U;
    static constexpr std::uint8_t RC_INPUT_SOURCE_PX4FMU_GHST = 15U;
    static constexpr std::uint8_t RC_INPUT_MAX_CHANNELS = 18U;
    static constexpr std::int8_t RSSI_MAX = 100;

    std::uint64_t timestamp;
    std::uint64_t timestamp_last_signal;
    std::uint8_t channel_count;
    std::int32_t rssi;                 // 小于 0 表示无有效 RSSI。
    bool rc_failsafe;
    bool rc_lost;
    std::uint16_t rc_lost_frame_count;
    std::uint16_t rc_total_frame_count;
    std::uint16_t rc_ppm_frame_length;
    std::uint8_t input_source;
    std::uint16_t values[RC_INPUT_MAX_CHANNELS];
    std::int8_t link_quality;          // 0..100，-1 表示无效。
    float rssi_dbm;                    // NaN 表示无效。
};

ORB_DECLARE(input_rc);
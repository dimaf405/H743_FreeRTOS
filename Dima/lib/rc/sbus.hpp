/****************************************************************************
 *
 * Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::rc {

// 来源：PX4-Autopilot v1.17.0 src/lib/rc/sbus.{h,cpp}，仅保留平台无关接收解析逻辑。
class SbusParser {
public:
    static constexpr std::size_t kFrameSize = 25U;
    static constexpr std::size_t kAnalogChannelCount = 16U;
    static constexpr std::size_t kChannelCount = 18U;
    static constexpr std::uint64_t kResyncGapUs = 4000U;

    struct Frame {
        std::uint16_t values[kChannelCount]{};
        std::uint8_t channel_count{0U};
        bool failsafe{false};
        bool frame_lost{false};
    };

    struct Stats {
        std::uint32_t bytes_received{0U};
        std::uint32_t valid_frames{0U};
        std::uint32_t dropped_frames{0U};
        std::uint32_t discarded_bytes{0U};
        std::uint32_t invalid_headers{0U};
        std::uint32_t invalid_footers{0U};
        std::uint32_t resyncs{0U};
        std::uint32_t frame_lost_flags{0U};
        std::uint32_t failsafe_frames{0U};
    };

    bool parse(std::uint64_t byte_arrival_us, std::uint8_t byte,
               Frame &frame) noexcept;
    void reset() noexcept;
    const Stats &stats() const noexcept { return stats_; }

private:
    bool decode(Frame &frame) noexcept;
    void recover_after_invalid_frame() noexcept;

    std::uint8_t frame_[kFrameSize]{};
    std::size_t frame_length_{0U};
    std::uint64_t last_rx_time_us_{0U};
    Stats stats_{};
};

} // namespace dima::rc

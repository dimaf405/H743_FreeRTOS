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

namespace dima::protocols::sbus {

// 协议层来源：PX4-Autopilot v1.17.0 src/lib/rc/sbus.{h,cpp}，仅保留平台无关接收解析逻辑。
// 解析器只拥有 25 B 帧同步与通道解包，不拥有 UART、反相、DMA、uORB 或
// COM_RC_LOSS_T。逐字节到达时间由平台后端提供，因而能用真实线间隔重同步。
class SbusParser {
public:
    // 16 路 11-bit 模拟量 + 2 路数字量，共 18 通道。100 kbit/s 8E2 每字节
    // 12 bit，一帧约 25*12/100000=3 ms；4 ms 静默可可靠判定新帧边界。
    static constexpr std::size_t kFrameSize = 25U;
    static constexpr std::size_t kAnalogChannelCount = 16U;
    static constexpr std::size_t kChannelCount = 18U;
    static constexpr std::uint64_t kResyncGapUs = 4000U;

    struct Frame {
        // values 已映射为约 1000..2000 us 的 PWM 语义值；frame_lost 表示接收机
        // 报告跳帧，failsafe 表示接收机进入失效保护，两者不能混同为串口掉线。
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

    // 每次输入一个带 ISR 时间戳的字节；仅完整且头尾合法的 25 B 帧返回 true。
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

} // namespace dima::protocols::sbus

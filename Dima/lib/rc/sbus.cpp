/****************************************************************************
 *
 * Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
 *
 * 本文件由 PX4-Autopilot v1.17.0 SBUS 接收解析器裁剪而来；保留原 BSD-3-Clause
 * 来源信息，并移除串口、POSIX、RTOS、uORB 与 SBUS 输出依赖。
 *
 ****************************************************************************/

#include "sbus.hpp"

namespace dima::rc {
namespace {

constexpr std::uint8_t kHeader = 0x0FU;
constexpr std::size_t kFlagsIndex = 23U;
constexpr std::uint8_t kDigital17Mask = 1U << 0U;
constexpr std::uint8_t kDigital18Mask = 1U << 1U;
constexpr std::uint8_t kFrameLostMask = 1U << 2U;
constexpr std::uint8_t kFailsafeMask = 1U << 3U;

bool valid_footer(std::uint8_t footer) noexcept
{
    // PX4 v1.17.0 同时接受 S.BUS1 尾字节和四种 S.BUS2 后续时隙标记。
    return footer == 0x00U || footer == 0x04U || footer == 0x14U ||
           footer == 0x24U || footer == 0x34U;
}

std::uint16_t raw_to_pwm(std::uint16_t raw) noexcept
{
    // PX4 映射：保持 PX4 的 200..1800 到约 999..1999 PWM 映射与整数四舍五入语义。
    return static_cast<std::uint16_t>(((static_cast<std::uint32_t>(raw) * 5U) + 4U) / 8U + 874U);
}

std::uint16_t unpack_11_bits(const std::uint8_t *payload,
                             std::size_t channel) noexcept
{
    const std::size_t bit_offset = channel * 11U;
    const std::size_t byte_offset = bit_offset / 8U;
    const std::uint8_t shift = static_cast<std::uint8_t>(bit_offset % 8U);
    std::uint32_t packed = payload[byte_offset];
    packed |= static_cast<std::uint32_t>(payload[byte_offset + 1U]) << 8U;

    if (byte_offset + 2U < 22U) {
        packed |= static_cast<std::uint32_t>(payload[byte_offset + 2U]) << 16U;
    }

    return static_cast<std::uint16_t>((packed >> shift) & 0x07FFU);
}

} // namespace

bool SbusParser::parse(std::uint64_t now_us, const std::uint8_t *data,
                       std::size_t length, Frame &latest_frame) noexcept
{
    if (data == nullptr || length == 0U) {
        return false;
    }

    // 25 字节在 100 kbit/s、8E2 下约占 3 ms；超过 4 ms 的调用间隔视为新帧边界。
    if (last_rx_time_us_ != 0U && now_us - last_rx_time_us_ > kResyncGapUs &&
        frame_length_ != 0U) {
        frame_length_ = 0U;
        ++stats_.resyncs;
        ++stats_.dropped_frames;
    }

    last_rx_time_us_ = now_us;
    stats_.bytes_received += static_cast<std::uint32_t>(length);
    bool decoded = false;

    for (std::size_t index = 0U; index < length; ++index) {
        const std::uint8_t byte = data[index];

        if (frame_length_ == 0U) {
            if (byte != kHeader) {
                ++stats_.discarded_bytes;
                continue;
            }
        }

        frame_[frame_length_++] = byte;

        if (frame_length_ == kFrameSize) {
            Frame candidate{};

            if (decode(candidate)) {
                latest_frame = candidate;
                decoded = true;
                frame_length_ = 0U;

            } else {
                recover_after_invalid_frame();
            }
        }
    }

    return decoded;
}

void SbusParser::reset() noexcept
{
    frame_length_ = 0U;
    last_rx_time_us_ = 0U;
    stats_ = Stats{};
}

bool SbusParser::decode(Frame &frame) noexcept
{
    if (frame_[0] != kHeader) {
        ++stats_.invalid_headers;
        ++stats_.dropped_frames;
        return false;
    }

    if (!valid_footer(frame_[kFrameSize - 1U])) {
        ++stats_.invalid_footers;
        ++stats_.dropped_frames;
        return false;
    }

    const std::uint8_t *const payload = &frame_[1];

    for (std::size_t channel = 0U; channel < kAnalogChannelCount; ++channel) {
        frame.values[channel] = raw_to_pwm(unpack_11_bits(payload, channel));
    }

    const std::uint8_t flags = frame_[kFlagsIndex];
    frame.values[16] = (flags & kDigital17Mask) != 0U ? 1998U : 998U;
    frame.values[17] = (flags & kDigital18Mask) != 0U ? 1998U : 998U;
    frame.channel_count = static_cast<std::uint8_t>(kChannelCount);
    frame.failsafe = (flags & kFailsafeMask) != 0U;
    frame.frame_lost = frame.failsafe || (flags & kFrameLostMask) != 0U;

    ++stats_.valid_frames;

    if (frame.frame_lost) {
        ++stats_.frame_lost_flags;
    }

    if (frame.failsafe) {
        ++stats_.failsafe_frames;
    }

    return true;
}

void SbusParser::recover_after_invalid_frame() noexcept
{
    // 与 PX4 的偏移恢复一致：优先保留候选帧内第二个 0x0F 及其后续字节。
    std::size_t next_header = 0U;

    for (std::size_t index = 1U; index < frame_length_; ++index) {
        if (frame_[index] == kHeader) {
            next_header = index;
            break;
        }
    }

    if (next_header == 0U) {
        frame_length_ = 0U;
        return;
    }

    const std::size_t remaining = frame_length_ - next_header;

    for (std::size_t index = 0U; index < remaining; ++index) {
        frame_[index] = frame_[next_header + index];
    }

    frame_length_ = remaining;
    ++stats_.resyncs;
}

} // namespace dima::rc

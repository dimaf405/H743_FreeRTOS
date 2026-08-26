/****************************************************************************
 * PX4-Autopilot v1.17.0 ICM42688P FIFO algorithms adapted for Dima.
 * Upstream: src/drivers/imu/invensense/icm42688p
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "ICM42688PRegisters.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace dima::drivers::imu::icm42688p::fifo {

using registers::FifoPacket;

enum class BatchStatus : std::uint8_t {
    Ready = 0U,
    Empty,
    Invalid,
    Overflow,
};

struct BatchPlan {
    BatchStatus status{BatchStatus::Invalid};
    std::size_t samples{0U};
    bool has_more{false};
};

constexpr BatchPlan plan_batch(std::uint8_t interrupt_status,
                               std::uint16_t fifo_count) noexcept
{
    if ((interrupt_status & registers::bits::FIFO_FULL_INT) != 0U ||
        fifo_count >= registers::kFifoBytes) {
        return {BatchStatus::Overflow, 0U, false};
    }
    if (fifo_count == 0U) {
        return {BatchStatus::Empty, 0U, false};
    }

    /* 完整样本数=floor(fifo_count/20)。单次最多取 watermark 的 10 个；余量通过
     * has_more 立即续读。少于一个完整包但非零视为 Invalid，不拼接半包。 */
    const std::size_t available_samples =
        fifo_count / sizeof(FifoPacket);
    if (available_samples == 0U) {
        return {BatchStatus::Invalid, 0U, false};
    }
    const std::size_t samples =
        available_samples < registers::kWatermarkSamples
            ? available_samples
            : registers::kWatermarkSamples;
    return {BatchStatus::Ready, samples, available_samples > samples};
}

struct DecodedSample {
    std::int32_t accel[3]{};
    std::int32_t gyro[3]{};
    std::int16_t temperature{0};
    std::uint16_t timestamp_ticks{0U};
};

constexpr bool decode_sample(const FifoPacket &packet,
                             DecodedSample &output) noexcept
{
    if (!registers::valid_fifo_header(packet.header)) {
        return false;
    }

    /* accel extension 取高 nibble，gyro combine_i20 内部再取低 nibble；芯片用
     * -2^19 表示无效高分辨率样本，温度 INT16_MIN 同样是无效哨兵。 */
    DecodedSample decoded{};
    decoded.accel[0] = registers::combine_i20(
        packet.accel_x_high, packet.accel_x_low,
        static_cast<std::uint8_t>((packet.extension_x >> 4U) & 0x0FU));
    decoded.accel[1] = registers::combine_i20(
        packet.accel_y_high, packet.accel_y_low,
        static_cast<std::uint8_t>((packet.extension_y >> 4U) & 0x0FU));
    decoded.accel[2] = registers::combine_i20(
        packet.accel_z_high, packet.accel_z_low,
        static_cast<std::uint8_t>((packet.extension_z >> 4U) & 0x0FU));
    decoded.gyro[0] = registers::combine_i20(
        packet.gyro_x_high, packet.gyro_x_low, packet.extension_x);
    decoded.gyro[1] = registers::combine_i20(
        packet.gyro_y_high, packet.gyro_y_low, packet.extension_y);
    decoded.gyro[2] = registers::combine_i20(
        packet.gyro_z_high, packet.gyro_z_low, packet.extension_z);
    decoded.temperature = registers::combine_i16(
        packet.temperature_high, packet.temperature_low);
    decoded.timestamp_ticks = registers::combine_u16(
        packet.timestamp_high, packet.timestamp_low);

    constexpr std::int32_t kInvalidHighResolutionSample = -524288;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        if (decoded.accel[axis] == kInvalidHighResolutionSample ||
            decoded.gyro[axis] == kInvalidHighResolutionSample) {
            return false;
        }
    }
    if (decoded.temperature == std::numeric_limits<std::int16_t>::min()) {
        return false;
    }

    output = decoded;
    return true;
}

constexpr bool requires_wide_scale(
    const std::int32_t (&values)[3]) noexcept
{
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        /* 任一 20-bit 值无法安全压入窄格式时，整批三轴统一使用 16-bit high word
         * 与 wide scale，避免同一消息内各轴量纲不同。 */
        if (values[axis] <= std::numeric_limits<std::int16_t>::min() ||
            values[axis] >= std::numeric_limits<std::int16_t>::max()) {
            return true;
        }
    }
    return false;
}

constexpr std::int16_t compact_accel(std::int32_t raw,
                                     std::int16_t high_word,
                                     bool wide) noexcept
{
    return wide ? high_word
                : static_cast<std::int16_t>(raw / 4);
}

constexpr std::int16_t compact_gyro(std::int32_t raw,
                                    std::int16_t high_word,
                                    bool wide) noexcept
{
    return wide ? high_word
                : static_cast<std::int16_t>(raw / 2);
}

constexpr float accel_scale(bool wide) noexcept
{
    constexpr float kOneG = 9.80665F;
    /* wide 保存 raw>>4，scale=g/2048；narrow 保存 raw/4，scale=g/8192，二者
     * 理想物理值均为 raw*g/32768，单位 m/s²。 */
    return wide ? kOneG / 2048.0F : kOneG / 8192.0F;
}

constexpr float gyro_scale(bool wide) noexcept
{
    constexpr float kRadiansPerDegree = 0.01745329251994329577F;
    /* wide 对应 ±2000 deg/s 的 16-bit high word；narrow 用 131 LSB/(deg/s)。
     * 最终统一转换为 rad/s。 */
    return wide ? (2000.0F * kRadiansPerDegree) / 32768.0F
                : kRadiansPerDegree / 131.0F;
}

} // namespace dima::drivers::imu::icm42688p::fifo

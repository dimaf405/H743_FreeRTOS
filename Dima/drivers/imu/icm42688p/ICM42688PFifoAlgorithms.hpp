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

BatchPlan plan_batch(std::uint8_t interrupt_status,
                               std::uint16_t fifo_count) noexcept;

struct DecodedSample {
    std::int32_t accel[3]{};
    std::int32_t gyro[3]{};
    std::int16_t temperature{0};
    std::uint16_t timestamp_ticks{0U};
};

bool decode_sample(const FifoPacket &packet,
                             DecodedSample &output) noexcept;

bool requires_wide_scale(
    const std::int32_t (&values)[3]) noexcept;

std::int16_t compact_accel(std::int32_t raw,
                                     std::int16_t high_word,
                                     bool wide) noexcept;

std::int16_t compact_gyro(std::int32_t raw,
                                    std::int16_t high_word,
                                    bool wide) noexcept;

float accel_scale(bool wide) noexcept;

float gyro_scale(bool wide) noexcept;

} // namespace dima::drivers::imu::icm42688p::fifo

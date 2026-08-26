/****************************************************************************
 * PX4-Autopilot v1.17.0 ICM42688P register/FIFO contract adapted for Dima.
 * Upstream: src/drivers/imu/invensense/icm42688p
 * @ d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::drivers::imu::icm42688p::registers {

constexpr std::uint8_t bit(std::uint8_t index) noexcept
{
    return static_cast<std::uint8_t>(1U << index);
}

constexpr std::uint8_t kRead = 0x80U;
constexpr std::uint8_t kWhoAmI = 0x47U;
constexpr std::size_t kFifoBytes = 2048U;
constexpr std::size_t kWatermarkSamples = 10U;
constexpr std::size_t kFifoPacketBytes = 20U;
constexpr float kTemperatureSensitivity = 132.48F;
constexpr float kTemperatureOffsetC = 25.0F;
/* FIFO timestamp 1 LSB = 16 us * 32/30 ≈ 17.0667 us（芯片 TMST 校准合同）。 */
constexpr float kTimestampTickUs = 16.0F * (32.0F / 30.0F);

enum class Bank : std::uint8_t {
    Bank0 = 0U,
    Bank1 = 1U,
    Bank2 = 2U,
};

namespace bank0 {
constexpr std::uint8_t DEVICE_CONFIG = 0x11U;
constexpr std::uint8_t INT_CONFIG = 0x14U;
constexpr std::uint8_t FIFO_CONFIG = 0x16U;
constexpr std::uint8_t INT_STATUS = 0x2DU;
constexpr std::uint8_t FIFO_COUNTH = 0x2EU;
constexpr std::uint8_t FIFO_COUNTL = 0x2FU;
constexpr std::uint8_t FIFO_DATA = 0x30U;
constexpr std::uint8_t SIGNAL_PATH_RESET = 0x4BU;
constexpr std::uint8_t INTF_CONFIG0 = 0x4CU;
constexpr std::uint8_t INTF_CONFIG1 = 0x4DU;
constexpr std::uint8_t PWR_MGMT0 = 0x4EU;
constexpr std::uint8_t GYRO_CONFIG0 = 0x4FU;
constexpr std::uint8_t ACCEL_CONFIG0 = 0x50U;
constexpr std::uint8_t GYRO_CONFIG1 = 0x51U;
constexpr std::uint8_t GYRO_ACCEL_CONFIG0 = 0x52U;
constexpr std::uint8_t ACCEL_CONFIG1 = 0x53U;
constexpr std::uint8_t TMST_CONFIG = 0x54U;
constexpr std::uint8_t FIFO_CONFIG1 = 0x5FU;
constexpr std::uint8_t FIFO_CONFIG2 = 0x60U;
constexpr std::uint8_t FIFO_CONFIG3 = 0x61U;
constexpr std::uint8_t INT_CONFIG0 = 0x63U;
constexpr std::uint8_t INT_CONFIG1 = 0x64U;
constexpr std::uint8_t INT_SOURCE0 = 0x65U;
constexpr std::uint8_t WHO_AM_I = 0x75U;
constexpr std::uint8_t REG_BANK_SEL = 0x76U;
} // namespace bank0

namespace bank1 {
constexpr std::uint8_t GYRO_CONFIG_STATIC2 = 0x0BU;
constexpr std::uint8_t GYRO_CONFIG_STATIC3 = 0x0CU;
constexpr std::uint8_t GYRO_CONFIG_STATIC4 = 0x0DU;
constexpr std::uint8_t GYRO_CONFIG_STATIC5 = 0x0EU;
} // namespace bank1

namespace bank2 {
constexpr std::uint8_t ACCEL_CONFIG_STATIC2 = 0x03U;
constexpr std::uint8_t ACCEL_CONFIG_STATIC3 = 0x04U;
constexpr std::uint8_t ACCEL_CONFIG_STATIC4 = 0x05U;
} // namespace bank2

namespace bits {
constexpr std::uint8_t SOFT_RESET = bit(0U);
constexpr std::uint8_t RESET_DONE_INT = bit(4U);
constexpr std::uint8_t FIFO_THRESHOLD_INT = bit(2U);
constexpr std::uint8_t FIFO_FULL_INT = bit(1U);
constexpr std::uint8_t FIFO_FLUSH = bit(1U);

constexpr std::uint8_t INT1_MODE_PULSED = bit(2U);
constexpr std::uint8_t INT1_PUSH_PULL = bit(1U);
constexpr std::uint8_t INT1_ACTIVE_HIGH = bit(0U);
constexpr std::uint8_t FIFO_STOP_ON_FULL = bit(7U) | bit(6U);
constexpr std::uint8_t FIFO_COUNT_BIG_ENDIAN = bit(5U);
constexpr std::uint8_t SENSOR_DATA_BIG_ENDIAN = bit(4U);
constexpr std::uint8_t DISABLE_I2C = bit(1U) | bit(0U);
constexpr std::uint8_t DISABLE_AFSR_SET = bit(6U);
constexpr std::uint8_t DISABLE_AFSR_CLEAR = bit(7U);
constexpr std::uint8_t GYRO_LOW_NOISE = bit(3U) | bit(2U);
constexpr std::uint8_t ACCEL_LOW_NOISE = bit(1U) | bit(0U);
constexpr std::uint8_t ODR_8KHZ_SET = bit(1U) | bit(0U);
constexpr std::uint8_t ODR_8KHZ_CLEAR = bit(3U) | bit(2U);
constexpr std::uint8_t GYRO_UI_FILTER_ORDER = bit(3U) | bit(2U);
constexpr std::uint8_t ACCEL_UI_FILTER_ORDER = bit(4U) | bit(3U);
constexpr std::uint8_t ACCEL_UI_FILTER_BW = 0xF0U;
constexpr std::uint8_t GYRO_UI_FILTER_BW = 0x0FU;
constexpr std::uint8_t TIMESTAMP_ENABLE = bit(0U) | bit(2U) | bit(3U) |
                                           bit(4U);
constexpr std::uint8_t TIMESTAMP_FSYNC = bit(1U);
constexpr std::uint8_t FIFO_ENABLE = bit(5U) | bit(4U) | bit(2U) |
                                     bit(1U) | bit(0U);
constexpr std::uint8_t FIFO_TIMESTAMP_FSYNC = bit(3U);
constexpr std::uint8_t CLEAR_FIFO_INT_ON_READ = bit(3U);
constexpr std::uint8_t INT_ASYNC_RESET = bit(4U);
constexpr std::uint8_t FIFO_THRESHOLD_TO_INT1 = bit(2U);

constexpr std::uint8_t GYRO_NOTCH_DISABLE = bit(0U);
constexpr std::uint8_t GYRO_AAF_DISABLE = bit(1U);
constexpr std::uint8_t AAF_DELTA_585_SET = bit(3U) | bit(2U) | bit(0U);
constexpr std::uint8_t AAF_DELTA_585_CLEAR = bit(5U) | bit(4U) | bit(1U);
constexpr std::uint8_t AAF_DELTA_SQUARED_LSB_SET =
    bit(7U) | bit(5U) | bit(3U) | bit(1U);
constexpr std::uint8_t AAF_DELTA_SQUARED_LSB_CLEAR =
    bit(6U) | bit(4U) | bit(2U) | bit(0U);
constexpr std::uint8_t AAF_BITSHIFT_SET = bit(7U);
constexpr std::uint8_t AAF_BITSHIFT_CLEAR = bit(6U) | bit(5U) | bit(4U);
constexpr std::uint8_t AAF_DELTA_SQUARED_MSB_CLEAR = 0x0FU;
constexpr std::uint8_t ACCEL_AAF_DELTA_585_SET =
    bit(4U) | bit(3U) | bit(1U);
constexpr std::uint8_t ACCEL_AAF_DELTA_585_CLEAR =
    bit(6U) | bit(5U) | bit(2U) | bit(0U);

constexpr std::uint8_t FIFO_HEADER_MESSAGE = bit(7U);
constexpr std::uint8_t FIFO_HEADER_ACCEL = bit(6U);
constexpr std::uint8_t FIFO_HEADER_GYRO = bit(5U);
constexpr std::uint8_t FIFO_HEADER_20BIT = bit(4U);
constexpr std::uint8_t FIFO_HEADER_TIMESTAMP_MASK = bit(3U) | bit(2U);
constexpr std::uint8_t FIFO_HEADER_ODR_TIMESTAMP = bit(3U);
constexpr std::uint8_t FIFO_HEADER_ODR_ACCEL = bit(1U);
constexpr std::uint8_t FIFO_HEADER_ODR_GYRO = bit(0U);
} // namespace bits

#pragma pack(push, 1)
/* 高分辨率 FIFO 包严格 20 B、线序大端。extension 每字节高半 nibble 属于 accel，
 * 低半 nibble 属于 gyro；任何填充都会破坏 DMA 批量 reinterpret。 */
struct FifoPacket {
    std::uint8_t header;
    std::uint8_t accel_x_high;
    std::uint8_t accel_x_low;
    std::uint8_t accel_y_high;
    std::uint8_t accel_y_low;
    std::uint8_t accel_z_high;
    std::uint8_t accel_z_low;
    std::uint8_t gyro_x_high;
    std::uint8_t gyro_x_low;
    std::uint8_t gyro_y_high;
    std::uint8_t gyro_y_low;
    std::uint8_t gyro_z_high;
    std::uint8_t gyro_z_low;
    std::uint8_t temperature_high;
    std::uint8_t temperature_low;
    std::uint8_t timestamp_high;
    std::uint8_t timestamp_low;
    std::uint8_t extension_x;
    std::uint8_t extension_y;
    std::uint8_t extension_z;
};
#pragma pack(pop)

static_assert(sizeof(FifoPacket) == kFifoPacketBytes);

constexpr std::uint16_t combine_u16(std::uint8_t high,
                                    std::uint8_t low) noexcept
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(high) << 8U) | low;
}

constexpr std::int16_t combine_i16(std::uint8_t high,
                                   std::uint8_t low) noexcept
{
    return static_cast<std::int16_t>(combine_u16(high, low));
}

constexpr std::int32_t combine_i20(std::uint8_t high,
                                   std::uint8_t low,
                                   std::uint8_t extension) noexcept
{
    /* 20-bit 二补码：value=(high<<12)|(low<<4)|(ext&0xf)，high.bit7 为符号，
     * 负数用 0xfff00000 扩展到 int32。 */
    std::uint32_t value =
        (static_cast<std::uint32_t>(high) << 12U) |
        (static_cast<std::uint32_t>(low) << 4U) |
        (extension & 0x0FU);
    if ((high & 0x80U) != 0U) {
        value |= 0xFFF00000UL;
    }
    return static_cast<std::int32_t>(value);
}

constexpr bool valid_fifo_header(std::uint8_t header) noexcept
{
    /* 只接受 accel+gyro+20bit+ODR timestamp 数据包；message、FSYNC/其他 timestamp
     * 形式和 ODR change 位均拒绝，避免按错误布局解析。 */
    const std::uint8_t required = bits::FIFO_HEADER_ACCEL |
                                  bits::FIFO_HEADER_GYRO |
                                  bits::FIFO_HEADER_20BIT;
    return (header & bits::FIFO_HEADER_MESSAGE) == 0U &&
           (header & required) == required &&
           (header & bits::FIFO_HEADER_TIMESTAMP_MASK) ==
               bits::FIFO_HEADER_ODR_TIMESTAMP &&
           (header & (bits::FIFO_HEADER_ODR_ACCEL |
                      bits::FIFO_HEADER_ODR_GYRO)) == 0U;
}

} // namespace dima::drivers::imu::icm42688p::registers

#include "ICM42688PRegisters.hpp"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::drivers::imu::icm42688p::registers {

std::uint16_t combine_u16(std::uint8_t high,
                                    std::uint8_t low) noexcept
{
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(high) << 8U) | low;
}

std::int16_t combine_i16(std::uint8_t high,
                                   std::uint8_t low) noexcept
{
    return static_cast<std::int16_t>(combine_u16(high, low));
}

std::int32_t combine_i20(std::uint8_t high,
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

bool valid_fifo_header(std::uint8_t header) noexcept
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

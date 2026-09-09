#include "GpsErrorCounter.hpp"

#include <limits>


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

namespace dima::drivers::gps {

void GpsErrorCounter::begin_session(std::uint32_t receive_errors,
                       std::uint32_t dropped_bytes) noexcept
{
    total_ = 0U;
    uart_receive_errors_ = 0U;
    uart_dropped_bytes_ = 0U;
    last_receive_errors_ = receive_errors;
    last_dropped_bytes_ = dropped_bytes;
}

void GpsErrorCounter::record(std::uint32_t increment) noexcept
{
    total_ = saturating_add(total_, increment);
}

void GpsErrorCounter::update_uart(std::uint32_t receive_errors,
                     std::uint32_t dropped_bytes) noexcept
{
    const std::uint32_t receive_increment =
        counter_increment(receive_errors, last_receive_errors_);
    const std::uint32_t dropped_increment =
        counter_increment(dropped_bytes, last_dropped_bytes_);
    record(receive_increment);
    record(dropped_increment);
    uart_receive_errors_ =
        saturating_add(uart_receive_errors_, receive_increment);
    uart_dropped_bytes_ =
        saturating_add(uart_dropped_bytes_, dropped_increment);
    last_receive_errors_ = receive_errors;
    last_dropped_bytes_ = dropped_bytes;
}

std::uint32_t GpsErrorCounter::total() const noexcept
{ return total_; }

std::uint32_t GpsErrorCounter::uart_receive_errors() const noexcept
{
    return uart_receive_errors_;
}

std::uint32_t GpsErrorCounter::uart_dropped_bytes() const noexcept
{
    return uart_dropped_bytes_;
}

std::uint32_t GpsErrorCounter::saturating_add(
        std::uint32_t value, std::uint32_t increment) noexcept
{
    // 饱和加法：increment > UINT32_MAX - value 时钳位到 UINT32_MAX。
    constexpr std::uint32_t maximum =
        std::numeric_limits<std::uint32_t>::max();
    return increment > maximum - value ? maximum : value + increment;
}

std::uint32_t GpsErrorCounter::counter_increment(
        std::uint32_t current, std::uint32_t previous) noexcept
{
    // HAL 计数器跨驱动会话累计；本层只计入相对会话基线的增量，
    // 若底层计数器复位，则从新的较小值继续累计而不产生无符号下溢。
    return current >= previous ? current - previous : current;
}

} // namespace dima::drivers::gps

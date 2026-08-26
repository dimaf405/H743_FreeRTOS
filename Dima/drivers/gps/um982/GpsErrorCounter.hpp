#pragma once

#include <cstdint>
#include <limits>

namespace dima::drivers::gps {

class GpsErrorCounter final {
public:
    // UART 统计量由底层端口跨驱动生命周期累计。每次重新探测或切换波特率后，
    // 先保存新的会话基线，避免把错误波特率探测期间的预期错误算入运行期健康度。
    void begin_session(std::uint32_t receive_errors,
                       std::uint32_t dropped_bytes) noexcept
    {
        total_ = 0U;
        uart_receive_errors_ = 0U;
        uart_dropped_bytes_ = 0U;
        last_receive_errors_ = receive_errors;
        last_dropped_bytes_ = dropped_bytes;
    }

    // total_ 是 DataValidator 使用的单调累计错误输入；饱和而不回绕，
    // 防止长时间运行后 UINT32 溢出突然把错误密度解释为“恢复健康”。
    void record(std::uint32_t increment = 1U) noexcept
    {
        total_ = saturating_add(total_, increment);
    }

    // 将底层“自启动以来累计值”转换为“本 GPS 会话新增值”，并分别保留
    // UART 接收错误和丢字节，供离线边沿日志区分线路错误与缓冲区拥塞。
    void update_uart(std::uint32_t receive_errors,
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

    constexpr std::uint32_t total() const noexcept { return total_; }
    constexpr std::uint32_t uart_receive_errors() const noexcept
    {
        return uart_receive_errors_;
    }
    constexpr std::uint32_t uart_dropped_bytes() const noexcept
    {
        return uart_dropped_bytes_;
    }

private:
    static constexpr std::uint32_t saturating_add(
        std::uint32_t value, std::uint32_t increment) noexcept
    {
        // 饱和加法：increment > UINT32_MAX - value 时钳位到 UINT32_MAX。
        constexpr std::uint32_t maximum =
            std::numeric_limits<std::uint32_t>::max();
        return increment > maximum - value ? maximum : value + increment;
    }

    static constexpr std::uint32_t counter_increment(
        std::uint32_t current, std::uint32_t previous) noexcept
    {
        // HAL 计数器跨驱动会话累计；本层只计入相对会话基线的增量，
        // 若底层计数器复位，则从新的较小值继续累计而不产生无符号下溢。
        return current >= previous ? current - previous : current;
    }

    std::uint32_t total_{0U};
    std::uint32_t uart_receive_errors_{0U};
    std::uint32_t uart_dropped_bytes_{0U};
    std::uint32_t last_receive_errors_{0U};
    std::uint32_t last_dropped_bytes_{0U};
};

} // namespace dima::drivers::gps

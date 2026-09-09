#pragma once

#include <cstdint>

namespace dima::drivers::gps {

class GpsErrorCounter final {
public:
    // UART 统计量由底层端口跨驱动生命周期累计。每次重新探测或切换波特率后，
    // 先保存新的会话基线，避免把错误波特率探测期间的预期错误算入运行期健康度。
    void begin_session(std::uint32_t receive_errors,
                       std::uint32_t dropped_bytes) noexcept;

    // total_ 是 DataValidator 使用的单调累计错误输入；饱和而不回绕，
    // 防止长时间运行后 UINT32 溢出突然把错误密度解释为“恢复健康”。
    void record(std::uint32_t increment = 1U) noexcept;

    // 将底层“自启动以来累计值”转换为“本 GPS 会话新增值”，并分别保留
    // UART 接收错误和丢字节，供离线边沿日志区分线路错误与缓冲区拥塞。
    void update_uart(std::uint32_t receive_errors,
                     std::uint32_t dropped_bytes) noexcept;

    std::uint32_t total() const noexcept;
    std::uint32_t uart_receive_errors() const noexcept;
    std::uint32_t uart_dropped_bytes() const noexcept;

private:
    static std::uint32_t saturating_add(
        std::uint32_t value, std::uint32_t increment) noexcept;

    static std::uint32_t counter_increment(
        std::uint32_t current, std::uint32_t previous) noexcept;

    std::uint32_t total_{0U};
    std::uint32_t uart_receive_errors_{0U};
    std::uint32_t uart_dropped_bytes_{0U};
    std::uint32_t last_receive_errors_{0U};
    std::uint32_t last_dropped_bytes_{0U};
};

} // namespace dima::drivers::gps

#pragma once

#include "Dima/middleware/work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum class SbusPort : std::int32_t {
    Disabled = 0,
    Uart4Pb8 = 1,
    Uart7Pe7 = 2,
    Uart8Pe0 = 3,
    Usart2Pd6 = 4,
};

struct SbusUartStats {
    std::uint32_t received_bytes;
    std::uint32_t overwritten_bytes;
    std::uint32_t uart_errors;
    std::uint32_t rearm_failures;
};

class SbusUartBackend {
public:
    bool start(SbusPort port, bool inverted, px4::WorkItem &consumer) noexcept;
    void stop() noexcept;
    std::size_t read(std::uint8_t *destination, std::size_t capacity) noexcept;
    bool service() noexcept;
    bool running() const noexcept { return running_; }
    bool handles_uart(const void *uart) const noexcept { return uart_ == uart; }
    SbusUartStats stats() const noexcept;

    void on_rx_position_from_isr(std::uint16_t position) noexcept;
    void on_error_from_isr(std::uint32_t error) noexcept;

private:
    bool arm_receive() noexcept;
    void notify_consumer_from_isr() noexcept;

    void *uart_{nullptr};
    px4::WorkItem *consumer_{nullptr};
    volatile std::uint32_t produced_{0U};
    std::uint32_t consumed_{0U};
    volatile std::uint16_t last_dma_position_{0U};
    volatile std::uint32_t pending_error_{0U};
    volatile std::uint32_t uart_errors_{0U};
    std::uint32_t overwritten_bytes_{0U};
    std::uint32_t rearm_failures_{0U};
    bool running_{false};
};

SbusUartBackend &sbus_uart_backend() noexcept;

} // namespace dima::platform

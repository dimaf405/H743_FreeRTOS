#pragma once

#include "rc/sbus_backend.hpp"

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

class SbusUartBackend final : public dima::rc::SbusBackend {
public:
    bool configure(std::int32_t port, bool inverted) noexcept override;
    bool start(px4::WorkItem &consumer) noexcept override;
    void stop() noexcept override;
    std::size_t read(std::uint8_t *destination,
                     std::size_t capacity) noexcept override;
    bool service() noexcept override;
    bool running() const noexcept override { return running_; }
    bool handles_uart(const void *uart) const noexcept { return uart_ == uart; }
    dima::rc::SbusBackendStats stats() const noexcept override;

    void on_rx_position_from_isr(std::uint16_t position) noexcept;
    void on_error_from_isr(std::uint32_t error) noexcept;

private:
    bool arm_receive() noexcept;
    void notify_consumer_from_isr() noexcept;

    SbusPort configured_port_{SbusPort::Disabled};
    bool configured_inverted_{true};
    void *uart_{nullptr};
    px4::WorkItem *consumer_{nullptr};
    volatile std::uint32_t produced_{0U};
    std::uint32_t consumed_{0U};
    volatile std::uint16_t last_dma_position_{0U};
    volatile std::uint32_t pending_error_{0U};
    volatile std::uint32_t uart_errors_{0U};
    std::uint32_t overwritten_bytes_{0U};
    std::uint32_t rearm_failures_{0U};
    bool dma_initialized_{false};
    volatile bool running_{false};
};

SbusUartBackend &sbus_uart_backend() noexcept;

} // namespace dima::platform

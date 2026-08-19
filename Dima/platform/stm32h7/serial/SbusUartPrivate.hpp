#pragma once

#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

// 仅供 SBUS 后端实现文件共享；公开 capability 仍由 HardwareServices.hpp 提供。

struct RxPinSnapshot {
    GPIO_TypeDef *port{nullptr};
    std::uint32_t index{0U};
    std::uint32_t mode{0U};
    std::uint32_t output_type{0U};
    std::uint32_t speed{0U};
    std::uint32_t pull{0U};
    std::uint32_t alternate{0U};
    bool valid{false};
};

std::uint32_t request_for(std::int32_t port) noexcept;
IRQn_Type irq_for(const UART_HandleTypeDef *uart) noexcept;
bool capture_rx_pin(std::int32_t port, RxPinSnapshot &snapshot) noexcept;
bool restore_rx_pin(const RxPinSnapshot &snapshot) noexcept;
void configure_sbus_rx_pin(std::int32_t port) noexcept;

} // namespace dima::platform::stm32h7

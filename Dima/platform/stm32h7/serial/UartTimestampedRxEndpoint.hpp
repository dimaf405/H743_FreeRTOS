#pragma once

#include "api/Serial.hpp"
#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

/* 时间戳 RX 端点临时接管一个只接收 UART 和 DMA1 Stream2，并在 stop 时恢复原
 * HAL/引脚配置；它与通用 duplex 端点互斥拥有线路。 */
TimestampedSerialInput &timestamped_serial_input() noexcept;

bool uart_timestamped_rx_endpoint_allows_line_configuration() noexcept;
bool uart_timestamped_rx_endpoint_reset_configuration() noexcept;
bool uart_timestamped_rx_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept;
bool uart_timestamped_rx_endpoint_on_error(
    UART_HandleTypeDef *uart, std::uint32_t error) noexcept;

} // namespace dima::platform::stm32h7

#pragma once

#include "api/Serial.hpp"
#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

/* 两个通用全双工端点分别独占 DMA1 Stream3 与 Stream4/5；全局 HAL 回调必须先经
 * UartIrqRouter 验证句柄所有权再进入以下钩子。 */
AsyncSerialPort &async_serial_port() noexcept;
AsyncSerialPort &telemetry_serial_port() noexcept;
bool uart_duplex_dma_endpoint_port_in_use(std::int32_t port) noexcept;
void uart_duplex_dma_endpoint_on_tx_complete(UART_HandleTypeDef *uart) noexcept;

bool uart_duplex_dma_endpoint_allows_line_configuration() noexcept;
bool uart_duplex_dma_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept;
bool uart_duplex_dma_endpoint_on_error(
    UART_HandleTypeDef *uart, std::uint32_t error) noexcept;

} // namespace dima::platform::stm32h7

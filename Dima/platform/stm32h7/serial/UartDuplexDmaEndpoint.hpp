#pragma once

#include "api/Serial.hpp"
#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

/* 通用全双工端点独占一个已配置 UART 和 DMA1 Stream3；全局 HAL 回调必须先经
 * UartIrqRouter 验证句柄所有权再进入以下钩子。 */
AsyncSerialPort &async_serial_port() noexcept;

bool uart_duplex_dma_endpoint_allows_line_configuration() noexcept;
bool uart_duplex_dma_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept;
bool uart_duplex_dma_endpoint_on_error(
    UART_HandleTypeDef *uart, std::uint32_t error) noexcept;

} // namespace dima::platform::stm32h7

#pragma once

#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

// HAL 全局回调按 UART 句柄路由到唯一端口 endpoint；position 是 ReceiveToIdle
// DMA 缓冲中的累计写位置，error 路径由 endpoint 自行区分瞬态噪声与硬故障。
void route_uart_rx_event(UART_HandleTypeDef *uart,
                         std::uint16_t position) noexcept;
void route_uart_error(UART_HandleTypeDef *uart) noexcept;

} // namespace dima::platform::stm32h7

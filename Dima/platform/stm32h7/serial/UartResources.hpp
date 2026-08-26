#pragma once

#include "api/Serial.hpp"
#include "usart.h"

#include <cstdint>

namespace dima::platform::stm32h7 {

struct UartRxPinSnapshot {
    /* 端点临时接管 UART 前保存 RX 引脚寄存器域；恢复时只修改该 pin 对应 bit，
     * 不覆盖同一 GPIO 端口上其他引脚的并发配置。 */
    GPIO_TypeDef *port{nullptr};
    std::uint32_t index{0U};
    std::uint32_t mode{0U};
    std::uint32_t output_type{0U};
    std::uint32_t speed{0U};
    std::uint32_t pull{0U};
    std::uint32_t alternate{0U};
    bool valid{false};
};

struct UartRxPinResource {
    GPIO_TypeDef *port{nullptr};
    std::uint32_t pin{0U};
    std::uint32_t alternate{0U};
    std::uint32_t index{0U};
};

UART_HandleTypeDef *uart_for(std::int32_t port) noexcept;
std::uint32_t request_for(std::int32_t port) noexcept;
IRQn_Type irq_for(const UART_HandleTypeDef *uart) noexcept;
std::uint32_t translate_uart_error(std::uint32_t error) noexcept;
UartRxPinResource rx_pin_for(std::int32_t port) noexcept;
bool capture_rx_pin(std::int32_t port,
                    UartRxPinSnapshot &snapshot) noexcept;
bool restore_rx_pin(const UartRxPinSnapshot &snapshot) noexcept;
bool uart_line_configuration_valid(
    const SerialLineConfiguration &configuration) noexcept;
bool uart_line_configuration_equal(
    const SerialLineConfiguration &lhs,
    const SerialLineConfiguration &rhs) noexcept;
bool reinitialize_uart(
    UART_HandleTypeDef *uart,
    const SerialLineConfiguration &configuration) noexcept;
bool configure_uart_rx_pull(std::int32_t port,
                            SerialRxPull pull) noexcept;
std::uint64_t serial_byte_time_us(
    const SerialLineConfiguration &configuration) noexcept;

} // namespace dima::platform::stm32h7

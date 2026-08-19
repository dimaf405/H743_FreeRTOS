#include "SbusUartPrivate.hpp"

#include "board_serial_config.hpp"
#include "platform/api/Execution.hpp"

namespace dima::platform::stm32h7 {
namespace {

struct RxPinConfig {
    GPIO_TypeDef *port{nullptr};
    std::uint32_t pin{0U};
    std::uint32_t alternate{0U};
    std::uint32_t index{0U};
};

RxPinConfig rx_pin_for(std::int32_t port) noexcept
{
    switch (port) {
#define DIMA_RX_PIN_CASE(id, handle, request, irq, gpio, pin, af, index) \
    case id: return {gpio, pin, af, index##U};
    DIMA_STM32_SERIAL_PORT_LIST(DIMA_RX_PIN_CASE)
#undef DIMA_RX_PIN_CASE
    default:
        return {};
    }
}

} // namespace

std::uint32_t request_for(std::int32_t port) noexcept
{
    switch (port) {
#define DIMA_REQUEST_CASE(id, handle, request, irq, gpio, pin, af, index) \
    case id: return request;
    DIMA_STM32_SERIAL_PORT_LIST(DIMA_REQUEST_CASE)
#undef DIMA_REQUEST_CASE
    default: return 0U;
    }
}

IRQn_Type irq_for(const UART_HandleTypeDef *uart) noexcept
{
#define DIMA_IRQ_IF(id, handle, request, irq, gpio, pin, af, index) \
    if (uart == &handle) return irq;
    DIMA_STM32_SERIAL_PORT_LIST(DIMA_IRQ_IF)
#undef DIMA_IRQ_IF
    return NonMaskableInt_IRQn;
}

bool capture_rx_pin(std::int32_t port, RxPinSnapshot &snapshot) noexcept
{
    const RxPinConfig pin = rx_pin_for(port);
    if (pin.port == nullptr || pin.pin == 0U || pin.index >= 16U) {
        return false;
    }

    const std::uint32_t pair_shift = pin.index * 2U;
    const std::uint32_t alternate_shift = (pin.index % 8U) * 4U;
    snapshot.port = pin.port;
    snapshot.index = pin.index;
    snapshot.mode = pin.port->MODER & (0x3U << pair_shift);
    snapshot.output_type = pin.port->OTYPER & (0x1U << pin.index);
    snapshot.speed = pin.port->OSPEEDR & (0x3U << pair_shift);
    snapshot.pull = pin.port->PUPDR & (0x3U << pair_shift);
    snapshot.alternate =
        pin.port->AFR[pin.index / 8U] & (0xFU << alternate_shift);
    snapshot.valid = true;
    return true;
}

bool restore_rx_pin(const RxPinSnapshot &snapshot) noexcept
{
    if (!snapshot.valid || snapshot.port == nullptr || snapshot.index >= 16U) {
        return false;
    }

    const std::uint32_t pair_shift = snapshot.index * 2U;
    const std::uint32_t pair_mask = 0x3U << pair_shift;
    const std::uint32_t pin_mask = 0x1U << snapshot.index;
    const std::uint32_t alternate_shift = (snapshot.index % 8U) * 4U;
    const std::uint32_t alternate_mask = 0xFU << alternate_shift;
    dima::platform::CriticalGuard guard;
    MODIFY_REG(snapshot.port->AFR[snapshot.index / 8U], alternate_mask,
               snapshot.alternate);
    MODIFY_REG(snapshot.port->OTYPER, pin_mask, snapshot.output_type);
    MODIFY_REG(snapshot.port->OSPEEDR, pair_mask, snapshot.speed);
    MODIFY_REG(snapshot.port->PUPDR, pair_mask, snapshot.pull);
    MODIFY_REG(snapshot.port->MODER, pair_mask, snapshot.mode);
    return true;
}

void configure_sbus_rx_pin(std::int32_t port) noexcept
{
    const RxPinConfig pin = rx_pin_for(port);
    if (pin.port == nullptr) {
        return;
    }

    GPIO_InitTypeDef gpio{};
    gpio.Pin = pin.pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    // 原始反相 SBUS 在 UART 启用 RXINV 前空闲为低，必须保持下拉偏置。
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = pin.alternate;
    HAL_GPIO_Init(pin.port, &gpio);
}

} // namespace dima::platform::stm32h7

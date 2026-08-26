#include "UartResources.hpp"

#include "BoardUartResources.hpp"
#include "api/Execution.hpp"

namespace dima::platform::stm32h7 {

std::uint32_t translate_uart_error(std::uint32_t error) noexcept
{
    /* HAL 位映射为平台稳定分类；未知位显式落入 Unknown，不能静默丢失。 */
    std::uint32_t translated = SerialInputErrorNone;
    if ((error & HAL_UART_ERROR_PE) != 0U) translated |= SerialInputErrorParity;
    if ((error & HAL_UART_ERROR_NE) != 0U) translated |= SerialInputErrorNoise;
    if ((error & HAL_UART_ERROR_FE) != 0U) translated |= SerialInputErrorFraming;
    if ((error & HAL_UART_ERROR_ORE) != 0U) translated |= SerialInputErrorOverrun;
    if ((error & HAL_UART_ERROR_DMA) != 0U) translated |= SerialInputErrorDma;
    if ((error & HAL_UART_ERROR_RTO) != 0U) translated |= SerialInputErrorTimeout;
    constexpr std::uint32_t known = HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                                    HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE |
                                    HAL_UART_ERROR_DMA | HAL_UART_ERROR_RTO;
    if ((error & ~known) != 0U) translated |= SerialInputErrorUnknown;
    return translated;
}

UartRxPinResource rx_pin_for(std::int32_t port) noexcept
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

UART_HandleTypeDef *uart_for(std::int32_t port) noexcept
{
    switch (port) {
#define DIMA_UART_CASE(id, handle, request, irq, gpio, pin, af, index) \
    case id: return &handle;
    DIMA_STM32_SERIAL_PORT_LIST(DIMA_UART_CASE)
#undef DIMA_UART_CASE
    default: return nullptr;
    }
}

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

bool capture_rx_pin(std::int32_t port,
                    UartRxPinSnapshot &snapshot) noexcept
{
    const UartRxPinResource pin = rx_pin_for(port);
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

bool restore_rx_pin(const UartRxPinSnapshot &snapshot) noexcept
{
    if (!snapshot.valid || snapshot.port == nullptr || snapshot.index >= 16U) {
        return false;
    }

    const std::uint32_t pair_shift = snapshot.index * 2U;
    const std::uint32_t pair_mask = 0x3U << pair_shift;
    const std::uint32_t pin_mask = 0x1U << snapshot.index;
    const std::uint32_t alternate_shift = (snapshot.index % 8U) * 4U;
    const std::uint32_t alternate_mask = 0xFU << alternate_shift;
    /* AFR/OTYPER/OSPEEDR/PUPDR 最后到 MODER 的顺序，保证模式重新开放前辅助属性
     * 已恢复；CriticalGuard 防止 ISR 与任务交错修改同一寄存器字。 */
    dima::platform::CriticalGuard guard;
    MODIFY_REG(snapshot.port->AFR[snapshot.index / 8U], alternate_mask,
               snapshot.alternate);
    MODIFY_REG(snapshot.port->OTYPER, pin_mask, snapshot.output_type);
    MODIFY_REG(snapshot.port->OSPEEDR, pair_mask, snapshot.speed);
    MODIFY_REG(snapshot.port->PUPDR, pair_mask, snapshot.pull);
    MODIFY_REG(snapshot.port->MODER, pair_mask, snapshot.mode);
    return true;
}

bool uart_line_configuration_valid(
    const SerialLineConfiguration &configuration) noexcept
{
    const bool parity_enabled = configuration.parity != SerialParity::None;
    /* STM32 WordLength 包含 parity 位：uart_word_bits = data_bits + parity_bits。
     * 例如通用 8E2 配置对应 9-bit word length、偶校验和 2 stop bits。 */
    const std::uint8_t uart_word_bits = static_cast<std::uint8_t>(
        configuration.data_bits + (parity_enabled ? 1U : 0U));
    return configuration.baudrate != 0U &&
           configuration.data_bits >= 7U &&
           configuration.data_bits <= 8U &&
           uart_word_bits >= 7U && uart_word_bits <= 9U &&
           (configuration.rx_enabled || configuration.tx_enabled) &&
           (!configuration.rx_inverted || configuration.rx_enabled) &&
           (!configuration.tx_inverted || configuration.tx_enabled);
}

bool uart_line_configuration_equal(
    const SerialLineConfiguration &lhs,
    const SerialLineConfiguration &rhs) noexcept
{
    return lhs.baudrate == rhs.baudrate &&
           lhs.data_bits == rhs.data_bits &&
           lhs.parity == rhs.parity &&
           lhs.stop_bits == rhs.stop_bits &&
           lhs.rx_pull == rhs.rx_pull &&
           lhs.rx_enabled == rhs.rx_enabled &&
           lhs.tx_enabled == rhs.tx_enabled &&
           lhs.rx_inverted == rhs.rx_inverted &&
           lhs.tx_inverted == rhs.tx_inverted;
}

bool reinitialize_uart(
    UART_HandleTypeDef *uart,
    const SerialLineConfiguration &configuration) noexcept
{
    /* HAL 不支持安全热改全部线路属性，先 DeInit 再一次性设置 word/parity/stop/
     * inversion；任一步失败由拥有端点负责回滚此前快照。 */
    if (uart == nullptr || !uart_line_configuration_valid(configuration) ||
        HAL_UART_DeInit(uart) != HAL_OK) {
        return false;
    }

    const bool parity_enabled = configuration.parity != SerialParity::None;
    const std::uint8_t uart_word_bits = static_cast<std::uint8_t>(
        configuration.data_bits + (parity_enabled ? 1U : 0U));
    switch (uart_word_bits) {
    case 7U: uart->Init.WordLength = UART_WORDLENGTH_7B; break;
    case 8U: uart->Init.WordLength = UART_WORDLENGTH_8B; break;
    case 9U: uart->Init.WordLength = UART_WORDLENGTH_9B; break;
    default: return false;
    }
    uart->Init.BaudRate = configuration.baudrate;
    uart->Init.StopBits = configuration.stop_bits == SerialStopBits::Two
                              ? UART_STOPBITS_2
                              : UART_STOPBITS_1;
    switch (configuration.parity) {
    case SerialParity::None: uart->Init.Parity = UART_PARITY_NONE; break;
    case SerialParity::Even: uart->Init.Parity = UART_PARITY_EVEN; break;
    case SerialParity::Odd: uart->Init.Parity = UART_PARITY_ODD; break;
    }
    uart->Init.Mode = (configuration.rx_enabled ? UART_MODE_RX : 0U) |
                      (configuration.tx_enabled ? UART_MODE_TX : 0U);
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    uart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (configuration.rx_inverted) {
        uart->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
        uart->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    }
    if (configuration.tx_inverted) {
        uart->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
        uart->AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
    }

    return HAL_UART_Init(uart) == HAL_OK &&
           HAL_UARTEx_SetTxFifoThreshold(
               uart, UART_TXFIFO_THRESHOLD_1_8) == HAL_OK &&
           HAL_UARTEx_SetRxFifoThreshold(
               uart, UART_RXFIFO_THRESHOLD_1_8) == HAL_OK &&
           HAL_UARTEx_DisableFifoMode(uart) == HAL_OK;
}

bool configure_uart_rx_pull(std::int32_t port, SerialRxPull pull) noexcept
{
    if (pull == SerialRxPull::Preserve) {
        return true;
    }
    const UartRxPinResource pin = rx_pin_for(port);
    if (pin.port == nullptr) {
        return false;
    }

    GPIO_InitTypeDef gpio{};
    gpio.Pin = pin.pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    switch (pull) {
    case SerialRxPull::Preserve: return true;
    case SerialRxPull::None: gpio.Pull = GPIO_NOPULL; break;
    case SerialRxPull::Up: gpio.Pull = GPIO_PULLUP; break;
    case SerialRxPull::Down: gpio.Pull = GPIO_PULLDOWN; break;
    }
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = pin.alternate;
    HAL_GPIO_Init(pin.port, &gpio);
    return true;
}

std::uint64_t serial_byte_time_us(
    const SerialLineConfiguration &configuration) noexcept
{
    if (!uart_line_configuration_valid(configuration)) {
        return 0U;
    }
    const std::uint64_t parity_bits =
        configuration.parity == SerialParity::None ? 0U : 1U;
    const std::uint64_t stop_bits =
        configuration.stop_bits == SerialStopBits::Two ? 2U : 1U;
    /* 每字节线路位数 = 1 start + data + optional parity + stop；
     * byte_time_us = ceil(frame_bits * 1e6 / baud)，向上取整避免时间戳倒推过短。 */
    const std::uint64_t frame_bits = 1U + configuration.data_bits +
                                     parity_bits + stop_bits;
    return (frame_bits * 1000000U + configuration.baudrate - 1U) /
           configuration.baudrate;
}

} // namespace dima::platform::stm32h7

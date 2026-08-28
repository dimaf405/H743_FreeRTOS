#include "UartIrqRouter.hpp"

#include "UartDuplexDmaEndpoint.hpp"
#include "UartTimestampedRxEndpoint.hpp"

namespace dima::platform::stm32h7 {

void route_uart_rx_event(UART_HandleTypeDef *uart,
                         std::uint16_t position) noexcept
{
    /* 同一 HAL 全局回调按唯一所有权路由；timestamped 优先认领，认领后不得再把
     * 相同 position 投递给 duplex，避免一个 UART 被双重消费。 */
    if (uart_timestamped_rx_endpoint_on_rx_event(uart, position)) {
        return;
    }
    (void)uart_duplex_dma_endpoint_on_rx_event(uart, position);
}

void route_uart_error(UART_HandleTypeDef *uart) noexcept
{
    /* ErrorCode 在一次回调中快照并交给实际所有者，未被任何端点持有时仅由 HAL
     * 完成底层清理，不凭端口号猜测消费者。 */
    if (uart_timestamped_rx_endpoint_on_error(uart, uart->ErrorCode)) {
        return;
    }
    (void)uart_duplex_dma_endpoint_on_error(uart, uart->ErrorCode);
}

} // namespace dima::platform::stm32h7

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,
                                           std::uint16_t position)
{
    dima::platform::stm32h7::route_uart_rx_event(uart, position);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    dima::platform::stm32h7::route_uart_error(uart);
}

extern "C" void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart4); }
extern "C" void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }
extern "C" void UART8_IRQHandler(void) { HAL_UART_IRQHandler(&huart8); }
extern "C" void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
extern "C" void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
extern "C" void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
extern "C" void USART6_IRQHandler(void) { HAL_UART_IRQHandler(&huart6); }

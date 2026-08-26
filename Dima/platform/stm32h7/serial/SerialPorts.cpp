#include "UartDuplexDmaEndpoint.hpp"
#include "UartTimestampedRxEndpoint.hpp"
#include "UartResources.hpp"

#include "api/Serial.hpp"

namespace dima::platform::stm32h7 {
namespace {

class Stm32SerialPorts final : public SerialPorts {
public:
    bool configure_line(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept override
    {
        /* 只有两个 DMA 端点都未持有任何 UART 时才允许改全局线路，防止绕过端点
         * 所有权热改寄存器。线路参数和 RX pull 必须作为一个业务操作成功。 */
        if (!uart_timestamped_rx_endpoint_allows_line_configuration() ||
            !uart_duplex_dma_endpoint_allows_line_configuration() ||
            !uart_line_configuration_valid(configuration)) {
            return false;
        }
        UART_HandleTypeDef *const uart = uart_for(port);
        if (uart == nullptr) {
            return false;
        }
        return reinitialize_uart(uart, configuration) &&
               configure_uart_rx_pull(port, configuration.rx_pull);
    }

    bool reset_configuration() noexcept override
    {
        /* reset 释放 timestamped 端点保存的“正常 UART”快照，使串口映射可重配。 */
        return uart_timestamped_rx_endpoint_reset_configuration();
    }
};

Stm32SerialPorts &instance() noexcept
{
    static Stm32SerialPorts value;
    return value;
}

} // namespace

SerialPorts &serial_ports() noexcept { return instance(); }

} // namespace dima::platform::stm32h7

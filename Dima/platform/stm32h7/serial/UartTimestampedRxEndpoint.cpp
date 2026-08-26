#include "UartTimestampedRxEndpoint.hpp"

#include "UartResources.hpp"
#include "stm32h7/HardwareServices.hpp"

#include "usart.h"

#include <algorithm>

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kDmaBufferSize = 64U;
constexpr std::size_t kReceiveRingCapacity = 256U;
constexpr std::uint32_t kReceiveFaultUart = 1U << 0U;
constexpr std::uint32_t kReceiveFaultOverflow = 1U << 1U;
constexpr std::uint32_t kReceiveFaultDmaPosition = 1U << 2U;
constexpr std::uint32_t kDmaIrqPriority = 6U;
constexpr std::uint32_t kUartIrqPriority = 7U;

struct ReceivedByte {
    std::uint64_t arrival_us{0U};
    std::uint8_t value{0U};
};

static_assert((kReceiveRingCapacity & (kReceiveRingCapacity - 1U)) == 0U,
              "timestamped RX ring capacity must be a power of two");

alignas(32) std::uint8_t g_dma_buffer[kDmaBufferSize]
    __attribute__((section(".dima_dma")));
alignas(8) ReceivedByte g_receive_ring[kReceiveRingCapacity]{};
/* 上述 CPU ring 只保存 ISR 从 DMA 快照出的字节与估算到达时间；DMA 不直接写
 * 该环，因此消费者在 write_sequence 发布后读取到的槽保持稳定。 */
DMA_HandleTypeDef g_timestamped_rx_dma{};

class UartTimestampedRxEndpoint final : public TimestampedSerialInput {
public:
    bool allows_line_configuration() const noexcept
    {
        return !running() && uart_ == nullptr && !dma_initialized_ &&
               !normal_configuration_valid_;
    }

    bool reset_configuration() noexcept
    {
        if (running() || uart_ != nullptr || dma_initialized_ ||
            takeover_active_) {
            return false;
        }
        configured_port_ = 0;
        normal_uart_ = nullptr;
        normal_init_ = {};
        normal_advanced_init_ = {};
        normal_rx_pin_ = {};
        line_configuration_ = {};
        byte_time_us_ = 0U;
        normal_fifo_mode_ = UART_FIFOMODE_DISABLE;
        normal_tx_fifo_threshold_ = UART_TXFIFO_THRESHOLD_1_8;
        normal_rx_fifo_threshold_ = UART_RXFIFO_THRESHOLD_1_8;
        normal_configuration_valid_ = false;
        return true;
    }

    bool configure(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept override
    {
        const std::uint64_t byte_time_us =
            serial_byte_time_us(configuration);
        if (running() || uart_ != nullptr || dma_initialized_ ||
            !configuration.rx_enabled || configuration.tx_enabled ||
            byte_time_us == 0U) {
            return false;
        }
        /* configure 建立一代新统计基线，并保存正常 UART/FIFO/GPIO 快照；这里只
         * 记录接管合同，实际 HAL 重配到 start 才发生。 */
        reset_statistics();
        const std::int32_t selected = port;
        auto *const uart = uart_for(selected);
        if (uart == nullptr || request_for(selected) == 0U) {
            return false;
        }
        UartRxPinSnapshot rx_pin{};
        if (!capture_rx_pin(selected, rx_pin)) {
            return false;
        }
        configured_port_ = selected;
        normal_uart_ = uart;
        normal_init_ = uart->Init;
        normal_advanced_init_ = uart->AdvancedInit;
        normal_fifo_mode_ = uart->FifoMode;
        normal_tx_fifo_threshold_ = uart->Instance->CR3 & USART_CR3_TXFTCFG;
        normal_rx_fifo_threshold_ = uart->Instance->CR3 & USART_CR3_RXFTCFG;
        normal_rx_pin_ = rx_pin;
        line_configuration_ = configuration;
        byte_time_us_ = byte_time_us;
        normal_configuration_valid_ = true;
        return true;
    }

    bool start(IsrCallback notification) noexcept override
    {
        if (!stop()) {
            return false;
        }
        auto *const uart = uart_for(configured_port_);
        const std::uint32_t request = request_for(configured_port_);
        const DmaBufferView dma_buffer =
            dma_memory().view(g_dma_buffer, sizeof(g_dma_buffer));
        if (uart == nullptr || uart != normal_uart_ ||
            !normal_configuration_valid_ || request == 0U ||
            !dma_memory().valid(dma_buffer) || notification.function == nullptr) {
            increment_recovery_failure();
            return false;
        }

        /* start 先调用 stop 清理上代残留，再验证生成的 DMA request 与 DMA 区；
         * takeover_active 一旦置位，所有失败路径都必须经 stop 尝试恢复正常 UART。 */
        uart_ = uart;
        takeover_active_ = true;
        if (!reinitialize_uart(uart, line_configuration_) ||
            !configure_uart_rx_pull(configured_port_,
                                    line_configuration_.rx_pull)) {
            increment_recovery_failure();
            (void)stop();
            return false;
        }
        __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_FEF | UART_CLEAR_PEF |
                                       UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(uart, UART_RXDATA_FLUSH_REQUEST);

        g_timestamped_rx_dma = DMA_HandleTypeDef{};
        g_timestamped_rx_dma.Instance = DMA1_Stream2;
        g_timestamped_rx_dma.Init.Request = request;
        g_timestamped_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        g_timestamped_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
        g_timestamped_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
        g_timestamped_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        g_timestamped_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        g_timestamped_rx_dma.Init.Mode = DMA_CIRCULAR;
        g_timestamped_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
        g_timestamped_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&g_timestamped_rx_dma) != HAL_OK) {
            increment_recovery_failure();
            (void)stop();
            return false;
        }
        dma_initialized_ = true;
        __HAL_LINKDMA(uart, hdmarx, g_timestamped_rx_dma);

        notification_ = notification;

        /* DMA half/full 回调必须先于同拍 UART IDLE 回调，圆形缓冲跨界时 position
         * 才保持有序；因此 DMA IRQ 数值优先级高于 UART IRQ。 */
        HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, kDmaIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
        HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        const IRQn_Type uart_irq = irq_for(uart);
        HAL_NVIC_SetPriority(uart_irq, kUartIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(uart_irq);
        HAL_NVIC_EnableIRQ(uart_irq);

        __atomic_store_n(&running_, true, __ATOMIC_RELEASE);
        if (!arm_receive()) {
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
            increment_recovery_failure();
            (void)stop();
            return false;
        }
        return true;
    }

    bool stop() noexcept override
    {
        /* 先撤销 running 阻止新回调写 ring，再停两级 IRQ/DMA，最后恢复接管前的
         * UART 和 RX pin；恢复失败时保留 ownership 以阻止其他模块抢用未知状态。 */
        __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        auto *const uart = static_cast<UART_HandleTypeDef *>(uart_);
        if (uart != nullptr) {
            const IRQn_Type uart_irq = irq_for(uart);
            HAL_NVIC_DisableIRQ(uart_irq);
            HAL_NVIC_ClearPendingIRQ(uart_irq);
        }
        HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);

        if (uart_ != nullptr) {
            (void)HAL_UART_AbortReceive(uart);
        }
        if (dma_initialized_) {
            (void)HAL_DMA_DeInit(&g_timestamped_rx_dma);
            dma_initialized_ = false;
        }
        if (uart != nullptr) {
            uart->hdmarx = nullptr;
            (void)HAL_UART_DeInit(uart);
        }
        bool restored = true;
        if (takeover_active_) {
            restored = restore_normal_uart(uart);
            if (!restored) {
                increment_recovery_failure();
            }
        }
        if (restored) {
            uart_ = nullptr;
            takeover_active_ = false;
        }
        notification_ = {};
        reset_receive_epoch();
        return restored;
    }

    std::size_t read(std::uint8_t *destination,
                     std::uint64_t *arrival_timestamps_us,
                     std::size_t capacity) noexcept override
    {
        if (destination == nullptr || arrival_timestamps_us == nullptr ||
            capacity == 0U) {
            return 0U;
        }
        const std::uint32_t consumed =
            __atomic_load_n(&ring_read_sequence_, __ATOMIC_RELAXED);
        const std::uint32_t produced =
            __atomic_load_n(&ring_write_sequence_, __ATOMIC_ACQUIRE);
        /* SPSC ring 由 ISR 单写、任务单读；acquire write_sequence 后槽内容完整。
         * 差值超过固定容量表示内部溢出，故返回 0 并由 pending_fault 报失效。 */
        const std::uint32_t available = produced - consumed;
        if (available > kReceiveRingCapacity) {
            return 0U;
        }
        const std::size_t count =
            std::min<std::size_t>(available, capacity);
        for (std::size_t index = 0U; index < count; ++index) {
            const ReceivedByte &received = g_receive_ring[
                (consumed + index) & (kReceiveRingCapacity - 1U)];
            destination[index] = received.value;
            arrival_timestamps_us[index] = received.arrival_us;
        }
        __atomic_store_n(
            &ring_read_sequence_,
            consumed + static_cast<std::uint32_t>(count),
            __ATOMIC_RELEASE);
        return count;
    }

    bool service() noexcept override
    {
        /* timestamped 端点对溢出/UART/DMA 位置错误采用 fail-closed，不在 service
         * 自动重启；上层必须 stop/start，避免时间连续性被悄悄拼接。 */
        return __atomic_load_n(&pending_fault_, __ATOMIC_ACQUIRE) == 0U &&
               running();
    }

    bool running() const noexcept override
    {
        return __atomic_load_n(&running_, __ATOMIC_ACQUIRE);
    }

    std::uint64_t byte_time_us() const noexcept { return byte_time_us_; }

    TimestampedSerialInputStats stats() const noexcept override
    {
        return {
            __atomic_load_n(&received_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&dropped_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&receive_errors_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&recovery_failures_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&receive_error_flags_, __ATOMIC_ACQUIRE),
        };
    }

    bool handles_uart(const void *uart) const noexcept
    {
        return uart_ == uart;
    }

    void on_rx_position_from_isr(
        std::uint16_t position,
        std::uint64_t last_byte_arrival_us) noexcept
    {
        if (!running()) {
            return;
        }
        if (position == 0U || position > kDmaBufferSize) {
            fail_from_isr(kReceiveFaultDmaPosition, true);
            return;
        }
        const std::uint16_t normalized =
            position == kDmaBufferSize ? 0U : position;
        const std::uint16_t previous = last_dma_position_;
        const std::uint16_t delta =
            position == kDmaBufferSize && previous == 0U
                ? static_cast<std::uint16_t>(kDmaBufferSize)
                : (normalized >= previous
                       ? normalized - previous
                       : static_cast<std::uint16_t>(
                             kDmaBufferSize - previous + normalized));
        if (delta == 0U) {
            return;
        }

        /* 已知本批最后字节时间，按固定线路字节周期反推：
         * first = last - (delta-1)*byte_time；每字节 arrival=first+i*byte_time。
         * 下溢饱和到 0，并用 previous_arrival 单调钳位抵抗取整误差。 */
        const std::uint64_t span_us =
            static_cast<std::uint64_t>(delta - 1U) * byte_time_us_;
        const std::uint64_t first_arrival =
            last_byte_arrival_us >= span_us
                ? last_byte_arrival_us - span_us
                : 0U;
        std::uint32_t write_sequence =
            __atomic_load_n(&ring_write_sequence_, __ATOMIC_RELAXED);
        const std::uint32_t read_sequence =
            __atomic_load_n(&ring_read_sequence_, __ATOMIC_ACQUIRE);
        std::uint32_t dropped = 0U;
        std::uint64_t previous_arrival = last_byte_arrival_us_;
        for (std::uint16_t offset = 0U; offset < delta; ++offset) {
            std::uint64_t arrival =
                first_arrival + static_cast<std::uint64_t>(offset) *
                                    byte_time_us_;
            if (arrival < previous_arrival) {
                arrival = previous_arrival;
            }
            const std::size_t index =
                (static_cast<std::size_t>(previous) + offset) %
                kDmaBufferSize;
            /* ring 满时丢弃最新字节并锁定故障/停止端点，不能覆盖仍待解析的数据
             * 后继续声称时间序列完整。 */
            if (write_sequence - read_sequence < kReceiveRingCapacity) {
                ReceivedByte &received = g_receive_ring[
                    write_sequence & (kReceiveRingCapacity - 1U)];
                received.arrival_us = arrival;
                received.value = g_dma_buffer[index];
                ++write_sequence;
            } else {
                ++dropped;
            }
            previous_arrival = arrival;
        }
        __atomic_store_n(&ring_write_sequence_, write_sequence,
                         __ATOMIC_RELEASE);
        (void)__atomic_add_fetch(&received_bytes_, delta, __ATOMIC_RELAXED);
        last_dma_position_ = normalized;
        last_byte_arrival_us_ = previous_arrival;
        if (dropped != 0U) {
            (void)__atomic_add_fetch(&dropped_bytes_, dropped,
                                     __ATOMIC_RELAXED);
            (void)__atomic_fetch_or(&pending_fault_, kReceiveFaultOverflow,
                                    __ATOMIC_RELEASE);
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        }
        notification_.invoke();
    }

    void on_error_from_isr(std::uint32_t error) noexcept
    {
        (void)__atomic_fetch_or(&receive_error_flags_,
                                translate_uart_error(error),
                                __ATOMIC_RELAXED);
        fail_from_isr(kReceiveFaultUart, true);
    }

private:
    bool restore_normal_uart(UART_HandleTypeDef *uart) noexcept
    {
        if (!normal_configuration_valid_ || uart == nullptr ||
            uart != normal_uart_) {
            return false;
        }

        uart->Init = normal_init_;
        uart->AdvancedInit = normal_advanced_init_;
        uart->hdmarx = nullptr;
        if (HAL_UART_Init(uart) != HAL_OK ||
            HAL_UARTEx_SetTxFifoThreshold(
                uart, normal_tx_fifo_threshold_) != HAL_OK ||
            HAL_UARTEx_SetRxFifoThreshold(
                uart, normal_rx_fifo_threshold_) != HAL_OK ||
            (normal_fifo_mode_ == UART_FIFOMODE_ENABLE
                 ? HAL_UARTEx_EnableFifoMode(uart)
                 : HAL_UARTEx_DisableFifoMode(uart)) != HAL_OK ||
            !restore_rx_pin(normal_rx_pin_)) {
            (void)HAL_UART_DeInit(uart);
            return false;
        }
        return true;
    }

    void fail_from_isr(std::uint32_t fault, bool count_error) noexcept
    {
        if (count_error) {
            (void)__atomic_add_fetch(&receive_errors_, 1U,
                                     __ATOMIC_RELAXED);
        }
        /* ISR 仅合并原因、撤销 running 并通知；HAL 资源清理由任务调用 stop 完成。 */
        (void)__atomic_fetch_or(&pending_fault_, fault, __ATOMIC_RELEASE);
        __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        notification_.invoke();
    }

    void increment_recovery_failure() noexcept
    {
        (void)__atomic_add_fetch(&recovery_failures_, 1U,
                                 __ATOMIC_RELAXED);
    }

    void reset_receive_epoch() noexcept
    {
        __atomic_store_n(&ring_write_sequence_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&ring_read_sequence_, 0U, __ATOMIC_RELAXED);
        for (auto &received : g_receive_ring) {
            received = ReceivedByte{};
        }
        last_dma_position_ = 0U;
        last_byte_arrival_us_ = 0U;
        __atomic_store_n(&pending_fault_, 0U, __ATOMIC_RELEASE);
    }

    void reset_statistics() noexcept
    {
        __atomic_store_n(&received_bytes_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&dropped_bytes_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&receive_errors_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&recovery_failures_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&receive_error_flags_, 0U, __ATOMIC_RELAXED);
    }

    bool arm_receive() noexcept
    {
        if (uart_ == nullptr) {
            return false;
        }
        auto *const uart = static_cast<UART_HandleTypeDef *>(uart_);
        return HAL_UARTEx_ReceiveToIdle_DMA(
                   uart, g_dma_buffer,
                   static_cast<std::uint16_t>(kDmaBufferSize)) == HAL_OK;
    }

    std::int32_t configured_port_{0};
    void *uart_{nullptr};
    UART_HandleTypeDef *normal_uart_{nullptr};
    UART_InitTypeDef normal_init_{};
    UART_AdvFeatureInitTypeDef normal_advanced_init_{};
    UartRxPinSnapshot normal_rx_pin_{};
    SerialLineConfiguration line_configuration_{};
    IsrCallback notification_{};
    std::uint32_t ring_write_sequence_{0U};
    std::uint32_t ring_read_sequence_{0U};
    std::uint16_t last_dma_position_{0U};
    std::uint64_t last_byte_arrival_us_{0U};
    std::uint32_t pending_fault_{0U};
    std::uint32_t received_bytes_{0U};
    std::uint32_t dropped_bytes_{0U};
    std::uint32_t receive_errors_{0U};
    std::uint32_t recovery_failures_{0U};
    std::uint32_t receive_error_flags_{0U};
    std::uint32_t normal_fifo_mode_{UART_FIFOMODE_DISABLE};
    std::uint32_t normal_tx_fifo_threshold_{UART_TXFIFO_THRESHOLD_1_8};
    std::uint32_t normal_rx_fifo_threshold_{UART_RXFIFO_THRESHOLD_1_8};
    std::uint64_t byte_time_us_{0U};
    bool normal_configuration_valid_{false};
    bool takeover_active_{false};
    bool dma_initialized_{false};
    bool running_{false};
};

UartTimestampedRxEndpoint &instance() noexcept
{
    static UartTimestampedRxEndpoint value;
    return value;
}

} // namespace

TimestampedSerialInput &timestamped_serial_input() noexcept
{
    return instance();
}

bool uart_timestamped_rx_endpoint_allows_line_configuration() noexcept
{
    return instance().allows_line_configuration();
}

bool uart_timestamped_rx_endpoint_reset_configuration() noexcept
{
    return instance().reset_configuration();
}

bool uart_timestamped_rx_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept
{
    auto &backend = instance();
    if (!backend.running() || !backend.handles_uart(uart)) {
        return false;
    }

    std::uint64_t arrival = clock().now_us();
    const std::uint32_t active_exception = __get_IPSR();
    const std::uint32_t uart_exception =
        static_cast<std::uint32_t>(irq_for(uart)) + 16U;
    const std::uint64_t byte_time_us = backend.byte_time_us();
    if (active_exception == uart_exception && arrival >= byte_time_us) {
        /* UART IDLE 中断通常在最后一个字节后的一个帧时间到达，减去 byte_time
         * 近似最后字节结束时刻；DMA half/full 回调不做该补偿。 */
        arrival -= byte_time_us;
    }
    backend.on_rx_position_from_isr(position, arrival);
    return true;
}

bool uart_timestamped_rx_endpoint_on_error(
    UART_HandleTypeDef *uart, std::uint32_t error) noexcept
{
    auto &backend = instance();
    if (!backend.running() || !backend.handles_uart(uart)) {
        return false;
    }
    backend.on_error_from_isr(error);
    return true;
}

} // namespace dima::platform::stm32h7

extern "C" void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_timestamped_rx_dma);
}

#include "Backend.hpp"

#include "usart.h"

#include <algorithm>

namespace dima::platform::stm32h7 {
namespace {

enum class SbusPort : std::int32_t {
    Disabled = 0,
    Uart4Pb8 = 1,
    Uart7Pe7 = 2,
    Uart8Pe0 = 3,
    Usart2Pd6 = 4,
};

constexpr std::size_t kDmaBufferSize = 64U;
constexpr std::size_t kReceiveRingCapacity = 256U;
constexpr std::uint64_t kSbusByteTimeUs = 120U;
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
              "SBUS receive ring capacity must be a power of two");

alignas(32) std::uint8_t g_dma_buffer[kDmaBufferSize]
    __attribute__((section(".dima_dma")));
/* CPU-only handoff storage. DMA must never own or write this ring. */
alignas(8) ReceivedByte g_receive_ring[kReceiveRingCapacity]{};
DMA_HandleTypeDef g_sbus_dma{};

UART_HandleTypeDef *uart_for(SbusPort port) noexcept
{
    switch (port) {
    case SbusPort::Uart4Pb8: return &huart4;
    case SbusPort::Uart7Pe7: return &huart7;
    case SbusPort::Uart8Pe0: return &huart8;
    case SbusPort::Usart2Pd6: return &huart2;
    default: return nullptr;
    }
}

std::uint32_t request_for(SbusPort port) noexcept
{
    switch (port) {
    case SbusPort::Uart4Pb8: return DMA_REQUEST_UART4_RX;
    case SbusPort::Uart7Pe7: return DMA_REQUEST_UART7_RX;
    case SbusPort::Uart8Pe0: return DMA_REQUEST_UART8_RX;
    case SbusPort::Usart2Pd6: return DMA_REQUEST_USART2_RX;
    default: return 0U;
    }
}

IRQn_Type irq_for(const UART_HandleTypeDef *uart) noexcept
{
    if (uart == &huart4) return UART4_IRQn;
    if (uart == &huart7) return UART7_IRQn;
    if (uart == &huart8) return UART8_IRQn;
    return USART2_IRQn;
}

class Stm32SbusInput final : public SbusInput {
public:
    bool configure(std::int32_t port, bool inverted) noexcept override
    {
        if (running() || uart_ != nullptr || dma_initialized_) {
            return false;
        }
        reset_statistics();
        const auto selected = static_cast<SbusPort>(port);
        if (uart_for(selected) == nullptr || request_for(selected) == 0U) {
            return false;
        }
        configured_port_ = selected;
        configured_inverted_ = inverted;
        return true;
    }

    bool start(IsrCallback notification) noexcept override
    {
        stop();
        auto *const uart = uart_for(configured_port_);
        const std::uint32_t request = request_for(configured_port_);
        const DmaBufferView dma_buffer =
            dma_memory().view(g_dma_buffer, sizeof(g_dma_buffer));
        if (uart == nullptr || request == 0U ||
            !dma_memory().valid(dma_buffer) || notification.function == nullptr) {
            increment_recovery_failure();
            return false;
        }

        (void)HAL_UART_DeInit(uart);
        uart->Init.BaudRate = 100000U;
        uart->Init.WordLength = UART_WORDLENGTH_9B;
        uart->Init.StopBits = UART_STOPBITS_2;
        uart->Init.Parity = UART_PARITY_EVEN;
        uart->Init.Mode = UART_MODE_RX;
        uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
        uart->Init.OverSampling = UART_OVERSAMPLING_16;
        uart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        uart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
        uart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
        uart->AdvancedInit.RxPinLevelInvert = configured_inverted_
            ? UART_ADVFEATURE_RXINV_ENABLE : UART_ADVFEATURE_RXINV_DISABLE;
        if (HAL_UART_Init(uart) != HAL_OK) {
            increment_recovery_failure();
            (void)HAL_UART_DeInit(uart);
            return false;
        }

        uart_ = uart;
        if (HAL_UARTEx_DisableFifoMode(uart) != HAL_OK) {
            increment_recovery_failure();
            stop();
            return false;
        }

        g_sbus_dma = DMA_HandleTypeDef{};
        g_sbus_dma.Instance = DMA1_Stream2;
        g_sbus_dma.Init.Request = request;
        g_sbus_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        g_sbus_dma.Init.PeriphInc = DMA_PINC_DISABLE;
        g_sbus_dma.Init.MemInc = DMA_MINC_ENABLE;
        g_sbus_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        g_sbus_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        g_sbus_dma.Init.Mode = DMA_CIRCULAR;
        g_sbus_dma.Init.Priority = DMA_PRIORITY_HIGH;
        g_sbus_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&g_sbus_dma) != HAL_OK) {
            increment_recovery_failure();
            stop();
            return false;
        }
        dma_initialized_ = true;
        __HAL_LINKDMA(uart, hdmarx, g_sbus_dma);

        notification_ = notification;

        /* DMA half/full callbacks must precede a simultaneous UART IDLE event
         * so callback positions remain ordered across a circular wrap. */
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
            stop();
            return false;
        }
        return true;
    }

    void stop() noexcept override
    {
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
            (void)HAL_DMA_DeInit(&g_sbus_dma);
            dma_initialized_ = false;
        }
        if (uart != nullptr) {
            uart->hdmarx = nullptr;
            (void)HAL_UART_DeInit(uart);
        }
        uart_ = nullptr;
        notification_ = {};
        reset_receive_epoch();
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
        return __atomic_load_n(&pending_fault_, __ATOMIC_ACQUIRE) == 0U &&
               running();
    }

    bool running() const noexcept override
    {
        return __atomic_load_n(&running_, __ATOMIC_ACQUIRE);
    }

    SbusInputStats stats() const noexcept override
    {
        return {
            __atomic_load_n(&received_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&overwritten_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&receive_errors_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&recovery_failures_, __ATOMIC_ACQUIRE),
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

        const std::uint64_t span_us =
            static_cast<std::uint64_t>(delta - 1U) * kSbusByteTimeUs;
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
                                    kSbusByteTimeUs;
            if (arrival < previous_arrival) {
                arrival = previous_arrival;
            }
            const std::size_t index =
                (static_cast<std::size_t>(previous) + offset) %
                kDmaBufferSize;
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
            (void)__atomic_add_fetch(&overwritten_bytes_, dropped,
                                     __ATOMIC_RELAXED);
            (void)__atomic_fetch_or(&pending_fault_, kReceiveFaultOverflow,
                                    __ATOMIC_RELEASE);
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        }
        notification_.invoke();
    }

    void on_error_from_isr(std::uint32_t error) noexcept
    {
        (void)error;
        fail_from_isr(kReceiveFaultUart, true);
    }

private:
    void fail_from_isr(std::uint32_t fault, bool count_error) noexcept
    {
        if (count_error) {
            (void)__atomic_add_fetch(&receive_errors_, 1U,
                                     __ATOMIC_RELAXED);
        }
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
        __atomic_store_n(&overwritten_bytes_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&receive_errors_, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&recovery_failures_, 0U, __ATOMIC_RELAXED);
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

    SbusPort configured_port_{SbusPort::Disabled};
    bool configured_inverted_{true};
    void *uart_{nullptr};
    IsrCallback notification_{};
    std::uint32_t ring_write_sequence_{0U};
    std::uint32_t ring_read_sequence_{0U};
    std::uint16_t last_dma_position_{0U};
    std::uint64_t last_byte_arrival_us_{0U};
    std::uint32_t pending_fault_{0U};
    std::uint32_t received_bytes_{0U};
    std::uint32_t overwritten_bytes_{0U};
    std::uint32_t receive_errors_{0U};
    std::uint32_t recovery_failures_{0U};
    bool dma_initialized_{false};
    bool running_{false};
};

Stm32SbusInput &instance() noexcept
{
    static Stm32SbusInput value;
    return value;
}

} // namespace

SbusInput &sbus_input() noexcept { return instance(); }

} // namespace dima::platform::stm32h7

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart,
                                             std::uint16_t position)
{
    auto &backend = dima::platform::stm32h7::instance();
    if (backend.running() && backend.handles_uart(uart)) {
        std::uint64_t arrival =
            dima::platform::stm32h7::clock().now_us();
        const std::uint32_t active_exception = __get_IPSR();
        const std::uint32_t uart_exception =
            static_cast<std::uint32_t>(
                dima::platform::stm32h7::irq_for(uart)) +
            16U;
        if (active_exception == uart_exception &&
            arrival >= dima::platform::stm32h7::kSbusByteTimeUs) {
            arrival -= dima::platform::stm32h7::kSbusByteTimeUs;
        }
        backend.on_rx_position_from_isr(position, arrival);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    auto &backend = dima::platform::stm32h7::instance();
    if (backend.running() && backend.handles_uart(uart)) {
        backend.on_error_from_isr(uart->ErrorCode);
    }
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_sbus_dma);
}

extern "C" void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart4); }
extern "C" void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }
extern "C" void UART8_IRQHandler(void) { HAL_UART_IRQHandler(&huart8); }
extern "C" void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

#include "sbus_uart_backend.hpp"

#include "dma.h"
#include "usart.h"

#include <algorithm>

namespace dima::platform {
namespace {

constexpr std::size_t kDmaBufferSize = 64U;
alignas(32) __attribute__((section(".dima_dma")))
std::uint8_t g_dma_buffer[kDmaBufferSize]{};
DMA_HandleTypeDef g_sbus_dma{};
SbusUartBackend g_backend{};

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

} // namespace

bool SbusUartBackend::configure(std::int32_t port, bool inverted) noexcept
{
    const auto selected = static_cast<SbusPort>(port);
    if (uart_for(selected) == nullptr || request_for(selected) == 0U) {
        return false;
    }
    configured_port_ = selected;
    configured_inverted_ = inverted;
    return true;
}

bool SbusUartBackend::start(px4::WorkItem &consumer) noexcept
{
    stop();
    auto *const uart = uart_for(configured_port_);
    const std::uint32_t request = request_for(configured_port_);
    if (uart == nullptr || request == 0U) {
        return false;
    }

    (void)HAL_UART_DeInit(uart);
    uart->Init.BaudRate = 100000U;
    // STM32 的 9-bit + even parity 实际承载 8 个 SBUS 数据位。
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
    if (HAL_UART_Init(uart) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(uart) != HAL_OK) {
        return false;
    }

    uart_ = uart;
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
        stop();
        return false;
    }
    dma_initialized_ = true;
    __HAL_LINKDMA(uart, hdmarx, g_sbus_dma);

    consumer_ = &consumer;
    produced_ = 0U;
    consumed_ = 0U;
    last_dma_position_ = 0U;
    pending_error_ = 0U;
    SCB_CleanInvalidateDCache_by_Addr(
        reinterpret_cast<std::uint32_t *>(g_dma_buffer), kDmaBufferSize);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6U, 0U);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    const IRQn_Type uart_irq = irq_for(uart);
    HAL_NVIC_SetPriority(uart_irq, 6U, 0U);
    HAL_NVIC_ClearPendingIRQ(uart_irq);
    HAL_NVIC_EnableIRQ(uart_irq);

    running_ = arm_receive();
    if (!running_) {
        stop();
        return false;
    }
    return true;
}

void SbusUartBackend::stop() noexcept
{
    running_ = false;
    consumer_ = nullptr;

    if (uart_ != nullptr) {
        auto *const uart = static_cast<UART_HandleTypeDef *>(uart_);
        const IRQn_Type uart_irq = irq_for(uart);
        HAL_NVIC_DisableIRQ(uart_irq);
        HAL_NVIC_ClearPendingIRQ(uart_irq);
        (void)HAL_UART_AbortReceive(uart);
        uart->hdmarx = nullptr;
    }

    HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
    if (dma_initialized_) {
        (void)HAL_DMA_DeInit(&g_sbus_dma);
        dma_initialized_ = false;
    }

    uart_ = nullptr;
    pending_error_ = 0U;
    last_dma_position_ = 0U;
}

bool SbusUartBackend::arm_receive() noexcept
{
    if (uart_ == nullptr) {
        return false;
    }
    auto *const uart = static_cast<UART_HandleTypeDef *>(uart_);
    return HAL_UARTEx_ReceiveToIdle_DMA(
        uart, g_dma_buffer, static_cast<std::uint16_t>(kDmaBufferSize)) == HAL_OK;
}

std::size_t SbusUartBackend::read(std::uint8_t *destination,
                                  std::size_t capacity) noexcept
{
    if (destination == nullptr || capacity == 0U) {
        return 0U;
    }

    const std::uint32_t produced = produced_;
    std::uint32_t available = produced - consumed_;
    if (available > kDmaBufferSize) {
        overwritten_bytes_ += available - kDmaBufferSize;
        consumed_ = produced - kDmaBufferSize;
        available = kDmaBufferSize;
    }

    const std::size_t count = std::min<std::size_t>(available, capacity);
    SCB_InvalidateDCache_by_Addr(
        reinterpret_cast<std::uint32_t *>(g_dma_buffer), kDmaBufferSize);
    for (std::size_t i = 0U; i < count; ++i) {
        destination[i] = g_dma_buffer[(consumed_ + i) % kDmaBufferSize];
    }
    consumed_ += static_cast<std::uint32_t>(count);
    return count;
}

bool SbusUartBackend::service() noexcept
{
    if (pending_error_ == 0U) {
        return running_;
    }

    auto *const uart = static_cast<UART_HandleTypeDef *>(uart_);
    if (uart == nullptr) {
        return false;
    }

    pending_error_ = 0U;
    running_ = false;
    (void)HAL_UART_AbortReceive(uart);
    // 重挂 DMA 后写指针从 0 开始；旧半帧不能继续交给解析器。
    consumed_ = produced_;
    last_dma_position_ = 0U;
    __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                               UART_CLEAR_FEF | UART_CLEAR_PEF |
                               UART_CLEAR_IDLEF);
    running_ = arm_receive();
    if (!running_) {
        ++rearm_failures_;
    }
    return running_;
}

void SbusUartBackend::notify_consumer_from_isr() noexcept
{
    if (consumer_ != nullptr) {
        (void)consumer_->ScheduleNowFromISR();
    }
}

void SbusUartBackend::on_rx_position_from_isr(std::uint16_t position) noexcept
{
    const std::uint16_t normalized = position >= kDmaBufferSize ? 0U : position;
    const std::uint16_t previous = last_dma_position_;
    const std::uint16_t delta = normalized >= previous
        ? normalized - previous
        : kDmaBufferSize - previous + normalized;
    __DMB();
    produced_ += delta;
    last_dma_position_ = normalized;
    notify_consumer_from_isr();
}

void SbusUartBackend::on_error_from_isr(std::uint32_t error) noexcept
{
    pending_error_ |= error;
    ++uart_errors_;
    running_ = false;
    notify_consumer_from_isr();
}

dima::rc::SbusBackendStats SbusUartBackend::stats() const noexcept
{
    return {produced_, overwritten_bytes_, uart_errors_, rearm_failures_};
}

SbusUartBackend &sbus_uart_backend() noexcept
{
    return g_backend;
}

} // namespace dima::platform

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                             uint16_t size)
{
    auto &backend = dima::platform::sbus_uart_backend();
    if (backend.running() && backend.handles_uart(huart)) {
        backend.on_rx_position_from_isr(size);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    auto &backend = dima::platform::sbus_uart_backend();
    if (backend.running() && backend.handles_uart(huart)) {
        backend.on_error_from_isr(huart->ErrorCode);
    }
}

extern "C" void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::g_sbus_dma);
}
extern "C" void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&huart4); }
extern "C" void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }
extern "C" void UART8_IRQHandler(void) { HAL_UART_IRQHandler(&huart8); }
extern "C" void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

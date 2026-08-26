#include "stm32h7/HardwareServices.hpp"

#include "board_bus_resources.h"
#include "SpiClockDivider.hpp"

#include "main.h"
#include "spi.h"

#include <cstddef>
#include <cstdint>

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kMaximumDmaBytes = 512U;
GPIO_TypeDef *const kChipSelectPort = DIMA_SPI4_DEVICE_CS_GPIO_Port;
constexpr std::uint16_t kChipSelectPin = DIMA_SPI4_DEVICE_CS_Pin;

void set_chip_select(bool active) noexcept
{
    HAL_GPIO_WritePin(kChipSelectPort, kChipSelectPin,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

std::uint32_t enter_critical() noexcept
{
    const std::uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void leave_critical(std::uint32_t primask) noexcept
{
    if (primask == 0U) {
        __enable_irq();
    }
}

std::uint32_t timeout_ms(Timeout timeout) noexcept
{
    if (timeout.infinite) {
        return HAL_MAX_DELAY;
    }
    if (timeout.microseconds == 0U) {
        return 0U;
    }
    /* HAL 只接收毫秒，使用 ceil(us/1000) 保证非零超时不缩短；加 999 前先饱和
     * 防 UINT64 上溢，最终再饱和到 HAL 的 uint32_t 范围。 */
    const std::uint64_t rounded =
        timeout.microseconds > UINT64_MAX - 999U
            ? UINT64_MAX
            : (timeout.microseconds + 999U) / 1000U;
    return rounded > UINT32_MAX ? UINT32_MAX
                                : static_cast<std::uint32_t>(rounded);
}

std::uint32_t spi4_kernel_frequency_hz() noexcept
{
    /* 本板 SPI4/5 固定取 D2PCLK1。STM32H7 的 HAL_RCCEx_GetPeriphCLKFreq 未实现
     * SPI45 查询并会返回 0，因此先核对选择寄存器，再读取对应 PCLK1。 */
    return __HAL_RCC_GET_SPI45_SOURCE() == RCC_SPI45CLKSOURCE_D2PCLK1
               ? HAL_RCC_GetPCLK1Freq()
               : 0U;
}

bool hal_prescaler(std::uint16_t divisor, std::uint32_t &prescaler) noexcept
{
    switch (divisor) {
    case 2U: prescaler = SPI_BAUDRATEPRESCALER_2; return true;
    case 4U: prescaler = SPI_BAUDRATEPRESCALER_4; return true;
    case 8U: prescaler = SPI_BAUDRATEPRESCALER_8; return true;
    case 16U: prescaler = SPI_BAUDRATEPRESCALER_16; return true;
    case 32U: prescaler = SPI_BAUDRATEPRESCALER_32; return true;
    case 64U: prescaler = SPI_BAUDRATEPRESCALER_64; return true;
    case 128U: prescaler = SPI_BAUDRATEPRESCALER_128; return true;
    case 256U: prescaler = SPI_BAUDRATEPRESCALER_256; return true;
    default: return false;
    }
}

bool hal_mode(SpiMode mode, std::uint32_t &polarity,
              std::uint32_t &phase) noexcept
{
    switch (mode) {
    case SpiMode::Mode0:
        polarity = SPI_POLARITY_LOW;
        phase = SPI_PHASE_1EDGE;
        return true;
    case SpiMode::Mode1:
        polarity = SPI_POLARITY_LOW;
        phase = SPI_PHASE_2EDGE;
        return true;
    case SpiMode::Mode2:
        polarity = SPI_POLARITY_HIGH;
        phase = SPI_PHASE_1EDGE;
        return true;
    case SpiMode::Mode3:
        polarity = SPI_POLARITY_HIGH;
        phase = SPI_PHASE_2EDGE;
        return true;
    }
    return false;
}

class Stm32Spi4Device final : public SpiDevice {
public:
    bool configure(const SpiConfiguration &configuration) noexcept override
    {
        if (__get_IPSR() != 0U || configuration.frequency_hz == 0U ||
            hspi4.Instance != SPI4 || initialized_ || busy()) {
            record_error(SpiErrorInvalidArgument);
            return false;
        }

        /* 先计算 mode 与离散分频并记录 requested/configured，全部合法后才重配 HAL；
         * 端点已初始化或有在途事务时拒绝热切换。 */
        const std::uint32_t source_hz = spi4_kernel_frequency_hz();
        const spi_clock::Selection clock = spi_clock::select(
            source_hz, configuration.frequency_hz);
        std::uint32_t prescaler = 0U;
        std::uint32_t polarity = 0U;
        std::uint32_t phase = 0U;
        record_clock_configuration(source_hz, configuration.frequency_hz,
                                   clock.frequency_hz);
        if (!hal_mode(configuration.mode, polarity, phase)) {
            record_error(SpiErrorInvalidArgument);
            return false;
        }
        if (!clock || !hal_prescaler(clock.divisor, prescaler)) {
            record_error(SpiErrorNotReady);
            return false;
        }

        hspi4.Init.CLKPolarity = polarity;
        hspi4.Init.CLKPhase = phase;
        hspi4.Init.BaudRatePrescaler = prescaler;
        hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi4.Init.NSS = SPI_NSS_SOFT;
        hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        if (HAL_SPI_Init(&hspi4) != HAL_OK) {
            record_error(SpiErrorPeripheral);
            return false;
        }

        set_chip_select(false);
        initialized_ = true;
        return true;
    }

    void shutdown() noexcept override
    {
        if (__get_IPSR() != 0U) {
            return;
        }
        if (!abort_transfer()) {
            return;
        }
        set_chip_select(false);
        initialized_ = false;
    }

    bool transfer(const std::uint8_t *transmit, std::uint8_t *receive,
                  std::size_t length, Timeout timeout) noexcept override
    {
        if (__get_IPSR() != 0U || !initialized_ || transmit == nullptr ||
            receive == nullptr || length == 0U || length > UINT16_MAX) {
            record_error(SpiErrorInvalidArgument);
            return false;
        }
        {
            const std::uint32_t primask = enter_critical();
            if (state_ != TransferState::Idle) {
                leave_critical(primask);
                record_error(SpiErrorBusy);
                return false;
            }
            state_ = TransferState::Blocking;
            leave_critical(primask);
        }

        set_chip_select(true);
        /* CS 覆盖完整 HAL 轮询事务；无论成功、超时或外设错误都在返回前释放。 */
        const HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
            &hspi4, const_cast<std::uint8_t *>(transmit), receive,
            static_cast<std::uint16_t>(length), timeout_ms(timeout));
        set_chip_select(false);

        const std::uint32_t primask = enter_critical();
        state_ = TransferState::Idle;
        if (result == HAL_OK) {
            ++stats_.blocking_transfers;
        }
        leave_critical(primask);

        if (result != HAL_OK) {
            record_error(result == HAL_TIMEOUT ? SpiErrorTimeout
                                               : SpiErrorPeripheral);
            return false;
        }
        return true;
    }

    bool start_transfer(const std::uint8_t *transmit,
                        std::uint8_t *receive, std::size_t length,
                        IsrCallback completion) noexcept override
    {
        if (__get_IPSR() != 0U || !initialized_ || transmit == nullptr ||
            receive == nullptr || length == 0U ||
            length > kMaximumDmaBytes || completion.function == nullptr) {
            record_error(SpiErrorInvalidArgument);
            return false;
        }

        {
            const std::uint32_t primask = enter_critical();
            if (state_ != TransferState::Idle) {
                leave_critical(primask);
                record_error(SpiErrorBusy);
                return false;
            }
            state_ = TransferState::Preparing;
            leave_critical(primask);
        }

        /* DMA 只能访问受合同保护的缓冲区：TX/RX 各占一个固定 bounce。任一申请
         * 失败都回滚 Preparing，不把调用方普通 cacheable 内存直接交给 DMA。 */
        DmaBufferView transmit_dma = dma_memory().acquire_bounce(
            transmit, length, DmaDirection::MemoryToPeripheral);
        if (!transmit_dma) {
            release_preparation();
            record_error(SpiErrorDma);
            return false;
        }
        DmaBufferView receive_dma = dma_memory().acquire_bounce(
            nullptr, length, DmaDirection::PeripheralToMemory);
        if (!receive_dma) {
            dma_memory().release_bounce(
                transmit_dma, nullptr, DmaDirection::MemoryToPeripheral);
            release_preparation();
            record_error(SpiErrorDma);
            return false;
        }

        /* 在临界区发布两端缓冲区、回调和 Active 状态后再拉低 CS 启动 HAL，
         * 使极快完成中断也能看到完整事务元数据。 */
        const std::uint32_t primask = enter_critical();
        transmit_dma_ = transmit_dma;
        receive_dma_ = receive_dma;
        receive_destination_ = receive;
        notification_ = completion;
        state_ = TransferState::Active;
        set_chip_select(true);
        const HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(
            &hspi4, transmit_dma.data, receive_dma.data,
            static_cast<std::uint16_t>(length));
        if (result != HAL_OK) {
            set_chip_select(false);
            state_ = TransferState::Releasing;
        }
        leave_critical(primask);

        if (result != HAL_OK) {
            release_dma(false);
            record_error(result == HAL_BUSY ? SpiErrorBusy : SpiErrorDma);
            return false;
        }

        ++stats_.dma_starts;
        return true;
    }

    SpiTransferResult finish_transfer() noexcept override
    {
        if (__get_IPSR() != 0U) {
            return SpiTransferResult::Error;
        }

        /* ISR 只把 Active 变为 Complete/Error 并通知；任务在此原子认领一次终态，
         * 转入 Releasing 后复制 RX、释放两个 bounce，最后回到 Idle。 */
        TransferState terminal = TransferState::Idle;
        {
            const std::uint32_t primask = enter_critical();
            if (state_ == TransferState::Blocking ||
                state_ == TransferState::Preparing ||
                state_ == TransferState::Active ||
                state_ == TransferState::Releasing) {
                leave_critical(primask);
                return SpiTransferResult::Pending;
            }
            if (state_ == TransferState::Complete ||
                state_ == TransferState::Error) {
                terminal = state_;
                state_ = TransferState::Releasing;
            }
            leave_critical(primask);
        }

        if (terminal == TransferState::Idle) {
            return SpiTransferResult::Idle;
        }

        const bool success = terminal == TransferState::Complete;
        release_dma(success);
        return success ? SpiTransferResult::Complete
                       : SpiTransferResult::Error;
    }

    bool abort_transfer() noexcept override
    {
        if (__get_IPSR() != 0U) {
            return false;
        }

        bool active = false;
        {
            const std::uint32_t primask = enter_critical();
            if (state_ == TransferState::Idle) {
                leave_critical(primask);
                return true;
            }
            if (state_ == TransferState::Blocking ||
                state_ == TransferState::Preparing ||
                state_ == TransferState::Releasing) {
                leave_critical(primask);
                record_error(SpiErrorBusy);
                return false;
            }
            /* Blocking/Preparing/Releasing 由其他调用栈拥有，不能并发 abort；终态
             * 或 Active 可由当前任务认领并阻止后续 ISR 再调用原回调。 */
            active = state_ == TransferState::Active;
            state_ = TransferState::Releasing;
            notification_ = {};
            leave_critical(primask);
        }

        bool success = true;
        if (active && HAL_SPI_Abort(&hspi4) != HAL_OK) {
            success = false;
            record_error(SpiErrorPeripheral);
        }
        set_chip_select(false);
        release_dma(false);
        const std::uint32_t primask = enter_critical();
        ++stats_.aborts;
        stats_.last_error_flags |= SpiErrorAborted;
        leave_critical(primask);
        return success;
    }

    bool busy() const noexcept override
    {
        const std::uint32_t primask = enter_critical();
        const bool result = state_ != TransferState::Idle;
        leave_critical(primask);
        return result;
    }

    std::size_t maximum_dma_transfer_size() const noexcept override
    {
        return kMaximumDmaBytes;
    }

    SpiStats stats() const noexcept override
    {
        const std::uint32_t primask = enter_critical();
        const SpiStats snapshot = stats_;
        leave_critical(primask);
        return snapshot;
    }

    void on_complete() noexcept
    {
        IsrCallback notification{};
        if (state_ == TransferState::Active) {
            set_chip_select(false);
            state_ = TransferState::Complete;
            ++stats_.dma_completions;
            notification = notification_;
            __DMB();
        }
        notification.invoke();
    }

    void on_error() noexcept
    {
        IsrCallback notification{};
        if (state_ == TransferState::Active) {
            set_chip_select(false);
            state_ = TransferState::Error;
            ++stats_.errors;
            stats_.last_error_flags = SpiErrorPeripheral | SpiErrorDma;
            notification = notification_;
            __DMB();
        }
        notification.invoke();
    }

private:
    enum class TransferState : std::uint8_t {
        /* 主路径：Idle -> Preparing -> Active -> Complete/Error -> Releasing -> Idle；
         * Blocking 是独立轮询事务，所有状态由短 PRIMASK 临界区串行化。 */
        Idle = 0U,
        Blocking,
        Preparing,
        Active,
        Complete,
        Error,
        Releasing,
    };

    void release_preparation() noexcept
    {
        const std::uint32_t primask = enter_critical();
        if (state_ == TransferState::Preparing) {
            state_ = TransferState::Idle;
        }
        leave_critical(primask);
    }

    void release_dma(bool copy_receive) noexcept
    {
        DmaBufferView transmit_dma{};
        DmaBufferView receive_dma{};
        std::uint8_t *receive_destination = nullptr;
        {
            const std::uint32_t primask = enter_critical();
            transmit_dma = transmit_dma_;
            receive_dma = receive_dma_;
            receive_destination = receive_destination_;
            transmit_dma_ = {};
            receive_dma_ = {};
            receive_destination_ = nullptr;
            notification_ = {};
            leave_critical(primask);
        }

        dma_memory().release_bounce(
            receive_dma, copy_receive ? receive_destination : nullptr,
            DmaDirection::PeripheralToMemory);
        dma_memory().release_bounce(
            transmit_dma, nullptr, DmaDirection::MemoryToPeripheral);

        const std::uint32_t primask = enter_critical();
        state_ = TransferState::Idle;
        leave_critical(primask);
    }

    void record_error(std::uint32_t flags) noexcept
    {
        const std::uint32_t primask = enter_critical();
        ++stats_.errors;
        stats_.last_error_flags = flags;
        leave_critical(primask);
    }

    void record_clock_configuration(std::uint32_t kernel_frequency_hz,
                                    std::uint32_t requested_frequency_hz,
                                    std::uint32_t configured_frequency_hz)
        noexcept
    {
        const std::uint32_t primask = enter_critical();
        stats_.kernel_frequency_hz = kernel_frequency_hz;
        stats_.requested_frequency_hz = requested_frequency_hz;
        stats_.configured_frequency_hz = configured_frequency_hz;
        leave_critical(primask);
    }

    DmaBufferView transmit_dma_{};
    DmaBufferView receive_dma_{};
    std::uint8_t *receive_destination_{nullptr};
    IsrCallback notification_{};
    volatile TransferState state_{TransferState::Idle};
    SpiStats stats_{};
    bool initialized_{false};
};

Stm32Spi4Device &instance() noexcept
{
    static Stm32Spi4Device value;
    return value;
}

} // namespace

SpiDevice &spi4() noexcept { return instance(); }

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle)
{
    if (handle == &hspi4) {
        instance().on_complete();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    if (handle == &hspi4) {
        instance().on_error();
    }
}

extern "C" void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *handle)
{
    if (handle == &hspi4) {
        instance().on_error();
    }
}

} // namespace dima::platform::stm32h7

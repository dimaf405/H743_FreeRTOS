#include "UartDuplexDmaEndpoint.hpp"

#include "UartResources.hpp"
#include "stm32h7/HardwareServices.hpp"
#include <algorithm>
#include <cstring>

namespace dima::platform::stm32h7 {
namespace {

constexpr std::size_t kDmaBufferSize = 4096U;
constexpr std::size_t kReceiveRingCapacity = 8192U;
constexpr std::size_t kTransmitBufferSize = 192U;
constexpr std::uint32_t kDmaIrqPriority = 7U;
constexpr std::uint32_t kUartIrqPriority = 8U;
constexpr std::uint32_t kErrorRecoveryNotificationIntervalUs = 100000U;

/* DMA 环形缓冲接收连续硬件流，ISR 再把新增字节复制到 8 KiB 软件 SPSC ring；
 * ring 容量为 2 的幂，序号使用自然 uint32 回绕与 mask 定位。TX 使用固定 192 B
 * 缓冲，保证 HAL IT 发送期间不依赖调用方内存生命周期。 */
static_assert((kReceiveRingCapacity & (kReceiveRingCapacity - 1U)) == 0U,
              "duplex RX ring capacity must be a power of two");

alignas(32) std::uint8_t g_duplex_rx_dma_buffer[kDmaBufferSize]
    __attribute__((section(".dima_dma")));
DMA_HandleTypeDef g_duplex_rx_dma{};

struct UartDuplexDmaState {
    UART_HandleTypeDef *uart;
    UART_InitTypeDef original_init;
    UART_AdvFeatureInitTypeDef original_advanced_init;
    UartRxPinSnapshot original_rx_pin;
    IsrCallback notification;
    alignas(8) std::uint8_t receive_ring[kReceiveRingCapacity];
    alignas(8) std::uint8_t transmit_buffer[kTransmitBufferSize];
    std::uint32_t ring_write_sequence;
    std::uint32_t ring_read_sequence;
    std::uint64_t last_arrival_us;
    std::uint32_t last_error_notification_us;
    std::uint32_t received_bytes;
    std::uint32_t dropped_bytes;
    std::uint32_t receive_errors;
    std::uint32_t receive_error_flags;
    std::uint32_t transmit_errors;
    std::uint32_t line_changes;
    std::uint32_t original_fifo_mode;
    std::uint32_t original_tx_fifo_threshold;
    std::uint32_t original_rx_fifo_threshold;
    SerialLineConfiguration line_configuration;
    std::int32_t configured_port;
    std::uint16_t last_dma_position;
    bool dma_initialized;
    bool tx_pending;
    bool receive_fault;
    bool running;
};

UartDuplexDmaState g_duplex_state{};

class UartDuplexDmaEndpoint final : public AsyncSerialPort {
public:
    explicit UartDuplexDmaEndpoint(UartDuplexDmaState &state) noexcept
        : uart_(state.uart),
          original_init_(state.original_init),
          original_advanced_init_(state.original_advanced_init),
          original_rx_pin_(state.original_rx_pin),
          notification_(state.notification),
          receive_ring_(state.receive_ring),
          transmit_buffer_(state.transmit_buffer),
          ring_write_sequence_(state.ring_write_sequence),
          ring_read_sequence_(state.ring_read_sequence),
          last_arrival_us_(state.last_arrival_us),
          last_error_notification_us_(state.last_error_notification_us),
          received_bytes_(state.received_bytes),
          dropped_bytes_(state.dropped_bytes),
          receive_errors_(state.receive_errors),
          receive_error_flags_(state.receive_error_flags),
          transmit_errors_(state.transmit_errors),
          line_changes_(state.line_changes),
          original_fifo_mode_(state.original_fifo_mode),
          original_tx_fifo_threshold_(state.original_tx_fifo_threshold),
          original_rx_fifo_threshold_(state.original_rx_fifo_threshold),
          line_configuration_(state.line_configuration),
          configured_port_(state.configured_port),
          last_dma_position_(state.last_dma_position),
          dma_initialized_(state.dma_initialized),
          tx_pending_(state.tx_pending),
          receive_fault_(state.receive_fault),
          running_(state.running)
    {
    }

    bool configure(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept override
    {
        if (running() || uart_ != nullptr ||
            !uart_line_configuration_valid(configuration) ||
            !configuration.rx_enabled || !configuration.tx_enabled) {
            return false;
        }
        UART_HandleTypeDef *const uart = uart_for(port);
        UartRxPinSnapshot rx_pin{};
        if (uart == nullptr || request_for(port) == 0U ||
            !capture_rx_pin(port, rx_pin)) {
            return false;
        }

        /* 接管前保存 HAL 初始化、FIFO 阈值和 RX GPIO 位域；stop 必须恢复这组快照，
         * 使临时协议端点不永久改变板级正常串口配置。 */
        original_init_ = uart->Init;
        original_advanced_init_ = uart->AdvancedInit;
        original_rx_pin_ = rx_pin;
        original_fifo_mode_ = uart->FifoMode;
        original_tx_fifo_threshold_ = uart->Instance->CR3 & USART_CR3_TXFTCFG;
        original_rx_fifo_threshold_ = uart->Instance->CR3 & USART_CR3_RXFTCFG;
        configured_port_ = port;
        uart_ = uart;
        if (!initialize_uart(configuration)) {
            uart_ = nullptr;
            configured_port_ = 0;
            return false;
        }
        line_configuration_ = configuration;
        reset_runtime_state();
        return true;
    }

    bool start(IsrCallback notification) noexcept override
    {
        if (running()) {
            return true;
        }
        if (uart_ == nullptr || configured_port_ == 0 ||
            notification.function == nullptr || !initialize_dma()) {
            return false;
        }
        /* 先建立 DMA 与回调、清运行态，再 release 发布 running；arm 失败则撤销
         * running、回调和 DMA，不留下可被路由器误认领的半启动端点。 */
        notification_ = notification;
        reset_runtime_state();
        __atomic_store_n(&running_, true, __ATOMIC_RELEASE);
        if (!arm_receive()) {
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
            notification_ = {};
            deinitialize_dma();
            return false;
        }
        return true;
    }

    bool stop() noexcept override
    {
        /* 先撤销 running 使全局回调停止路由，再关 UART IRQ/DMA、abort，并恢复
         * 原配置；只有恢复完成后才释放 uart_ 所有权。 */
        __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        notification_ = {};
        if (uart_ == nullptr) {
            reset_runtime_state();
            return true;
        }

        const IRQn_Type uart_irq = irq_for(uart_);
        HAL_NVIC_DisableIRQ(uart_irq);
        HAL_NVIC_ClearPendingIRQ(uart_irq);
        deinitialize_dma();
        (void)HAL_UART_Abort(uart_);
        uart_->hdmarx = nullptr;
        const bool restored = restore_original_uart();
        uart_ = nullptr;
        configured_port_ = 0;
        line_configuration_ = {};
        reset_runtime_state();
        return restored;
    }

    bool set_line_configuration(
        const SerialLineConfiguration &configuration) noexcept override
    {
        if (uart_ == nullptr ||
            !uart_line_configuration_valid(configuration) ||
            !configuration.rx_enabled || !configuration.tx_enabled ||
            !tx_complete()) {
            return false;
        }
        if (uart_line_configuration_equal(configuration, line_configuration_)) {
            return true;
        }

        /* 改线参数是一笔可回滚事务：保存 previous，停止 RX 并重配；任一步失败
         * 尝试恢复旧配置及原 running 状态，不能静默留在混合线路参数。 */
        const bool was_running = running();
        const SerialLineConfiguration previous_configuration =
            line_configuration_;
        __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        deinitialize_dma();
        (void)HAL_UART_Abort(uart_);
        uart_->hdmarx = nullptr;
        if (!initialize_uart(configuration)) {
            (void)recover_previous_configuration(previous_configuration,
                                                 was_running);
            return false;
        }
        reset_runtime_state();
        if (was_running) {
            if (!initialize_dma() || !arm_receive()) {
                (void)recover_previous_configuration(previous_configuration,
                                                     true);
                return false;
            }
            __atomic_store_n(&running_, true, __ATOMIC_RELEASE);
        }
        line_configuration_ = configuration;
        (void)__atomic_add_fetch(&line_changes_, 1U, __ATOMIC_RELAXED);
        return true;
    }

    bool write(const std::uint8_t *data,
               std::size_t length) noexcept override
    {
        if (!running() || uart_ == nullptr || data == nullptr || length == 0U ||
            length > sizeof(transmit_buffer_) || !tx_complete()) {
            return false;
        }
        /* 先复制到端点固定缓冲，再交给 HAL 中断发送；tx_complete 为 true 前拒绝
         * 第二笔写入，避免覆盖仍由 UART 读取的数据。 */
        std::memcpy(transmit_buffer_, data, length);
        if (HAL_UART_Transmit_IT(
                uart_, transmit_buffer_, static_cast<std::uint16_t>(length)) !=
            HAL_OK) {
            (void)__atomic_add_fetch(&transmit_errors_, 1U,
                                     __ATOMIC_RELAXED);
            return false;
        }
        __atomic_store_n(&tx_pending_, true, __ATOMIC_RELEASE);
        return true;
    }

    bool tx_complete() const noexcept override
    {
        if (uart_ == nullptr) {
            return true;
        }
        const bool complete = uart_->gState == HAL_UART_STATE_READY;
        if (complete) {
            __atomic_store_n(&tx_pending_, false, __ATOMIC_RELEASE);
        }
        return complete;
    }

    std::size_t read(std::uint8_t *destination, std::size_t capacity,
                     std::uint64_t &last_arrival_us) noexcept override
    {
        if (destination == nullptr || capacity == 0U || !recover_receive()) {
            return 0U;
        }
        const std::uint32_t consumed =
            __atomic_load_n(&ring_read_sequence_, __ATOMIC_RELAXED);
        const std::uint32_t produced =
            __atomic_load_n(&ring_write_sequence_, __ATOMIC_ACQUIRE);
        /* 单调序号差在固定容量 ring 上可安全处理自然回绕；若差值超过容量说明
         * 状态不一致，清空 RX 而不是读取越界历史数据。 */
        const std::uint32_t available = produced - consumed;
        if (available > kReceiveRingCapacity) {
            clear_rx();
            return 0U;
        }
        const std::size_t count = std::min<std::size_t>(available, capacity);
        for (std::size_t index = 0U; index < count; ++index) {
            destination[index] = receive_ring_[
                (consumed + static_cast<std::uint32_t>(index)) &
                (kReceiveRingCapacity - 1U)];
        }
        __atomic_store_n(&ring_read_sequence_,
                         consumed + static_cast<std::uint32_t>(count),
                         __ATOMIC_RELEASE);
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        last_arrival_us = last_arrival_us_;
        if (primask == 0U) {
            __enable_irq();
        }
        return count;
    }

    void clear_rx() noexcept override
    {
        const std::uint32_t primask = __get_PRIMASK();
        __disable_irq();
        /* read 追到当前 write，同时把 DMA 游标对齐到硬件剩余计数；否则清空后下一
         * 回调会把清空前的 DMA 区间重新复制进软件 ring。 */
        const std::uint32_t produced = ring_write_sequence_;
        ring_read_sequence_ = produced;
        if (dma_initialized_ && uart_ != nullptr && uart_->hdmarx != nullptr) {
            const std::uint32_t remaining =
                __HAL_DMA_GET_COUNTER(&g_duplex_rx_dma);
            last_dma_position_ = static_cast<std::uint16_t>(
                (kDmaBufferSize - remaining) % kDmaBufferSize);
        }
        if (primask == 0U) {
            __enable_irq();
        }
    }

    bool running() const noexcept override
    {
        return __atomic_load_n(&running_, __ATOMIC_ACQUIRE);
    }

    bool allows_line_configuration() const noexcept
    {
        return !running() && uart_ == nullptr && !dma_initialized_;
    }

    std::int32_t port() const noexcept override { return configured_port_; }
    SerialLineConfiguration line_configuration() const noexcept override
    {
        return line_configuration_;
    }

    AsyncSerialPortStats stats() const noexcept override
    {
        return {
            __atomic_load_n(&received_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&dropped_bytes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&receive_errors_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&transmit_errors_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&line_changes_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&receive_error_flags_, __ATOMIC_ACQUIRE),
        };
    }

    bool handles_uart(const UART_HandleTypeDef *uart) const noexcept
    {
        return uart_ == uart;
    }

    void on_rx_position_from_isr(std::uint16_t position) noexcept
    {
        if (!running() || position == 0U || position > kDmaBufferSize) {
            return;
        }
        /* HAL position 范围为 1..buffer_size，满缓冲位置归一为 0。delta 按圆环
         * 前进距离计算；previous=0/full 的特殊回调表示本轮正好新增完整缓冲。 */
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

        /* 软件 ring 满时保留旧的未消费数据并丢弃最新字节，同时累计 dropped；
         * last_arrival_us 记录本批回调时间，不伪造逐字节时间戳。 */
        std::uint32_t write_sequence = ring_write_sequence_;
        const std::uint32_t read_sequence = ring_read_sequence_;
        std::uint32_t dropped = 0U;
        for (std::uint16_t offset = 0U; offset < delta; ++offset) {
            if (write_sequence - read_sequence < kReceiveRingCapacity) {
                const std::size_t dma_index =
                    (static_cast<std::size_t>(previous) + offset) %
                    kDmaBufferSize;
                receive_ring_[write_sequence & (kReceiveRingCapacity - 1U)] =
                    g_duplex_rx_dma_buffer[dma_index];
                ++write_sequence;
            } else {
                ++dropped;
            }
        }
        ring_write_sequence_ = write_sequence;
        last_dma_position_ = normalized;
        last_arrival_us_ = clock().now_us();
        (void)__atomic_add_fetch(&received_bytes_, delta, __ATOMIC_RELAXED);
        if (dropped != 0U) {
            (void)__atomic_add_fetch(&dropped_bytes_, dropped,
                                     __ATOMIC_RELAXED);
        }
        __DMB();
        notification_.invoke();
    }

    void on_error_from_isr(std::uint32_t error) noexcept
    {
        if (!handles_uart(uart_) || error == HAL_UART_ERROR_NONE) {
            return;
        }
        (void)__atomic_add_fetch(&receive_errors_, 1U, __ATOMIC_RELAXED);
        (void)__atomic_fetch_or(&receive_error_flags_,
                                translate_uart_error(error),
                                __ATOMIC_RELAXED);
        const bool fault_already_pending = __atomic_exchange_n(
            &receive_fault_, true, __ATOMIC_ACQ_REL);
        if (fault_already_pending) return;

        /* 线路参数不匹配时每个字节都可能触发 framing/noise error。ISR 只合并
         * receive_fault，通知频率限制为 10 Hz；真正 abort/clear/re-arm 由 read()
         * 所在任务上下文执行，避免形成 IRQ 与工作队列风暴。 */
        const std::uint32_t now_us =
            static_cast<std::uint32_t>(clock().now_us());
        const std::uint32_t last_us = __atomic_load_n(
            &last_error_notification_us_, __ATOMIC_RELAXED);
        if (last_us != 0U &&
            static_cast<std::uint32_t>(now_us - last_us) <
                kErrorRecoveryNotificationIntervalUs) {
            return;
        }
        __atomic_store_n(&last_error_notification_us_, now_us,
                         __ATOMIC_RELAXED);
        notification_.invoke();
    }

private:
    bool recover_previous_configuration(
        const SerialLineConfiguration &configuration,
        bool restart_receive) noexcept
    {
        /* 回滚也采用完整 stop/reinit/optional restart 序列；旧配置本身恢复失败时
         * 清空 line_configuration，使调用方明确看到端点不可继续使用。 */
        __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
        deinitialize_dma();
        (void)HAL_UART_Abort(uart_);
        if (uart_ != nullptr) {
            uart_->hdmarx = nullptr;
        }
        if (!initialize_uart(configuration)) {
            line_configuration_ = {};
            return false;
        }

        line_configuration_ = configuration;
        reset_runtime_state();
        if (!restart_receive) {
            return true;
        }
        if (!initialize_dma() || !arm_receive()) {
            deinitialize_dma();
            return false;
        }
        __atomic_store_n(&running_, true, __ATOMIC_RELEASE);
        return true;
    }

    bool initialize_uart(
        const SerialLineConfiguration &configuration) noexcept
    {
        return reinitialize_uart(uart_, configuration) &&
               configure_uart_rx_pull(configured_port_,
                                      configuration.rx_pull);
    }

    bool restore_original_uart() noexcept
    {
        if (uart_ == nullptr || HAL_UART_DeInit(uart_) != HAL_OK) {
            return false;
        }
        uart_->Init = original_init_;
        uart_->AdvancedInit = original_advanced_init_;
        return HAL_UART_Init(uart_) == HAL_OK &&
               HAL_UARTEx_SetTxFifoThreshold(
                   uart_, original_tx_fifo_threshold_) == HAL_OK &&
               HAL_UARTEx_SetRxFifoThreshold(
                   uart_, original_rx_fifo_threshold_) == HAL_OK &&
               (original_fifo_mode_ == UART_FIFOMODE_ENABLE
                    ? HAL_UARTEx_EnableFifoMode(uart_)
                    : HAL_UARTEx_DisableFifoMode(uart_)) == HAL_OK &&
               restore_rx_pin(original_rx_pin_);
    }

    bool initialize_dma() noexcept
    {
        if (uart_ == nullptr || dma_initialized_) {
            return false;
        }
        /* Stream3 是此端点的板级唯一资源，request 由生成的端口合同映射；DMA IRQ
         * 优先于 UART IRQ，保证圆环位置更新先于同拍 IDLE/error 处理。 */
        g_duplex_rx_dma = DMA_HandleTypeDef{};
        g_duplex_rx_dma.Instance = DMA1_Stream3;
        g_duplex_rx_dma.Init.Request = request_for(configured_port_);
        g_duplex_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        g_duplex_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
        g_duplex_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
        g_duplex_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        g_duplex_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        g_duplex_rx_dma.Init.Mode = DMA_CIRCULAR;
        g_duplex_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
        g_duplex_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&g_duplex_rx_dma) != HAL_OK) {
            return false;
        }
        dma_initialized_ = true;
        __HAL_LINKDMA(uart_, hdmarx, g_duplex_rx_dma);
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, kDmaIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
        const IRQn_Type uart_irq = irq_for(uart_);
        HAL_NVIC_SetPriority(uart_irq, kUartIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(uart_irq);
        HAL_NVIC_EnableIRQ(uart_irq);
        return true;
    }

    void deinitialize_dma() noexcept
    {
        HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
        if (uart_ != nullptr && dma_initialized_) {
            (void)HAL_UART_AbortReceive(uart_);
            (void)HAL_DMA_DeInit(&g_duplex_rx_dma);
            uart_->hdmarx = nullptr;
        }
        dma_initialized_ = false;
    }

    bool arm_receive() noexcept
    {
        return uart_ != nullptr && dma_initialized_ &&
               HAL_UARTEx_ReceiveToIdle_DMA(
                   uart_, g_duplex_rx_dma_buffer,
                   static_cast<std::uint16_t>(kDmaBufferSize)) == HAL_OK;
    }

    bool recover_receive() noexcept
    {
        /* 原子认领合并故障，仅一个任务执行恢复。重启失败时置 running=false，
         * 后续读返回 0，等待上层重新配置/启动。 */
        if (!__atomic_exchange_n(&receive_fault_, false, __ATOMIC_ACQ_REL)) {
            return running();
        }
        if (!running() || uart_ == nullptr) {
            return false;
        }
        (void)HAL_UART_AbortReceive(uart_);
        clear_rx();
        if (!arm_receive()) {
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
            return false;
        }
        return true;
    }

    void reset_runtime_state() noexcept
    {
        ring_write_sequence_ = 0U;
        ring_read_sequence_ = 0U;
        last_dma_position_ = 0U;
        last_arrival_us_ = 0U;
        last_error_notification_us_ = 0U;
        receive_fault_ = false;
        tx_pending_ = false;
        __atomic_store_n(&receive_error_flags_, 0U, __ATOMIC_RELAXED);
    }

    UART_HandleTypeDef *&uart_;
    UART_InitTypeDef &original_init_;
    UART_AdvFeatureInitTypeDef &original_advanced_init_;
    UartRxPinSnapshot &original_rx_pin_;
    IsrCallback &notification_;
    std::uint8_t (&receive_ring_)[kReceiveRingCapacity];
    std::uint8_t (&transmit_buffer_)[kTransmitBufferSize];
    std::uint32_t &ring_write_sequence_;
    std::uint32_t &ring_read_sequence_;
    std::uint64_t &last_arrival_us_;
    std::uint32_t &last_error_notification_us_;
    std::uint32_t &received_bytes_;
    std::uint32_t &dropped_bytes_;
    std::uint32_t &receive_errors_;
    std::uint32_t &receive_error_flags_;
    std::uint32_t &transmit_errors_;
    std::uint32_t &line_changes_;
    std::uint32_t &original_fifo_mode_;
    std::uint32_t &original_tx_fifo_threshold_;
    std::uint32_t &original_rx_fifo_threshold_;
    SerialLineConfiguration &line_configuration_;
    std::int32_t &configured_port_;
    std::uint16_t &last_dma_position_;
    bool &dma_initialized_;
    bool &tx_pending_;
    bool &receive_fault_;
    bool &running_;
};

UartDuplexDmaEndpoint &instance() noexcept
{
    static UartDuplexDmaEndpoint value{g_duplex_state};
    return value;
}

} // namespace

AsyncSerialPort &async_serial_port() noexcept { return instance(); }

bool uart_duplex_dma_endpoint_allows_line_configuration() noexcept
{
    return instance().allows_line_configuration();
}

bool uart_duplex_dma_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept
{
    auto &backend = instance();
    if (!backend.running() || !backend.handles_uart(uart)) {
        return false;
    }
    backend.on_rx_position_from_isr(position);
    return true;
}

bool uart_duplex_dma_endpoint_on_error(
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

extern "C" void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_duplex_rx_dma);
}

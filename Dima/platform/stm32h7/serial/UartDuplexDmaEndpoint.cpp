#include "UartDuplexDmaEndpoint.hpp"

#include "UartResources.hpp"
#include "UartTimestampedRxEndpoint.hpp"
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
    std::uint32_t recoveries;
    std::uint32_t recovery_failures;
    std::uint32_t original_fifo_mode;
    std::uint32_t original_tx_fifo_threshold;
    std::uint32_t original_rx_fifo_threshold;
    std::int32_t configured_port;
    std::uint16_t last_dma_position;
    bool dma_initialized;
    bool tx_pending;
    bool receive_fault;
    bool running;
};

// 线路默认值包含 data_bits=8、RX/TX=true。将这 12 B 配置独立保存，使
// 8 KiB RX Ring、TX 缓冲和其余全零状态进入 .bss，不再占 Flash 初始化镜像。
// 仍由同一个端点独占，启动/恢复时序、缓冲容量和独立 DMA 段均保持。
UartDuplexDmaState g_duplex_state{};
SerialLineConfiguration g_duplex_line_configuration{};
UartDuplexDmaState g_telemetry_state{};
SerialLineConfiguration g_telemetry_line_configuration{};
alignas(8) std::uint8_t g_duplex_rx_ring[kReceiveRingCapacity]{};
alignas(8) std::uint8_t g_duplex_tx_buffer[kTransmitBufferSize]{};
alignas(8) std::uint8_t g_telemetry_rx_ring[4096U]{};
alignas(32) std::uint8_t g_telemetry_rx_dma_buffer[1024U]
    __attribute__((section(".dima_dma")));
alignas(32) std::uint8_t g_telemetry_tx_dma_buffer[512U]
    __attribute__((section(".dima_dma")));
DMA_HandleTypeDef g_telemetry_rx_dma{};
DMA_HandleTypeDef g_telemetry_tx_dma{};

// 端点实现共享，缓冲与 DMA 资源按实例独占；第二端点复用同一套线路与收发生命周期。
struct DuplexResources {
    std::uint8_t *dma_buffer;
    std::size_t dma_size;
    std::uint8_t *receive_ring;
    std::size_t ring_capacity;
    std::uint8_t *transmit_buffer;
    std::size_t transmit_capacity;
    DMA_HandleTypeDef *rx_dma;
    DMA_Stream_TypeDef *rx_stream;
    IRQn_Type rx_irq;
    DMA_HandleTypeDef *tx_dma;
    DMA_Stream_TypeDef *tx_stream;
    IRQn_Type tx_irq;
};
const DuplexResources g_duplex_resources{
    g_duplex_rx_dma_buffer, sizeof(g_duplex_rx_dma_buffer),
    g_duplex_rx_ring, sizeof(g_duplex_rx_ring),
    g_duplex_tx_buffer, sizeof(g_duplex_tx_buffer),
    &g_duplex_rx_dma, DMA1_Stream3, DMA1_Stream3_IRQn,
    nullptr, nullptr, DMA1_Stream3_IRQn};
const DuplexResources g_telemetry_resources{
    g_telemetry_rx_dma_buffer, sizeof(g_telemetry_rx_dma_buffer),
    g_telemetry_rx_ring, sizeof(g_telemetry_rx_ring),
    g_telemetry_tx_dma_buffer, sizeof(g_telemetry_tx_dma_buffer),
    &g_telemetry_rx_dma, DMA1_Stream4, DMA1_Stream4_IRQn,
    &g_telemetry_tx_dma, DMA1_Stream5, DMA1_Stream5_IRQn};


class UartDuplexDmaEndpoint final : public AsyncSerialPort {
public:
    explicit UartDuplexDmaEndpoint(
        UartDuplexDmaState &state,
        SerialLineConfiguration &line_configuration,
        const DuplexResources &resources) noexcept
        : resources_(resources), uart_(state.uart),
          original_init_(state.original_init),
          original_advanced_init_(state.original_advanced_init),
          original_rx_pin_(state.original_rx_pin),
          notification_(state.notification),
          receive_ring_(resources.receive_ring),
          transmit_buffer_(resources.transmit_buffer),
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
          recoveries_(state.recoveries),
          recovery_failures_(state.recovery_failures),
          original_fifo_mode_(state.original_fifo_mode),
          original_tx_fifo_threshold_(state.original_tx_fifo_threshold),
          original_rx_fifo_threshold_(state.original_rx_fifo_threshold),
          line_configuration_(line_configuration),
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
            uart_duplex_dma_endpoint_port_in_use(port) ||
            uart_timestamped_rx_endpoint_port_in_use(port) ||
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
            (void)restore_original_uart();
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

    bool service() noexcept override { return recover_receive(); }

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
            length > resources_.transmit_capacity || !tx_complete()) {
            return false;
        }
        // 先复制并发布在途状态，再启动 IT/DMA，避免极短帧完成 IRQ 早于状态置位。
        // DMA 完成只代表缓冲搬运；真正可复用要等 UART TC 的 TxCplt 回调。
        std::memcpy(transmit_buffer_, data, length);
        __atomic_store_n(&tx_pending_, true, __ATOMIC_RELEASE);
        const HAL_StatusTypeDef result = resources_.tx_dma != nullptr
            ? HAL_UART_Transmit_DMA(uart_, transmit_buffer_, static_cast<std::uint16_t>(length))
            : HAL_UART_Transmit_IT(uart_, transmit_buffer_, static_cast<std::uint16_t>(length));
        if (result != HAL_OK) {
            __atomic_store_n(&tx_pending_, false, __ATOMIC_RELEASE);
            (void)__atomic_add_fetch(&transmit_errors_, 1U,
                                     __ATOMIC_RELAXED);
            return false;
        }
        return true;
    }

    void on_tx_complete_from_isr() noexcept
    {
        // HAL 的 TX 完成回调发生于 UART TC，而非 DMA TC；通知只唤醒 owner。
        __atomic_store_n(&tx_pending_, false, __ATOMIC_RELEASE);
        if (notification_.function != nullptr) {
            notification_.function(notification_.context);
        }
    }

    bool tx_complete() const noexcept override
    {
        if (uart_ == nullptr) {
            return true;
        }
        // HAL 错误/abort 也可能置 gState=READY，只有 TC 回调或显式恢复才能
        // 撤销 tx_pending；不能把 DMA 错误造成的 READY 当作正常发送完成。
        return !__atomic_load_n(&tx_pending_, __ATOMIC_ACQUIRE) &&
               uart_->gState == HAL_UART_STATE_READY;
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
        if (available > resources_.ring_capacity) {
            clear_rx();
            return 0U;
        }
        const std::size_t count = std::min<std::size_t>(available, capacity);
        for (std::size_t index = 0U; index < count; ++index) {
            destination[index] = receive_ring_[
                (consumed + static_cast<std::uint32_t>(index)) &
                (resources_.ring_capacity - 1U)];
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
                __HAL_DMA_GET_COUNTER(resources_.rx_dma);
            last_dma_position_ = static_cast<std::uint16_t>(
                (resources_.dma_size - remaining) % resources_.dma_size);
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
            __atomic_load_n(&recoveries_, __ATOMIC_ACQUIRE),
            __atomic_load_n(&recovery_failures_, __ATOMIC_ACQUIRE),
        };
    }

    bool handles_uart(const UART_HandleTypeDef *uart) const noexcept
    {
        return uart_ == uart;
    }

    void on_rx_position_from_isr(std::uint16_t position) noexcept
    {
        if (!running() || position == 0U || position > resources_.dma_size) {
            return;
        }
        /* HAL position 范围为 1..buffer_size，满缓冲位置归一为 0。delta 按圆环
         * 前进距离计算；previous=0/full 的特殊回调表示本轮正好新增完整缓冲。 */
        const std::uint16_t normalized =
            position == resources_.dma_size ? 0U : position;
        const std::uint16_t previous = last_dma_position_;
        const std::uint16_t delta =
            position == resources_.dma_size && previous == 0U
                ? static_cast<std::uint16_t>(resources_.dma_size)
                : (normalized >= previous
                       ? normalized - previous
                       : static_cast<std::uint16_t>(
                             resources_.dma_size - previous + normalized));
        if (delta == 0U) {
            return;
        }

        /* 软件 ring 满时保留旧的未消费数据并丢弃最新字节，同时累计 dropped；
         * last_arrival_us 记录本批回调时间，不伪造逐字节时间戳。 */
        std::uint32_t write_sequence = ring_write_sequence_;
        const std::uint32_t read_sequence = ring_read_sequence_;
        std::uint32_t dropped = 0U;
        for (std::uint16_t offset = 0U; offset < delta; ++offset) {
            if (write_sequence - read_sequence < resources_.ring_capacity) {
                const std::size_t dma_index =
                    (static_cast<std::size_t>(previous) + offset) %
                    resources_.dma_size;
                receive_ring_[write_sequence & (resources_.ring_capacity - 1U)] =
                    resources_.dma_buffer[dma_index];
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
            // 第二端点发生溢出后按一次接收故障恢复，旧 Ring/半帧不能跨丢字节边界使用。
            if (resources_.tx_dma != nullptr) __atomic_store_n(&receive_fault_, true, __ATOMIC_RELEASE);
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
        /* DMA stream 由固定实例资源提供，request 来自既有 UART 硬件映射；DMA IRQ
         * 优先于 UART IRQ，保证圆环位置更新先于同拍 IDLE/error 处理。 */
        *resources_.rx_dma = DMA_HandleTypeDef{};
        resources_.rx_dma->Instance = resources_.rx_stream;
        resources_.rx_dma->Init.Request = request_for(configured_port_);
        resources_.rx_dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
        resources_.rx_dma->Init.PeriphInc = DMA_PINC_DISABLE;
        resources_.rx_dma->Init.MemInc = DMA_MINC_ENABLE;
        resources_.rx_dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        resources_.rx_dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        resources_.rx_dma->Init.Mode = DMA_CIRCULAR;
        resources_.rx_dma->Init.Priority = DMA_PRIORITY_HIGH;
        resources_.rx_dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(resources_.rx_dma) != HAL_OK) {
            return false;
        }
        dma_initialized_ = true;
        __HAL_LINKDMA(uart_, hdmarx, (*resources_.rx_dma));
        HAL_NVIC_SetPriority(resources_.rx_irq, kDmaIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(resources_.rx_irq);
        HAL_NVIC_EnableIRQ(resources_.rx_irq);
        if (resources_.tx_dma != nullptr) {
            auto &tx = *resources_.tx_dma;
            tx = DMA_HandleTypeDef{};
            tx.Instance = resources_.tx_stream;
            tx.Init = resources_.rx_dma->Init;
            tx.Init.Request = tx_request_for(configured_port_);
            tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
            tx.Init.Mode = DMA_NORMAL;
            if (tx.Init.Request == 0U || HAL_DMA_Init(&tx) != HAL_OK) {
                (void)HAL_DMA_DeInit(&tx);
                deinitialize_dma();
                return false;
            }
            __HAL_LINKDMA(uart_, hdmatx, tx);
            HAL_NVIC_SetPriority(resources_.tx_irq, kDmaIrqPriority, 0U);
            HAL_NVIC_ClearPendingIRQ(resources_.tx_irq);
            HAL_NVIC_EnableIRQ(resources_.tx_irq);
        }
        const IRQn_Type uart_irq = irq_for(uart_);
        HAL_NVIC_SetPriority(uart_irq, kUartIrqPriority, 0U);
        HAL_NVIC_ClearPendingIRQ(uart_irq);
        HAL_NVIC_EnableIRQ(uart_irq);
        return true;
    }

    void deinitialize_dma() noexcept
    {
        HAL_NVIC_DisableIRQ(resources_.rx_irq);
        HAL_NVIC_ClearPendingIRQ(resources_.rx_irq);
        if (resources_.tx_dma != nullptr) {
            HAL_NVIC_DisableIRQ(resources_.tx_irq);
            HAL_NVIC_ClearPendingIRQ(resources_.tx_irq);
            if (uart_ != nullptr && uart_->hdmatx != nullptr) {
                (void)HAL_UART_AbortTransmit(uart_);
                (void)HAL_DMA_DeInit(resources_.tx_dma);
                uart_->hdmatx = nullptr;
            }
        }
        if (uart_ != nullptr && dma_initialized_) {
            (void)HAL_UART_AbortReceive(uart_);
            (void)HAL_DMA_DeInit(resources_.rx_dma);
            uart_->hdmarx = nullptr;
        }
        dma_initialized_ = false;
    }

    bool arm_receive() noexcept
    {
        return uart_ != nullptr && dma_initialized_ &&
               HAL_UARTEx_ReceiveToIdle_DMA(
                   uart_, resources_.dma_buffer,
                   static_cast<std::uint16_t>(resources_.dma_size)) == HAL_OK;
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
        if (resources_.tx_dma != nullptr && !tx_complete()) {
            (void)HAL_UART_AbortTransmit(uart_);
            __atomic_store_n(&tx_pending_, false, __ATOMIC_RELEASE);
            (void)__atomic_add_fetch(&transmit_errors_, 1U, __ATOMIC_RELAXED);
        }
        (void)HAL_UART_AbortReceive(uart_);
        clear_rx();
        if (!arm_receive()) {
            (void)__atomic_add_fetch(&recovery_failures_, 1U, __ATOMIC_RELAXED);
            __atomic_store_n(&running_, false, __ATOMIC_RELEASE);
            return false;
        }
        (void)__atomic_add_fetch(&recoveries_, 1U, __ATOMIC_RELAXED);
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

    const DuplexResources &resources_;
    UART_HandleTypeDef *&uart_;
    UART_InitTypeDef &original_init_;
    UART_AdvFeatureInitTypeDef &original_advanced_init_;
    UartRxPinSnapshot &original_rx_pin_;
    IsrCallback &notification_;
    std::uint8_t *const receive_ring_;
    std::uint8_t *const transmit_buffer_;
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
    std::uint32_t &recoveries_;
    std::uint32_t &recovery_failures_;
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
    static UartDuplexDmaEndpoint value{
        g_duplex_state, g_duplex_line_configuration, g_duplex_resources};
    return value;
}

UartDuplexDmaEndpoint &telemetry_instance() noexcept
{
    static UartDuplexDmaEndpoint value{
        g_telemetry_state, g_telemetry_line_configuration, g_telemetry_resources};
    return value;
}

} // namespace

AsyncSerialPort &async_serial_port() noexcept { return instance(); }

AsyncSerialPort &telemetry_serial_port() noexcept { return telemetry_instance(); }

bool uart_duplex_dma_endpoint_port_in_use(std::int32_t port) noexcept
{
    return (g_duplex_state.uart != nullptr && g_duplex_state.configured_port == port) ||
           (g_telemetry_state.uart != nullptr && g_telemetry_state.configured_port == port);
}

bool uart_duplex_dma_endpoint_allows_line_configuration() noexcept
{
    return instance().allows_line_configuration() &&
           telemetry_instance().allows_line_configuration();
}

bool uart_duplex_dma_endpoint_on_rx_event(
    UART_HandleTypeDef *uart, std::uint16_t position) noexcept
{
    for (auto *backend : {&instance(), &telemetry_instance()}) {
        if (backend->running() && backend->handles_uart(uart)) {
            backend->on_rx_position_from_isr(position);
            return true;
        }
    }
    return false;
}

bool uart_duplex_dma_endpoint_on_error(
    UART_HandleTypeDef *uart, std::uint32_t error) noexcept
{
    for (auto *backend : {&instance(), &telemetry_instance()}) {
        if (backend->running() && backend->handles_uart(uart)) {
            backend->on_error_from_isr(error);
            return true;
        }
    }
    return false;
}

void uart_duplex_dma_endpoint_on_tx_complete(UART_HandleTypeDef *uart) noexcept
{
    for (auto *backend : {&instance(), &telemetry_instance()}) {
        if (backend->running() && backend->handles_uart(uart)) {
            backend->on_tx_complete_from_isr();
            return;
        }
    }
}

} // namespace dima::platform::stm32h7

extern "C" void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_duplex_rx_dma);
}
extern "C" void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_telemetry_rx_dma);
}
extern "C" void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dima::platform::stm32h7::g_telemetry_tx_dma);
}

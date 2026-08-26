#include "Fdcan1.hpp"

#include "fdcan.h"
#include "main.h"
#include "api/Time.hpp"

#include <cstring>

namespace dima::platform::stm32h7 {

constexpr std::size_t kFdcanRxRingCapacity = 32U;

/* rx_head 仅由 ISR 推进，rx_tail 仅由消费任务推进，形成无锁 SPSC ring；统计量
 * 和 pending_error_flags 跨 ISR/任务共享，恢复动作延后到 service。 */
struct Fdcan1State {
    dima::platform::CanFrame rx_ring[kFdcanRxRingCapacity];
    volatile std::uint32_t rx_head;
    volatile std::uint32_t rx_tail;
    volatile std::uint32_t pending_error_flags;
    dima::platform::CanConfiguration configuration;
    volatile bool started;
    volatile std::uint32_t received_frames;
    volatile std::uint32_t transmitted_frames;
    volatile std::uint32_t receive_overruns;
    volatile std::uint32_t receive_errors;
    volatile std::uint32_t transmit_errors;
    volatile std::uint32_t bus_off_events;
    volatile std::uint32_t recovery_attempts;
    volatile std::uint32_t recovery_failures;
    volatile std::uint32_t last_error_flags;
};

namespace {

constexpr std::uint32_t kRecoveryErrorMask = FDCAN_IT_BUS_OFF;
/* PEA/PED 会为每个协议错误触发；在断线或速率不匹配时启用会形成 IRQ 风暴，
 * 进而饿死 IMU/UART DMA 路径。只保留 warning/passive 状态迁移和 bus-off 等
 * 有界健康信号，不按每个坏帧中断。 */
constexpr std::uint32_t kEnabledNotifications =
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
    FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
    FDCAN_IT_RAM_ACCESS_FAILURE |
    FDCAN_IT_RAM_WATCHDOG |
    FDCAN_IT_ERROR_PASSIVE |
    FDCAN_IT_ERROR_WARNING |
    FDCAN_IT_BUS_OFF;

Fdcan1State g_fdcan1_state{};

constexpr bool supported_bitrate(std::uint32_t bitrate) noexcept
{
    return bitrate == 125000U || bitrate == 250000U ||
           bitrate == 500000U || bitrate == 1000000U;
}

constexpr std::uint32_t identifier_mask(
    dima::platform::CanIdentifierType type) noexcept
{
    /* 标准帧 11 bit、扩展帧 29 bit；收发与过滤器都使用同一掩码合同。 */
    return type == dima::platform::CanIdentifierType::Standard
               ? 0x7FFU
               : 0x1FFFFFFFU;
}

constexpr bool valid_identifier_type(
    dima::platform::CanIdentifierType type) noexcept
{
    return type == dima::platform::CanIdentifierType::Standard ||
           type == dima::platform::CanIdentifierType::Extended;
}

constexpr bool valid_frame_type(dima::platform::CanFrameType type) noexcept
{
    return type == dima::platform::CanFrameType::Data ||
           type == dima::platform::CanFrameType::Remote;
}

constexpr bool valid_filter_type(dima::platform::CanFilterType type) noexcept
{
    return type == dima::platform::CanFilterType::Range ||
           type == dima::platform::CanFilterType::Dual ||
           type == dima::platform::CanFilterType::Mask;
}

constexpr bool valid_acceptance(
    dima::platform::CanAcceptance acceptance) noexcept
{
    return acceptance == dima::platform::CanAcceptance::Reject ||
           acceptance == dima::platform::CanAcceptance::Accept;
}

bool same_filter(const dima::platform::CanFilter &lhs,
                 const dima::platform::CanFilter &rhs) noexcept
{
    return lhs.enabled == rhs.enabled &&
           lhs.identifier_type == rhs.identifier_type &&
           lhs.type == rhs.type && lhs.identifier1 == rhs.identifier1 &&
           lhs.identifier2 == rhs.identifier2;
}

bool same_configuration(
    const dima::platform::CanConfiguration &lhs,
    const dima::platform::CanConfiguration &rhs) noexcept
{
    return lhs.bitrate == rhs.bitrate &&
           same_filter(lhs.acceptance.filter, rhs.acceptance.filter) &&
           lhs.acceptance.nonmatching_standard ==
               rhs.acceptance.nonmatching_standard &&
           lhs.acceptance.nonmatching_extended ==
               rhs.acceptance.nonmatching_extended &&
           lhs.acceptance.standard_remote ==
               rhs.acceptance.standard_remote &&
           lhs.acceptance.extended_remote ==
               rhs.acceptance.extended_remote;
}

std::uint32_t hal_identifier_type(
    dima::platform::CanIdentifierType type) noexcept
{
    return type == dima::platform::CanIdentifierType::Standard
               ? FDCAN_STANDARD_ID
               : FDCAN_EXTENDED_ID;
}

std::uint32_t hal_frame_type(dima::platform::CanFrameType type) noexcept
{
    return type == dima::platform::CanFrameType::Remote
               ? FDCAN_REMOTE_FRAME
               : FDCAN_DATA_FRAME;
}

std::uint32_t hal_filter_type(dima::platform::CanFilterType type) noexcept
{
    switch (type) {
    case dima::platform::CanFilterType::Range: return FDCAN_FILTER_RANGE;
    case dima::platform::CanFilterType::Dual: return FDCAN_FILTER_DUAL;
    case dima::platform::CanFilterType::Mask: return FDCAN_FILTER_MASK;
    }
    return FDCAN_FILTER_MASK;
}

std::uint32_t hal_nonmatching_action(
    dima::platform::CanAcceptance action) noexcept
{
    return action == dima::platform::CanAcceptance::Accept
               ? FDCAN_ACCEPT_IN_RX_FIFO0
               : FDCAN_REJECT;
}

std::uint32_t hal_remote_action(
    dima::platform::CanAcceptance action) noexcept
{
    return action == dima::platform::CanAcceptance::Accept
               ? FDCAN_FILTER_REMOTE
               : FDCAN_REJECT_REMOTE;
}

bool configure_filter_slot(std::uint32_t identifier_type, bool enabled,
                           const dima::platform::CanFilter &filter) noexcept
{
    FDCAN_FilterTypeDef hal_filter{};
    hal_filter.IdType = identifier_type;
    hal_filter.FilterIndex = 0U;
    hal_filter.FilterType = hal_filter_type(filter.type);
    hal_filter.FilterConfig = enabled ? FDCAN_FILTER_TO_RXFIFO0
                                      : FDCAN_FILTER_DISABLE;
    hal_filter.FilterID1 = enabled ? filter.identifier1 : 0U;
    hal_filter.FilterID2 = enabled ? filter.identifier2 : 0U;
    return HAL_FDCAN_ConfigFilter(&hfdcan1, &hal_filter) == HAL_OK;
}

} // namespace

Fdcan1::Fdcan1(Fdcan1State &state) noexcept : state_(&state) {}

bool Fdcan1::valid_configuration(
    const dima::platform::CanConfiguration &configuration) noexcept
{
    const auto &acceptance = configuration.acceptance;
    if (!supported_bitrate(configuration.bitrate) ||
        !valid_acceptance(acceptance.nonmatching_standard) ||
        !valid_acceptance(acceptance.nonmatching_extended) ||
        !valid_acceptance(acceptance.standard_remote) ||
        !valid_acceptance(acceptance.extended_remote)) {
        return false;
    }

    const auto &filter = acceptance.filter;
    if (!filter.enabled) return true;
    if (!valid_identifier_type(filter.identifier_type) ||
        !valid_filter_type(filter.type) ||
        filter.identifier1 > identifier_mask(filter.identifier_type) ||
        filter.identifier2 > identifier_mask(filter.identifier_type)) {
        return false;
    }
    return filter.identifier_type ==
                   dima::platform::CanIdentifierType::Standard
               ? hfdcan1.Init.StdFiltersNbr > 0U
               : hfdcan1.Init.ExtFiltersNbr > 0U;
}

bool Fdcan1::start(
    const dima::platform::CanConfiguration &configuration) noexcept
{
    auto &state = *state_;
    /* 相同配置保持幂等；配置变化必须先 silent+stop，再重建 timing/filter，避免
     * 收发过程中出现一半旧过滤器、一半新波特率。 */
    if (!valid_configuration(configuration)) return false;
    if (state.started && same_configuration(state.configuration,
                                            configuration)) {
        return true;
    }
    stop();
    state.configuration = configuration;
    reset_ring();
    return configure_and_start();
}

void Fdcan1::stop() noexcept
{
    auto &state = *state_;
    /* 先置收发器 silent，再停控制器和通知；即使 state.started 已丢失，也优先
     * 保证物理 TX 不再驱动总线。 */
    HAL_GPIO_WritePin(CAN1_SILENT_GPIO_Port, CAN1_SILENT_Pin, GPIO_PIN_SET);
    if (state.started) {
        (void)HAL_FDCAN_DeactivateNotification(&hfdcan1,
                                               kEnabledNotifications);
        (void)HAL_FDCAN_Stop(&hfdcan1);
    }
    state.started = false;
    __atomic_store_n(&state.pending_error_flags, 0U, __ATOMIC_RELEASE);
    reset_ring();
}

bool Fdcan1::configure_and_start() noexcept
{
    auto &state = *state_;
    const auto &configuration = state.configuration;
    const auto &acceptance = configuration.acceptance;
    const auto &filter = acceptance.filter;
    if (DIMA_FDCAN1_ConfigureBitrate(configuration.bitrate) != HAL_OK) {
        return false;
    }

    const bool standard_filter_enabled =
        filter.enabled && filter.identifier_type ==
                              dima::platform::CanIdentifierType::Standard;
    const bool extended_filter_enabled =
        filter.enabled && filter.identifier_type ==
                              dima::platform::CanIdentifierType::Extended;
    if ((hfdcan1.Init.StdFiltersNbr > 0U &&
         !configure_filter_slot(FDCAN_STANDARD_ID,
                                standard_filter_enabled, filter)) ||
        (hfdcan1.Init.ExtFiltersNbr > 0U &&
         !configure_filter_slot(FDCAN_EXTENDED_ID,
                                extended_filter_enabled, filter)) ||
        HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            hal_nonmatching_action(acceptance.nonmatching_standard),
            hal_nonmatching_action(acceptance.nonmatching_extended),
            hal_remote_action(acceptance.standard_remote),
            hal_remote_action(acceptance.extended_remote)) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(
            &hfdcan1, kEnabledNotifications, 0U) != HAL_OK) {
        return false;
    }

    /* 所有 HAL 配置成功后才解除 silent；HAL_FDCAN_Start 失败立即重新静默。 */
    HAL_GPIO_WritePin(CAN1_SILENT_GPIO_Port, CAN1_SILENT_Pin,
                      GPIO_PIN_RESET);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        HAL_GPIO_WritePin(CAN1_SILENT_GPIO_Port, CAN1_SILENT_Pin,
                          GPIO_PIN_SET);
        return false;
    }
    state.started = true;
    return true;
}

bool Fdcan1::service() noexcept
{
    auto &state = *state_;
    if (!state.started) return false;
    /* ISR 只 OR 错误位；任务原子取走批次。当前只有 bus-off 触发重启，普通
     * warning/passive 仅统计，避免在噪声总线上高频 stop/start。 */
    const std::uint32_t errors = __atomic_exchange_n(
        &state.pending_error_flags, 0U, __ATOMIC_ACQ_REL);
    if ((errors & kRecoveryErrorMask) == 0U) return true;

    ++state.recovery_attempts;
    state.started = false;
    HAL_GPIO_WritePin(CAN1_SILENT_GPIO_Port, CAN1_SILENT_Pin, GPIO_PIN_SET);
    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (!configure_and_start()) {
        ++state.recovery_failures;
        return false;
    }
    return true;
}

std::size_t Fdcan1::receive(dima::platform::CanFrame *frames,
                            std::size_t capacity) noexcept
{
    auto &state = *state_;
    if (frames == nullptr || capacity == 0U) return 0U;
    std::size_t count = 0U;
    while (count < capacity) {
        /* ISR 先写帧再 release head；消费者 acquire head 后读取完整槽，再推进 tail。 */
        const std::uint32_t tail = state.rx_tail;
        __DMB();
        if (tail == state.rx_head) break;
        frames[count++] = state.rx_ring[tail];
        __DMB();
        state.rx_tail = (tail + 1U) % kFdcanRxRingCapacity;
    }
    return count;
}

dima::platform::CanTransmitResult Fdcan1::transmit(
    const dima::platform::CanFrame &frame) noexcept
{
    auto &state = *state_;
    if (!state.started || frame.data_length > 8U ||
        !valid_identifier_type(frame.identifier_type) ||
        !valid_frame_type(frame.frame_type) ||
        frame.identifier > identifier_mask(frame.identifier_type)) {
        ++state.transmit_errors;
        return dima::platform::CanTransmitResult::Error;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        return dima::platform::CanTransmitResult::Busy;
    }

    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = frame.identifier;
    header.IdType = hal_identifier_type(frame.identifier_type);
    header.TxFrameType = hal_frame_type(frame.frame_type);
    header.DataLength = static_cast<std::uint32_t>(frame.data_length) << 16U;
    /* STM32 HAL 以 DLC 字段位于 [19:16] 的编码接收 Classic CAN 0..8 字节长度。 */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    std::uint8_t payload[8]{};
    std::memcpy(payload, frame.data, frame.data_length);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, payload) != HAL_OK) {
        ++state.transmit_errors;
        return dima::platform::CanTransmitResult::Error;
    }
    ++state.transmitted_frames;
    return dima::platform::CanTransmitResult::Sent;
}

dima::platform::CanStats Fdcan1::stats() const noexcept
{
    const auto &state = *state_;
    return dima::platform::CanStats{
        state.received_frames, state.transmitted_frames,
        state.receive_overruns, state.receive_errors,
        state.transmit_errors, state.bus_off_events,
        state.recovery_attempts, state.recovery_failures,
        state.last_error_flags};
}

bool Fdcan1::push_from_isr(
    const dima::platform::CanFrame &frame) noexcept
{
    auto &state = *state_;
    const std::uint32_t head = state.rx_head;
    const std::uint32_t next = (head + 1U) % kFdcanRxRingCapacity;
    /* 留空一个槽区分满/空；ring 满时丢弃最新帧并计 overrun，不覆盖尚未消费帧。 */
    if (next == state.rx_tail) {
        ++state.receive_overruns;
        return false;
    }
    state.rx_ring[head] = frame;
    __DMB();
    state.rx_head = next;
    ++state.received_frames;
    return true;
}

void Fdcan1::handle_rx_fifo0_irq() noexcept
{
    /* 单次 IRQ 排空硬件 FIFO0，校验 HAL header 后才写软件 ring；时间戳取软件
     * 收到帧的单调时间，不声称是 CAN 总线 SOF 的硬件捕获时间。 */
    auto &state = *state_;
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header{};
        std::uint8_t payload[8]{};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                   &header, payload) != HAL_OK) {
            ++state.receive_errors;
            break;
        }
        const std::uint32_t length = (header.DataLength >> 16U) & 0xFU;
        if ((header.IdType != FDCAN_STANDARD_ID &&
             header.IdType != FDCAN_EXTENDED_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME &&
             header.RxFrameType != FDCAN_REMOTE_FRAME) ||
            header.FDFormat != FDCAN_CLASSIC_CAN || length > 8U) {
            ++state.receive_errors;
            continue;
        }
        dima::platform::CanFrame frame{};
        frame.timestamp_us = hrt_absolute_time();
        frame.identifier_type = header.IdType == FDCAN_STANDARD_ID
                                    ? dima::platform::CanIdentifierType::Standard
                                    : dima::platform::CanIdentifierType::Extended;
        frame.frame_type = header.RxFrameType == FDCAN_REMOTE_FRAME
                               ? dima::platform::CanFrameType::Remote
                               : dima::platform::CanFrameType::Data;
        frame.identifier =
            header.Identifier & identifier_mask(frame.identifier_type);
        frame.data_length = static_cast<std::uint8_t>(length);
        std::memcpy(frame.data, payload, frame.data_length);
        (void)push_from_isr(frame);
    }
}

void Fdcan1::handle_error_irq(std::uint32_t flags) noexcept
{
    auto &state = *state_;
    state.last_error_flags = flags;
    if ((flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        ++state.receive_overruns;
    }
    if ((flags & FDCAN_IT_BUS_OFF) != 0U) ++state.bus_off_events;
    if ((flags & (FDCAN_IT_RAM_ACCESS_FAILURE |
                  FDCAN_IT_RAM_WATCHDOG |
                  FDCAN_IT_ARB_PROTOCOL_ERROR |
                  FDCAN_IT_DATA_PROTOCOL_ERROR |
                  FDCAN_IT_ERROR_PASSIVE |
                  FDCAN_IT_ERROR_WARNING)) != 0U) {
        ++state.receive_errors;
    }
    __atomic_fetch_or(&state.pending_error_flags, flags, __ATOMIC_RELEASE);
}

void Fdcan1::handle_hal_error_irq(std::uint32_t error) noexcept
{
    if (error == HAL_FDCAN_ERROR_NONE) return;
    auto &state = *state_;
    state.last_error_flags = error;
    ++state.receive_errors;
}

void Fdcan1::reset_ring() noexcept
{
    state_->rx_head = 0U;
    state_->rx_tail = 0U;
}

bool Fdcan1::running() const noexcept { return state_->started; }

Fdcan1 &fdcan1() noexcept
{
    static Fdcan1 value{g_fdcan1_state};
    return value;
}

dima::platform::CanTransport &can1() noexcept
{
    return fdcan1();
}

extern "C" void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *handle, std::uint32_t interrupts)
{
    if (handle != nullptr && handle->Instance == FDCAN1 &&
        (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        dima::platform::stm32h7::fdcan1().handle_rx_fifo0_irq();
    }
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(
    FDCAN_HandleTypeDef *handle, std::uint32_t interrupts)
{
    if (handle != nullptr && handle->Instance == FDCAN1) {
        dima::platform::stm32h7::fdcan1().handle_error_irq(interrupts);
    }
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *handle)
{
    if (handle != nullptr && handle->Instance == FDCAN1) {
        dima::platform::stm32h7::fdcan1().handle_hal_error_irq(
            handle->ErrorCode);
    }
}

} // namespace dima::platform::stm32h7

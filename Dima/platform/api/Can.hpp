#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum class CanIdentifierType : std::uint8_t {
    Standard = 0U,
    Extended,
};

enum class CanFrameType : std::uint8_t {
    Data = 0U,
    Remote,
};

struct CanFrame {
    /* timestamp_us 为帧进入软件接收路径的单调时钟时间；Classic CAN 负载固定上限
     * 8 字节，data_length 必须在 0..8，identifier 是否为 11/29 位由类型显式给出。 */
    std::uint64_t timestamp_us{0U};
    std::uint32_t identifier{0U};
    std::uint8_t data[8]{};
    std::uint8_t data_length{0U};
    CanIdentifierType identifier_type{CanIdentifierType::Extended};
    CanFrameType frame_type{CanFrameType::Data};
};

enum class CanFilterType : std::uint8_t {
    Range = 0U,
    Dual,
    Mask,
};

enum class CanAcceptance : std::uint8_t {
    Reject = 0U,
    Accept,
};

/** 单个硬件验收过滤器；匹配帧进入 RX FIFO0，两个 identifier 字段的解释由
 * Range/Dual/Mask 决定，禁止调用方自行拼接 HAL 寄存器编码。 */
struct CanFilter {
    bool enabled{false};
    CanIdentifierType identifier_type{CanIdentifierType::Extended};
    CanFilterType type{CanFilterType::Mask};
    std::uint32_t identifier1{0U};
    std::uint32_t identifier2{0U};
};

struct CanAcceptanceConfiguration {
    CanFilter filter{};
    CanAcceptance nonmatching_standard{CanAcceptance::Reject};
    CanAcceptance nonmatching_extended{CanAcceptance::Reject};
    CanAcceptance standard_remote{CanAcceptance::Reject};
    CanAcceptance extended_remote{CanAcceptance::Reject};
};

struct CanConfiguration {
    std::uint32_t bitrate{0U};
    CanAcceptanceConfiguration acceptance{};
};

enum class CanTransmitResult : std::uint8_t {
    Sent,
    Busy,
    Error,
};

struct CanStats {
    /* 全部计数自 start 后累计；last_error_flags 保存最近一次 HAL 错误位快照，
     * 它不是当前总线状态，也不能单独作为硬件故障结论。 */
    std::uint32_t received_frames{0U};
    std::uint32_t transmitted_frames{0U};
    std::uint32_t receive_overruns{0U};
    std::uint32_t receive_errors{0U};
    std::uint32_t transmit_errors{0U};
    std::uint32_t bus_off_events{0U};
    std::uint32_t recovery_attempts{0U};
    std::uint32_t recovery_failures{0U};
    std::uint32_t last_error_flags{0U};
};

/** Classic CAN 传输边界。ISR 只收发和记账，协议解析始终留在任务上下文；
 * service() 负责处理 bus-off 等延后恢复状态机。 */
class CanTransport {
public:
    virtual ~CanTransport() = default;

    virtual bool start(const CanConfiguration &configuration) noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual bool service() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual std::size_t receive(CanFrame *frames,
                                std::size_t capacity) noexcept = 0;
    virtual CanTransmitResult transmit(const CanFrame &frame) noexcept = 0;
    virtual CanStats stats() const noexcept = 0;
};

} // namespace dima::platform

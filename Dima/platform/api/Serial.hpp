#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

enum class SerialParity : std::uint8_t {
    None = 0U,
    Even,
    Odd,
};

enum class SerialStopBits : std::uint8_t {
    One = 0U,
    Two,
};

enum class SerialRxPull : std::uint8_t {
    Preserve = 0U,
    None,
    Up,
    Down,
};

struct SerialLineConfiguration {
    /* 线路合同必须一次性提交；rx/tx 关闭、反相及上下拉均属于电气配置，不能在
     * 驱动运行期间被不同消费者分别修改。 */
    std::uint32_t baudrate{0U};
    std::uint8_t data_bits{8U};
    SerialParity parity{SerialParity::None};
    SerialStopBits stop_bits{SerialStopBits::One};
    SerialRxPull rx_pull{SerialRxPull::Preserve};
    bool rx_enabled{true};
    bool tx_enabled{true};
    bool rx_inverted{false};
    bool tx_inverted{false};
};

struct TimestampedSerialInputStats {
    /* 计数由具体端点按其配置生命周期累计，消费者要计算会话增量时应在 start
     * 处保存基线；error_flags 是分类位集合而不是错误次数。 */
    std::uint32_t received_bytes{0U};
    std::uint32_t dropped_bytes{0U};
    std::uint32_t receive_errors{0U};
    std::uint32_t recovery_failures{0U};
    std::uint32_t receive_error_flags{0U};
};

struct AsyncSerialPortStats {
    std::uint32_t received_bytes{0U};
    std::uint32_t dropped_bytes{0U};
    std::uint32_t receive_errors{0U};
    std::uint32_t transmit_errors{0U};
    std::uint32_t line_changes{0U};
    std::uint32_t receive_error_flags{0U};
};

enum SerialInputError : std::uint32_t {
    SerialInputErrorNone = 0U,
    SerialInputErrorParity = 1U << 0U,
    SerialInputErrorNoise = 1U << 1U,
    SerialInputErrorFraming = 1U << 2U,
    SerialInputErrorOverrun = 1U << 3U,
    SerialInputErrorDma = 1U << 4U,
    SerialInputErrorTimeout = 1U << 5U,
    SerialInputErrorUnknown = 1U << 31U,
};

class SerialPorts {
public:
    virtual ~SerialPorts() = default;
    virtual bool configure_line(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept = 0;
    virtual bool reset_configuration() noexcept = 0;
};

class TimestampedSerialInput {
public:
    virtual ~TimestampedSerialInput() = default;
    virtual bool configure(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept = 0;
    virtual bool start(IsrCallback notification) noexcept = 0;
    virtual bool stop() noexcept = 0;
    /* read 为每个返回字节提供对应到达时间；数组容量必须与 destination 相同，
     * 供 SBUS 等固定帧协议判断真实线路间隔，而不是任务被调度的时间。 */
    virtual std::size_t read(std::uint8_t *destination,
                             std::uint64_t *arrival_timestamps_us,
                             std::size_t capacity) noexcept = 0;
    virtual bool service() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual TimestampedSerialInputStats stats() const noexcept = 0;
};

class AsyncSerialPort {
public:
    virtual ~AsyncSerialPort() = default;
    virtual bool configure(
        std::int32_t port,
        const SerialLineConfiguration &configuration) noexcept = 0;
    virtual bool start(IsrCallback notification) noexcept = 0;
    virtual bool stop() noexcept = 0;
    /* 动态改线参数只能由当前端口所有者调用；实现必须先停止 DMA/清错误，再以
     * 新配置恢复，失败时保持可判定的停止状态。 */
    virtual bool set_line_configuration(
        const SerialLineConfiguration &configuration) noexcept = 0;
    virtual bool write(const std::uint8_t *data,
                       std::size_t length) noexcept = 0;
    virtual bool tx_complete() const noexcept = 0;
    virtual std::size_t read(std::uint8_t *destination,
                             std::size_t capacity,
                             std::uint64_t &last_arrival_us) noexcept = 0;
    virtual void clear_rx() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual std::int32_t port() const noexcept = 0;
    virtual SerialLineConfiguration line_configuration() const noexcept = 0;
    virtual AsyncSerialPortStats stats() const noexcept = 0;
};

} // namespace dima::platform

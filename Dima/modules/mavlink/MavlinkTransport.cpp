#define MODULE_NAME "mavlink"
#include "MavlinkTransport.hpp"
#include "api/Time.hpp"
#include "logging/logging.hpp"

#include <cerrno>

namespace dima::modules::mavlink {

UsbMavlinkTransport::UsbMavlinkTransport(dima::platform::Console &console) noexcept
    : console_(console)
{
}
void UsbMavlinkTransport::service() noexcept { console_.service(); }
bool UsbMavlinkTransport::ready() const noexcept { return console_.ready(); }
std::size_t UsbMavlinkTransport::read(std::uint8_t *data, std::size_t size) noexcept
{
    return console_.read(data, size);
}
int UsbMavlinkTransport::write(const std::uint8_t *data, std::size_t size,
                              std::uint32_t timeout_ms) noexcept
{
    return console_.write(data, size, timeout_ms);
}
bool UsbMavlinkTransport::tx_idle() noexcept { return console_.tx_idle(); }
std::size_t UsbMavlinkTransport::tx_free_bytes() noexcept
{
    return ready() && tx_idle() ? dima::platform::Console::kWriteCapacity : 0U;
}
std::uint32_t UsbMavlinkTransport::baudrate() const noexcept { return 0U; }
std::uint32_t UsbMavlinkTransport::error_generation() const noexcept { return 0U; }

SerialMavlinkTransport::SerialMavlinkTransport(dima::platform::AsyncSerialPort &port) noexcept
    : port_(port)
{
}
bool SerialMavlinkTransport::open(std::int32_t number, std::uint32_t baud,
                                 dima::platform::IsrCallback callback) noexcept
{
    // 所有权来自已提交的 SerialConfig；不尝试 Auto 波特率，也不抢其他协议端口。
    if (number <= 0 || baud == 0U || !port_.stop()) return false;
    dima::platform::SerialLineConfiguration line{};
    line.baudrate = baud;
    if (!port_.configure(number, line) || !port_.start(callback)) {
        (void)port_.stop();
        return false;
    }
    return true;
}
bool SerialMavlinkTransport::close() noexcept { return port_.stop(); }
void SerialMavlinkTransport::service() noexcept
{
    (void)port_.service();
    const auto generation = error_generation();
    const auto now = hrt_absolute_time();
    // 累计诊断可通过任一 QGC 消息面板观察；连续错速/噪声最多每秒报告一次。
    if (generation != reported_errors_ &&
        (last_error_report_us_ == 0U || now - last_error_report_us_ >= 1000000ULL)) {
        const auto stats = port_.stats();
        PX4_WARN("UART MAVLink drop=%lu rxerr=%lu txerr=%lu recover=%lu fail=%lu",
            static_cast<unsigned long>(stats.dropped_bytes),
            static_cast<unsigned long>(stats.receive_errors),
            static_cast<unsigned long>(stats.transmit_errors),
            static_cast<unsigned long>(stats.recoveries),
            static_cast<unsigned long>(stats.recovery_failures));
        reported_errors_ = generation;
        last_error_report_us_ = now;
    }
}
bool SerialMavlinkTransport::ready() const noexcept { return port_.running(); }
std::size_t SerialMavlinkTransport::read(std::uint8_t *data, std::size_t size) noexcept
{
    std::uint64_t arrival{};
    return port_.read(data, size, arrival);
}
int SerialMavlinkTransport::write(const std::uint8_t *data, std::size_t size,
                                 std::uint32_t) noexcept
{
    if (!ready()) { errno = EPIPE; return -1; }
    if (!port_.tx_complete()) { errno = EAGAIN; return -1; }
    if (!port_.write(data, size)) { errno = EIO; return -1; }
    return static_cast<int>(size);
}
bool SerialMavlinkTransport::tx_idle() noexcept { return port_.tx_complete(); }
std::size_t SerialMavlinkTransport::tx_free_bytes() noexcept
{
    return ready() && tx_idle() ? 512U : 0U;
}
std::uint32_t SerialMavlinkTransport::baudrate() const noexcept
{
    return port_.line_configuration().baudrate;
}
std::uint32_t SerialMavlinkTransport::error_generation() const noexcept
{
    // 统计是累计值；任何 UART/DMA 故障或 RX 丢字节都撤销当前协议半帧。
    const auto stats = port_.stats();
    return stats.receive_errors + stats.transmit_errors + stats.dropped_bytes;
}

} // namespace dima::modules::mavlink

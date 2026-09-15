#pragma once

#include "api/Console.hpp"
#include "api/Serial.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::modules::mavlink {

// write 成功表示完整字节已被传输层接收；tx_idle 仅表示可复用缓冲。
// 完成代数与发送失败代数共同区分正常发送、在途等待和中止后的空闲。
// UART 绝不等待线路发送，USB 保留原有受统一截止时间约束的 Console 写入。
class MavlinkTransport {
public:
    virtual ~MavlinkTransport() = default;
    virtual void service() noexcept = 0;
    virtual bool ready() const noexcept = 0;
    virtual std::size_t read(std::uint8_t *data, std::size_t size) noexcept = 0;
    virtual int write(const std::uint8_t *data, std::size_t size,
                      std::uint32_t timeout_ms) noexcept = 0;
    virtual bool tx_idle() noexcept = 0;
    virtual std::size_t tx_free_bytes() noexcept = 0;
    virtual std::uint32_t baudrate() const noexcept = 0;
    virtual std::uint32_t error_generation() const noexcept = 0;
    virtual std::uint32_t tx_completion_generation() const noexcept = 0;
    virtual std::uint32_t tx_error_generation() const noexcept = 0;
};

class UsbMavlinkTransport final : public MavlinkTransport {
public:
    explicit UsbMavlinkTransport(dima::platform::Console &console) noexcept;
    void service() noexcept override;
    bool ready() const noexcept override;
    std::size_t read(std::uint8_t *data, std::size_t size) noexcept override;
    int write(const std::uint8_t *data, std::size_t size,
              std::uint32_t timeout_ms) noexcept override;
    bool tx_idle() noexcept override;
    std::size_t tx_free_bytes() noexcept override;
    std::uint32_t baudrate() const noexcept override;
    std::uint32_t error_generation() const noexcept override;
    std::uint32_t tx_completion_generation() const noexcept override;
    std::uint32_t tx_error_generation() const noexcept override;
private:
    dima::platform::Console &console_;
    std::uint32_t tx_completions_{0U};
};

class SerialMavlinkTransport final : public MavlinkTransport {
public:
    explicit SerialMavlinkTransport(dima::platform::AsyncSerialPort &port) noexcept;
    bool open(std::int32_t number, std::uint32_t baud,
              dima::platform::IsrCallback callback) noexcept;
    bool close() noexcept;
    void service() noexcept override;
    bool ready() const noexcept override;
    std::size_t read(std::uint8_t *data, std::size_t size) noexcept override;
    int write(const std::uint8_t *data, std::size_t size,
              std::uint32_t timeout_ms) noexcept override;
    bool tx_idle() noexcept override;
    std::size_t tx_free_bytes() noexcept override;
    std::uint32_t baudrate() const noexcept override;
    std::uint32_t error_generation() const noexcept override;
    std::uint32_t tx_completion_generation() const noexcept override;
    std::uint32_t tx_error_generation() const noexcept override;
private:
    dima::platform::AsyncSerialPort &port_;
    std::uint64_t last_error_report_us_{0U};
    std::uint32_t reported_errors_{0U};
};

} // namespace dima::modules::mavlink

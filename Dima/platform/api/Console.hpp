#pragma once

#include <cstddef>
#include <cstdint>

namespace dima::platform {

enum class ConsoleTransmitResult : std::uint8_t {
    Accepted,
    Busy,
    Failed,
};

class ConsoleTransport {
public:
    virtual ~ConsoleTransport() = default;
    virtual bool initialize() noexcept = 0;
    virtual void service() noexcept = 0;
    virtual bool ready() const noexcept = 0;
    virtual ConsoleTransmitResult transmit(const std::uint8_t *data,
                                             std::size_t length) noexcept = 0;
};

class Console {
public:
    virtual ~Console() = default;
    virtual bool initialize() noexcept = 0;
    virtual bool shutdown() noexcept = 0;
    virtual void service() noexcept = 0;
    virtual bool ready() const noexcept = 0;
    virtual int write(const std::uint8_t *data, std::size_t length,
                      std::uint32_t timeout_ms) noexcept = 0;
    virtual std::size_t read(std::uint8_t *data,
                             std::size_t capacity) noexcept = 0;
    virtual bool read_byte(std::uint8_t &byte) noexcept = 0;
    virtual std::size_t available() const noexcept = 0;
    virtual std::uint32_t overflow_count() const noexcept = 0;
    virtual void receive_from_isr(const std::uint8_t *data,
                                  std::size_t length) noexcept = 0;
    virtual void transmit_complete_from_isr() noexcept = 0;
    virtual void transport_connected_from_isr() noexcept = 0;
    virtual void transport_disconnected_from_isr() noexcept = 0;
};

} // namespace dima::platform

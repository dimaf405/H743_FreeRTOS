#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

struct SbusInputStats {
    std::uint32_t received_bytes{0U};
    std::uint32_t overwritten_bytes{0U};
    std::uint32_t receive_errors{0U};
    std::uint32_t recovery_failures{0U};
    std::uint32_t receive_error_flags{0U};
};

enum SbusInputError : std::uint32_t {
    SbusInputErrorNone = 0U,
    SbusInputErrorParity = 1U << 0U,
    SbusInputErrorNoise = 1U << 1U,
    SbusInputErrorFraming = 1U << 2U,
    SbusInputErrorOverrun = 1U << 3U,
    SbusInputErrorDma = 1U << 4U,
    SbusInputErrorTimeout = 1U << 5U,
    SbusInputErrorUnknown = 1U << 31U,
};

class SerialPorts {
public:
    virtual ~SerialPorts() = default;
    /** Apply the normal 8N1 baudrate for a real board SERIAL port.
     *  Auto/0 leaves final baud selection to the function driver. */
    virtual bool configure_normal_baud(std::int32_t port,
                                       std::uint32_t baudrate) noexcept = 0;
    virtual bool reset_normal_configuration() noexcept = 0;
};

class SbusInput {
public:
    virtual ~SbusInput() = default;
    virtual bool configure(std::int32_t port) noexcept = 0;
    virtual bool start(IsrCallback notification) noexcept = 0;
    virtual bool stop() noexcept = 0;
    virtual std::size_t read(std::uint8_t *destination,
                             std::uint64_t *arrival_timestamps_us,
                             std::size_t capacity) noexcept = 0;
    virtual bool service() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual SbusInputStats stats() const noexcept = 0;
};

} // namespace dima::platform

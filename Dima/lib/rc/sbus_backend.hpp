#pragma once

#include "work_queue/WorkQueue.hpp"

#include <cstddef>
#include <cstdint>

namespace dima::rc {

struct SbusBackendStats {
    std::uint32_t received_bytes;
    std::uint32_t overwritten_bytes;
    std::uint32_t receive_errors;
    std::uint32_t recovery_failures;
};

class SbusBackend {
public:
    virtual ~SbusBackend() = default;
    virtual bool configure(std::int32_t port, bool inverted) noexcept = 0;
    virtual bool start(px4::WorkItem &consumer) noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual std::size_t read(std::uint8_t *destination,
                             std::size_t capacity) noexcept = 0;
    virtual bool service() noexcept = 0;
    virtual bool running() const noexcept = 0;
    virtual SbusBackendStats stats() const noexcept = 0;
};

} // namespace dima::rc

#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

enum Icm42688InterruptMask : std::uint32_t {
    Icm42688InterruptNone = 0U,
    Icm42688InterruptInt1 = 1U << 0U,
    Icm42688InterruptInt2 = 1U << 1U,
};

struct Icm42688InterruptSnapshot {
    std::uint32_t pending_mask{0U};
    std::uint32_t int1_count{0U};
    std::uint32_t int2_count{0U};
    std::uint64_t int1_timestamp_us{0U};
    std::uint64_t int2_timestamp_us{0U};
};

class SensorInterrupts {
public:
    virtual ~SensorInterrupts() = default;
    virtual bool register_icm42688(IsrCallback notification) noexcept = 0;
    virtual void unregister_icm42688() noexcept = 0;
    virtual Icm42688InterruptSnapshot consume_icm42688() noexcept = 0;
};

} // namespace dima::platform

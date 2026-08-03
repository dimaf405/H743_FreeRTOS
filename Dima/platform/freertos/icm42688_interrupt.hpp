#pragma once

#include <cstdint>

namespace px4 {
class WorkItem;
}

namespace dima::platform {

enum Icm42688InterruptMask : std::uint32_t {
    Icm42688InterruptNone = 0U,
    Icm42688InterruptInt1 = 1U << 0U,
    Icm42688InterruptInt2 = 1U << 1U,
};

struct Icm42688InterruptSnapshot {
    std::uint32_t pending_mask;
    std::uint32_t int1_count;
    std::uint32_t int2_count;
    std::uint64_t int1_timestamp_us;
    std::uint64_t int2_timestamp_us;
};

// Registration and snapshot access are task-context-only operations.
bool icm42688_interrupt_register(px4::WorkItem &consumer) noexcept;
void icm42688_interrupt_unregister(px4::WorkItem &consumer) noexcept;
Icm42688InterruptSnapshot icm42688_interrupt_consume() noexcept;

} // namespace dima::platform

extern "C" void dima_icm42688_exti_isr(std::uint16_t pending_pins);

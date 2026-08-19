/*
 * Read the STM32H7 96-bit unique device ID (UID).
 *
 * The UID is stored at UID_BASE (3 × 32-bit words).
 * For STM32H743: UID_BASE = 0x1FF1E800.
 *
 * We combine word0 and word1 into a 64-bit value, which provides
 * sufficient uniqueness for MAVLink AUTOPILOT_VERSION uid field.
 */

#include "platform/stm32h7/HardwareServices.hpp"

#include <cstdint>

/* CMSIS device header defines UID_BASE. */
#include "stm32h7xx.h"

namespace dima::platform::stm32h7 {

std::uint64_t board_hardware_uid() noexcept
{
    const volatile uint32_t *uid =
        reinterpret_cast<const volatile uint32_t *>(UID_BASE);

    /* Combine the first two 32-bit words into a 64-bit UID. */
    return static_cast<std::uint64_t>(uid[0])
         | (static_cast<std::uint64_t>(uid[1]) << 32);
}

}  // namespace dima::platform::stm32h7

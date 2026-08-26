/*
 * Read the STM32H7 96-bit unique device ID (UID).
 *
 * The UID is stored at UID_BASE (3 × 32-bit words).
 * For STM32H743: UID_BASE = 0x1FF1E800.
 *
 * We combine word0 and word1 into a 64-bit value, which provides
 * sufficient uniqueness for the product hardware identity contract.
 */

/* 中文语义：STM32H743 在 UID_BASE 提供 3 个 32-bit 唯一字。本产品取 word0
 * 为低 32 位、word1 为高 32 位组成 64-bit 板级身份；它用于设备匹配，不是
 * 加密密钥，也不编码固件版本或 PCB 修订。 */
#include "stm32h7/HardwareServices.hpp"

#include <cstdint>

/* CMSIS device header defines UID_BASE. */
#include "stm32h7xx.h"

namespace dima::platform::stm32h7 {

std::uint64_t board_hardware_uid() noexcept
{
    const volatile uint32_t *uid =
        reinterpret_cast<const volatile uint32_t *>(UID_BASE);

    /* Combine the first two 32-bit words into a 64-bit UID. */
    /* 数值拼接公式：uid64 = word0 | (word1 << 32)。 */
    return static_cast<std::uint64_t>(uid[0])
         | (static_cast<std::uint64_t>(uid[1]) << 32);
}

}  // namespace dima::platform::stm32h7

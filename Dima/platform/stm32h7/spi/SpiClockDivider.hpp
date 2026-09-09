#pragma once

#include <cstdint>

namespace dima::platform::stm32h7::spi_clock {

struct Selection {
    std::uint16_t divisor{0U};
    std::uint32_t frequency_hz{0U};

    explicit operator bool() const noexcept;
};

/** 从 STM32 支持的 2^n 分频中选择“不超过设备上限”的最快 SCK。 */
Selection select(std::uint32_t kernel_frequency_hz,
                           std::uint32_t maximum_frequency_hz) noexcept;

} // namespace dima::platform::stm32h7::spi_clock

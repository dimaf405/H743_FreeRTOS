#pragma once

#include <cstdint>

namespace dima::platform::stm32h7::spi_clock {

struct Selection {
    std::uint16_t divisor{0U};
    std::uint32_t frequency_hz{0U};

    explicit constexpr operator bool() const noexcept
    {
        return divisor != 0U && frequency_hz != 0U;
    }
};

/** 从 STM32 支持的 2^n 分频中选择“不超过设备上限”的最快 SCK。 */
constexpr Selection select(std::uint32_t kernel_frequency_hz,
                           std::uint32_t maximum_frequency_hz) noexcept
{
    if (kernel_frequency_hz == 0U || maximum_frequency_hz == 0U) {
        return {};
    }

    constexpr std::uint16_t divisors[]{2U, 4U, 8U, 16U, 32U,
                                       64U, 128U, 256U};
    for (const std::uint16_t divisor : divisors) {
        const std::uint32_t frequency = kernel_frequency_hz / divisor;
        /* 用 kernel <= max * divisor 比较精确有理数，再报告整数 Hz；若只比较
         * 向下取整的 kernel/divisor，可能接受实际超上限不到 1 Hz 的配置。 */
        if (frequency != 0U &&
            static_cast<std::uint64_t>(kernel_frequency_hz) <=
                static_cast<std::uint64_t>(maximum_frequency_hz) * divisor) {
            return {divisor, frequency};
        }
    }
    return {};
}

} // namespace dima::platform::stm32h7::spi_clock

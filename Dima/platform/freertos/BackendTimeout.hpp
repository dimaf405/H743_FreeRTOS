#pragma once

#include "api/PlatformTypes.hpp"
#include "api/platform_config.h"

extern "C" {
#include "FreeRTOS.h"
}

namespace dima::platform::freertos {
namespace {

// 仅由 Backend.cpp 包含，以内部链接保留原 timeout 换算机器码；不得作为公共契约引用。
constexpr std::uint64_t kMicrosecondsPerSecond = 1000000ULL;

TickType_t timeout_to_ticks(Timeout timeout) noexcept
{
    if (timeout.infinite) {
        return portMAX_DELAY;
    }
    if (timeout.microseconds == 0U) {
        return 0U;
    }

    constexpr TickType_t kMaximumFiniteTicks = portMAX_DELAY - 1U;
    constexpr std::uint64_t kMaximumFiniteUs =
        (static_cast<std::uint64_t>(kMaximumFiniteTicks) *
         kMicrosecondsPerSecond) /
        DIMA_KERNEL_TICK_HZ;
    // 有限超时必须饱和在永久等待哨兵之前，不能被 FreeRTOS 误解释为 forever。
    if (timeout.microseconds >= kMaximumFiniteUs) {
        return kMaximumFiniteTicks;
    }

    /* 向上取整：ticks = ceil(us * tick_hz / 1e6)，确保非零有限超时不会被截断成
     * 立即返回；超过可表达范围时饱和到 portMAX_DELAY-1，保留 forever 哨兵。 */
    std::uint64_t ticks =
        (timeout.microseconds * DIMA_KERNEL_TICK_HZ +
         kMicrosecondsPerSecond - 1U) /
        kMicrosecondsPerSecond;
    if (ticks == 0U) {
        ticks = 1U;
    }
    return static_cast<TickType_t>(ticks);
}

} // namespace
} // namespace dima::platform::freertos

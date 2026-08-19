#pragma once

#include "Execution.hpp"

using hrt_abstime = std::uint64_t;

inline hrt_abstime hrt_absolute_time() noexcept
{
    return dima::platform::platform_time_us();
}

inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then) noexcept
{
    return then == nullptr ? 0U : hrt_absolute_time() - *then;
}

inline std::uint64_t hrt_absolute_time_ms() noexcept
{
    return dima::platform::platform_time_ms();
}

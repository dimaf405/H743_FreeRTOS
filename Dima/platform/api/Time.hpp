#pragma once

#include "Execution.hpp"

using hrt_abstime = std::uint64_t;

hrt_abstime hrt_absolute_time() noexcept;

hrt_abstime hrt_elapsed_time(const hrt_abstime *then) noexcept;

std::uint64_t hrt_absolute_time_ms() noexcept;

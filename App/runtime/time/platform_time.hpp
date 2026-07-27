#pragma once

#include <stdint.h>

namespace app::runtime::time {

uint64_t platform_time_us();
uint64_t platform_time_ms();

} // namespace app::runtime::time

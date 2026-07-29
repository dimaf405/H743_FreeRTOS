#include "App/runtime/time/platform_time.hpp"

#include "Dima/platform/freertos/hrt.hpp"

namespace app::runtime::time {

uint64_t platform_time_us()
{
    return hrt_absolute_time();
}

uint64_t platform_time_ms()
{
    return hrt_absolute_time_ms();
}

} // namespace app::runtime::time

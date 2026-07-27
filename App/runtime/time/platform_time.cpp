#include "App/runtime/time/platform_time.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace app::runtime::time {
namespace {

uint64_t extended_tick_count()
{
    TimeOut_t snapshot{};
    vTaskSetTimeOutState(&snapshot);
    const uint64_t overflow = static_cast<uint32_t>(snapshot.xOverflowCount);
    return (overflow << 32U) | static_cast<uint32_t>(snapshot.xTimeOnEntering);
}

} // namespace

uint64_t platform_time_us()
{
    return (extended_tick_count() * 1000000ULL) /
           static_cast<uint32_t>(configTICK_RATE_HZ);
}

uint64_t platform_time_ms()
{
    return (extended_tick_count() * 1000ULL) /
           static_cast<uint32_t>(configTICK_RATE_HZ);
}

} // namespace app::runtime::time

#pragma once

#include "api/PlatformTypes.hpp"
#include "api/platform_config.h"

extern "C" {
#include "FreeRTOS.h"
}

namespace dima::platform::freertos {

// 仅供 FreeRTOS 后端调用；时间换算实体位于同名源文件，不作为上层公共契约。
constexpr std::uint64_t kMicrosecondsPerSecond = 1000000ULL;

TickType_t timeout_to_ticks(Timeout timeout) noexcept;

} // namespace dima::platform::freertos

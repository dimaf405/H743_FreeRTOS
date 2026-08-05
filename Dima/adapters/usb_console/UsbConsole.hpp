#pragma once

#include "platform/api/Platform.hpp"

namespace dima::adapters {

platform::Console &usb_console(
    platform::Synchronization &synchronization,
    platform::TaskRuntime &tasks,
    platform::ExecutionContext &execution,
    platform::MonotonicClock &clock,
    platform::ConsoleTransport &transport) noexcept;

} // namespace dima::adapters

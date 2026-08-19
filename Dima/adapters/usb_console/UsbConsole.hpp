#pragma once

#include "platform/api/Console.hpp"
#include "platform/api/Execution.hpp"
#include "platform/api/Synchronization.hpp"
#include "platform/api/TaskRuntime.hpp"

namespace dima::adapters {

platform::Console &usb_console(
    platform::Synchronization &synchronization,
    platform::TaskRuntime &tasks,
    platform::ExecutionContext &execution,
    platform::MonotonicClock &clock,
    platform::ConsoleTransport &transport) noexcept;

} // namespace dima::adapters

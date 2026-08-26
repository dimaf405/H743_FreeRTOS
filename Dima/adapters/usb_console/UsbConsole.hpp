#pragma once

#include "api/Console.hpp"
#include "api/Execution.hpp"
#include "api/Synchronization.hpp"
#include "api/TaskRuntime.hpp"

namespace dima::adapters {

// 返回进程期唯一 USB Console 适配器。依赖均由板级组合根持有，适配器只保存引用；
// 调用方不得用不同后端重复构造同一静态实例。
platform::Console &usb_console(
    platform::Synchronization &synchronization,
    platform::TaskRuntime &tasks,
    platform::ExecutionContext &execution,
    platform::MonotonicClock &clock,
    platform::ConsoleTransport &transport) noexcept;

} // namespace dima::adapters

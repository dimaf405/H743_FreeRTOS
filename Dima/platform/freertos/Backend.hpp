#pragma once

#include "platform/api/Execution.hpp"
#include "platform/api/Flash.hpp"
#include "platform/api/Memory.hpp"
#include "platform/api/Synchronization.hpp"
#include "platform/api/TaskRuntime.hpp"

namespace dima::platform::freertos {

bool initialize() noexcept;
ExecutionContext &execution_context() noexcept;
CriticalSection &critical_section() noexcept;
Synchronization &synchronization() noexcept;
TaskRuntime &task_runtime() noexcept;
Heap &heap() noexcept;
FlashTransactionManager &flash_transactions() noexcept;

} // namespace dima::platform::freertos

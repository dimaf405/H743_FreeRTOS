#pragma once

#include "platform/api/Platform.hpp"

namespace dima::platform::freertos {

bool initialize() noexcept;
ExecutionContext &execution_context() noexcept;
CriticalSection &critical_section() noexcept;
Synchronization &synchronization() noexcept;
TaskRuntime &task_runtime() noexcept;
Heap &heap() noexcept;
FlashTransactionManager &flash_transactions() noexcept;

} // namespace dima::platform::freertos

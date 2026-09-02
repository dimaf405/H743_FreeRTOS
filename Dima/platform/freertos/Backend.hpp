#pragma once

#include "api/Execution.hpp"
#include "api/Flash.hpp"
#include "api/Memory.hpp"
#include "api/AtomicFileStore.hpp"
#include "api/LogFileStore.hpp"
#include "api/Synchronization.hpp"
#include "api/TaskRuntime.hpp"

namespace dima::platform::freertos {

/* 单一 Backend 同时实现执行上下文、同步、任务、heap 和 Flash 事务接口；
 * initialize 必须在 Services 发布前完成，后续访问器只返回该静态实例的不同视图。 */
bool initialize() noexcept;
ExecutionContext &execution_context() noexcept;
CriticalSection &critical_section() noexcept;
Synchronization &synchronization() noexcept;
TaskRuntime &task_runtime() noexcept;
Heap &heap() noexcept;
FlashTransactionManager &flash_transactions() noexcept;
AtomicFileStore &atomic_file_store() noexcept;
LogFileStore &log_file_store() noexcept;

} // namespace dima::platform::freertos

#pragma once

#include "PlatformTypes.hpp"
#include "platform_config.h"

namespace dima::platform {

using TaskEntry = void (*)(void *argument);

struct TaskConfig {
    const char *name{nullptr};
    std::uint8_t priority{0U};
    std::uint32_t stack_bytes{0U};
    bool realtime{false};
};

struct CpuUsage {
    std::uint64_t timestamp_us{0U};
    std::uint32_t window_us{0U};
    std::uint16_t load_permille{0U};
    std::uint16_t task_count{0U};
    bool valid{false};
};

struct TaskCpuUsage {
    // id 是内核分配的任务代次标识，不是可操作任务的 capability 句柄。
    std::uint32_t id{0U};
    std::uint32_t runtime_counter{0U};
    std::uint16_t load_permille{0U};
    bool valid{false};
    char name[DIMA_TASK_NAME_CAPACITY]{};
};

class TaskRuntime {
public:
    virtual ~TaskRuntime() = default;
    virtual TaskHandle create(const TaskConfig &config, TaskEntry entry,
                              void *argument) noexcept = 0;
    virtual bool destroy(TaskHandle handle) noexcept = 0;
    virtual TaskHandle current() const noexcept = 0;
    virtual void suspend_current() noexcept = 0;
    virtual void delay(Timeout duration) noexcept = 0;
    // cpu_usage 维护低频快照；任务明细只读，不暴露 FreeRTOS 的 TCB 或句柄。
    virtual CpuUsage cpu_usage() noexcept = 0;
    virtual bool task_cpu_usage(std::size_t index,
                                TaskCpuUsage &usage) noexcept = 0;
};

} // namespace dima::platform

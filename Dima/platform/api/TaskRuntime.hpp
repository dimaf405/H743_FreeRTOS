#pragma once

#include "PlatformTypes.hpp"

namespace dima::platform {

using TaskEntry = void (*)(void *argument);

struct TaskConfig {
    const char *name{nullptr};
    std::uint8_t priority{0U};
    std::uint32_t stack_bytes{0U};
    bool realtime{false};
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
};

} // namespace dima::platform

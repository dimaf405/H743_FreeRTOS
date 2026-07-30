#pragma once

#include <stdint.h>

namespace dima::middleware::lifecycle {

enum class ModuleState : uint8_t {
    Stopped,
    Running,
    Error,
};

class ModuleBase {
public:
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual ModuleState state() const = 0;

protected:
    ModuleBase() = default;
    ~ModuleBase() = default;
};

} // namespace dima::middleware::lifecycle

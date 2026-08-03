#pragma once

#include "module_base.hpp"

#include <stdint.h>

namespace dima::middleware::lifecycle {

class ModuleManager {
public:
    static constexpr uint8_t kMaxModules = 16U;

    bool register_module(ModuleBase &module);
    bool start(ModuleBase &module);
    bool stop(ModuleBase &module);
    ModuleState status(const ModuleBase &module) const;
    void reset();

private:
    bool registered(const ModuleBase &module) const;

    ModuleBase *modules_[kMaxModules]{};
    uint8_t module_count_{0U};
};

} // namespace dima::middleware::lifecycle

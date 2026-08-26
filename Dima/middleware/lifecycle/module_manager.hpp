#pragma once

#include "module_base.hpp"

#include <stdint.h>

namespace dima::middleware::lifecycle {

class ModuleManager {
public:
    /* 模块表固定容量且不拥有对象；调用方必须保证 ModuleBase 生命周期覆盖注册期。
     * start/stop 只允许合法状态迁移，reset 用于组合根按逆序结束后清注册关系。 */
    static constexpr uint8_t kMaxModules = 24U;

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

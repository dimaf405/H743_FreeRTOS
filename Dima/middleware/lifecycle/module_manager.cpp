#include "Dima/middleware/lifecycle/module_manager.hpp"

namespace dima::middleware::lifecycle {

bool ModuleManager::register_module(ModuleBase &module)
{
    if (registered(module) || module_count_ >= kMaxModules) {
        return false;
    }

    modules_[module_count_] = &module;
    ++module_count_;
    return true;
}

bool ModuleManager::start(ModuleBase &module)
{
    return registered(module) && module.start();
}

bool ModuleManager::stop(ModuleBase &module)
{
    if (!registered(module)) {
        return false;
    }

    module.stop();
    return true;
}

ModuleState ModuleManager::status(const ModuleBase &module) const
{
    return registered(module) ? module.state() : ModuleState::Error;
}

void ModuleManager::reset()
{
    for (auto &module : modules_) {
        module = nullptr;
    }
    module_count_ = 0U;
}

bool ModuleManager::registered(const ModuleBase &module) const
{
    for (uint8_t index = 0U; index < module_count_; ++index) {
        if (modules_[index] == &module) {
            return true;
        }
    }
    return false;
}

} // namespace dima::middleware::lifecycle

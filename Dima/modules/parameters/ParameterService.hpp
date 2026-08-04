#pragma once

#include "lifecycle/module_base.hpp"
#include "parameters/autosave.h"
#include "work_queue/ScheduledWorkItem.hpp"

namespace dima::modules::parameters {

using FlashWriteAllowedHook = bool (*)();

bool flash_write_allowed() noexcept;
void set_flash_write_allowed_hook(FlashWriteAllowedHook hook) noexcept;

class ParameterService final : public dima::middleware::lifecycle::ModuleBase,
                               public px4::ScheduledWorkItem {
public:
    ParameterService() noexcept;
    bool init() noexcept;
    bool start() noexcept override;
    void stop() noexcept override;
    dima::middleware::lifecycle::ModuleState state() const noexcept override;

private:
    void Run() override;
    ParamAutosave autosave_{};
    dima::middleware::lifecycle::ModuleState state_{
        dima::middleware::lifecycle::ModuleState::Stopped};
    bool initialized_{false};
};

} // namespace dima::modules::parameters

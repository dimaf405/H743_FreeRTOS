#pragma once

#include "parameters/autosave.h"
#include "work_queue/ScheduledWorkItem.hpp"

namespace dima::product::rover {

using FlashWriteAllowedHook = bool (*)();

bool flash_write_allowed() noexcept;
void set_flash_write_allowed_hook(FlashWriteAllowedHook hook) noexcept;

class ParameterService final : public px4::ScheduledWorkItem {
public:
    ParameterService() noexcept;
    bool init() noexcept;
    bool start() noexcept;
    void stop() noexcept;

private:
    void Run() override;
    ParamAutosave autosave_{};
    bool initialized_{false};
    bool started_{false};
};

} // namespace dima::product::rover

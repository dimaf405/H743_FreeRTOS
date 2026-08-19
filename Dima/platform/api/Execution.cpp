#include "Execution.hpp"

#include "Services.hpp"

namespace dima::platform {

CriticalGuard::CriticalGuard() noexcept
{
    Services *const installed = try_services();
    if (installed != nullptr) {
        section_ = &installed->critical;
        token_ = section_->enter();
    }
}

CriticalGuard::CriticalGuard(CriticalSection &section) noexcept
    : section_(&section), token_(section.enter())
{
}

CriticalGuard::~CriticalGuard()
{
    if (section_ != nullptr) {
        section_->leave(token_);
    }
}

bool in_interrupt_context() noexcept
{
    Services *const installed = try_services();
    return installed != nullptr && installed->execution.in_interrupt();
}

bool in_realtime_context() noexcept
{
    Services *const installed = try_services();
    return installed != nullptr && installed->execution.in_realtime_task();
}

TimeUs platform_time_us() noexcept
{
    Services *const installed = try_services();
    return installed != nullptr ? installed->clock.now_us() : 0U;
}

TimeMs platform_time_ms() noexcept { return platform_time_us() / 1000U; }

} // namespace dima::platform

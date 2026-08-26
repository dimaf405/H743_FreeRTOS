#include "api/Execution.hpp"

#include "api/Services.hpp"

namespace dima::platform {

CriticalGuard::CriticalGuard() noexcept
{
    /* 启动极早期允许 Services 尚未发布，此时 guard 为空操作；发布后则保存 enter
     * 返回的上下文 token，确保 ISR 与任务分别使用匹配的退出原语。 */
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
    /* 未安装时钟时返回 0，0 因而只表示“时间服务不可用/起点”，不能解释为一条
     * 有效传感器到达时间。 */
    Services *const installed = try_services();
    return installed != nullptr ? installed->clock.now_us() : 0U;
}

/* 整数除法向下取整：毫秒接口只用于粗粒度超时，不保留微秒余数。 */
TimeMs platform_time_ms() noexcept { return platform_time_us() / 1000U; }

} // namespace dima::platform

#include "api/Boot.hpp"

namespace dima::platform {

StartupResetInfo StartupDiagnostics::reset_info() const noexcept
{
    // 不支持保留复位记录的平台明确返回未知，不伪造 watchdog 或启动计数。
    return {};
}

} // namespace dima::platform

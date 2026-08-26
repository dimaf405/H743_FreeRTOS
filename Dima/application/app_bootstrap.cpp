#include "app_main.h"

#include "api/Services.hpp"
#include "api/TaskRuntime.hpp"

namespace {

// appMain 是应用 Runtime 的唯一顶层任务。句柄保存在进程期静态存储中，
// 既防止 CubeMX 重复调用时创建第二个所有者，也不把任务生命周期泄漏给 C 入口。
constexpr std::uint32_t kAppMainStackBytes = 2048U;
constexpr std::uint8_t kAppMainPriority = 24U;
constexpr char kAppMainTaskName[] = "appMainTask";
dima::platform::TaskHandle g_app_main_task{};

} // namespace

extern "C" bool app_bootstrap_create(void)
{
    // 创建接口必须幂等：已有句柄表示 Runtime 已被接管，直接报告成功。
    if (g_app_main_task) {
        return true;
    }
    // 硬件组合根必须先安装 Services；否则任务一旦开始便会访问未绑定的后端。
    if (!dima::platform::services_installed()) {
        return false;
    }
    const dima::platform::TaskConfig config{
        kAppMainTaskName, kAppMainPriority, kAppMainStackBytes, false};
    g_app_main_task = dima::platform::services().tasks.create(
        config, app_main_task, nullptr);
    return static_cast<bool>(g_app_main_task);
}

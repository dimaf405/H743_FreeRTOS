#include "app_main.h"

#include "platform/api/Services.hpp"
#include "platform/api/TaskRuntime.hpp"

namespace {

constexpr std::uint32_t kAppMainStackBytes = 2048U;
constexpr std::uint8_t kAppMainPriority = 24U;
constexpr char kAppMainTaskName[] = "appMainTask";
dima::platform::TaskHandle g_app_main_task{};

} // namespace

extern "C" bool app_bootstrap_create(void)
{
    if (g_app_main_task) {
        return true;
    }
    if (!dima::platform::services_installed()) {
        return false;
    }
    const dima::platform::TaskConfig config{
        kAppMainTaskName, kAppMainPriority, kAppMainStackBytes, false};
    g_app_main_task = dima::platform::services().tasks.create(
        config, app_main_task, nullptr);
    return static_cast<bool>(g_app_main_task);
}

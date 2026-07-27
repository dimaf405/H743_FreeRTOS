#include "App/application/app_main.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace {

constexpr uint32_t kAppMainStackBytes = 2048U;
constexpr UBaseType_t kAppMainPriority = 24U;
constexpr char kAppMainTaskName[] = "appMainTask";

StaticTask_t g_app_main_task_buffer{};
StackType_t g_app_main_stack[kAppMainStackBytes / sizeof(StackType_t)]{};
TaskHandle_t g_app_main_task_handle{nullptr};

static_assert(sizeof(g_app_main_stack) == kAppMainStackBytes,
              "app_main stack must remain exactly 2048 bytes");
static_assert(sizeof(kAppMainTaskName) <= configMAX_TASK_NAME_LEN,
              "app_main task name must fit the FreeRTOS task name buffer");

} // namespace

extern "C" bool app_bootstrap_create(void)
{
    if (g_app_main_task_handle != nullptr) {
        return true;
    }

    if (uxTaskGetNumberOfTasks() > 0U) {
        g_app_main_task_handle = xTaskGetHandle(kAppMainTaskName);
        if (g_app_main_task_handle != nullptr) {
            return true;
        }
    }

    g_app_main_task_handle = xTaskCreateStatic(
        app_main_task,
        kAppMainTaskName,
        kAppMainStackBytes / sizeof(StackType_t),
        nullptr,
        kAppMainPriority,
        g_app_main_stack,
        &g_app_main_task_buffer);

    return g_app_main_task_handle != nullptr;
}

#include "app_main.h"

#include "ApplicationContext.hpp"
#include "boot_diagnostics.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

extern "C" void app_main_task(void *argument)
{
    (void)argument;

    dima_boot_stage_set(DIMA_BOOT_STAGE_APP_TASK_ENTER);
    auto &application = dima::rover::application_context();
    if (!application.init() || !application.start()) {
        dima_boot_stage_set(DIMA_BOOT_STAGE_APPLICATION_FAILED);
        dima_boot_diagnostics_panic(
            DIMA_BOOT_FAILURE_ERROR_HANDLER,
            DIMA_BOOT_STAGE_APPLICATION_FAILED, 0U);
    } else {
        dima_boot_stage_set(DIMA_BOOT_STAGE_APPLICATION_RUNNING);
    }

    // 装配根具有静态生命周期；appMain 仅保留产品运行状态。
    for (;;) {
        vTaskSuspend(nullptr);
    }
}

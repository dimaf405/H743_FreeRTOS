#include "App/application/app_main.h"

#include "Dima/product/rover/ApplicationContext.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

extern "C" void app_main_task(void *argument)
{
    (void)argument;

    auto &application = dima::product::rover::application_context();
    if (!application.init() || !application.start()) {
        application.stop();
    }

    // 装配根具有静态生命周期；appMain 仅保留产品运行状态。
    for (;;) {
        vTaskSuspend(nullptr);
    }
}

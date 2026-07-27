#include "App/application/app_main.h"

#include "FreeRTOS.h"
#include "task.h"

extern "C" void app_main_task(void *argument)
{
    (void)argument;
}

extern "C" TaskHandle_t xTaskCreateStatic(TaskFunction_t task_function,
                                            const char *task_name,
                                            uint32_t stack_depth,
                                            void *argument,
                                            UBaseType_t priority,
                                            StackType_t *stack_buffer,
                                            StaticTask_t *task_buffer)
{
    (void)task_function;
    (void)task_name;
    (void)stack_depth;
    (void)argument;
    (void)priority;
    (void)stack_buffer;
    return static_cast<TaskHandle_t>(task_buffer);
}

extern "C" UBaseType_t uxTaskGetNumberOfTasks(void)
{
    return 1U;
}

#if (INCLUDE_xTaskGetHandle == 1)
extern "C" TaskHandle_t xTaskGetHandle(const char *task_name)
{
    (void)task_name;
    return nullptr;
}
#endif

int main()
{
    return app_bootstrap_create() ? 0 : 1;
}

#pragma once

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*TaskFunction_t)(void *argument);
typedef void *TaskHandle_t;

TaskHandle_t xTaskCreateStatic(TaskFunction_t task_function,
                               const char *task_name,
                               uint32_t stack_depth,
                               void *argument,
                               UBaseType_t priority,
                               StackType_t *stack_buffer,
                               StaticTask_t *task_buffer);
UBaseType_t uxTaskGetNumberOfTasks(void);

#if (INCLUDE_xTaskGetHandle == 1)
TaskHandle_t xTaskGetHandle(const char *task_name);
#endif

#ifdef __cplusplus
}
#endif

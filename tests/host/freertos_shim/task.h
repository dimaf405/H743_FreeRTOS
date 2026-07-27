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
TaskHandle_t xTaskGetHandle(const char *task_name);
void xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(BaseType_t clear_count_on_exit,
                          TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif

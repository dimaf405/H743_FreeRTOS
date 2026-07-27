#pragma once

#include <stdint.h>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;

typedef struct StaticSemaphore {
    uint32_t kind;
    uint32_t count;
} StaticSemaphore_t;

#define pdFALSE 0
#define pdTRUE 1
#define pdFAIL 0
#define pdPASS 1
#define configTICK_RATE_HZ ((TickType_t)1000U)
#define INCLUDE_vTaskSuspend 1
#define portMAX_DELAY ((TickType_t)UINT32_MAX)
#define portTICK_PERIOD_MS ((TickType_t)1U)
#define pdMS_TO_TICKS(milliseconds) \
    ((TickType_t)(((TickType_t)(milliseconds) * configTICK_RATE_HZ) / 1000U))

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t xPortIsInsideInterrupt(void);
void freertos_usb_test_yield_from_isr(BaseType_t higher_priority_task_woken);

#ifdef __cplusplus
}
#endif

#define portYIELD_FROM_ISR(higher_priority_task_woken) \
    freertos_usb_test_yield_from_isr((higher_priority_task_woken))

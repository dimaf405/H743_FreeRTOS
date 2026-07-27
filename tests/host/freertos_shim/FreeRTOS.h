#pragma once

#include <stdint.h>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;

typedef struct StaticTask {
    uintptr_t opaque[8];
} StaticTask_t;

#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define configTICK_RATE_HZ ((TickType_t)1000U)
#define configMAX_TASK_NAME_LEN 16U
#define tskIDLE_PRIORITY ((UBaseType_t)0U)

#ifdef __cplusplus
extern "C" {
#endif

void freertos_test_enter_critical(void);
void freertos_test_exit_critical(void);
void freertos_test_assert(BaseType_t condition);
BaseType_t xPortIsInsideInterrupt(void);

#ifdef __cplusplus
}
#endif

#define taskENTER_CRITICAL() freertos_test_enter_critical()
#define taskEXIT_CRITICAL() freertos_test_exit_critical()
#define configASSERT(condition) freertos_test_assert((condition) ? pdTRUE : pdFALSE)

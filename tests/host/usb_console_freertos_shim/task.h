#pragma once

#include "FreeRTOS.h"

#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
#define taskSCHEDULER_SUSPENDED 2

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks_to_delay);

#ifdef __cplusplus
}
#endif

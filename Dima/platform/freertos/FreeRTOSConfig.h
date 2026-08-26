#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "api/platform_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((noreturn)) void dima_freertos_assert_failed(
    const char *file, uint32_t line);

#ifdef __cplusplus
}
#endif

#define configENABLE_FPU                         0
#define configENABLE_MPU                         0
#define configUSE_PREEMPTION                     1
#define configUSE_TICKLESS_IDLE                  0
#define configSUPPORT_STATIC_ALLOCATION          1
#define configUSE_MALLOC_FAILED_HOOK             1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configCPU_CLOCK_HZ                       DIMA_CPU_CLOCK_HZ
#define configTICK_RATE_HZ                       ((TickType_t)DIMA_KERNEL_TICK_HZ)
#define configMAX_PRIORITIES                     56
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
/* 平台 Backend 使用 heap_5 在链接脚本的 256 KiB 区域上定义 heap；该宏仍供
 * FreeRTOS/CMSIS 编译期接口兼容，不是产品实际可分配总量。 */
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
#define configMAX_TASK_NAME_LEN                  DIMA_TASK_NAME_CAPACITY
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          2
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                2
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256
#define configUSE_OS2_THREAD_SUSPEND_RESUME      1
#define configUSE_OS2_THREAD_ENUMERATE           0
#define configUSE_OS2_EVENTFLAGS_FROM_ISR        1
#define configUSE_OS2_THREAD_FLAGS               1
#define configUSE_OS2_TIMER                      0
#define configUSE_OS2_MUTEX                      1
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xQueueGetMutexHolder             1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTaskGetHandle                   1

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

/* 数值越小中断优先级越高；仅逻辑优先级不高于 MAX_SYSCALL 边界的 ISR 才能
 * 调用 FreeRTOS FromISR API。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY \
    DIMA_LOWEST_INTERRUPT_PRIORITY
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY \
    DIMA_MAX_SYSCALL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x) do { \
    if ((x) == 0) { \
        dima_freertos_assert_failed(__FILE__, (uint32_t)__LINE__); \
    } \
} while (0)

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 1

#endif /* FREERTOS_CONFIG_H */

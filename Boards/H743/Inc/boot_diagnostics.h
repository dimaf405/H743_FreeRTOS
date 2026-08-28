#ifndef DIMA_BOOT_DIAGNOSTICS_H
#define DIMA_BOOT_DIAGNOSTICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DIMA_BOOT_DIAGNOSTICS_MAGIC          0x44424447UL
#define DIMA_BOOT_DIAGNOSTICS_VERSION_V1     1UL
#define DIMA_BOOT_DIAGNOSTICS_VERSION        2UL
#define DIMA_BOOT_DIAGNOSTICS_CAPTURE_VALID  0x43505452UL
#define DIMA_BOOT_DIAGNOSTICS_ADDRESS        0x38000000UL
#define DIMA_BOOT_DETAIL_APPLICATION_BRIDGE  UINT32_C(0x41505042)

#define DIMA_BOOT_FLASH_RECORD_MAGIC         0x44424652UL
#define DIMA_BOOT_FLASH_RECORD_VERSION_V1    1UL
#define DIMA_BOOT_FLASH_RECORD_VERSION       2UL
#define DIMA_BOOT_FLASH_RECORD_COMMIT        0x434D4954UL
#define DIMA_BOOT_FLASH_RECORD_SIZE          256UL

#define DIMA_BOOT_FAILURE_NONE               0UL
#define DIMA_BOOT_FAILURE_NMI                1UL
#define DIMA_BOOT_FAILURE_HARDFAULT          2UL
#define DIMA_BOOT_FAILURE_MEMMANAGE          3UL
#define DIMA_BOOT_FAILURE_BUSFAULT           4UL
#define DIMA_BOOT_FAILURE_USAGEFAULT         5UL
#define DIMA_BOOT_FAILURE_ERROR_HANDLER      6UL
#define DIMA_BOOT_FAILURE_FREERTOS_ASSERT    7UL
#define DIMA_BOOT_FAILURE_STACK_OVERFLOW     8UL
#define DIMA_BOOT_FAILURE_PLATFORM_CONTRACT  9UL

typedef enum {
    DIMA_BOOT_STAGE_SYSTEM_INIT = 0x0100U,
    DIMA_BOOT_STAGE_MAIN_ENTER = 0x0200U,
    DIMA_BOOT_STAGE_MEMORY_CONTRACT = 0x0208U,
    DIMA_BOOT_STAGE_HAL_INIT = 0x0210U,
    DIMA_BOOT_STAGE_SYSTEM_CLOCK = 0x0220U,
    DIMA_BOOT_STAGE_PERIPHERAL_CLOCK = 0x0230U,

    DIMA_BOOT_STAGE_GPIO = 0x0301U,
    DIMA_BOOT_STAGE_DMA = 0x0302U,
    DIMA_BOOT_STAGE_FDCAN1 = 0x0303U,
    DIMA_BOOT_STAGE_I2C2 = 0x0304U,
    DIMA_BOOT_STAGE_SPI4 = 0x0305U,
    DIMA_BOOT_STAGE_UART4 = 0x0306U,
    /* 0x0307 保留为空洞，避免改变既有 UART7 及后续诊断值。 */
    DIMA_BOOT_STAGE_UART7 = 0x0308U,
    DIMA_BOOT_STAGE_UART8 = 0x0309U,
    DIMA_BOOT_STAGE_USART1 = 0x030AU,
    DIMA_BOOT_STAGE_USART2 = 0x030BU,
    DIMA_BOOT_STAGE_USART3 = 0x030CU,
    DIMA_BOOT_STAGE_USART6 = 0x030DU,
    DIMA_BOOT_STAGE_TIM5 = 0x030EU,
    DIMA_BOOT_STAGE_TIM8 = 0x030FU,
    DIMA_BOOT_STAGE_MOTOR_PWM_SAFE = 0x0310U,
    DIMA_BOOT_STAGE_BOARD_READY = 0x03FFU,

    DIMA_BOOT_STAGE_PLATFORM_EARLY = 0x0400U,
    DIMA_BOOT_STAGE_HEAP_INIT = 0x0410U,
    DIMA_BOOT_STAGE_HRT_INIT = 0x0420U,
    DIMA_BOOT_STAGE_PLATFORM_READY = 0x04FFU,

    DIMA_BOOT_STAGE_KERNEL_INIT = 0x0500U,
    DIMA_BOOT_STAGE_APP_TASK_CREATE = 0x0510U,
    DIMA_BOOT_STAGE_SCHEDULER_START = 0x0520U,

    DIMA_BOOT_STAGE_APP_TASK_ENTER = 0x0600U,
    DIMA_BOOT_STAGE_USB_INIT = 0x0610U,
    DIMA_BOOT_STAGE_USB_READY = 0x061FU,
    DIMA_BOOT_STAGE_WORK_QUEUE_INIT = 0x0620U,
    DIMA_BOOT_STAGE_UORB_INIT = 0x0630U,
    DIMA_BOOT_STAGE_PARAMETER_INIT = 0x0640U,
    DIMA_BOOT_STAGE_MODULE_REGISTER = 0x0650U,
    DIMA_BOOT_STAGE_APPLICATION_INITIALIZED = 0x06FFU,

    DIMA_BOOT_STAGE_BOOT_HEALTH_START = 0x0700U,
    DIMA_BOOT_STAGE_HELLO_START = 0x0710U,
    DIMA_BOOT_STAGE_PARAMETER_START = 0x0720U,
    DIMA_BOOT_STAGE_LOG_START = 0x0730U,
    DIMA_BOOT_STAGE_MOTOR_OUTPUT_START = 0x0738U,
    DIMA_BOOT_STAGE_COMMANDER_START = 0x0740U,
    DIMA_BOOT_STAGE_RC_START = 0x0750U,
    DIMA_BOOT_STAGE_APPLICATION_RUNNING = 0x07FFU,
    DIMA_BOOT_STAGE_APPLICATION_FAILED = 0x0F00U
} dima_boot_stage_t;

#define DIMA_BOOT_DIAGNOSTICS_V1_FIELDS \
    uint32_t magic; \
    uint32_t version; \
    uint32_t size; \
    uint32_t boot_count; \
    uint32_t reset_flags; \
    uint32_t stage; \
    uint32_t detail; \
    uint32_t failure_kind; \
    uint32_t capture_valid; \
    uint32_t previous_stage; \
    uint32_t previous_failure_kind; \
    uint32_t previous_pc; \
    uint32_t previous_cfsr; \
    uint32_t exception_return; \
    uint32_t stacked_r0; \
    uint32_t stacked_r1; \
    uint32_t stacked_r2; \
    uint32_t stacked_r3; \
    uint32_t stacked_r12; \
    uint32_t stacked_lr; \
    uint32_t stacked_pc; \
    uint32_t stacked_xpsr; \
    uint32_t msp; \
    uint32_t psp; \
    uint32_t primask; \
    uint32_t basepri; \
    uint32_t faultmask; \
    uint32_t control; \
    uint32_t cfsr; \
    uint32_t hfsr; \
    uint32_t dfsr; \
    uint32_t afsr; \
    uint32_t mmfar; \
    uint32_t bfar; \
    uint32_t icsr; \
    uint32_t shcsr; \
    uint32_t system_core_clock; \
    uint32_t system_d2_clock; \
    uint32_t rcc_cfgr; \
    uint32_t rcc_d1cfgr; \
    uint32_t rcc_d2cfgr; \
    uint32_t systick_ctrl; \
    uint32_t systick_load; \
    uint32_t systick_value; \
    uint32_t tim2_psc; \
    uint32_t tim2_arr; \
    uint32_t tim2_cnt; \
    uint32_t tim2_sr

typedef struct {
    DIMA_BOOT_DIAGNOSTICS_V1_FIELDS;
} dima_boot_diagnostics_v1_t;

typedef struct {
    DIMA_BOOT_DIAGNOSTICS_V1_FIELDS;
    uint32_t abfsr;
    uint32_t scb_ccr;
    uint32_t mpu_ctrl;
    uint32_t previous_abfsr;
} dima_boot_diagnostics_t;

#undef DIMA_BOOT_DIAGNOSTICS_V1_FIELDS

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t sequence;
    dima_boot_diagnostics_v1_t diagnostics;
    uint32_t crc32;
    uint32_t reserved[10];
    uint32_t commit;
} dima_boot_flash_record_v1_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t sequence;
    dima_boot_diagnostics_t diagnostics;
    uint32_t crc32;
    uint32_t reserved[6];
    uint32_t commit;
} dima_boot_flash_record_t;

extern volatile dima_boot_diagnostics_t dima_boot_diagnostics;

void dima_boot_diagnostics_early_init(void);
void dima_boot_stage_set(uint32_t stage);

__attribute__((noreturn)) void dima_boot_diagnostics_capture_fault(
    uint32_t failure_kind, const uint32_t *stacked_frame,
    uint32_t exception_return);
__attribute__((noreturn)) void dima_boot_diagnostics_panic(
    uint32_t failure_kind, uint32_t detail, uint32_t auxiliary);
__attribute__((noreturn)) void dima_freertos_assert_failed(
    const char *file, uint32_t line);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_BOOT_DIAGNOSTICS_H */

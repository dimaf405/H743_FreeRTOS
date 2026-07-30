#ifndef DIMA_PLATFORM_FREERTOS_FLASH_OPERATION_LOCK_H
#define DIMA_PLATFORM_FREERTOS_FLASH_OPERATION_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 串行化应用内所有 HAL Flash 擦写，禁止在 ISR 中调用。 */
int dima_flash_operation_lock(void);
void dima_flash_operation_unlock(void);

#ifdef __cplusplus
}
#endif

#endif

#ifndef DIMA_PLATFORM_FREERTOS_FLASH_OPERATION_LOCK_H
#define DIMA_PLATFORM_FREERTOS_FLASH_OPERATION_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 非阻塞获取应用 Flash 串行锁；忙、未初始化或 ISR 上下文均返回 0。 */
int dima_flash_operation_lock(void);
void dima_flash_operation_unlock(void);

#ifdef __cplusplus
}
#endif

#endif

#ifndef DIMA_BOOT_REQUEST_H
#define DIMA_BOOT_REQUEST_H

#include <stdint.h>

/* BKP31R is reserved for one-shot MCUboot handoffs.  The backup domain
 * survives NVIC_SystemReset(), but not a backup-domain reset. */
#define DIMA_BOOT_REQUEST_RECOVERY_MAGIC UINT32_C(0xD14AB007)
#define DIMA_BOOT_REQUEST_APPLICATION_MAGIC UINT32_C(0xD14A4A50)
#define DIMA_BOOT_REQUEST_ACCESS_ATTEMPTS UINT32_C(1024)

#ifdef __cplusplus
extern "C" {
#endif

int dima_boot_request_enable_access(void);

uint32_t dima_boot_request_read(void);

int dima_boot_request_write(uint32_t value);

int dima_boot_request_set_recovery(void);

int dima_boot_request_set_application(void);

int dima_boot_request_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_BOOT_REQUEST_H */

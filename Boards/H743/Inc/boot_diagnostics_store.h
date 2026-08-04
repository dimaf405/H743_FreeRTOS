#ifndef H743_BOOT_DIAGNOSTICS_STORE_H
#define H743_BOOT_DIAGNOSTICS_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIMA_BOOT_DIAGNOSTICS_STORE_NONE = 0,
    DIMA_BOOT_DIAGNOSTICS_STORE_OK = 1,
    DIMA_BOOT_DIAGNOSTICS_STORE_EXISTING = 2,
    DIMA_BOOT_DIAGNOSTICS_STORE_NOT_READY = -1,
    DIMA_BOOT_DIAGNOSTICS_STORE_FULL = -2,
    DIMA_BOOT_DIAGNOSTICS_STORE_BUSY = -3,
    DIMA_BOOT_DIAGNOSTICS_STORE_ERROR = -4,
} dima_boot_diagnostics_store_result_t;

void dima_boot_diagnostics_store_enable(void);
int dima_boot_diagnostics_capture_pending(void);
dima_boot_diagnostics_store_result_t
dima_boot_diagnostics_store_pending(int clear_capture);

#ifdef __cplusplus
}
#endif

#endif /* H743_BOOT_DIAGNOSTICS_STORE_H */

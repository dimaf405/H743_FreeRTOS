#ifndef DIMA_MODULES_SAFETY_ARMING_FLASH_INTERLOCK_H
#define DIMA_MODULES_SAFETY_ARMING_FLASH_INTERLOCK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

enum dima_flash_begin_result {
    DIMA_FLASH_BEGIN_DEFERRED = 0,
    DIMA_FLASH_BEGIN_ACQUIRED = 1,
};

/* Atomically enters ARMED unless a Flash write is already active. */
bool dima_arming_flash_try_arm(void);

/* Leaves ARMED without changing an in-progress Flash operation. */
void dima_arming_flash_disarm(void);

/* Atomically starts a Flash write unless ARMED or another write is active. */
int dima_arming_flash_begin(void);

/* Completes the Flash write acquired by dima_arming_flash_begin(). */
void dima_arming_flash_end(void);

bool dima_arming_flash_is_armed(void);
bool dima_arming_flash_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* DIMA_MODULES_SAFETY_ARMING_FLASH_INTERLOCK_H */

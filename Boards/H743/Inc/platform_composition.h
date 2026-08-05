#ifndef H743_PLATFORM_COMPOSITION_H
#define H743_PLATFORM_COMPOSITION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Install the OS and MCU capabilities after board clocks/peripherals exist and
 * before any product task is created.  The implementation is idempotent. */
bool dima_platform_early_init(void);

#ifdef __cplusplus
}
#endif

#endif /* H743_PLATFORM_COMPOSITION_H */

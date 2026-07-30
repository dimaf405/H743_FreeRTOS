#ifndef APP_APPLICATION_APP_MAIN_H
#define APP_APPLICATION_APP_MAIN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool app_bootstrap_create(void);
void app_main_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_APPLICATION_APP_MAIN_H */

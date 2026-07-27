#pragma once

#include <stdint.h>

typedef struct {
    uint8_t dev_state;
    void *pClassData;
} USBD_HandleTypeDef;

#define USBD_STATE_DEFAULT 1U
#define USBD_STATE_CONFIGURED 2U
#define USBD_STATE_SUSPENDED 3U

#ifdef __cplusplus
extern "C" {
#endif

extern USBD_HandleTypeDef hUsbDeviceFS;

#ifdef __cplusplus
}
#endif

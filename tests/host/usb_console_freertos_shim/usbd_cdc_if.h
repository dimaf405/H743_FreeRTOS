#pragma once

#include <stdint.h>

#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

#ifdef __cplusplus
extern "C" {
#endif

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

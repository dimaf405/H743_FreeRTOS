#ifndef APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_INTERNAL_H
#define APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CDC DeInit normally runs in OTG IRQ context, but synchronous USBD_Stop/DeInit
 * also invokes it from the calling task. */
void usb_console_transport_connected(void);
void usb_console_transport_disconnected(void);

/* 仅供CDC OUT回调使用：ISR只复制字节并返回，不解析、不分配。 */
void usb_console_receive_from_isr(const uint8_t *data, size_t length);

#ifdef APP_USB_CONSOLE_FREERTOS_TEST_SEAM
void usb_console_freertos_test_reset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif

#ifndef APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_INTERNAL_H
#define APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* CDC DeInit normally runs in OTG IRQ context, but synchronous USBD_Stop/DeInit
 * also invokes it from the calling task. */
void usb_console_transport_connected(void);
void usb_console_transport_disconnected(void);

#ifdef APP_USB_CONSOLE_FREERTOS_TEST_SEAM
void usb_console_freertos_test_reset(void);
#endif

#ifdef __cplusplus
}
#endif

#endif

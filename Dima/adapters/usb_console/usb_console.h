#ifndef DIMA_ADAPTERS_USB_CONSOLE_USB_CONSOLE_H
#define DIMA_ADAPTERS_USB_CONSOLE_USB_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* usb_console_init's bootstrap setvbuf call is the only stdio exception.
 * After initialization, stdio is a single-task contract:
 * printf/FILE/errno may be used only from wq:lp_default.  usb_console_write
 * serializes the raw USB transport, but does not make newlib FILE or errno
 * state safe across tasks. */
void usb_console_init(void);
void usb_console_service(void);
bool usb_console_ready(void);
int usb_console_write(const uint8_t *data, size_t length, uint32_t timeout_ms);
size_t usb_console_read(uint8_t *data, size_t capacity);
bool usb_console_read_byte(uint8_t *byte);
size_t usb_console_rx_available(void);
uint32_t usb_console_rx_overflow_count(void);
void usb_console_tx_complete_from_isr(void);
int _write(int fd, char *data, int length);

#ifdef __cplusplus
}
#endif

#endif

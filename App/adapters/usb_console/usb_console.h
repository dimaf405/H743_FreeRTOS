#ifndef APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_H
#define APP_ADAPTERS_USB_CONSOLE_USB_CONSOLE_H

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
bool usb_console_ready(void);
int usb_console_write(const uint8_t *data, size_t length, uint32_t timeout_ms);
void usb_console_tx_complete_from_isr(void);
int _write(int fd, char *data, int length);

#ifdef APP_HOST_TEST
typedef enum {
    USB_CONSOLE_TX_ACCEPTED = 0,
    USB_CONSOLE_TX_BUSY,
    USB_CONSOLE_TX_FAILED,
} usb_console_tx_result_t;

typedef struct {
    void *context;
    void *class_data;
    bool (*is_configured)(void *context);
    usb_console_tx_result_t (*transmit)(void *context, const uint8_t *data,
                                        size_t length);
    uint32_t (*now_ms)(void *context);
    void (*poll)(void *context);
    bool (*is_in_isr)(void *context);
    bool (*is_scheduler_running)(void *context);
    int (*set_stdout_unbuffered)(void *context);
} usb_console_test_backend_t;

void usb_console_test_set_backend(const usb_console_test_backend_t *backend);
void usb_console_test_transport_connected(void *class_data);
void usb_console_test_transport_disconnected(void);
#endif

#ifdef __cplusplus
}
#endif

#endif

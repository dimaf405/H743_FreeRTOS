#ifndef H743_BOOT_USB_H
#define H743_BOOT_USB_H

#include <stdint.h>

int boot_usb_init(void);
void boot_usb_deinit(void);
int boot_usb_console_read(char *str, int count, int *newline);
void boot_usb_console_write(const char *data, int count);

#endif /* H743_BOOT_USB_H */

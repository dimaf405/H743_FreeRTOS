#ifndef H743_BOOT_USBD_DESC_H
#define H743_BOOT_USBD_DESC_H

#include "usbd_def.h"

#define DEVICE_ID1  (UID_BASE)
#define DEVICE_ID2  (UID_BASE + 0x4UL)
#define DEVICE_ID3  (UID_BASE + 0x8UL)
#define USB_SIZ_STRING_SERIAL 0x1AU

extern USBD_DescriptorsTypeDef Boot_FS_Desc;

#endif /* H743_BOOT_USBD_DESC_H */

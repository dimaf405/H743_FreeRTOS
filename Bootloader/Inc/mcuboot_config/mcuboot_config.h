#ifndef H743_MCUBOOT_CONFIG_H
#define H743_MCUBOOT_CONFIG_H

#include <stdint.h>

#include "boot_watchdog.h"

#define MCUBOOT_IMAGE_NUMBER                    1
#define MCUBOOT_MAX_IMG_SECTORS                 6
#define MCUBOOT_BOOT_MAX_ALIGN                  32
#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS      1

#define MCUBOOT_SWAP_USING_SCRATCH              1
#define MCUBOOT_VALIDATE_PRIMARY_SLOT           1
#define MCUBOOT_DEV_WITH_ERASE                  1
#define MCUBOOT_USE_TLV_ALLOW_LIST              1

#define MCUBOOT_SIGN_EC256                      1
#define MCUBOOT_USE_TINYCRYPT                   1

#define MCUBOOT_SERIAL                          1
#define MCUBOOT_BOOT_MGMT_ECHO                  1
#define MCUBOOT_SERIAL_DIRECT_IMAGE_UPLOAD      1
#define MCUBOOT_SERIAL_IMG_GRP_IMAGE_STATE      1
#define MCUBOOT_SERIAL_IMG_GRP_HASH             1
#define MCUBOOT_SERIAL_IMG_GRP_SLOT_INFO        1
#define MCUBOOT_ERASE_PROGRESSIVELY             1
#define MCUBOOT_SERIAL_WAIT_FOR_DFU             1
#define MCUBOOT_SERIAL_MAX_RECEIVE_SIZE         512
#define MCUBOOT_SERIAL_UNALIGNED_BUFFER_SIZE    512
#define MCUBOOT_PERUSER_MGMT_GROUP_ENABLED      0

#define MCUBOOT_WATCHDOG_FEED()                 boot_watchdog_feed()
#define MCUBOOT_CPU_IDLE()                      do { } while (0)

#endif /* H743_MCUBOOT_CONFIG_H */

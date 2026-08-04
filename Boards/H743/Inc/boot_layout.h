#ifndef H743_BOOT_LAYOUT_H
#define H743_BOOT_LAYOUT_H

#include <stdint.h>

/* STM32H743VI: 2 MiB dual-bank flash, 128 KiB erase sectors. */
#define H743_FLASH_BASE             0x08000000UL
#define H743_FLASH_BANK2_BASE       0x08100000UL
#define H743_FLASH_END              0x08200000UL
#define H743_FLASH_SECTOR_SIZE      0x00020000UL
#define H743_FLASH_WRITE_SIZE       32UL

#define H743_MCUBOOT_BASE           0x08000000UL
#define H743_MCUBOOT_SIZE           0x00020000UL
#define H743_BOOT_DIAGNOSTICS_BASE  0x08020000UL
#define H743_BOOT_DIAGNOSTICS_SIZE  0x00020000UL

#define H743_PRIMARY_SLOT_BASE      0x08040000UL
#define H743_PRIMARY_SLOT_SIZE      0x000C0000UL
#define H743_SECONDARY_SLOT_BASE    0x08100000UL
#define H743_SECONDARY_SLOT_SIZE    0x000C0000UL
#define H743_SCRATCH_BASE           0x081C0000UL
#define H743_SCRATCH_SIZE           0x00020000UL
#define H743_STORAGE_BASE           0x081E0000UL
#define H743_STORAGE_SIZE           0x00020000UL

/* A 1 KiB header keeps the H743's 0x298-byte vector table 1 KiB aligned. */
#define H743_MCUBOOT_HEADER_SIZE    0x00000400UL
#define H743_APP_VECTOR_BASE        (H743_PRIMARY_SLOT_BASE + H743_MCUBOOT_HEADER_SIZE)

#if (H743_MCUBOOT_BASE + H743_MCUBOOT_SIZE) != H743_BOOT_DIAGNOSTICS_BASE
#error "MCUboot and boot diagnostics layout is not contiguous"
#endif
#if (H743_BOOT_DIAGNOSTICS_BASE + H743_BOOT_DIAGNOSTICS_SIZE) != H743_PRIMARY_SLOT_BASE
#error "Boot diagnostics and primary slot layout is not contiguous"
#endif
#if (H743_PRIMARY_SLOT_BASE + H743_PRIMARY_SLOT_SIZE) != H743_FLASH_BANK2_BASE
#error "Primary slot must end at the Bank 1 boundary"
#endif
#if (H743_SECONDARY_SLOT_BASE + H743_SECONDARY_SLOT_SIZE) != H743_SCRATCH_BASE
#error "Secondary slot and scratch layout is not contiguous"
#endif
#if (H743_STORAGE_BASE + H743_STORAGE_SIZE) != H743_FLASH_END
#error "Reserved storage must end at the physical flash boundary"
#endif

#endif /* H743_BOOT_LAYOUT_H */

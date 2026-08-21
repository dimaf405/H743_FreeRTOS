/*----------------------------------------------------------------------------/
/  FatFs - Generic FAT file system module  R0.12c                             /
/-----------------------------------------------------------------------------/
/
/ Copyright (C) 2017, ChaN, all right reserved.
/ Portions Copyright (C) STMicroelectronics, all right reserved.
/
/ FatFs module is an open source software. Redistribution and use of FatFs in
/ source and binary forms, with or without modification, are permitted provided
/ that the following condition is met:

/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.

/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------/
/  FatFs - FAT file system module configuration file  (Dima H743)
/---------------------------------------------------------------------------*/

#define _FFCONF 68300	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0
/* 0:Read/Write, 1:Read-only */

#define _FS_MINIMIZE	0
/* 0: All basic functions enabled */

#define	_USE_STRFUNC	0
/* 0: Disable string functions */

#define _USE_FIND		0
/* 0: Disable filtered directory read */

#define	_USE_MKFS		0
/* Parameter storage never formats the card. */

#define	_USE_FASTSEEK	0
/* Parameter files are read and written sequentially. */

#define	_USE_EXPAND		0
/* 0: Disable f_expand function */

#define _USE_CHMOD		0
/* 0: Disable attribute manipulation */

#define _USE_LABEL		0
/* 0: Disable volume label functions */

#define	_USE_FORWARD	0
/* 0: Disable f_forward() */

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	1
/* ASCII-only 8.3 paths; no OEM/Unicode conversion tables are needed. */

#define	_USE_LFN	0
#define	_MAX_LFN	255
/* All parameter storage paths fit the 8.3 format. */

#define _LFN_UNICODE	0
/* ANSI/OEM string encoding */

#define _STRF_ENCODE	3
/* UTF-8 (no effect when _LFN_UNICODE == 0) */

#define _FS_RPATH	0
/* 0: Disable relative path */

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define _VOLUMES	1
/* Single volume: SD card on SDMMC1 */

#define _STR_VOLUME_ID	0
#define	_MULTI_PARTITION	0

#define	_MIN_SS		512
#define	_MAX_SS		512
/* Fixed 512-byte sectors (standard for SD cards) */

#define	_USE_TRIM	0
/* 0: Disable ATA-TRIM */

#define _FS_NOFSINFO	0
/* Trust FSINFO free cluster count */

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define	_FS_TINY	0
/* 0: Normal mode (FIL has private 512-byte buffer) */

#define _FS_EXFAT	0
/* 0: Disable exFAT */

#define _FS_NORTC	1
#define _NORTC_MON	1
#define _NORTC_MDAY	1
#define _NORTC_YEAR	2026
/* No RTC available yet; fixed timestamp for file operations */

#define	_FS_LOCK	0
/* FileStorage serializes access and owns the only FIL object. */

#define _FS_REENTRANT	0
/* FileStorage serializes the only FatFs client with its backend mutex. */

/*--- End of configuration options ---*/

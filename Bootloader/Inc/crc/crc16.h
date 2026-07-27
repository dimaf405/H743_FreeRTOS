#ifndef H743_MCUBOOT_CRC16_H
#define H743_MCUBOOT_CRC16_H
#include <stdint.h>

/* Seed required by MCUboot's serial recovery framing protocol. */
#define CRC16_INITIAL_CRC 0U

uint16_t crc16_ccitt(uint16_t initial_crc, const void *data, int len);
#endif

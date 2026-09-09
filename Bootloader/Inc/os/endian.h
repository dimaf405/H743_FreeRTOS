#ifndef H743_MCUBOOT_ENDIAN_H
#define H743_MCUBOOT_ENDIAN_H
#include <stdint.h>
uint16_t h743_bswap16(uint16_t value);
#define htons(value) h743_bswap16((uint16_t)(value))
#define ntohs(value) h743_bswap16((uint16_t)(value))
#endif

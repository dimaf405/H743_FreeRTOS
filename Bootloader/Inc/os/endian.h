#ifndef H743_MCUBOOT_ENDIAN_H
#define H743_MCUBOOT_ENDIAN_H
#include <stdint.h>
static inline uint16_t h743_bswap16(uint16_t value)
{
    return (uint16_t)((value << 8) | (value >> 8));
}
#define htons(value) h743_bswap16((uint16_t)(value))
#define ntohs(value) h743_bswap16((uint16_t)(value))
#endif

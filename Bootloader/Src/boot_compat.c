#include "stm32h7xx_hal.h"

#include <stddef.h>
#include <stdint.h>

#include "base64/base64.h"
#include "crc/crc16.h"

/* MCUboot only verifies ECDSA signatures on this target. TinyCrypt retains a
 * link-time RNG hook for its optional signing/ECDH paths; those paths are not
 * enabled, so fail closed if they are ever called accidentally. */
int default_CSPRNG(uint8_t *destination, unsigned int size)
{
    (void)destination;
    (void)size;
    return 0;
}

uint32_t k_uptime_get_32(void)
{
    return HAL_GetTick();
}

void os_cputime_delay_usecs(uint32_t usecs)
{
    HAL_Delay((usecs + 999U) / 1000U);
}

void hal_system_reset(void)
{
    NVIC_SystemReset();
    for (;;) {
    }
}

uint16_t crc16_ccitt(uint16_t crc, const void *data, int len)
{
    const uint8_t *bytes = (const uint8_t *)data;

    while (len-- > 0) {
        crc ^= (uint16_t)(*bytes++) << 8;
        for (unsigned int bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static const char base64_alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_value(unsigned char value)
{
    if (value >= 'A' && value <= 'Z') {
        return value - 'A';
    }
    if (value >= 'a' && value <= 'z') {
        return value - 'a' + 26;
    }
    if (value >= '0' && value <= '9') {
        return value - '0' + 52;
    }
    if (value == '+') {
        return 62;
    }
    if (value == '/') {
        return 63;
    }
    return -1;
}

int base64_encode(const void *src, int len, char *dst, int pad)
{
    const uint8_t *input = (const uint8_t *)src;
    int output_len = 0;

    for (int offset = 0; offset < len; offset += 3) {
        const int remaining = len - offset;
        const uint32_t value = ((uint32_t)input[offset] << 16) |
                               ((remaining > 1 ? input[offset + 1] : 0U) << 8) |
                               (remaining > 2 ? input[offset + 2] : 0U);

        dst[output_len++] = base64_alphabet[(value >> 18) & 0x3FU];
        dst[output_len++] = base64_alphabet[(value >> 12) & 0x3FU];
        if (remaining > 1) {
            dst[output_len++] = base64_alphabet[(value >> 6) & 0x3FU];
        } else if (pad) {
            dst[output_len++] = '=';
        }
        if (remaining > 2) {
            dst[output_len++] = base64_alphabet[value & 0x3FU];
        } else if (pad) {
            dst[output_len++] = '=';
        }
    }

    dst[output_len] = '\0';
    return output_len;
}

int base64_decode_len(const char *src)
{
    int valid = 0;
    int padding = 0;

    while (*src != '\0' && *src != '\r' && *src != '\n') {
        if (*src == '=') {
            ++padding;
            ++valid;
        } else if (base64_value((unsigned char)*src) >= 0) {
            ++valid;
        }
        ++src;
    }

    return (valid / 4) * 3 - padding + 3;
}

int base64_decode(const char *src, void *dst)
{
    uint8_t *output = (uint8_t *)dst;
    uint32_t accumulator = 0;
    int bits = 0;
    int output_len = 0;

    while (*src != '\0' && *src != '\r' && *src != '\n') {
        if (*src == '=') {
            break;
        }

        const int value = base64_value((unsigned char)*src++);
        if (value < 0) {
            continue;
        }

        accumulator = (accumulator << 6) | (uint32_t)value;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            output[output_len++] = (uint8_t)(accumulator >> bits);
            accumulator &= (bits == 0) ? 0U : ((1UL << bits) - 1UL);
        }
    }

    return output_len;
}

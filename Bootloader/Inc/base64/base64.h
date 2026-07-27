#ifndef H743_MCUBOOT_BASE64_H
#define H743_MCUBOOT_BASE64_H
#include <stddef.h>

/* MCUboot's Mynewt-compatible serial transport expects this helper here. */
#define BASE64_ENCODE_SIZE(in_size) \
    ((((((in_size) - 1U) / 3U) * 4U) + 4U) + 1U)

int base64_encode(const void *src, int len, char *dst, int pad);
int base64_decode_len(const char *src);
int base64_decode(const char *src, void *dst);
#endif

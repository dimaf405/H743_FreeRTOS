#ifndef H743_BOOT_PRIMARY_H
#define H743_BOOT_PRIMARY_H

#include <stdint.h>

/* Validate Primary without processing swap state or changing any trailer. */
int boot_primary_image_validate(uint32_t *vector_address);

#endif /* H743_BOOT_PRIMARY_H */

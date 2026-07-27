#ifndef H743_BOOT_PLATFORM_H
#define H743_BOOT_PLATFORM_H

#include <stdint.h>

void boot_jump_to_application(uint32_t vector_address);
int boot_vector_is_valid(uint32_t vector_address);

#endif /* H743_BOOT_PLATFORM_H */

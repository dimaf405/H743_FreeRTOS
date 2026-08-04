#include "boot_primary.h"

#include "boot_layout.h"
#include "boot_platform.h"
#include "bootutil/bootutil.h"
#include "bootutil_loader.h"
#include "bootutil_priv.h"

int boot_primary_image_validate(uint32_t *vector_address)
{
    static struct boot_sector_buffer sectors;
    struct boot_loader_state *state = boot_get_loader_state();
    FIH_DECLARE(image_result, FIH_FAILURE);
    int areas_open = 0;
    int valid = 0;

    if (vector_address == NULL) {
        return 0;
    }
    *vector_address = 0U;

    boot_state_init(state);
    if (boot_open_all_flash_areas(state) != 0) {
        goto out;
    }
    areas_open = 1;

    if (boot_read_sectors(state, &sectors) != 0 ||
        boot_read_image_header(state, BOOT_SLOT_PRIMARY,
                               boot_img_hdr(state, BOOT_SLOT_PRIMARY),
                               NULL) != 0 ||
        !boot_check_header_valid(state, BOOT_SLOT_PRIMARY) ||
        (boot_img_hdr(state, BOOT_SLOT_PRIMARY)->ih_flags &
         IMAGE_F_NON_BOOTABLE) != 0U) {
        goto out;
    }

    FIH_CALL(boot_check_image, image_result, state, NULL,
             BOOT_SLOT_PRIMARY);
    if (FIH_NOT_EQ(image_result, FIH_SUCCESS)) {
        goto out;
    }

    const uint32_t candidate =
        boot_img_slot_off(state, BOOT_SLOT_PRIMARY) +
        boot_img_hdr(state, BOOT_SLOT_PRIMARY)->ih_hdr_size;
    if (candidate != H743_APP_VECTOR_BASE ||
        !boot_vector_is_valid(candidate)) {
        goto out;
    }

    *vector_address = candidate;
    valid = 1;

out:
    if (areas_open != 0) {
        boot_close_all_flash_areas(state);
    }
    boot_state_clear(state);
    return valid;
}

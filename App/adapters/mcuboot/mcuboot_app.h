#ifndef H743_MCUBOOT_APP_H
#define H743_MCUBOOT_APP_H

#ifdef __cplusplus
extern "C" {
#endif

enum mcuboot_confirm_result {
    MCUBOOT_CONFIRM_OK = 0,
    MCUBOOT_CONFIRM_ALREADY_CONFIRMED = 1,
    MCUBOOT_CONFIRM_NOT_A_TEST_IMAGE = 2,
    MCUBOOT_CONFIRM_FLASH_ERROR = -1,
};

int mcuboot_confirm_running_image(void);

#ifdef __cplusplus
}
#endif

#endif /* H743_MCUBOOT_APP_H */

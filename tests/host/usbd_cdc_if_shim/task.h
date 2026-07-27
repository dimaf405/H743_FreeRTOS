#pragma once

void usbd_cdc_if_test_enter_critical(void);
void usbd_cdc_if_test_exit_critical(void);

#define taskENTER_CRITICAL() usbd_cdc_if_test_enter_critical()
#define taskEXIT_CRITICAL() usbd_cdc_if_test_exit_critical()

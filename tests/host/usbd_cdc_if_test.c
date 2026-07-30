#include "usbd_cdc_if.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

USBD_HandleTypeDef hUsbDeviceFS;

static USBD_CDC_HandleTypeDef g_cdc;
static uint8_t g_payload[8];
static uint8_t g_set_tx_result;
static uint8_t g_set_rx_result;
static uint8_t g_transmit_result;
static unsigned g_set_tx_calls;
static unsigned g_set_rx_calls;
static unsigned g_receive_calls;
static unsigned g_transmit_calls;
static unsigned g_enter_calls;
static unsigned g_exit_calls;
static unsigned g_critical_depth;
static unsigned g_connected_calls;
static unsigned g_disconnected_calls;
static unsigned g_completion_calls;
static uint8_t *g_last_tx_buffer;
static uint32_t g_last_tx_length;
static void (*g_enter_hook)(void);
static unsigned g_failures;

#define CHECK(expression)                                                        \
    do {                                                                         \
        if (!(expression)) {                                                     \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,           \
                    #expression);                                                \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

static void reset_fixture(void)
{
    memset(&hUsbDeviceFS, 0, sizeof(hUsbDeviceFS));
    memset(&g_cdc, 0, sizeof(g_cdc));
    g_set_tx_result = USBD_OK;
    g_set_rx_result = USBD_OK;
    g_transmit_result = USBD_OK;
    g_set_tx_calls = 0U;
    g_set_rx_calls = 0U;
    g_receive_calls = 0U;
    g_transmit_calls = 0U;
    g_enter_calls = 0U;
    g_exit_calls = 0U;
    g_critical_depth = 0U;
    g_connected_calls = 0U;
    g_disconnected_calls = 0U;
    g_completion_calls = 0U;
    g_last_tx_buffer = NULL;
    g_last_tx_length = 0U;
    g_enter_hook = NULL;
}

static void configure_ready(void)
{
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.classId = 0U;
    hUsbDeviceFS.pClassData = &g_cdc;
    hUsbDeviceFS.pClassDataCmsit[0] = &g_cdc;
}

static void check_balanced_critical(unsigned expected_entries)
{
    CHECK(g_enter_calls == expected_entries);
    CHECK(g_exit_calls == expected_entries);
    CHECK(g_critical_depth == 0U);
}

void usbd_cdc_if_test_enter_critical(void)
{
    ++g_enter_calls;
    ++g_critical_depth;
    if (g_enter_hook != NULL) {
        void (*hook)(void) = g_enter_hook;
        g_enter_hook = NULL;
        hook();
    }
}

void usbd_cdc_if_test_exit_critical(void)
{
    CHECK(g_critical_depth == 1U);
    ++g_exit_calls;
    --g_critical_depth;
}

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer,
                             uint32_t length)
{
    CHECK(device == &hUsbDeviceFS);
    ++g_set_tx_calls;
    g_last_tx_buffer = buffer;
    g_last_tx_length = length;
    return g_set_tx_result;
}

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *buffer)
{
    CHECK(device == &hUsbDeviceFS);
    CHECK(buffer != NULL);
    ++g_set_rx_calls;
    return g_set_rx_result;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{
    CHECK(device == &hUsbDeviceFS);
    ++g_receive_calls;
    return USBD_OK;
}

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    CHECK(device == &hUsbDeviceFS);
    ++g_transmit_calls;
    return g_transmit_result;
}

void usb_console_receive_from_isr(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
}

void usb_console_transport_connected(void)
{
    ++g_connected_calls;
}

void usb_console_transport_disconnected(void)
{
    ++g_disconnected_calls;
}

void usb_console_tx_complete_from_isr(void)
{
    ++g_completion_calls;
}

static void test_rejects_invalid_state_before_critical_section(void)
{
    reset_fixture();
    configure_ready();
    CHECK(CDC_Transmit_FS(NULL, 1U) == USBD_FAIL);
    CHECK(CDC_Transmit_FS(g_payload, 0U) == USBD_FAIL);

    hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
    CHECK(CDC_Transmit_FS(g_payload, 1U) == USBD_FAIL);
    configure_ready();
    hUsbDeviceFS.pClassData = NULL;
    CHECK(CDC_Transmit_FS(g_payload, 1U) == USBD_FAIL);
    configure_ready();
    hUsbDeviceFS.pClassDataCmsit[0] = NULL;
    CHECK(CDC_Transmit_FS(g_payload, 1U) == USBD_FAIL);

    check_balanced_critical(0U);
    CHECK(g_set_tx_calls == 0U);
    CHECK(g_transmit_calls == 0U);
}

static void test_reports_busy_with_balanced_critical_section(void)
{
    reset_fixture();
    configure_ready();
    g_cdc.TxState = 1U;

    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_BUSY);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 0U);
    CHECK(g_transmit_calls == 0U);
}

static void invalidate_device_state(void)
{
    hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
}

static void invalidate_composite_class_data(void)
{
    hUsbDeviceFS.pClassDataCmsit[0] = NULL;
}

static void test_rechecks_transport_state_inside_critical_section(void)
{
    reset_fixture();
    configure_ready();
    g_enter_hook = &invalidate_device_state;
    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_FAIL);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 0U);
    CHECK(g_transmit_calls == 0U);

    reset_fixture();
    configure_ready();
    g_enter_hook = &invalidate_composite_class_data;
    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_FAIL);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 0U);
    CHECK(g_transmit_calls == 0U);
}

static void test_propagates_set_buffer_and_packet_failures(void)
{
    reset_fixture();
    configure_ready();
    g_set_tx_result = USBD_FAIL;
    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_FAIL);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 1U);
    CHECK(g_transmit_calls == 0U);

    reset_fixture();
    configure_ready();
    g_transmit_result = USBD_FAIL;
    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_FAIL);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 1U);
    CHECK(g_transmit_calls == 1U);

    reset_fixture();
    configure_ready();
    g_transmit_result = USBD_BUSY;
    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_BUSY);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 1U);
    CHECK(g_transmit_calls == 1U);
}

static void test_success_forwards_exact_buffer_and_length(void)
{
    reset_fixture();
    configure_ready();

    CHECK(CDC_Transmit_FS(g_payload, sizeof(g_payload)) == USBD_OK);
    check_balanced_critical(1U);
    CHECK(g_set_tx_calls == 1U);
    CHECK(g_transmit_calls == 1U);
    CHECK(g_last_tx_buffer == g_payload);
    CHECK(g_last_tx_length == sizeof(g_payload));
}

static void test_lifecycle_callbacks_only_publish_successful_init(void)
{
    reset_fixture();
    g_set_tx_result = USBD_FAIL;
    CHECK(USBD_Interface_fops_FS.Init() == (int8_t)USBD_FAIL);
    CHECK(g_connected_calls == 0U);

    reset_fixture();
    g_set_rx_result = USBD_FAIL;
    CHECK(USBD_Interface_fops_FS.Init() == (int8_t)USBD_FAIL);
    CHECK(g_connected_calls == 0U);

    reset_fixture();
    CHECK(USBD_Interface_fops_FS.Init() == (int8_t)USBD_OK);
    CHECK(g_connected_calls == 1U);
    CHECK(g_set_tx_calls == 1U);
    CHECK(g_set_rx_calls == 1U);
    CHECK(g_last_tx_length == 0U);

    CHECK(USBD_Interface_fops_FS.DeInit() == (int8_t)USBD_OK);
    CHECK(g_disconnected_calls == 1U);
    CHECK(USBD_Interface_fops_FS.TransmitCplt(g_payload, &g_last_tx_length, 1U)
          == (int8_t)USBD_OK);
    CHECK(g_completion_calls == 1U);
}

int main(void)
{
    test_rejects_invalid_state_before_critical_section();
    test_reports_busy_with_balanced_critical_section();
    test_rechecks_transport_state_inside_critical_section();
    test_propagates_set_buffer_and_packet_failures();
    test_success_forwards_exact_buffer_and_length();
    test_lifecycle_callbacks_only_publish_successful_init();

    if (g_failures != 0U) {
        fprintf(stderr, "USB CDC glue host test: %u failure(s)\n", g_failures);
        return 1;
    }
    puts("USB CDC glue host test passed");
    return 0;
}

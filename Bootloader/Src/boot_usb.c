#include "boot_usb.h"

#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "boot_watchdog.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"

#define BOOT_USB_RX_RING_SIZE 2048U
#define BOOT_USB_PACKET_SIZE  64U
#define BOOT_USB_TX_TIMEOUT_MS 1000U

USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t usb_rx_packet[BOOT_USB_PACKET_SIZE];
static uint8_t usb_tx_packet[BOOT_USB_PACKET_SIZE];
static uint8_t usb_rx_ring[BOOT_USB_RX_RING_SIZE];
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static volatile uint8_t usb_rx_overflow;

static int8_t cdc_init(void);
static int8_t cdc_deinit(void);
static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length);
static int8_t cdc_receive(uint8_t *buffer, uint32_t *length);
static int8_t cdc_transmit_complete(uint8_t *buffer, uint32_t *length,
                                    uint8_t endpoint);

static USBD_CDC_ItfTypeDef boot_cdc_interface = {
    cdc_init,
    cdc_deinit,
    cdc_control,
    cdc_receive,
    cdc_transmit_complete,
};

static int8_t cdc_init(void)
{
    usb_rx_head = 0U;
    usb_rx_tail = 0U;
    usb_rx_overflow = 0U;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_packet, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_packet);
    return (int8_t)USBD_OK;
}

static int8_t cdc_deinit(void)
{
    return (int8_t)USBD_OK;
}

static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    static uint8_t line_coding[7] = {
        0x00U, 0xC2U, 0x01U, 0x00U, /* 115200 baud */
        0x00U,                       /* one stop bit */
        0x00U,                       /* no parity */
        0x08U,                       /* eight data bits */
    };

    if (command == CDC_SET_LINE_CODING && length >= sizeof(line_coding)) {
        memcpy(line_coding, buffer, sizeof(line_coding));
    } else if (command == CDC_GET_LINE_CODING && length >= sizeof(line_coding)) {
        memcpy(buffer, line_coding, sizeof(line_coding));
    }

    return (int8_t)USBD_OK;
}

static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
    for (uint32_t index = 0; index < *length; ++index) {
        const uint16_t next = (uint16_t)((usb_rx_head + 1U) &
                                         (BOOT_USB_RX_RING_SIZE - 1U));
        if (next == usb_rx_tail) {
            usb_rx_overflow = 1U;
            break;
        }
        usb_rx_ring[usb_rx_head] = buffer[index];
        usb_rx_head = next;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_packet);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t cdc_transmit_complete(uint8_t *buffer, uint32_t *length,
                                    uint8_t endpoint)
{
    (void)buffer;
    (void)length;
    (void)endpoint;
    return (int8_t)USBD_OK;
}

int boot_usb_init(void)
{
    HAL_PWREx_EnableUSBVoltageDetector();

    if (USBD_Init(&hUsbDeviceFS, &Boot_FS_Desc, DEVICE_FS) != USBD_OK ||
        USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK ||
        USBD_CDC_RegisterInterface(&hUsbDeviceFS, &boot_cdc_interface) != USBD_OK ||
        USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        return -1;
    }

    return 0;
}

void boot_usb_deinit(void)
{
    (void)USBD_Stop(&hUsbDeviceFS);
    (void)USBD_DeInit(&hUsbDeviceFS);
}

int boot_usb_console_read(char *str, int count, int *newline)
{
    int copied = 0;

    boot_watchdog_feed();

    if (str == NULL || count <= 0 || newline == NULL) {
        return -1;
    }

    *newline = 0;
    while (copied < count && usb_rx_tail != usb_rx_head) {
        const uint8_t value = usb_rx_ring[usb_rx_tail];
        usb_rx_tail = (uint16_t)((usb_rx_tail + 1U) &
                                 (BOOT_USB_RX_RING_SIZE - 1U));
        str[copied++] = (char)value;
        if (value == '\n') {
            *newline = 1;
            break;
        }
    }

    if (usb_rx_overflow != 0U && usb_rx_tail == usb_rx_head) {
        usb_rx_overflow = 0U;
    }

    return copied;
}

static int wait_until_ready_to_transmit(void)
{
    const uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < BOOT_USB_TX_TIMEOUT_MS) {
        boot_watchdog_feed();
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
            continue;
        }

        USBD_CDC_HandleTypeDef *cdc =
            (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (cdc != NULL && cdc->TxState == 0U) {
            return 0;
        }
    }

    return -1;
}

void boot_usb_console_write(const char *data, int count)
{
    while (count > 0) {
        boot_watchdog_feed();
        const uint16_t chunk = count > (int)BOOT_USB_PACKET_SIZE
                                   ? (uint16_t)BOOT_USB_PACKET_SIZE
                                   : (uint16_t)count;
        if (wait_until_ready_to_transmit() != 0) {
            return;
        }

        memcpy(usb_tx_packet, data, chunk);
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_packet, chunk);
        if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) {
            return;
        }
        if (wait_until_ready_to_transmit() != 0) {
            return;
        }

        data += chunk;
        count -= chunk;
    }
}

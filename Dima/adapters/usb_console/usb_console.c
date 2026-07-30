#include "usb_console.h"
#include "usb_console_internal.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

static StaticSemaphore_t usb_console_mutex_storage;
static StaticSemaphore_t usb_console_completion_storage;
static SemaphoreHandle_t usb_console_mutex;
static SemaphoreHandle_t usb_console_completion;

_Static_assert(configTICK_RATE_HZ == 1000U,
               "usb console deadline conversion requires the project 1 kHz tick");

static TickType_t usb_console_ms_to_ticks(uint32_t milliseconds)
{
    const uint64_t ticks =
        (((uint64_t)milliseconds * (uint64_t)configTICK_RATE_HZ) + 999U)
        / 1000U;
    const uint64_t max_finite_ticks = (uint64_t)portMAX_DELAY - 1U;
    return (TickType_t)(ticks > max_finite_ticks ? max_finite_ticks : ticks);
}

typedef enum {
    USB_CONSOLE_LOW_ACCEPTED = 0,
    USB_CONSOLE_LOW_BUSY,
    USB_CONSOLE_LOW_FAILED,
} usb_console_low_result_t;

enum { USB_CONSOLE_RX_CAPACITY = 1024U };
_Static_assert((USB_CONSOLE_RX_CAPACITY & (USB_CONSOLE_RX_CAPACITY - 1U)) == 0U,
               "USB RX ring capacity must be a power of two");

static uint8_t usb_console_rx_ring[USB_CONSOLE_RX_CAPACITY];
static uint32_t usb_console_rx_head;
static uint32_t usb_console_rx_tail;
static uint32_t usb_console_rx_overflows;

static uint8_t usb_console_tx_staging[256U];
static bool usb_console_initialized;
static bool usb_console_stdout_unbuffered;
static bool usb_console_in_flight;
static uint32_t usb_console_in_flight_epoch;
static uint32_t usb_console_transport_epoch;
static bool usb_console_transport_online;

static uint32_t usb_console_epoch_load(void)
{
    return __atomic_load_n(&usb_console_transport_epoch, __ATOMIC_ACQUIRE);
}

static void usb_console_epoch_advance(void)
{
    (void)__atomic_add_fetch(&usb_console_transport_epoch, 1U, __ATOMIC_ACQ_REL);
}

static bool usb_console_transport_is_online(void)
{
    return __atomic_load_n(&usb_console_transport_online, __ATOMIC_ACQUIRE);
}

static void usb_console_transport_set_online(bool online)
{
    __atomic_store_n(&usb_console_transport_online, online, __ATOMIC_RELEASE);
}

static uint32_t usb_console_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t usb_console_remaining_ms(uint32_t start_ms, uint32_t timeout_ms)
{
    const uint32_t elapsed_ms = usb_console_now_ms() - start_ms;
    return elapsed_ms >= timeout_ms ? 0U : timeout_ms - elapsed_ms;
}

static bool usb_console_take_mutex(uint32_t wait_ms)
{
    return xSemaphoreTake(usb_console_mutex,
                          usb_console_ms_to_ticks(wait_ms)) == pdTRUE;
}

static void usb_console_give_mutex(void)
{
    (void)xSemaphoreGive(usb_console_mutex);
}

static bool usb_console_take_completion(uint32_t wait_ms)
{
    return xSemaphoreTake(usb_console_completion,
                          usb_console_ms_to_ticks(wait_ms)) == pdTRUE;
}

static void usb_console_drain_completion(void)
{
    while (usb_console_take_completion(0U)) {
    }
}

static void usb_console_signal_completion(void)
{
    if (!usb_console_initialized) {
        return;
    }
    if (xPortIsInsideInterrupt() != pdFALSE) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(usb_console_completion,
                                    &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    } else {
        (void)xSemaphoreGive(usb_console_completion);
    }
}

static bool usb_console_is_in_isr(void)
{
    return xPortIsInsideInterrupt() != pdFALSE;
}

static bool usb_console_scheduler_is_running(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

static usb_console_low_result_t usb_console_transmit(const uint8_t *data,
                                                     size_t length)
{
    const uint8_t status = CDC_Transmit_FS((uint8_t *)data, (uint16_t)length);
    if (status == USBD_OK) {
        return USB_CONSOLE_LOW_ACCEPTED;
    }
    if (status == USBD_BUSY) {
        return USB_CONSOLE_LOW_BUSY;
    }
    return USB_CONSOLE_LOW_FAILED;
}

static bool usb_console_yield_one_tick(uint32_t remaining_ms)
{
    if (remaining_ms == 0U) {
        return false;
    }
    vTaskDelay(usb_console_ms_to_ticks(1U));
    return true;
}

static int usb_console_fail(int error_number)
{
    errno = error_number;
    return -1;
}

static int usb_console_set_stdout_unbuffered(void)
{
    return setvbuf(stdout, NULL, _IONBF, 0);
}

void usb_console_init(void)
{
    if (!usb_console_initialized) {
        __atomic_store_n(&usb_console_rx_head, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&usb_console_rx_tail, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&usb_console_rx_overflows, 0U, __ATOMIC_RELEASE);
        usb_console_mutex = xSemaphoreCreateMutexStatic(&usb_console_mutex_storage);
        usb_console_completion =
            xSemaphoreCreateBinaryStatic(&usb_console_completion_storage);
        usb_console_initialized = usb_console_mutex != NULL
                                  && usb_console_completion != NULL;
    }

    if (usb_console_initialized && !usb_console_stdout_unbuffered
        && usb_console_set_stdout_unbuffered() == 0) {
        usb_console_stdout_unbuffered = true;
    }
}

void usb_console_receive_from_isr(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0U) {
        return;
    }

    uint32_t head = __atomic_load_n(&usb_console_rx_head, __ATOMIC_RELAXED);
    const uint32_t tail = __atomic_load_n(&usb_console_rx_tail, __ATOMIC_ACQUIRE);
    size_t accepted = 0U;

    while (accepted < length && (head - tail) < USB_CONSOLE_RX_CAPACITY) {
        usb_console_rx_ring[head & (USB_CONSOLE_RX_CAPACITY - 1U)] = data[accepted++];
        ++head;
    }

    __atomic_store_n(&usb_console_rx_head, head, __ATOMIC_RELEASE);
    if (accepted < length) {
        (void)__atomic_add_fetch(&usb_console_rx_overflows,
                                 (uint32_t)(length - accepted),
                                 __ATOMIC_RELAXED);
    }
}

size_t usb_console_rx_available(void)
{
    const uint32_t head = __atomic_load_n(&usb_console_rx_head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n(&usb_console_rx_tail, __ATOMIC_RELAXED);
    const uint32_t available = head - tail;
    return available > USB_CONSOLE_RX_CAPACITY ? USB_CONSOLE_RX_CAPACITY
                                                : (size_t)available;
}

bool usb_console_read_byte(uint8_t *byte)
{
    if (byte == NULL) {
        return false;
    }

    uint32_t tail = __atomic_load_n(&usb_console_rx_tail, __ATOMIC_RELAXED);
    const uint32_t head = __atomic_load_n(&usb_console_rx_head, __ATOMIC_ACQUIRE);
    if (tail == head) {
        return false;
    }

    *byte = usb_console_rx_ring[tail & (USB_CONSOLE_RX_CAPACITY - 1U)];
    __atomic_store_n(&usb_console_rx_tail, tail + 1U, __ATOMIC_RELEASE);
    return true;
}

size_t usb_console_read(uint8_t *data, size_t capacity)
{
    if (data == NULL || capacity == 0U) {
        return 0U;
    }

    size_t count = 0U;
    while (count < capacity && usb_console_read_byte(&data[count])) {
        ++count;
    }
    return count;
}

uint32_t usb_console_rx_overflow_count(void)
{
    return __atomic_load_n(&usb_console_rx_overflows, __ATOMIC_ACQUIRE);
}

void usb_console_service(void)
{
    if (usb_console_initialized) {
        (void)CDC_RearmRx_FS();
    }
}

bool usb_console_ready(void)
{
    if (!usb_console_initialized || !usb_console_transport_is_online()) {
        return false;
    }
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
           && hUsbDeviceFS.pClassData != NULL;
}

int usb_console_write(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    if (data == NULL && length > 0U) {
        return usb_console_fail(EINVAL);
    }
    if (length > sizeof(usb_console_tx_staging)) {
        return usb_console_fail(EMSGSIZE);
    }
    if (!usb_console_initialized) {
        return usb_console_fail(EAGAIN);
    }
    if (usb_console_is_in_isr()) {
        return usb_console_fail(EPERM);
    }
    if (!usb_console_scheduler_is_running()) {
        return usb_console_fail(EAGAIN);
    }
    if (length == 0U) {
        return 0;
    }
    if (!usb_console_ready()) {
        return usb_console_fail(EAGAIN);
    }

    const uint32_t start_ms = usb_console_now_ms();
    if (!usb_console_take_mutex(usb_console_remaining_ms(start_ms, timeout_ms))) {
        return usb_console_fail(ETIMEDOUT);
    }

    int result = -1;
    int error_number = 0;

    if (!usb_console_ready()) {
        error_number = EAGAIN;
        goto done;
    }

    /* USB suspend clears readiness without invalidating an accepted transfer.
     * Only its completion or a transport-epoch change releases staging. */
    if (usb_console_in_flight) {
        const uint32_t old_epoch = usb_console_in_flight_epoch;
        if (usb_console_epoch_load() != old_epoch) {
            usb_console_in_flight = false;
            usb_console_drain_completion();
        } else if (usb_console_take_completion(
                       usb_console_remaining_ms(start_ms, timeout_ms))) {
            usb_console_in_flight = false;
            if (usb_console_epoch_load() != old_epoch) {
                usb_console_drain_completion();
                error_number = EPIPE;
                goto done;
            }
        } else if (usb_console_epoch_load() != old_epoch) {
            usb_console_in_flight = false;
            usb_console_drain_completion();
            error_number = EPIPE;
            goto done;
        } else {
            error_number = ETIMEDOUT;
            goto done;
        }
    }

    if (!usb_console_ready()) {
        error_number = EPIPE;
        goto done;
    }

    usb_console_drain_completion();
    memcpy(usb_console_tx_staging, data, length);

    for (;;) {
        if (!usb_console_ready()) {
            error_number = EPIPE;
            break;
        }

        usb_console_drain_completion();
        const uint32_t submit_epoch = usb_console_epoch_load();
        if (!usb_console_ready()) {
            error_number = EPIPE;
            break;
        }

        usb_console_in_flight = true;
        usb_console_in_flight_epoch = submit_epoch;
        const usb_console_low_result_t transmit_result =
            usb_console_transmit(usb_console_tx_staging, length);

        if (transmit_result == USB_CONSOLE_LOW_ACCEPTED) {
            if (usb_console_take_completion(
                    usb_console_remaining_ms(start_ms, timeout_ms))) {
                usb_console_in_flight = false;
                if (usb_console_epoch_load() != submit_epoch) {
                    usb_console_drain_completion();
                    error_number = EPIPE;
                } else {
                    result = (int)length;
                }
            } else if (usb_console_epoch_load() != submit_epoch) {
                usb_console_in_flight = false;
                usb_console_drain_completion();
                error_number = EPIPE;
            } else {
                error_number = ETIMEDOUT;
            }
            break;
        }

        usb_console_in_flight = false;
        if (usb_console_epoch_load() != submit_epoch || !usb_console_ready()) {
            usb_console_drain_completion();
            error_number = EPIPE;
            break;
        }
        if (transmit_result == USB_CONSOLE_LOW_FAILED) {
            error_number = EIO;
            break;
        }

        const uint32_t remaining_ms =
            usb_console_remaining_ms(start_ms, timeout_ms);
        if (!usb_console_yield_one_tick(remaining_ms)
            || usb_console_remaining_ms(start_ms, timeout_ms) == 0U) {
            error_number = ETIMEDOUT;
            break;
        }
    }

done:
    usb_console_give_mutex();
    if (result < 0) {
        errno = error_number;
    }
    return result;
}

void usb_console_tx_complete_from_isr(void)
{
    usb_console_signal_completion();
}

void usb_console_transport_connected(void)
{
    usb_console_epoch_advance();
    usb_console_transport_set_online(true);
}

void usb_console_transport_disconnected(void)
{
    usb_console_transport_set_online(false);
    usb_console_epoch_advance();
    usb_console_signal_completion();
}

int _write(int fd, char *data, int length)
{
    if (fd != 1 && fd != 2) {
        return usb_console_fail(EBADF);
    }
    if (length < 0) {
        return usb_console_fail(EINVAL);
    }
    return usb_console_write((const uint8_t *)data, (size_t)length, 100U);
}

#ifdef APP_USB_CONSOLE_FREERTOS_TEST_SEAM
void usb_console_freertos_test_reset(void)
{
    usb_console_mutex = NULL;
    usb_console_completion = NULL;
    memset(&usb_console_mutex_storage, 0, sizeof(usb_console_mutex_storage));
    memset(&usb_console_completion_storage, 0,
           sizeof(usb_console_completion_storage));
    usb_console_initialized = false;
    usb_console_in_flight = false;
    usb_console_in_flight_epoch = 0U;
    __atomic_store_n(&usb_console_transport_epoch, 0U, __ATOMIC_RELEASE);
    usb_console_transport_set_online(false);
    memset(usb_console_tx_staging, 0, sizeof(usb_console_tx_staging));
}
#endif

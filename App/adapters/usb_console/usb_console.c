#include "usb_console.h"
#include "usb_console_internal.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#ifdef APP_HOST_TEST

static usb_console_test_backend_t usb_console_backend;
static void *usb_console_class_data;
static bool usb_console_host_mutex_available;
static bool usb_console_host_completion;

#else

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

#endif

typedef enum {
    USB_CONSOLE_LOW_ACCEPTED = 0,
    USB_CONSOLE_LOW_BUSY,
    USB_CONSOLE_LOW_FAILED,
} usb_console_low_result_t;

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
#ifdef APP_HOST_TEST
    if (usb_console_backend.now_ms == NULL) {
        return 0U;
    }
    return usb_console_backend.now_ms(usb_console_backend.context);
#else
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
#endif
}

static uint32_t usb_console_remaining_ms(uint32_t start_ms, uint32_t timeout_ms)
{
    const uint32_t elapsed_ms = usb_console_now_ms() - start_ms;
    return elapsed_ms >= timeout_ms ? 0U : timeout_ms - elapsed_ms;
}

#ifdef APP_HOST_TEST
static bool usb_console_host_take(bool *available, uint32_t wait_ms)
{
    if (*available) {
        *available = false;
        return true;
    }
    if (usb_console_backend.poll == NULL) {
        return false;
    }

    const uint32_t start_ms = usb_console_now_ms();
    for (uint32_t polls = 0U; polls < wait_ms; ++polls) {
        usb_console_backend.poll(usb_console_backend.context);
        if (*available) {
            *available = false;
            return true;
        }
        if ((usb_console_now_ms() - start_ms) >= wait_ms) {
            break;
        }
    }
    return false;
}
#endif

static bool usb_console_take_mutex(uint32_t wait_ms)
{
#ifdef APP_HOST_TEST
    return usb_console_host_take(&usb_console_host_mutex_available, wait_ms);
#else
    return xSemaphoreTake(usb_console_mutex,
                          usb_console_ms_to_ticks(wait_ms)) == pdTRUE;
#endif
}

static void usb_console_give_mutex(void)
{
#ifdef APP_HOST_TEST
    usb_console_host_mutex_available = true;
#else
    (void)xSemaphoreGive(usb_console_mutex);
#endif
}

static bool usb_console_take_completion(uint32_t wait_ms)
{
#ifdef APP_HOST_TEST
    return usb_console_host_take(&usb_console_host_completion, wait_ms);
#else
    return xSemaphoreTake(usb_console_completion,
                          usb_console_ms_to_ticks(wait_ms)) == pdTRUE;
#endif
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
#ifdef APP_HOST_TEST
    usb_console_host_completion = true;
#else
    if (xPortIsInsideInterrupt() != pdFALSE) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        (void)xSemaphoreGiveFromISR(usb_console_completion,
                                    &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    } else {
        (void)xSemaphoreGive(usb_console_completion);
    }
#endif
}

static bool usb_console_is_in_isr(void)
{
#ifdef APP_HOST_TEST
    return usb_console_backend.is_in_isr != NULL
           && usb_console_backend.is_in_isr(usb_console_backend.context);
#else
    return xPortIsInsideInterrupt() != pdFALSE;
#endif
}

static bool usb_console_scheduler_is_running(void)
{
#ifdef APP_HOST_TEST
    return usb_console_backend.is_scheduler_running != NULL
           && usb_console_backend.is_scheduler_running(usb_console_backend.context);
#else
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
#endif
}

static usb_console_low_result_t usb_console_transmit(const uint8_t *data,
                                                     size_t length)
{
#ifdef APP_HOST_TEST
    if (usb_console_backend.transmit == NULL) {
        return USB_CONSOLE_LOW_FAILED;
    }
    switch (usb_console_backend.transmit(usb_console_backend.context, data, length)) {
    case USB_CONSOLE_TX_ACCEPTED:
        return USB_CONSOLE_LOW_ACCEPTED;
    case USB_CONSOLE_TX_BUSY:
        return USB_CONSOLE_LOW_BUSY;
    case USB_CONSOLE_TX_FAILED:
    default:
        return USB_CONSOLE_LOW_FAILED;
    }
#else
    const uint8_t status = CDC_Transmit_FS((uint8_t *)data, (uint16_t)length);
    if (status == USBD_OK) {
        return USB_CONSOLE_LOW_ACCEPTED;
    }
    if (status == USBD_BUSY) {
        return USB_CONSOLE_LOW_BUSY;
    }
    return USB_CONSOLE_LOW_FAILED;
#endif
}

static bool usb_console_yield_one_tick(uint32_t remaining_ms)
{
    if (remaining_ms == 0U) {
        return false;
    }
#ifdef APP_HOST_TEST
    if (usb_console_backend.poll == NULL) {
        return false;
    }
    usb_console_backend.poll(usb_console_backend.context);
#else
    vTaskDelay(usb_console_ms_to_ticks(1U));
#endif
    return true;
}

static int usb_console_fail(int error_number)
{
    errno = error_number;
    return -1;
}

static int usb_console_set_stdout_unbuffered(void)
{
#ifdef APP_HOST_TEST
    if (usb_console_backend.set_stdout_unbuffered != NULL) {
        return usb_console_backend.set_stdout_unbuffered(
            usb_console_backend.context);
    }
#endif
    return setvbuf(stdout, NULL, _IONBF, 0);
}

void usb_console_init(void)
{
    if (!usb_console_initialized) {
#ifdef APP_HOST_TEST
        usb_console_host_mutex_available = true;
        usb_console_host_completion = false;
        usb_console_initialized = true;
#else
        usb_console_mutex = xSemaphoreCreateMutexStatic(&usb_console_mutex_storage);
        usb_console_completion =
            xSemaphoreCreateBinaryStatic(&usb_console_completion_storage);
        usb_console_initialized = usb_console_mutex != NULL
                                  && usb_console_completion != NULL;
#endif
    }

    if (usb_console_initialized && !usb_console_stdout_unbuffered
        && usb_console_set_stdout_unbuffered() == 0) {
        usb_console_stdout_unbuffered = true;
    }
}

bool usb_console_ready(void)
{
    if (!usb_console_initialized || !usb_console_transport_is_online()) {
        return false;
    }
#ifdef APP_HOST_TEST
    return usb_console_class_data != NULL
           && usb_console_backend.is_configured != NULL
           && usb_console_backend.is_configured(usb_console_backend.context);
#else
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
           && hUsbDeviceFS.pClassData != NULL;
#endif
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

#ifdef APP_HOST_TEST
void usb_console_test_set_backend(const usb_console_test_backend_t *backend)
{
    memset(&usb_console_backend, 0, sizeof(usb_console_backend));
    if (backend != NULL) {
        usb_console_backend = *backend;
        usb_console_class_data = backend->class_data;
    } else {
        usb_console_class_data = NULL;
    }
    usb_console_initialized = false;
    usb_console_host_mutex_available = false;
    usb_console_host_completion = false;
    usb_console_stdout_unbuffered = false;
    usb_console_in_flight = false;
    usb_console_in_flight_epoch = 0U;
    __atomic_store_n(&usb_console_transport_epoch, 0U, __ATOMIC_RELEASE);
    usb_console_transport_set_online(false);
    memset(usb_console_tx_staging, 0, sizeof(usb_console_tx_staging));
}

void usb_console_test_transport_connected(void *class_data)
{
    usb_console_class_data = class_data;
    usb_console_transport_connected();
}

void usb_console_test_transport_disconnected(void)
{
    usb_console_class_data = NULL;
    usb_console_transport_disconnected();
}
#endif

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

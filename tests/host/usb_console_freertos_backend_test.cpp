#include "test_framework.hpp"

extern "C" {
#include "Dima/adapters/usb_console/usb_console.h"
#include "Dima/adapters/usb_console/usb_console_internal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
}

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {

enum class TxResult : std::uint8_t {
    Accepted = USBD_OK,
    Busy = USBD_BUSY,
    Failed = USBD_FAIL,
};

BaseType_t g_scheduler_state{taskSCHEDULER_RUNNING};
bool g_inside_isr{false};
TickType_t g_tick_count{0U};
unsigned g_mutex_create_calls{0U};
unsigned g_binary_create_calls{0U};
SemaphoreHandle_t g_mutex_handle{nullptr};
unsigned g_take_calls{0U};
unsigned g_give_calls{0U};
unsigned g_give_from_isr_calls{0U};
unsigned g_yield_from_isr_calls{0U};
unsigned g_delay_calls{0U};
TickType_t g_last_take_ticks{0U};
TickType_t g_last_delay_ticks{0U};
void (*g_take_hook)(){nullptr};
std::array<TxResult, 8> g_tx_results{};
std::size_t g_tx_result_count{0U};
unsigned g_transmit_calls{0U};
unsigned g_complete_during_transmit_call{0U};
std::uint8_t *g_accepted_buffer{nullptr};
std::uint16_t g_accepted_length{0U};
int g_class_storage{0};

void reset_fixture()
{
    usb_console_freertos_test_reset();
    hUsbDeviceFS = {};
    g_scheduler_state = taskSCHEDULER_RUNNING;
    g_inside_isr = false;
    g_tick_count = 0U;
    g_mutex_create_calls = 0U;
    g_binary_create_calls = 0U;
    g_mutex_handle = nullptr;
    g_take_calls = 0U;
    g_give_calls = 0U;
    g_give_from_isr_calls = 0U;
    g_yield_from_isr_calls = 0U;
    g_delay_calls = 0U;
    g_last_take_ticks = 0U;
    g_last_delay_ticks = 0U;
    g_take_hook = nullptr;
    g_tx_results = {};
    g_tx_result_count = 0U;
    g_transmit_calls = 0U;
    g_complete_during_transmit_call = 0U;
    g_accepted_buffer = nullptr;
    g_accepted_length = 0U;
}

void configure_transport()
{
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = &g_class_storage;
    usb_console_transport_connected();
}

void complete_from_interrupt()
{
    g_inside_isr = true;
    usb_console_tx_complete_from_isr();
    g_inside_isr = false;
}

} // namespace

extern "C" {
USBD_HandleTypeDef hUsbDeviceFS = {};
}

extern "C" BaseType_t xPortIsInsideInterrupt(void)
{
    return g_inside_isr ? pdTRUE : pdFALSE;
}

extern "C" BaseType_t xTaskGetSchedulerState(void)
{
    return g_scheduler_state;
}

extern "C" TickType_t xTaskGetTickCount(void)
{
    return g_tick_count;
}

extern "C" void vTaskDelay(TickType_t ticks_to_delay)
{
    ++g_delay_calls;
    g_last_delay_ticks = ticks_to_delay;
    g_tick_count += ticks_to_delay;
}

extern "C" SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *storage)
{
    ++g_mutex_create_calls;
    storage->kind = 1U;
    storage->count = 1U;
    g_mutex_handle = storage;
    return storage;
}

extern "C" SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *storage)
{
    ++g_binary_create_calls;
    storage->kind = 2U;
    storage->count = 0U;
    return storage;
}

extern "C" BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore,
                                       TickType_t ticks_to_wait)
{
    ++g_take_calls;
    g_last_take_ticks = ticks_to_wait;
    if (semaphore->count > 0U) {
        --semaphore->count;
        return pdTRUE;
    }
    if (ticks_to_wait > 0U && g_take_hook != nullptr) {
        void (*const hook)() = g_take_hook;
        g_take_hook = nullptr;
        hook();
        if (semaphore->count > 0U) {
            --semaphore->count;
            return pdTRUE;
        }
    }
    g_tick_count += ticks_to_wait;
    return pdFALSE;
}

extern "C" BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    ++g_give_calls;
    semaphore->count = 1U;
    return pdTRUE;
}

extern "C" BaseType_t xSemaphoreGiveFromISR(
    SemaphoreHandle_t semaphore, BaseType_t *higher_priority_task_woken)
{
    ++g_give_from_isr_calls;
    semaphore->count = 1U;
    if (higher_priority_task_woken != nullptr) {
        *higher_priority_task_woken = pdTRUE;
    }
    return pdTRUE;
}

extern "C" void freertos_usb_test_yield_from_isr(
    BaseType_t higher_priority_task_woken)
{
    CHECK_EQ(higher_priority_task_woken, pdTRUE);
    ++g_yield_from_isr_calls;
}

extern "C" uint8_t CDC_RearmRx_FS(void)
{
    return USBD_OK;
}

extern "C" uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    ++g_transmit_calls;
    const std::size_t slot = g_transmit_calls - 1U;
    const TxResult result = slot < g_tx_result_count
                                ? g_tx_results[slot]
                                : TxResult::Accepted;
    if (result == TxResult::Accepted) {
        g_accepted_buffer = buffer;
        g_accepted_length = length;
        if (g_complete_during_transmit_call == g_transmit_calls) {
            complete_from_interrupt();
        }
    }
    return static_cast<uint8_t>(result);
}

HOST_TEST(usb_console_freertos_backend_creates_only_static_primitives_and_init_is_idempotent)
{
    reset_fixture();
    usb_console_init();
    CHECK_EQ(g_mutex_create_calls, 1U);
    CHECK_EQ(g_binary_create_calls, 1U);
    usb_console_init();
    CHECK_EQ(g_mutex_create_calls, 1U);
    CHECK_EQ(g_binary_create_calls, 1U);

    configure_transport();
    CHECK(usb_console_ready());
}

HOST_TEST(usb_console_freertos_backend_rejects_isr_and_nonrunning_scheduler_before_sync_calls)
{
    reset_fixture();
    usb_console_init();
    configure_transport();

    g_inside_isr = true;
    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("x"), 1U, 4U), -1);
    CHECK_EQ(errno, EPERM);
    g_inside_isr = false;
    g_scheduler_state = taskSCHEDULER_NOT_STARTED;
    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("x"), 1U, 4U), -1);
    CHECK_EQ(errno, EAGAIN);
    g_scheduler_state = taskSCHEDULER_SUSPENDED;
    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("x"), 1U, 4U), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(g_take_calls, 0U);
    CHECK_EQ(g_transmit_calls, 0U);
}

HOST_TEST(usb_console_freertos_backend_busy_path_yields_one_tick_instead_of_spinning)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    g_tx_results[0] = TxResult::Busy;
    g_tx_results[1] = TxResult::Accepted;
    g_tx_result_count = 2U;
    g_complete_during_transmit_call = 2U;

    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("ok"), 2U, 4U), 2);
    CHECK_EQ(g_delay_calls, 1U);
    CHECK_EQ(g_last_delay_ticks, 1U);
    CHECK_EQ(g_transmit_calls, 2U);
    CHECK(g_take_calls >= 2U);
    CHECK(g_give_calls >= 1U);
}

HOST_TEST(usb_console_freertos_backend_mutex_contention_blocks_submission_until_deadline)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    CHECK(g_mutex_handle != nullptr);
    g_mutex_handle->count = 0U;

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("x"), 1U, 3U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_tick_count, 3U);
    CHECK_EQ(g_transmit_calls, 0U);
    CHECK_EQ(g_give_calls, 0U);
}

HOST_TEST(usb_console_freertos_backend_clamps_uint32_max_timeout_to_finite_ticks)
{
    reset_fixture();
    usb_console_init();
    configure_transport();

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("x"), 1U,
                               UINT32_MAX), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_last_take_ticks, portMAX_DELAY - 1U);
    CHECK(g_last_take_ticks != portMAX_DELAY);
    CHECK_EQ(g_transmit_calls, 1U);
}

HOST_TEST(usb_console_freertos_backend_timeout_keeps_staging_until_isr_completion)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    std::uint8_t caller[] = {'o', 'l', 'd'};

    errno = 0;
    CHECK_EQ(usb_console_write(caller, sizeof(caller), 4U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_transmit_calls, 1U);
    CHECK_EQ(g_accepted_length, sizeof(caller));
    caller[0] = 'X';
    CHECK_EQ(g_accepted_buffer[0], static_cast<std::uint8_t>('o'));

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 2U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_transmit_calls, 1U);

    complete_from_interrupt();
    CHECK_EQ(g_give_from_isr_calls, 1U);
    CHECK_EQ(g_yield_from_isr_calls, 1U);
    g_complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 2U), 3);
    CHECK_EQ(g_transmit_calls, 2U);
}

HOST_TEST(usb_console_freertos_backend_suspend_without_epoch_keeps_staging_owned)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    g_take_hook = [] {
        hUsbDeviceFS.dev_state = USBD_STATE_SUSPENDED;
    };

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("old"), 3U, 2U),
             -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_transmit_calls, 1U);
    CHECK_EQ(g_accepted_length, 3U);
    CHECK(std::memcmp(g_accepted_buffer, "old", 3U) == 0);

    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 1U),
             -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(g_transmit_calls, 1U);
    CHECK(std::memcmp(g_accepted_buffer, "old", 3U) == 0);

    complete_from_interrupt();
    g_complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 1U),
             3);
    CHECK_EQ(g_transmit_calls, 2U);
    CHECK(std::memcmp(g_accepted_buffer, "new", 3U) == 0);
}

HOST_TEST(usb_console_freertos_backend_disconnect_hook_wakes_waiter_and_changes_epoch)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    g_take_hook = [] {
        hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
        hUsbDeviceFS.pClassData = nullptr;
        g_inside_isr = true;
        usb_console_transport_disconnected();
        g_inside_isr = false;
    };

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("old"), 3U, 8U), -1);
    CHECK_EQ(errno, EPIPE);
    CHECK_EQ(g_give_from_isr_calls, 1U);
    CHECK_EQ(g_yield_from_isr_calls, 1U);

    configure_transport();
    g_complete_during_transmit_call = 2U;
    errno = 0;
    const int reconnect_result =
        usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 2U);
    CHECK_EQ(g_transmit_calls, 2U);
    CHECK_EQ(g_give_from_isr_calls, 2U);
    CHECK_EQ(errno, 0);
    CHECK_EQ(reconnect_result, 3);
}

HOST_TEST(usb_console_freertos_backend_task_disconnect_uses_task_semaphore_api)
{
    reset_fixture();
    usb_console_init();
    configure_transport();
    g_take_hook = [] {
        usb_console_transport_disconnected();
    };

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("old"), 3U, 8U),
             -1);
    CHECK_EQ(errno, EPIPE);
    CHECK_EQ(g_give_from_isr_calls, 0U);
    CHECK_EQ(g_yield_from_isr_calls, 0U);
    CHECK_EQ(g_give_calls, 2U);
    CHECK(!usb_console_ready());

    errno = 0;
    CHECK_EQ(usb_console_write(reinterpret_cast<const std::uint8_t *>("new"), 3U, 1U),
             -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(g_transmit_calls, 1U);
    CHECK(std::memcmp(g_accepted_buffer, "old", 3U) == 0);
}

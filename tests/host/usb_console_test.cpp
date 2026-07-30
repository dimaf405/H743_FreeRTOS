#include "test_framework.hpp"

extern "C" {
#include "Dima/adapters/usb_console/usb_console.h"
#include "Dima/adapters/usb_console/usb_console_internal.h"
}

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <string>
#include <type_traits>

static_assert(std::is_same<decltype(&usb_console_init), void (*)(void)>::value,
              "usb_console_init must be declared exactly as void(void)");
static_assert(std::is_same<decltype(&usb_console_ready), bool (*)(void)>::value,
              "usb_console_ready must be declared exactly as bool(void)");
static_assert(std::is_same<decltype(&usb_console_write),
                           int (*)(const std::uint8_t *, std::size_t, std::uint32_t)>::value,
              "usb_console_write must have the fixed public C API signature");
static_assert(std::is_same<decltype(&usb_console_tx_complete_from_isr), void (*)(void)>::value,
              "usb_console_tx_complete_from_isr must be declared exactly as void(void)");
static_assert(std::is_same<decltype(&_write), int (*)(int, char *, int)>::value,
              "_write must have the fixed newlib hook signature");

namespace {

constexpr std::size_t kMaxRecordedTransmits = 128U;
constexpr std::size_t kMaxPayload = 256U;

struct FakeUsb {
    bool configured{false};
    bool inside_isr{false};
    bool scheduler_running{true};
    std::uint32_t now{0U};
    unsigned transmit_calls{0U};
    unsigned poll_calls{0U};
    unsigned complete_during_transmit_call{0U};
    unsigned complete_on_poll_number{0U};
    unsigned disconnect_during_transmit_call{0U};
    unsigned disconnect_on_poll_number{0U};
    unsigned suspend_on_poll_number{0U};
    unsigned setvbuf_calls{0U};
    unsigned setvbuf_failures_remaining{0U};
    std::array<usb_console_tx_result_t, kMaxRecordedTransmits> results{};
    std::size_t result_count{0U};
    usb_console_tx_result_t default_result{USB_CONSOLE_TX_ACCEPTED};
    const std::uint8_t *accepted_buffer{nullptr};
    std::size_t accepted_length{0U};
    std::array<std::array<std::uint8_t, kMaxPayload>, kMaxRecordedTransmits> submitted{};
    std::array<std::size_t, kMaxRecordedTransmits> submitted_lengths{};

    void set_results(std::initializer_list<usb_console_tx_result_t> sequence)
    {
        CHECK(sequence.size() <= results.size());
        result_count = sequence.size();
        std::size_t index = 0U;
        for (const auto result : sequence) {
            results[index++] = result;
        }
    }

    static bool is_configured(void *context)
    {
        return static_cast<FakeUsb *>(context)->configured;
    }

    static usb_console_tx_result_t transmit(void *context, const std::uint8_t *data,
                                            std::size_t length)
    {
        auto &fake = *static_cast<FakeUsb *>(context);
        ++fake.transmit_calls;
        CHECK(fake.transmit_calls <= kMaxRecordedTransmits);
        CHECK(length <= kMaxPayload);
        const std::size_t slot = fake.transmit_calls - 1U;
        fake.submitted_lengths[slot] = length;
        if (length > 0U) {
            std::memcpy(fake.submitted[slot].data(), data, length);
        }

        if (fake.disconnect_during_transmit_call == fake.transmit_calls) {
            fake.configured = false;
            usb_console_test_transport_disconnected();
        }

        const usb_console_tx_result_t result =
            slot < fake.result_count ? fake.results[slot] : fake.default_result;
        if (result == USB_CONSOLE_TX_ACCEPTED) {
            fake.accepted_buffer = data;
            fake.accepted_length = length;
            if (fake.complete_during_transmit_call == fake.transmit_calls) {
                usb_console_tx_complete_from_isr();
            }
        }
        return result;
    }

    static std::uint32_t now_ms(void *context)
    {
        return static_cast<FakeUsb *>(context)->now;
    }

    static void poll(void *context)
    {
        auto &fake = *static_cast<FakeUsb *>(context);
        ++fake.poll_calls;
        ++fake.now;
        if (fake.suspend_on_poll_number != 0U &&
            fake.poll_calls >= fake.suspend_on_poll_number) {
            fake.suspend_on_poll_number = 0U;
            fake.configured = false;
        }
        if (fake.disconnect_on_poll_number != 0U &&
            fake.poll_calls >= fake.disconnect_on_poll_number) {
            fake.disconnect_on_poll_number = 0U;
            fake.configured = false;
            usb_console_test_transport_disconnected();
        }
        if (fake.complete_on_poll_number != 0U &&
            fake.poll_calls >= fake.complete_on_poll_number) {
            fake.complete_on_poll_number = 0U;
            usb_console_tx_complete_from_isr();
        }
    }

    static bool is_in_isr(void *context)
    {
        return static_cast<FakeUsb *>(context)->inside_isr;
    }

    static bool is_scheduler_running(void *context)
    {
        return static_cast<FakeUsb *>(context)->scheduler_running;
    }

    static int set_stdout_unbuffered(void *context)
    {
        auto &fake = *static_cast<FakeUsb *>(context);
        ++fake.setvbuf_calls;
        if (fake.setvbuf_failures_remaining > 0U) {
            --fake.setvbuf_failures_remaining;
            return -1;
        }
        return 0;
    }

    usb_console_test_backend_t backend(void *class_data = nullptr)
    {
        return {this, class_data, &is_configured, &transmit, &now_ms, &poll,
                &is_in_isr, &is_scheduler_running, &set_stdout_unbuffered};
    }
};

void configure(FakeUsb &fake, void *class_data = reinterpret_cast<void *>(1))
{
    const usb_console_test_backend_t backend = fake.backend(class_data);
    usb_console_test_set_backend(&backend);
    usb_console_init();
    if (class_data != nullptr) {
        usb_console_test_transport_connected(class_data);
    }
}

const std::uint8_t *bytes(const char *text)
{
    return reinterpret_cast<const std::uint8_t *>(text);
}

} // namespace

HOST_TEST(usb_console_init_retries_setvbuf_until_stdout_is_unbuffered)
{
    FakeUsb fake;
    fake.configured = true;
    fake.setvbuf_failures_remaining = 1U;
    configure(fake);

    CHECK_EQ(fake.setvbuf_calls, 1U);
    usb_console_init();
    CHECK_EQ(fake.setvbuf_calls, 2U);
    usb_console_init();
    CHECK_EQ(fake.setvbuf_calls, 2U);
}

HOST_TEST(usb_console_ready_requires_configured_device_and_nonnull_class_data)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);
    CHECK(usb_console_ready());

    fake.configured = false;
    CHECK(!usb_console_ready());
    fake.configured = true;
    usb_console_test_transport_disconnected();
    CHECK(!usb_console_ready());
}

HOST_TEST(usb_console_task_disconnect_blocks_writers_before_st_clears_class_state)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);

    usb_console_transport_disconnected();
    CHECK(!usb_console_ready());
    errno = 0;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 1U), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_handles_null_pclassdata_without_crashing)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake, nullptr);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 0U), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_returns_eagain_when_device_is_not_configured)
{
    FakeUsb fake;
    configure(fake);

    CHECK(!usb_console_ready());
    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 0U), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_retries_busy_with_bounded_yield_then_waits_for_completion)
{
    FakeUsb fake;
    fake.configured = true;
    fake.set_results({USB_CONSOLE_TX_BUSY, USB_CONSOLE_TX_ACCEPTED});
    fake.complete_during_transmit_call = 2U;
    configure(fake);

    CHECK_EQ(usb_console_write(bytes("ok"), 2U, 4U), 2);
    CHECK_EQ(fake.transmit_calls, 2U);
    CHECK_EQ(fake.poll_calls, 1U);
}

HOST_TEST(usb_console_busy_with_zero_timeout_is_single_attempt_and_not_a_spin_loop)
{
    FakeUsb fake;
    fake.configured = true;
    fake.default_result = USB_CONSOLE_TX_BUSY;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 0U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.transmit_calls, 1U);
    CHECK_EQ(fake.poll_calls, 0U);
}

HOST_TEST(usb_console_low_level_failure_returns_eio_immediately_and_can_recover)
{
    FakeUsb fake;
    fake.configured = true;
    fake.set_results({USB_CONSOLE_TX_FAILED, USB_CONSOLE_TX_ACCEPTED});
    fake.complete_during_transmit_call = 2U;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("bad"), 3U, 10U), -1);
    CHECK_EQ(errno, EIO);
    CHECK_EQ(fake.poll_calls, 0U);
    CHECK_EQ(usb_console_write(bytes("good"), 4U, 10U), 4);
    CHECK_EQ(fake.transmit_calls, 2U);
}

HOST_TEST(usb_console_completion_before_wait_is_not_lost)
{
    FakeUsb fake;
    fake.configured = true;
    fake.complete_during_transmit_call = 1U;
    configure(fake);

    CHECK_EQ(usb_console_write(bytes("sync"), 4U, 0U), 4);
    CHECK_EQ(fake.poll_calls, 0U);
}

HOST_TEST(usb_console_callback_does_not_depend_on_backend_context_matching_class_data)
{
    FakeUsb fake;
    int unrelated_class_storage = 0;
    fake.configured = true;
    fake.complete_on_poll_number = 1U;
    configure(fake, &unrelated_class_storage);

    CHECK_EQ(usb_console_write(bytes("x"), 1U, 2U), 1);
    CHECK_NE(static_cast<void *>(&fake), static_cast<void *>(&unrelated_class_storage));
}

HOST_TEST(usb_console_busy_wait_is_bounded_by_fake_clock_not_wall_time)
{
    FakeUsb fake;
    fake.configured = true;
    fake.default_result = USB_CONSOLE_TX_BUSY;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 4U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK(fake.poll_calls <= 4U);
    CHECK_EQ(fake.now, 4U);
}

HOST_TEST(usb_console_total_deadline_is_uint32_wrap_safe)
{
    FakeUsb fake;
    fake.configured = true;
    fake.default_result = USB_CONSOLE_TX_BUSY;
    fake.now = UINT32_MAX - 1U;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 4U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.poll_calls, 4U);
    CHECK_EQ(fake.now, 2U);
}

HOST_TEST(usb_console_can_transmit_again_after_completion)
{
    FakeUsb fake;
    fake.configured = true;
    fake.complete_on_poll_number = 1U;
    configure(fake);

    CHECK_EQ(usb_console_write(bytes("first"), 5U, 3U), 5);
    fake.complete_on_poll_number = fake.poll_calls + 1U;
    CHECK_EQ(usb_console_write(bytes("next"), 4U, 3U), 4);
    CHECK_EQ(fake.transmit_calls, 2U);
}

HOST_TEST(usb_console_timeout_after_acceptance_preserves_staging_ownership)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);

    std::uint8_t first[] = {'f', 'i', 'r', 's', 't'};
    errno = 0;
    CHECK_EQ(usb_console_write(first, sizeof(first), 4U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    const std::string staged_before(reinterpret_cast<const char *>(fake.accepted_buffer),
                                    fake.accepted_length);
    first[0] = 'X';
    first[1] = 'X';
    first[2] = 'X';
    first[3] = 'X';
    first[4] = 'X';
    const std::uint8_t second[] = {'s', 'e', 'c', 'o', 'n', 'd'};

    errno = 0;
    CHECK_EQ(usb_console_write(second, sizeof(second), 3U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    const std::string staged_after(reinterpret_cast<const char *>(fake.accepted_buffer),
                                   fake.accepted_length);
    CHECK_EQ(staged_before, "first");
    CHECK_EQ(staged_after, "first");
    CHECK_EQ(fake.transmit_calls, 1U);
}

HOST_TEST(usb_console_suspend_without_epoch_preserves_staging_until_late_completion)
{
    FakeUsb fake;
    fake.configured = true;
    fake.suspend_on_poll_number = 1U;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("old"), 3U, 2U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.transmit_calls, 1U);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(fake.accepted_buffer), 3U),
             "old");

    fake.configured = true;
    errno = 0;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 1U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.transmit_calls, 1U);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(fake.accepted_buffer), 3U),
             "old");

    usb_console_tx_complete_from_isr();
    fake.complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 1U), 3);
    CHECK_EQ(fake.transmit_calls, 2U);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(fake.submitted[1].data()), 3U),
             "new");
}

HOST_TEST(usb_console_late_completion_releases_staging_for_a_new_transfer)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("old"), 3U, 2U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    usb_console_tx_complete_from_isr();
    fake.complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 2U), 3);
    CHECK_EQ(fake.transmit_calls, 2U);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(fake.submitted[1].data()), 3U),
             "new");
}

HOST_TEST(usb_console_disconnect_before_accept_is_reported_as_broken_transport)
{
    FakeUsb fake;
    fake.configured = true;
    fake.default_result = USB_CONSOLE_TX_BUSY;
    fake.disconnect_during_transmit_call = 1U;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 5U), -1);
    CHECK_EQ(errno, EPIPE);
    CHECK_EQ(fake.transmit_calls, 1U);
}

HOST_TEST(usb_console_disconnect_after_accept_wakes_waiter_and_new_connection_recovers)
{
    FakeUsb fake;
    int class_storage = 0;
    fake.configured = true;
    fake.disconnect_on_poll_number = 1U;
    configure(fake, &class_storage);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("old"), 3U, 5U), -1);
    CHECK_EQ(errno, EPIPE);
    CHECK_EQ(fake.poll_calls, 1U);

    fake.configured = true;
    usb_console_test_transport_connected(&class_storage);
    fake.complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 2U), 3);
}

HOST_TEST(usb_console_same_pointer_reconnect_uses_epoch_and_discards_stale_wakeup)
{
    FakeUsb fake;
    int reused_class_storage = 0;
    fake.configured = true;
    configure(fake, &reused_class_storage);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("old"), 3U, 2U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    usb_console_test_transport_disconnected();
    fake.configured = true;
    usb_console_test_transport_connected(&reused_class_storage);
    fake.complete_during_transmit_call = 2U;

    CHECK_EQ(usb_console_write(bytes("new"), 3U, 2U), 3);
    CHECK_EQ(fake.transmit_calls, 2U);
}

HOST_TEST(usb_console_repeated_init_does_not_destroy_an_inflight_transfer)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("old"), 3U, 2U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    usb_console_init();
    errno = 0;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 0U), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.transmit_calls, 1U);

    usb_console_tx_complete_from_isr();
    fake.complete_during_transmit_call = 2U;
    CHECK_EQ(usb_console_write(bytes("new"), 3U, 1U), 3);
}

HOST_TEST(usb_console_validates_null_zero_and_maximum_lengths)
{
    FakeUsb fake;
    fake.configured = true;
    fake.complete_during_transmit_call = 1U;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(nullptr, 1U, 1U), -1);
    CHECK_EQ(errno, EINVAL);
    CHECK_EQ(usb_console_write(nullptr, 0U, 1U), 0);

    std::array<std::uint8_t, kMaxPayload> exact{};
    exact.front() = 0xA5U;
    exact.back() = 0x5AU;
    CHECK_EQ(usb_console_write(exact.data(), exact.size(), 1U),
             static_cast<int>(exact.size()));
    CHECK_EQ(fake.submitted_lengths[0], exact.size());
    CHECK_EQ(fake.submitted[0].front(), 0xA5U);
    CHECK_EQ(fake.submitted[0].back(), 0x5AU);

    errno = 0;
    CHECK_EQ(usb_console_write(exact.data(), exact.size() + 1U, 1U), -1);
    CHECK_EQ(errno, EMSGSIZE);
    CHECK_EQ(fake.transmit_calls, 1U);
}

HOST_TEST(usb_console_preserves_embedded_nul_bytes)
{
    FakeUsb fake;
    fake.configured = true;
    fake.complete_during_transmit_call = 1U;
    configure(fake);
    const std::uint8_t payload[] = {'A', 0U, 'B', 0U, 'C'};

    CHECK_EQ(usb_console_write(payload, sizeof(payload), 1U),
             static_cast<int>(sizeof(payload)));
    CHECK_EQ(fake.submitted_lengths[0], sizeof(payload));
    CHECK(std::memcmp(fake.submitted[0].data(), payload, sizeof(payload)) == 0);
}

HOST_TEST(usb_console_write_hook_forwards_stdout_and_stderr_with_binary_fidelity)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);
    char stdout_data[] = {'o', 0, '1'};
    char stderr_data[] = {'e', 0, '2'};

    fake.complete_during_transmit_call = 1U;
    CHECK_EQ(_write(1, stdout_data, 3), 3);
    fake.complete_during_transmit_call = 2U;
    CHECK_EQ(_write(2, stderr_data, 3), 3);
    CHECK(std::memcmp(fake.submitted[0].data(), stdout_data, 3U) == 0);
    CHECK(std::memcmp(fake.submitted[1].data(), stderr_data, 3U) == 0);
}

HOST_TEST(usb_console_write_hook_uses_one_hundred_millisecond_total_timeout)
{
    FakeUsb fake;
    fake.configured = true;
    fake.default_result = USB_CONSOLE_TX_BUSY;
    configure(fake);
    char message[] = "x";

    errno = 0;
    CHECK_EQ(_write(1, message, 1), -1);
    CHECK_EQ(errno, ETIMEDOUT);
    CHECK_EQ(fake.now, 100U);
    CHECK(fake.poll_calls <= 100U);
}

HOST_TEST(usb_console_write_hook_rejects_invalid_descriptor_and_negative_length)
{
    FakeUsb fake;
    fake.configured = true;
    configure(fake);
    char message[] = "x";

    errno = 0;
    CHECK_EQ(_write(17, message, 1), -1);
    CHECK_EQ(errno, EBADF);
    errno = 0;
    CHECK_EQ(_write(1, message, -1), -1);
    CHECK_EQ(errno, EINVAL);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_write_hook_rejects_unconfigured_transport_with_eagain)
{
    FakeUsb fake;
    configure(fake);
    char message[] = "x";

    errno = 0;
    CHECK_EQ(_write(1, message, 1), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_rejects_isr_context_without_polling_or_transmitting)
{
    FakeUsb fake;
    fake.configured = true;
    fake.inside_isr = true;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 5U), -1);
    CHECK_EQ(errno, EPERM);
    CHECK_EQ(fake.poll_calls, 0U);
    CHECK_EQ(fake.transmit_calls, 0U);
}

HOST_TEST(usb_console_rejects_stopped_or_suspended_scheduler_without_blocking)
{
    FakeUsb fake;
    fake.configured = true;
    fake.scheduler_running = false;
    configure(fake);

    errno = 0;
    CHECK_EQ(usb_console_write(bytes("x"), 1U, 5U), -1);
    CHECK_EQ(errno, EAGAIN);
    CHECK_EQ(fake.poll_calls, 0U);
    CHECK_EQ(fake.transmit_calls, 0U);
}

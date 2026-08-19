#include "UsbConsole.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace dima::adapters {
namespace {

constexpr std::size_t kRxCapacity = 1024U;
/* MAVLink 2 FILE_TRANSFER_PROTOCOL is 266 bytes unsigned; retain the full
 * generated MAVLINK_MAX_PACKET_LEN envelope without coupling this adapter to
 * protocol headers. */
constexpr std::size_t kTxCapacity = 280U;
static_assert((kRxCapacity & (kRxCapacity - 1U)) == 0U);

class UsbConsole final : public platform::Console {
public:
    UsbConsole(platform::Synchronization &synchronization,
               platform::TaskRuntime &tasks,
               platform::ExecutionContext &execution,
               platform::MonotonicClock &clock,
               platform::ConsoleTransport &transport) noexcept
        : synchronization_(synchronization), tasks_(tasks),
          execution_(execution), clock_(clock), transport_(transport)
    {
    }

    bool initialize() noexcept override
    {
        if (!initialized()) {
            reset_runtime_buffers();
            if (!tx_mutex_.initialize(synchronization_)) {
                return false;
            }
            if (!completion_.initialize(synchronization_)) {
                tx_mutex_.reset();
                return false;
            }
            if (!transport_.initialize()) {
                completion_.reset();
                tx_mutex_.reset();
                return false;
            }
            if (transport_.ready()) {
                __atomic_store_n(&transport_online_, true, __ATOMIC_RELEASE);
            }
            set_initialized(true);
        }
        if (initialized() && !stdout_unbuffered_ &&
            std::setvbuf(stdout, nullptr, _IONBF, 0) == 0) {
            stdout_unbuffered_ = true;
        }
        return initialized();
    }

    bool shutdown() noexcept override
    {
        if (execution_.in_interrupt()) {
            return false;
        }

        set_initialized(false);
        advance_epoch();
        if (completion_.valid()) {
            completion_.notify();
        }

        bool tx_locked = false;
        if (tx_mutex_.valid()) {
            tx_locked = tx_mutex_.lock();
            if (!tx_locked) {
                return false;
            }
        }

        in_flight_ = false;
        in_flight_epoch_ = 0U;
        if (completion_.valid()) {
            drain_completion();
        }
        reset_runtime_buffers();

        if (tx_locked) {
            tx_mutex_.unlock();
        }
        completion_.reset();
        tx_mutex_.reset();
        return true;
    }

    void service() noexcept override
    {
        if (initialized()) {
            transport_.service();
        }
    }

    bool ready() const noexcept override
    {
        return initialized() && transport_online() && transport_.ready();
    }

    int write(const std::uint8_t *data, std::size_t length,
              std::uint32_t timeout_ms) noexcept override
    {
        if (data == nullptr && length != 0U) {
            return fail(EINVAL);
        }
        if (length > sizeof(tx_staging_)) {
            return fail(EMSGSIZE);
        }
        if (!initialized() || !execution_.scheduler_running()) {
            return fail(EAGAIN);
        }
        if (execution_.in_interrupt()) {
            return fail(EPERM);
        }
        if (length == 0U) {
            return 0;
        }
        if (!ready()) {
            return fail(EAGAIN);
        }

        const std::uint32_t started = now_ms();
        platform::MutexGuard guard{
            tx_mutex_, platform::Timeout::from_ms(
                           remaining_ms(started, timeout_ms))};
        if (!guard) {
            return fail(ETIMEDOUT);
        }
        if (!ready()) {
            return fail(EAGAIN);
        }

        int previous_result = complete_previous_transfer(started, timeout_ms);
        if (previous_result != 0) {
            return fail(previous_result);
        }
        if (!ready()) {
            return fail(EPIPE);
        }

        drain_completion();
        std::memcpy(tx_staging_, data, length);
        for (;;) {
            if (!ready()) {
                return fail(EPIPE);
            }
            drain_completion();
            const std::uint32_t submit_epoch = epoch();
            if (!ready()) {
                return fail(EPIPE);
            }

            in_flight_ = true;
            in_flight_epoch_ = submit_epoch;
            const platform::ConsoleTransmitResult result =
                transport_.transmit(tx_staging_, length);
            if (result == platform::ConsoleTransmitResult::Accepted) {
                if (wait_completion(started, timeout_ms)) {
                    in_flight_ = false;
                    if (epoch() != submit_epoch) {
                        drain_completion();
                        return fail(EPIPE);
                    }
                    return static_cast<int>(length);
                }
                if (epoch() != submit_epoch) {
                    in_flight_ = false;
                    drain_completion();
                    return fail(EPIPE);
                }
                return fail(ETIMEDOUT);
            }

            in_flight_ = false;
            if (epoch() != submit_epoch || !ready()) {
                drain_completion();
                return fail(EPIPE);
            }
            if (result == platform::ConsoleTransmitResult::Failed) {
                return fail(EIO);
            }

            const std::uint32_t remaining =
                remaining_ms(started, timeout_ms);
            if (remaining == 0U) {
                return fail(ETIMEDOUT);
            }
            tasks_.delay(platform::Timeout::from_ms(1U));
            if (remaining_ms(started, timeout_ms) == 0U) {
                return fail(ETIMEDOUT);
            }
        }
    }

    std::size_t read(std::uint8_t *data,
                     std::size_t capacity) noexcept override
    {
        if (data == nullptr || capacity == 0U) {
            return 0U;
        }
        std::size_t count = 0U;
        while (count < capacity && read_byte(data[count])) {
            ++count;
        }
        return count;
    }

    bool read_byte(std::uint8_t &byte) noexcept override
    {
        if (!initialized()) {
            return false;
        }
        std::uint32_t tail =
            __atomic_load_n(&rx_tail_, __ATOMIC_RELAXED);
        const std::uint32_t head =
            __atomic_load_n(&rx_head_, __ATOMIC_ACQUIRE);
        if (tail == head) {
            return false;
        }
        byte = rx_ring_[tail & (kRxCapacity - 1U)];
        __atomic_store_n(&rx_tail_, tail + 1U, __ATOMIC_RELEASE);
        return true;
    }

    std::size_t available() const noexcept override
    {
        if (!initialized()) {
            return 0U;
        }
        const std::uint32_t head =
            __atomic_load_n(&rx_head_, __ATOMIC_ACQUIRE);
        const std::uint32_t tail =
            __atomic_load_n(&rx_tail_, __ATOMIC_RELAXED);
        const std::uint32_t count = head - tail;
        return count > kRxCapacity ? kRxCapacity
                                   : static_cast<std::size_t>(count);
    }

    std::uint32_t overflow_count() const noexcept override
    {
        if (!initialized()) {
            return 0U;
        }
        return __atomic_load_n(&rx_overflows_, __ATOMIC_ACQUIRE);
    }

    void receive_from_isr(const std::uint8_t *data,
                          std::size_t length) noexcept override
    {
        if (!initialized() || data == nullptr || length == 0U) {
            return;
        }
        std::uint32_t head =
            __atomic_load_n(&rx_head_, __ATOMIC_RELAXED);
        const std::uint32_t tail =
            __atomic_load_n(&rx_tail_, __ATOMIC_ACQUIRE);
        std::size_t accepted = 0U;
        while (accepted < length && head - tail < kRxCapacity) {
            rx_ring_[head & (kRxCapacity - 1U)] = data[accepted++];
            ++head;
        }
        __atomic_store_n(&rx_head_, head, __ATOMIC_RELEASE);
        if (accepted < length) {
            (void)__atomic_add_fetch(
                &rx_overflows_,
                static_cast<std::uint32_t>(length - accepted),
                __ATOMIC_RELAXED);
        }
    }

    void transmit_complete_from_isr() noexcept override
    {
        signal_completion();
    }

    void transport_connected_from_isr() noexcept override
    {
        advance_epoch();
        __atomic_store_n(&transport_online_, true, __ATOMIC_RELEASE);
    }

    void transport_disconnected_from_isr() noexcept override
    {
        __atomic_store_n(&transport_online_, false, __ATOMIC_RELEASE);
        advance_epoch();
        signal_completion();
    }

private:
    static int fail(int error_number) noexcept
    {
        errno = error_number;
        return -1;
    }

    std::uint32_t now_ms() const noexcept
    {
        return static_cast<std::uint32_t>(clock_.now_ms());
    }

    std::uint32_t remaining_ms(std::uint32_t start,
                               std::uint32_t timeout) const noexcept
    {
        const std::uint32_t elapsed = now_ms() - start;
        return elapsed >= timeout ? 0U : timeout - elapsed;
    }

    bool wait_completion(std::uint32_t start,
                         std::uint32_t timeout) noexcept
    {
        return completion_.wait(platform::Timeout::from_ms(
            remaining_ms(start, timeout)));
    }

    void drain_completion() noexcept
    {
        while (completion_.wait(platform::Timeout::no_wait())) {
        }
    }

    void signal_completion() noexcept
    {
        if (!initialized()) {
            return;
        }
        if (execution_.in_interrupt()) {
            completion_.notify_from_isr();
        } else {
            completion_.notify();
        }
    }

    int complete_previous_transfer(std::uint32_t start,
                                   std::uint32_t timeout) noexcept
    {
        if (!in_flight_) {
            return 0;
        }
        const std::uint32_t old_epoch = in_flight_epoch_;
        if (epoch() != old_epoch) {
            in_flight_ = false;
            drain_completion();
            return 0;
        }
        if (wait_completion(start, timeout)) {
            in_flight_ = false;
            if (epoch() != old_epoch) {
                drain_completion();
                return EPIPE;
            }
            return 0;
        }
        if (epoch() != old_epoch) {
            in_flight_ = false;
            drain_completion();
            return EPIPE;
        }
        return ETIMEDOUT;
    }

    std::uint32_t epoch() const noexcept
    {
        return __atomic_load_n(&transport_epoch_, __ATOMIC_ACQUIRE);
    }

    void advance_epoch() noexcept
    {
        (void)__atomic_add_fetch(&transport_epoch_, 1U, __ATOMIC_ACQ_REL);
    }

    bool transport_online() const noexcept
    {
        return __atomic_load_n(&transport_online_, __ATOMIC_ACQUIRE);
    }

    bool initialized() const noexcept
    {
        return __atomic_load_n(&initialized_, __ATOMIC_ACQUIRE);
    }

    void set_initialized(bool value) noexcept
    {
        __atomic_store_n(&initialized_, value, __ATOMIC_RELEASE);
    }

    void reset_runtime_buffers() noexcept
    {
        __atomic_store_n(&rx_head_, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&rx_tail_, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&rx_overflows_, 0U, __ATOMIC_RELEASE);
        std::memset(rx_ring_, 0, sizeof(rx_ring_));
        std::memset(tx_staging_, 0, sizeof(tx_staging_));
    }

    platform::Synchronization &synchronization_;
    platform::TaskRuntime &tasks_;
    platform::ExecutionContext &execution_;
    platform::MonotonicClock &clock_;
    platform::ConsoleTransport &transport_;
    platform::Mutex tx_mutex_{};
    platform::Signal completion_{};
    std::uint8_t rx_ring_[kRxCapacity]{};
    std::uint8_t tx_staging_[kTxCapacity]{};
    std::uint32_t rx_head_{0U};
    std::uint32_t rx_tail_{0U};
    std::uint32_t rx_overflows_{0U};
    std::uint32_t transport_epoch_{0U};
    bool transport_online_{false};
    bool initialized_{false};
    bool stdout_unbuffered_{false};
    bool in_flight_{false};
    std::uint32_t in_flight_epoch_{0U};
};

} // namespace

platform::Console &usb_console(
    platform::Synchronization &synchronization,
    platform::TaskRuntime &tasks,
    platform::ExecutionContext &execution,
    platform::MonotonicClock &clock,
    platform::ConsoleTransport &transport) noexcept
{
    static UsbConsole instance{synchronization, tasks, execution, clock,
                               transport};
    return instance;
}

} // namespace dima::adapters

extern "C" void usb_console_receive_from_isr(const std::uint8_t *data,
                                                std::size_t length)
{
    if (auto *services = dima::platform::try_services()) {
        services->console.receive_from_isr(data, length);
    }
}

extern "C" void usb_console_tx_complete_from_isr(void)
{
    if (auto *services = dima::platform::try_services()) {
        services->console.transmit_complete_from_isr();
    }
}

extern "C" void usb_console_transport_connected(void)
{
    if (auto *services = dima::platform::try_services()) {
        services->console.transport_connected_from_isr();
    }
}

extern "C" void usb_console_transport_disconnected(void)
{
    if (auto *services = dima::platform::try_services()) {
        services->console.transport_disconnected_from_isr();
    }
}

extern "C" int _write(int file, char *data, int length)
{
    if (file != 1 && file != 2) {
        errno = EBADF;
        return -1;
    }
    if (length < 0) {
        errno = EINVAL;
        return -1;
    }
    auto *services = dima::platform::try_services();
    if (services == nullptr) {
        errno = EAGAIN;
        return -1;
    }
    return services->console.write(
        reinterpret_cast<const std::uint8_t *>(data),
        static_cast<std::size_t>(length), 100U);
}

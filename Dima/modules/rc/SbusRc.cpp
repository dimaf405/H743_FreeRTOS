/****************************************************************************
 * PX4-Autopilot v1.17.0 SbusRc receive flow adapted to the Dima platform.
 ****************************************************************************/
#include "SbusRc.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"

#include <cmath>

namespace dima::modules::rc {
namespace {

constexpr std::uint32_t kEventConfigInvalid = 0x52435301U;
constexpr std::uint32_t kEventBackendFailure = 0x52435302U;
constexpr std::uint32_t kEventFailsafe = 0x52435303U;
constexpr std::uint32_t kEventPublishFailure = 0x52435304U;

void count_delta(perf_counter_t counter, std::uint32_t current,
                 std::uint32_t &previous) noexcept
{
    const std::uint32_t delta = current - previous;
    for (std::uint32_t index = 0U; index < delta; ++index) perf_count(counter);
    previous = current;
}

} // namespace

SbusRc::SbusRc(dima::platform::SbusInput &backend) noexcept
    : px4::ScheduledWorkItem("sbus_rc", px4::wq_configurations::io), backend_(backend)
{
}

void SbusRc::notify_from_isr(void *context) noexcept
{
    if (context != nullptr) {
        (void)static_cast<SbusRc *>(context)->ScheduleNowFromISR();
    }
}

bool SbusRc::start()
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    reset_runtime_state();
    if (!rc_port_.bind() || !rc_protocol_.bind() ||
        !sbus_inverted_.bind()) {
        rc_port_.invalidate();
        rc_protocol_.invalidate();
        sbus_inverted_.invalidate();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SBUS parameters unavailable");
        return false;
    }
    if (rc_protocol_.get() != 2 || rc_port_.get() <= 0 ||
        !backend_.configure(rc_port_.get(), sbus_inverted_.get())) {
        ++stats_.start_failures;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SBUS configuration invalid");
        (void)dima::events::report(kEventConfigInvalid, dima::events::Severity::Error);
        return false;
    }
    allocate_perf_counters();
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("SBUS port=%ld inverted=%d", static_cast<long>(rc_port_.get()), sbus_inverted_.get() ? 1 : 0);
    if (!ScheduleNow()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        free_perf_counters();
        return false;
    }
    return true;
}

void SbusRc::stop()
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    backend_.stop();
    free_perf_counters();
    rc_port_.invalidate();
    rc_protocol_.invalidate();
    sbus_inverted_.invalidate();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState SbusRc::state() const { return state_; }

void SbusRc::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    if (!backend_started_) {
        const dima::platform::IsrCallback notification{
            &SbusRc::notify_from_isr, this};
        if (!backend_.start(notification)) {
            ++stats_.start_failures;
            if (!backend_fault_reported_) {
                PX4_ERR("SBUS UART/DMA start failed");
                (void)dima::events::report(kEventBackendFailure,
                                           dima::events::Severity::Error);
                backend_fault_reported_ = true;
            }
            schedule_retry();
            return;
        }
        backend_started_ = true;
        backend_fault_reported_ = false;
    }
    if (!backend_.service() || !backend_.running()) {
        ++stats_.service_failures;
        const auto backend_stats = backend_.stats();
        count_delta(uart_error_count_, backend_stats.receive_errors, last_uart_errors_);
        backend_.stop();
        backend_started_ = false;
        if (signal_locked_) PX4_WARN("SBUS signal lost after UART/DMA error");
        signal_locked_ = false;
        if (!backend_fault_reported_) {
            const std::uint32_t arguments[2]{backend_stats.receive_errors,
                                             backend_stats.recovery_failures};
            (void)dima::events::report(kEventBackendFailure,
                                       dima::events::Severity::Error, arguments, 2U);
            backend_fault_reported_ = true;
        }
        schedule_retry();
        return;
    }

    std::uint8_t buffer[kReadBufferSize]{};
    std::uint64_t arrival_timestamps_us[kReadBufferSize]{};
    bool received = false;
    for (;;) {
        const std::size_t count = backend_.read(
            buffer, arrival_timestamps_us, sizeof(buffer));
        if (count == 0U) break;
        received = true;
        for (std::size_t index = 0U; index < count; ++index) {
            perf_count(byte_count_);
            dima::rc::SbusParser::Frame frame{};
            if (!parser_.parse(arrival_timestamps_us[index], buffer[index],
                               frame)) {
                continue;
            }

            const std::uint64_t frame_arrival_us = arrival_timestamps_us[index];
            timestamp_last_signal_us_ = frame_arrival_us;
            if (!signal_locked_) {
                PX4_INFO(signal_seen_ ? "SBUS signal recovered channels=%u"
                                      : "SBUS signal locked channels=%u",
                         frame.channel_count);
                signal_locked_ = true;
                signal_seen_ = true;
            }
            publish(frame, frame_arrival_us);
        }
        const auto &parser_stats = parser_.stats();
        count_delta(invalid_frame_count_,
                    parser_stats.invalid_headers + parser_stats.invalid_footers,
                    last_invalid_frames_);
    }
    if (received) ++stats_.read_wakeups;
}

void SbusRc::reset_runtime_state() noexcept
{
    parser_.reset();
    timestamp_last_signal_us_ = 0U;
    backend_started_ = false;
    signal_locked_ = false;
    signal_seen_ = false;
    failsafe_active_ = false;
    backend_fault_reported_ = false;
    last_invalid_frames_ = 0U;
    last_uart_errors_ = 0U;
    stats_ = Stats{};
}

void SbusRc::allocate_perf_counters() noexcept
{
    if (byte_count_ == nullptr) byte_count_ = perf_alloc(PC_COUNT, "sbus:bytes");
    if (frame_count_ == nullptr) frame_count_ = perf_alloc(PC_COUNT, "sbus:frames");
    if (invalid_frame_count_ == nullptr) invalid_frame_count_ = perf_alloc(PC_COUNT, "sbus:invalid");
    if (lost_frame_count_ == nullptr) lost_frame_count_ = perf_alloc(PC_COUNT, "sbus:lost");
    if (uart_error_count_ == nullptr) uart_error_count_ = perf_alloc(PC_COUNT, "sbus:uart_err");
    if (publish_interval_ == nullptr) publish_interval_ = perf_alloc(PC_INTERVAL, "sbus:input_rc_interval");
}

void SbusRc::free_perf_counters() noexcept
{
    perf_free(byte_count_);
    perf_free(frame_count_);
    perf_free(invalid_frame_count_);
    perf_free(lost_frame_count_);
    perf_free(uart_error_count_);
    perf_free(publish_interval_);
    byte_count_ = frame_count_ = invalid_frame_count_ = nullptr;
    lost_frame_count_ = uart_error_count_ = publish_interval_ = nullptr;
}

void SbusRc::schedule_retry() noexcept
{
    if (!ScheduleDelayed(kRetryDelayUs)) state_ = dima::middleware::lifecycle::ModuleState::Error;
}

void SbusRc::publish(const dima::rc::SbusParser::Frame &frame,
                     std::uint64_t frame_arrival_us) noexcept
{
    input_rc_s message{};
    message.timestamp = frame_arrival_us;
    message.timestamp_last_signal = timestamp_last_signal_us_;
    message.channel_count = frame.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS
        ? input_rc_s::RC_INPUT_MAX_CHANNELS : frame.channel_count;
    message.rssi = -1;
    message.rc_failsafe = frame.failsafe;
    // SBUS frame-lost 只表示跳帧，不能误判为整条 RC 链路失联。
    message.rc_lost = message.channel_count == 0U;
    message.rc_lost_frame_count = static_cast<std::uint16_t>(parser_.stats().frame_lost_flags);
    message.rc_total_frame_count = static_cast<std::uint16_t>(parser_.stats().valid_frames);
    message.rc_ppm_frame_length = 0U;
    message.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
    message.link_quality = -1;
    message.rssi_dbm = NAN;
    for (std::size_t i = 0U; i < message.channel_count; ++i) message.values[i] = frame.values[i];

    perf_count(frame_count_);
    perf_count(publish_interval_);
    if (frame.frame_lost) perf_count(lost_frame_count_);
    if (frame.failsafe != failsafe_active_) {
        const std::uint32_t active = frame.failsafe ? 1U : 0U;
        if (frame.failsafe) PX4_WARN("SBUS receiver failsafe");
        else PX4_INFO("SBUS receiver failsafe cleared");
        (void)dima::events::report(kEventFailsafe,
                                   frame.failsafe ? dima::events::Severity::Warning
                                                  : dima::events::Severity::Info,
                                   &active, 1U);
        failsafe_active_ = frame.failsafe;
    }
    if (input_rc_pub_.publish(message)) {
        ++stats_.publications;
    } else {
        (void)dima::events::report(kEventPublishFailure,
                                   dima::events::Severity::Warning);
    }
}

} // namespace dima::modules::rc

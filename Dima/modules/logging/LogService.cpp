#include "LogService.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "platform/api/Time.hpp"

namespace dima::modules::logging {
namespace {

constexpr std::size_t kMaxEventsPerRun = 4U;
constexpr std::size_t kMaxLogBytesPerRun = 512U;

std::size_t usb_service_write(void *context, const std::uint8_t *data,
                              std::size_t length) noexcept
{
    if (context == nullptr) {
        return 0U;
    }
    auto &console = *static_cast<dima::platform::Console *>(context);
    const int written = console.write(data, length, 2U);
    return written > 0 ? static_cast<std::size_t>(written) : 0U;
}

dima::logging::Level event_level(std::uint8_t severity) noexcept
{
    using dima::events::Severity;
    using dima::logging::Level;

    switch (static_cast<Severity>(severity)) {
    case Severity::Debug:
        return Level::Debug;
    case Severity::Info:
        return Level::Info;
    case Severity::Warning:
        return Level::Warning;
    case Severity::Error:
        return Level::Error;
    case Severity::Critical:
        return Level::Panic;
    }
    return Level::Error;
}

void enqueue_structured_events() noexcept
{
    for (std::size_t count = 0U; count < kMaxEventsPerRun; ++count) {
        dima::events::DimaEvent event{};
        if (!dima::events::pop(event)) {
            break;
        }

        (void)dima::logging::writef(
            event_level(event.severity),
            "event ts_us=%llu id=0x%08lx severity=%u argc=%u "
            "args=0x%08lx,0x%08lx,0x%08lx,0x%08lx",
            static_cast<unsigned long long>(event.timestamp),
            static_cast<unsigned long>(event.id),
            static_cast<unsigned int>(event.severity),
            static_cast<unsigned int>(event.argument_count),
            static_cast<unsigned long>(event.arguments[0]),
            static_cast<unsigned long>(event.arguments[1]),
            static_cast<unsigned long>(event.arguments[2]),
            static_cast<unsigned long>(event.arguments[3]));
    }
}

} // namespace

LogService::LogService(dima::platform::Console &console) noexcept
    : ScheduledWorkItem("dima_log", px4::wq_configurations::lp_default),
      console_(console)
{
}

bool LogService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    reset_debug_state();
    if (!ScheduleOnInterval(kFlushIntervalUs, kFlushIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("USB debug logging ready");
    return true;
}

void LogService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    reset_debug_state();
}

dima::middleware::lifecycle::ModuleState LogService::state() const noexcept
{
    return state_;
}

void LogService::reset_debug_state() noexcept
{
    last_sbus_sample_timestamp_us_ = 0U;
    last_sbus_output_time_us_ = 0U;
    sbus_sample_pending_ = false;
}

void LogService::enqueue_sbus_data(std::uint64_t now_us) noexcept
{
    using dima::logging::Level;
    using dima::logging::Source;
    using dima::logging::config::kSbus;

    if constexpr (!kSbus.data_to_usb) {
        (void)now_us;
        return;
    }

    if (input_rc_subscription_.update()) {
        const input_rc_s &latest = input_rc_subscription_.get();
        sbus_sample_pending_ = latest.timestamp != 0U &&
                               latest.timestamp >
                                   last_sbus_sample_timestamp_us_;
    }
    if (!sbus_sample_pending_) {
        return;
    }

    constexpr std::uint64_t interval_us =
        static_cast<std::uint64_t>(kSbus.data_period_ms) * 1000ULL;
    if (last_sbus_output_time_us_ != 0U &&
        now_us - last_sbus_output_time_us_ < interval_us) {
        return;
    }

    const input_rc_s &input = input_rc_subscription_.get();
    (void)dima::logging::write_module(
        Source::Sbus, Level::Debug, "sbus",
        "ts=%llu ch=%u values=[%u,%u,%u,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%u,%u,%u,%u,%u,%u,%u] failsafe=%u lost=%u "
        "total=%u lost_frames=%u",
        static_cast<unsigned long long>(input.timestamp),
        static_cast<unsigned int>(input.channel_count),
        static_cast<unsigned int>(input.values[0]),
        static_cast<unsigned int>(input.values[1]),
        static_cast<unsigned int>(input.values[2]),
        static_cast<unsigned int>(input.values[3]),
        static_cast<unsigned int>(input.values[4]),
        static_cast<unsigned int>(input.values[5]),
        static_cast<unsigned int>(input.values[6]),
        static_cast<unsigned int>(input.values[7]),
        static_cast<unsigned int>(input.values[8]),
        static_cast<unsigned int>(input.values[9]),
        static_cast<unsigned int>(input.values[10]),
        static_cast<unsigned int>(input.values[11]),
        static_cast<unsigned int>(input.values[12]),
        static_cast<unsigned int>(input.values[13]),
        static_cast<unsigned int>(input.values[14]),
        static_cast<unsigned int>(input.values[15]),
        static_cast<unsigned int>(input.values[16]),
        static_cast<unsigned int>(input.values[17]),
        input.rc_failsafe ? 1U : 0U, input.rc_lost ? 1U : 0U,
        static_cast<unsigned int>(input.rc_total_frame_count),
        static_cast<unsigned int>(input.rc_lost_frame_count));
    last_sbus_sample_timestamp_us_ = input.timestamp;
    last_sbus_output_time_us_ = now_us;
    sbus_sample_pending_ = false;
}

void LogService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) {
        return;
    }
    // USB 未连接时保留环形内容；生产者仍保持非阻塞，满后覆盖最旧数据。
    if (!console_.ready()) {
        return;
    }
    enqueue_structured_events();
    enqueue_sbus_data(hrt_absolute_time());
    const dima::logging::ServiceWriter writer{&console_, &usb_service_write};
    (void)dima::logging::service_flush(writer, kMaxLogBytesPerRun);
}

} // namespace dima::modules::logging

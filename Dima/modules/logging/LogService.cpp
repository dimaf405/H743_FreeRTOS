#include "LogService.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"

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
}

dima::middleware::lifecycle::ModuleState LogService::state() const noexcept
{
    return state_;
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
    const dima::logging::ServiceWriter writer{&console_, &usb_service_write};
    (void)dima::logging::service_flush(writer, kMaxLogBytesPerRun);
}

} // namespace dima::modules::logging

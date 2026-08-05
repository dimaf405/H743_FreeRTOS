#include "LogService.hpp"

#include "logging/logging.hpp"

namespace dima::modules::logging {
namespace {

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
    dima::logging::reset();
    if (!ScheduleOnInterval(kFlushIntervalUs, kFlushIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    return true;
}

void LogService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    dima::logging::reset();
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
    const dima::logging::ServiceWriter writer{&console_, &usb_service_write};
    (void)dima::logging::service_flush(writer, 256U);
}

} // namespace dima::modules::logging

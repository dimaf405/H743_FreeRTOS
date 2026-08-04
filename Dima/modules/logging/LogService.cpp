#include "LogService.hpp"

#include "usb_console/usb_console.h"
#include "logging/logging.hpp"

namespace dima::modules::logging {
namespace {

std::size_t usb_service_write(void *, const std::uint8_t *data,
                              std::size_t length) noexcept
{
    const int written = usb_console_write(data, length, 2U);
    return written > 0 ? static_cast<std::size_t>(written) : 0U;
}

} // namespace

LogService::LogService() noexcept
    : ScheduledWorkItem("dima_log", px4::wq_configurations::lp_default)
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
    if (!usb_console_ready()) {
        return;
    }
    const dima::logging::ServiceWriter writer{nullptr, &usb_service_write};
    (void)dima::logging::service_flush(writer, 256U);
}

} // namespace dima::modules::logging

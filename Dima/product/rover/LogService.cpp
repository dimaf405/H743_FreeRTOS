#include "LogService.hpp"

#include "usb_console/usb_console.h"
#include "logging/logging.hpp"

namespace dima::product::rover {
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
    if (started_) {
        return true;
    }
    started_ = ScheduleOnInterval(kFlushIntervalUs, kFlushIntervalUs);
    return started_;
}

void LogService::stop() noexcept
{
    ScheduleClear();
    started_ = false;
}

void LogService::Run()
{
    // USB 未连接时保留环形内容；生产者仍保持非阻塞，满后覆盖最旧数据。
    if (!usb_console_ready()) {
        return;
    }
    const dima::logging::ServiceWriter writer{nullptr, &usb_service_write};
    (void)dima::logging::service_flush(writer, 256U);
}

} // namespace dima::product::rover

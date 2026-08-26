#include "LogService.hpp"

#include "events/events.hpp"
#include "api/Time.hpp"

#include <algorithm>
#include <cstring>

namespace dima::modules::logging {
namespace {

constexpr std::size_t kMaxEventsPerRun = 4U;

/* 每轮最多搬运四个结构化事件，给同一低优先级 work queue 上的参数/存储任务
 * 留出执行机会。Level -> MAV_SEVERITY 使用线协议固定数值，数值越小越严重。 */
std::uint8_t mav_severity(dima::logging::Level level) noexcept
{
    using dima::logging::Level;
    switch (level) {
    case Level::Debug:   return 7U;   /* MAV_SEVERITY_DEBUG */
    case Level::Info:    return 6U;   /* MAV_SEVERITY_INFO */
    case Level::Warning: return 4U;   /* MAV_SEVERITY_WARNING */
    case Level::Error:   return 3U;   /* MAV_SEVERITY_ERROR */
    case Level::Panic:   return 0U;   /* MAV_SEVERITY_EMERGENCY */
    case Level::Off:     return 6U;   /* filtered before the sink */
    }
    return 3U;
}

dima::logging::Level event_level(std::uint8_t severity) noexcept
{
    using dima::events::Severity;
    using dima::logging::Level;

    // 未知事件级别按 Error fail-closed，避免损坏的 severity 被降级为普通信息。
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
    // pop 是有界队列消费；单轮上限只延后剩余事件，不丢弃也不在 work queue 中
    // 无界清空积压。
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

uORB::Publication<mavlink_log_s> LogService::mavlink_log_publication_{
    ORB_ID(mavlink_log)};

LogService::LogService() noexcept
    : ScheduledWorkItem("dima_log", px4::wq_configurations::lp_default)
{
}

bool LogService::initialize() noexcept
{
    if (!initialized_) {
        // 全局 structured sink 只绑定本服务的静态桥接函数，不捕获对象或堆内存。
        dima::logging::set_structured_sink(nullptr, &LogService::structured_sink);
        initialized_ = true;
    }
    return true;
}

void LogService::shutdown() noexcept
{
    // 先停止 ScheduledWorkItem，再解除全局 sink，封住停机中回调已释放状态的竞态。
    stop();
    dima::logging::set_structured_sink(nullptr, nullptr);
    initialized_ = false;
}

bool LogService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!initialized_ || !ScheduleEnable()) {
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
    PX4_INFO("Structured logging ready");
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

bool LogService::structured_sink(void *context, dima::logging::Level level,
                                 const char *text, std::size_t length) noexcept
{
    (void)context;
    if (text == nullptr || length == 0U) {
        return false;
    }

    // mavlink_log Topic 保存以 '\0' 结尾的定长文本；超长日志只截断 payload，
    // severity 与本次单调时间仍完整保留，不尝试动态分配扩容。
    mavlink_log_s record{};
    record.timestamp = hrt_absolute_time();
    record.severity = mav_severity(level);
    const std::size_t copy =
        std::min(length, static_cast<std::size_t>(mavlink_log_s::TEXT_LEN - 1U));
    std::memcpy(record.text, text, copy);
    record.text[copy] = '\0';
    return mavlink_log_publication_.publish(record);
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

    // uORB 只保留最新样本：限流期间持续覆盖 pending 内容，最终输出最新一帧，
    // 而不是把高频 SBUS 帧排成日志积压。
    if (input_rc_subscription_.update()) {
        const input_rc_s &latest = input_rc_subscription_.get();
        sbus_sample_pending_ = latest.timestamp != 0U &&
                               latest.timestamp >
                                   last_sbus_sample_timestamp_us_;
    }
    if (!sbus_sample_pending_) {
        return;
    }

    // 配置单位为 ms，转换为 us 后用原始 now_us 做单调节流；0 表示每轮均可输出。
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
    // 先释放最多四个高价值事件，再处理可丢帧的 SBUS 调试样本。
    enqueue_structured_events();
    enqueue_sbus_data(hrt_absolute_time());
}

} // namespace dima::modules::logging

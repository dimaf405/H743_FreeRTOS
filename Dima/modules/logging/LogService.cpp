#include "LogService.hpp"

#include "api/BoardIdentity.hpp"
#include "api/Time.hpp"
#include "events/events.hpp"
#include "parameters/param.h"

#include <algorithm>
#include <cstring>

namespace dima::modules::logging {
namespace {

constexpr std::size_t kMaxEventsPerRun = 4U;

/* 每轮最多搬运四个结构化事件，给同一低优先级 work queue 上的
 * MAVLink 和校准任务留出执行机会。Level -> MAV_SEVERITY 使用线协议固定数值，
 * 数值越小越严重；SD/FatFs 已隔离到 storage 队列。 */
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

LogService::LogService(dima::platform::LogFileStore &log_files) noexcept
    : ScheduledWorkItem("dima_log", px4::wq_configurations::lp_default),
      sd_writer_(log_files)
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

std::int32_t LogService::load_integer_parameter(
    const generated::IntegerParameterContract &contract) noexcept
{
    const param_t handle = param_handle(contract.parameter);
    param_set_used(handle);
    std::int32_t value = contract.default_value;
    const bool loaded = param_get(handle, &value) == 0;
    if (loaded && value >= contract.minimum && value <= contract.maximum) {
        return value;
    }

    /* 参数存储损坏或越界时回退到由 module_logger.yaml 生成并由 Logger
     * 合同交叉验证的编译默认值。Logger 是非关键能力，错误只告警，不能把
     * Commander、控制或 MAVLink 服务拖入启动失败。 */
    std::int32_t generated_default = contract.default_value;
    if (param_get_system_default_value(handle, &generated_default) != 0 ||
        generated_default < contract.minimum ||
        generated_default > contract.maximum) {
        generated_default = contract.default_value;
    }
    const char *name = param_name(handle);
    PX4_WARN("Invalid %s; using generated default %ld",
             name != nullptr ? name : "Logger parameter",
             static_cast<long>(generated_default));
    return generated_default;
}

void LogService::load_logger_configuration(
    SdLogWriter::Configuration &configuration) noexcept
{
    const std::int32_t mode =
        load_integer_parameter(generated::kModeParameter);
    const std::int32_t profile =
        load_integer_parameter(generated::kProfileParameter);
    const std::int32_t directories =
        load_integer_parameter(generated::kDirectoriesParameter);
    log_mode_ = static_cast<generated::LogMode>(mode);
    configuration.profile = static_cast<std::uint8_t>(profile);
    configuration.maximum_directories =
        static_cast<std::uint16_t>(directories);
    configuration.hardware_uid = dima::platform::board_hardware_uid();
    if (configuration.hardware_uid == 0U) {
        PX4_WARN("Hardware UID is zero; ULog sys_uuid will be all zeros");
    }
}

void LogService::set_sd_recording_intent(bool enabled) noexcept
{
    if (sd_recording_intent_ == enabled) {
        return;
    }
    sd_recording_intent_ = enabled;
    if (sd_writer_available_) {
        sd_writer_.set_recording_intent(enabled);
    }
}

void LogService::apply_initial_mode() noexcept
{
    armed_state_ = false;
    sd_recording_intent_ = false;

    /* Mode 1/2 的记录意图从启动即成立；Mode 0/3 等待真实 armed=true。
     * 这里只设置 SD 会话意图，LogService 自身始终运行以继续转发结构化文本。 */
    if (log_mode_ == generated::LogMode::BootUntilFirstPostArmDisarm ||
        log_mode_ == generated::LogMode::BootUntilShutdown) {
        set_sd_recording_intent(true);
    }
}

void LogService::process_armed_state(bool armed) noexcept
{
    // 启动时 armed_state_=false，首个 Armed 样本自然形成上升沿；
    // 下降沿本身就证明观察过 Armed，无需再保存一份历史标志。
    const bool rising = !armed_state_ && armed;
    const bool falling = armed_state_ && !armed;
    armed_state_ = armed;

    switch (log_mode_) {
    case generated::LogMode::ArmedSessions:
        if (rising) {
            set_sd_recording_intent(true);
        } else if (falling) {
            set_sd_recording_intent(false);
        }
        break;

    case generated::LogMode::BootUntilFirstPostArmDisarm:
        /* 上电后即记录，首次下降沿停止；本模式没有重新开启入口，
         * 后续下降沿由 set_sd_recording_intent 的幂等检查直接跳过。 */
        if (falling) {
            set_sd_recording_intent(false);
        }
        break;

    case generated::LogMode::BootUntilShutdown:
        break;

    case generated::LogMode::FirstArmUntilShutdown:
        if (rising) {
            set_sd_recording_intent(true);
        }
        break;
    }
}

bool LogService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) {
        return true;
    }
    if (!initialized_ || !ScheduleEnable()) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        return false;
    }

    SdLogWriter::Configuration configuration{};
    load_logger_configuration(configuration);
    sd_writer_available_ = sd_writer_.start(configuration);
    if (!sd_writer_available_) {
        /* SD ULog 初始化失败不能关闭 structured sink；QGC STATUSTEXT 和 Event
         * 仍由本服务继续承载，故模块保持 Running 并给出明确降级告警。 */
        PX4_WARN("SD ULog unavailable; structured logging remains active");
    }
    armed_callback_registered_ =
        actuator_armed_subscription_.registerCallback();
    if (!armed_callback_registered_) {
        PX4_WARN("Logger armed callback unavailable; using 20 ms polling");
    }
    reset_debug_state();
    apply_initial_mode();
    if (!ScheduleOnInterval(kFlushIntervalUs, kFlushIntervalUs)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        if (armed_callback_registered_) {
            actuator_armed_subscription_.unregisterCallback();
            armed_callback_registered_ = false;
        }
        ScheduleCancelAndDrain();
        sd_writer_.stop();
        return false;
    }
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    PX4_INFO("Structured logging ready");
    return true;
}

void LogService::stop() noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    if (armed_callback_registered_) {
        actuator_armed_subscription_.unregisterCallback();
        armed_callback_registered_ = false;
    }
    ScheduleCancelAndDrain();
    sd_writer_.stop();
    sd_writer_available_ = false;
    sd_recording_intent_ = false;
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
    // 文本容量直接从 PX4 生成结构推导，不复制 schema 中的数组长度。
    const std::size_t copy =
        std::min(length, sizeof(record.text) - 1U);
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
    actuator_armed_s armed{};
    for (std::size_t count = 0U;
         count < 4U && actuator_armed_subscription_.copy(&armed); ++count) {
        process_armed_state(armed.armed);
    }
    // 先处理会话安全边沿，再释放高价值事件和可丢帧的 SBUS 调试样本。
    enqueue_structured_events();
    enqueue_sbus_data(hrt_absolute_time());
}

} // namespace dima::modules::logging

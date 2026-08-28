/****************************************************************************
 * PX4-Autopilot v1.17.0 SbusRc receive flow adapted to the Dima platform.
 ****************************************************************************/
#define MODULE_NAME "sbus"
#include "SbusRc.hpp"

#include "events/events.hpp"
#include "logging/logging.hpp"
#include "api/Time.hpp"

#include <cmath>

namespace dima::drivers::rc {
namespace {

constexpr std::uint32_t kEventConfigInvalid = 0x52435301U;
constexpr std::uint32_t kEventBackendFailure = 0x52435302U;
constexpr std::uint32_t kEventFailsafe = 0x52435303U;
constexpr std::uint32_t kEventPublishFailure = 0x52435304U;
constexpr std::uint32_t kEventBackendLineError = 0x52435305U;

constexpr dima::platform::SerialLineConfiguration
sbus_line_configuration() noexcept
{
    // SBUS 物理合同为 100000 bit/s、8E2、RX-only、反相。data_bits=8 表示有效
    // 数据位，平台 word-length 会另计偶校验位；RX 下拉用于反相后的空闲电平。
    dima::platform::SerialLineConfiguration configuration{};
    configuration.baudrate = 100000U;
    configuration.data_bits = 8U;
    configuration.parity = dima::platform::SerialParity::Even;
    configuration.stop_bits = dima::platform::SerialStopBits::Two;
    configuration.rx_pull = dima::platform::SerialRxPull::Down;
    configuration.rx_enabled = true;
    configuration.tx_enabled = false;
    configuration.rx_inverted = true;
    return configuration;
}

constexpr std::uint32_t kRecoverableLineErrorMask =
    dima::platform::SerialInputErrorParity |
    dima::platform::SerialInputErrorNoise |
    dima::platform::SerialInputErrorFraming;

constexpr bool is_recoverable_line_error(
    const dima::platform::TimestampedSerialInputStats &stats) noexcept
{
    // 只有“存在错误、无 CPU ring 丢字节、无恢复失败、flags 非空且完全属于
    // PE/NE/FE”才是可恢复线路噪声。ORE/DMA/RTO/Unknown 或组合硬错均 fail-closed。
    return stats.receive_errors != 0U && stats.dropped_bytes == 0U &&
           stats.recovery_failures == 0U &&
           (stats.receive_error_flags & kRecoverableLineErrorMask) != 0U &&
           (stats.receive_error_flags & ~kRecoverableLineErrorMask) == 0U;
}

constexpr bool should_publish_frame(bool signal_locked,
                                    bool receiver_failsafe) noexcept
{
    // 未重新锁定的普通恢复帧不发布；接收机显式 failsafe 无条件立即发布。
    return signal_locked || receiver_failsafe;
}

static_assert(is_recoverable_line_error({0U, 0U, 1U, 0U,
                                         dima::platform::SerialInputErrorNoise}));
static_assert(!is_recoverable_line_error({0U, 1U, 1U, 0U,
                                          dima::platform::SerialInputErrorNoise}));
static_assert(!is_recoverable_line_error({0U, 0U, 1U, 1U,
                                          dima::platform::SerialInputErrorNoise}));
static_assert(!is_recoverable_line_error({0U, 0U, 1U, 0U,
                                          dima::platform::SerialInputErrorDma}));
static_assert(!should_publish_frame(false, false));
static_assert(should_publish_frame(true, false));
static_assert(should_publish_frame(false, true));

void count_delta(perf_counter_t counter, std::uint32_t current,
                  std::uint32_t &previous) noexcept
{
    // 后端计数在同一生命周期内单调累计；perf_counter 只补记本轮增量，避免
    // 每次 Run 重复计算历史 UART 错误。
    const std::uint32_t delta = current - previous;
    for (std::uint32_t index = 0U; index < delta; ++index) perf_count(counter);
    previous = current;
}

void report_backend_failure(
    const dima::platform::TimestampedSerialInputStats &stats) noexcept
{
    /* count, CPU Ring drops, restart failures, accumulated backend flags */
    const std::uint32_t arguments[4]{
        stats.receive_errors,
        stats.dropped_bytes,
        stats.recovery_failures,
        stats.receive_error_flags,
    };
    const std::uint32_t flags = stats.receive_error_flags;
    PX4_ERR("UART/DMA fault errors=%lu ring_drop=%lu recovery_fail=%lu "
            "flags=0x%08lx pe=%u ne=%u fe=%u ore=%u dma=%u rto=%u",
            static_cast<unsigned long>(stats.receive_errors),
            static_cast<unsigned long>(stats.dropped_bytes),
            static_cast<unsigned long>(stats.recovery_failures),
            static_cast<unsigned long>(flags),
            (flags & dima::platform::SerialInputErrorParity) != 0U ? 1U : 0U,
            (flags & dima::platform::SerialInputErrorNoise) != 0U ? 1U : 0U,
            (flags & dima::platform::SerialInputErrorFraming) != 0U ? 1U : 0U,
            (flags & dima::platform::SerialInputErrorOverrun) != 0U ? 1U : 0U,
            (flags & dima::platform::SerialInputErrorDma) != 0U ? 1U : 0U,
            (flags & dima::platform::SerialInputErrorTimeout) != 0U ? 1U : 0U);
    (void)dima::events::report(kEventBackendFailure,
                               dima::events::Severity::Error,
                               arguments, 4U);
}

void report_backend_line_error(
    std::int32_t port,
    const dima::platform::TimestampedSerialInputStats &stats) noexcept
{
    const std::uint32_t arguments[4]{
        stats.receive_errors,
        stats.dropped_bytes,
        stats.recovery_failures,
        stats.receive_error_flags,
    };
    const std::uint32_t flags = stats.receive_error_flags;
    PX4_WARN("SERIAL%ld SBUS line error; DMA restarting errors=%lu "
             "flags=0x%08lx pe=%u ne=%u fe=%u",
             static_cast<long>(port),
             static_cast<unsigned long>(stats.receive_errors),
             static_cast<unsigned long>(flags),
             (flags & dima::platform::SerialInputErrorParity) != 0U ? 1U : 0U,
             (flags & dima::platform::SerialInputErrorNoise) != 0U ? 1U : 0U,
             (flags & dima::platform::SerialInputErrorFraming) != 0U ? 1U : 0U);
    (void)dima::events::report(kEventBackendLineError,
                               dima::events::Severity::Warning,
                               arguments, 4U);
}

} // namespace

SbusRc::SbusRc(
    dima::platform::TimestampedSerialInput &backend,
    dima::lib::serial::SerialPortAssignments &serial_assignments) noexcept
    : px4::ScheduledWorkItem("sbus_rc", px4::wq_configurations::io),
      backend_(backend), serial_assignments_(serial_assignments)
{
}

void SbusRc::notify_from_isr(void *context) noexcept
{
    // DMA/串口 ISR 只请求立即调度，不在中断内解析 25 B 帧或发布 uORB。
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
    if (!rc_protocol_.bind() || !rc_loss_timeout_.bind()) {
        invalidate_parameters();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SBUS parameters unavailable");
        return false;
    }
    // RC_INPUT_PROTO=0 是明确禁用，不占用串口；=2 才启用 SBUS。端口所有权
    // 由 SerialConfig 从 Dima 生成参数目录解析后提供，驱动不维护串口参数清单。
    const std::int32_t protocol = rc_protocol_.get();
    const std::int32_t port = serial_assignments_.rc_input_port();
    const float loss_timeout_s = rc_loss_timeout_.get();
    if (protocol == 0) {
        state_ = dima::middleware::lifecycle::ModuleState::Running;
        DIMA_LOG_SOURCE(dima::logging::Source::Sbus,
                        dima::logging::Level::Info,
                        "disabled protocol=%ld; serial ports remain normal",
                        static_cast<long>(protocol));
        return true;
    }
    if (protocol != 2 || port <= 0 ||
        !std::isfinite(loss_timeout_s) || loss_timeout_s < 0.1F ||
        loss_timeout_s > 35.0F ||
        !backend_.configure(port, sbus_line_configuration())) {
        ++stats_.start_failures;
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        ScheduleCancelAndDrain();
        PX4_ERR("SBUS configuration invalid protocol=%ld port=%ld",
                static_cast<long>(protocol), static_cast<long>(port));
        (void)dima::events::report(kEventConfigInvalid, dima::events::Severity::Error);
        return false;
    }
    // COM_RC_LOSS_T 参数层单位为秒，内部调度转换为微秒；允许范围 0.1..35 s。
    signal_loss_timeout_us_ = static_cast<std::uint64_t>(
        static_cast<double>(loss_timeout_s) * 1000000.0);
    allocate_perf_counters();
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    DIMA_LOG_SOURCE(dima::logging::Source::Sbus,
                    dima::logging::Level::Info,
                    "SERIAL%ld protocol=SBUS 100000 8E2 rxinv=auto",
                    static_cast<long>(port));
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
    // 先排空 WorkQueue 回调，再让后端恢复普通 UART 配置；恢复失败意味着串口
    // 资源所有权不确定，模块必须进入 Error 而不能假装已经释放。
    const bool was_started = backend_started_;
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
    ScheduleCancelAndDrain();
    const bool restored = backend_.stop();
    if (!restored) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        PX4_ERR("SBUS release failed; UART normal configuration not restored");
        report_backend_failure(backend_.stats());
    } else if (was_started) {
        DIMA_LOG_SOURCE(dima::logging::Source::Sbus,
                        dima::logging::Level::Info,
                        "released; UART normal configuration restored");
    }
    free_perf_counters();
    invalidate_parameters();
    reset_runtime_state();
}

dima::middleware::lifecycle::ModuleState SbusRc::state() const { return state_; }

void SbusRc::invalidate_parameters() noexcept
{
    rc_protocol_.invalidate();
    rc_loss_timeout_.invalidate();
}

void SbusRc::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    // backend start/restart 与协议锁定分离：DMA 启动成功不代表 SBUS 信号恢复，
    // 必须等解析器收到连续健康帧。
    if (!backend_started_) {
        const dima::platform::IsrCallback notification{
            &SbusRc::notify_from_isr, this};
        if (!backend_.start(notification)) {
            ++stats_.start_failures;
            if (!backend_fault_reported_) {
                PX4_ERR("SBUS UART/DMA start failed");
                report_backend_failure(backend_.stats());
                backend_fault_reported_ = true;
            }
            schedule_retry();
            return;
        }
        backend_started_ = true;
        const auto backend_stats = backend_.stats();
        last_backend_faults_ = backend_stats.receive_errors +
                               backend_stats.dropped_bytes;
    }
    // service/running 失败先抓取统计、清协议半帧、停止并恢复普通 UART，然后按
    // 故障等级决定是否立即发布 rc_lost 以及使用 1 ms/100 ms 退避。
    if (!backend_.service() || !backend_.running()) {
        ++stats_.service_failures;
        const auto fault_stats = backend_.stats();
        count_delta(uart_error_count_,
                    fault_stats.receive_errors +
                        fault_stats.dropped_bytes,
                    last_backend_faults_);
        parser_.reset();
        const bool was_signal_locked = signal_locked_;
        const bool restored = backend_.stop();
        backend_started_ = false;
        signal_locked_ = false;
        consecutive_healthy_frames_ = 0U;
        if (!restored) {
            state_ = dima::middleware::lifecycle::ModuleState::Error;
            PX4_ERR("SBUS fault rollback failed; UART not restored");
            report_backend_failure(backend_.stats());
            if (!publish_backend_loss(hrt_absolute_time())) {
                PX4_ERR("SBUS stopped: RC loss publication failed");
            }
            return;
        }

        const auto stopped_stats = backend_.stats();
        const bool recoverable_line_error =
            is_recoverable_line_error(stopped_stats);
        if (recoverable_line_error) {
            /* A PE/NE/FE invalidates the current byte, not the whole RC link.
             * HAL aborts DMA, so restart it and withhold untrusted recovery
             * frames. RCUpdate owns COM_RC_LOSS_T from the last published
             * valid frame and declares loss only if recovery misses it. */
            // 单次 PE/NE/FE 只使当前字节/半帧不可信，HAL 会中止 DMA，因此立即
            // 重启并扣留前两帧。RCUpdate 从最后一份已发布帧按 COM_RC_LOSS_T
            // 判持续失联；这里不能把瞬态噪声直接升级成整条 RC 链路丢失。
            if (!backend_line_error_reported_) {
                report_backend_line_error(serial_assignments_.rc_input_port(),
                                          stopped_stats);
                backend_line_error_reported_ = true;
            }
        } else {
            // Ring 溢出、DMA/RTO/ORE、恢复失败或未知错误属于硬故障：立即发布
            // rc_lost，再用 100 ms 退避，保证执行器安全路径不会等待重锁。
            if (was_signal_locked) {
                PX4_WARN("SBUS signal lost after UART/DMA error");
            }
            if (!backend_fault_reported_) {
                report_backend_failure(stopped_stats);
                backend_fault_reported_ = true;
            }
            if (!publish_backend_loss(hrt_absolute_time())) {
                state_ = dima::middleware::lifecycle::ModuleState::Error;
                PX4_ERR("SBUS stopped: RC loss publication failed");
                return;
            }
        }
        schedule_retry(recoverable_line_error ? kLineErrorRetryDelayUs
                                              : kRetryDelayUs);
        return;
    }

    std::uint8_t buffer[kReadBufferSize]{};
    std::uint64_t arrival_timestamps_us[kReadBufferSize]{};
    bool received = false;
    // 每次最多从后端取 64 个带时间戳字节，但循环排空 CPU ring。空时先安排
    // 精确的信号超时唤醒，再二次读取以关闭 ISR 与 ScheduleAt 的竞态窗口。
    for (;;) {
        std::size_t count = backend_.read(
            buffer, arrival_timestamps_us, sizeof(buffer));
        if (count == 0U) {
            if (!backend_.service() || !backend_.running()) {
                if (!ScheduleNow()) {
                    fail_scheduling("SBUS fault service scheduling failed");
                }
                return;
            }
            if (!schedule_signal_timeout()) {
                fail_scheduling("SBUS signal timeout scheduling failed");
                return;
            }
            /* Close the race where an ISR filled the CPU Ring immediately
             * before the timeout request replaced its immediate wakeup. */
            // 若 ISR 恰在超时调度覆盖“立即唤醒”前写入 ring，二次 read 会当场
            // 消费该数据，避免直到 COM_RC_LOSS_T 才处理已经到达的有效帧。
            count = backend_.read(
                buffer, arrival_timestamps_us, sizeof(buffer));
            if (count == 0U) {
                if ((!backend_.service() || !backend_.running()) &&
                    !ScheduleNow()) {
                    fail_scheduling("SBUS fault rescheduling failed");
                }
                break;
            }
        }
        received = true;
        for (std::size_t index = 0U; index < count; ++index) {
            perf_count(byte_count_);
            dima::protocols::sbus::SbusParser::Frame frame{};
            const std::uint32_t dropped_before =
                parser_.stats().dropped_frames;
            if (!parser_.parse(arrival_timestamps_us[index], buffer[index],
                               frame)) {
                if (!signal_locked_ &&
                    parser_.stats().dropped_frames != dropped_before) {
                    consecutive_healthy_frames_ = 0U;
                }
                continue;
            }

            const std::uint64_t frame_arrival_us = arrival_timestamps_us[index];
            // 完整协议帧更新时间戳；显式 failsafe 立即解除普通信号锁，健康帧则
            // 连续累计到 3 帧。任何未锁定期间的坏帧都会把连续计数清零。
            timestamp_last_signal_us_ = frame_arrival_us;
            /* Peripheral start is not recovery; a valid frame is. */
            // DMA 重新启动不是恢复证据；只有成功解码的帧才能清除故障日志门限。
            backend_fault_reported_ = false;
            backend_line_error_reported_ = false;
            bool just_locked = false;
            if (frame.failsafe) {
                consecutive_healthy_frames_ = 0U;
                signal_locked_ = false;
            } else if (!signal_locked_) {
                if (consecutive_healthy_frames_ < kRequiredLockFrames) {
                    ++consecutive_healthy_frames_;
                }
                if (consecutive_healthy_frames_ == kRequiredLockFrames) {
                    signal_locked_ = true;
                    just_locked = true;
                }
            }
            if (just_locked) {
                DIMA_LOG_SOURCE(
                    dima::logging::Source::Sbus,
                    dima::logging::Level::Info,
                    signal_seen_ ? "signal recovered channels=%u"
                                 : "signal locked channels=%u",
                    frame.channel_count);
                signal_seen_ = true;
            }
            if (should_publish_frame(signal_locked_, frame.failsafe)) {
                publish(frame, frame_arrival_us);
            }
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
    consecutive_healthy_frames_ = 0U;
    failsafe_active_ = false;
    backend_fault_reported_ = false;
    backend_line_error_reported_ = false;
    last_invalid_frames_ = 0U;
    last_backend_faults_ = 0U;
    signal_loss_timeout_us_ = 500000U;
    stats_ = Stats{};
}

void SbusRc::allocate_perf_counters() noexcept
{
    // perf 句柄来自全局固定池，只在启用 SBUS 时分配；分配失败得到 nullptr，
    // perf API 按空句柄安全降级，不影响控制链启动。
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

void SbusRc::schedule_retry(std::uint32_t delay_us) noexcept
{
    if (!ScheduleDelayed(delay_us)) {
        state_ = dima::middleware::lifecycle::ModuleState::Error;
    }
}

bool SbusRc::schedule_signal_timeout() noexcept
{
    if (!signal_locked_ || timestamp_last_signal_us_ == 0U) {
        return true;
    }

    const std::uint64_t now_us = hrt_absolute_time();
    // timeout=(now-last)>=COM_RC_LOSS_T；时间戳位于未来同样 fail-closed，避免
    // 无符号下溢把时钟异常伪装成长期健康。
    const bool timestamp_invalid = timestamp_last_signal_us_ > now_us;
    const bool timed_out = timestamp_invalid ||
                           now_us - timestamp_last_signal_us_ >=
                               signal_loss_timeout_us_;
    if (timed_out) {
        DIMA_LOG_SOURCE(dima::logging::Source::Sbus,
                        dima::logging::Level::Warning,
                        "signal lost last_frame_us=%llu timeout_us=%llu",
                        static_cast<unsigned long long>(
                            timestamp_last_signal_us_),
                        static_cast<unsigned long long>(
                            signal_loss_timeout_us_));
        signal_locked_ = false;
        consecutive_healthy_frames_ = 0U;
        return true;
    }

    // 尚未超时时直接调度到绝对截止点，避免固定轮询带来的额外失联延迟。
    return ScheduleAt(timestamp_last_signal_us_ + signal_loss_timeout_us_);
}

void SbusRc::fail_scheduling(const char *reason) noexcept
{
    state_ = dima::middleware::lifecycle::ModuleState::Error;
    const bool restored = backend_.stop();
    backend_started_ = false;
    PX4_ERR("%s", reason != nullptr ? reason : "SBUS scheduling failed");
    if (!restored) {
        PX4_ERR("SBUS scheduling rollback failed; UART not restored");
        report_backend_failure(backend_.stats());
    }
}

bool SbusRc::publish_backend_loss(std::uint64_t now) noexcept
{
    // 硬后端故障发布零通道 rc_lost；RSSI/link quality 不由 SBUS 提供，用 -1/NaN
    // 明确表示未知。last_signal 保留最近可信帧，供上层计算失联持续时间。
    input_rc_s message{};
    message.timestamp = now;
    message.timestamp_last_signal = timestamp_last_signal_us_;
    message.rssi = -1;
    message.rc_lost = true;
    message.rc_lost_frame_count = static_cast<std::uint16_t>(
        parser_.stats().frame_lost_flags);
    message.rc_total_frame_count = static_cast<std::uint16_t>(
        parser_.stats().valid_frames);
    message.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SBUS;
    message.link_quality = -1;
    message.rssi_dbm = NAN;

    if (input_rc_pub_.publish(message)) {
        ++stats_.publications;
        return true;
    }
    (void)dima::events::report(kEventPublishFailure,
                               dima::events::Severity::Warning);
    return false;
}

void SbusRc::publish(const dima::protocols::sbus::SbusParser::Frame &frame,
                     std::uint64_t frame_arrival_us) noexcept
{
    input_rc_s message{};
    message.timestamp = frame_arrival_us;
    message.timestamp_last_signal = timestamp_last_signal_us_;
    // 发布前再次钳位到消息 ABI 的最大通道数，避免解析器未来扩展越过 uORB 数组。
    message.channel_count = frame.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS
        ? input_rc_s::RC_INPUT_MAX_CHANNELS : frame.channel_count;
    message.rssi = -1;
    message.rc_failsafe = frame.failsafe;
    // SBUS frame-lost 只表示跳帧，不能误判为整条 RC 链路失联。
    // receiver failsafe 由 rc_failsafe 单独表达；rc_lost 只由本地锁状态/零通道决定。
    message.rc_lost = !signal_locked_ || message.channel_count == 0U;
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
        else DIMA_LOG_SOURCE(dima::logging::Source::Sbus,
                             dima::logging::Level::Info,
                             "receiver failsafe cleared");
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

} // namespace dima::drivers::rc

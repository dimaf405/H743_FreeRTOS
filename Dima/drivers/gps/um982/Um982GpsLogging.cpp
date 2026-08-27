#define MODULE_NAME "um982"
#include "Um982Gps.hpp"
#include "Um982MessageContract.hpp"
#include "Um982QgcLog.hpp"

#include "format/Format.hpp"

#include <cmath>

namespace dima::drivers::gps {
namespace {

constexpr std::uint8_t kAllPortsMask = 0x07U;
// 消息数量、名称、命令与周期均来自 um982_messages.json 生成的合同。位 i 只表示
// 生成合同中的第 i 项；禁止在驱动中另写消息清单或复制字符串常量。
constexpr std::uint8_t kAllLogsMask = static_cast<std::uint8_t>(
    (1U << dima::protocols::um982::generated::kMessageContractCount) - 1U);
constexpr std::uint8_t kConfigurationCommandCount =
    static_cast<std::uint8_t>(
        2U * dima::protocols::um982::generated::kMessageContractCount);
constexpr std::uint32_t kCommandRetryUs = 10000U;

constexpr std::uint8_t kReadSendConfig = 0U;
constexpr std::uint8_t kReadWaitConfig = 1U;
constexpr std::uint8_t kReadProbePort = 2U;
constexpr std::uint8_t kReadSendUnilog = 3U;
constexpr std::uint8_t kReadWaitUnilog = 4U;
// CONFIG 可能返回一个或多个端口行；首次响应后等待 200 ms 静默即可封存，
// 不能把查询等待拖到 maintenance 同为 750 ms 的 no-progress 边界。
constexpr std::uint32_t kConfigResponseQuietUs = 200000U;
// R4.10 将 UNILOGLIST 拆成约 20 ms 间隔的多行文本。最后一条合同项之后保持
// 200 ms 静默才封存快照，既能收齐重复项，又明显小于 750 ms 查询/维护超时。
constexpr std::uint32_t kUnilogListQuietUs = 200000U;

const char *maintenance_reason_name(
    dima::middleware::maintenance::RuntimeMaintenanceCoordinator::
        FailureReason reason) noexcept
{
    using FailureReason = dima::middleware::maintenance::
        RuntimeMaintenanceCoordinator::FailureReason;
    // 文本只用于一次性故障边沿；枚举值由协调器记录，不能再用 expired 混淆原因。
    switch (reason) {
    case FailureReason::None: return "none";
    case FailureReason::RuntimeUnhealthy: return "health";
    case FailureReason::MaintenanceUnsafe: return "unsafe";
    case FailureReason::ProgressTimeout: return "no-progress";
    case FailureReason::HardDeadline: return "deadline";
    case FailureReason::InvalidTicket: return "ticket";
    case FailureReason::InvalidProgress: return "progress";
    }
    return "unknown";
}

constexpr dima::platform::SerialLineConfiguration
um982_line_configuration(std::uint32_t baudrate) noexcept
{
    dima::platform::SerialLineConfiguration configuration{};
    configuration.baudrate = baudrate;
    configuration.data_bits = 8U;
    configuration.parity = dima::platform::SerialParity::None;
    configuration.stop_bits = dima::platform::SerialStopBits::One;
    configuration.rx_pull = dima::platform::SerialRxPull::Preserve;
    configuration.rx_enabled = true;
    configuration.tx_enabled = true;
    return configuration;
}

bool close_period(float value, float expected) noexcept
{
    // 接收机回读周期单位为秒，允许 1 ms 的文本/浮点舍入误差。
    return std::isfinite(value) && std::fabs(value - expected) <= 0.001F;
}

} // namespace

bool Um982Gps::send_command(const char *body) noexcept
{
    // make_command 只负责附加 CRLF 并检查固定缓冲容量；write 成功仅表示已接收
    // 到 UART TX 队列，后续状态必须等待 tx_complete 后才能推进。
    char command[96]{};
    const std::size_t length =
        dima::protocols::um982::Um982Protocol::make_command(
            body, command, sizeof(command));
    return length != 0U && uart_.write(
        reinterpret_cast<const std::uint8_t *>(command), length);
}

void Um982Gps::begin_configuration_read() noexcept
{
    begin_configuration_query(Phase::ReadConfiguration);
}

void Um982Gps::begin_configuration_verification() noexcept
{
    begin_configuration_query(Phase::VerifyConfiguration);
}

void Um982Gps::begin_configuration_query(Phase phase) noexcept
{
    // 配置读取从干净快照开始：先 CONFIG 收集三个 COM 波特率，再唯一识别当前
    // 物理端口，最后 UNILOGLIST 比对生成合同。
    for (auto &configuration : port_config_) {
        configuration = {};
    }
    unilog_ = {};
    config_mask_ = 0U;
    selected_receiver_port_ = 0U;
    port_probe_index_ = 0U;
    log_update_mask_ = 0U;
    configuration_read_step_ = kReadSendConfig;
    command_pending_ = false;
    configuration_query_tx_complete_ = false;
    config_response_progress_pending_ = false;
    version_seen_ = false;
    unilog_seen_ = false;
    unilog_entry_seen_ = false;
    unilog_entry_progress_pending_ = false;
    last_config_response_arrival_us_ = 0U;
    last_unilog_entry_arrival_us_ = 0U;
    transition(phase);
}

bool Um982Gps::identify_receiver_port() noexcept
{
    // 只有恰好一个接收机 COM 的回读波特率等于当前已探测波特率时才能确认端口；
    // 0 个或多个匹配都存在歧义，必须转入逐端口 VERSIONA 主动探测。
    std::uint8_t candidate = 0U;
    std::uint8_t matches = 0U;
    for (std::uint8_t index = 0U;
         index < dima::protocols::um982::Um982Protocol::kReceiverPortCount;
         ++index) {
        if (port_config_[index].baudrate == detected_baudrate_) {
            candidate = static_cast<std::uint8_t>(index + 1U);
            ++matches;
        }
    }
    if (matches != 1U) return false;
    selected_receiver_port_ = candidate;
    return true;
}

void Um982Gps::consume_unilog_list_entry(
    const dima::protocols::um982::Um982Protocol::UnilogList &entry,
    std::uint64_t arrival_us) noexcept
{
    // 无校验清单行只允许污染当前 UNILOGLIST 查询快照；运行期偶发 '<' 文本或
    // 延迟到达的旧响应一律忽略，避免错误触发配置收敛。
    const bool query_phase = phase_ == Phase::ReadConfiguration ||
                             phase_ == Phase::VerifyConfiguration;
    if (!query_phase || configuration_read_step_ != kReadWaitUnilog ||
        unilog_seen_) {
        return;
    }

    for (std::uint8_t port = 0U;
         port < dima::protocols::um982::Um982Protocol::kReceiverPortCount;
         ++port) {
        for (std::size_t index = 0U;
             index < dima::protocols::um982::generated::kMessageContractCount;
             ++index) {
            const std::uint8_t incoming =
                entry.instance_count[port][index];
            if (incoming == 0U) continue;

            // 实例数采用饱和加法；重复清单项必须保持 count>1，不能因 uint8
            // 回绕伪装成单实例。名称/索引仍全部来自生成合同。
            std::uint8_t &instances =
                unilog_.instance_count[port][index];
            instances = instances > UINT8_MAX - incoming
                            ? UINT8_MAX
                            : static_cast<std::uint8_t>(instances + incoming);
            unilog_.present_mask[port] |=
                static_cast<std::uint8_t>(1U << index);
            unilog_.period_s[port][index] = entry.period_s[port][index];
        }
    }
    // 合同外清单行不参与收敛判定，但仍用于延长响应活动窗口；否则大量附加
    // 日志可能把后续合同项隔开 200 ms，导致快照过早封存。
    unilog_entry_seen_ = true;
    unilog_entry_progress_pending_ = true;
    last_unilog_entry_arrival_us_ = arrival_us;
}

void Um982Gps::build_log_update_mask() noexcept
{
    // 一项仅在“存在、恰好一个实例、周期匹配”三者同时满足时视为已收敛；
    // 重复日志也必须重配，否则接收频率会叠加并破坏 10 Hz 业务合同。
    log_update_mask_ = 0U;
    if (selected_receiver_port_ < 1U || selected_receiver_port_ > 3U) {
        log_update_mask_ = kAllLogsMask;
        return;
    }
    const std::uint8_t port =
        static_cast<std::uint8_t>(selected_receiver_port_ - 1U);
    for (std::size_t index = 0U;
         index < dima::protocols::um982::generated::kMessageContractCount;
         ++index) {
        const auto &entry =
            dima::protocols::um982::generated::kMessageContracts[index];
        if ((unilog_.present_mask[port] & (1U << index)) == 0U ||
            unilog_.instance_count[port][index] != 1U ||
            !close_period(unilog_.period_s[port][index],
                          entry.expected_period_s)) {
            log_update_mask_ |= static_cast<std::uint8_t>(1U << index);
        }
    }
}

void Um982Gps::run_configuration_read(std::uint64_t now_us,
                                      bool verification) noexcept
{
    // 读取子状态机使用发送/等待分相，绝不在 WorkQueue 上阻塞等待串口响应。
    // 每次等待由 200 ms 进度检查和 750 ms 总超时共同约束。
    if (verification && !configuration_maintenance_valid(now_us)) {
        defer_configuration(false, "verify maintenance");
        return;
    }
    switch (configuration_read_step_) {
    case kReadSendConfig:
        if (!uart_.tx_complete() || !send_command("CONFIG")) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = true;
        configuration_query_tx_complete_ = false;
        configuration_read_step_ = kReadWaitConfig;
        phase_started_us_ = now_us;
        schedule(kCommandRetryUs);
        return;

    case kReadWaitConfig: {
        if (!configuration_query_tx_complete_) {
            if (!uart_.tx_complete()) {
                schedule(kCommandRetryUs);
                return;
            }
            command_pending_ = false;
            configuration_query_tx_complete_ = true;
            phase_started_us_ = now_us;
            if (verification &&
                !report_configuration_progress(now_us)) {
                defer_configuration(false, "CONFIG TX progress");
                return;
            }
        }
        if (verification && config_response_progress_pending_) {
            if (!report_configuration_progress(now_us)) {
                defer_configuration(false, "CONFIG response progress");
                return;
            }
            config_response_progress_pending_ = false;
        }
        const bool all_ports_seen =
            (config_mask_ & kAllPortsMask) == kAllPortsMask;
        const bool response_quiet = config_mask_ != 0U &&
            now_us >= last_config_response_arrival_us_ &&
            now_us - last_config_response_arrival_us_ >=
                kConfigResponseQuietUs;
        const bool query_time_valid = now_us >= phase_started_us_;
        const std::uint64_t query_elapsed = query_time_valid
            ? now_us - phase_started_us_
            : UINT64_MAX;
        if (!all_ports_seen && !response_quiet && query_time_valid &&
            query_elapsed < kConfigTimeoutUs) {
            std::uint32_t delay_us = kConfigProgressUs;
            if (config_mask_ != 0U &&
                now_us >= last_config_response_arrival_us_) {
                const std::uint64_t quiet_elapsed =
                    now_us - last_config_response_arrival_us_;
                if (quiet_elapsed < kConfigResponseQuietUs) {
                    const auto quiet_remaining = static_cast<std::uint32_t>(
                        kConfigResponseQuietUs - quiet_elapsed);
                    if (quiet_remaining < delay_us) {
                        delay_us = quiet_remaining;
                    }
                }
            } else {
                const auto timeout_remaining = static_cast<std::uint32_t>(
                    kConfigTimeoutUs - query_elapsed);
                if (timeout_remaining < delay_us) {
                    delay_us = timeout_remaining;
                }
            }
            schedule(delay_us);
            return;
        }
        if (verification && config_mask_ != 0U &&
            !report_configuration_progress(now_us)) {
            defer_configuration(false, "CONFIG progress");
            return;
        }
        if (identify_receiver_port()) {
            configuration_read_step_ = kReadSendUnilog;
        } else {
            port_probe_index_ = 1U;
            command_pending_ = false;
            configuration_read_step_ = kReadProbePort;
        }
        schedule();
        return;
    }

    case kReadProbePort:
        // CONFIG 波特率无法唯一映射时，对 COM1..COM3 逐一发送 VERSIONA；
        // UART RX 在每个候选前清空，避免上一端口的延迟响应误认当前端口。
        if (command_pending_) {
            if (!configuration_query_tx_complete_) {
                if (!uart_.tx_complete()) {
                    schedule(kCommandRetryUs);
                    return;
                }
                configuration_query_tx_complete_ = true;
                phase_started_us_ = now_us;
                if (verification &&
                    !report_configuration_progress(now_us)) {
                    defer_configuration(false, "VERSION TX progress");
                    return;
                }
            }
            if (version_seen_) {
                if (verification &&
                    !report_configuration_progress(now_us)) {
                    defer_configuration(false, "VERSION progress");
                    return;
                }
                selected_receiver_port_ = port_probe_index_;
                command_pending_ = false;
                configuration_read_step_ = kReadSendUnilog;
                schedule();
            } else if (now_us - phase_started_us_ >= kProbeTimeoutUs) {
                command_pending_ = false;
                ++port_probe_index_;
                schedule();
            } else {
                schedule(kConfigProgressUs);
            }
            return;
        }
        if (port_probe_index_ >
            dima::protocols::um982::Um982Protocol::kReceiverPortCount) {
            defer_configuration(false, "receiver COM unresolved");
            return;
        }
        {
            char body[32]{};
            const int length = dima::format::format_to(
                body, sizeof(body), "VERSIONA COM%u", port_probe_index_);
            if (length <= 0 ||
                static_cast<std::size_t>(length) >= sizeof(body) ||
                !uart_.tx_complete()) {
                schedule(kCommandRetryUs);
                return;
            }
            uart_.clear_rx();
            protocol_.reset();
            version_seen_ = false;
            if (!send_command(body)) {
                schedule(kCommandRetryUs);
                return;
            }
            command_pending_ = true;
            configuration_query_tx_complete_ = false;
            phase_started_us_ = now_us;
            schedule(kCommandRetryUs);
        }
        return;

    case kReadSendUnilog:
        unilog_ = {};
        unilog_seen_ = false;
        if (!uart_.tx_complete() || !send_command("UNILOGLIST")) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = true;
        configuration_query_tx_complete_ = false;
        configuration_read_step_ = kReadWaitUnilog;
        phase_started_us_ = now_us;
        schedule(kCommandRetryUs);
        return;

    case kReadWaitUnilog: {
        if (!configuration_query_tx_complete_) {
            if (!uart_.tx_complete()) {
                schedule(kCommandRetryUs);
                return;
            }
            command_pending_ = false;
            configuration_query_tx_complete_ = true;
            phase_started_us_ = now_us;
            if (verification &&
                !report_configuration_progress(now_us)) {
                defer_configuration(false, "UNILOGLIST TX progress");
                return;
            }
        }
        if (verification && unilog_entry_progress_pending_) {
            // 新的有效清单行已经到达，属于真实且不可重复的接收进度；按批次
            // 续报一次即可，合同外日志只延长活动窗口而不进入收敛快照。
            if (!report_configuration_progress(now_us)) {
                defer_configuration(false, "UNILOGLIST entry progress");
                return;
            }
            unilog_entry_progress_pending_ = false;
        }

        // 带 CRC 的整帧仍可立即完成；R4.10 增量行则在最后一条之后等待固定
        // 静默窗口。arrival 时间倒退时保持未完成并交由总超时失败关闭。
        if (!unilog_seen_ && unilog_entry_seen_ &&
            now_us >= last_unilog_entry_arrival_us_ &&
            now_us - last_unilog_entry_arrival_us_ >=
                kUnilogListQuietUs) {
            unilog_seen_ = true;
        }
        if (!unilog_seen_ &&
            now_us - phase_started_us_ < kConfigTimeoutUs) {
            std::uint32_t delay_us = kConfigProgressUs;
            if (unilog_entry_seen_ &&
                now_us >= last_unilog_entry_arrival_us_) {
                const std::uint64_t quiet_elapsed =
                    now_us - last_unilog_entry_arrival_us_;
                if (quiet_elapsed < kUnilogListQuietUs) {
                    const auto quiet_remaining = static_cast<std::uint32_t>(
                        kUnilogListQuietUs - quiet_elapsed);
                    if (quiet_remaining < delay_us) {
                        delay_us = quiet_remaining;
                    }
                }
            }
            schedule(delay_us);
            return;
        }
        // 若清单恰好贴近 750 ms 总超时到达，使用已经收集的保守快照；缺失项
        // 仍会进入 update mask，绝不把不完整回读误判为收敛。
        if (!unilog_seen_ && unilog_entry_seen_) {
            unilog_seen_ = true;
        }
        // “没有收到清单”是未知态，不等价于六项全部缺失。此时只能回到 Run，
        // 延后重试只读查询；禁止在没有回读证据时执行 UNLOG/LOG/SAVECONFIG，
        // 否则解析或链路异常会被放大成周期性全量重配并挤压 MAVLink/QGC。
        if (!unilog_seen_) {
            defer_configuration(false, "UNILOGLIST unavailable");
            return;
        }
        build_log_update_mask();
        if (verification &&
            !report_configuration_progress(now_us)) {
            defer_configuration(false, "UNILOGLIST progress");
            return;
        }
        const bool converged =
            detected_baudrate_ == active_target_baudrate_ &&
            log_update_mask_ == 0U;
        if (verification) {
            // UM982 命令即时修改运行配置；只有 CONFIG/UNILOGLIST 回读完全收敛
            // 才允许 SAVECONFIG，避免把未被接收机接受的命令写入 NVM。
            if (!converged) {
                defer_configuration(false, "verify mismatch");
            } else {
                transition(Phase::SaveConfiguration);
            }
        } else if (converged && !configuration_persistence_pending_) {
            configuration_complete_ = true;
            configuration_retry_after_us_ = 0U;
            configuration_fault_active_ = false;
            UM982_QGC_INFO("GPS cfg ready COM%u b=%lu logs=%u",
                     static_cast<unsigned int>(selected_receiver_port_),
                     static_cast<unsigned long>(detected_baudrate_),
                     static_cast<unsigned int>(
                         dima::protocols::um982::generated::
                             kMessageContractCount));
            transition(Phase::Run, kReceiveScheduleUs);
        } else {
            begin_configuration_apply();
        }
        return;
    }

    default:
        defer_configuration(false, "configuration read step invalid");
        return;
    }
}

void Um982Gps::begin_configuration_apply() noexcept
{
    // apply 是唯一会进入写配置控制面的边沿；保留 mask 可直接判断后续是否发生
    // 非预期重复修复，同时不输出逐条 UNLOG/LOG 命令原文。
    UM982_QGC_INFO("GPS cfg apply COM%u b=%lu logs=0x%02x save=%u",
             static_cast<unsigned int>(selected_receiver_port_),
             static_cast<unsigned long>(detected_baudrate_),
             static_cast<unsigned int>(log_update_mask_),
             configuration_persistence_pending_ ? 1U : 0U);
    configuration_command_index_ = 0U;
    command_pending_ = false;
    configuration_query_tx_complete_ = false;
    baud_change_complete_ =
        detected_baudrate_ == active_target_baudrate_;
    maintenance_ready_ = false;
    maintenance_progress_ = 0U;
    transition(Phase::ApplyConfiguration, kReceiveScheduleUs);
}

bool Um982Gps::configuration_maintenance_valid(
    std::uint64_t now_us) noexcept
{
    // permit 轮询只确认票据仍有效，不更新 last_progress；TX 或回读卡住时必须让
    // 协调器的 750 ms 无真实进度保护按设计触发。
    return maintenance_ticket_ != 0U && !armed_.armed() &&
           maintenance_.permit(maintenance_ticket_, now_us) ==
               dima::middleware::maintenance::
                   RuntimeMaintenanceCoordinator::Permit::Ready;
}

bool Um982Gps::report_configuration_progress(
    std::uint64_t now_us) noexcept
{
    // 只有调用点已经完成一次不可重复的状态迁移才递增；饱和时失败关闭，禁止
    // uint32 回绕把旧进度重新伪装成新进度。
    if (maintenance_progress_ == UINT32_MAX) return false;
    ++maintenance_progress_;
    return maintenance_.report_progress(
        maintenance_ticket_, maintenance_progress_, now_us);
}

void Um982Gps::release_configuration_maintenance(bool complete) noexcept
{
    // 先从对象摘出票据，再通知协调器，最后释放 armed interlock；即使回调引发
    // 后续调度，也不会重复 complete/cancel 同一票据。
    const auto ticket = maintenance_ticket_;
    maintenance_ticket_ = 0U;
    if (ticket != 0U) {
        if (complete) {
            maintenance_.complete(ticket);
        } else {
            maintenance_.cancel(ticket);
        }
    }
    if (maintenance_interlock_acquired_) {
        armed_.end_maintenance();
        maintenance_interlock_acquired_ = false;
    }
    maintenance_ready_ = false;
}

void Um982Gps::defer_configuration(bool rescan, const char *reason) noexcept
{
    using FailureReason = dima::middleware::maintenance::
        RuntimeMaintenanceCoordinator::FailureReason;
    const bool report_failure = !configuration_fault_active_;
    const FailureReason maintenance_failure = maintenance_ticket_ == 0U
        ? FailureReason::None
        : maintenance_.failure_reason(maintenance_ticket_);
    const auto failed_phase = phase_;
    const std::uint8_t failed_command = configuration_command_index_;
    const std::uint8_t failed_logs = log_update_mask_;
    const std::uint8_t failed_config_mask = config_mask_;
    // list=0 表示没有合法清单行，1 表示已收到增量行但快照未封存，2 表示已有
    // 可用于判定的清单快照；比逐帧原文更直接地定位配置回读卡在哪一层。
    const std::uint8_t failed_list_state = unilog_seen_
        ? 2U
        : (unilog_entry_seen_ ? 1U : 0U);
    const bool tx_complete = uart_.tx_complete();
    release_configuration_maintenance(false);
    configuration_complete_ = false;
    configuration_fault_active_ = true;
    command_pending_ = false;
    configuration_query_tx_complete_ = false;
    const std::uint64_t now_us = clock_.now_us();
    // 配置失败不使 GPS 数据面离线：按饱和加法设置 30 s 重试点，期间回到 Run；
    // 只有本地波特率切换失败等会话失配场景才立即重新扫描。
    configuration_retry_after_us_ =
        now_us > UINT64_MAX - kConfigurationRetryUs
            ? UINT64_MAX
            : now_us + kConfigurationRetryUs;
    if (report_failure) {
        // 一条边沿日志保留失败点、协调器第一原因、TX 和待更新位；重试期间不刷屏。
        UM982_QGC_ERR("GPS cfg %s m=%s p=%u i=%u tx=%u cfg=0x%02x "
                "list=%u logs=0x%02x",
                reason == nullptr ? "unknown" : reason,
                maintenance_reason_name(maintenance_failure),
                static_cast<unsigned int>(failed_phase),
                static_cast<unsigned int>(failed_command),
                tx_complete ? 1U : 0U,
                static_cast<unsigned int>(failed_config_mask),
                static_cast<unsigned int>(failed_list_state),
                static_cast<unsigned int>(failed_logs));
    }
    if (rescan) {
        receiver_status_ = ReceiverStatus::Probing;
        build_scan_baudrates(active_target_baudrate_);
        transition(Phase::Detect);
    } else {
        transition(Phase::Run, kReceiveScheduleUs);
    }
}

void Um982Gps::run_configuration_save(std::uint64_t now_us) noexcept
{
    // R1.15 明确规定 SAVECONFIG 只写 NVM，不会重启接收机；运行配置已经在前一
    // 阶段回读验证，因此 TX 完成后直接沿用当前 UART 会话进入 Run。
    if (!maintenance_ready_ || !configuration_maintenance_valid(now_us)) {
        defer_configuration(false, "save maintenance");
        return;
    }
    if (!command_pending_) {
        if (!uart_.tx_complete() || !send_command("SAVECONFIG")) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = true;
        schedule(kCommandRetryUs);
        return;
    }
    if (!uart_.tx_complete()) {
        schedule(kCommandRetryUs);
        return;
    }
    if (!report_configuration_progress(now_us)) {
        defer_configuration(false, "SAVECONFIG progress");
        return;
    }

    command_pending_ = false;
    configuration_persistence_pending_ = false;
    configuration_complete_ = true;
    configuration_retry_after_us_ = 0U;
    configuration_fault_active_ = false;
    UM982_QGC_INFO("GPS cfg saved COM%u b=%lu logs=%u",
             static_cast<unsigned int>(selected_receiver_port_),
             static_cast<unsigned long>(detected_baudrate_),
             static_cast<unsigned int>(
                 dima::protocols::um982::generated::kMessageContractCount));
    release_configuration_maintenance(true);
    transition(Phase::Run, kReceiveScheduleUs);
}

void Um982Gps::run_configuration_apply(std::uint64_t now_us) noexcept
{
    // Apply 先取得“未武装 + flash maintenance”双重许可，再修改接收机波特率/
    // 日志和执行 SAVECONFIG；等待许可期间仍以 10 Hz 调度，不能阻塞其他模块。
    if (!maintenance_ready_) {
        if (maintenance_ticket_ == 0U) {
            if (now_us < maintenance_retry_after_us_) {
                schedule(kReceiveScheduleUs);
                return;
            }
            if (armed_.armed() || !armed_.begin_maintenance()) {
                maintenance_retry_after_us_ = now_us + kMaintenanceRetryUs;
                schedule(kReceiveScheduleUs);
                return;
            }
            maintenance_interlock_acquired_ = true;
            maintenance_ticket_ = maintenance_.request(now_us);
            if (maintenance_ticket_ == 0U) {
                armed_.end_maintenance();
                maintenance_interlock_acquired_ = false;
                maintenance_retry_after_us_ = now_us + kMaintenanceRetryUs;
                schedule(kReceiveScheduleUs);
                return;
            }
        }
        const auto permit = maintenance_.permit(maintenance_ticket_, now_us);
        if (permit == dima::middleware::maintenance::
                          RuntimeMaintenanceCoordinator::Permit::Waiting) {
            schedule(kReceiveScheduleUs);
            return;
        }
        if (permit == dima::middleware::maintenance::
                          RuntimeMaintenanceCoordinator::Permit::Denied) {
            defer_configuration(false, "maintenance denied");
            return;
        }
        maintenance_ready_ = true;
    }

    if (!configuration_maintenance_valid(now_us)) {
        defer_configuration(false, "apply maintenance");
        return;
    }

    if (!baud_change_complete_) {
        if (selected_receiver_port_ < 1U ||
            selected_receiver_port_ >
                dima::protocols::um982::Um982Protocol::kReceiverPortCount) {
            defer_configuration(false, "receiver COM unknown");
            return;
        }
        // 先让接收机 COM 切换，再等待 TX 完成后切本地 UART；顺序反转会丢失命令
        // 尾部并让两端波特率永久失配。
        if (!command_pending_) {
            char body[64]{};
            const int length = dima::format::format_to(
                body, sizeof(body), "CONFIG COM%u %lu 8 N 1",
                selected_receiver_port_,
                static_cast<unsigned long>(active_target_baudrate_));
            if (length <= 0 ||
                static_cast<std::size_t>(length) >= sizeof(body) ||
                !uart_.tx_complete() || !send_command(body)) {
                schedule(kCommandRetryUs);
                return;
            }
            command_pending_ = true;
            // write 接受后即保守认为接收机可能收到完整命令；即使随后 TX 完成事件
            // 丢失，下一轮回读正确时也必须补做 SAVECONFIG。
            configuration_persistence_pending_ = true;
            schedule(kCommandRetryUs);
            return;
        }
        if (!uart_.tx_complete()) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = false;
        if (!uart_.set_line_configuration(
                um982_line_configuration(active_target_baudrate_))) {
            defer_configuration(true, "local baud switch failed");
            return;
        }
        protocol_.reset();
        uart_.clear_rx();
        clear_measurement_cache();
        detected_baudrate_ = active_target_baudrate_;
        last_confirmed_baudrate_ = detected_baudrate_;
        baud_change_complete_ = true;
        configuration_persistence_pending_ = true;
        if (!report_configuration_progress(now_us)) {
            defer_configuration(false, "baud progress");
            return;
        }
        schedule(kReceiveScheduleUs);
        return;
    }

    if (selected_receiver_port_ < 1U ||
        selected_receiver_port_ >
            dima::protocols::um982::Um982Protocol::kReceiverPortCount) {
        defer_configuration(false, "receiver COM unknown");
        return;
    }

    // 每个待更新日志固定执行两步：UNLOG 清除旧/重复实例，再按生成合同 LOG。
    // command_index/2 映射合同项，奇偶位映射 UNLOG/LOG；未置位项成对跳过。
    while (configuration_command_index_ < kConfigurationCommandCount &&
           (log_update_mask_ &
            (1U << (configuration_command_index_ / 2U))) == 0U) {
        configuration_command_index_ = static_cast<std::uint8_t>(
            configuration_command_index_ + 2U);
    }
    if (configuration_command_index_ >= kConfigurationCommandCount) {
        command_pending_ = false;
        // 所有修改命令 TX 完成后先回读当前易失配置。纯阶段迁移不得续报进度；
        // 下一次真实进度只能来自查询命令 TX-complete 或有效响应回读。
        begin_configuration_verification();
        return;
    }

    if (command_pending_) {
        if (!uart_.tx_complete()) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = false;
        ++configuration_command_index_;
        configuration_persistence_pending_ = true;
        if (!report_configuration_progress(now_us)) {
            defer_configuration(false, "log command progress");
            return;
        }
        schedule();
        return;
    }

    const std::uint8_t log_index =
        static_cast<std::uint8_t>(configuration_command_index_ / 2U);
    // 名称、命令关键字和周期字符串只读取生成的 kMessageContracts。
    const auto &entry =
        dima::protocols::um982::generated::kMessageContracts[log_index];
    char body[64]{};
    int length = 0;
    if ((configuration_command_index_ & 1U) == 0U) {
        length = dima::format::format_to(
            body, sizeof(body), "UNLOG COM%u %s",
            selected_receiver_port_, entry.log_name);
    } else {
        length = dima::format::format_to(
            body, sizeof(body), "%s COM%u %s", entry.command_name,
            selected_receiver_port_, entry.period_s);
    }
    if (length <= 0 ||
        static_cast<std::size_t>(length) >= sizeof(body) ||
        !uart_.tx_complete() || !send_command(body)) {
        schedule(kCommandRetryUs);
        return;
    }
    command_pending_ = true;
    configuration_persistence_pending_ = true;
    schedule(kCommandRetryUs);
}

} // namespace dima::drivers::gps

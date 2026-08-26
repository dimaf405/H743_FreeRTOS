#define MODULE_NAME "um982"
#include "Um982Gps.hpp"
#include "Um982MessageContract.hpp"

#include "format/Format.hpp"
#include "logging/logging.hpp"

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
    version_seen_ = false;
    unilog_seen_ = false;
    transition(Phase::ReadConfiguration);
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

void Um982Gps::run_configuration_read(std::uint64_t now_us) noexcept
{
    // 读取子状态机使用发送/等待分相，绝不在 WorkQueue 上阻塞等待串口响应。
    // 每次等待由 200 ms 进度检查和 750 ms 总超时共同约束。
    switch (configuration_read_step_) {
    case kReadSendConfig:
        if (!uart_.tx_complete() || !send_command("CONFIG")) {
            schedule(kCommandRetryUs);
            return;
        }
        configuration_read_step_ = kReadWaitConfig;
        phase_started_us_ = now_us;
        schedule(kConfigProgressUs);
        return;

    case kReadWaitConfig:
        if ((config_mask_ & kAllPortsMask) != kAllPortsMask &&
            now_us - phase_started_us_ < kConfigTimeoutUs) {
            schedule(kConfigProgressUs);
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

    case kReadProbePort:
        // CONFIG 波特率无法唯一映射时，对 COM1..COM3 逐一发送 VERSIONA；
        // UART RX 在每个候选前清空，避免上一端口的延迟响应误认当前端口。
        if (command_pending_) {
            if (version_seen_) {
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
            phase_started_us_ = now_us;
            schedule(kConfigProgressUs);
        }
        return;

    case kReadSendUnilog:
        unilog_ = {};
        unilog_seen_ = false;
        if (!uart_.tx_complete() || !send_command("UNILOGLIST")) {
            schedule(kCommandRetryUs);
            return;
        }
        configuration_read_step_ = kReadWaitUnilog;
        phase_started_us_ = now_us;
        schedule(kConfigProgressUs);
        return;

    case kReadWaitUnilog:
        if (!unilog_seen_ &&
            now_us - phase_started_us_ < kConfigTimeoutUs) {
            schedule(kConfigProgressUs);
            return;
        }
        // UNILOGLIST 超时按“全部需要更新”处理，而不是猜测接收机已经正确配置。
        if (!unilog_seen_) {
            log_update_mask_ = kAllLogsMask;
        } else {
            build_log_update_mask();
        }
        if (detected_baudrate_ == active_target_baudrate_ &&
            log_update_mask_ == 0U) {
            configuration_complete_ = true;
            configuration_retry_after_us_ = 0U;
            transition(Phase::Run, kReceiveScheduleUs);
        } else {
            begin_configuration_apply();
        }
        return;

    default:
        defer_configuration(false, "configuration read step invalid");
        return;
    }
}

void Um982Gps::begin_configuration_apply() noexcept
{
    configuration_command_index_ = 0U;
    command_pending_ = false;
    baud_change_complete_ =
        detected_baudrate_ == active_target_baudrate_;
    maintenance_ready_ = false;
    maintenance_progress_ = 0U;
    transition(Phase::ApplyConfiguration, kReceiveScheduleUs);
}

bool Um982Gps::keep_configuration_alive(std::uint64_t now_us) noexcept
{
    // 配置写入期间每次推进都续报 maintenance 进度；武装、票据过期或进度计数
    // 饱和任一条件成立即失败关闭，防止写接收机 Flash 与车辆运行并发。
    return maintenance_ticket_ != 0U && !armed_.armed() &&
           maintenance_progress_ != UINT32_MAX &&
           maintenance_.report_progress(
               maintenance_ticket_, ++maintenance_progress_, now_us);
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
    const bool report_failure = configuration_retry_after_us_ == 0U;
    release_configuration_maintenance(false);
    configuration_complete_ = false;
    command_pending_ = false;
    const std::uint64_t now_us = clock_.now_us();
    // 配置失败不使 GPS 数据面离线：按饱和加法设置 30 s 重试点，期间回到 Run；
    // 只有本地波特率切换失败等会话失配场景才立即重新扫描。
    configuration_retry_after_us_ =
        now_us > UINT64_MAX - kConfigurationRetryUs
            ? UINT64_MAX
            : now_us + kConfigurationRetryUs;
    if (report_failure) {
        PX4_ERR("GPS config failed: %s",
                reason == nullptr ? "unknown" : reason);
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
    // SAVECONFIG 是唯一写接收机非易失存储的步骤。命令完全发送后才释放维护锁，
    // 随后等待接收机重启并重新探测，不能直接沿用保存前的串口会话状态。
    if (!maintenance_ready_ || !keep_configuration_alive(now_us)) {
        defer_configuration(false, "maintenance expired before save");
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

    command_pending_ = false;
    release_configuration_maintenance(true);
    configuration_complete_ = false;
    configuration_retry_after_us_ = 0U;
    transition(Phase::WaitRestart, kReceiverRestartWaitUs);
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

    if (!keep_configuration_alive(now_us)) {
        defer_configuration(false, "maintenance expired");
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
        transition(Phase::SaveConfiguration);
        return;
    }

    if (command_pending_) {
        if (!uart_.tx_complete()) {
            schedule(kCommandRetryUs);
            return;
        }
        command_pending_ = false;
        ++configuration_command_index_;
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
    schedule(kCommandRetryUs);
}

} // namespace dima::drivers::gps

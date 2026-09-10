#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"
#include "logging/logging.hpp"
#include "api/Time.hpp"

namespace dima::modules::mavlink {
namespace {
/* Auto 扫描候选覆盖常见数传电台速率，档数与 UM982 官方八档保持一致；
 * 每档观察窗口必须容纳至少两个 1 Hz GCS 心跳，锁定才算有效证据。 */
constexpr std::uint32_t kScanBaudRates[]{
    9600U, 19200U, 38400U, 57600U, 115200U, 230400U, 460800U, 921600U};
constexpr std::uint64_t kScanWindowUs = 4000000ULL;
constexpr std::uint32_t kScanLockFrames = 2U;
}

MavlinkService::MavlinkService(dima::platform::Console &console,
    dima::platform::BootControl &boot, dima::modules::mission::MissionService &mission,
    dima::platform::LogFileStore &logs, dima::platform::AsyncSerialPort &serial,
    const dima::lib::serial::SerialPortAssignments &assignments) noexcept
    : px4::ScheduledWorkItem("mavlink", px4::wq_configurations::lp_default),
      usb_transport_(console), uart_transport_(serial),
      usb_(usb_transport_, shared_, MAVLINK_COMM_0, boot, mission, logs),
      uart_(uart_transport_, shared_, MAVLINK_COMM_1, boot, mission, logs),
      assignments_(assignments)
{
}

bool MavlinkService::start() noexcept
{
    if (state_ == dima::middleware::lifecycle::ModuleState::Running) return true;
    shared_.reset();
    quiesce_requested_.store(false);
    uart_quiesced_.store(false);
    quiesce_failed_.store(false);
    quiesce_deadline_us_ = retry_uart_after_us_ = 0U;
    if (!ScheduleEnable() || !usb_.start()) {
        stop();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    // UART 配置或资源启动失败仅禁用该链路；USB 始终保留配置入口。
    if (!uart_.start()) PX4_ERR("UART MAVLink protocol unavailable; USB remains active");
    state_ = dima::middleware::lifecycle::ModuleState::Running;
    if (!ScheduleOnInterval(10000U, 10000U)) { stop(); return false; }
    return true;
}

void MavlinkService::stop() noexcept
{
    ScheduleCancelAndDrain();
    (void)uart_transport_.close();
    uart_.stop();
    usb_.stop();
    shared_.reset();
    scan_index_ = 0U;
    scan_locked_ = false;
    state_ = dima::middleware::lifecycle::ModuleState::Stopped;
}

dima::middleware::lifecycle::ModuleState MavlinkService::state() const noexcept
{
    return state_;
}

bool MavlinkService::deliver_ack(void *context, std::uint8_t channel, std::uint32_t epoch,
                                const vehicle_command_ack_s &ack) noexcept
{
    auto &self = *static_cast<MavlinkService *>(context);
    return (channel == MAVLINK_COMM_0 ? self.usb_ : self.uart_).queue_command_ack(ack, epoch);
}

void MavlinkService::notify_from_isr(void *context) noexcept
{
    (void)static_cast<MavlinkService *>(context)->ScheduleNowFromISR();
}

bool MavlinkService::prepare_serial_reconfigure() noexcept
{
    quiesce_requested_.store(true, std::memory_order_release);
    ScheduleNow();
    return uart_quiesced_.load(std::memory_order_acquire);
}

bool MavlinkService::serial_reconfigure_failed() const noexcept
{
    return quiesce_failed_.load(std::memory_order_acquire);
}

bool MavlinkService::apply_serial_configuration() noexcept
{
    // 仅 appMain 持有维护许可且 LP 已停止访问 UART 时调用。新驱动失败可在
    // GPS/RC 尚未恢复前回滚串口快照，避免提交 owner 后才发现 DMA 无法启动。
    if (!uart_quiesced_.load(std::memory_order_acquire)) return false;
    if (!uart_transport_.close()) return false;
    // 重配置后旧扫描证据全部作废：显式 BAUD 直接打开，Auto 交给扫描状态机。
    scan_index_ = 0U;
    scan_locked_ = false;
    retry_uart_after_us_ = 0U;
    if (assignments_.telemetry_port() == 0) return true;
    if (!assignments_.telemetry_configuration_valid()) return false;
    if (assignments_.telemetry_baudrate() == 0U) return true;
    return uart_transport_.open(
        assignments_.telemetry_port(), assignments_.telemetry_baudrate(),
        {&MavlinkService::notify_from_isr, this});
}

void MavlinkService::service_uart_scan(std::uint64_t now) noexcept
{
    /* Auto 扫描状态机：SERIALx_BAUD=0 时逐档尝试候选速率，每档窗口内收到
     * 不少于 kScanLockFrames 帧完整 MAVLink 帧（CRC/签名已由 parser 校验）
     * 即锁定该档。锁定速率只保留在本次 Runtime 的端口线配置中，不回写参数；
     * 窗口无帧则关闭换下一档，循环进行，不产生错误风暴。 */
    if (scan_locked_) return;
    if (!uart_transport_.ready()) {
        // 候选未打开（首轮、换档后或 quiesce 关闭后）：打开并重置窗口基线。
        if (now < retry_uart_after_us_) return;
        retry_uart_after_us_ = now + 1000000ULL;
        const std::uint32_t baud = kScanBaudRates[scan_index_];
        if (!uart_transport_.open(assignments_.telemetry_port(), baud,
                                  {&MavlinkService::notify_from_isr, this})) {
            // 该档硬件打开失败（引脚/DMA 被占等）：直接换下一档。
            scan_index_ = (scan_index_ + 1U) %
                (sizeof(kScanBaudRates) / sizeof(kScanBaudRates[0]));
            return;
        }
        scan_window_started_us_ = now;
        scan_frames_baseline_ = uart_.parsed_frames_;
        return;
    }
    if (uart_.parsed_frames_ - scan_frames_baseline_ >= kScanLockFrames) {
        scan_locked_ = true;
        PX4_INFO("UART MAVLink auto-baud locked at %lu B/s",
                 static_cast<unsigned long>(uart_transport_.baudrate()));
        return;
    }
    if (now - scan_window_started_us_ >= kScanWindowUs) {
        // 窗口内无有效帧：关闭当前档，换下一档继续观察。
        (void)uart_transport_.close();
        scan_index_ = (scan_index_ + 1U) %
            (sizeof(kScanBaudRates) / sizeof(kScanBaudRates[0]));
    }
}

void MavlinkService::resume_serial() noexcept
{
    quiesce_requested_.store(false, std::memory_order_release);
    ScheduleNow();
}

void MavlinkService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    const auto now = hrt_absolute_time();
    shared_.service_acks(now, &MavlinkService::deliver_ack, this);
    usb_.Run();
    if (quiesce_requested_.load(std::memory_order_acquire)) {
        if (uart_quiesced_.load(std::memory_order_acquire) || quiesce_failed_.load()) return;
        if (quiesce_deadline_us_ == 0U) {
            quiesce_deadline_us_ = now + uart_.drain_timeout_us();
            quiesce_error_generation_ = uart_.error_generation_;
        }
        uart_.Run(true);
        if (quiesce_error_generation_ != uart_transport_.error_generation()) {
            // 故障恢复清空队列不等于旧回应送达，本轮维护必须失败。
            quiesce_failed_.store(true, std::memory_order_release);
        } else if (uart_.tx_drained()) {
            uart_.reset_link();
            if (uart_transport_.close()) uart_quiesced_.store(true, std::memory_order_release);
            else quiesce_failed_.store(true, std::memory_order_release);
        } else if (now >= quiesce_deadline_us_) {
            // 未排空时拒绝本轮变更，appMain 取消请求并恢复旧链路；不强制截断回应。
            quiesce_failed_.store(true, std::memory_order_release);
            PX4_WARN("UART MAVLink drain timeout; keeping active configuration");
        }
        return;
    }
    uart_quiesced_.store(false, std::memory_order_release);
    quiesce_failed_.store(false, std::memory_order_release);
    quiesce_deadline_us_ = 0U;
    if (uart_.state() == dima::middleware::lifecycle::ModuleState::Running &&
        assignments_.telemetry_port() != 0) {
        if (assignments_.telemetry_baudrate() == 0U) {
            // BAUD=Auto：帧格式仍为 8N1，仅波特率由扫描状态机逐档探测。
            service_uart_scan(now);
        } else if (!uart_transport_.ready() && now >= retry_uart_after_us_) {
            retry_uart_after_us_ = now + 1000000ULL;
            if (!uart_transport_.open(assignments_.telemetry_port(), assignments_.telemetry_baudrate(),
                                     {&MavlinkService::notify_from_isr, this})) {
                PX4_WARN("UART MAVLink start failed; USB remains active");
            }
        }
    }
    uart_.Run();
}

} // namespace dima::modules::mavlink

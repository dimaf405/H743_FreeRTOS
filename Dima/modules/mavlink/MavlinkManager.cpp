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
    retry_uart_after_us_ = 0U;
    if (!ScheduleEnable() || !usb_.start()) {
        stop();
        state_ = dima::middleware::lifecycle::ModuleState::Error;
        return false;
    }
    /* 串口映射为重启生效合同（对齐 ArduPilot/PX4）：端口选择在启动时已由
     * SerialConfig 提交，本服务只负责打开；UART 打开或资源失败仅禁用该
     * 链路，USB 始终保留配置入口，不存在运行期热重配。 */
    if (assignments_.telemetry_port() != 0) {
        if (assignments_.telemetry_baudrate() > 0U) {
            if (!uart_transport_.open(assignments_.telemetry_port(),
                                      assignments_.telemetry_baudrate(),
                                      {&MavlinkService::notify_from_isr, this})) {
                PX4_ERR("UART MAVLink transport unavailable; USB remains active");
            }
        }
        // BAUD=Auto：波特率由下方扫描状态机逐档探测。
    }
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

void MavlinkService::service_uart_scan(std::uint64_t now) noexcept
{
    /* Auto 扫描状态机：SERIALx_BAUD=0 时逐档尝试候选速率，每档窗口内收到
     * 不少于 kScanLockFrames 帧完整 MAVLink 帧（CRC/签名已由 parser 校验）
     * 即锁定该档。锁定速率只保留在本次 Runtime 的端口线配置中，不回写参数；
     * 窗口无帧则关闭换下一档，循环进行，不产生错误风暴。 */
    if (scan_locked_) return;
    if (!uart_transport_.ready()) {
        // 候选未打开（首轮或换档后）：打开并重置窗口基线。
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

void MavlinkService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    const auto now = hrt_absolute_time();
    shared_.service_acks(now, &MavlinkService::deliver_ack, this);
    usb_.Run();
    }
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

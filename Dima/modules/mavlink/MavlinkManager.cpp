#define MODULE_NAME "mavlink"
#include "MavlinkService.hpp"
#include "logging/logging.hpp"
#include "api/Time.hpp"

namespace dima::modules::mavlink {
namespace {
/* Auto 扫描候选覆盖常见数传电台速率，档数与 UM982 官方八档保持一致；
 * 每档观察窗口必须容纳至少两个 1 Hz GCS 心跳，锁定才算有效证据。每轮
 * RX 解析预算限制单次调度耗时，错速噪声高峰也不能压住 lp_default。 */
constexpr std::uint32_t kScanBaudRates[]{
    9600U, 19200U, 38400U, 57600U, 115200U, 230400U, 460800U, 921600U};
constexpr std::size_t kScanBaudRateCount =
    sizeof(kScanBaudRates) / sizeof(kScanBaudRates[0]);
constexpr std::uint64_t kScanWindowUs = 4000000ULL;
constexpr std::uint32_t kScanLockFrames = 2U;
constexpr std::size_t kScanRxBudgetBytes = 256U;
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
    sensor_summary_sent_ = false;
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
     * 不少于 kScanLockFrames 帧完整 MAVLink 帧（CRC/签名已由本地 parser
     * 校验）即锁定该档。锁定速率只保留在本次 Runtime 的端口线配置中，不回写
     * 参数；窗口无帧则关闭换下一档。扫描期间不运行端点 Run：不派发、不应答、
     * 不发送心跳，错速噪声的累计错误也不会触发端点的协议复位风暴。 */
    if (scan_locked_ && uart_transport_.ready()) return;
    if (scan_locked_) {
        // 锁定后端点被关闭：从当前档位重新扫描。
        scan_locked_ = false;
    }
    if (!uart_transport_.ready()) {
        // 候选未打开（首轮或换档后）：打开并重置窗口基线。
        if (now < retry_uart_after_us_) return;
        retry_uart_after_us_ = now + 1000000ULL;
        const std::uint32_t baud = kScanBaudRates[scan_index_];
        if (!uart_transport_.open(assignments_.telemetry_port(), baud,
                                  {&MavlinkService::notify_from_isr, this})) {
            // 该档硬件打开失败（引脚/DMA 被占等）：直接换下一档。
            scan_index_ = (scan_index_ + 1U) % kScanBaudRateCount;
            return;
        }
        scan_window_started_us_ = now;
        scan_frames_baseline_ = scan_frames_;
        return;
    }
    // 传输层维护（错误恢复/1 Hz 诊断）继续执行；RX 只经本地 parser 计数。
    uart_transport_.service();
    std::uint8_t buffer[64];
    std::size_t budget = kScanRxBudgetBytes;
    while (budget != 0U) {
        const std::size_t size = uart_transport_.read(buffer, sizeof(buffer));
        if (size == 0U) break;
        budget = budget >= size ? budget - size : 0U;
        mavlink_message_t message{};
        mavlink_status_t status{};
        for (std::size_t index = 0U; index < size; ++index) {
            if (mavlink_parse_char(MAVLINK_COMM_1, buffer[index],
                                   &message, &status) != 0) {
                ++scan_frames_;
            }
        }
    }
    if (scan_frames_ - scan_frames_baseline_ >= kScanLockFrames) {
        // 锁定：端点重建干净协议上下文后恢复正常 Run 与故障语义。
        uart_.reset_link();
        scan_locked_ = true;
        PX4_INFO("UART MAVLink auto-baud locked at %lu B/s",
                 static_cast<unsigned long>(uart_transport_.baudrate()));
        return;
    }
    if (now - scan_window_started_us_ >= kScanWindowUs) {
        // 窗口内无有效帧：关闭当前档，换下一档继续观察。
        (void)uart_transport_.close();
        scan_index_ = (scan_index_ + 1U) % kScanBaudRateCount;
    }
}

void MavlinkService::Run()
{
    if (state_ != dima::middleware::lifecycle::ModuleState::Running) return;
    const auto now = hrt_absolute_time();
    shared_.service_acks(now, &MavlinkService::deliver_ack, this);
    usb_.Run();
    /* 传感器摘要一次性输出：任一端点首次链路就绪时打印一条，本 Runtime 不再
     * 重复。摘要属于非错误 Info，不随链路重建/传输错误重演；健康变化由
     * detected/timeout/recovered 边沿消息承担，持续健康经 SYS_STATUS 流出。 */
    if (!sensor_summary_sent_ &&
        (usb_.was_link_ready_ || uart_.was_link_ready_)) {
        sensor_summary_sent_ = true;
        (usb_.was_link_ready_ ? usb_ : uart_).report_sensor_link_summary();
    }
    if (uart_.state() == dima::middleware::lifecycle::ModuleState::Running &&
        assignments_.telemetry_port() != 0) {
        if (assignments_.telemetry_baudrate() == 0U) {
            // BAUD=Auto：帧格式仍为 8N1，仅波特率由扫描状态机逐档探测；
            // 本轮锁定后立即落入下方正常 Run。
            service_uart_scan(now);
        } else if (!uart_transport_.ready() && now >= retry_uart_after_us_) {
            retry_uart_after_us_ = now + 1000000ULL;
            if (!uart_transport_.open(assignments_.telemetry_port(), assignments_.telemetry_baudrate(),
                                     {&MavlinkService::notify_from_isr, this})) {
                PX4_WARN("UART MAVLink start failed; USB remains active");
            }
        }
    }
    if (!(assignments_.telemetry_port() != 0 &&
          assignments_.telemetry_baudrate() == 0U && !scan_locked_)) {
        uart_.Run();
    }
}

} // namespace dima::modules::mavlink

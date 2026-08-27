#define MODULE_NAME "um982"
#include "Um982Gps.hpp"
#include "Um982QgcLog.hpp"

namespace dima::drivers::gps {
namespace {

constexpr dima::platform::SerialLineConfiguration
um982_line_configuration(std::uint32_t baudrate) noexcept
{
    // UM982 产品串口合同固定为 8N1；仅波特率随探测/配置变化，RX 上拉保持
    // 板级配置，避免驱动层擅自改变外部收发器的电气偏置。
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

} // namespace

void Um982Gps::build_scan_baudrates(std::uint32_t target) noexcept
{
    // 优先探测参数目标值，其次是本次启动中最近确认值，再尝试常见 UM982
    // 波特率。固定数组容量为 8，因此去重也保证目标/历史值不会挤占多个槽位。
    const std::uint32_t preferred[]{
        target, last_confirmed_baudrate_, 9600U, 19200U, 38400U,
        57600U, 115200U, 230400U, 460800U, 921600U,
    };
    scan_count_ = 0U;
    for (const std::uint32_t candidate : preferred) {
        if (candidate == 0U) continue;
        bool duplicate = false;
        for (std::uint8_t index = 0U; index < scan_count_; ++index) {
            duplicate = duplicate || scan_baudrates_[index] == candidate;
        }
        if (!duplicate && scan_count_ < 8U) {
            scan_baudrates_[scan_count_++] = candidate;
        }
    }
    scan_index_ = 0U;
    candidate_active_ = false;
}

bool Um982Gps::start_scan_candidate() noexcept
{
    if (scan_index_ >= scan_count_) return false;
    // 候选切换前先停止 DMA、清协议半帧和测量缓存，保证上一波特率的残留字节
    // 不会与新候选拼成“有效帧”。每个候选最多观察 kProbeTimeoutUs=400 ms。
    (void)uart_.stop();
    protocol_.reset();
    clear_measurement_cache();
    const std::uint32_t candidate = scan_baudrates_[scan_index_];
    if (!uart_.configure(active_port_, um982_line_configuration(candidate)) ||
        !uart_.start({&Um982Gps::uart_notification, this})) {
        return false;
    }
    uart_.clear_rx();
    candidate_active_ = true;
    phase_started_us_ = clock_.now_us();
    schedule(kProbeTimeoutUs);
    return true;
}

void Um982Gps::complete_probe() noexcept
{
    // 只有解析器已经产生有效接收机数据才完成探测；单纯收到噪声字节不算在线。
    detected_baudrate_ = uart_.line_configuration().baudrate;
    last_confirmed_baudrate_ = detected_baudrate_;
    receiver_status_ = ReceiverStatus::Operational;
    retry_backoff_us_ = kInitialBackoffUs;
    candidate_active_ = false;
    // online 只在离线/探测到有效 UM982 测量的边沿输出一次，不按帧重复；结合
    // 后续 10 s 接收计数即可区分“串口已锁定”和“六类业务消息均持续到达”。
    UM982_QGC_INFO("GPS online S%ld b=%lu",
             static_cast<long>(active_port_),
             static_cast<unsigned long>(detected_baudrate_));
    configuration_complete_ = false;
    selected_receiver_port_ = 0U;
    config_mask_ = 0U;
    version_seen_ = false;
    unilog_seen_ = false;
    begin_configuration_read();
}

void Um982Gps::run_detect(std::uint64_t now_us) noexcept
{
    // Detect 在候选窗口内由 UART 通知提前唤醒；超时后推进下一个候选。
    // 全部候选失败统一进入 fail() 的指数退避，避免离线串口形成忙循环。
    if (candidate_active_ && last_valid_data_arrival_us_ != 0U) {
        complete_probe();
        return;
    }
    if (!candidate_active_) {
        if (scan_index_ >= scan_count_) {
            fail();
        } else if (!start_scan_candidate()) {
            ++scan_index_;
            schedule();
        }
        return;
    }
    if (now_us - phase_started_us_ >= kProbeTimeoutUs) {
        ++scan_index_;
        candidate_active_ = false;
        schedule();
    } else {
        schedule(kProbeTimeoutUs - static_cast<std::uint32_t>(
            now_us - phase_started_us_));
    }
}

} // namespace dima::drivers::gps

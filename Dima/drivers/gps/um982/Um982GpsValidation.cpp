#define MODULE_NAME "um982"
#include "Um982Gps.hpp"

namespace dima::drivers::gps {

bool Um982Gps::assignment_changed() const noexcept
{
    // 端口或目标波特率任一变化都使当前 UART 所有权/探测结果失效。
    return serial_assignments_.gps_port() != active_port_ ||
           serial_assignments_.gps_target_baudrate() !=
               active_target_baudrate_;
}

void Um982Gps::run_assignment() noexcept
{
    const std::int32_t port = serial_assignments_.gps_port();
    const std::uint32_t target =
        serial_assignments_.gps_target_baudrate();
    // 0 表示未分配：停止端口但保持模块任务存活，以 1 Hz 等待参数生成的
    // SerialPortAssignments 更新，而不是把“未配置”提升为模块错误。
    if (port == 0 || target == 0U) {
        (void)uart_.stop();
        active_port_ = 0;
        active_target_baudrate_ = 0U;
        receiver_status_ = ReceiverStatus::Unassigned;
        schedule(1000000U);
        return;
    }

    // 新分配会清除配置退避并从目标波特率开始完整探测。
    active_port_ = port;
    active_target_baudrate_ = target;
    receiver_status_ = ReceiverStatus::Probing;
    retry_backoff_us_ = kInitialBackoffUs;
    configuration_complete_ = false;
    configuration_retry_after_us_ = 0U;
    maintenance_retry_after_us_ = 0U;
    build_scan_baudrates(active_target_baudrate_);
    transition(Phase::Detect);
}

void Um982Gps::run_normal(std::uint64_t now_us) noexcept
{
    // 有效测量时间戳倒退或超过 1.3 s 均视为数据面失联；配置尚未收敛则在
    // 30 s 退避到期后重读配置，期间仍按 10 Hz 接收并发布 GPS 数据。
    if (last_valid_data_arrival_us_ == 0U ||
        now_us < last_valid_data_arrival_us_ ||
        now_us - last_valid_data_arrival_us_ > kReceiveTimeoutUs) {
        fail();
    } else if (!configuration_complete_ &&
               now_us >= configuration_retry_after_us_) {
        begin_configuration_read();
    } else {
        schedule(kReceiveScheduleUs);
    }
}

} // namespace dima::drivers::gps

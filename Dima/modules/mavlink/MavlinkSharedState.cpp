#include "MavlinkSharedState.hpp"

#include "api/Time.hpp"

#include <cmath>

namespace dima::modules::mavlink {

std::uint8_t MavlinkSharedState::submit(std::uint8_t channel, std::uint32_t epoch,
                                       const vehicle_command_s &command) noexcept
{
    if (command_pending_ || reboot_pending_) return MAV_RESULT_TEMPORARILY_REJECTED;
    // 先精确校验离散重启模式，避免 Commander 的整数转换把 3.x/非有限值
    // 解释成 Recovery；该上传入口只属于 USB。
    if (command.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN) {
        if (!std::isfinite(command.param1) ||
            (command.param1 != 0.0F && command.param1 != 1.0F && command.param1 != 3.0F)) {
            return MAV_RESULT_UNSUPPORTED;
        }
        if (channel != MAVLINK_COMM_0 && command.param1 == 3.0F) return MAV_RESULT_DENIED;
    }

    channel_ = channel;
    epoch_ = epoch;
    source_system_ = command.source_system;
    source_component_ = command.source_component;
    command_ = command.command;
    command_started_us_ = hrt_absolute_time();
    owner_valid_ = true;
    command_pending_ = true;
    ack_pending_ = false;
    if (!command_publication_.publish(command)) {
        command_pending_ = false;
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

void MavlinkSharedState::service_acks(std::uint64_t now, AckSink sink,
                                     void *context) noexcept
{
    // 只有本服务消费一次 ACK 并映射回请求链路；相同 sys/comp 出现在两个端口
    // 也不会令 ACK 串线。断线后的旧请求仍等待其 ACK/超时，防止误配下一条命令。
    vehicle_command_ack_s ack{};
    while (!ack_pending_ && ack_subscription_.copy(&ack)) {
        if (command_pending_ && ack.from_external && ack.command == command_ &&
            ack.target_system == source_system_ &&
            ack.target_component == source_component_) {
            pending_ack_ = ack;
            ack_pending_ = true;
        }
    }
    if (command_pending_ && !ack_pending_ &&
        (now < command_started_us_ || now - command_started_us_ >= 5000000ULL)) {
        pending_ack_ = {};
        pending_ack_.command = command_;
        pending_ack_.target_system = source_system_;
        pending_ack_.target_component = source_component_;
        pending_ack_.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
        pending_ack_.from_external = true;
        ack_pending_ = true;
    }
    if (ack_pending_ && (!owner_valid_ || sink(context, channel_, epoch_, pending_ack_))) {
        // IN_PROGRESS 仍归原事务，最终 ACK 才释放车辆级确认槽。
        if (pending_ack_.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            command_pending_ = false;
        } else {
            command_started_us_ = now;
        }
        ack_pending_ = false;
    }
}

void MavlinkSharedState::reset_link(std::uint8_t channel, std::uint32_t epoch) noexcept
{
    if (command_pending_ && channel_ == channel && epoch_ == epoch) owner_valid_ = false;
}

void MavlinkSharedState::reset() noexcept
{
    command_pending_ = ack_pending_ = owner_valid_ = reboot_pending_ = false;
    vehicle_command_ack_s ack{};
    while (ack_subscription_.copy(&ack)) {}
}
bool MavlinkSharedState::reboot_pending() const noexcept { return reboot_pending_; }
void MavlinkSharedState::cancel_reboot() noexcept { reboot_pending_ = false; }
void MavlinkSharedState::mark_reboot_pending() noexcept { reboot_pending_ = true; }

} // namespace dima::modules::mavlink

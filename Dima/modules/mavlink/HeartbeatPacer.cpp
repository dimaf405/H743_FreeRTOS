#include "HeartbeatPacer.hpp"

#include <cstring>

namespace dima::modules::mavlink {

HeartbeatPacer::HeartbeatPacer(MavlinkIdentity &identity) noexcept
    : identity_(identity)
{
}

bool HeartbeatPacer::tick(std::uint64_t now_us,
                          mavlink_message_t &msg) noexcept
{
    // HEARTBEAT 使用固定 1 Hz 节拍；其他 stream 的请求频率由生成合同分别管理。
    if (now_us - last_send_us_ < kIntervalUs) {
        return false;
    }
    pack_now(now_us, msg);
    return true;
}

void HeartbeatPacer::pack_now(std::uint64_t now_us,
                              mavlink_message_t &msg) noexcept
{
    // 先记录节拍再刷新状态并编码，使同一次调用形成一致的 HEARTBEAT 快照。
    last_send_us_ = now_us;
    refresh_state_from_orb();

    mavlink_heartbeat_t heartbeat{};
    heartbeat.custom_mode = custom_mode_;
    heartbeat.type = MavlinkIdentity::MAV_TYPE_VALUE;
    heartbeat.autopilot = MavlinkIdentity::MAV_AUTOPILOT_VALUE;
    heartbeat.base_mode = identity_.base_mode();
    heartbeat.system_status = identity_.system_status();
    heartbeat.mavlink_version = MAVLINK_VERSION;

    mavlink_msg_heartbeat_encode(MavlinkIdentity::SYSTEM_ID,
                                 MavlinkIdentity::COMPONENT_ID,
                                 &msg, &heartbeat);
}

void HeartbeatPacer::pack_autopilot_version(
    mavlink_message_t &msg) const noexcept
{
    // AUTOPILOT_VERSION 是构建身份与硬件身份的组合；flight 与 middleware
    // 当前共同指向同一 Dima 仓库提交，未单独跟踪的 OS 版本明确保持零值。
    mavlink_autopilot_version_t version{};
    version.capabilities = identity_.capabilities();
    version.flight_sw_version = identity_.flight_sw_version();
    version.middleware_sw_version = 0;
    version.os_sw_version = 0;
    version.board_version = identity_.board_version();
    identity_.get_flight_custom_version(version.flight_custom_version);
    identity_.get_middleware_custom_version(
        version.middleware_custom_version);
    identity_.get_os_custom_version(version.os_custom_version);
    version.vendor_id = identity_.vendor_id();
    version.product_id = identity_.product_id();
    version.uid = identity_.uid();
    // 当前硬件合同只有 64-bit uid；uid2 全零表示“不提供”，不是未知随机值。
    std::memset(version.uid2, 0, sizeof(version.uid2));

    mavlink_msg_autopilot_version_encode(MavlinkIdentity::SYSTEM_ID,
                                         MavlinkIdentity::COMPONENT_ID,
                                         &msg, &version);
}

void HeartbeatPacer::reset() noexcept
{
    // USB 会话重置后清除节拍，使新连接可立即获得 HEARTBEAT，并回到安全默认
    // custom_mode，直到新的 Commander Topic 再次建立证据。
    last_send_us_ = 0U;
    custom_mode_ = kPx4CustomModeManual;
}

void HeartbeatPacer::refresh_state_from_orb() noexcept
{
    const bool status_updated = status_subscription_.update();
    const bool control_mode_updated = control_mode_subscription_.update();
    // 任一 Topic 更新时都用 SubscriptionData 保存的最新两份快照重算；两者均未
    // 更新则保留上次 identity，避免无意义地重复写状态。
    if (!status_updated && !control_mode_updated) {
        return;
    }

    const vehicle_status_s &status = status_subscription_.get();
    const vehicle_control_mode_s &control_mode =
        control_mode_subscription_.get();

    // 只编码本 Rover 实现的 Manual/Termination；未知 nav_state 不伪装成某模式。
    switch (status.nav_state) {
    case vehicle_status_s::NAVIGATION_STATE_MANUAL:
        custom_mode_ = kPx4CustomModeManual;
        break;
    case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
        custom_mode_ = kPx4CustomModeTermination;
        break;
    default:
        custom_mode_ = 0U;
        break;
    }

    // base_mode 每一位都有独立证据：custom_mode 已知、Manual 控制实际启用、
    // Commander 明确 Armed。未发布的能力位保持 0。
    std::uint8_t base_mode = custom_mode_ != 0U
        ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED : 0U;
    if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
        control_mode.flag_control_manual_enabled) {
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
    if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // 严重性优先级为 Failsafe > Calibration > Armed > Standby，保证同时出现
    // 多个条件时 HEARTBEAT 不会被较轻状态覆盖。
    std::uint8_t system_status;
    if (status.failsafe) {
        system_status = MAV_STATE_CRITICAL;
    } else if (status.calibration_enabled ||
               status.rc_calibration_in_progress) {
        system_status = MAV_STATE_CALIBRATING;
    } else if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        system_status = MAV_STATE_ACTIVE;
    } else {
        system_status = MAV_STATE_STANDBY;
    }

    identity_.set_state(base_mode, system_status);
}

} // namespace dima::modules::mavlink

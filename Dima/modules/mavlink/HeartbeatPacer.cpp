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
    if (now_us - last_send_us_ < kIntervalUs) {
        return false;
    }
    pack_now(now_us, msg);
    return true;
}

void HeartbeatPacer::pack_now(std::uint64_t now_us,
                              mavlink_message_t &msg) noexcept
{
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
    std::memset(version.uid2, 0, sizeof(version.uid2));

    mavlink_msg_autopilot_version_encode(MavlinkIdentity::SYSTEM_ID,
                                         MavlinkIdentity::COMPONENT_ID,
                                         &msg, &version);
}

void HeartbeatPacer::reset() noexcept
{
    last_send_us_ = 0U;
    custom_mode_ = kPx4CustomModeManual;
}

void HeartbeatPacer::refresh_state_from_orb() noexcept
{
    const bool status_updated = status_subscription_.update();
    const bool control_mode_updated = control_mode_subscription_.update();
    if (!status_updated && !control_mode_updated) {
        return;
    }

    const vehicle_status_s &status = status_subscription_.get();
    const vehicle_control_mode_s &control_mode =
        control_mode_subscription_.get();

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

    std::uint8_t base_mode = custom_mode_ != 0U
        ? MAV_MODE_FLAG_CUSTOM_MODE_ENABLED : 0U;
    if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
        control_mode.flag_control_manual_enabled) {
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
    if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    std::uint8_t system_status;
    if (status.failsafe) {
        system_status = MAV_STATE_CRITICAL;
    } else if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
        system_status = MAV_STATE_ACTIVE;
    } else {
        system_status = MAV_STATE_STANDBY;
    }

    identity_.set_state(base_mode, system_status);
}

} // namespace dima::modules::mavlink
